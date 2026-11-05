/**
 * NavPilot - Cargo Ship Real-Time Path-Planning Simulator
 * Ship Hydrodynamics & Domain Model (ship.js)
 *
 * Implements:
 * - Ship Domain: SS = L * B * D (Length * Beam * Draft)
 * - Start Port M and End Port N handling
 * - Hydrodynamic stopping distance & turning constraints
 * - Autonomous Surface Vessel (ASV) energy & kinematics model
 */

class CargoShip {
  constructor(options = {}) {
    // Vessel physical characteristics
    this.length = options.length || 200.0;             // L (m)
    this.beam = options.beam || 32.0;                 // B (m)
    this.draft = options.draft || 12.0;               // D (m)
    this.displacement = options.displacement || 50000;// metric tonnes (default 50,000t)
    this.maxSpeedKnots = options.maxSpeed || 18.0;    // knots (default 18 knots)
    this.minTurningRadius = options.minTurningRadius || 400.0; // meters (default 400m)

    // Current kinematic state (in grid coordinates and world coordinates)
    this.x = 5.0;            // grid X
    this.y = 44.0;           // grid Y
    this.heading = -Math.PI / 4; // heading in radians (0 = East, -PI/2 = North)
    this.speed = 0.0;        // current speed in m/s
    this.targetSpeed = 0.0;  // ordered telegraph speed in m/s
    this.rudderAngle = 0.0;  // rudder angle in radians

    // Engine & energy tracking
    this.enginePower = 1.0;
    this.totalEnergyConsumed = 0.0; // accumulated energy metric
    this.state = 'IDLE';     // 'IDLE', 'CRUISING', 'HOLD_WAIT', 'EMERGENCY_STOP', 'ARRIVED'

    // Ports M (Departure) and N (Arrival)
    this.portM = { x: 5, y: 44, name: 'Port M (Departure)' };
    this.portN = { x: 44, y: 6, name: 'Port N (Destination)' };

    // Multi-stage path tracking
    this.stage1Path = null; // Stage 1: A* Coarse Route
    this.stage2Path = null; // Stage 2: RRT* Kinematic Feasible Route
    this.stage3Path = null; // Stage 3: Somersault Monkey Optimized Route
    this.activePath = null; // Currently executed path
    this.originalPath = null; // Path A (dashed on replan)
    this.replanPath = null;   // Path B (active avoidance route)
    this.hasReplanned = false;
    this.totalReplans = 0;
    this.pathIndex = 0;

    // Trail history for wake / swept visualization
    this.trail = [];
    this.maxTrailLength = 120;
  }

  /**
   * Computes Ship Domain SS = L * B * D (volumetric domain metric in m^3)
   */
  get shipDomainVolume() {
    return this.length * this.beam * this.draft;
  }

  /**
   * Planar Ship Domain 2D boundary radii in meters
   * Longitudinal clearance: 2.0 * L + stopping distance
   * Transverse clearance: 1.6 * B
   */
  get domainRadiusMeters() {
    const halfBeam = this.beam / 2.0;
    const baseMargin = 15.0;
    const displacementBuffer = (this.displacement / 50000.0) * 12.0;
    const speedRatio = this.speed > 0.1 ? (this.speed / this.maxSpeedMs) : 0.8;
    const dynamicBuffer = speedRatio * 8.0;

    return halfBeam + baseMargin + displacementBuffer + dynamicBuffer;
  }

  updateParameters(params) {
    if (params.length !== undefined) this.length = Math.max(50, Math.min(450, Number(params.length)));
    if (params.beam !== undefined) this.beam = Math.max(10, Math.min(70, Number(params.beam)));
    if (params.draft !== undefined) this.draft = Math.max(4, Math.min(25, Number(params.draft)));
    if (params.displacement !== undefined) this.displacement = Math.max(5000, Math.min(250000, Number(params.displacement)));
    if (params.maxSpeed !== undefined) this.maxSpeedKnots = Math.max(5, Math.min(30, Number(params.maxSpeed)));
    if (params.minTurningRadius !== undefined) this.minTurningRadius = Math.max(100, Math.min(1000, Number(params.minTurningRadius)));
  }

  get maxSpeedMs() {
    return this.maxSpeedKnots * 0.514444;
  }

  get minRadiusCells() {
    return this.minTurningRadius / 50.0;
  }

  get maxCurvature() {
    return 1.0 / this.minTurningRadius;
  }

  /**
   * Hydrodynamic stopping distance in meters:
   * d_stop = (displacement * v^2) / (2 * F_reverse)
   */
  computeStoppingDistanceMeters(speedMs = null) {
    const v = (speedMs !== null) ? speedMs : (this.speed > 0.5 ? this.speed : this.maxSpeedMs);
    const effectiveBrakingForce = this.displacement * 1000.0 * 0.12 * 9.81;
    const massKg = this.displacement * 1000.0;
    const stopDistance = (massKg * v * v) / (2.0 * effectiveBrakingForce);
    return Math.max(25.0, stopDistance);
  }

  computeRequiredClearanceMeters() {
    return this.domainRadiusMeters;
  }

  get lookaheadHorizonSeconds() {
    const stoppingDist = this.computeStoppingDistanceMeters();
    const v = Math.max(1.0, this.speed);
    const stopTime = stoppingDist / v;
    return Math.max(15.0, stopTime + 8.0);
  }

  get maxTurnRate() {
    const v = Math.max(2.0, this.speed > 0.5 ? this.speed : this.maxSpeedMs * 0.7);
    const inertiaFactor = Math.pow(50000.0 / this.displacement, 0.35);
    return (v / this.minTurningRadius) * Math.min(1.2, Math.max(0.4, inertiaFactor));
  }

  resetToStart(startPos) {
    this.x = startPos.x;
    this.y = startPos.y;
    this.heading = -Math.PI / 4;
    this.speed = 0.0;
    this.targetSpeed = 0.0;
    this.rudderAngle = 0.0;
    this.state = 'IDLE';
    this.pathIndex = 0;
    this.activePath = null;
    this.originalPath = null;
    this.replanPath = null;
    this.stage1Path = null;
    this.stage2Path = null;
    this.stage3Path = null;
    this.hasReplanned = false;
    this.totalReplans = 0;
    this.totalEnergyConsumed = 0.0;
    this.trail = [];
  }

  setPath(pathPoints, isReplan = false) {
    if (!pathPoints || pathPoints.length === 0) return;

    if (!isReplan) {
      this.originalPath = pathPoints;
      this.activePath = pathPoints;
      this.replanPath = null;
      this.hasReplanned = false;
      this.pathIndex = 0;
      this.x = pathPoints[0].x;
      this.y = pathPoints[0].y;
      if (pathPoints.length > 1) {
        this.heading = Math.atan2(pathPoints[1].y - pathPoints[0].y, pathPoints[1].x - pathPoints[0].x);
      }
    } else {
      this.replanPath = pathPoints;
      this.activePath = pathPoints;
      this.hasReplanned = true;
      this.totalReplans++;
      this.pathIndex = 0;
    }
    this.targetSpeed = this.maxSpeedMs;
    this.state = 'CRUISING';
  }

  setStagePaths(stage1, stage2, stage3) {
    this.stage1Path = stage1;
    this.stage2Path = stage2;
    this.stage3Path = stage3;
    this.setPath(stage3 || stage2 || stage1, false);
  }

  holdAndWait() {
    this.state = 'HOLD_WAIT';
    this.targetSpeed = 0.0;
  }

  emergencyStop() {
    this.state = 'EMERGENCY_STOP';
    this.targetSpeed = 0.0;
    this.enginePower = -0.7;
  }

  update(dt, cellSizeMeters = 50.0) {
    if (!this.activePath || this.state === 'IDLE' || this.state === 'ARRIVED') {
      return;
    }

    const accelRate = 0.35 * Math.pow(50000.0 / this.displacement, 0.4);
    const decelRate = 0.55 * Math.pow(50000.0 / this.displacement, 0.4);

    if (this.speed < this.targetSpeed) {
      this.speed = Math.min(this.targetSpeed, this.speed + accelRate * dt);
    } else if (this.speed > this.targetSpeed) {
      this.speed = Math.max(this.targetSpeed, this.speed - decelRate * dt);
    }

    // Accumulate energy consumption: baseline propulsion + drag from rudder deflection
    const thrustEnergy = (this.speed / this.maxSpeedMs) * 100.0;
    const rudderDragEnergy = Math.abs(this.rudderAngle) * 60.0;
    this.totalEnergyConsumed += (thrustEnergy + rudderDragEnergy) * dt;

    this.trail.push({ x: this.x, y: this.y, heading: this.heading });
    if (this.trail.length > this.maxTrailLength) {
      this.trail.shift();
    }

    if (this.state === 'HOLD_WAIT' || this.state === 'EMERGENCY_STOP') {
      if (this.speed <= 0.05) this.speed = 0.0;
      return;
    }

    if (this.pathIndex >= this.activePath.length) {
      this.state = 'ARRIVED';
      this.speed = 0.0;
      return;
    }

    const target = this.activePath[this.pathIndex];
    const dx = target.x - this.x;
    const dy = target.y - this.y;
    const distToTargetCells = Math.hypot(dx, dy);

    const waypointToleranceCells = Math.max(0.4, (this.speed / 10.0));
    if (distToTargetCells <= waypointToleranceCells) {
      this.pathIndex++;
      if (this.pathIndex >= this.activePath.length) {
        this.state = 'ARRIVED';
        this.speed = 0.0;
        return;
      }
    }

    const desiredHeading = Math.atan2(dy, dx);
    let headingDiff = desiredHeading - this.heading;

    while (headingDiff > Math.PI) headingDiff -= Math.PI * 2;
    while (headingDiff < -Math.PI) headingDiff += Math.PI * 2;

    const maxHeadingDelta = this.maxTurnRate * dt;
    const turnStep = Math.max(-maxHeadingDelta, Math.min(maxHeadingDelta, headingDiff));
    this.heading += turnStep;

    this.rudderAngle = Math.max(-0.6, Math.min(0.6, headingDiff * 1.5));

    const moveDistMeters = this.speed * dt;
    const moveDistCells = moveDistMeters / cellSizeMeters;

    this.x += Math.cos(this.heading) * moveDistCells;
    this.y += Math.sin(this.heading) * moveDistCells;
  }
}

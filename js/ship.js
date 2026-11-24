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

    // Blue Circle / Ship Domain disturbance detection state
    this.domainDisturbance = false;
    this.domainDisturbanceReason = '';
    this.domainDisturbancePulseTimer = 0;

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

  /**
   * Returns exact 2D Ship Domain ellipse parameters (in meters)
   * matching the visible blue envelope rendered on screen.
   */
  getDomainEllipseParameters(cellSizeMeters = 50.0) {
    const halfLenMeters = this.length / 2.0;
    const halfBmMeters = this.beam / 2.0;
    const domainRadius = this.domainRadiusMeters;

    const semiMajorM = halfLenMeters * 1.5 + domainRadius * 0.8; // longitudinal radius
    const semiMinorM = halfBmMeters + domainRadius;              // transverse radius
    const centerOffsetM = halfLenMeters * 0.4;                   // shifted forward towards bow

    return {
      semiMajorM,
      semiMinorM,
      centerOffsetM,
      heading: this.heading,
      centerX: this.x + (Math.cos(this.heading) * centerOffsetM) / cellSizeMeters,
      centerY: this.y + (Math.sin(this.heading) * centerOffsetM) / cellSizeMeters
    };
  }

  /**
   * Tests whether a continuous world coordinate (wx, wy in grid cells)
   * falls inside or touches the blue Ship Domain envelope.
   * @param {number} wx - X coordinate in grid cells
   * @param {number} wy - Y coordinate in grid cells
   * @param {number} extraMarginMeters - extra safety boundary margin
   * @param {number} cellSizeMeters - grid cell size
   * @returns {boolean}
   */
  isPointInsideBlueDomain(wx, wy, extraMarginMeters = 0.0, cellSizeMeters = 50.0) {
    const params = this.getDomainEllipseParameters(cellSizeMeters);
    const dxMeters = (wx - this.x) * cellSizeMeters;
    const dyMeters = (wy - this.y) * cellSizeMeters;

    const cosH = Math.cos(this.heading);
    const sinH = Math.sin(this.heading);

    // Transform world delta into vessel-aligned local coordinates
    const lx = dxMeters * cosH + dyMeters * sinH;
    const ly = -dxMeters * sinH + dyMeters * cosH;

    // Relocate to ellipse center
    const elx = lx - params.centerOffsetM;
    const ely = ly;

    const a = params.semiMajorM + extraMarginMeters;
    const b = params.semiMinorM + extraMarginMeters;

    const normDistSq = (elx * elx) / (a * a) + (ely * ely) / (b * b);
    return normDistSq <= 1.0;
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
    this.selectedAlgorithmName = '—';
    this.hasReplanned = false;
    this.totalReplans = 0;
    this.totalEnergyConsumed = 0.0;
    this.domainDisturbance = false;
    this.domainDisturbanceReason = '';
    this.domainDisturbancePulseTimer = 0;
    this.trail = [];
  }

  setPath(pathPoints, isReplan = false, algorithmLabel = null, grid = null) {
    if (!pathPoints || pathPoints.length === 0) return;
    if (algorithmLabel) this.selectedAlgorithmName = algorithmLabel;

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
      // If ship's current position is inside or touching land, move out to the safe passable water point!
      const currentCellBlocked = grid && (
        !grid.inBounds(Math.round(this.x), Math.round(this.y)) ||
        grid.cells[Math.round(this.y)][Math.round(this.x)] === grid.TYPE_LAND ||
        grid.depths[Math.round(this.y)][Math.round(this.x)] < this.draft
      );

      if (currentCellBlocked) {
        this.x = pathPoints[0].x;
        this.y = pathPoints[0].y;
        this.activePath = pathPoints;
      } else {
        const d0 = Math.hypot(pathPoints[0].x - this.x, pathPoints[0].y - this.y);
        if (d0 > 0.05 && d0 < 3.0) {
          this.activePath = [{ x: this.x, y: this.y }, ...pathPoints];
        } else {
          this.activePath = pathPoints;
        }
      }
      this.replanPath = this.activePath;
      this.hasReplanned = true;
      this.totalReplans++;
      this.pathIndex = 0;
      if (this.activePath.length > 1) {
        this.heading = Math.atan2(this.activePath[1].y - this.y, this.activePath[1].x - this.x);
      }
    }
    this.targetSpeed = this.maxSpeedMs;
    this.state = 'CRUISING';
  }

  setStagePaths(stage1, stage2, stage3, activeOptimal = null, algorithmLabel = '3-Stage Hybrid Optimal') {
    this.stage1Path = stage1;
    this.stage2Path = stage2;
    this.stage3Path = stage3;
    const chosen = activeOptimal || stage3 || stage2 || stage1;
    if (chosen) {
      this.setPath(chosen, false, algorithmLabel);
    }
  }

  holdAndWait() {
    this.state = 'HOLD_WAIT';
    this.targetSpeed = 0.0;
  }

  immediateFullStop(reason = '') {
    this.state = 'HOLD_WAIT';
    this.speed = 0.0;
    this.targetSpeed = 0.0;
    this.domainDisturbance = true;
    if (reason) this.domainDisturbanceReason = reason;
    this.domainDisturbancePulseTimer = 3.5;
  }

  emergencyStop() {
    this.state = 'EMERGENCY_STOP';
    this.targetSpeed = 0.0;
    this.enginePower = -0.7;
  }

  update(dt, grid = null, cellSizeMeters = 50.0) {
    if (!this.activePath || this.activePath.length < 2 || this.state === 'IDLE' || this.state === 'ARRIVED') {
      return;
    }

    const accelRate = 0.5 * Math.pow(50000.0 / this.displacement, 0.4);
    const decelRate = 0.8 * Math.pow(50000.0 / this.displacement, 0.4);

    if (this.domainDisturbancePulseTimer > 0) {
      this.domainDisturbancePulseTimer -= dt;
      if (this.domainDisturbancePulseTimer <= 0) {
        this.domainDisturbance = false;
      }
    }

    if (this.speed < this.targetSpeed) {
      this.speed = Math.min(this.targetSpeed, this.speed + accelRate * dt);
    } else if (this.speed > this.targetSpeed) {
      this.speed = Math.max(this.targetSpeed, this.speed - decelRate * dt);
    }

    if (this.state === 'HOLD_WAIT' || this.state === 'EMERGENCY_STOP') {
      if (this.speed <= 0.05) this.speed = 0.0;
      return;
    }

    // Distance to advance along the collision-free path this frame
    const moveDistCells = (this.speed * dt) / cellSizeMeters;
    if (moveDistCells <= 0) return;

    let remainingMove = moveDistCells;

    while (remainingMove > 0 && this.pathIndex < this.activePath.length - 1) {
      const pCurrent = this.activePath[this.pathIndex];
      const pNext = this.activePath[this.pathIndex + 1];

      const segDx = pNext.x - this.x;
      const segDy = pNext.y - this.y;
      const distToNext = Math.hypot(segDx, segDy);

      if (distToNext <= 0.001) {
        this.pathIndex++;
        continue;
      }

      let candidateX = this.x;
      let candidateY = this.y;
      let candidateDist = 0;

      if (remainingMove >= distToNext) {
        candidateX = pNext.x;
        candidateY = pNext.y;
        candidateDist = distToNext;
      } else {
        const ratio = remainingMove / distToNext;
        candidateX = this.x + segDx * ratio;
        candidateY = this.y + segDy * ratio;
        candidateDist = remainingMove;
      }

      // STRICT COLLISION PREVENTION: Check if candidate move enters land or shallow shoal
      if (grid) {
        const cgx = Math.round(candidateX);
        const cgy = Math.round(candidateY);
        const isBlocked = !grid.inBounds(cgx, cgy) ||
                          grid.cells[cgy][cgx] === grid.TYPE_LAND ||
                          grid.depths[cgy][cgx] < this.draft;
        if (isBlocked) {
          // An obstacle was placed directly ahead! Stop ship immediately, do NOT step into land!
          this.speed = 0.0;
          this.holdAndWait();
          return;
        }
      }

      this.x = candidateX;
      this.y = candidateY;
      remainingMove -= candidateDist;
      if (candidateX === pNext.x && candidateY === pNext.y) {
        this.pathIndex++;
      }
    }

    // Smooth heading alignment using short lookahead tangent along path
    if (this.pathIndex < this.activePath.length - 1) {
      const lookaheadIdx = Math.min(this.activePath.length - 1, this.pathIndex + 2);
      const pTarget = this.activePath[lookaheadIdx];
      const targetHeading = Math.atan2(pTarget.y - this.y, pTarget.x - this.x);

      let diff = targetHeading - this.heading;
      while (diff > Math.PI) diff -= Math.PI * 2;
      while (diff < -Math.PI) diff += Math.PI * 2;

      const maxTurn = Math.max(0.4, (this.speed / this.minTurningRadius) * (cellSizeMeters / 50.0) * 14.0) * dt;
      const appliedTurn = Math.max(-maxTurn, Math.min(maxTurn, diff));
      this.heading += appliedTurn;
      this.rudderAngle = Math.max(-0.6, Math.min(0.6, diff * 1.5));
    }

    // HARD OBSTACLE SAFETY GUARD: Continuous border + draft safety check
    if (grid) {
      const gx = Math.round(this.x);
      const gy = Math.round(this.y);
      const exactEdge = grid.getExactHazardDistanceMeters ? grid.getExactHazardDistanceMeters(this.x, this.y, this.draft) : 50.0;
      const minSafeEdge = Math.min(20.0, this.beam * 0.45);
      if (!grid.inBounds(gx, gy) ||
          grid.cells[gy][gx] === grid.TYPE_LAND ||
          grid.depths[gy][gx] < this.draft ||
          exactEdge < minSafeEdge * 0.25) {
        // Vessel is in or dangerously close to an obstacle: push out to nearest safe passable cell
        const safeCell = grid.findNearestPassableCell ? grid.findNearestPassableCell(this.x, this.y, this.draft, 12.0) : null;
        if (safeCell) {
          this.x = safeCell.x;
          this.y = safeCell.y;
        }
        this.speed = 0.0;
        this.holdAndWait();
      }
    }

    // Energy tracking
    const thrustEnergy = (this.speed / this.maxSpeedMs) * 100.0;
    const rudderDragEnergy = Math.abs(this.rudderAngle) * 60.0;
    this.totalEnergyConsumed += (thrustEnergy + rudderDragEnergy) * dt;

    this.trail.push({ x: this.x, y: this.y, heading: this.heading });
    if (this.trail.length > this.maxTrailLength) {
      this.trail.shift();
    }

    if (this.pathIndex >= this.activePath.length - 1) {
      const finalDest = this.activePath[this.activePath.length - 1];
      if (Math.hypot(finalDest.x - this.x, finalDest.y - this.y) < 0.25) {
        this.state = 'ARRIVED';
        this.speed = 0.0;
      }
    }
  }
}

/**
 * NavPilot - Cargo Ship Real-Time Path-Planning Simulator
 * Ship Hydrodynamic & Kinematic Model (ship.js)
 *
 * Implements realistic physical ship attributes:
 * - Length (m), Beam (m), Draft (m), Displacement (tonnes)
 * - Hydrodynamic stopping distance based on inertia & drag
 * - Dynamic safety margin scaling with displacement and speed
 * - Turning constraints (minimum turning radius, rudder deflection, rate of turn)
 * - Oriented Bounding Box (OBB) swept area footprint
 */

class CargoShip {
  constructor(options = {}) {
    // Vessel physical characteristics
    this.length = options.length || 200.0;             // meters (default 200m)
    this.beam = options.beam || 32.0;                 // meters (default 32m)
    this.draft = options.draft || 12.0;               // meters (default 12m)
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

    // Engine and deceleration dynamics
    this.enginePower = 1.0;  // 1.0 = full ahead, 0.0 = stopped, -0.7 = emergency reverse
    this.state = 'IDLE';     // 'IDLE', 'CRUISING', 'HOLD_WAIT', 'EMERGENCY_STOP', 'ARRIVED'

    // Real-time animation tracking
    this.pathIndex = 0;
    this.activePath = null;
    this.originalPath = null; // Path A (dashed when replanning occurs)
    this.replanPath = null;   // Path B (active avoidance route)
    this.hasReplanned = false;
    this.totalReplans = 0;

    // Trail history for wake / swept visualization
    this.trail = [];
    this.maxTrailLength = 120;
  }

  /**
   * Updates physical parameters from UI controls
   */
  updateParameters(params) {
    if (params.length !== undefined) this.length = Math.max(50, Math.min(450, Number(params.length)));
    if (params.beam !== undefined) this.beam = Math.max(10, Math.min(70, Number(params.beam)));
    if (params.draft !== undefined) this.draft = Math.max(4, Math.min(25, Number(params.draft)));
    if (params.displacement !== undefined) this.displacement = Math.max(5000, Math.min(250000, Number(params.displacement)));
    if (params.maxSpeed !== undefined) this.maxSpeedKnots = Math.max(5, Math.min(30, Number(params.maxSpeed)));
    if (params.minTurningRadius !== undefined) this.minTurningRadius = Math.max(100, Math.min(1000, Number(params.minTurningRadius)));
  }

  /**
   * Converts knots to m/s
   */
  get maxSpeedMs() {
    return this.maxSpeedKnots * 0.514444;
  }

  /**
   * Minimum turning radius in grid cells (1 cell = 50m)
   */
  get minRadiusCells() {
    return this.minTurningRadius / 50.0;
  }

  /**
   * Maximum allowable path curvature kappa_max = 1 / R_min (in 1/meters)
   */
  get maxCurvature() {
    return 1.0 / this.minTurningRadius;
  }

  /**
   * Computes hydrodynamic stopping distance:
   * d_stop = (displacement * v^2) / (2 * F_reverse)
   * Heavier displacement drastically increases stopping distance.
   */
  computeStoppingDistanceMeters(speedMs = null) {
    const v = (speedMs !== null) ? speedMs : (this.speed > 0.5 ? this.speed : this.maxSpeedMs);
    // Reverse hydrodynamic braking force ~ 10-15% of vessel weight force
    const effectiveBrakingForce = this.displacement * 1000.0 * 0.12 * 9.81; // Newtons
    const massKg = this.displacement * 1000.0;
    // Work-energy theorem: 0.5 * m * v^2 = F * d => d = (m * v^2) / (2 * F)
    const stopDistance = (massKg * v * v) / (2.0 * effectiveBrakingForce);
    return Math.max(25.0, stopDistance);
  }

  /**
   * Required collision clearance margin (in meters):
   * Half the ship's beam + baseline margin + inertia buffer based on displacement and speed.
   */
  computeRequiredClearanceMeters() {
    const halfBeam = this.beam / 2.0;
    const baseMargin = 15.0; // 15m minimum nautical buffer
    // Heavier ships need a larger safety buffer due to drift, squat, and windage
    const displacementBuffer = (this.displacement / 50000.0) * 12.0;
    const speedRatio = this.speed > 0.1 ? (this.speed / this.maxSpeedMs) : 0.8;
    const dynamicBuffer = speedRatio * 8.0;

    return halfBeam + baseMargin + displacementBuffer + dynamicBuffer;
  }

  /**
   * Dynamic Lookahead Horizon (seconds):
   * Time window to scan for dynamic obstacles ahead, accounting for stopping distance.
   */
  get lookaheadHorizonSeconds() {
    const stoppingDist = this.computeStoppingDistanceMeters();
    const v = Math.max(1.0, this.speed);
    const stopTime = stoppingDist / v;
    // Lookahead is stopping time + reaction and maneuvering reserve (at least 15s)
    return Math.max(15.0, stopTime + 8.0);
  }

  /**
   * Maximum rate of turn (radians per second):
   * omega = (v / R_min) * (inertia_factor)
   */
  get maxTurnRate() {
    const v = Math.max(2.0, this.speed > 0.5 ? this.speed : this.maxSpeedMs * 0.7);
    // Angular inertia penalty for supertankers/large container ships
    const inertiaFactor = Math.pow(50000.0 / this.displacement, 0.35);
    return (v / this.minTurningRadius) * Math.min(1.2, Math.max(0.4, inertiaFactor));
  }

  /**
   * Resets vessel to start position
   */
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
    this.hasReplanned = false;
    this.totalReplans = 0;
    this.trail = [];
  }

  /**
   * Assigns a newly planned route to the ship
   */
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
      // Preserve Path A as original for rendering, activate Path B as avoidance route
      this.replanPath = pathPoints;
      this.activePath = pathPoints;
      this.hasReplanned = true;
      this.totalReplans++;
      this.pathIndex = 0; // Starts from ship's current location
    }
    this.targetSpeed = this.maxSpeedMs;
    this.state = 'CRUISING';
  }

  /**
   * Orders ship to hold and wait (slow down / reverse thrust)
   */
  holdAndWait() {
    this.state = 'HOLD_WAIT';
    this.targetSpeed = 0.0;
  }

  /**
   * Orders emergency safe stop
   */
  emergencyStop() {
    this.state = 'EMERGENCY_STOP';
    this.targetSpeed = 0.0;
    this.enginePower = -0.7; // reverse thrusters
  }

  /**
   * Step physics forward by dt seconds (scaled by simulation speed factor)
   */
  update(dt, cellSizeMeters = 50.0) {
    if (!this.activePath || this.state === 'IDLE' || this.state === 'ARRIVED') {
      return;
    }

    // Hydrodynamic acceleration / deceleration towards target speed
    const accelRate = 0.35 * Math.pow(50000.0 / this.displacement, 0.4); // m/s^2
    const decelRate = 0.55 * Math.pow(50000.0 / this.displacement, 0.4);

    if (this.speed < this.targetSpeed) {
      this.speed = Math.min(this.targetSpeed, this.speed + accelRate * dt);
    } else if (this.speed > this.targetSpeed) {
      this.speed = Math.max(this.targetSpeed, this.speed - decelRate * dt);
    }

    // Record trail for wake effect
    this.trail.push({ x: this.x, y: this.y, heading: this.heading });
    if (this.trail.length > this.maxTrailLength) {
      this.trail.shift();
    }

    // If holding or stopped, don't advance along path
    if (this.state === 'HOLD_WAIT' || this.state === 'EMERGENCY_STOP') {
      if (this.speed <= 0.05) {
        this.speed = 0.0;
      }
      return;
    }

    // Path Following (Pure Pursuit / Waypoint Progression)
    if (this.pathIndex >= this.activePath.length) {
      this.state = 'ARRIVED';
      this.speed = 0.0;
      return;
    }

    // Lookahead waypoint target
    const target = this.activePath[this.pathIndex];
    const dx = target.x - this.x;
    const dy = target.y - this.y;
    const distToTargetCells = Math.hypot(dx, dy);
    const distToTargetMeters = distToTargetCells * cellSizeMeters;

    // Check if reached current waypoint
    const waypointToleranceCells = Math.max(0.4, (this.speed / 10.0));
    if (distToTargetCells <= waypointToleranceCells) {
      this.pathIndex++;
      if (this.pathIndex >= this.activePath.length) {
        this.state = 'ARRIVED';
        this.speed = 0.0;
        return;
      }
    }

    // Heading calculation and steering
    const desiredHeading = Math.atan2(dy, dx);
    let headingDiff = desiredHeading - this.heading;

    // Normalize angle to [-PI, PI]
    while (headingDiff > Math.PI) headingDiff -= Math.PI * 2;
    while (headingDiff < -Math.PI) headingDiff += Math.PI * 2;

    // Rate of turn constraint based on turning radius & speed
    const maxHeadingDelta = this.maxTurnRate * dt;
    const turnStep = Math.max(-maxHeadingDelta, Math.min(maxHeadingDelta, headingDiff));
    this.heading += turnStep;

    // Compute rudder visual angle
    this.rudderAngle = Math.max(-0.6, Math.min(0.6, headingDiff * 1.5));

    // Move forward along actual ship heading
    const moveDistMeters = this.speed * dt;
    const moveDistCells = moveDistMeters / cellSizeMeters;

    this.x += Math.cos(this.heading) * moveDistCells;
    this.y += Math.sin(this.heading) * moveDistCells;
  }

  /**
   * Computes the 4 corners of the ship's Oriented Bounding Box (OBB)
   * in grid coordinates (plus bow point).
   */
  getHullCorners(cellSizeMeters = 50.0) {
    const halfLen = (this.length / 2.0) / cellSizeMeters;
    const halfBm = (this.beam / 2.0) / cellSizeMeters;

    const cosH = Math.cos(this.heading);
    const sinH = Math.sin(this.heading);

    // Forward vector and Starboard vector
    const fwdX = cosH;
    const fwdY = sinH;
    const sideX = -sinH;
    const sideY = cosH;

    // Bow tip (pointed ship bow)
    const bowTip = {
      x: this.x + fwdX * (halfLen * 1.15),
      y: this.y + fwdY * (halfLen * 1.15)
    };

    // Stern and bow rectangle corners
    const bowStarboard = {
      x: this.x + fwdX * (halfLen * 0.8) + sideX * halfBm,
      y: this.y + fwdY * (halfLen * 0.8) + sideY * halfBm
    };
    const bowPort = {
      x: this.x + fwdX * (halfLen * 0.8) - sideX * halfBm,
      y: this.y + fwdY * (halfLen * 0.8) - sideY * halfBm
    };
    const sternStarboard = {
      x: this.x - fwdX * halfLen + sideX * halfBm,
      y: this.y - fwdY * halfLen + sideY * halfBm
    };
    const sternPort = {
      x: this.x - fwdX * halfLen - sideX * halfBm,
      y: this.y - fwdY * halfLen - sideY * halfBm
    };

    return [bowTip, bowStarboard, sternStarboard, sternPort, bowPort];
  }
}

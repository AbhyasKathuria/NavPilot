/**
 * NavPilot - Cargo Ship Real-Time Path-Planning Simulator
 * Dynamic Obstacles & Collision Avoidance (dynamic.js)
 *
 * Implements:
 * - Dynamic maritime obstacles: Moving Vessels, Drifting Icebergs, Drifting Containers
 * - Forward trajectory prediction over variable time horizon
 * - Closest Point of Approach (CPA) and Time to CPA (TCPA) calculation
 * - Dynamic collision prediction and automatic replanning trigger (Path A -> Path B)
 * - Hold-and-wait / Slowdown maneuver when navigation channel is obstructed
 * - Emergency safe stop fallback
 */

class DynamicObstacle {
  constructor(id, x, y, vx, vy, type = 'vessel', radiusMeters = 30.0) {
    this.id = id;
    this.x = x; // grid X
    this.y = y; // grid Y
    this.vx = vx; // grid cells per second
    this.vy = vy;
    this.type = type; // 'vessel', 'iceberg', 'debris'
    this.radiusMeters = radiusMeters;
    this.radiusCells = radiusMeters / 50.0; // 50m per cell
    this.heading = Math.atan2(vy, vx);

    this.trail = [];
    this.maxTrail = 35;
  }

  update(dt, grid) {
    this.x += this.vx * dt;
    this.y += this.vy * dt;
    this.heading = Math.atan2(this.vy, this.vx);

    this.trail.push({ x: this.x, y: this.y });
    if (this.trail.length > this.maxTrail) {
      this.trail.shift();
    }

    // Boundary bounce
    const pad = 1.0;
    if (this.x < pad) {
      this.x = pad;
      this.vx = Math.abs(this.vx);
    } else if (this.x > grid.cols - 1 - pad) {
      this.x = grid.cols - 1 - pad;
      this.vx = -Math.abs(this.vx);
    }

    if (this.y < pad) {
      this.y = pad;
      this.vy = Math.abs(this.vy);
    } else if (this.y > grid.rows - 1 - pad) {
      this.y = grid.rows - 1 - pad;
      this.vy = -Math.abs(this.vy);
    }

    // Land collision reflection
    const gx = Math.round(this.x);
    const gy = Math.round(this.y);
    if (grid.inBounds(gx, gy) && grid.cells[gy][gx] === grid.TYPE_LAND) {
      this.vx = -this.vx + (Math.random() - 0.5) * 0.2;
      this.vy = -this.vy + (Math.random() - 0.5) * 0.2;
    }
  }

  /**
   * Predicts future position at time t (seconds) ahead
   */
  predictPosition(t) {
    return {
      x: this.x + this.vx * t,
      y: this.y + this.vy * t
    };
  }
}

class DynamicObstacleManager {
  constructor(grid) {
    this.grid = grid;
    this.obstacles = [];
    this.nextId = 1;

    // Collision prediction & replanning state
    this.lastReplanTime = 0;
    this.replanCooldown = 3.5; // seconds between replans to prevent thrashing
    this.isReplanning = false;
    this.replanBannerTimer = 0;
    this.activeAlert = null;
  }

  clear() {
    this.obstacles = [];
    this.isReplanning = false;
    this.replanBannerTimer = 0;
    this.activeAlert = null;
  }

  addObstacle(x, y, vx, vy, type = 'vessel', radiusMeters = 35.0) {
    const obs = new DynamicObstacle(this.nextId++, x, y, vx, vy, type, radiusMeters);
    this.obstacles.push(obs);
    return obs;
  }

  addRandomObstacle() {
    // Pick an open sea cell
    for (let attempts = 0; attempts < 50; attempts++) {
      const x = 8 + Math.random() * (this.grid.cols - 16);
      const y = 8 + Math.random() * (this.grid.rows - 16);
      const gx = Math.round(x);
      const gy = Math.round(y);

      if (this.grid.inBounds(gx, gy) && this.grid.cells[gy][gx] === this.grid.TYPE_SEA) {
        const types = ['vessel', 'vessel', 'iceberg', 'debris'];
        const chosenType = types[Math.floor(Math.random() * types.length)];
        const speedCells = chosenType === 'vessel' ? (0.8 + Math.random() * 0.8) : (0.2 + Math.random() * 0.4);
        const angle = Math.random() * Math.PI * 2;
        const vx = Math.cos(angle) * speedCells;
        const vy = Math.sin(angle) * speedCells;
        const radius = chosenType === 'vessel' ? 40.0 : (chosenType === 'iceberg' ? 32.0 : 20.0);

        return this.addObstacle(x, y, vx, vy, chosenType, radius);
      }
    }
  }

  update(dt) {
    for (let i = 0; i < this.obstacles.length; i++) {
      this.obstacles[i].update(dt, this.grid);
    }

    if (this.replanBannerTimer > 0) {
      this.replanBannerTimer -= dt;
      if (this.replanBannerTimer <= 0) {
        this.isReplanning = false;
        this.activeAlert = null;
      }
    }
  }

  /**
   * Calculates Closest Point of Approach (CPA) and Time to CPA (TCPA)
   * between the ship's projected path and all dynamic obstacles.
   */
  evaluateCPA(ship, horizonSeconds = 25.0) {
    if (!ship.activePath || ship.activePath.length === 0 || ship.state === 'IDLE' || ship.state === 'ARRIVED') {
      return { collisionImminent: false, criticalObstacle: null, cpaMeters: Infinity, tcpaSeconds: Infinity };
    }

    const shipSpeedCells = (ship.speed > 0.5 ? ship.speed : ship.maxSpeedMs) / this.grid.cellSizeMeters;
    const requiredClearance = ship.computeRequiredClearanceMeters();

    let closestCpaMeters = Infinity;
    let closestTcpa = Infinity;
    let criticalObstacle = null;
    let collisionImminent = false;

    for (let o = 0; o < this.obstacles.length; o++) {
      const obs = this.obstacles[o];
      const safetyBufferMeters = requiredClearance + obs.radiusMeters;

      // Sample along time horizon in 0.5s increments
      const dtSample = 0.5;
      const numSteps = Math.ceil(horizonSeconds / dtSample);

      for (let s = 1; s <= numSteps; s++) {
        const t = s * dtSample;

        // Ship position at future time t
        const distAlongPathCells = shipSpeedCells * t;
        const futureShipPos = this.getFutureShipPosition(ship, distAlongPathCells);
        if (!futureShipPos) break;

        // Obstacle position at future time t
        const futureObsPos = obs.predictPosition(t);

        const distCells = Math.hypot(futureShipPos.x - futureObsPos.x, futureShipPos.y - futureObsPos.y);
        const distMeters = distCells * this.grid.cellSizeMeters;

        if (distMeters < closestCpaMeters) {
          closestCpaMeters = distMeters;
          closestTcpa = t;
        }

        if (distMeters < safetyBufferMeters) {
          collisionImminent = true;
          criticalObstacle = obs;
          return {
            collisionImminent: true,
            criticalObstacle: obs,
            cpaMeters: distMeters,
            tcpaSeconds: t,
            safetyBufferMeters
          };
        }
      }
    }

    return {
      collisionImminent,
      criticalObstacle,
      cpaMeters: closestCpaMeters,
      tcpaSeconds: closestTcpa
    };
  }

  /**
   * Estimates ship position after traveling distCells along its active path
   */
  getFutureShipPosition(ship, distCells) {
    if (!ship.activePath || ship.pathIndex >= ship.activePath.length) return null;

    let accumulatedDist = 0;
    let lastX = ship.x;
    let lastY = ship.y;

    for (let i = ship.pathIndex; i < ship.activePath.length; i++) {
      const pt = ship.activePath[i];
      const d = Math.hypot(pt.x - lastX, pt.y - lastY);
      if (accumulatedDist + d >= distCells) {
        const remaining = distCells - accumulatedDist;
        const ratio = d > 0 ? remaining / d : 0;
        return {
          x: lastX + (pt.x - lastX) * ratio,
          y: lastY + (pt.y - lastY) * ratio
        };
      }
      accumulatedDist += d;
      lastX = pt.x;
      lastY = pt.y;
    }

    return ship.activePath[ship.activePath.length - 1];
  }

  /**
   * Handles collision avoidance check and triggers replanning
   */
  checkAndAvoidCollisions(ship, planner, logCallback) {
    const now = performance.now() / 1000.0;
    if (now - this.lastReplanTime < this.replanCooldown) {
      return; // in cooldown
    }

    const cpaResult = this.evaluateCPA(ship, ship.lookaheadHorizonSeconds);

    if (cpaResult.collisionImminent) {
      this.lastReplanTime = now;
      const obs = cpaResult.criticalObstacle;

      if (logCallback) {
        logCallback(`Collision alert: Obstacle #${obs.id} (${obs.type.toUpperCase()}) TCPA: ${cpaResult.tcpaSeconds.toFixed(1)}s, CPA: ${cpaResult.cpaMeters.toFixed(1)}m. Buffer: ${cpaResult.safetyBufferMeters.toFixed(1)}m.`, 'warning');
        logCallback('Initiating dynamic avoidance replan (Path A -> Path B)...', 'info');
      }

      this.isReplanning = true;
      this.replanBannerTimer = 3.5;
      this.activeAlert = {
        obstacleId: obs.id,
        type: obs.type,
        tcpa: cpaResult.tcpaSeconds,
        cpa: cpaResult.cpaMeters
      };

      // Temporarily mark dynamic obstacle predicted trajectory envelope as impassable
      const tempImpassable = [];
      const horizon = Math.min(20.0, cpaResult.tcpaSeconds + 8.0);
      for (let t = 0; t <= horizon; t += 1.0) {
        const pos = obs.predictPosition(t);
        const rCells = Math.ceil((obs.radiusMeters + ship.computeRequiredClearanceMeters()) / this.grid.cellSizeMeters);
        for (let dy = -rCells; dy <= rCells; dy++) {
          for (let dx = -rCells; dx <= rCells; dx++) {
            const gx = Math.round(pos.x + dx);
            const gy = Math.round(pos.y + dy);
            if (this.grid.inBounds(gx, gy) && Math.hypot(dx, dy) <= rCells) {
              if (this.grid.cells[gy][gx] === this.grid.TYPE_SEA) {
                this.grid.cells[gy][gx] = this.grid.TYPE_LAND; // temporary virtual obstacle
                tempImpassable.push({ x: gx, y: gy });
              }
            }
          }
        }
      }

      // Recompute distance field with temporary virtual obstacle
      this.grid.computeDistanceField(ship.draft);

      // Current ship position as start of replanned Path B
      const replanStart = { x: ship.x, y: ship.y };
      const replanResult = planner.plan(replanStart, this.grid.destination, ship);

      // Restore temporary virtual obstacle cells
      for (let i = 0; i < tempImpassable.length; i++) {
        const c = tempImpassable[i];
        this.grid.cells[c.y][c.x] = this.grid.TYPE_SEA;
      }
      this.grid.computeDistanceField(ship.draft);

      if (replanResult.success && replanResult.path.length > 0) {
        // Activate Path B
        ship.setPath(replanResult.path, true);
        if (logCallback) {
          logCallback(`Replanning successful! Path B active. Length: ${replanResult.lengthMeters.toFixed(0)}m, Smoothness: ${replanResult.smoothnessDeg.toFixed(1)}deg.`, 'success');
        }
      } else {
        // Area ahead is blocked! Apply hold and wait manoeuvre
        if (logCallback) {
          logCallback('Fairway constricted. Applying HOLD-AND-WAIT reverse thrust manoeuvre...', 'warning');
        }
        ship.holdAndWait();

        // Check if completely trapped with no passage anywhere
        const fallbackCheck = planner.plan(replanStart, this.grid.destination, ship);
        if (!fallbackCheck.success) {
          ship.emergencyStop();
          if (logCallback) {
            logCallback('EMERGENCY SAFE STOP: No feasible collision-free corridor exists.', 'error');
          }
        }
      }
    }
  }
}

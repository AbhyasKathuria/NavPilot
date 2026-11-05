/**
 * NavPilot - Cargo Ship Real-Time Path-Planning Simulator
 * Dynamic Obstacles & AIS Collision Avoidance (dynamic.js)
 *
 * Implements:
 * - Dynamic obstacles moving continuously before, during, and after path planning
 * - AIS traffic collision risk evaluation against Ship Domain SS = L * B * D
 * - Automatic dynamic rerouting using the 3-Stage Hybrid Pipeline
 * - Hold-and-wait slowdown maneuver & emergency safe stop
 */

class DynamicObstacle {
  constructor(id, x, y, vx, vy, type = 'iceberg', radiusMeters = 30.0) {
    this.id = id;
    this.x = x;
    this.y = y;
    this.vx = vx;
    this.vy = vy;
    this.type = type; // 'iceberg', 'debris'
    this.radiusMeters = radiusMeters;
    this.heading = Math.atan2(vy, vx);
  }

  update(dt, grid) {
    this.x += this.vx * dt;
    this.y += this.vy * dt;

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

    const gx = Math.round(this.x);
    const gy = Math.round(this.y);
    if (grid.inBounds(gx, gy) && grid.cells[gy][gx] === grid.TYPE_LAND) {
      this.vx = -this.vx + (Math.random() - 0.5) * 0.2;
      this.vy = -this.vy + (Math.random() - 0.5) * 0.2;
    }
  }

  predictPosition(t) {
    return {
      x: this.x + this.vx * t,
      y: this.y + this.vy * t
    };
  }
}

class DynamicObstacleManager {
  constructor(grid, aisSystem = null) {
    this.grid = grid;
    this.aisSystem = aisSystem;
    this.obstacles = [];
    this.nextId = 1;

    this.lastReplanTime = 0;
    this.replanCooldown = 3.0; // seconds cooldown
    this.isReplanning = false;
    this.replanBannerTimer = 0;
  }

  setAISSystem(aisSystem) {
    this.aisSystem = aisSystem;
  }

  clear() {
    this.obstacles = [];
    this.isReplanning = false;
    this.replanBannerTimer = 0;
  }

  addObstacle(x, y, vx, vy, type = 'iceberg', radiusMeters = 30.0) {
    const obs = new DynamicObstacle(this.nextId++, x, y, vx, vy, type, radiusMeters);
    this.obstacles.push(obs);
    return obs;
  }

  addRandomObstacle() {
    for (let attempts = 0; attempts < 50; attempts++) {
      const x = 8 + Math.random() * (this.grid.cols - 16);
      const y = 8 + Math.random() * (this.grid.rows - 16);
      const gx = Math.round(x);
      const gy = Math.round(y);

      if (this.grid.inBounds(gx, gy) && this.grid.cells[gy][gx] === this.grid.TYPE_SEA) {
        const type = Math.random() > 0.5 ? 'iceberg' : 'debris';
        const speed = 0.2 + Math.random() * 0.4;
        const angle = Math.random() * Math.PI * 2;
        return this.addObstacle(x, y, Math.cos(angle) * speed, Math.sin(angle) * speed, type, 30.0);
      }
    }
  }

  update(dt) {
    // Dynamic obstacles move continuously before, during, and after planning
    for (let i = 0; i < this.obstacles.length; i++) {
      this.obstacles[i].update(dt, this.grid);
    }

    if (this.replanBannerTimer > 0) {
      this.replanBannerTimer -= dt;
      if (this.replanBannerTimer <= 0) {
        this.isReplanning = false;
      }
    }
  }

  /**
   * Evaluates collision threats against AIS vessels and dynamic debris,
   * triggering dynamic rerouting if necessary.
   */
  checkAndReroute(ship, hybridPlanner, logCallback) {
    if (!ship.activePath || ship.state === 'IDLE' || ship.state === 'ARRIVED') {
      return;
    }

    const now = performance.now() / 1000.0;
    if (now - this.lastReplanTime < this.replanCooldown) {
      return;
    }

    // Check AIS Traffic Risk against Ship Domain
    let collisionRisk = false;
    let threatName = '';
    let threatDetails = '';

    if (this.aisSystem) {
      const trafficRisk = this.aisSystem.evaluateTrafficRisk(ship, ship.lookaheadHorizonSeconds);
      if (trafficRisk.conflictDetected && trafficRisk.criticalVessel) {
        const v = trafficRisk.criticalVessel;
        collisionRisk = true;
        threatName = `${v.name} (MMSI: ${v.mmsi})`;
        threatDetails = `TCPA: ${trafficRisk.tcpaSeconds.toFixed(1)}s, CPA: ${trafficRisk.cpaMeters.toFixed(0)}m`;
      }
    }

    // Also check physical dynamic debris
    if (!collisionRisk) {
      const debrisThreat = this.evaluateDebrisRisk(ship);
      if (debrisThreat.conflictDetected) {
        collisionRisk = true;
        threatName = `Hazard #${debrisThreat.id} (${debrisThreat.type.toUpperCase()})`;
        threatDetails = `CPA: ${debrisThreat.cpaMeters.toFixed(0)}m`;
      }
    }

    if (collisionRisk) {
      this.lastReplanTime = now;
      this.isReplanning = true;
      this.replanBannerTimer = 4.0;

      if (logCallback) {
        logCallback(`Collision Risk Detected: ${threatName} encroaching Ship Domain (${threatDetails}).`, 'warning');
        logCallback('Initiating dynamic 3-Stage Hybrid Reroute (Stage 1 A* -> Stage 2 RRT* -> Stage 3 SMSO)...', 'info');
      }

      // Current position of ASV as start of rerouted route
      const rerouteStart = { x: Math.round(ship.x), y: Math.round(ship.y) };
      const dest = this.grid.destination;

      // Execute 3-Stage Hybrid Rerouting
      const rerouteResult = hybridPlanner.plan(rerouteStart, dest, ship, this.aisSystem);

      if (rerouteResult.success && rerouteResult.path.length > 0) {
        ship.setPath(rerouteResult.path, true);
        if (logCallback) {
          logCallback(`Dynamic Reroute successful! Path B active. Length: ${rerouteResult.lengthMeters.toFixed(0)}m, Energy Index: ${rerouteResult.energyIndex}.`, 'success');
        }
      } else {
        // Fairway constricted: apply hold-and-wait maneuver
        if (logCallback) {
          logCallback('Corridor constricted by oncoming traffic. Applying HOLD-AND-WAIT reverse thrust maneuver...', 'warning');
        }
        ship.holdAndWait();

        // Check if trapped
        const fallback = hybridPlanner.plan(rerouteStart, dest, ship, null);
        if (!fallback.success) {
          ship.emergencyStop();
          if (logCallback) {
            logCallback('EMERGENCY SAFE STOP: No navigable passage exists.', 'error');
          }
        }
      }
    }
  }

  evaluateDebrisRisk(ship) {
    const requiredClearance = ship.computeRequiredClearanceMeters();
    const shipSpeedCells = (ship.speed > 0.5 ? ship.speed : ship.maxSpeedMs) / this.grid.cellSizeMeters;

    for (let i = 0; i < this.obstacles.length; i++) {
      const obs = this.obstacles[i];
      const safetyDist = requiredClearance + obs.radiusMeters;

      for (let t = 1; t <= 15; t += 2) {
        const distAlong = shipSpeedCells * t;
        const asvPos = this.aisSystem ? this.aisSystem.getFutureAsvPosition(ship, distAlong) : null;
        if (!asvPos) break;

        const obsPos = obs.predictPosition(t);
        const d = Math.hypot(asvPos.x - obsPos.x, asvPos.y - obsPos.y) * this.grid.cellSizeMeters;
        if (d < safetyDist) {
          return { conflictDetected: true, id: obs.id, type: obs.type, cpaMeters: d };
        }
      }
    }

    return { conflictDetected: false };
  }
}

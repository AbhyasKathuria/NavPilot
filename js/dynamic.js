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
    if (Math.hypot(this.vx, this.vy) < 0.001) return;

    const prevX = this.x;
    const prevY = this.y;
    let nextX = prevX + this.vx * dt;
    let nextY = prevY + this.vy * dt;

    const pad = 1.0;
    if (nextX < pad) {
      nextX = pad;
      this.vx = Math.abs(this.vx);
    } else if (nextX > grid.cols - 1 - pad) {
      nextX = grid.cols - 1 - pad;
      this.vx = -Math.abs(this.vx);
    }

    if (nextY < pad) {
      nextY = pad;
      this.vy = Math.abs(this.vy);
    } else if (nextY > grid.rows - 1 - pad) {
      nextY = grid.rows - 1 - pad;
      this.vy = -Math.abs(this.vy);
    }

    const gx = Math.round(nextX);
    const gy = Math.round(nextY);
    if (!grid.inBounds(gx, gy) || grid.cells[gy][gx] === grid.TYPE_LAND) {
      nextX = prevX;
      nextY = prevY;
      this.vx = -this.vx;
      this.vy = -this.vy;
    }

    this.x = nextX;
    this.y = nextY;
    this.heading = Math.atan2(this.vy, this.vx);
  }

  predictPosition(t, grid = null) {
    let px = this.x + this.vx * t;
    let py = this.y + this.vy * t;
    if (grid) {
      px = Math.max(1, Math.min(grid.cols - 2, px));
      py = Math.max(1, Math.min(grid.rows - 2, py));
    }
    return { x: px, y: py };
  }
}

class DynamicObstacleManager {
  constructor(grid, aisSystem = null) {
    this.grid = grid;
    this.aisSystem = aisSystem;
    this.obstacles = [];
    this.nextId = 1;

    this.simTime = 0;
    this.lastReplanSimTime = -999;
    this.replanCooldownSimSec = 0.7; // Simulation-time cooldown so 5.0x never misses collisions
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
    this.lastReplanSimTime = -999;
  }

  addObstacle(x, y, vx, vy, type = 'iceberg', radiusMeters = 30.0) {
    const obs = new DynamicObstacle(this.nextId++, x, y, vx, vy, type, radiusMeters);
    this.obstacles.push(obs);
    return obs;
  }

  removeNear(x, y, maxDistCells = 2.0) {
    let bestIdx = -1;
    let bestDist = maxDistCells;
    for (let i = 0; i < this.obstacles.length; i++) {
      const d = Math.hypot(this.obstacles[i].x - x, this.obstacles[i].y - y);
      if (d <= bestDist) {
        bestDist = d;
        bestIdx = i;
      }
    }
    if (bestIdx !== -1) {
      return this.obstacles.splice(bestIdx, 1)[0];
    }
    return null;
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
    return null;
  }

  update(dt) {
    this.simTime += dt;

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
   * Spatiotemporal hazard check for path planners (A*, RRT*, Monkey Search).
   */
  isSpatiotemporalSafe(x, y, tSeconds = 0, shipDomainMeters = 48.0, extraMarginMeters = 18.0) {
    for (let i = 0; i < this.obstacles.length; i++) {
      const obs = this.obstacles[i];
      const safetyDist = shipDomainMeters + obs.radiusMeters + extraMarginMeters;

      const obsPos = obs.predictPosition(Math.min(tSeconds, 45.0), this.grid);
      const dMeters = Math.hypot(x - obsPos.x, y - obsPos.y) * this.grid.cellSizeMeters;
      if (dMeters < safetyDist) return false;

      if (tSeconds < 8.0) {
        const dNowMeters = Math.hypot(x - obs.x, y - obs.y) * this.grid.cellSizeMeters;
        if (dNowMeters < shipDomainMeters + obs.radiusMeters) return false;
      }
    }
    return true;
  }

  /**
   * Evaluates collision threats against AIS vessels and dynamic debris,
   * triggering dynamic rerouting or hold-and-wait if necessary.
   */
  checkAndReroute(ship, hybridPlanner, logCallback) {
    if (!ship.activePath || ship.state === 'IDLE' || ship.state === 'ARRIVED') {
      return;
    }

    const horizonSec = ship.lookaheadHorizonSeconds;
    const trafficRisk = this.aisSystem
      ? this.aisSystem.evaluateTrafficRisk(ship, horizonSec)
      : { conflictDetected: false, emergencyImminent: false };
    const debrisThreat = this.evaluateDebrisRisk(ship, horizonSec);

    const anyConflict = trafficRisk.conflictDetected || debrisThreat.conflictDetected;

    // If vessel was holding & waiting and path ahead is now clear, resume navigation!
    if (ship.state === 'HOLD_WAIT' && !anyConflict && !trafficRisk.emergencyImminent) {
      ship.state = 'CRUISING';
      ship.targetSpeed = ship.maxSpeedMs;
      if (logCallback) {
        logCallback('Fairway clear of dynamic traffic. Resuming voyage along optimal Path B.', 'success');
      }
      return;
    }

    // Immediate emergency shield if another vessel or hazard enters immediate hull envelope
    if (trafficRisk.emergencyImminent || debrisThreat.emergencyImminent) {
      if (ship.state !== 'HOLD_WAIT') {
        ship.holdAndWait();
        if (logCallback) {
          logCallback('IMMEDIATE PROXIMITY GUARD: Slowing / holding position while computing evasive avoidance route!', 'warning');
        }
      }
    }

    if (!anyConflict) {
      return;
    }

    // Check simulation-time debounce so high-speed simulations never miss evasive windows
    if (this.simTime - this.lastReplanSimTime < this.replanCooldownSimSec) {
      return;
    }

    let threatName = '';
    let threatDetails = '';
    if (trafficRisk.conflictDetected && trafficRisk.criticalVessel) {
      const v = trafficRisk.criticalVessel;
      threatName = `${v.name} (MMSI: ${v.mmsi})`;
      threatDetails = `TCPA: ${trafficRisk.tcpaSeconds.toFixed(1)}s, CPA: ${trafficRisk.cpaMeters.toFixed(0)}m`;
    } else if (debrisThreat.conflictDetected) {
      threatName = `Hazard #${debrisThreat.id} (${debrisThreat.type.toUpperCase()})`;
      threatDetails = `CPA: ${debrisThreat.cpaMeters.toFixed(0)}m`;
    }

    this.lastReplanSimTime = this.simTime;
    this.isReplanning = true;
    this.replanBannerTimer = 3.5;

    if (logCallback) {
      logCallback(`Collision Risk Detected: ${threatName} encroaching Ship Domain (${threatDetails}).`, 'warning');
      logCallback('Executing real-time 3-Stage Hybrid Avoidance Reroute (Path B)...', 'info');
    }

    // Keep exact passable continuous start position so ship never jumps into nearby coastline
    const rerouteStart = this.grid.isPassable(ship.x, ship.y, ship.draft, 16.0)
      ? { x: ship.x, y: ship.y }
      : this.grid.findNearestPassableCell(ship.x, ship.y, ship.draft, 18.0);
    const dest = this.grid.destination;

    const rerouteResult = hybridPlanner.plan(rerouteStart, dest, ship, this.aisSystem, {
      obstacleManager: this,
      monkeyIterations: 18
    });

    if (rerouteResult.success && rerouteResult.path && rerouteResult.path.length >= 2) {
      ship.setPath(rerouteResult.path, true);
      if (logCallback) {
        logCallback(`Dynamic Avoidance Route (Path B) active! Length: ${rerouteResult.lengthMeters.toFixed(0)}m, Min Clearance: ${rerouteResult.minClearanceMeters.toFixed(1)}m.`, 'success');
      }
    } else {
      if (ship.state !== 'HOLD_WAIT') {
        if (logCallback) {
          logCallback('Fairway constricted by approaching traffic. Applying HOLD-AND-WAIT safety maneuver...', 'warning');
        }
        ship.holdAndWait();
      }

      const staticFallback = hybridPlanner.plan(rerouteStart, dest, ship, null, {
        obstacleManager: null,
        monkeyIterations: 15
      });
      if (!staticFallback.success) {
        ship.emergencyStop();
        if (logCallback) {
          logCallback('EMERGENCY SAFE STOP: No navigable channel exists between hazards.', 'error');
        }
      }
    }
  }

  evaluateDebrisRisk(ship, horizonSeconds = 22.0) {
    const requiredClearance = ship.computeRequiredClearanceMeters();
    const shipSpeedCells = (ship.speed > 0.5 ? ship.speed : ship.maxSpeedMs) / this.grid.cellSizeMeters;

    let emergencyImminent = false;

    for (let i = 0; i < this.obstacles.length; i++) {
      const obs = this.obstacles[i];
      const safetyDist = requiredClearance + obs.radiusMeters + 12.0;

      const currentDist = Math.hypot(ship.x - obs.x, ship.y - obs.y) * this.grid.cellSizeMeters;
      if (currentDist < (ship.beam * 0.5 + obs.radiusMeters + 25.0)) {
        emergencyImminent = true;
      }
      if (currentDist < safetyDist) {
        return { conflictDetected: true, emergencyImminent, id: obs.id, type: obs.type, cpaMeters: currentDist };
      }

      const dtSample = 0.5;
      const steps = Math.ceil(horizonSeconds / dtSample);
      for (let s = 0; s <= steps; s++) {
        const t = s * dtSample;
        const distAlong = shipSpeedCells * t;
        const asvPos = (t === 0)
          ? { x: ship.x, y: ship.y }
          : (this.aisSystem ? this.aisSystem.getFutureAsvPosition(ship, distAlong) : null);
        if (!asvPos) break;

        const obsPos = obs.predictPosition(t, this.grid);
        const d = Math.hypot(asvPos.x - obsPos.x, asvPos.y - obsPos.y) * this.grid.cellSizeMeters;
        if (d < safetyDist) {
          return { conflictDetected: true, emergencyImminent, id: obs.id, type: obs.type, cpaMeters: d };
        }
      }
    }

    return { conflictDetected: false, emergencyImminent };
  }
}

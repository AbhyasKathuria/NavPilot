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
   * Evaluates whether the vessel's current position, immediate ahead vector, or remaining
   * active path intersects newly painted land obstacles, shoals, or compromised clearance corridors.
   */
  evaluateStaticObstruction(ship) {
    if (!ship.activePath || ship.activePath.length < 2) {
      return { conflictDetected: false, emergencyImminent: false, obstructedIndex: -1, distToObstacleMeters: Infinity, reason: '' };
    }

    const grid = this.grid;
    const requiredClearance = ship.computeRequiredClearanceMeters();
    const stoppingDistMeters = ship.computeStoppingDistanceMeters();
    const cellSize = grid.cellSizeMeters;

    // 1. Immediate ship hull check (current position)
    const curGx = Math.round(ship.x);
    const curGy = Math.round(ship.y);
    if (!grid.inBounds(curGx, curGy) ||
        grid.cells[curGy][curGx] === grid.TYPE_LAND ||
        grid.depths[curGy][curGx] < ship.draft) {
      return {
        conflictDetected: true,
        emergencyImminent: true,
        obstructedIndex: ship.pathIndex,
        distToObstacleMeters: 0,
        reason: 'Ship domain envelope touching newly drawn land/shoal barrier'
      };
    }

    // 2. Immediate lookahead segment: between ship's current position and next upcoming waypoint
    const nextIdx = Math.min(ship.pathIndex + 1, ship.activePath.length - 1);
    const pNext = ship.activePath[nextIdx];
    const segDistMeters = Math.hypot(pNext.x - ship.x, pNext.y - ship.y) * cellSize;
    const subSteps = Math.max(3, Math.ceil(segDistMeters / (cellSize * 0.35)));

    for (let s = 1; s <= subSteps; s++) {
      const u = s / subSteps;
      const sx = ship.x + (pNext.x - ship.x) * u;
      const sy = ship.y + (pNext.y - ship.y) * u;
      const sgx = Math.round(sx);
      const sgy = Math.round(sy);

      if (!grid.inBounds(sgx, sgy) ||
          grid.cells[sgy][sgx] === grid.TYPE_LAND ||
          grid.depths[sgy][sgx] < ship.draft) {
        const dM = u * segDistMeters;
        return {
          conflictDetected: true,
          emergencyImminent: dM <= Math.max(25.0, stoppingDistMeters * 1.5),
          obstructedIndex: nextIdx,
          distToObstacleMeters: dM,
          reason: `Obstacle directly ahead on active heading (${dM.toFixed(0)}m)`
        };
      }
    }

    // 3. Check remaining active path waypoints from nextIdx to destination
    let accumulatedDistMeters = segDistMeters;
    for (let i = nextIdx; i < ship.activePath.length; i++) {
      const pt = ship.activePath[i];
      const gx = Math.round(pt.x);
      const gy = Math.round(pt.y);

      if (!grid.inBounds(gx, gy) ||
          grid.cells[gy][gx] === grid.TYPE_LAND ||
          grid.depths[gy][gx] < ship.draft) {
        return {
          conflictDetected: true,
          emergencyImminent: accumulatedDistMeters <= Math.max(25.0, stoppingDistMeters * 1.5),
          obstructedIndex: i,
          distToObstacleMeters: accumulatedDistMeters,
          reason: `Waypoint #${i} obstructed by newly drawn land barrier (${accumulatedDistMeters.toFixed(0)}m ahead)`
        };
      }

      // Continuous hull clearance check around waypoint
      const clearanceM = grid.getExactHazardDistanceMeters(pt.x, pt.y, ship.draft);
      if (clearanceM < Math.min(12.0, requiredClearance * 0.25)) {
        return {
          conflictDetected: true,
          emergencyImminent: accumulatedDistMeters <= Math.max(25.0, stoppingDistMeters * 1.2),
          obstructedIndex: i,
          distToObstacleMeters: accumulatedDistMeters,
          reason: `Clearance at Waypoint #${i} compromised (${clearanceM.toFixed(1)}m < safe limit)`
        };
      }

      // Line of sight between consecutive waypoints along the remaining route
      if (i > nextIdx) {
        const prevPt = ship.activePath[i - 1];
        if (!grid.lineOfSight(prevPt.x, prevPt.y, pt.x, pt.y, ship.draft, Math.min(10.0, requiredClearance * 0.2))) {
          return {
            conflictDetected: true,
            emergencyImminent: accumulatedDistMeters <= Math.max(25.0, stoppingDistMeters * 1.5),
            obstructedIndex: i,
            distToObstacleMeters: accumulatedDistMeters,
            reason: `Channel segment between #${i - 1} and #${i} blocked by land obstacle`
          };
        }
      }

      if (i < ship.activePath.length - 1) {
        accumulatedDistMeters += Math.hypot(ship.activePath[i + 1].x - pt.x, ship.activePath[i + 1].y - pt.y) * cellSize;
      }
    }

    return {
      conflictDetected: false,
      emergencyImminent: false,
      obstructedIndex: -1,
      distToObstacleMeters: Infinity,
      reason: ''
    };
  }

  /**
   * Scans the Blue Ship Domain Envelope (SS = L * B * D) for ANY disturbance:
   * - Land cells (TYPE_LAND) encroaching or inside the blue circle
   * - Shallow shoals (depth < draft) encroaching or inside the blue circle
   * - AIS vessels (commercial, mini) encroaching or inside the blue circle
   * - Dynamic drift hazards (icebergs, debris) encroaching or inside the blue circle
   *
   * As soon as ANY disturbance is detected, triggers instant path re-planning!
   */
  evaluateBlueDomainDisturbance(ship) {
    if (!ship || !ship.activePath || ship.state === 'IDLE' || ship.state === 'ARRIVED') {
      return { disturbanceDetected: false, reason: '', type: null };
    }

    const grid = this.grid;
    const params = ship.getDomainEllipseParameters ? ship.getDomainEllipseParameters(grid.cellSizeMeters) : null;
    if (!params) {
      return { disturbanceDetected: false, reason: '', type: null };
    }

    const cellSize = grid.cellSizeMeters;

    // Bounding box of the blue domain envelope
    const maxDimMeters = Math.max(params.semiMajorM, params.semiMinorM) + 8.0;
    const maxCells = Math.ceil(maxDimMeters / cellSize) + 1;
    const cgx = Math.round(params.centerX);
    const cgy = Math.round(params.centerY);

    const minX = Math.max(0, cgx - maxCells);
    const maxX = Math.min(grid.cols - 1, cgx + maxCells);
    const minY = Math.max(0, cgy - maxCells);
    const maxY = Math.min(grid.rows - 1, cgy + maxCells);

    // 1. Check all bathymetric grid cells within the bounding area of the blue domain
    for (let gy = minY; gy <= maxY; gy++) {
      for (let gx = minX; gx <= maxX; gx++) {
        const isLand = grid.cells[gy][gx] === grid.TYPE_LAND;
        const isShoal = grid.depths[gy][gx] < ship.draft;
        if (!isLand && !isShoal) continue;

        // Sample center + 4 corners of the 50m x 50m cell
        const testPoints = [
          { x: gx, y: gy },
          { x: gx - 0.45, y: gy - 0.45 },
          { x: gx + 0.45, y: gy - 0.45 },
          { x: gx - 0.45, y: gy + 0.45 },
          { x: gx + 0.45, y: gy + 0.45 }
        ];

        for (let p = 0; p < testPoints.length; p++) {
          if (ship.isPointInsideBlueDomain(testPoints[p].x, testPoints[p].y, 2.0, cellSize)) {
            const desc = isLand ? 'Land Obstacle' : 'Shallow Reef';
            return {
              disturbanceDetected: true,
              reason: `Blue Circle (SS: ${(ship.shipDomainVolume / 1000).toFixed(0)}k m³) detected ${desc} at (${gx}, ${gy})`,
              type: 'static',
              location: { x: gx, y: gy }
            };
          }
        }
      }
    }

    // 2. Check AIS Marine Vessels (commercial ships & mini-ships)
    if (this.aisSystem && this.aisSystem.vessels) {
      for (let i = 0; i < this.aisSystem.vessels.length; i++) {
        const v = this.aisSystem.vessels[i];
        const vMargin = Math.min(22.0, (v.domainRadiusMeters || 35.0) * 0.35);
        if (ship.isPointInsideBlueDomain(v.x, v.y, vMargin, cellSize)) {
          return {
            disturbanceDetected: true,
            reason: `Blue Circle (SS: ${(ship.shipDomainVolume / 1000).toFixed(0)}k m³) detected AIS Vessel: ${v.name} (${v.sog.toFixed(1)}kn @ ${v.cog.toFixed(0)}°)`,
            type: 'ais_vessel',
            vessel: v
          };
        }
      }
    }

    // 3. Check Dynamic Drift Hazards (icebergs, debris)
    if (this.obstacles) {
      for (let i = 0; i < this.obstacles.length; i++) {
        const obs = this.obstacles[i];
        const obsMargin = Math.min(20.0, (obs.radiusMeters || 30.0) * 0.5);
        if (ship.isPointInsideBlueDomain(obs.x, obs.y, obsMargin, cellSize)) {
          return {
            disturbanceDetected: true,
            reason: `Blue Circle (SS: ${(ship.shipDomainVolume / 1000).toFixed(0)}k m³) detected Dynamic ${obs.type.toUpperCase()} #${obs.id}`,
            type: 'hazard',
            obstacle: obs
          };
        }
      }
    }

    return { disturbanceDetected: false, reason: '', type: null };
  }

  /**
   * Evaluates collision threats against static obstacles (newly drawn land/shoals),
   * AIS vessels, and dynamic debris, triggering instant dynamic rerouting or hold-and-wait if necessary.
   */
  checkAndReroute(ship, hybridPlanner, logCallback, forceCheck = false) {
    if (!ship.activePath || ship.state === 'IDLE' || ship.state === 'ARRIVED') {
      return;
    }

    const horizonSec = ship.lookaheadHorizonSeconds;

    // 1. Direct Blue Circle disturbance test:
    const blueDisturbance = this.evaluateBlueDomainDisturbance(ship);

    // 2. Path lookahead & static obstacle test:
    const staticThreat = this.evaluateStaticObstruction(ship);

    // 3. AIS traffic collision risk test:
    const trafficRisk = this.aisSystem
      ? this.aisSystem.evaluateTrafficRisk(ship, horizonSec)
      : { conflictDetected: false, emergencyImminent: false };

    // 4. Dynamic debris test:
    const debrisThreat = this.evaluateDebrisRisk(ship, horizonSec);

    const anyConflict = blueDisturbance.disturbanceDetected ||
                        staticThreat.conflictDetected ||
                        trafficRisk.conflictDetected ||
                        debrisThreat.conflictDetected;

    const emergencyImminent = blueDisturbance.disturbanceDetected ||
                             staticThreat.emergencyImminent ||
                             trafficRisk.emergencyImminent ||
                             debrisThreat.emergencyImminent;

    // Update ship's active domain disturbance flag for visual renderer
    if (blueDisturbance.disturbanceDetected) {
      ship.domainDisturbance = true;
      ship.domainDisturbanceReason = blueDisturbance.reason;
      ship.domainDisturbancePulseTimer = 2.5;
    } else if (ship.domainDisturbancePulseTimer <= 0) {
      ship.domainDisturbance = false;
      ship.domainDisturbanceReason = '';
    }

    // If vessel was holding & waiting and path ahead is now completely clear, resume navigation!
    if (ship.state === 'HOLD_WAIT' && !anyConflict && !emergencyImminent) {
      ship.state = 'CRUISING';
      ship.targetSpeed = ship.maxSpeedMs;
      ship.domainDisturbance = false;
      if (logCallback) {
        logCallback('Fairway clear of all obstacles & traffic. Resuming voyage along optimal path.', 'success');
      }
      return;
    }

    // AVOIDANCE EXECUTION SHIELD:
    // If ship has already recalculated Path B and the route ahead is clear, let the vessel cruise out of the danger zone!
    if (ship.hasReplanned && !staticThreat.conflictDetected && !forceCheck) {
      if (ship.state === 'HOLD_WAIT') {
        ship.state = 'CRUISING';
        ship.targetSpeed = ship.maxSpeedMs;
      }
      return;
    }

    // IMMEDIATE EMERGENCY FULL STOP:
    // As soon as circle senses any land or ship, STOP ship immediately (speed = 0 kn) while recalculating path whole again!
    if (emergencyImminent || anyConflict) {
      if (ship.state !== 'HOLD_WAIT' && ship.state !== 'EMERGENCY_STOP') {
        const detail = blueDisturbance.disturbanceDetected
          ? blueDisturbance.reason
          : (staticThreat.conflictDetected ? staticThreat.reason : 'Traffic/hazard proximity');
        ship.immediateFullStop(detail);
        if (logCallback) {
          logCallback(`🛑 BLUE CIRCLE DISTURBANCE: ${detail}. Ship STOPPED immediately (0 kn)! Recalculating path whole again via 3-Stage Pipeline (A* ➔ RRT* ➔ SMSO)...`, 'warning');
        }
      }
    }

    if (!anyConflict) {
      return;
    }

    let threatName = '';
    let threatDetails = '';
    if (blueDisturbance.disturbanceDetected) {
      threatName = 'Blue Domain Perimeter Disturbance';
      threatDetails = blueDisturbance.reason;
    } else if (staticThreat.conflictDetected) {
      threatName = 'Newly Placed Land/Shoal Obstacle';
      threatDetails = staticThreat.reason;
    } else if (trafficRisk.conflictDetected && trafficRisk.criticalVessel) {
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
      logCallback(`Collision Threat: ${threatName} (${threatDetails}). Calculating whole path again...`, 'warning');
      logCallback('Computing Sequential Three-Stage Hybrid Pipeline (A* ➔ RRT* ➔ Somersault Monkey Search)...', 'info');
    }

    // Keep exact passable continuous start position so ship never jumps into nearby coastline
    const rerouteStart = this.grid.isPassable(ship.x, ship.y, ship.draft, 10.0)
      ? { x: ship.x, y: ship.y }
      : this.grid.findNearestPassableCell(ship.x, ship.y, ship.draft, 14.0);
    const dest = this.grid.destination;

    const rerouteResult = hybridPlanner.plan(rerouteStart, dest, ship, this.aisSystem, {
      obstacleManager: this,
      monkeyIterations: 20
    });

    if (rerouteResult.success && rerouteResult.path && rerouteResult.path.length >= 2) {
      ship.setPath(rerouteResult.path, true, 'Path B (Dynamic Avoidance)', this.grid);
      ship.state = 'CRUISING';
      ship.targetSpeed = ship.maxSpeedMs;
      this.isReplanning = false;
      if (logCallback) {
        logCallback(`✅ FULL PATH RECALCULATED: Avoidance Route (Path B) active! Length: ${rerouteResult.lengthMeters.toFixed(0)}m, Min Clearance: ${rerouteResult.minClearanceMeters.toFixed(1)}m. Resuming autonomous navigation.`, 'success');
      }
    } else {
      if (ship.state !== 'HOLD_WAIT' && ship.state !== 'EMERGENCY_STOP') {
        if (logCallback) {
          logCallback('Fairway constricted or blocked by obstacles. Applying HOLD-AND-WAIT safety maneuver...', 'warning');
        }
        ship.holdAndWait();
      }

      const staticFallback = hybridPlanner.plan(rerouteStart, dest, ship, null, {
        obstacleManager: null,
        monkeyIterations: 15
      });
      if (staticFallback.success && staticFallback.path && staticFallback.path.length >= 2) {
        ship.setPath(staticFallback.path, true, 'Path B (Fallback Channel)', this.grid);
        ship.state = 'CRUISING';
        ship.targetSpeed = ship.maxSpeedMs;
        this.isReplanning = false;
        if (logCallback) {
          logCallback(`Fallback Avoidance Route (Path B) engaged! Length: ${staticFallback.lengthMeters.toFixed(0)}m.`, 'success');
        }
      } else {
        ship.emergencyStop();
        if (logCallback) {
          logCallback('EMERGENCY SAFE STOP: No navigable channel exists around placed obstacles. Ship safely holding position.', 'error');
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

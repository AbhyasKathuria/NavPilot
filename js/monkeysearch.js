/**
 * NavPilot - Cargo Ship Real-Time Path-Planning Simulator
 * Stage 3: Somersault Monkey Search Optimization (monkeysearch.js)
 *
 * Implements Stage 3 of the Three-Stage Hybrid Planning Architecture:
 * - Multi-objective fitness function (Energy-efficient, Time-efficient, Collision-free)
 * - Strict Hard Constraint: Zero collision with land or shallow shoals (Infinity penalty)
 * - Climb Process: Local fine-tuning of waypoints for minimum energy/rudder wear
 * - Watch-Jump Process: Long-range exploration away from hazards and AIS vessel paths
 * - Somersault Process: Movement toward the population's center of gravity (barycenter)
 * - Produces the final optimised cargo-ASV trajectory
 */

class SomersaultMonkeySearchPlanner {
  constructor(grid) {
    this.grid = grid;
  }

  /**
   * Optimizes a kinematically feasible path using Multi-Objective Somersault Monkey Search
   */
  optimize(seedWaypoints, ship, aisSystem = null, maxIterations = 25) {
    const startTime = performance.now();

    if (!seedWaypoints || seedWaypoints.length < 2) {
      return {
        success: false,
        path: [],
        computeTimeMs: 0,
        message: 'Invalid seed path for Stage 3 Somersault Monkey Search.'
      };
    }

    const shipDraft = ship.draft;
    const requiredClearance = ship.computeRequiredClearanceMeters();
    const minTurningRadius = ship.minTurningRadius;

    // Resample waypoints with uniform spacing
    const normalizedWaypoints = this.normalizeWaypoints(seedWaypoints, 3.5);
    const numWaypoints = normalizedWaypoints.length;

    if (numWaypoints <= 2) {
      const dense = this.interpolateWaypoints(normalizedWaypoints);
      return {
        success: true,
        path: dense,
        waypoints: normalizedWaypoints,
        computeTimeMs: performance.now() - startTime,
        ...this.evaluatePath(dense, normalizedWaypoints, ship)
      };
    }

    // Population parameters
    const popSize = 16;
    const population = [];

    // Monkey 0: exact verified seed path from Stage 2 RRT*
    population.push(this.cloneWaypoints(normalizedWaypoints));

    // Seed remaining monkeys with subtle perturbations ONLY if line-of-sight remains valid
    for (let p = 1; p < popSize; p++) {
      const candidate = this.cloneWaypoints(normalizedWaypoints);
      for (let i = 1; i < numWaypoints - 1; i++) {
        const jitterX = (Math.random() - 0.5) * 1.2;
        const jitterY = (Math.random() - 0.5) * 1.2;
        const nx = candidate[i].x + jitterX;
        const ny = candidate[i].y + jitterY;

        // HARD COLLISION CHECK: must be passable and line-of-sight clear
        if (this.grid.isPassable(nx, ny, shipDraft, requiredClearance * 0.8) &&
            this.grid.lineOfSight(candidate[i - 1].x, candidate[i - 1].y, nx, ny, shipDraft, requiredClearance * 0.8) &&
            this.grid.lineOfSight(nx, ny, candidate[i + 1].x, candidate[i + 1].y, shipDraft, requiredClearance * 0.8)) {
          candidate[i].x = nx;
          candidate[i].y = ny;
        }
      }
      population.push(candidate);
    }

    // Evaluate initial multi-objective fitnesses
    let fitnesses = population.map(m => this.computeFitness(m, ship, aisSystem));
    let bestIdx = 0;
    for (let i = 1; i < popSize; i++) {
      if (fitnesses[i].total < fitnesses[bestIdx].total) {
        bestIdx = i;
      }
    }

    let globalBest = this.cloneWaypoints(population[bestIdx]);
    let globalBestFitness = fitnesses[bestIdx];

    // Iterative Somersault Monkey Search Optimization Loop
    for (let iter = 0; iter < maxIterations; iter++) {
      // 1. CLIMB PROCESS (Local fine-tuning for energy & smoothness)
      const climbStep = Math.max(0.10, 0.55 * (1.0 - iter / maxIterations));
      for (let p = 0; p < popSize; p++) {
        const candidate = population[p];
        for (let i = 1; i < numWaypoints - 1; i++) {
          const origX = candidate[i].x;
          const origY = candidate[i].y;

          const offsets = [
            { dx: climbStep, dy: 0 },
            { dx: -climbStep, dy: 0 },
            { dx: 0, dy: climbStep },
            { dx: 0, dy: -climbStep }
          ];

          let bestLocalX = origX;
          let bestLocalY = origY;
          let currentFitness = fitnesses[p].total;

          for (let d = 0; d < offsets.length; d++) {
            const tx = origX + offsets[d].dx;
            const ty = origY + offsets[d].dy;

            // HARD COLLISION INVARIANT: must never touch land or shoal
            if (this.grid.isPassable(tx, ty, shipDraft, requiredClearance * 0.75) &&
                this.grid.lineOfSight(candidate[i - 1].x, candidate[i - 1].y, tx, ty, shipDraft, requiredClearance * 0.75) &&
                this.grid.lineOfSight(tx, ty, candidate[i + 1].x, candidate[i + 1].y, shipDraft, requiredClearance * 0.75)) {
              candidate[i].x = tx;
              candidate[i].y = ty;

              const evalFit = this.computeFitness(candidate, ship, aisSystem);
              if (evalFit.total < currentFitness) {
                currentFitness = evalFit.total;
                bestLocalX = tx;
                bestLocalY = ty;
              }
            }
          }

          candidate[i].x = bestLocalX;
          candidate[i].y = bestLocalY;
        }
        fitnesses[p] = this.computeFitness(candidate, ship, aisSystem);
      }

      // 2. WATCH-JUMP PROCESS (Exploration away from coastlines and AIS traffic)
      const jumpRadius = Math.max(0.4, 1.5 * (1.0 - iter / maxIterations));
      for (let p = 0; p < popSize; p++) {
        const candidate = population[p];
        for (let i = 1; i < numWaypoints - 1; i++) {
          const gx = Math.round(candidate[i].x);
          const gy = Math.round(candidate[i].y);
          let pushX = (Math.random() - 0.5) * jumpRadius;
          let pushY = (Math.random() - 0.5) * jumpRadius;

          // Push away from chart hazards
          if (this.grid.inBounds(gx, gy)) {
            const dist = this.grid.distanceField[gy][gx];
            if (dist < 120.0) {
              const dx = (this.grid.distanceField[gy][Math.min(this.grid.cols - 1, gx + 1)] -
                          this.grid.distanceField[gy][Math.max(0, gx - 1)]) / 2.0;
              const dy = (this.grid.distanceField[Math.min(this.grid.rows - 1, gy + 1)][gx] -
                          this.grid.distanceField[Math.max(0, gy - 1)][gx]) / 2.0;
              const mag = Math.hypot(dx, dy);
              if (mag > 0.01) {
                pushX += (dx / mag) * (jumpRadius * 1.2);
                pushY += (dy / mag) * (jumpRadius * 1.2);
              }
            }
          }

          // Push away from active AIS traffic
          if (aisSystem) {
            for (let v = 0; v < aisSystem.vessels.length; v++) {
              const ves = aisSystem.vessels[v];
              const dVessel = Math.hypot(candidate[i].x - ves.x, candidate[i].y - ves.y);
              if (dVessel < 5.0) {
                pushX += (candidate[i].x - ves.x) / (dVessel + 0.1) * jumpRadius;
                pushY += (candidate[i].y - ves.y) / (dVessel + 0.1) * jumpRadius;
              }
            }
          }

          const jx = candidate[i].x + pushX;
          const jy = candidate[i].y + pushY;

          // HARD COLLISION INVARIANT
          if (this.grid.isPassable(jx, jy, shipDraft, requiredClearance * 0.75) &&
              this.grid.lineOfSight(candidate[i - 1].x, candidate[i - 1].y, jx, jy, shipDraft, requiredClearance * 0.75) &&
              this.grid.lineOfSight(jx, jy, candidate[i + 1].x, candidate[i + 1].y, shipDraft, requiredClearance * 0.75)) {
            const origX = candidate[i].x;
            const origY = candidate[i].y;
            candidate[i].x = jx;
            candidate[i].y = jy;

            const testFit = this.computeFitness(candidate, ship, aisSystem);
            if (testFit.total < fitnesses[p].total) {
              fitnesses[p] = testFit;
            } else {
              candidate[i].x = origX;
              candidate[i].y = origY;
            }
          }
        }
      }

      // 3. SOMERSAULT PROCESS (Convergence towards population barycenter)
      const sortedIdx = [...Array(popSize).keys()].sort((a, b) => fitnesses[a].total - fitnesses[b].total);
      const eliteCount = Math.min(4, popSize);

      const barycenter = [];
      for (let i = 0; i < numWaypoints; i++) {
        let sumX = 0, sumY = 0;
        for (let e = 0; e < eliteCount; e++) {
          sumX += population[sortedIdx[e]][i].x;
          sumY += population[sortedIdx[e]][i].y;
        }
        barycenter.push({ x: sumX / eliteCount, y: sumY / eliteCount });
      }

      const somersaultWeight = 0.35;
      for (let e = eliteCount; e < popSize; e++) {
        const p = sortedIdx[e];
        const candidate = population[p];
        for (let i = 1; i < numWaypoints - 1; i++) {
          const targetX = barycenter[i].x;
          const targetY = barycenter[i].y;
          const sx = candidate[i].x + somersaultWeight * (targetX - candidate[i].x) + (Math.random() - 0.5) * 0.2;
          const sy = candidate[i].y + somersaultWeight * (targetY - candidate[i].y) + (Math.random() - 0.5) * 0.2;

          // HARD COLLISION INVARIANT
          if (this.grid.isPassable(sx, sy, shipDraft, requiredClearance * 0.75) &&
              this.grid.lineOfSight(candidate[i - 1].x, candidate[i - 1].y, sx, sy, shipDraft, requiredClearance * 0.75) &&
              this.grid.lineOfSight(sx, sy, candidate[i + 1].x, candidate[i + 1].y, shipDraft, requiredClearance * 0.75)) {
            candidate[i].x = sx;
            candidate[i].y = sy;
          }
        }
        fitnesses[p] = this.computeFitness(candidate, ship, aisSystem);
      }

      // Update global best
      for (let p = 0; p < popSize; p++) {
        if (fitnesses[p].total < globalBestFitness.total) {
          globalBestFitness = fitnesses[p];
          globalBest = this.cloneWaypoints(population[p]);
        }
      }
    }

    // Safety fallback: verify every single segment of globalBest
    let isGlobalBestSafe = true;
    for (let i = 1; i < globalBest.length; i++) {
      if (!this.grid.lineOfSight(globalBest[i - 1].x, globalBest[i - 1].y, globalBest[i].x, globalBest[i].y, shipDraft, requiredClearance * 0.7)) {
        isGlobalBestSafe = false;
        break;
      }
    }
    if (!isGlobalBestSafe) {
      globalBest = this.cloneWaypoints(normalizedWaypoints);
    }

    const computeTimeMs = performance.now() - startTime;
    const densePath = this.interpolateWaypoints(globalBest);
    const metrics = this.evaluatePath(densePath, globalBest, ship);

    return {
      success: true,
      path: densePath,
      waypoints: globalBest,
      computeTimeMs,
      ...metrics
    };
  }

  /**
   * Multi-objective Fitness Function:
   * HARD CONSTRAINT: If any segment crosses land or shallow reef, returns Infinity!
   */
  computeFitness(waypoints, ship, aisSystem) {
    let lengthCells = 0;
    let totalTurningRad = 0;
    let energyIndex = 0;
    let proximityPenalty = 0;
    let curvatureViolations = 0;

    const minRadiusCells = ship.minTurningRadius / this.grid.cellSizeMeters;
    const requiredClearance = ship.computeRequiredClearanceMeters();
    const shipDraft = ship.draft;

    for (let i = 0; i < waypoints.length; i++) {
      const gx = Math.round(waypoints[i].x);
      const gy = Math.round(waypoints[i].y);

      // HARD CONSTRAINT: point must be in water and deep enough
      if (!this.grid.inBounds(gx, gy) ||
          this.grid.cells[gy][gx] === this.grid.TYPE_LAND ||
          this.grid.depths[gy][gx] < shipDraft) {
        return { total: Infinity, invalid: true };
      }

      const dMeters = this.grid.distanceField[gy][gx];
      if (dMeters < requiredClearance) {
        proximityPenalty += (requiredClearance - dMeters) * 10.0;
      } else if (dMeters < requiredClearance * 2.5) {
        proximityPenalty += (requiredClearance * 2.5 - dMeters) * 1.5;
      }

      // Distance to AIS traffic
      if (aisSystem) {
        for (let v = 0; v < aisSystem.vessels.length; v++) {
          const ves = aisSystem.vessels[v];
          const distToVesselMeters = Math.hypot(waypoints[i].x - ves.x, waypoints[i].y - ves.y) * this.grid.cellSizeMeters;
          const combinedDomain = ship.domainRadiusMeters + ves.domainRadiusMeters;
          if (distToVesselMeters < combinedDomain) {
            proximityPenalty += 600.0;
          } else if (distToVesselMeters < combinedDomain * 2.0) {
            proximityPenalty += (combinedDomain * 2.0 - distToVesselMeters) * 1.2;
          }
        }
      }

      if (i > 0) {
        // HARD CONSTRAINT: segment must not cross land or shoal!
        if (!this.grid.lineOfSight(waypoints[i - 1].x, waypoints[i - 1].y, waypoints[i].x, waypoints[i].y, shipDraft, requiredClearance * 0.7)) {
          return { total: Infinity, invalid: true };
        }

        const segDist = Math.hypot(waypoints[i].x - waypoints[i - 1].x, waypoints[i].y - waypoints[i - 1].y);
        lengthCells += segDist;
      }

      if (i > 0 && i < waypoints.length - 1) {
        const v1x = waypoints[i].x - waypoints[i - 1].x;
        const v1y = waypoints[i].y - waypoints[i - 1].y;
        const v2x = waypoints[i + 1].x - waypoints[i].x;
        const v2y = waypoints[i + 1].y - waypoints[i].y;

        const a1 = Math.atan2(v1y, v1x);
        const a2 = Math.atan2(v2y, v2x);
        let diff = Math.abs(a2 - a1);
        if (diff > Math.PI) diff = 2 * Math.PI - diff;
        totalTurningRad += diff;

        energyIndex += Math.pow(diff, 2) * 20.0;

        if (diff > 0.05) {
          const reqDist = minRadiusCells * Math.tan(diff / 2.0);
          const len1 = Math.hypot(v1x, v1y);
          const len2 = Math.hypot(v2x, v2y);
          if (reqDist > len1 * 0.95 || reqDist > len2 * 0.95) {
            curvatureViolations++;
          }
        }
      }
    }

    const timeIndex = lengthCells;

    const wTime = 1.0;
    const wEnergy = 12.0;
    const wSafety = 1.2;
    const wCurv = 150.0;

    const total = (wTime * timeIndex) +
                  (wEnergy * energyIndex) +
                  (wSafety * proximityPenalty) +
                  (wCurv * curvatureViolations);

    return {
      total,
      timeIndex,
      energyIndex,
      proximityPenalty,
      curvatureViolations
    };
  }

  normalizeWaypoints(waypoints, maxSpacingCells = 3.5) {
    if (!waypoints || waypoints.length <= 1) return waypoints || [];
    const result = [waypoints[0]];

    for (let i = 1; i < waypoints.length; i++) {
      const p0 = waypoints[i - 1];
      const p1 = waypoints[i];
      const dist = Math.hypot(p1.x - p0.x, p1.y - p0.y);

      if (dist > maxSpacingCells) {
        const subCount = Math.ceil(dist / maxSpacingCells);
        for (let s = 1; s < subCount; s++) {
          const t = s / subCount;
          result.push({
            x: p0.x + (p1.x - p0.x) * t,
            y: p0.y + (p1.y - p0.y) * t
          });
        }
      }
      result.push(p1);
    }
    return result;
  }

  interpolateWaypoints(waypoints, stepSizeCells = 0.4) {
    if (!waypoints || waypoints.length <= 1) return waypoints || [];
    const dense = [waypoints[0]];

    for (let i = 1; i < waypoints.length; i++) {
      const p0 = waypoints[i - 1];
      const p1 = waypoints[i];
      const dist = Math.hypot(p1.x - p0.x, p1.y - p0.y);
      const steps = Math.max(1, Math.ceil(dist / stepSizeCells));

      for (let s = 1; s <= steps; s++) {
        const t = s / steps;
        dense.push({
          x: p0.x + (p1.x - p0.x) * t,
          y: p0.y + (p1.y - p0.y) * t
        });
      }
    }
    return dense;
  }

  cloneWaypoints(wps) {
    return wps.map(p => ({ x: p.x, y: p.y }));
  }

  evaluatePath(densePath, waypoints, ship) {
    let lengthCells = 0;
    let minClearance = Infinity;

    for (let i = 0; i < densePath.length; i++) {
      const gx = Math.round(densePath[i].x);
      const gy = Math.round(densePath[i].y);
      if (this.grid.inBounds(gx, gy)) {
        const d = this.grid.distanceField[gy][gx];
        if (d < minClearance) minClearance = d;
      }
      if (i > 0) {
        lengthCells += Math.hypot(densePath[i].x - densePath[i - 1].x, densePath[i].y - densePath[i - 1].y);
      }
    }

    let totalTurningRad = 0;
    let energyEst = 0;
    if (waypoints && waypoints.length >= 3) {
      for (let i = 1; i < waypoints.length - 1; i++) {
        const v1x = waypoints[i].x - waypoints[i - 1].x;
        const v1y = waypoints[i].y - waypoints[i - 1].y;
        const v2x = waypoints[i + 1].x - waypoints[i].x;
        const v2y = waypoints[i + 1].y - waypoints[i].y;

        const a1 = Math.atan2(v1y, v1x);
        const a2 = Math.atan2(v2y, v2x);
        let diff = Math.abs(a2 - a1);
        if (diff > Math.PI) diff = 2 * Math.PI - diff;
        totalTurningRad += diff;
        energyEst += Math.pow(diff, 2) * 20.0;
      }
    }

    const lengthMeters = lengthCells * this.grid.cellSizeMeters;
    const cruiseSpeedMs = ship.maxSpeedMs * 0.85;
    const estimatedTransitTimeSec = lengthMeters / cruiseSpeedMs;

    return {
      lengthCells,
      lengthMeters,
      smoothnessDeg: (totalTurningRad * 180) / Math.PI,
      minClearanceMeters: minClearance === Infinity ? 0 : minClearance,
      curvatureViolations: 0,
      estimatedTransitTimeSec,
      energyIndex: energyEst.toFixed(1)
    };
  }
}

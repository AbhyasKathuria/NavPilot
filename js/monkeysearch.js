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
   * Optimizes a kinematically feasible path using Multi-Objective Somersault Monkey Search.
   * Supports AIS vessels, dynamic debris, exact continuous edge checks, and swarm animation history.
   */
  optimize(seedWaypoints, ship, aisSystem = null, maxIterations = 25, obstacleManager = null) {
    const startTime = performance.now();

    if (!seedWaypoints || seedWaypoints.length < 2) {
      return {
        success: false,
        path: [],
        iterationHistory: [],
        computeTimeMs: 0,
        message: 'Invalid seed path for Stage 3 Somersault Monkey Search.'
      };
    }

    const shipDraft = ship.draft;
    const requiredClearance = ship.computeRequiredClearanceMeters();
    const minTurningRadius = ship.minTurningRadius;

    // Resample waypoints cleanly while keeping search crisp and responsive
    const normalizedWaypoints = this.normalizeWaypoints(seedWaypoints, 8.5);
    const numWaypoints = normalizedWaypoints.length;
    const iterationHistory = [];

    if (numWaypoints <= 2) {
      const dense = this.interpolateWaypoints(normalizedWaypoints);
      iterationHistory.push(this.cloneWaypoints(normalizedWaypoints));
      return {
        success: true,
        path: dense,
        waypoints: normalizedWaypoints,
        iterationHistory,
        computeTimeMs: performance.now() - startTime,
        ...this.evaluatePath(dense, normalizedWaypoints, ship)
      };
    }

    const popSize = 10;
    const effIters = Math.min(14, maxIterations);
    const population = [];

    // Monkey 0: verified seed path from Stage 2 RRT*
    population.push(this.cloneWaypoints(normalizedWaypoints));

    // Seed remaining monkeys with valid perturbations
    for (let p = 1; p < popSize; p++) {
      const candidate = this.cloneWaypoints(normalizedWaypoints);
      for (let i = 1; i < numWaypoints - 1; i++) {
        const jitterX = (Math.random() - 0.5) * 1.4;
        const jitterY = (Math.random() - 0.5) * 1.4;
        const nx = candidate[i].x + jitterX;
        const ny = candidate[i].y + jitterY;

        if (this.grid.isPassable(nx, ny, shipDraft, requiredClearance) &&
            this.grid.lineOfSight(candidate[i - 1].x, candidate[i - 1].y, nx, ny, shipDraft, requiredClearance, 0.5) &&
            this.grid.lineOfSight(nx, ny, candidate[i + 1].x, candidate[i + 1].y, shipDraft, requiredClearance, 0.5)) {
          candidate[i].x = nx;
          candidate[i].y = ny;
        }
      }
      population.push(candidate);
    }

    let fitnesses = population.map(m => this.computeFitness(m, ship, aisSystem, obstacleManager, true));
    let bestIdx = 0;
    for (let i = 1; i < popSize; i++) {
      if (fitnesses[i].total < fitnesses[bestIdx].total) {
        bestIdx = i;
      }
    }

    let globalBest = this.cloneWaypoints(population[bestIdx]);
    let globalBestFitness = fitnesses[bestIdx];
    iterationHistory.push(this.cloneWaypoints(globalBest));

    for (let iter = 0; iter < effIters; iter++) {
      // 1. CLIMB PROCESS (Local fine-tuning for minimum energy & path length)
      const climbStep = Math.max(0.15, 0.65 * (1.0 - iter / effIters));
      for (let p = 0; p < popSize; p++) {
        const candidate = population[p];
        for (let i = 1; i < numWaypoints - 1; i++) {
          const origX = candidate[i].x;
          const origY = candidate[i].y;

          const midX = (candidate[i - 1].x + candidate[i + 1].x) * 0.5;
          const midY = (candidate[i - 1].y + candidate[i + 1].y) * 0.5;
          const pullDx = (midX - origX) * 0.4;
          const pullDy = (midY - origY) * 0.4;

          const offsets = [
            { dx: climbStep, dy: 0 },
            { dx: -climbStep, dy: 0 },
            { dx: 0, dy: climbStep },
            { dx: 0, dy: -climbStep },
            { dx: pullDx, dy: pullDy }
          ];

          let bestLocalX = origX;
          let bestLocalY = origY;
          let currentFitness = fitnesses[p].total;

          for (let d = 0; d < offsets.length; d++) {
            const tx = origX + offsets[d].dx;
            const ty = origY + offsets[d].dy;

            if (this.grid.isPassable(tx, ty, shipDraft, requiredClearance) &&
                this.grid.lineOfSight(candidate[i - 1].x, candidate[i - 1].y, tx, ty, shipDraft, requiredClearance, 0.5) &&
                this.grid.lineOfSight(tx, ty, candidate[i + 1].x, candidate[i + 1].y, shipDraft, requiredClearance, 0.5)) {
              candidate[i].x = tx;
              candidate[i].y = ty;

              const evalFit = this.computeFitness(candidate, ship, aisSystem, obstacleManager, true);
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
        fitnesses[p] = this.computeFitness(candidate, ship, aisSystem, obstacleManager, true);
      }

      // 2. WATCH-JUMP PROCESS (Exploration away from coastlines, shoals, AIS ships & drifting hazards)
      const jumpRadius = Math.max(0.35, 1.4 * (1.0 - iter / effIters));
      for (let p = 0; p < popSize; p++) {
        const candidate = population[p];
        for (let i = 1; i < numWaypoints - 1; i++) {
          let pushX = (Math.random() - 0.5) * jumpRadius;
          let pushY = (Math.random() - 0.5) * jumpRadius;

          const edgeDist = this.grid.getExactHazardDistanceMeters(candidate[i].x, candidate[i].y, shipDraft);
          if (edgeDist < 75.0) {
            const dx = this.grid.getExactHazardDistanceMeters(candidate[i].x + 0.5, candidate[i].y, shipDraft) -
                       this.grid.getExactHazardDistanceMeters(candidate[i].x - 0.5, candidate[i].y, shipDraft);
            const dy = this.grid.getExactHazardDistanceMeters(candidate[i].x, candidate[i].y + 0.5, shipDraft) -
                       this.grid.getExactHazardDistanceMeters(candidate[i].x, candidate[i].y - 0.5, shipDraft);
            const mag = Math.hypot(dx, dy);
            if (mag > 0.01) {
              pushX += (dx / mag) * (jumpRadius * 1.1);
              pushY += (dy / mag) * (jumpRadius * 1.1);
            }
          }

          if (aisSystem) {
            for (let v = 0; v < aisSystem.vessels.length; v++) {
              const ves = aisSystem.vessels[v];
              const dVessel = Math.hypot(candidate[i].x - ves.x, candidate[i].y - ves.y);
              if (dVessel < 5.0) {
                pushX += ((candidate[i].x - ves.x) / (dVessel + 0.1)) * jumpRadius;
                pushY += ((candidate[i].y - ves.y) / (dVessel + 0.1)) * jumpRadius;
              }
            }
          }

          if (obstacleManager) {
            for (let o = 0; o < obstacleManager.obstacles.length; o++) {
              const obs = obstacleManager.obstacles[o];
              const dObs = Math.hypot(candidate[i].x - obs.x, candidate[i].y - obs.y);
              if (dObs < 4.5) {
                pushX += ((candidate[i].x - obs.x) / (dObs + 0.1)) * jumpRadius;
                pushY += ((candidate[i].y - obs.y) / (dObs + 0.1)) * jumpRadius;
              }
            }
          }

          const jx = candidate[i].x + pushX;
          const jy = candidate[i].y + pushY;

          if (this.grid.isPassable(jx, jy, shipDraft, requiredClearance) &&
              this.grid.lineOfSight(candidate[i - 1].x, candidate[i - 1].y, jx, jy, shipDraft, requiredClearance, 0.5) &&
              this.grid.lineOfSight(jx, jy, candidate[i + 1].x, candidate[i + 1].y, shipDraft, requiredClearance, 0.5)) {
            const origX = candidate[i].x;
            const origY = candidate[i].y;
            candidate[i].x = jx;
            candidate[i].y = jy;

            const testFit = this.computeFitness(candidate, ship, aisSystem, obstacleManager, true);
            if (testFit.total < fitnesses[p].total) {
              fitnesses[p] = testFit;
            } else {
              candidate[i].x = origX;
              candidate[i].y = origY;
            }
          }
        }
      }

      // 3. SOMERSAULT PROCESS (Convergence towards elite barycenter)
      const sortedIdx = [...Array(popSize).keys()].sort((a, b) => fitnesses[a].total - fitnesses[b].total);
      const eliteCount = Math.min(3, popSize);

      const barycenter = [];
      for (let i = 0; i < numWaypoints; i++) {
        let sumX = 0, sumY = 0;
        for (let e = 0; e < eliteCount; e++) {
          sumX += population[sortedIdx[e]][i].x;
          sumY += population[sortedIdx[e]][i].y;
        }
        barycenter.push({ x: sumX / eliteCount, y: sumY / eliteCount });
      }

      const somersaultWeight = 0.4;
      for (let e = eliteCount; e < popSize; e++) {
        const p = sortedIdx[e];
        const candidate = population[p];
        for (let i = 1; i < numWaypoints - 1; i++) {
          const targetX = barycenter[i].x;
          const targetY = barycenter[i].y;
          const sx = candidate[i].x + somersaultWeight * (targetX - candidate[i].x) + (Math.random() - 0.5) * 0.2;
          const sy = candidate[i].y + somersaultWeight * (targetY - candidate[i].y) + (Math.random() - 0.5) * 0.2;

          if (this.grid.isPassable(sx, sy, shipDraft, requiredClearance) &&
              this.grid.lineOfSight(candidate[i - 1].x, candidate[i - 1].y, sx, sy, shipDraft, requiredClearance, 0.5) &&
              this.grid.lineOfSight(sx, sy, candidate[i + 1].x, candidate[i + 1].y, shipDraft, requiredClearance, 0.5)) {
            candidate[i].x = sx;
            candidate[i].y = sy;
          }
        }
        fitnesses[p] = this.computeFitness(candidate, ship, aisSystem, obstacleManager, true);
      }

      for (let p = 0; p < popSize; p++) {
        if (fitnesses[p].total < globalBestFitness.total) {
          globalBestFitness = fitnesses[p];
          globalBest = this.cloneWaypoints(population[p]);
        }
      }

      if (iter % 2 === 0 || iter === effIters - 1) {
        iterationHistory.push(this.cloneWaypoints(globalBest));
      }
    }

    // Prune collinear / redundant intermediate bends (strictly verifying static + dynamic safety)
    const prunedBest = this.pruneRedundantWaypoints(globalBest, ship, aisSystem, obstacleManager);
    const prunedFit = this.computeFitness(prunedBest, ship, aisSystem, obstacleManager);
    if (!prunedFit.invalid && prunedFit.total <= globalBestFitness.total) {
      globalBest = prunedBest;
    }

    // Final safety invariant over every single segment
    let isGlobalBestSafe = true;
    for (let i = 1; i < globalBest.length; i++) {
      if (!this.grid.lineOfSight(globalBest[i - 1].x, globalBest[i - 1].y, globalBest[i].x, globalBest[i].y, shipDraft, requiredClearance)) {
        isGlobalBestSafe = false;
        break;
      }
    }
    if (!isGlobalBestSafe) {
      globalBest = this.cloneWaypoints(seedWaypoints);
    }

    iterationHistory.push(this.cloneWaypoints(globalBest));

    const computeTimeMs = performance.now() - startTime;
    const densePath = this.interpolateWaypoints(globalBest);
    const metrics = this.evaluatePath(densePath, globalBest, ship);

    return {
      success: true,
      path: densePath,
      waypoints: globalBest,
      iterationHistory,
      computeTimeMs,
      ...metrics
    };
  }

  pruneRedundantWaypoints(waypoints, ship, aisSystem, obstacleManager) {
    if (!waypoints || waypoints.length <= 2) return waypoints;
    const draft = ship.draft;
    const clearance = ship.computeRequiredClearanceMeters();
    const cleaned = [waypoints[0]];
    let i = 0;

    while (i < waypoints.length - 1) {
      let chosen = i + 1;
      for (let j = waypoints.length - 1; j > i + 1; j--) {
        if (this.grid.lineOfSight(waypoints[i].x, waypoints[i].y, waypoints[j].x, waypoints[j].y, draft, clearance)) {
          // Verify pruned shortcut does not worsen overall multi-objective cost
          const candidateTest = [...cleaned, waypoints[j], ...waypoints.slice(j + 1)];
          const currTest = [...cleaned, ...waypoints.slice(i + 1)];
          const fCand = this.computeFitness(candidateTest, ship, aisSystem, obstacleManager);
          const fCurr = this.computeFitness(currTest, ship, aisSystem, obstacleManager);
          if (!fCand.invalid && fCand.total <= fCurr.total) {
            chosen = j;
            break;
          }
        }
      }
      cleaned.push(waypoints[chosen]);
      i = chosen;
    }
    return cleaned;
  }

  /**
   * Multi-objective Fitness Function:
   * HARD CONSTRAINT: If any segment crosses land, shallow reef, or ship domain -> Infinity!
   */
  computeFitness(waypoints, ship, aisSystem, obstacleManager = null, skipLos = false) {
    let lengthCells = 0;
    let totalTurningRad = 0;
    let energyIndex = 0;
    let proximityPenalty = 0;
    let curvatureViolations = 0;

    const minRadiusCells = ship.minTurningRadius / this.grid.cellSizeMeters;
    const requiredClearance = ship.computeRequiredClearanceMeters();
    const shipDraft = ship.draft;
    const cruiseSpeedCells = (ship.maxSpeedMs * 0.85) / this.grid.cellSizeMeters;

    for (let i = 0; i < waypoints.length; i++) {
      const gx = Math.round(waypoints[i].x);
      const gy = Math.round(waypoints[i].y);

      if (!this.grid.inBounds(gx, gy) ||
          this.grid.cells[gy][gx] === this.grid.TYPE_LAND ||
          this.grid.depths[gy][gx] < shipDraft) {
        return { total: Infinity, invalid: true };
      }

      if (!skipLos) {
        const edgeDistMeters = this.grid.getExactHazardDistanceMeters(waypoints[i].x, waypoints[i].y, shipDraft);
        const minEdgeReq = this.grid.toEdgeClearanceMeters(requiredClearance);
        if (edgeDistMeters < minEdgeReq) {
          return { total: Infinity, invalid: true };
        }
        if (edgeDistMeters < 65.0) {
          proximityPenalty += (65.0 - edgeDistMeters) * 1.4;
        }
      } else {
        const fastD = Math.max(0, this.grid.distanceField[gy][gx] - 25.0);
        if (fastD < 65.0) {
          proximityPenalty += (65.0 - fastD) * 1.4;
        }
      }

      const arrivalTimeSec = cruiseSpeedCells > 0 ? lengthCells / cruiseSpeedCells : 0;

      // Spatiotemporal distance to AIS ships & mini ships
      if (aisSystem && aisSystem.vessels.length > 0) {
        for (let v = 0; v < aisSystem.vessels.length; v++) {
          const ves = aisSystem.vessels[v];
          const vPos = ves.sog <= 0.2 ? { x: ves.x, y: ves.y } : ves.predictPosition(Math.min(arrivalTimeSec, 45.0), this.grid);
          const distToVesselMeters = Math.hypot(waypoints[i].x - vPos.x, waypoints[i].y - vPos.y) * this.grid.cellSizeMeters;
          const combinedDomain = ship.domainRadiusMeters + ves.domainRadiusMeters;
          if (distToVesselMeters < combinedDomain) {
            proximityPenalty += 900.0;
          } else if (distToVesselMeters < combinedDomain * 2.0) {
            proximityPenalty += (combinedDomain * 2.0 - distToVesselMeters) * 1.5;
          }
        }
      }

      // Spatiotemporal distance to drifting hazards
      if (obstacleManager && obstacleManager.obstacles.length > 0) {
        for (let o = 0; o < obstacleManager.obstacles.length; o++) {
          const obs = obstacleManager.obstacles[o];
          const oPos = obs.predictPosition(Math.min(arrivalTimeSec, 45.0), this.grid);
          const distToObsMeters = Math.hypot(waypoints[i].x - oPos.x, waypoints[i].y - oPos.y) * this.grid.cellSizeMeters;
          const safetyDist = ship.domainRadiusMeters + obs.radiusMeters;
          if (distToObsMeters < safetyDist) {
            proximityPenalty += 900.0;
          } else if (distToObsMeters < safetyDist * 1.8) {
            proximityPenalty += (safetyDist * 1.8 - distToObsMeters) * 1.2;
          }
        }
      }

      if (i > 0) {
        if (!skipLos && !this.grid.lineOfSight(waypoints[i - 1].x, waypoints[i - 1].y, waypoints[i].x, waypoints[i].y, shipDraft, requiredClearance)) {
          return { total: Infinity, invalid: true };
        }
        const segDist = Math.hypot(waypoints[i].x - waypoints[i - 1].x, waypoints[i].y - waypoints[i - 1].y);

        // Check intermediate points along segment against AIS traffic & dynamic debris
        const hasDynamic = (aisSystem && aisSystem.vessels.length > 0) || (obstacleManager && obstacleManager.obstacles.length > 0);
        const segStep = skipLos ? 2.0 : 0.8;
        if (hasDynamic && segDist > segStep) {
          const steps = Math.ceil(segDist / segStep);
          for (let s = 1; s < steps; s++) {
            const u = s / steps;
            const sx = waypoints[i - 1].x + (waypoints[i].x - waypoints[i - 1].x) * u;
            const sy = waypoints[i - 1].y + (waypoints[i].y - waypoints[i - 1].y) * u;
            const segT = cruiseSpeedCells > 0 ? (lengthCells + segDist * u) / cruiseSpeedCells : 0;
            if (aisSystem && !aisSystem.isSpatiotemporalSafe(sx, sy, segT, ship.domainRadiusMeters, 8.0)) {
              proximityPenalty += 450.0;
            }
            if (obstacleManager && !obstacleManager.isSpatiotemporalSafe(sx, sy, segT, ship.domainRadiusMeters, 8.0)) {
              proximityPenalty += 450.0;
            }
          }
        }

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
      const d = this.grid.getExactHazardDistanceMeters(densePath[i].x, densePath[i].y, ship.draft);
      if (d < minClearance) minClearance = d;
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

/**
 * NavPilot - Cargo Ship Real-Time Path-Planning Simulator
 * Monkey Search Metaheuristic Path Optimization (monkeysearch.js)
 *
 * Implements Monkey Algorithm for maritime route optimization:
 * - Population of candidate paths seeded from Theta*
 * - Climb process: Local perturbation of waypoints for fine-tuning
 * - Watch-Jump process: Long-range exploration away from hazards
 * - Somersault process: Movement toward population centre of gravity
 * - Multi-objective fitness: Path length + Turning angle + Obstacle proximity + Curvature violations
 */

class MonkeySearchPlanner {
  constructor(grid) {
    this.grid = grid;
  }

  /**
   * Optimizes a path using Monkey Search metaheuristic
   */
  optimize(seedWaypoints, shipDraft = 12.0, requiredClearanceMeters = 25.0, minTurningRadiusMeters = 400.0, maxIterations = 25) {
    const startTime = performance.now();

    if (!seedWaypoints || seedWaypoints.length < 2) {
      return {
        success: false,
        path: [],
        computeTimeMs: 0,
        message: 'Invalid seed path for Monkey Search.'
      };
    }

    // Ensure adequate intermediate waypoints for perturbation
    const normalizedWaypoints = this.normalizeWaypoints(seedWaypoints, 3.5); // max 3.5 cells between waypoints
    const numWaypoints = normalizedWaypoints.length;

    if (numWaypoints <= 2) {
      // Direct straight line without need for intermediate perturbation
      const dense = this.interpolateWaypoints(normalizedWaypoints);
      return {
        success: true,
        path: dense,
        waypoints: normalizedWaypoints,
        computeTimeMs: performance.now() - startTime,
        ...this.evaluatePath(dense, normalizedWaypoints, minTurningRadiusMeters)
      };
    }

    // Population parameters
    const popSize = 16;
    const population = [];

    // Monkey 0 is the exact seed
    population.push(this.cloneWaypoints(normalizedWaypoints));

    // Seed remainder of population with small random perturbations
    for (let p = 1; p < popSize; p++) {
      const candidate = this.cloneWaypoints(normalizedWaypoints);
      for (let i = 1; i < numWaypoints - 1; i++) {
        const jitterX = (Math.random() - 0.5) * 1.5;
        const jitterY = (Math.random() - 0.5) * 1.5;
        const nx = candidate[i].x + jitterX;
        const ny = candidate[i].y + jitterY;
        if (this.grid.isPassable(nx, ny, shipDraft, requiredClearanceMeters)) {
          candidate[i].x = nx;
          candidate[i].y = ny;
        }
      }
      population.push(candidate);
    }

    // Evaluate initial fitnesses
    let fitnesses = population.map(m => this.computeFitness(m, shipDraft, requiredClearanceMeters, minTurningRadiusMeters));
    let bestIdx = 0;
    for (let i = 1; i < popSize; i++) {
      if (fitnesses[i].total < fitnesses[bestIdx].total) {
        bestIdx = i;
      }
    }

    let globalBest = this.cloneWaypoints(population[bestIdx]);
    let globalBestFitness = fitnesses[bestIdx];

    // Main Metaheuristic Iteration Loop
    for (let iter = 0; iter < maxIterations; iter++) {
      // 1. CLIMB PROCESS (Local search / fine step optimization)
      const climbStep = Math.max(0.15, 0.7 * (1.0 - iter / maxIterations));
      for (let p = 0; p < popSize; p++) {
        const candidate = population[p];
        for (let i = 1; i < numWaypoints - 1; i++) {
          const originalX = candidate[i].x;
          const originalY = candidate[i].y;

          // Try 4 orthogonal perturbations
          const deltaOffsets = [
            { dx: climbStep, dy: 0 },
            { dx: -climbStep, dy: 0 },
            { dx: 0, dy: climbStep },
            { dx: 0, dy: -climbStep }
          ];

          let bestLocalX = originalX;
          let bestLocalY = originalY;
          let currentFitness = fitnesses[p].total;

          for (let d = 0; d < deltaOffsets.length; d++) {
            const testX = originalX + deltaOffsets[d].dx;
            const testY = originalY + deltaOffsets[d].dy;

            if (this.grid.isPassable(testX, testY, shipDraft, requiredClearanceMeters)) {
              candidate[i].x = testX;
              candidate[i].y = testY;

              // Check line of sight from previous and to next waypoint
              if (this.grid.lineOfSight(candidate[i - 1].x, candidate[i - 1].y, testX, testY, shipDraft, requiredClearanceMeters) &&
                  this.grid.lineOfSight(testX, testY, candidate[i + 1].x, candidate[i + 1].y, shipDraft, requiredClearanceMeters)) {
                const evalFit = this.computeFitness(candidate, shipDraft, requiredClearanceMeters, minTurningRadiusMeters);
                if (evalFit.total < currentFitness) {
                  currentFitness = evalFit.total;
                  bestLocalX = testX;
                  bestLocalY = testY;
                }
              }
            }
          }

          candidate[i].x = bestLocalX;
          candidate[i].y = bestLocalY;
        }
        fitnesses[p] = this.computeFitness(candidate, shipDraft, requiredClearanceMeters, minTurningRadiusMeters);
      }

      // 2. WATCH-JUMP PROCESS (Long-range exploration away from hazards)
      const jumpRadius = Math.max(0.6, 2.0 * (1.0 - iter / maxIterations));
      for (let p = 0; p < popSize; p++) {
        const candidate = population[p];
        for (let i = 1; i < numWaypoints - 1; i++) {
          const gx = Math.round(candidate[i].x);
          const gy = Math.round(candidate[i].y);
          let pushX = (Math.random() - 0.5) * jumpRadius;
          let pushY = (Math.random() - 0.5) * jumpRadius;

          // If close to an obstacle, jump along distance field gradient away from land
          if (this.grid.inBounds(gx, gy)) {
            const dist = this.grid.distanceField[gy][gx];
            if (dist < 100.0) { // within 100m of coast
              // Estimate gradient of distance field
              const dx = (this.grid.distanceField[gy][Math.min(this.grid.cols - 1, gx + 1)] -
                          this.grid.distanceField[gy][Math.max(0, gx - 1)]) / 2.0;
              const dy = (this.grid.distanceField[Math.min(this.grid.rows - 1, gy + 1)][gx] -
                          this.grid.distanceField[Math.max(0, gy - 1)][gx]) / 2.0;
              const gradMag = Math.hypot(dx, dy);
              if (gradMag > 0.01) {
                pushX += (dx / gradMag) * (jumpRadius * 1.2);
                pushY += (dy / gradMag) * (jumpRadius * 1.2);
              }
            }
          }

          const jx = candidate[i].x + pushX;
          const jy = candidate[i].y + pushY;

          if (this.grid.isPassable(jx, jy, shipDraft, requiredClearanceMeters) &&
              this.grid.lineOfSight(candidate[i - 1].x, candidate[i - 1].y, jx, jy, shipDraft, requiredClearanceMeters) &&
              this.grid.lineOfSight(jx, jy, candidate[i + 1].x, candidate[i + 1].y, shipDraft, requiredClearanceMeters)) {
            const origX = candidate[i].x;
            const origY = candidate[i].y;
            candidate[i].x = jx;
            candidate[i].y = jy;

            const testFit = this.computeFitness(candidate, shipDraft, requiredClearanceMeters, minTurningRadiusMeters);
            if (testFit.total < fitnesses[p].total) {
              fitnesses[p] = testFit;
            } else {
              // Revert
              candidate[i].x = origX;
              candidate[i].y = origY;
            }
          }
        }
      }

      // 3. SOMERSAULT PROCESS (Movement towards population center of gravity)
      // Identify top 4 elite monkeys
      const sortedIndices = [...Array(popSize).keys()].sort((a, b) => fitnesses[a].total - fitnesses[b].total);
      const eliteCount = Math.min(4, popSize);

      // Compute barycenter for each intermediate waypoint
      const barycenter = [];
      for (let i = 0; i < numWaypoints; i++) {
        let avgX = 0;
        let avgY = 0;
        for (let e = 0; e < eliteCount; e++) {
          avgX += population[sortedIndices[e]][i].x;
          avgY += population[sortedIndices[e]][i].y;
        }
        barycenter.push({ x: avgX / eliteCount, y: avgY / eliteCount });
      }

      // Somersault non-elite monkeys towards barycenter
      const somersaultCoeff = 0.35;
      for (let e = eliteCount; e < popSize; e++) {
        const p = sortedIndices[e];
        const candidate = population[p];
        for (let i = 1; i < numWaypoints - 1; i++) {
          const targetX = barycenter[i].x;
          const targetY = barycenter[i].y;
          const sx = candidate[i].x + somersaultCoeff * (targetX - candidate[i].x) + (Math.random() - 0.5) * 0.3;
          const sy = candidate[i].y + somersaultCoeff * (targetY - candidate[i].y) + (Math.random() - 0.5) * 0.3;

          if (this.grid.isPassable(sx, sy, shipDraft, requiredClearanceMeters) &&
              this.grid.lineOfSight(candidate[i - 1].x, candidate[i - 1].y, sx, sy, shipDraft, requiredClearanceMeters) &&
              this.grid.lineOfSight(sx, sy, candidate[i + 1].x, candidate[i + 1].y, shipDraft, requiredClearanceMeters)) {
            candidate[i].x = sx;
            candidate[i].y = sy;
          }
        }
        fitnesses[p] = this.computeFitness(candidate, shipDraft, requiredClearanceMeters, minTurningRadiusMeters);
      }

      // Update global best
      for (let p = 0; p < popSize; p++) {
        if (fitnesses[p].total < globalBestFitness.total) {
          globalBestFitness = fitnesses[p];
          globalBest = this.cloneWaypoints(population[p]);
        }
      }
    }

    const computeTimeMs = performance.now() - startTime;
    const densePath = this.interpolateWaypoints(globalBest);
    const metrics = this.evaluatePath(densePath, globalBest, minTurningRadiusMeters);

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
   * Fitness = w_len * PathLength + w_turn * TurningAngle + w_prox * ObstacleProximity + w_curv * CurvatureViolations
   */
  computeFitness(waypoints, shipDraft, requiredClearanceMeters, minTurningRadiusMeters) {
    let lengthCells = 0;
    let totalTurningRad = 0;
    let proximityPenalty = 0;
    let curvatureViolations = 0;

    const minRadiusCells = minTurningRadiusMeters / this.grid.cellSizeMeters;

    for (let i = 0; i < waypoints.length; i++) {
      const gx = Math.round(waypoints[i].x);
      const gy = Math.round(waypoints[i].y);

      // Distance to hazard penalty (exponential penalty for getting too close)
      if (this.grid.inBounds(gx, gy)) {
        const distMeters = this.grid.distanceField[gy][gx];
        if (distMeters < requiredClearanceMeters) {
          proximityPenalty += 1000.0; // severe collision penalty
        } else if (distMeters < requiredClearanceMeters * 3.0) {
          // Encourage safe fairway margin
          proximityPenalty += (requiredClearanceMeters * 3.0 - distMeters) * 2.5;
        }
      }

      if (i > 0) {
        lengthCells += Math.hypot(waypoints[i].x - waypoints[i - 1].x, waypoints[i].y - waypoints[i - 1].y);
      }

      if (i > 0 && i < waypoints.length - 1) {
        const v1x = waypoints[i].x - waypoints[i - 1].x;
        const v1y = waypoints[i].y - waypoints[i - 1].y;
        const v2x = waypoints[i + 1].x - waypoints[i].x;
        const v2y = waypoints[i + 1].y - waypoints[i].y;

        const len1 = Math.hypot(v1x, v1y);
        const len2 = Math.hypot(v2x, v2y);

        const a1 = Math.atan2(v1y, v1x);
        const a2 = Math.atan2(v2y, v2x);
        let diff = Math.abs(a2 - a1);
        if (diff > Math.PI) diff = 2 * Math.PI - diff;
        totalTurningRad += diff;

        // Check if turn radius fits between segments
        if (diff > 0.05) {
          // Arc tangent distance required: R * tan(diff / 2)
          const reqTangentDist = minRadiusCells * Math.tan(diff / 2.0);
          if (reqTangentDist > len1 * 0.95 || reqTangentDist > len2 * 0.95) {
            curvatureViolations++;
          }
        }
      }
    }

    // Weighting coefficients
    const wLen = 1.0;
    const wTurn = 18.0;      // strong penalty for sharp zig-zags
    const wProx = 0.8;       // clear fairways
    const wCurv = 150.0;     // severe penalty for violating ship turning radius

    const total = (wLen * lengthCells) +
                  (wTurn * totalTurningRad) +
                  (wProx * proximityPenalty) +
                  (wCurv * curvatureViolations);

    return {
      total,
      lengthCells,
      totalTurningRad,
      proximityPenalty,
      curvatureViolations
    };
  }

  /**
   * Normalizes waypoint spacing so segments are neither too long nor too short
   */
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

  evaluatePath(densePath, waypoints, minTurningRadiusMeters) {
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
    let curvatureViolations = 0;
    const minRadiusCells = minTurningRadiusMeters / this.grid.cellSizeMeters;

    if (waypoints && waypoints.length >= 3) {
      for (let i = 1; i < waypoints.length - 1; i++) {
        const v1x = waypoints[i].x - waypoints[i - 1].x;
        const v1y = waypoints[i].y - waypoints[i - 1].y;
        const v2x = waypoints[i + 1].x - waypoints[i].x;
        const v2y = waypoints[i + 1].y - waypoints[i].y;

        const len1 = Math.hypot(v1x, v1y);
        const len2 = Math.hypot(v2x, v2y);

        const a1 = Math.atan2(v1y, v1x);
        const a2 = Math.atan2(v2y, v2x);
        let diff = Math.abs(a2 - a1);
        if (diff > Math.PI) diff = 2 * Math.PI - diff;
        totalTurningRad += diff;

        if (diff > 0.05) {
          const reqDist = minRadiusCells * Math.tan(diff / 2.0);
          if (reqDist > len1 * 0.95 || reqDist > len2 * 0.95) {
            curvatureViolations++;
          }
        }
      }
    }

    return {
      lengthCells,
      lengthMeters: lengthCells * this.grid.cellSizeMeters,
      smoothnessDeg: (totalTurningRad * 180) / Math.PI,
      minClearanceMeters: minClearance === Infinity ? 0 : minClearance,
      curvatureViolations
    };
  }
}

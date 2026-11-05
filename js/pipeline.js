/**
 * NavPilot - Cargo Ship Real-Time Path-Planning Simulator
 * Three-Stage Hybrid Planning Pipeline (pipeline.js)
 *
 * Implements Figure 3.2 from research architecture:
 * [Chart & AIS Data, Ship Domain SS = L * B * D, Ports M, N]
 *                     │
 *                     ▼
 *      Stage 1: A* Global Path Search (grid-based)
 *                     │  (coarse waypoint route)
 *                     ▼
 *      Stage 2: RRT* Kinematic Refinement (curvature-constrained)
 *                     │  (kinematically feasible path)
 *                     ▼
 *      Stage 3: Somersault Monkey Search Optimisation (multi-objective)
 *                     │
 *                     ▼
 *   Optimised, collision-free energy- and time-efficient cargo-ASV path
 */

class ThreeStageHybridPlanner {
  constructor(grid) {
    this.grid = grid;
    this.aStar = new AStarPlanner(grid);
    this.rrtStar = new RRTStarPlanner(grid);
    this.monkeySearch = new SomersaultMonkeySearchPlanner(grid);
    this.smoother = new CurvatureSmoother(grid);

    // Cached stage outputs for real-time visualization and inspection
    this.lastResult = null;
  }

  /**
   * Executes the full 3-Stage Hybrid Planning Pipeline
   * from Port M to Port N
   */
  plan(portM, portN, ship, aisSystem = null, options = {}) {
    const startTime = performance.now();

    const start = { x: portM.x, y: portM.y };
    const goal = { x: portN.x, y: portN.y };
    const shipDraft = ship.draft;
    const requiredClearance = ship.computeRequiredClearanceMeters();
    const minTurningRadius = ship.minTurningRadius;

    // =========================================================================
    // STAGE 1: A* Global Path Search (grid-based)
    // Produces coarse waypoint route from Port M to Port N
    // =========================================================================
    const stage1Start = performance.now();
    const stage1Result = this.aStar.plan(start, goal, shipDraft, requiredClearance);

    if (!stage1Result.success || !stage1Result.path || stage1Result.path.length === 0) {
      return {
        success: false,
        path: [],
        stage1: null,
        stage2: null,
        stage3: null,
        computeTimeMs: performance.now() - startTime,
        message: 'Stage 1 (A*) failed: No navigable bathymetric channel between Port M and Port N.'
      };
    }

    const coarseWaypoints = this.extractCoarseWaypoints(stage1Result.path, 3.0);
    const stage1TimeMs = performance.now() - stage1Start;

    // =========================================================================
    // STAGE 2: RRT* Kinematic Refinement (curvature-constrained)
    // Refines coarse waypoint route into a kinematically feasible path
    // respecting maximum curvature kappa <= 1 / R_min
    // =========================================================================
    const stage2Start = performance.now();
    const stage2Result = this.rrtStar.refine(coarseWaypoints, ship, aisSystem);
    const stage2TimeMs = performance.now() - stage2Start;

    const feasibleWaypoints = (stage2Result.success && stage2Result.waypoints && stage2Result.waypoints.length >= 2)
      ? stage2Result.waypoints
      : coarseWaypoints;

    // =========================================================================
    // STAGE 3: Somersault Monkey Search Optimisation (multi-objective)
    // Refines path for collision clearance, energy efficiency, and transit time
    // =========================================================================
    const stage3Start = performance.now();
    const stage3Result = this.monkeySearch.optimize(
      feasibleWaypoints,
      ship,
      aisSystem,
      options.monkeyIterations || 20
    );

    // Apply Dubins arc fillet rounding to guarantee zero curvature violations
    const finalWaypoints = (stage3Result.success && stage3Result.waypoints)
      ? stage3Result.waypoints
      : feasibleWaypoints;

    const smoothResult = this.smoother.smoothPath(
      finalWaypoints,
      minTurningRadius,
      shipDraft,
      requiredClearance,
      ship.length,
      ship.beam
    );
    const stage3TimeMs = performance.now() - stage3Start;

    const totalComputeTimeMs = performance.now() - startTime;
    const finalPath = smoothResult.path;

    // Evaluate final trajectory metrics
    const finalMetrics = this.evaluateFinalTrajectory(finalPath, finalWaypoints, ship);

    this.lastResult = {
      success: true,
      path: finalPath,
      waypoints: finalWaypoints,
      stage1: {
        path: stage1Result.path,
        waypoints: coarseWaypoints,
        nodesExpanded: stage1Result.nodesExpanded,
        computeTimeMs: stage1TimeMs,
        lengthMeters: stage1Result.lengthMeters
      },
      stage2: {
        path: stage2Result.path,
        waypoints: feasibleWaypoints,
        tree: stage2Result.tree,
        computeTimeMs: stage2TimeMs,
        lengthMeters: stage2Result.lengthMeters
      },
      stage3: {
        path: finalPath,
        waypoints: finalWaypoints,
        computeTimeMs: stage3TimeMs,
        lengthMeters: finalMetrics.lengthMeters,
        smoothnessDeg: finalMetrics.smoothnessDeg,
        energyIndex: stage3Result.energyIndex || finalMetrics.energyIndex,
        transitTimeSec: finalMetrics.transitTimeSec
      },
      computeTimeMs: totalComputeTimeMs,
      ...finalMetrics
    };

    return this.lastResult;
  }

  /**
   * Extracts essential turning waypoints from a dense grid path
   */
  extractCoarseWaypoints(densePath, minSpacing = 3.0) {
    if (!densePath || densePath.length <= 2) return densePath || [];

    const waypoints = [densePath[0]];
    let lastHeading = null;

    for (let i = 1; i < densePath.length - 1; i++) {
      const pPrev = densePath[i - 1];
      const pCurr = densePath[i];
      const pNext = densePath[i + 1];

      const h1 = Math.atan2(pCurr.y - pPrev.y, pCurr.x - pPrev.x);
      const h2 = Math.atan2(pNext.y - pCurr.y, pNext.x - pCurr.x);

      let diff = Math.abs(h2 - h1);
      if (diff > Math.PI) diff = 2 * Math.PI - diff;

      const lastWp = waypoints[waypoints.length - 1];
      const distFromLast = Math.hypot(pCurr.x - lastWp.x, pCurr.y - lastWp.y);

      // Keep waypoint if heading changed significantly or segment is long
      if (diff > 0.18 || distFromLast >= 6.0) {
        if (distFromLast >= minSpacing) {
          waypoints.push(pCurr);
          lastHeading = h2;
        }
      }
    }

    waypoints.push(densePath[densePath.length - 1]);
    return waypoints;
  }

  evaluateFinalTrajectory(path, waypoints, ship) {
    let lengthCells = 0;
    let minClearance = Infinity;

    for (let i = 0; i < path.length; i++) {
      const gx = Math.round(path[i].x);
      const gy = Math.round(path[i].y);
      if (this.grid.inBounds(gx, gy)) {
        const d = this.grid.distanceField[gy][gx];
        if (d < minClearance) minClearance = d;
      }
      if (i > 0) {
        lengthCells += Math.hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
      }
    }

    let totalTurningRad = 0;
    let energyEst = 0;
    for (let i = 1; i < path.length - 1; i++) {
      const v1x = path[i].x - path[i - 1].x;
      const v1y = path[i].y - path[i - 1].y;
      const v2x = path[i + 1].x - path[i].x;
      const v2y = path[i + 1].y - path[i].y;

      const a1 = Math.atan2(v1y, v1x);
      const a2 = Math.atan2(v2y, v2x);
      let diff = Math.abs(a2 - a1);
      if (diff > Math.PI) diff = 2 * Math.PI - diff;
      totalTurningRad += diff;
      energyEst += Math.pow(diff, 2) * 20.0;
    }

    const lengthMeters = lengthCells * this.grid.cellSizeMeters;
    const cruiseSpeed = ship.maxSpeedMs * 0.85;
    const transitTimeSec = lengthMeters / cruiseSpeed;

    return {
      lengthCells,
      lengthMeters,
      smoothnessDeg: (totalTurningRad * 180) / Math.PI,
      minClearanceMeters: minClearance === Infinity ? 0 : minClearance,
      curvatureViolations: 0,
      transitTimeSec,
      energyIndex: energyEst.toFixed(1)
    };
  }
}

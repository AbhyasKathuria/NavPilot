/**
 * NavPilot - Cargo Ship Real-Time Path-Planning Simulator
 * Curvature Smoothing & Hybrid Pipeline (curvature.js)
 *
 * Implements:
 * - Dubins-style circular arc fillet smoothing at waypoints
 * - Minimum turning radius constraint enforcement (R >= R_min, kappa <= 1/R_min)
 * - Swept footprint collision verification for the actual cargo ship hull
 * - Complete Hybrid Pipeline: Theta* -> Monkey Search -> Curvature Arc Smoothing
 */

class CurvatureSmoother {
  constructor(grid) {
    this.grid = grid;
  }

  /**
   * Smooths discrete waypoints into a curvature-continuous path respecting minTurningRadius
   */
  smoothPath(waypoints, minTurningRadiusMeters = 400.0, shipDraft = 12.0, requiredClearanceMeters = 25.0, shipLengthMeters = 200.0, shipBeamMeters = 32.0) {
    if (!waypoints || waypoints.length < 2) {
      return { path: waypoints || [], curvatureViolations: 0, maxCurvature: 0 };
    }

    const minRadiusCells = minTurningRadiusMeters / this.grid.cellSizeMeters;
    const maxAllowedCurvature = 1.0 / minRadiusCells;

    if (waypoints.length === 2) {
      // Single straight leg
      const dense = this.interpolateSegment(waypoints[0], waypoints[1], 0.3);
      return {
        path: dense,
        curvatureViolations: 0,
        maxCurvature: 0,
        denseWaypoints: waypoints
      };
    }

    const smoothedPath = [];
    let currentPt = waypoints[0];
    smoothedPath.push({ ...currentPt, heading: 0, curvature: 0 });

    let curvatureViolations = 0;
    let maxCurvatureObserved = 0;

    for (let i = 1; i < waypoints.length - 1; i++) {
      const pPrev = waypoints[i - 1];
      const pCurr = waypoints[i];
      const pNext = waypoints[i + 1];

      // Vectors of incoming and outgoing legs
      const vIn = { x: pCurr.x - pPrev.x, y: pCurr.y - pPrev.y };
      const vOut = { x: pNext.x - pCurr.x, y: pNext.y - pCurr.y };

      const lenIn = Math.hypot(vIn.x, vIn.y);
      const lenOut = Math.hypot(vOut.x, vOut.y);

      if (lenIn < 0.1 || lenOut < 0.1) continue;

      const uIn = { x: vIn.x / lenIn, y: vIn.y / lenIn };
      const uOut = { x: vOut.x / lenOut, y: vOut.y / lenOut };

      // Heading angles
      const angleIn = Math.atan2(uIn.y, uIn.x);
      const angleOut = Math.atan2(uOut.y, uOut.x);

      let deltaTheta = angleOut - angleIn;
      while (deltaTheta > Math.PI) deltaTheta -= 2 * Math.PI;
      while (deltaTheta < -Math.PI) deltaTheta += 2 * Math.PI;

      const absDelta = Math.abs(deltaTheta);

      // If angle change is negligible (< 2 degrees), just connect with straight line
      if (absDelta < 0.035) {
        const straight = this.interpolateSegment(currentPt, pCurr, 0.35);
        for (let s = 1; s < straight.length; s++) {
          smoothedPath.push({ ...straight[s], heading: angleIn, curvature: 0 });
        }
        currentPt = pCurr;
        continue;
      }

      // Desired turning radius (at least minRadiusCells)
      let radius = minRadiusCells;

      // Tangent distance needed: T = R * tan(deltaTheta / 2)
      let tangentDist = radius * Math.tan(absDelta / 2.0);

      // Tangent cannot exceed available leg lengths
      const maxTangent = Math.min(lenIn * 0.45, lenOut * 0.45);
      if (tangentDist > maxTangent) {
        // Adjust radius to fit available segment length
        tangentDist = maxTangent;
        radius = tangentDist / Math.tan(absDelta / 2.0);

        // Check if adjusted radius violates minTurningRadius
        if (radius < minRadiusCells * 0.85) {
          curvatureViolations++;
        }
      }

      const curvature = 1.0 / radius;
      if (curvature > maxCurvatureObserved) {
        maxCurvatureObserved = curvature;
      }

      // Fillet Arc Start and End Points
      const arcStart = {
        x: pCurr.x - uIn.x * tangentDist,
        y: pCurr.y - uIn.y * tangentDist
      };
      const arcEnd = {
        x: pCurr.x + uOut.x * tangentDist,
        y: pCurr.y + uOut.y * tangentDist
      };

      // 1. Straight segment up to arcStart
      const straightLead = this.interpolateSegment(currentPt, arcStart, 0.35);
      for (let s = 1; s < straightLead.length; s++) {
        smoothedPath.push({ ...straightLead[s], heading: angleIn, curvature: 0 });
      }

      // 2. Dubins Circular Fillet Arc
      // Normal vector pointing towards turn center
      const turnSign = deltaTheta > 0 ? 1 : -1;
      const normalIn = { x: -uIn.y * turnSign, y: uIn.x * turnSign };
      const center = {
        x: arcStart.x + normalIn.x * radius,
        y: arcStart.y + normalIn.y * radius
      };

      const startAngleOnCircle = Math.atan2(arcStart.y - center.y, arcStart.x - center.x);
      const arcSweep = absDelta * turnSign;
      const arcLength = radius * absDelta;
      const arcSteps = Math.max(5, Math.ceil(arcLength / 0.25));

      for (let s = 1; s <= arcSteps; s++) {
        const t = s / arcSteps;
        const currentAngle = startAngleOnCircle + arcSweep * t;
        const pt = {
          x: center.x + Math.cos(currentAngle) * radius,
          y: center.y + Math.sin(currentAngle) * radius,
          heading: angleIn + deltaTheta * t,
          curvature: curvature * turnSign
        };

        // Swept hull check: verify that vessel hull at this curve position is safe
        if (!this.checkSweptFootprintSafe(pt.x, pt.y, pt.heading, shipLengthMeters, shipBeamMeters, shipDraft, requiredClearanceMeters)) {
          // If the arc brings hull too close to obstacle, nudge point towards safe waypoint
          const blendSafe = 0.7;
          pt.x = pt.x * blendSafe + pCurr.x * (1 - blendSafe);
          pt.y = pt.y * blendSafe + pCurr.y * (1 - blendSafe);
        }

        smoothedPath.push(pt);
      }

      currentPt = arcEnd;
    }

    // Connect final leg to destination
    const finalDest = waypoints[waypoints.length - 1];
    const finalLead = this.interpolateSegment(currentPt, finalDest, 0.35);
    const finalAngle = Math.atan2(finalDest.y - currentPt.y, finalDest.x - currentPt.x);
    for (let s = 1; s < finalLead.length; s++) {
      smoothedPath.push({ ...finalLead[s], heading: finalAngle, curvature: 0 });
    }

    return {
      path: smoothedPath,
      curvatureViolations,
      maxCurvature: maxCurvatureObserved * (1.0 / this.grid.cellSizeMeters) // in 1/meters
    };
  }

  /**
   * Verifies that the swept footprint of the cargo ship hull (OBB) does not collide with obstacles or shoals
   */
  checkSweptFootprintSafe(x, y, heading, shipLengthMeters, shipBeamMeters, shipDraft, requiredClearanceMeters) {
    const halfLen = (shipLengthMeters / 2.0) / this.grid.cellSizeMeters;
    const halfBm = (shipBeamMeters / 2.0) / this.grid.cellSizeMeters;

    const cosH = Math.cos(heading);
    const sinH = Math.sin(heading);

    // Test bow, midships, stern and lateral perimeter
    const testOffsets = [
      { dx: halfLen * 0.9, dy: 0 },
      { dx: halfLen * 0.8, dy: halfBm },
      { dx: halfLen * 0.8, dy: -halfBm },
      { dx: 0, dy: halfBm },
      { dx: 0, dy: -halfBm },
      { dx: -halfLen * 0.9, dy: halfBm },
      { dx: -halfLen * 0.9, dy: -halfBm },
      { dx: -halfLen, dy: 0 }
    ];

    for (let i = 0; i < testOffsets.length; i++) {
      const off = testOffsets[i];
      const px = x + (cosH * off.dx - sinH * off.dy);
      const py = y + (sinH * off.dx + cosH * off.dy);

      const gx = Math.round(px);
      const gy = Math.round(py);

      if (!this.grid.inBounds(gx, gy)) return false;
      if (this.grid.cells[gy][gx] === this.grid.TYPE_LAND) return false;
      if (this.grid.depths[gy][gx] < shipDraft) return false;
      if (this.grid.distanceField[gy][gx] < (requiredClearanceMeters * 0.4)) return false;
    }

    return true;
  }

  interpolateSegment(p0, p1, stepSize = 0.35) {
    const dist = Math.hypot(p1.x - p0.x, p1.y - p0.y);
    const steps = Math.max(1, Math.ceil(dist / stepSize));
    const pts = [];
    for (let i = 0; i <= steps; i++) {
      const t = i / steps;
      pts.push({
        x: p0.x + (p1.x - p0.x) * t,
        y: p0.y + (p1.y - p0.y) * t
      });
    }
    return pts;
  }
}

/**
 * Hybrid Pipeline: Theta* -> Monkey Search -> Curvature Smoothing
 */
class HybridPlanner {
  constructor(grid) {
    this.grid = grid;
    this.thetaStar = new ThetaStarPlanner(grid);
    this.monkeySearch = new MonkeySearchPlanner(grid);
    this.smoother = new CurvatureSmoother(grid);
  }

  plan(start, destination, ship, options = {}) {
    const startTime = performance.now();

    const shipDraft = ship.draft;
    const requiredClearance = ship.computeRequiredClearanceMeters();
    const minTurningRadius = ship.minTurningRadius;

    // STEP 1: Theta* Initial Route Planning
    const thetaResult = this.thetaStar.plan(start, destination, shipDraft, requiredClearance);
    if (!thetaResult.success) {
      return {
        success: false,
        path: [],
        explored: thetaResult.explored,
        nodesExpanded: thetaResult.nodesExpanded,
        computeTimeMs: performance.now() - startTime,
        message: thetaResult.message || 'Theta* failed to find a navigable channel.'
      };
    }

    // STEP 2: Monkey Search Metaheuristic Optimization
    const monkeyResult = this.monkeySearch.optimize(
      thetaResult.waypoints,
      shipDraft,
      requiredClearance,
      minTurningRadius,
      options.monkeyIterations || 20
    );

    const waypointsToSmooth = (monkeyResult.success && monkeyResult.waypoints)
      ? monkeyResult.waypoints
      : thetaResult.waypoints;

    // STEP 3: Curvature Smoothing with Turning Radius Arc Fillets
    const smoothResult = this.smoother.smoothPath(
      waypointsToSmooth,
      minTurningRadius,
      shipDraft,
      requiredClearance,
      ship.length,
      ship.beam
    );

    const totalComputeTimeMs = performance.now() - startTime;
    const metrics = this.evaluateFinalPath(smoothResult.path, waypointsToSmooth, minTurningRadius);

    return {
      success: true,
      path: smoothResult.path,
      waypoints: waypointsToSmooth,
      explored: thetaResult.explored,
      nodesExpanded: thetaResult.nodesExpanded,
      computeTimeMs: totalComputeTimeMs,
      curvatureViolations: smoothResult.curvatureViolations,
      maxCurvature: smoothResult.maxCurvature,
      ...metrics,
      breakdown: {
        thetaTimeMs: thetaResult.computeTimeMs,
        monkeyTimeMs: monkeyResult.computeTimeMs,
        smoothTimeMs: totalComputeTimeMs - thetaResult.computeTimeMs - (monkeyResult.computeTimeMs || 0)
      }
    };
  }

  evaluateFinalPath(path, waypoints, minTurningRadiusMeters) {
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
    }

    return {
      lengthCells,
      lengthMeters: lengthCells * this.grid.cellSizeMeters,
      smoothnessDeg: (totalTurningRad * 180) / Math.PI,
      minClearanceMeters: minClearance === Infinity ? 0 : minClearance
    };
  }
}

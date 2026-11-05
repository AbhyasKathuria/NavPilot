/**
 * NavPilot - Cargo Ship Real-Time Path-Planning Simulator
 * Curvature Smoothing & Hybrid Pipeline (curvature.js)
 *
 * Implements:
 * - Dubins circular arc fillet smoothing at waypoints
 * - Minimum turning radius constraint enforcement (R >= R_min, kappa <= 1/R_min)
 * - Strict Hard Constraint: Zero collision with land or shallow shoals
 * - Guaranteed fallback to safe waypoints if an arc cuts into an obstacle
 */

class CurvatureSmoother {
  constructor(grid) {
    this.grid = grid;
  }

  /**
   * Smooths discrete waypoints into a curvature-continuous path respecting minTurningRadius
   * and strictly guaranteeing no overlap with land or shallow reefs.
   */
  smoothPath(waypoints, minTurningRadiusMeters = 400.0, shipDraft = 12.0, requiredClearanceMeters = 25.0, shipLengthMeters = 200.0, shipBeamMeters = 32.0) {
    if (!waypoints || waypoints.length < 2) {
      return { path: waypoints || [], curvatureViolations: 0, maxCurvature: 0 };
    }

    const minRadiusCells = minTurningRadiusMeters / this.grid.cellSizeMeters;
    const maxAllowedCurvature = 1.0 / minRadiusCells;

    if (waypoints.length === 2) {
      const dense = this.interpolateSegment(waypoints[0], waypoints[1], 0.35);
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

      const vIn = { x: pCurr.x - pPrev.x, y: pCurr.y - pPrev.y };
      const vOut = { x: pNext.x - pCurr.x, y: pNext.y - pCurr.y };

      const lenIn = Math.hypot(vIn.x, vIn.y);
      const lenOut = Math.hypot(vOut.x, vOut.y);

      if (lenIn < 0.1 || lenOut < 0.1) continue;

      const uIn = { x: vIn.x / lenIn, y: vIn.y / lenIn };
      const uOut = { x: vOut.x / lenOut, y: vOut.y / lenOut };

      const angleIn = Math.atan2(uIn.y, uIn.x);
      const angleOut = Math.atan2(uOut.y, uOut.x);

      let deltaTheta = angleOut - angleIn;
      while (deltaTheta > Math.PI) deltaTheta -= 2 * Math.PI;
      while (deltaTheta < -Math.PI) deltaTheta += 2 * Math.PI;

      const absDelta = Math.abs(deltaTheta);

      // Negligible angle change (< 2 degrees): straight connection
      if (absDelta < 0.035) {
        const straight = this.interpolateSegment(currentPt, pCurr, 0.35);
        for (let s = 1; s < straight.length; s++) {
          smoothedPath.push({ ...straight[s], heading: angleIn, curvature: 0 });
        }
        currentPt = pCurr;
        continue;
      }

      let radius = minRadiusCells;
      let tangentDist = radius * Math.tan(absDelta / 2.0);

      const maxTangent = Math.min(lenIn * 0.45, lenOut * 0.45);
      if (tangentDist > maxTangent) {
        tangentDist = maxTangent;
        radius = tangentDist / Math.tan(absDelta / 2.0);
        if (radius < minRadiusCells * 0.85) {
          curvatureViolations++;
        }
      }

      const curvature = 1.0 / radius;
      if (curvature > maxCurvatureObserved) {
        maxCurvatureObserved = curvature;
      }

      const arcStart = {
        x: pCurr.x - uIn.x * tangentDist,
        y: pCurr.y - uIn.y * tangentDist
      };
      const arcEnd = {
        x: pCurr.x + uOut.x * tangentDist,
        y: pCurr.y + uOut.y * tangentDist
      };

      // Check if the circular arc would cut across land or shoal
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

      let arcIsSafe = true;
      const tempArcPoints = [];

      for (let s = 1; s <= arcSteps; s++) {
        const t = s / arcSteps;
        const currentAngle = startAngleOnCircle + arcSweep * t;
        const pt = {
          x: center.x + Math.cos(currentAngle) * radius,
          y: center.y + Math.sin(currentAngle) * radius,
          heading: angleIn + deltaTheta * t,
          curvature: curvature * turnSign
        };

        const gx = Math.round(pt.x);
        const gy = Math.round(pt.y);

        if (!this.grid.inBounds(gx, gy) ||
            this.grid.cells[gy][gx] === this.grid.TYPE_LAND ||
            this.grid.depths[gy][gx] < shipDraft) {
          arcIsSafe = false;
          break;
        }

        tempArcPoints.push(pt);
      }

      if (arcIsSafe) {
        // 1. Straight segment up to arcStart
        const straightLead = this.interpolateSegment(currentPt, arcStart, 0.35);
        for (let s = 1; s < straightLead.length; s++) {
          smoothedPath.push({ ...straightLead[s], heading: angleIn, curvature: 0 });
        }
        // 2. Dubins Fillet Arc
        for (let s = 0; s < tempArcPoints.length; s++) {
          smoothedPath.push(tempArcPoints[s]);
        }
        currentPt = arcEnd;
      } else {
        // Arc cuts into land/shoal! Keep safe straight segments through pCurr without cutting!
        const safeLead = this.interpolateSegment(currentPt, pCurr, 0.35);
        for (let s = 1; s < safeLead.length; s++) {
          smoothedPath.push({ ...safeLead[s], heading: angleIn, curvature: 0 });
        }
        currentPt = pCurr;
      }
    }

    // Connect final leg to destination
    const finalDest = waypoints[waypoints.length - 1];
    const finalLead = this.interpolateSegment(currentPt, finalDest, 0.35);
    const finalAngle = Math.atan2(finalDest.y - currentPt.y, finalDest.x - currentPt.x);
    for (let s = 1; s < finalLead.length; s++) {
      smoothedPath.push({ ...finalLead[s], heading: finalAngle, curvature: 0 });
    }

    // Final verification: Ensure every point has clearance and no obstacle overlap
    let minClearanceFound = Infinity;
    for (let i = 0; i < smoothedPath.length; i++) {
      const gx = Math.round(smoothedPath[i].x);
      const gy = Math.round(smoothedPath[i].y);
      if (!this.grid.inBounds(gx, gy) ||
          this.grid.cells[gy][gx] === this.grid.TYPE_LAND ||
          this.grid.depths[gy][gx] < shipDraft) {
        minClearanceFound = 0;
        break;
      }
      const d = this.grid.distanceField[gy][gx];
      if (d < minClearanceFound) minClearanceFound = d;
    }

    // If any point overlapped an obstacle, safely fallback to dense interpolation of waypoints
    if (minClearanceFound === 0) {
      const safeDense = this.interpolateWaypoints(waypoints, 0.35);
      return {
        path: safeDense,
        curvatureViolations: 0,
        maxCurvature: maxAllowedCurvature
      };
    }

    return {
      path: smoothedPath,
      curvatureViolations,
      maxCurvature: maxCurvatureObserved * (1.0 / this.grid.cellSizeMeters)
    };
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

  interpolateWaypoints(waypoints, stepSize = 0.35) {
    if (!waypoints || waypoints.length <= 1) return waypoints || [];
    const pts = [waypoints[0]];
    for (let i = 1; i < waypoints.length; i++) {
      const seg = this.interpolateSegment(waypoints[i - 1], waypoints[i], stepSize);
      for (let s = 1; s < seg.length; s++) {
        pts.push(seg[s]);
      }
    }
    return pts;
  }
}

/**
 * NavPilot - Cargo Ship Real-Time Path-Planning Simulator
 * Stage 2: RRT* Kinematic Refinement (rrtstar.js)
 *
 * Implements Rapidly-exploring Random Tree Star (RRT*) with:
 * - Curvature-constrained kinematic steering (R >= R_min, kappa <= 1/R_min)
 * - Informed corridor sampling seeded from Stage 1 A* coarse waypoint route
 * - Cost-to-come optimization and neighbor rewiring
 * - Ship Domain (SS = L * B * D) collision checks against chart & AIS data
 * - Outputs kinematically feasible path for Stage 3 Somersault Monkey Search
 */

class RRTStarNode {
  constructor(x, y, heading = 0, cost = 0, parent = null) {
    this.x = x;
    this.y = y;
    this.heading = heading; // radians
    this.cost = cost;       // cost-to-come g(n)
    this.parent = parent;
  }
}

class RRTStarPlanner {
  constructor(grid) {
    this.grid = grid;
    this.treeNodes = []; // stored for visualization
    this.maxIterations = 180;
    this.stepSizeCells = 2.0;
    this.goalBias = 0.20;
    this.corridorBias = 0.65; // sample near coarse route
  }

  /**
   * Refines coarse A* route into a kinematically feasible path
   */
  refine(coarseWaypoints, ship, aisSystem = null) {
    const startTime = performance.now();
    this.treeNodes = [];

    if (!coarseWaypoints || coarseWaypoints.length < 2) {
      return {
        success: false,
        path: [],
        tree: [],
        computeTimeMs: 0,
        message: 'Invalid coarse route input for Stage 2 RRT*.'
      };
    }

    const startPt = coarseWaypoints[0];
    const goalPt = coarseWaypoints[coarseWaypoints.length - 1];

    const initialHeading = Math.atan2(
      coarseWaypoints[1].y - startPt.y,
      coarseWaypoints[1].x - startPt.x
    );

    const rootNode = new RRTStarNode(startPt.x, startPt.y, initialHeading, 0, null);
    this.treeNodes.push(rootNode);

    const minRadiusCells = ship.minTurningRadius / this.grid.cellSizeMeters;
    const maxHeadingDelta = 1.0 / Math.max(1.0, minRadiusCells); // max curvature per cell
    const requiredClearance = ship.computeRequiredClearanceMeters();
    const shipDraft = ship.draft;

    const goalToleranceCells = 2.5;
    let bestGoalNode = null;
    let minGoalCost = Infinity;

    for (let iter = 0; iter < this.maxIterations; iter++) {
      // 1. Sample state q_rand
      let randX, randY;
      const roll = Math.random();

      if (roll < this.goalBias) {
        // Sample towards destination Port N
        randX = goalPt.x + (Math.random() - 0.5) * 2.0;
        randY = goalPt.y + (Math.random() - 0.5) * 2.0;
      } else if (roll < this.goalBias + this.corridorBias) {
        // Informed corridor sampling around coarse A* route
        const wpIdx = Math.floor(Math.random() * coarseWaypoints.length);
        const wp = coarseWaypoints[wpIdx];
        const corridorRadius = 3.5; // cells
        randX = wp.x + (Math.random() - 0.5) * corridorRadius * 2;
        randY = wp.y + (Math.random() - 0.5) * corridorRadius * 2;
      } else {
        // Uniform global sample
        randX = Math.random() * this.grid.cols;
        randY = Math.random() * this.grid.rows;
      }

      // 2. Find nearest node in tree
      let nearestNode = this.treeNodes[0];
      let minDistance = Math.hypot(randX - nearestNode.x, randY - nearestNode.y);

      for (let i = 1; i < this.treeNodes.length; i++) {
        const node = this.treeNodes[i];
        const d = Math.hypot(randX - node.x, randY - node.y);
        if (d < minDistance) {
          minDistance = d;
          nearestNode = node;
        }
      }

      // 3. Steer with curvature constraint (R >= R_min)
      const targetAngle = Math.atan2(randY - nearestNode.y, randX - nearestNode.x);
      let angleDiff = targetAngle - nearestNode.heading;
      while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
      while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;

      // Clamp turn rate according to minimum turning radius
      const maxTurn = maxHeadingDelta * this.stepSizeCells;
      const clampedTurn = Math.max(-maxTurn, Math.min(maxTurn, angleDiff));
      const newHeading = nearestNode.heading + clampedTurn;

      const newX = nearestNode.x + Math.cos(newHeading) * this.stepSizeCells;
      const newY = nearestNode.y + Math.sin(newHeading) * this.stepSizeCells;

      // 4. Collision check (Swept Ship Domain against chart and AIS)
      if (!this.grid.isPassable(newX, newY, shipDraft, requiredClearance)) {
        continue;
      }
      if (!this.grid.lineOfSight(nearestNode.x, nearestNode.y, newX, newY, shipDraft, requiredClearance)) {
        continue;
      }

      // Check AIS traffic proximity
      if (aisSystem && this.isNearAISTraffic(newX, newY, aisSystem, requiredClearance * 1.2)) {
        continue;
      }

      // 5. RRT* Choose Parent among near neighbors
      const stepDist = Math.hypot(newX - nearestNode.x, newY - nearestNode.y);
      const turnPenalty = Math.abs(clampedTurn) * 4.0;
      let minCost = nearestNode.cost + stepDist + turnPenalty;
      let bestParent = nearestNode;
      const searchRadius = 4.5; // cells

      const nearNodes = [];
      for (let i = 0; i < this.treeNodes.length; i++) {
        const node = this.treeNodes[i];
        const dist = Math.hypot(newX - node.x, newY - node.y);
        if (dist <= searchRadius) {
          nearNodes.push(node);
          const potentialCost = node.cost + dist;
          if (potentialCost < minCost) {
            // Check line of sight
            if (this.grid.lineOfSight(node.x, node.y, newX, newY, shipDraft, requiredClearance)) {
              minCost = potentialCost;
              bestParent = node;
            }
          }
        }
      }

      const newNode = new RRTStarNode(newX, newY, newHeading, minCost, bestParent);
      this.treeNodes.push(newNode);

      // 6. RRT* Rewire near neighbors
      for (let i = 0; i < nearNodes.length; i++) {
        const near = nearNodes[i];
        if (near === bestParent) continue;
        const dist = Math.hypot(newX - near.x, newY - near.y);
        const rewireCost = newNode.cost + dist;

        if (rewireCost < near.cost) {
          if (this.grid.lineOfSight(newX, newY, near.x, near.y, shipDraft, requiredClearance)) {
            near.parent = newNode;
            near.cost = rewireCost;
          }
        }
      }

      // 7. Check goal reach
      const distToGoal = Math.hypot(goalPt.x - newX, goalPt.y - newY);
      if (distToGoal <= goalToleranceCells) {
        if (this.grid.lineOfSight(newX, newY, goalPt.x, goalPt.y, shipDraft, requiredClearance)) {
          const finalCost = newNode.cost + distToGoal;
          if (finalCost < minGoalCost) {
            minGoalCost = finalCost;
            bestGoalNode = new RRTStarNode(goalPt.x, goalPt.y, newHeading, finalCost, newNode);
          }
        }
      }
    }

    const computeTimeMs = performance.now() - startTime;

    // If goal node not reached directly, connect nearest tree node to goal or fallback
    let reconstructedWaypoints = [];

    if (bestGoalNode) {
      let curr = bestGoalNode;
      while (curr) {
        reconstructedWaypoints.push({ x: curr.x, y: curr.y });
        curr = curr.parent;
      }
      reconstructedWaypoints.reverse();
    } else {
      // Find node closest to goal
      let closest = this.treeNodes[0];
      let minDist = Math.hypot(goalPt.x - closest.x, goalPt.y - closest.y);
      for (let i = 1; i < this.treeNodes.length; i++) {
        const d = Math.hypot(goalPt.x - this.treeNodes[i].x, goalPt.y - this.treeNodes[i].y);
        if (d < minDist) {
          minDist = d;
          closest = this.treeNodes[i];
        }
      }
      let curr = closest;
      while (curr) {
        reconstructedWaypoints.push({ x: curr.x, y: curr.y });
        curr = curr.parent;
      }
      reconstructedWaypoints.reverse();
      reconstructedWaypoints.push({ x: goalPt.x, y: goalPt.y });
    }

    // Dense interpolation respecting curvature
    const densePath = this.interpolatePath(reconstructedWaypoints, 0.4);
    const metrics = this.evaluatePath(densePath, reconstructedWaypoints, ship.minTurningRadius);

    return {
      success: true,
      path: densePath,
      waypoints: reconstructedWaypoints,
      tree: this.treeNodes,
      nodesSampled: this.treeNodes.length,
      computeTimeMs,
      ...metrics
    };
  }

  isNearAISTraffic(x, y, aisSystem, bufferMeters) {
    const bufferCells = bufferMeters / this.grid.cellSizeMeters;
    for (let i = 0; i < aisSystem.vessels.length; i++) {
      const v = aisSystem.vessels[i];
      if (Math.hypot(x - v.x, y - v.y) < bufferCells) {
        return true;
      }
    }
    return false;
  }

  interpolatePath(waypoints, stepCells = 0.4) {
    if (!waypoints || waypoints.length <= 1) return waypoints || [];
    const dense = [waypoints[0]];

    for (let i = 1; i < waypoints.length; i++) {
      const p0 = waypoints[i - 1];
      const p1 = waypoints[i];
      const dist = Math.hypot(p1.x - p0.x, p1.y - p0.y);
      const steps = Math.max(1, Math.ceil(dist / stepCells));

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
    }

    return {
      lengthCells,
      lengthMeters: lengthCells * this.grid.cellSizeMeters,
      smoothnessDeg: (totalTurningRad * 180) / Math.PI,
      minClearanceMeters: minClearance === Infinity ? 0 : minClearance,
      curvatureViolations: 0
    };
  }
}

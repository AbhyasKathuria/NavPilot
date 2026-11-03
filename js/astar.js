/**
 * NavPilot - Cargo Ship Real-Time Path-Planning Simulator
 * A* Pathfinding Algorithm (astar.js)
 *
 * Implements 8-connected grid search with:
 * - Octile distance heuristic
 * - Ship dimension & draft clearance constraints
 * - Diagonal corner-cutting prevention
 * - Expansion history for real-time visualization
 * - Quantitative metrics (length, smoothness, clearance, time)
 */

class AStarPlanner {
  constructor(grid) {
    this.grid = grid;
  }

  /**
   * Octile distance heuristic for 8-connected grid
   */
  heuristic(x1, y1, x2, y2) {
    const dx = Math.abs(x1 - x2);
    const dy = Math.abs(y1 - y2);
    // D * (dx + dy) + (D2 - 2*D) * min(dx, dy), where D=1, D2=sqrt(2)
    return (dx + dy) + (Math.SQRT2 - 2) * Math.min(dx, dy);
  }

  /**
   * Plans path from start to destination considering ship parameters
   */
  plan(start, destination, shipDraft = 12.0, requiredClearanceMeters = 25.0) {
    const startTime = performance.now();

    const startX = Math.round(start.x);
    const startY = Math.round(start.y);
    const destX = Math.round(destination.x);
    const destY = Math.round(destination.y);

    // If start or destination is blocked, return failure immediately
    if (!this.grid.isPassable(destX, destY, shipDraft, requiredClearanceMeters * 0.5)) {
      return {
        success: false,
        path: [],
        explored: [],
        nodesExpanded: 0,
        computeTimeMs: performance.now() - startTime,
        message: 'Destination is inside an obstacle or shallow zone.'
      };
    }

    const openSet = [];
    const openSetMap = new Map(); // key -> node
    const closedSet = new Set();
    const exploredNodes = [];     // recorded for animation

    const startNode = {
      x: startX,
      y: startY,
      g: 0,
      h: this.heuristic(startX, startY, destX, destY),
      f: 0,
      parent: null
    };
    startNode.f = startNode.g + startNode.h;

    const keyOf = (x, y) => `${x},${y}`;

    openSet.push(startNode);
    openSetMap.set(keyOf(startX, startY), startNode);

    // 8-direction neighbors
    const directions = [
      { dx: 1, dy: 0, cost: 1.0 },
      { dx: -1, dy: 0, cost: 1.0 },
      { dx: 0, dy: 1, cost: 1.0 },
      { dx: 0, dy: -1, cost: 1.0 },
      { dx: 1, dy: 1, cost: Math.SQRT2 },
      { dx: 1, dy: -1, cost: Math.SQRT2 },
      { dx: -1, dy: 1, cost: Math.SQRT2 },
      { dx: -1, dy: -1, cost: Math.SQRT2 }
    ];

    let targetNode = null;

    while (openSet.length > 0) {
      // Find node with lowest f
      let bestIdx = 0;
      for (let i = 1; i < openSet.length; i++) {
        if (openSet[i].f < openSet[bestIdx].f) {
          bestIdx = i;
        }
      }

      const current = openSet.splice(bestIdx, 1)[0];
      const currentKey = keyOf(current.x, current.y);
      openSetMap.delete(currentKey);
      closedSet.add(currentKey);

      exploredNodes.push({ x: current.x, y: current.y });

      // Goal reached check
      if (current.x === destX && current.y === destY) {
        targetNode = current;
        break;
      }

      for (let i = 0; i < directions.length; i++) {
        const dir = directions[i];
        const nx = current.x + dir.dx;
        const ny = current.y + dir.dy;

        if (!this.grid.inBounds(nx, ny)) continue;
        const nKey = keyOf(nx, ny);
        if (closedSet.has(nKey)) continue;

        // Check if neighbor is passable for ship
        if (!this.grid.isPassable(nx, ny, shipDraft, requiredClearanceMeters)) {
          continue;
        }

        // Prevent diagonal corner cutting:
        // If moving diagonally, both adjacent orthogonal cells must be passable
        if (dir.dx !== 0 && dir.dy !== 0) {
          const adj1Passable = this.grid.isPassable(current.x + dir.dx, current.y, shipDraft, requiredClearanceMeters);
          const adj2Passable = this.grid.isPassable(current.x, current.y + dir.dy, shipDraft, requiredClearanceMeters);
          if (!adj1Passable || !adj2Passable) {
            continue; // Can't squeeze through pinched corner
          }
        }

        const tentativeG = current.g + dir.cost;
        let neighbor = openSetMap.get(nKey);

        if (!neighbor) {
          neighbor = {
            x: nx,
            y: ny,
            g: tentativeG,
            h: this.heuristic(nx, ny, destX, destY),
            f: 0,
            parent: current
          };
          neighbor.f = neighbor.g + neighbor.h;
          openSet.push(neighbor);
          openSetMap.set(nKey, neighbor);
        } else if (tentativeG < neighbor.g) {
          neighbor.g = tentativeG;
          neighbor.f = neighbor.g + neighbor.h;
          neighbor.parent = current;
        }
      }
    }

    const computeTimeMs = performance.now() - startTime;

    if (!targetNode) {
      return {
        success: false,
        path: [],
        explored: exploredNodes,
        nodesExpanded: closedSet.size,
        computeTimeMs,
        message: 'No feasible collision-free path found.'
      };
    }

    // Reconstruct path
    const rawPath = [];
    let curr = targetNode;
    while (curr) {
      rawPath.push({ x: curr.x, y: curr.y });
      curr = curr.parent;
    }
    rawPath.reverse();

    // Compute metrics
    const metrics = this.evaluatePath(rawPath, shipDraft);

    return {
      success: true,
      path: rawPath,
      explored: exploredNodes,
      nodesExpanded: closedSet.size,
      computeTimeMs,
      ...metrics
    };
  }

  /**
   * Evaluates path metrics: length, smoothness (total turning angle), min clearance
   */
  evaluatePath(path, shipDraft = 12.0) {
    if (!path || path.length < 2) {
      return {
        lengthMeters: 0,
        lengthCells: 0,
        smoothnessDeg: 0,
        minClearanceMeters: 0,
        curvatureViolations: 0
      };
    }

    let lengthCells = 0;
    let totalTurningRad = 0;
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

      if (i > 0 && i < path.length - 1) {
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
    }

    const lengthMeters = lengthCells * this.grid.cellSizeMeters;
    const smoothnessDeg = (totalTurningRad * 180) / Math.PI;

    return {
      lengthCells,
      lengthMeters,
      smoothnessDeg,
      minClearanceMeters: minClearance === Infinity ? 0 : minClearance,
      curvatureViolations: 0
    };
  }
}

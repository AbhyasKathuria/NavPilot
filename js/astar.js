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
   * Plans path from start to destination considering ship parameters,
   * bathymetric draft, continuous corner clearance, AIS traffic & drifting hazards.
   */
  plan(start, destination, shipDraft = 12.0, requiredClearanceMeters = 25.0, options = {}) {
    const startTime = performance.now();

    const startX = Math.round(start.x);
    const startY = Math.round(start.y);
    const destX = Math.round(destination.x);
    const destY = Math.round(destination.y);

    const aisSystem = options.aisSystem || null;
    const obstacleManager = options.obstacleManager || null;
    const shipDomainMeters = options.shipDomainMeters || requiredClearanceMeters;
    const cruiseSpeedCells = Math.max(0.05, options.cruiseSpeedCells || 0.16);

    // Check if start or destination is physically inside solid land or too shallow
    if (!this.grid.inBounds(destX, destY) ||
        this.grid.cells[destY][destX] === this.grid.TYPE_LAND ||
        this.grid.depths[destY][destX] < shipDraft) {
      return {
        success: false,
        path: [],
        explored: [],
        nodesExpanded: 0,
        computeTimeMs: performance.now() - startTime,
        message: 'Destination is inside a land obstacle or shallow shoal.'
      };
    }

    // Attempt full clearance first; if harbor entrance is tight, fallback once cleanly to hull-safe clearance
    let result = this._runSearch(startX, startY, destX, destY, shipDraft, requiredClearanceMeters, {
      aisSystem, obstacleManager, shipDomainMeters, cruiseSpeedCells, strictDynamic: true
    });

    if (!result.targetNode && (aisSystem || obstacleManager)) {
      result = this._runSearch(startX, startY, destX, destY, shipDraft, requiredClearanceMeters, {
        aisSystem, obstacleManager, shipDomainMeters, cruiseSpeedCells, strictDynamic: false
      });
    }

    if (!result.targetNode && requiredClearanceMeters > 24.0) {
      // Narrow strait fallback: still enforce strict static hull clearance (> halfBeam), never cut land!
      const narrowClearance = Math.max(22.0, requiredClearanceMeters * 0.72);
      result = this._runSearch(startX, startY, destX, destY, shipDraft, narrowClearance, {
        aisSystem, obstacleManager, shipDomainMeters, cruiseSpeedCells, strictDynamic: false
      });
    }

    const computeTimeMs = performance.now() - startTime;

    if (!result.targetNode) {
      return {
        success: false,
        path: [],
        explored: result.exploredNodes,
        nodesExpanded: result.closedCount,
        computeTimeMs,
        message: 'No feasible collision-free path found.'
      };
    }

    // Reconstruct path preserving exact continuous start coordinate if supplied
    const rawPath = [];
    let curr = result.targetNode;
    while (curr) {
      rawPath.push({ x: curr.x, y: curr.y });
      curr = curr.parent;
    }
    rawPath.reverse();
    if (Math.hypot(start.x - startX, start.y - startY) > 0.05 &&
        this.grid.lineOfSight(start.x, start.y, rawPath[0].x, rawPath[0].y, shipDraft, 16.0)) {
      rawPath[0] = { x: start.x, y: start.y };
    }

    const metrics = this.evaluatePath(rawPath, shipDraft);

    return {
      success: true,
      path: rawPath,
      explored: result.exploredNodes,
      nodesExpanded: result.closedCount,
      computeTimeMs,
      ...metrics
    };
  }

  _runSearch(startX, startY, destX, destY, shipDraft, requiredClearanceMeters, ctx) {
    const openSet = [];
    const openSetMap = new Map();
    const closedSet = new Set();
    const exploredNodes = [];

    const keyOf = (x, y) => `${x},${y}`;

    const startNode = {
      x: startX,
      y: startY,
      g: 0,
      h: this.heuristic(startX, startY, destX, destY),
      f: 0,
      parent: null
    };
    startNode.f = startNode.g + startNode.h;

    openSet.push(startNode);
    openSetMap.set(keyOf(startX, startY), startNode);

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

        const isGoalOrNearStart =
          (nx === destX && ny === destY) ||
          Math.hypot(nx - startX, ny - startY) <= 1.5 ||
          Math.hypot(nx - destX, ny - destY) <= 1.5;

        const effClearance = isGoalOrNearStart ? Math.min(requiredClearanceMeters, 28.0) : requiredClearanceMeters;

        if (!this.grid.isPassable(nx, ny, shipDraft, effClearance)) {
          continue;
        }

        // Prevent diagonal corner cutting on land or shallow cells
        if (dir.dx !== 0 && dir.dy !== 0) {
          if (!this.grid.isPassable(current.x + dir.dx, current.y, shipDraft, effClearance * 0.8) ||
              !this.grid.isPassable(current.x, current.y + dir.dy, shipDraft, effClearance * 0.8)) {
            continue;
          }
        }

        // Spatiotemporal avoidance of stationary/moving AIS vessels & dynamic debris
        const estDistCells = current.g + dir.cost;
        const arrivalTimeSec = estDistCells / ctx.cruiseSpeedCells;
        let dynamicPenalty = 0.0;

        if (ctx.aisSystem && ctx.aisSystem.vessels.length > 0) {
          const safeAIS = ctx.aisSystem.isSpatiotemporalSafe(nx, ny, arrivalTimeSec, ctx.shipDomainMeters, 15.0);
          if (!safeAIS) {
            if (ctx.strictDynamic) continue;
            dynamicPenalty += 25.0;
          }
        }
        if (ctx.obstacleManager && ctx.obstacleManager.obstacles.length > 0) {
          const safeObs = ctx.obstacleManager.isSpatiotemporalSafe(nx, ny, arrivalTimeSec, ctx.shipDomainMeters, 12.0);
          if (!safeObs) {
            if (ctx.strictDynamic) continue;
            dynamicPenalty += 25.0;
          }
        }

        // Subtle clearance cost gradient: keeps channels comfortably centered away from walls
        const edgeDist = this.grid.getExactHazardDistanceMeters(nx, ny, shipDraft);
        const safetyBias = edgeDist < 60.0 ? (60.0 - edgeDist) * 0.015 : 0.0;

        const tentativeG = current.g + dir.cost + dynamicPenalty + safetyBias;
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

    return {
      targetNode,
      exploredNodes,
      closedCount: closedSet.size
    };
  }

  /**
   * Evaluates path metrics: length, smoothness (total turning angle), exact min clearance
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
      const d = this.grid.getExactHazardDistanceMeters(path[i].x, path[i].y, shipDraft);
      if (d < minClearance) minClearance = d;

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

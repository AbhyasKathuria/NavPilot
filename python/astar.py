"""
NavPilot - Cargo Ship Real-Time Path-Planning Simulator
Stage 1: A* Global Path Search (astar.py)

Implements Stage 1 of Three-Stage Hybrid Planning Architecture (Figure 3.2):
- Global grid-based informed path search with octile heuristic
- Strictly checks ship draft and required clearance
- Prevents diagonal corner cutting
- Generates coarse waypoint route from Port M to Port N
"""

import heapq
import math
import time
from typing import Dict, List, Tuple


class AStarPlanner:
    def __init__(self, grid):
        self.grid = grid

    def heuristic(self, x1: float, y1: float, x2: float, y2: float) -> float:
        dx = abs(x1 - x2)
        dy = abs(y1 - y2)
        return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

    def plan(self, start: Dict[str, float], destination: Dict[str, float], ship_draft: float = 12.0, required_clearance: float = 25.0) -> Dict:
        start_time = time.perf_counter()

        sx, sy = int(round(start["x"])), int(round(start["y"]))
        dx, dy = int(round(destination["x"])), int(round(destination["y"]))

        if not self.grid.is_passable(dx, dy, ship_draft, required_clearance * 0.5):
            return {
                "success": False,
                "path": [],
                "nodes_expanded": 0,
                "compute_time_ms": (time.perf_counter() - start_time) * 1000.0,
                "message": "Destination Port N is impassable."
            }

        open_heap = []
        open_set_map = {}
        closed_set = set()
        explored_nodes = []

        start_h = self.heuristic(sx, sy, dx, dy)
        heapq.heappush(open_heap, (start_h, 0.0, (sx, sy)))
        open_set_map[(sx, sy)] = (0.0, None)

        directions = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))
        ]

        target_parent = None

        while open_heap:
            f, g, (cx, cy) = heapq.heappop(open_heap)

            if (cx, cy) in closed_set:
                continue

            closed_set.add((cx, cy))
            explored_nodes.append({"x": cx, "y": cy})

            if cx == dx and cy == dy:
                target_parent = open_set_map[(cx, cy)][1]
                break

            for d_x, d_y, cost in directions:
                nx, ny = cx + d_x, cy + d_y

                if not (0 <= nx < self.grid.cols and 0 <= ny < self.grid.rows):
                    continue
                if (nx, ny) in closed_set:
                    continue

                if not self.grid.is_passable(nx, ny, ship_draft, required_clearance):
                    continue

                # Diagonal corner cutting prevention
                if d_x != 0 and d_y != 0:
                    adj1 = self.grid.is_passable(cx + d_x, cy, ship_draft, required_clearance)
                    adj2 = self.grid.is_passable(cx, cy + d_y, ship_draft, required_clearance)
                    if not adj1 or not adj2:
                        continue

                tentative_g = g + cost
                if (nx, ny) not in open_set_map or tentative_g < open_set_map[(nx, ny)][0]:
                    h = self.heuristic(nx, ny, dx, dy)
                    open_set_map[(nx, ny)] = (tentative_g, (cx, cy))
                    heapq.heappush(open_heap, (tentative_g + h, tentative_g, (nx, ny)))

        compute_time_ms = (time.perf_counter() - start_time) * 1000.0

        if (dx, dy) not in closed_set and target_parent is None:
            return {
                "success": False,
                "path": [],
                "nodes_expanded": len(closed_set),
                "compute_time_ms": compute_time_ms,
                "message": "No collision-free path found."
            }

        # Reconstruct path
        raw_path = []
        curr = (dx, dy)
        while curr:
            raw_path.append({"x": curr[0], "y": curr[1]})
            curr = open_set_map[curr][1]
        raw_path.reverse()

        length_cells = 0.0
        min_clearance = float("inf")
        for i, pt in enumerate(raw_path):
            gx, gy = int(round(pt["x"])), int(round(pt["y"]))
            if self.grid.in_bounds(gx, gy):
                d = self.grid.distance_field[gy][gx]
                if d < min_clearance:
                    min_clearance = d
            if i > 0:
                length_cells += math.hypot(pt["x"] - raw_path[i - 1]["x"], pt["y"] - raw_path[i - 1]["y"])

        total_turning_rad = 0.0
        for i in range(1, len(raw_path) - 1):
            v1x = raw_path[i]["x"] - raw_path[i - 1]["x"]
            v1y = raw_path[i]["y"] - raw_path[i - 1]["y"]
            v2x = raw_path[i + 1]["x"] - raw_path[i]["x"]
            v2y = raw_path[i + 1]["y"] - raw_path[i]["y"]

            a1 = math.atan2(v1y, v1x)
            a2 = math.atan2(v2y, v2x)
            diff = abs(a2 - a1)
            if diff > math.pi:
                diff = 2 * math.pi - diff
            total_turning_rad += diff

        return {
            "success": True,
            "path": raw_path,
            "nodes_expanded": len(closed_set),
            "compute_time_ms": compute_time_ms,
            "length_meters": length_cells * self.grid.cell_size_meters,
            "length_cells": length_cells,
            "smoothness_deg": math.degrees(total_turning_rad),
            "min_clearance_meters": 0.0 if min_clearance == float("inf") else min_clearance
        }

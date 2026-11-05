"""
NavPilot - Cargo Ship Real-Time Path-Planning Simulator
Stage 3: Somersault Monkey Search Optimization (monkeysearch.py)

Implements Stage 3 of Three-Stage Hybrid Planning Architecture (Figure 3.2):
- Multi-objective fitness function (Transit time, Energy/rudder wear, Collision safety, Curvature)
- Strict Hard Constraint: Zero collision with land or shallow shoals (Infinity fitness penalty)
- Climb Process: Local fine-tuning of waypoints for minimum hydrodynamic energy & rudder wear
- Watch-Jump Process: Long-range exploratory jumping away from hazards and AIS vessel paths
- Somersault Process: Convergence towards elite population barycenter (center of gravity)
- Produces the final optimised cargo-ASV path
"""

import math
import random
import time
from typing import Dict, List, Optional, Tuple


class SomersaultMonkeySearchPlanner:
    def __init__(self, grid):
        self.grid = grid

    def optimize(self, seed_waypoints: List[Dict[str, float]], ship, ais_system=None, max_iterations: int = 25) -> Dict:
        """
        Optimizes a kinematically feasible path using Multi-Objective Somersault Monkey Search.
        """
        start_time = time.perf_counter()

        if not seed_waypoints or len(seed_waypoints) < 2:
            return {
                "success": False,
                "path": [],
                "waypoints": [],
                "compute_time_ms": 0.0,
                "message": "Invalid seed path for Stage 3 Somersault Monkey Search."
            }

        ship_draft = ship.draft
        required_clearance = ship.compute_required_clearance_meters()

        normalized_waypoints = self.normalize_waypoints(seed_waypoints, 3.5)
        num_waypoints = len(normalized_waypoints)

        if num_waypoints <= 2:
            dense = self.interpolate_waypoints(normalized_waypoints)
            return {
                "success": True,
                "path": dense,
                "waypoints": normalized_waypoints,
                "compute_time_ms": (time.perf_counter() - start_time) * 1000.0,
                **self.evaluate_path(dense, normalized_waypoints, ship)
            }

        pop_size = 16
        population: List[List[Dict[str, float]]] = []

        # Monkey 0: exact verified seed path from Stage 2 RRT*
        population.append(self.clone_waypoints(normalized_waypoints))

        # Seed remaining monkeys with subtle perturbations ONLY if line-of-sight remains valid
        for _ in range(1, pop_size):
            candidate = self.clone_waypoints(normalized_waypoints)
            for i in range(1, num_waypoints - 1):
                jitter_x = (random.random() - 0.5) * 1.2
                jitter_y = (random.random() - 0.5) * 1.2
                nx = candidate[i]["x"] + jitter_x
                ny = candidate[i]["y"] + jitter_y

                # HARD COLLISION CHECK
                if (self.grid.is_passable(nx, ny, ship_draft, required_clearance * 0.8) and
                    self.grid.line_of_sight(candidate[i - 1]["x"], candidate[i - 1]["y"], nx, ny, ship_draft, required_clearance * 0.8) and
                    self.grid.line_of_sight(nx, ny, candidate[i + 1]["x"], candidate[i + 1]["y"], ship_draft, required_clearance * 0.8)):
                    candidate[i]["x"] = nx
                    candidate[i]["y"] = ny

            population.append(candidate)

        # Initial fitness evaluation
        fitnesses = [self.compute_fitness(m, ship, ais_system) for m in population]
        best_idx = 0
        for i in range(1, pop_size):
            if fitnesses[i]["total"] < fitnesses[best_idx]["total"]:
                best_idx = i

        global_best = self.clone_waypoints(population[best_idx])
        global_best_fitness = fitnesses[best_idx]

        # Iterative Somersault Monkey Search Optimization Loop
        for iter_num in range(max_iterations):
            # 1. CLIMB PROCESS (Local fine-tuning for energy & smoothness)
            climb_step = max(0.10, 0.55 * (1.0 - iter_num / max_iterations))
            for p in range(pop_size):
                candidate = population[p]
                for i in range(1, num_waypoints - 1):
                    orig_x = candidate[i]["x"]
                    orig_y = candidate[i]["y"]

                    offsets = [
                        (climb_step, 0.0),
                        (-climb_step, 0.0),
                        (0.0, climb_step),
                        (0.0, -climb_step)
                    ]

                    best_local_x = orig_x
                    best_local_y = orig_y
                    current_fitness = fitnesses[p]["total"]

                    for dx, dy in offsets:
                        tx = orig_x + dx
                        ty = orig_y + dy

                        # HARD COLLISION INVARIANT
                        if (self.grid.is_passable(tx, ty, ship_draft, required_clearance * 0.75) and
                            self.grid.line_of_sight(candidate[i - 1]["x"], candidate[i - 1]["y"], tx, ty, ship_draft, required_clearance * 0.75) and
                            self.grid.line_of_sight(tx, ty, candidate[i + 1]["x"], candidate[i + 1]["y"], ship_draft, required_clearance * 0.75)):
                            candidate[i]["x"] = tx
                            candidate[i]["y"] = ty

                            eval_fit = self.compute_fitness(candidate, ship, ais_system)
                            if eval_fit["total"] < current_fitness:
                                current_fitness = eval_fit["total"]
                                best_local_x = tx
                                best_local_y = ty

                    candidate[i]["x"] = best_local_x
                    candidate[i]["y"] = best_local_y

                fitnesses[p] = self.compute_fitness(candidate, ship, ais_system)

            # 2. WATCH-JUMP PROCESS (Exploration away from coastlines and AIS traffic)
            jump_radius = max(0.4, 1.5 * (1.0 - iter_num / max_iterations))
            for p in range(pop_size):
                candidate = population[p]
                for i in range(1, num_waypoints - 1):
                    gx = int(round(candidate[i]["x"]))
                    gy = int(round(candidate[i]["y"]))
                    push_x = (random.random() - 0.5) * jump_radius
                    push_y = (random.random() - 0.5) * jump_radius

                    # Push away from chart hazards
                    if self.grid.in_bounds(gx, gy):
                        dist = self.grid.distance_field[gy][gx]
                        if dist < 120.0:
                            dx = (self.grid.distance_field[gy][min(self.grid.cols - 1, gx + 1)] -
                                  self.grid.distance_field[gy][max(0, gx - 1)]) / 2.0
                            dy = (self.grid.distance_field[min(self.grid.rows - 1, gy + 1)][gx] -
                                  self.grid.distance_field[max(0, gy - 1)][gx]) / 2.0
                            mag = math.hypot(dx, dy)
                            if mag > 0.01:
                                push_x += (dx / mag) * (jump_radius * 1.2)
                                push_y += (dy / mag) * (jump_radius * 1.2)

                    # Push away from active AIS traffic
                    if ais_system:
                        for ves in ais_system.vessels:
                            d_vessel = math.hypot(candidate[i]["x"] - ves.x, candidate[i]["y"] - ves.y)
                            if d_vessel < 5.0:
                                push_x += (candidate[i]["x"] - ves.x) / (d_vessel + 0.1) * jump_radius
                                push_y += (candidate[i]["y"] - ves.y) / (d_vessel + 0.1) * jump_radius

                    jx = candidate[i]["x"] + push_x
                    jy = candidate[i]["y"] + push_y

                    # HARD COLLISION INVARIANT
                    if (self.grid.is_passable(jx, jy, ship_draft, required_clearance * 0.75) and
                        self.grid.line_of_sight(candidate[i - 1]["x"], candidate[i - 1]["y"], jx, jy, ship_draft, required_clearance * 0.75) and
                        self.grid.line_of_sight(jx, jy, candidate[i + 1]["x"], candidate[i + 1]["y"], ship_draft, required_clearance * 0.75)):
                        orig_x = candidate[i]["x"]
                        orig_y = candidate[i]["y"]
                        candidate[i]["x"] = jx
                        candidate[i]["y"] = jy

                        test_fit = self.compute_fitness(candidate, ship, ais_system)
                        if test_fit["total"] < fitnesses[p]["total"]:
                            fitnesses[p] = test_fit
                        else:
                            candidate[i]["x"] = orig_x
                            candidate[i]["y"] = orig_y

            # 3. SOMERSAULT PROCESS (Convergence towards population barycenter)
            sorted_indices = sorted(range(pop_size), key=lambda idx: fitnesses[idx]["total"])
            elite_count = min(4, pop_size)

            barycenter = []
            for i in range(num_waypoints):
                sum_x = sum(population[sorted_indices[e]][i]["x"] for e in range(elite_count))
                sum_y = sum(population[sorted_indices[e]][i]["y"] for e in range(elite_count))
                barycenter.append({"x": sum_x / elite_count, "y": sum_y / elite_count})

            somersault_weight = 0.35
            for e in range(elite_count, pop_size):
                p = sorted_indices[e]
                candidate = population[p]
                for i in range(1, num_waypoints - 1):
                    target_x = barycenter[i]["x"]
                    target_y = barycenter[i]["y"]
                    sx = candidate[i]["x"] + somersault_weight * (target_x - candidate[i]["x"]) + (random.random() - 0.5) * 0.2
                    sy = candidate[i]["y"] + somersault_weight * (target_y - candidate[i]["y"]) + (random.random() - 0.5) * 0.2

                    # HARD COLLISION INVARIANT
                    if (self.grid.is_passable(sx, sy, ship_draft, required_clearance * 0.75) and
                        self.grid.line_of_sight(candidate[i - 1]["x"], candidate[i - 1]["y"], sx, sy, ship_draft, required_clearance * 0.75) and
                        self.grid.line_of_sight(sx, sy, candidate[i + 1]["x"], candidate[i + 1]["y"], ship_draft, required_clearance * 0.75)):
                        candidate[i]["x"] = sx
                        candidate[i]["y"] = sy

                fitnesses[p] = self.compute_fitness(candidate, ship, ais_system)

            # Update global best
            for p in range(pop_size):
                if fitnesses[p]["total"] < global_best_fitness["total"]:
                    global_best_fitness = fitnesses[p]
                    global_best = self.clone_waypoints(population[p])

        # Safety fallback check on every segment of global best
        is_safe = True
        for i in range(1, len(global_best)):
            if not self.grid.line_of_sight(global_best[i - 1]["x"], global_best[i - 1]["y"], global_best[i]["x"], global_best[i]["y"], ship_draft, required_clearance * 0.7):
                is_safe = False
                break

        if not is_safe:
            global_best = self.clone_waypoints(normalized_waypoints)

        compute_time_ms = (time.perf_counter() - start_time) * 1000.0
        dense_path = self.interpolate_waypoints(global_best)
        metrics = self.evaluate_path(dense_path, global_best, ship)

        return {
            "success": True,
            "path": dense_path,
            "waypoints": global_best,
            "compute_time_ms": compute_time_ms,
            **metrics
        }

    def compute_fitness(self, waypoints: List[Dict[str, float]], ship, ais_system) -> Dict:
        """
        Multi-objective Fitness Function:
        HARD CONSTRAINT: If any segment crosses land or shallow reef, returns Infinity!
        """
        length_cells = 0.0
        total_turning_rad = 0.0
        energy_index = 0.0
        proximity_penalty = 0.0
        curvature_violations = 0

        min_radius_cells = ship.min_turning_radius / self.grid.cell_size_meters
        required_clearance = ship.compute_required_clearance_meters()
        ship_draft = ship.draft

        for i, pt in enumerate(waypoints):
            gx = int(round(pt["x"]))
            gy = int(round(pt["y"]))

            # HARD CONSTRAINT
            if (not self.grid.in_bounds(gx, gy) or
                self.grid.cells[gy][gx] == self.grid.TYPE_LAND or
                self.grid.depths[gy][gx] < ship_draft):
                return {"total": float("inf"), "invalid": True}

            d_meters = self.grid.distance_field[gy][gx]
            if d_meters < required_clearance:
                proximity_penalty += (required_clearance - d_meters) * 10.0
            elif d_meters < required_clearance * 2.5:
                proximity_penalty += (required_clearance * 2.5 - d_meters) * 1.5

            if ais_system:
                for ves in ais_system.vessels:
                    d_vessel_m = math.hypot(pt["x"] - ves.x, pt["y"] - ves.y) * self.grid.cell_size_meters
                    combined_domain = ship.domain_radius_meters + ves.domain_radius_meters
                    if d_vessel_m < combined_domain:
                        proximity_penalty += 600.0
                    elif d_vessel_m < combined_domain * 2.0:
                        proximity_penalty += (combined_domain * 2.0 - d_vessel_m) * 1.2

            if i > 0:
                # HARD CONSTRAINT on segment
                if not self.grid.line_of_sight(waypoints[i - 1]["x"], waypoints[i - 1]["y"], pt["x"], pt["y"], ship_draft, required_clearance * 0.7):
                    return {"total": float("inf"), "invalid": True}

                seg_dist = math.hypot(pt["x"] - waypoints[i - 1]["x"], pt["y"] - waypoints[i - 1]["y"])
                length_cells += seg_dist

            if 0 < i < len(waypoints) - 1:
                v1x = pt["x"] - waypoints[i - 1]["x"]
                v1y = pt["y"] - waypoints[i - 1]["y"]
                v2x = waypoints[i + 1]["x"] - pt["x"]
                v2y = waypoints[i + 1]["y"] - pt["y"]

                a1 = math.atan2(v1y, v1x)
                a2 = math.atan2(v2y, v2x)
                diff = abs(a2 - a1)
                if diff > math.pi:
                    diff = 2 * math.pi - diff
                total_turning_rad += diff

                energy_index += (diff ** 2) * 20.0

                if diff > 0.05:
                    req_dist = min_radius_cells * math.tan(diff / 2.0)
                    len1 = math.hypot(v1x, v1y)
                    len2 = math.hypot(v2x, v2y)
                    if req_dist > len1 * 0.95 or req_dist > len2 * 0.95:
                        curvature_violations += 1

        time_index = length_cells
        w_time = 1.0
        w_energy = 12.0
        w_safety = 1.2
        w_curv = 150.0

        total = (w_time * time_index) + (w_energy * energy_index) + (w_safety * proximity_penalty) + (w_curv * curvature_violations)
        return {
            "total": total,
            "time_index": time_index,
            "energy_index": energy_index,
            "proximity_penalty": proximity_penalty,
            "curvature_violations": curvature_violations
        }

    def normalize_waypoints(self, waypoints: List[Dict[str, float]], max_spacing_cells: float = 3.5) -> List[Dict[str, float]]:
        if not waypoints or len(waypoints) <= 1:
            return list(waypoints) or []
        result = [waypoints[0]]

        for i in range(1, len(waypoints)):
            p0 = waypoints[i - 1]
            p1 = waypoints[i]
            dist = math.hypot(p1["x"] - p0["x"], p1["y"] - p0["y"])

            if dist > max_spacing_cells:
                sub_count = int(math.ceil(dist / max_spacing_cells))
                for s in range(1, sub_count):
                    t = s / sub_count
                    result.append({
                        "x": p0["x"] + (p1["x"] - p0["x"]) * t,
                        "y": p0["y"] + (p1["y"] - p0["y"]) * t
                    })
            result.append(p1)
        return result

    def interpolate_waypoints(self, waypoints: List[Dict[str, float]], step_size_cells: float = 0.4) -> List[Dict[str, float]]:
        if not waypoints or len(waypoints) <= 1:
            return list(waypoints) or []
        dense = [waypoints[0]]

        for i in range(1, len(waypoints)):
            p0 = waypoints[i - 1]
            p1 = waypoints[i]
            dist = math.hypot(p1["x"] - p0["x"], p1["y"] - p0["y"])
            steps = max(1, int(math.ceil(dist / step_size_cells)))

            for s in range(1, steps + 1):
                t = s / steps
                dense.append({
                    "x": p0["x"] + (p1["x"] - p0["x"]) * t,
                    "y": p0["y"] + (p1["y"] - p0["y"]) * t
                })
        return dense

    def clone_waypoints(self, wps: List[Dict[str, float]]) -> List[Dict[str, float]]:
        return [{"x": p["x"], "y": p["y"]} for p in wps]

    def evaluate_path(self, dense_path: List[Dict[str, float]], waypoints: List[Dict[str, float]], ship) -> Dict:
        length_cells = 0.0
        min_clearance = float("inf")

        for i, pt in enumerate(dense_path):
            gx = int(round(pt["x"]))
            gy = int(round(pt["y"]))
            if self.grid.in_bounds(gx, gy):
                d = self.grid.distance_field[gy][gx]
                if d < min_clearance:
                    min_clearance = d
            if i > 0:
                length_cells += math.hypot(pt["x"] - dense_path[i - 1]["x"], pt["y"] - dense_path[i - 1]["y"])

        total_turning_rad = 0.0
        energy_est = 0.0
        if waypoints and len(waypoints) >= 3:
            for i in range(1, len(waypoints) - 1):
                v1x = waypoints[i]["x"] - waypoints[i - 1]["x"]
                v1y = waypoints[i]["y"] - waypoints[i - 1]["y"]
                v2x = waypoints[i + 1]["x"] - waypoints[i]["x"]
                v2y = waypoints[i + 1]["y"] - waypoints[i]["y"]

                a1 = math.atan2(v1y, v1x)
                a2 = math.atan2(v2y, v2x)
                diff = abs(a2 - a1)
                if diff > math.pi:
                    diff = 2 * math.pi - diff
                total_turning_rad += diff
                energy_est += (diff ** 2) * 20.0

        length_meters = length_cells * self.grid.cell_size_meters
        cruise_speed_ms = ship.max_speed_ms * 0.85
        estimated_transit_time_sec = length_meters / max(1.0, cruise_speed_ms)

        return {
            "length_cells": length_cells,
            "length_meters": length_meters,
            "smoothness_deg": math.degrees(total_turning_rad),
            "min_clearance_meters": 0.0 if min_clearance == float("inf") else min_clearance,
            "curvature_violations": 0,
            "estimated_transit_time_sec": estimated_transit_time_sec,
            "energy_index": round(energy_est, 1)
        }

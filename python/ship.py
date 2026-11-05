"""
NavPilot - Cargo Ship Real-Time Path-Planning Simulator
Ship Hydrodynamics & Domain Model (ship.py)

Implements:
- Ship Domain: SS = L * B * D (Length * Beam * Draft)
- Non-holonomic kinematics and turning radius limits
- Stopping distance and hydrodynamic drag
- Multi-stage trajectory storage
"""

import math
from typing import Dict, List, Optional


class CargoASV:
    def __init__(self,
                 length: float = 200.0,
                 beam: float = 32.0,
                 draft: float = 12.0,
                 displacement: float = 50000.0,
                 max_speed_knots: float = 18.0,
                 min_turning_radius: float = 400.0):
        # Physical Dimensions
        self.length = length             # L (meters)
        self.beam = beam                 # B (meters)
        self.draft = draft               # D (meters)
        self.displacement = displacement # metric tonnes
        self.max_speed_knots = max_speed_knots
        self.min_turning_radius = min_turning_radius # meters

        # Live Kinematic State
        self.x = 4.0
        self.y = 45.0
        self.heading = -math.pi / 4.0
        self.speed = 0.0
        self.target_speed = 0.0
        self.rudder_angle = 0.0

        # State string
        self.state = "IDLE" # IDLE, CRUISING, HOLD_WAIT, EMERGENCY_STOP, ARRIVED

        # Stage Trajectories
        self.stage1_path: Optional[List[Dict[str, float]]] = None
        self.stage2_path: Optional[List[Dict[str, float]]] = None
        self.stage3_path: Optional[List[Dict[str, float]]] = None
        self.active_path: Optional[List[Dict[str, float]]] = None
        self.replan_path: Optional[List[Dict[str, float]]] = None
        self.path_index = 0
        self.has_replanned = False
        self.total_replans = 0

    @property
    def ship_domain_volume(self) -> float:
        """Computes volumetric Ship Domain: SS = L * B * D in m^3"""
        return self.length * self.beam * self.draft

    @property
    def domain_radius_meters(self) -> float:
        """Planar 2D safety domain radius in meters"""
        half_beam = self.beam / 2.0
        base_margin = 15.0
        displacement_buffer = (self.displacement / 50000.0) * 12.0
        speed_ratio = (self.speed / self.max_speed_ms) if self.speed > 0.1 else 0.8
        dynamic_buffer = speed_ratio * 8.0
        return half_beam + base_margin + displacement_buffer + dynamic_buffer

    @property
    def max_speed_ms(self) -> float:
        return self.max_speed_knots * 0.514444

    @property
    def max_curvature(self) -> float:
        return 1.0 / self.min_turning_radius

    def compute_stopping_distance_meters(self, speed_ms: Optional[float] = None) -> float:
        v = speed_ms if speed_ms is not None else (self.speed if self.speed > 0.5 else self.max_speed_ms)
        effective_braking_force = self.displacement * 1000.0 * 0.12 * 9.81
        mass_kg = self.displacement * 1000.0
        return max(25.0, (mass_kg * v * v) / (2.0 * effective_braking_force))

    def compute_required_clearance_meters(self) -> float:
        return self.domain_radius_meters

    @property
    def lookahead_horizon_seconds(self) -> float:
        stopping_dist = self.compute_stopping_distance_meters()
        v = max(1.0, self.speed)
        return max(15.0, (stopping_dist / v) + 8.0)

    @property
    def max_turn_rate(self) -> float:
        v = max(2.0, self.speed if self.speed > 0.5 else self.max_speed_ms * 0.7)
        inertia_factor = (50000.0 / self.displacement) ** 0.35
        return (v / self.min_turning_radius) * min(1.2, max(0.4, inertia_factor))

    def set_stage_paths(self, stage1, stage2, stage3):
        self.stage1_path = stage1
        self.stage2_path = stage2
        self.stage3_path = stage3
        self.set_path(stage3 or stage2 or stage1)

    def set_path(self, path_points: List[Dict[str, float]], is_replan: bool = False):
        if not path_points:
            return
        if not is_replan:
            self.active_path = path_points
            self.replan_path = None
            self.has_replanned = False
            self.path_index = 0
            self.x = path_points[0]["x"]
            self.y = path_points[0]["y"]
            if len(path_points) > 1:
                self.heading = math.atan2(path_points[1]["y"] - path_points[0]["y"],
                                          path_points[1]["x"] - path_points[0]["x"])
        else:
            self.replan_path = path_points
            self.active_path = path_points
            self.has_replanned = True
            self.total_replans += 1
            self.path_index = 0
        self.target_speed = self.max_speed_ms
        self.state = "CRUISING"

    def hold_and_wait(self):
        self.state = "HOLD_WAIT"
        self.target_speed = 0.0

    def emergency_stop(self):
        self.state = "EMERGENCY_STOP"
        self.target_speed = 0.0

    def update(self, dt: float, grid=None, cell_size_meters: float = 50.0):
        if not self.active_path or len(self.active_path) < 2 or self.state in ("IDLE", "ARRIVED"):
            return

        accel_rate = 0.5 * ((50000.0 / self.displacement) ** 0.4)
        decel_rate = 0.8 * ((50000.0 / self.displacement) ** 0.4)

        if self.speed < self.target_speed:
            self.speed = min(self.target_speed, self.speed + accel_rate * dt)
        elif self.speed > self.target_speed:
            self.speed = max(self.target_speed, self.speed - decel_rate * dt)

        if self.state in ("HOLD_WAIT", "EMERGENCY_STOP"):
            if self.speed <= 0.05:
                self.speed = 0.0
            return

        move_cells = (self.speed * dt) / cell_size_meters
        if move_cells <= 0:
            return

        remaining_move = move_cells
        while remaining_move > 0 and self.path_index < len(self.active_path) - 1:
            p_next = self.active_path[self.path_index + 1]
            seg_dx = p_next["x"] - self.x
            seg_dy = p_next["y"] - self.y
            dist_to_next = math.hypot(seg_dx, seg_dy)

            if dist_to_next <= 0.001:
                self.path_index += 1
                continue

            if remaining_move >= dist_to_next:
                self.x = p_next["x"]
                self.y = p_next["y"]
                remaining_move -= dist_to_next
                self.path_index += 1
            else:
                ratio = remaining_move / dist_to_next
                self.x += seg_dx * ratio
                self.y += seg_dy * ratio
                remaining_move = 0.0

        if self.path_index < len(self.active_path) - 1:
            p_curr = self.active_path[self.path_index]
            p_next = self.active_path[min(len(self.active_path) - 1, self.path_index + 1)]
            target_heading = math.atan2(p_next["y"] - p_curr["y"], p_next["x"] - p_curr["x"])

            diff = target_heading - self.heading
            while diff > math.pi: diff -= 2 * math.pi
            while diff < -math.pi: diff += 2 * math.pi

            max_turn = max(0.15, (self.speed / self.min_turning_radius) * (cell_size_meters / 50.0) * 12.0) * dt
            applied_turn = max(-max_turn, min(max_turn, diff))
            self.heading += applied_turn
            self.rudder_angle = max(-0.6, min(0.6, diff * 1.5))

        # Hard obstacle collision guard: Never overlap land or shoal
        if grid:
            gx = int(round(self.x))
            gy = int(round(self.y))
            if not grid.in_bounds(gx, gy) or grid.cells[gy][gx] == grid.TYPE_LAND or grid.depths[gy][gx] < self.draft:
                safe_pt = self.active_path[min(self.path_index, len(self.active_path) - 1)]
                self.x = safe_pt["x"]
                self.y = safe_pt["y"]

        if self.path_index >= len(self.active_path) - 1:
            final_dest = self.active_path[-1]
            if math.hypot(final_dest["x"] - self.x, final_dest["y"] - self.y) < 0.25:
                self.state = "ARRIVED"
                self.speed = 0.0

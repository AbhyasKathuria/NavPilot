"""
NavPilot - Cargo Ship Real-Time Path-Planning Simulator
AIS Marine Traffic Simulation System (ais.py)

Implements:
- Live AIS broadcasts: MMSI, Vessel Name, SOG, COG, Heading, Length, Beam, Draft
- Continuous vessel kinematics before, during, and after path planning
- Trajectory prediction and CPA / TCPA risk assessment against Ship Domain SS = L * B * D
"""

import math
from typing import Dict, List, Optional, Tuple


class AISVessel:
    def __init__(self,
                 mmsi: int,
                 name: str,
                 vessel_type: str,
                 x: float,
                 y: float,
                 sog: float,
                 cog: float,
                 length: float = 200.0,
                 beam: float = 32.0,
                 draft: float = 11.5):
        self.mmsi = mmsi
        self.name = name
        self.type = vessel_type
        self.x = x
        self.y = y
        self.sog = sog  # knots
        self.cog = cog  # degrees
        self.heading = cog
        self.length = length
        self.beam = beam
        self.draft = draft

        self.vx = 0.0
        self.vy = 0.0
        self.update_velocity()

        self.cpa_meters = float("inf")
        self.tcpa_seconds = float("inf")
        self.risk_level = "CLEAR" # CLEAR, CAUTION, WARNING, COLLISION_ALERT

    def update_velocity(self):
        speed_ms = self.sog * 0.514444
        speed_cells = speed_ms / 50.0
        rad = math.radians(self.cog)
        self.vx = math.sin(rad) * speed_cells
        self.vy = -math.cos(rad) * speed_cells

    def update(self, dt: float, grid):
        self.x += self.vx * dt
        self.y += self.vy * dt

        pad = 1.0
        bounced = False
        if self.x < pad:
            self.x = pad
            self.cog = 360.0 - self.cog
            bounced = True
        elif self.x > grid.cols - 1 - pad:
            self.x = grid.cols - 1 - pad
            self.cog = 360.0 - self.cog
            bounced = True

        if self.y < pad:
            self.y = pad
            self.cog = (180.0 - self.cog + 360.0) % 360.0
            bounced = True
        elif self.y > grid.rows - 1 - pad:
            self.y = grid.rows - 1 - pad
            self.cog = (180.0 - self.cog + 360.0) % 360.0
            bounced = True

        gx = int(round(self.x))
        gy = int(round(self.y))
        if 0 <= gx < grid.cols and 0 <= gy < grid.rows:
            if grid.cells[gy][gx] == grid.TYPE_LAND:
                self.cog = (self.cog + 140.0) % 360.0
                bounced = True

        if bounced:
            self.heading = self.cog
            self.update_velocity()

    def predict_position(self, t_seconds: float) -> Tuple[float, float]:
        return (self.x + self.vx * t_seconds, self.y + self.vy * t_seconds)

    @property
    def domain_radius_meters(self) -> float:
        return max(35.0, self.length * 0.65)


class AISSystem:
    def __init__(self, grid):
        self.grid = grid
        self.vessels: List[AISVessel] = []
        self.init_default_traffic()

    def init_traffic(self):
        self.init_default_traffic()

    def init_default_traffic(self, preset: str = "archipelago"):
        if preset == "singapore":
            self.vessels = [
                AISVessel(563042000, "MV PACIFIC PIONEER", "Container Vessel", 10.0, 24.0, 15.0, 90.0, 260.0, 38.0, 13.5),
                AISVessel(211345000, "STENA POLARIS", "Oil Tanker", 40.0, 26.0, 12.0, 270.0, 220.0, 36.0, 14.0),
                AISVessel(636019000, "COSCO SHIPPING ASIA", "Bulk Carrier", 20.0, 23.0, 13.5, 90.0, 195.0, 32.0, 11.8),
                AISVessel(477123000, "COASTAL PATROL 08", "Patrol Craft", 30.0, 27.0, 18.0, 270.0, 85.0, 16.0, 6.2)
            ]
        else:
            self.vessels = [
                AISVessel(563042000, "MV PACIFIC PIONEER", "Container Vessel", 16.0, 10.0, 14.5, 135.0, 260.0, 38.0, 13.5),
                AISVessel(211345000, "STENA POLARIS", "Oil Tanker", 38.0, 34.0, 11.2, 295.0, 220.0, 36.0, 14.0),
                AISVessel(636019000, "COSCO SHIPPING ASIA", "Bulk Carrier", 29.0, 18.0, 12.0, 210.0, 195.0, 32.0, 11.8),
                AISVessel(477123000, "COASTAL PATROL 08", "Patrol Craft", 10.0, 36.0, 18.5, 50.0, 85.0, 16.0, 6.2)
            ]

    def update(self, dt: float, current_path: Optional[List[Dict[str, float]]] = None, ship_domain_radius: float = 40.0):
        for v in self.vessels:
            v.update(dt, self.grid)

        # Update CPA/TCPA against current path
        if current_path:
            for v in self.vessels:
                min_dist = float("inf")
                best_t = 0.0
                for s in range(1, 31):
                    t = float(s)
                    vx, vy = v.predict_position(t)
                    # Find closest point along path
                    for pt in current_path[::2]:
                        d = math.hypot(pt["x"] - vx, pt["y"] - vy) * self.grid.cell_size_meters
                        if d < min_dist:
                            min_dist = d
                            best_t = t
                v.cpa_meters = min_dist
                v.tcpa_seconds = best_t
                safety_buffer = ship_domain_radius + v.domain_radius_meters
                if min_dist < safety_buffer:
                    v.risk_level = "COLLISION_ALERT"
                elif min_dist < safety_buffer * 1.5:
                    v.risk_level = "WARNING"
                elif min_dist < safety_buffer * 2.5:
                    v.risk_level = "CAUTION"
                else:
                    v.risk_level = "CLEAR"

    def get_traffic_summary(self) -> List[Dict]:
        return [
            {
                "mmsi": v.mmsi,
                "name": v.name,
                "type": v.type,
                "sog": v.sog,
                "cog": v.cog,
                "cpa_meters": v.cpa_meters,
                "tcpa_seconds": v.tcpa_seconds,
                "risk": v.risk_level
            }
            for v in self.vessels
        ]

    def evaluate_traffic_risk(self, asv, horizon_seconds: float = 30.0) -> Dict:
        asv_speed_cells = (asv.speed if asv.speed > 0.5 else asv.max_speed_ms) / self.grid.cell_size_meters
        asv_domain = asv.domain_radius_meters

        conflict = False
        critical_vessel = None
        min_cpa = float("inf")
        min_tcpa = float("inf")

        for v in self.vessels:
            safety_buffer = asv_domain + v.domain_radius_meters
            closest_dist = float("inf")
            closest_time = float("inf")

            dt_sample = 0.5
            steps = math.ceil(horizon_seconds / dt_sample)

            for s in range(1, steps + 1):
                t = s * dt_sample
                asv_dist_cells = asv_speed_cells * t
                asv_pos = self.get_future_asv_pos(asv, asv_dist_cells)
                if not asv_pos:
                    break
                vx, vy = v.predict_position(t)
                d = math.hypot(asv_pos[0] - vx, asv_pos[1] - vy) * self.grid.cell_size_meters
                if d < closest_dist:
                    closest_dist = d
                    closest_time = t

            v.cpa_meters = closest_dist
            v.tcpa_seconds = closest_time

            if closest_dist < safety_buffer and closest_time < horizon_seconds:
                v.risk_level = "COLLISION_ALERT"
                conflict = True
                if closest_dist < min_cpa:
                    min_cpa = closest_dist
                    min_tcpa = closest_time
                    critical_vessel = v
            elif closest_dist < safety_buffer * 1.6 and closest_time < horizon_seconds:
                v.risk_level = "WARNING"
            elif closest_dist < safety_buffer * 2.5:
                v.risk_level = "CAUTION"
            else:
                v.risk_level = "CLEAR"

        return {
            "conflict_detected": conflict,
            "critical_vessel": critical_vessel,
            "cpa_meters": min_cpa,
            "tcpa_seconds": min_tcpa
        }

    def get_future_asv_pos(self, asv, dist_cells: float) -> Optional[Tuple[float, float]]:
        if not asv.active_path or asv.path_index >= len(asv.active_path):
            return None
        accum = 0.0
        last_x, last_y = asv.x, asv.y
        for i in range(asv.path_index, len(asv.active_path)):
            pt = asv.active_path[i]
            d = math.hypot(pt["x"] - last_x, pt["y"] - last_y)
            if accum + d >= dist_cells:
                ratio = (dist_cells - accum) / d if d > 0 else 0
                return (last_x + (pt["x"] - last_x) * ratio, last_y + (pt["y"] - last_y) * ratio)
            accum += d
            last_x, last_y = pt["x"], pt["y"]
        end_pt = asv.active_path[-1]
        return (end_pt["x"], end_pt["y"])

"""
NavPilot - Cargo Ship Real-Time Path-Planning Simulator
Bathymetric Marine Grid System (grid.py)

Implements:
- 50x50 bathymetric marine grid with cell depths (Deep Sea, Shallow Shoals, Land)
- Port M (Departure Port) and Port N (Destination Port)
- Euclidean Distance Transform (EDT) for distance to nearest hazard
- Continuous swept capsule Line-of-Sight (LOS) checking
- Maritime scenario presets (Archipelago, Singapore Strait, Iceberg Alley, Canal)
"""

import math
from collections import deque
from typing import Dict, List, Tuple


class BathymetricGrid:
    TYPE_SEA = 0      # Deep navigable sea (>20m)
    TYPE_SHALLOW = 1  # Shallow reef/shoal (~8m)
    TYPE_LAND = 2     # Land / island rock obstacle (0m)

    def __init__(self, cols: int = 50, rows: int = 50, cell_size_meters: float = 50.0):
        self.cols = cols
        self.rows = rows
        self.cell_size_meters = cell_size_meters  # 50m/cell (2500m x 2500m area)

        # 2D matrices: [y][x]
        self.cells = [[self.TYPE_SEA for _ in range(cols)] for _ in range(rows)]
        self.depths = [[28.0 for _ in range(cols)] for _ in range(rows)]
        self.distance_field = [[9999.0 for _ in range(cols)] for _ in range(rows)]

        # Departure Port M and Destination Port N
        self.port_m = {"x": 5, "y": 44, "name": "Port M (Departure)"}
        self.port_n = {"x": 44, "y": 6, "name": "Port N (Destination)"}

        self.load_preset("archipelago")

    def in_bounds(self, x: float, y: float) -> bool:
        gx = int(round(x))
        gy = int(round(y))
        return 0 <= gx < self.cols and 0 <= gy < self.rows

    def set_cell(self, x: int, y: int, cell_type: int, depth: float = None):
        if not (0 <= x < self.cols and 0 <= y < self.rows):
            return
        self.cells[y][x] = cell_type
        if depth is not None:
            self.depths[y][x] = depth
        else:
            if cell_type == self.TYPE_LAND:
                self.depths[y][x] = 0.0
            elif cell_type == self.TYPE_SHALLOW:
                self.depths[y][x] = 8.0
            else:
                self.depths[y][x] = 28.0

    def compute_distance_field(self, ship_draft: float = 12.0):
        """Computes Euclidean distance in meters from each cell to nearest hazard (land or depth < draft)."""
        dist = [[float("inf") for _ in range(self.cols)] for _ in range(self.rows)]
        queue = deque()

        for y in range(self.rows):
            for x in range(self.cols):
                is_hazard = (self.cells[y][x] == self.TYPE_LAND) or (self.depths[y][x] < ship_draft)
                if is_hazard:
                    dist[y][x] = 0.0
                    queue.append((x, y, 0.0))

        directions = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))
        ]

        while queue:
            cx, cy, d = queue.popleft()
            for dx, dy, step_d in directions:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < self.cols and 0 <= ny < self.rows:
                    new_dist = d + step_d
                    if new_dist < dist[ny][nx]:
                        dist[ny][nx] = new_dist
                        queue.append((nx, ny, new_dist))

        for y in range(self.rows):
            for x in range(self.cols):
                self.distance_field[y][x] = dist[y][x] * self.cell_size_meters

    def is_passable(self, x: float, y: float, ship_draft: float = 12.0, required_clearance: float = 25.0) -> bool:
        gx = int(round(x))
        gy = int(round(y))
        if not (0 <= gx < self.cols and 0 <= gy < self.rows):
            return False

        if self.cells[gy][gx] == self.TYPE_LAND:
            return False
        if self.depths[gy][gx] < ship_draft:
            return False

        if self.distance_field[gy][gx] < required_clearance:
            return False
        return True

    def line_of_sight(self, x0: float, y0: float, x1: float, y1: float, ship_draft: float = 12.0, required_clearance: float = 25.0) -> bool:
        """Continuous raycast checking if swept capsule passes cleanly without touching land or shoals."""
        dx = x1 - x0
        dy = y1 - y0
        dist_cells = math.hypot(dx, dy)
        if dist_cells == 0:
            return self.is_passable(x0, y0, ship_draft, required_clearance)

        steps = max(1, math.ceil(dist_cells / 0.35))
        for s in range(steps + 1):
            t = s / steps
            px = x0 + dx * t
            py = y0 + dy * t
            if not self.is_passable(px, py, ship_draft, required_clearance):
                return False
        return True

    def get_straight_line_distance(self) -> Dict[str, float]:
        dx = (self.port_n["x"] - self.port_m["x"]) * self.cell_size_meters
        dy = (self.port_n["y"] - self.port_m["y"]) * self.cell_size_meters
        meters = math.hypot(dx, dy)
        km = meters / 1000.0
        nautical_miles = meters / 1852.0
        return {"meters": meters, "km": km, "nautical_miles": nautical_miles}

    def draw_circular_island(self, cx: float, cy: float, radius_land: float, radius_shallow: float):
        total_r = radius_land + radius_shallow
        for dy in range(-int(math.ceil(total_r)), int(math.ceil(total_r)) + 1):
            for dx in range(-int(math.ceil(total_r)), int(math.ceil(total_r)) + 1):
                x = int(round(cx + dx))
                y = int(round(cy + dy))
                if 0 <= x < self.cols and 0 <= y < self.rows:
                    d = math.hypot(dx, dy)
                    if d <= radius_land:
                        self.cells[y][x] = self.TYPE_LAND
                        self.depths[y][x] = 0.0
                    elif d <= total_r and self.cells[y][x] != self.TYPE_LAND:
                        self.cells[y][x] = self.TYPE_SHALLOW
                        self.depths[y][x] = 7.5

    def load_preset(self, name: str = "archipelago"):
        self.cells = [[self.TYPE_SEA for _ in range(self.cols)] for _ in range(self.rows)]
        self.depths = [[28.0 for _ in range(self.cols)] for _ in range(self.rows)]

        if name == "archipelago":
            self.port_m = {"x": 4, "y": 45, "name": "Port M (Departure)"}
            self.port_n = {"x": 45, "y": 4, "name": "Port N (Destination)"}
            self.draw_circular_island(14, 28, 6, 3)
            self.draw_circular_island(34, 20, 7, 3)
            self.draw_circular_island(20, 10, 5, 2)
            self.draw_circular_island(38, 38, 5, 2)
            for x in range(22, 29):
                self.set_cell(x, 26, self.TYPE_LAND)
                self.set_cell(x, 27, self.TYPE_LAND)
                self.set_cell(x, 28, self.TYPE_SHALLOW, 6.0)
                self.set_cell(x, 25, self.TYPE_SHALLOW, 7.5)

        elif name == "singapore":
            self.port_m = {"x": 3, "y": 25, "name": "Port M (Departure)"}
            self.port_n = {"x": 46, "y": 24, "name": "Port N (Destination)"}
            for x in range(self.cols):
                land_h = int(10 + math.sin(x * 0.25) * 3)
                for y in range(land_h):
                    self.set_cell(x, y, self.TYPE_LAND)
                for y in range(land_h, land_h + 3):
                    self.set_cell(x, y, self.TYPE_SHALLOW, 7.0)

                island_top = int(38 - math.sin(x * 0.2) * 4)
                for y in range(island_top, self.rows):
                    self.set_cell(x, y, self.TYPE_LAND)
                for y in range(island_top - 3, island_top):
                    self.set_cell(x, y, self.TYPE_SHALLOW, 7.5)

            self.draw_circular_island(25, 22, 3, 2)
            self.draw_circular_island(15, 29, 2.5, 2)
            self.draw_circular_island(36, 18, 3, 2)

        elif name == "ice_alley":
            self.port_m = {"x": 4, "y": 44, "name": "Port M (Departure)"}
            self.port_n = {"x": 44, "y": 5, "name": "Port N (Destination)"}
            self.draw_circular_island(12, 36, 4, 2)
            self.draw_circular_island(18, 32, 3, 2)
            self.draw_circular_island(25, 25, 5, 2)
            self.draw_circular_island(22, 18, 4, 2)
            self.draw_circular_island(30, 20, 3, 2)
            self.draw_circular_island(36, 12, 4, 2)
            self.draw_circular_island(42, 18, 3, 2)
            for y in range(10, 41, 4):
                self.set_cell(10, y, self.TYPE_SHALLOW, 7.0)
                self.set_cell(38, y, self.TYPE_SHALLOW, 7.0)

        elif name == "canal":
            self.port_m = {"x": 5, "y": 42, "name": "Port M (Departure)"}
            self.port_n = {"x": 42, "y": 8, "name": "Port N (Destination)"}
            for x in range(35):
                self.set_cell(x, 20, self.TYPE_LAND)
                self.set_cell(x, 19, self.TYPE_SHALLOW, 6.0)
            for x in range(15, self.cols):
                self.set_cell(x, 30, self.TYPE_LAND)
                self.set_cell(x, 31, self.TYPE_SHALLOW, 6.0)
            self.draw_circular_island(12, 36, 3, 2)
            self.draw_circular_island(38, 14, 3, 2)

        # Clear Port M and Port N
        self.set_cell(self.port_m["x"], self.port_m["y"], self.TYPE_SEA)
        self.set_cell(self.port_n["x"], self.port_n["y"], self.TYPE_SEA)
        self.compute_distance_field()

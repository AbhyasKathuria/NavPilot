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

    def get_exact_hazard_distance_meters(self, x: float, y: float, ship_draft: float = 12.0) -> float:
        """Continuous distance in meters from (x, y) to nearest hazard square border [cx-0.5, cx+0.5]."""
        if x < -0.49 or x > self.cols - 0.51 or y < -0.49 or y > self.rows - 0.51:
            return 0.0

        gx = max(0, min(self.cols - 1, int(round(x))))
        gy = max(0, min(self.rows - 1, int(round(y))))
        quick_center = self.distance_field[gy][gx]
        if quick_center >= 150.0:
            return quick_center - 0.5 * self.cell_size_meters

        min_cx = max(0, math.floor(x) - 2)
        max_cx = min(self.cols - 1, math.ceil(x) + 2)
        min_cy = max(0, math.floor(y) - 2)
        max_cy = min(self.rows - 1, math.ceil(y) + 2)

        min_dist_cells = float("inf")
        for cy in range(min_cy, max_cy + 1):
            for cx in range(min_cx, max_cx + 1):
                if self.cells[cy][cx] == self.TYPE_LAND or self.depths[cy][cx] < ship_draft:
                    dx = max(0.0, abs(x - cx) - 0.5)
                    dy = max(0.0, abs(y - cy) - 0.5)
                    d = math.hypot(dx, dy)
                    if d < min_dist_cells:
                        min_dist_cells = d
                        if d == 0.0:
                            return 0.0

        if min_dist_cells != float("inf"):
            return min_dist_cells * self.cell_size_meters

        return max(0.0, quick_center - 0.5 * self.cell_size_meters)

    def to_edge_clearance_meters(self, required_clearance: float = 25.0) -> float:
        if required_clearance <= 0.0:
            return 0.0
        if required_clearance >= 0.5 * self.cell_size_meters:
            return max(18.0, required_clearance - 0.5 * self.cell_size_meters)
        return required_clearance

    def is_passable(self, x: float, y: float, ship_draft: float = 12.0, required_clearance: float = 25.0) -> bool:
        gx = int(round(x))
        gy = int(round(y))
        if not (0 <= gx < self.cols and 0 <= gy < self.rows):
            return False

        if self.cells[gy][gx] == self.TYPE_LAND:
            return False
        if self.depths[gy][gx] < ship_draft:
            return False

        edge_dist = self.get_exact_hazard_distance_meters(x, y, ship_draft)
        if edge_dist <= 0.0:
            return False

        if edge_dist < self.to_edge_clearance_meters(required_clearance):
            return False
        return True

    def line_of_sight(self, x0: float, y0: float, x1: float, y1: float, ship_draft: float = 12.0, required_clearance: float = 25.0, step_cells: float = 0.25) -> bool:
        """Continuous raycast checking if swept capsule passes cleanly without touching land or shoals."""
        dx = x1 - x0
        dy = y1 - y0
        dist_cells = math.hypot(dx, dy)
        if dist_cells == 0:
            return self.is_passable(x0, y0, ship_draft, required_clearance)

        steps = max(2, math.ceil(dist_cells / step_cells))
        for s in range(steps + 1):
            t = s / steps
            px = x0 + dx * t
            py = y0 + dy * t
            if not self.is_passable(px, py, ship_draft, required_clearance):
                return False
        return True

    def find_nearest_passable_cell(self, target_x: float, target_y: float, ship_draft: float = 12.0, min_edge_clearance: float = 18.0) -> Dict[str, int]:
        sx = max(0, min(self.cols - 1, int(round(target_x))))
        sy = max(0, min(self.rows - 1, int(round(target_y))))
        for r in range(max(self.cols, self.rows)):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if abs(dx) != r and abs(dy) != r:
                        continue
                    nx, ny = sx + dx, sy + dy
                    if 0 <= nx < self.cols and 0 <= ny < self.rows:
                        if self.cells[ny][nx] != self.TYPE_LAND and self.depths[ny][nx] >= ship_draft:
                            if self.get_exact_hazard_distance_meters(nx, ny, ship_draft) >= min_edge_clearance:
                                return {"x": nx, "y": ny}
        return {"x": sx, "y": sy}

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

    def load_from_json(self, json_path: str, ship_draft: float = 12.0):
        import json
        with open(json_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        self.cells = [[self.TYPE_SEA for _ in range(self.cols)] for _ in range(self.rows)]
        self.depths = [[28.0 for _ in range(self.cols)] for _ in range(self.rows)]
        for y in range(self.rows):
            for x in range(self.cols):
                c_type = data["cells"][y][x]
                c_depth = data["depths"][y][x] if "depths" in data else None
                self.set_cell(x, y, c_type, c_depth)
        if "start" in data:
            self.port_m = {"x": data["start"]["x"], "y": data["start"]["y"], "name": "Port M (Departure)"}
            self.set_cell(self.port_m["x"], self.port_m["y"], self.TYPE_SEA)
        if "destination" in data:
            self.port_n = {"x": data["destination"]["x"], "y": data["destination"]["y"], "name": "Port N (Destination)"}
            self.set_cell(self.port_n["x"], self.port_n["y"], self.TYPE_SEA)
        self.compute_distance_field(ship_draft)

    def load_from_image(self, image_path: str, ship_draft: float = 12.0, sensitivity: float = 0.52):
        """Auto-recognizes obstacles, shoals, and optional green/red ports from an uploaded map image."""
        from PIL import Image
        import numpy as np

        sub = 5
        w, h = self.cols * sub, self.rows * sub
        img = Image.open(image_path).convert("RGB").resize((w, h), Image.Resampling.BILINEAR)
        arr = np.asarray(img, dtype=np.float32)

        r = arr[:, :, 0]
        g = arr[:, :, 1]
        b = arr[:, :, 2]
        lum = 0.299 * r + 0.587 * g + 0.114 * b
        cmax = np.maximum(np.maximum(r, g), b)
        cmin = np.minimum(np.minimum(r, g), b)
        sat = np.where(cmax > 0, (cmax - cmin) / cmax, 0.0)

        blue_frac = float(np.mean((b >= r + 14) & ((b >= g + 2) | ((b - r >= 65) & (b >= g * 0.88) & (lum >= 115))) & (lum >= 45)))
        light_frac = float(np.mean((lum >= 195) & (sat < 0.25)))
        dark_frac = float(np.mean(lum <= 90))

        mode = "maritime_chart" if blue_frac >= 0.18 else ("light_obstacles" if dark_frac > light_frac * 1.25 else "dark_obstacles")
        min_votes = max(5, min(20, round(14 - sensitivity * 9)))
        lum_cut_dark = 115 + (sensitivity - 0.5) * 110
        lum_cut_light = 145 - (sensitivity - 0.5) * 90

        self.cells = [[self.TYPE_SEA for _ in range(self.cols)] for _ in range(self.rows)]
        self.depths = [[28.0 for _ in range(self.cols)] for _ in range(self.rows)]

        green_cells = []
        red_cells = []

        for gy in range(self.rows):
            for gx in range(self.cols):
                y0, y1 = gy * sub, (gy + 1) * sub
                x0, x1 = gx * sub, (gx + 1) * sub
                rb, gb, bb = r[y0:y1, x0:x1], g[y0:y1, x0:x1], b[y0:y1, x0:x1]
                lb, sb = lum[y0:y1, x0:x1], sat[y0:y1, x0:x1]

                is_v_green = (gb >= 125) & ((gb - rb) >= 42) & ((gb - bb) >= 42) & (sb >= 0.35)
                is_v_red = (rb >= 135) & ((rb - gb) >= 45) & ((rb - bb) >= 45) & (sb >= 0.35)
                if int(np.sum(is_v_green)) >= 7:
                    green_cells.append((gx, gy))
                if int(np.sum(is_v_red)) >= 7:
                    red_cells.append((gx, gy))

                is_shoal = (rb >= 180) & (gb >= 145) & (bb >= 80) & (gb > bb + 10) & ((rb - bb) >= 18) & (lb >= 158)
                if mode == "maritime_chart":
                    is_blue_sea = ((bb >= rb + 14) & (bb >= gb + 2) & (lb >= 45)) | ((bb - rb >= 65) & (bb >= gb * 0.88) & (lb >= 115)) | ((lb >= 225) & (sb < 0.16) & (bb >= rb - 6))
                    land_votes = int(np.sum(~is_blue_sea & ~is_shoal & ~is_v_green & ~is_v_red))
                    shoal_votes = int(np.sum(is_shoal & ~is_v_red))
                elif mode == "light_obstacles":
                    land_votes = int(np.sum((lb > lum_cut_light) & ~is_v_green & ~is_v_red))
                    shoal_votes = 0
                else:
                    land_votes = int(np.sum(((lb < lum_cut_dark) | ((sb > 0.28) & (lb < 205))) & ~is_v_green & ~is_v_red))
                    shoal_votes = int(np.sum(is_shoal & ~is_v_red))

                if land_votes >= min_votes:
                    self.set_cell(gx, gy, self.TYPE_LAND, 0.0)
                elif shoal_votes >= min_votes:
                    self.set_cell(gx, gy, self.TYPE_SHALLOW, 7.5)

        if 0 < len(green_cells) <= int(self.cols * self.rows * 0.045):
            mx = round(sum(p[0] for p in green_cells) / len(green_cells))
            my = round(sum(p[1] for p in green_cells) / len(green_cells))
            for px, py in green_cells:
                self.set_cell(px, py, self.TYPE_SEA, 28.0)
            self.compute_distance_field(ship_draft)
            sm = self.find_nearest_passable_cell(mx, my, ship_draft, 10.0)
            self.port_m = {"x": sm["x"], "y": sm["y"], "name": "Port M (Departure)"}
        else:
            self.compute_distance_field(ship_draft)
            sm = self.find_nearest_passable_cell(4, self.rows - 5, ship_draft, 18.0)
            self.port_m = {"x": sm["x"], "y": sm["y"], "name": "Port M (Departure)"}

        if 0 < len(red_cells) <= int(self.cols * self.rows * 0.045):
            nx = round(sum(p[0] for p in red_cells) / len(red_cells))
            ny = round(sum(p[1] for p in red_cells) / len(red_cells))
            for px, py in red_cells:
                self.set_cell(px, py, self.TYPE_SEA, 28.0)
            self.compute_distance_field(ship_draft)
            sn = self.find_nearest_passable_cell(nx, ny, ship_draft, 10.0)
            self.port_n = {"x": sn["x"], "y": sn["y"], "name": "Port N (Destination)"}
        else:
            sn = self.find_nearest_passable_cell(self.cols - 5, 4, ship_draft, 18.0)
            self.port_n = {"x": sn["x"], "y": sn["y"], "name": "Port N (Destination)"}

        self.set_cell(self.port_m["x"], self.port_m["y"], self.TYPE_SEA)
        self.set_cell(self.port_n["x"], self.port_n["y"], self.TYPE_SEA)
        self.compute_distance_field(ship_draft)

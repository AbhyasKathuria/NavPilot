/**
 * NavPilot - Cargo Ship Real-Time Path-Planning Simulator
 * Grid System (grid.js)
 *
 * Implements a 50x50 bathymetric sea map with:
 * - Dynamic water depth levels (deep sea vs shallow shoals vs land)
 * - Ship draft restriction verification
 * - Fast Euclidean Distance Transform (EDT) for obstacle clearance fields
 * - Swept capsule Line-of-Sight (LOS) for continuous navigation checks
 * - Procedural maritime presets (Singapore Strait, Suez approach, Archipelago)
 */

class SeaGrid {
  constructor(cols = 50, rows = 50, cellSizeMeters = 50) {
    this.cols = cols;
    this.rows = rows;
    this.cellSizeMeters = cellSizeMeters; // 1 cell = 50m (2500m x 2500m maritime zone)

    // Cell type constants
    this.TYPE_SEA = 0;      // Deep open navigable water (depth ~ 25m)
    this.TYPE_SHALLOW = 1;  // Shallow reef/shoal (depth ~ 8m)
    this.TYPE_LAND = 2;     // Land/island/rock obstacle (depth = 0m)

    // 2D arrays: grid[y][x]
    this.cells = [];
    this.depths = []; // water depth in meters
    this.distanceField = []; // Euclidean distance in meters to nearest impassable hazard

    this.start = { x: 5, y: 44 };
    this.destination = { x: 44, y: 6 };

    this.init();
    this.loadPreset('archipelago');
  }

  /**
   * Initializes grid matrices
   */
  init() {
    this.cells = [];
    this.depths = [];
    this.distanceField = [];

    for (let y = 0; y < this.rows; y++) {
      const cellRow = [];
      const depthRow = [];
      const distRow = [];
      for (let x = 0; x < this.cols; x++) {
        cellRow.push(this.TYPE_SEA);
        depthRow.push(28.0); // 28m default deep sea
        distRow.push(9999.0);
      }
      this.cells.push(cellRow);
      this.depths.push(depthRow);
      this.distanceField.push(distRow);
    }
  }

  /**
   * Checks whether cell coordinates are within grid boundaries
   */
  inBounds(x, y) {
    return x >= 0 && x < this.cols && y >= 0 && y < this.rows;
  }

  /**
   * Sets cell properties
   */
  setCell(x, y, type, depth = null) {
    if (!this.inBounds(x, y)) return;
    this.cells[y][x] = type;
    if (depth !== null) {
      this.depths[y][x] = depth;
    } else {
      if (type === this.TYPE_LAND) this.depths[y][x] = 0.0;
      else if (type === this.TYPE_SHALLOW) this.depths[y][x] = 8.0;
      else this.depths[y][x] = 28.0;
    }
  }

  /**
   * Toggles obstacle at (x, y)
   */
  toggleObstacle(x, y) {
    if (!this.inBounds(x, y)) return;
    if (this.isStart(x, y) || this.isDestination(x, y)) return;

    if (this.cells[y][x] === this.TYPE_LAND) {
      this.setCell(x, y, this.TYPE_SEA);
    } else {
      this.setCell(x, y, this.TYPE_LAND);
    }
    this.computeDistanceField();
  }

  /**
   * Toggles shallow water at (x, y)
   */
  toggleShallow(x, y) {
    if (!this.inBounds(x, y)) return;
    if (this.isStart(x, y) || this.isDestination(x, y)) return;

    if (this.cells[y][x] === this.TYPE_SHALLOW) {
      this.setCell(x, y, this.TYPE_SEA);
    } else {
      this.setCell(x, y, this.TYPE_SHALLOW, 8.0);
    }
    this.computeDistanceField();
  }

  isStart(x, y) {
    return this.start.x === x && this.start.y === y;
  }

  isDestination(x, y) {
    return this.destination.x === x && this.destination.y === y;
  }

  setStart(x, y) {
    if (!this.inBounds(x, y)) return false;
    this.start = { x, y };
    // Clear obstacle if any
    this.setCell(x, y, this.TYPE_SEA);
    this.computeDistanceField();
    return true;
  }

  setDestination(x, y) {
    if (!this.inBounds(x, y)) return false;
    this.destination = { x, y };
    this.setCell(x, y, this.TYPE_SEA);
    this.computeDistanceField();
    return true;
  }

  /**
   * Computes 2D Euclidean Distance Field (EDT) from every cell to nearest hazard.
   * Hazards are Land cells OR cells where depth < shipDraft (if shipDraft supplied).
   */
  computeDistanceField(shipDraft = 12.0) {
    // 8-direction BFS / brushfire algorithm for accurate distance transform
    const dist = [];
    const queue = [];

    for (let y = 0; y < this.rows; y++) {
      const row = [];
      for (let x = 0; x < this.cols; x++) {
        const isHazard = (this.cells[y][x] === this.TYPE_LAND) ||
                         (this.depths[y][x] < shipDraft);
        if (isHazard) {
          row.push(0.0);
          queue.push({ x, y, d: 0.0 });
        } else {
          row.push(Infinity);
        }
      }
      dist.push(row);
    }

    // Directions with exact Euclidean distance increments
    const neighbors = [
      { dx: 1, dy: 0, d: 1.0 },
      { dx: -1, dy: 0, d: 1.0 },
      { dx: 0, dy: 1, d: 1.0 },
      { dx: 0, dy: -1, d: 1.0 },
      { dx: 1, dy: 1, d: Math.SQRT2 },
      { dx: 1, dy: -1, d: Math.SQRT2 },
      { dx: -1, dy: 1, d: Math.SQRT2 },
      { dx: -1, dy: -1, d: Math.SQRT2 }
    ];

    let head = 0;
    while (head < queue.length) {
      const curr = queue[head++];
      for (let i = 0; i < neighbors.length; i++) {
        const nx = curr.x + neighbors[i].dx;
        const ny = curr.y + neighbors[i].dy;
        if (this.inBounds(nx, ny)) {
          const newDist = curr.d + neighbors[i].d;
          if (newDist < dist[ny][nx]) {
            dist[ny][nx] = newDist;
            queue.push({ x: nx, y: ny, d: newDist });
          }
        }
      }
    }

    // Convert cell distances to real meters
    for (let y = 0; y < this.rows; y++) {
      for (let x = 0; x < this.cols; x++) {
        this.distanceField[y][x] = dist[y][x] * this.cellSizeMeters;
      }
    }
  }

  /**
   * Checks if a point or grid coordinate is passable for a ship
   * with given beam, draft, and safety buffer.
   */
  isPassable(x, y, shipDraft = 12.0, requiredClearanceMeters = 25.0) {
    const gx = Math.round(x);
    const gy = Math.round(y);
    if (!this.inBounds(gx, gy)) return false;

    // Check direct cell obstruction
    if (this.cells[gy][gx] === this.TYPE_LAND) return false;
    if (this.depths[gy][gx] < shipDraft) return false;

    // Check clearance margin around ship hull
    if (this.distanceField[gy][gx] < requiredClearanceMeters) {
      return false;
    }
    return true;
  }

  /**
   * Continuous swept Line-Of-Sight (LOS) test between (x0, y0) and (x1, y1) in grid coordinates.
   * Checks if a ship-sized buffer can traverse cleanly without touching obstacles or shoals.
   */
  lineOfSight(x0, y0, x1, y1, shipDraft = 12.0, requiredClearanceMeters = 25.0) {
    const dx = x1 - x0;
    const dy = y1 - y0;
    const distanceCells = Math.hypot(dx, dy);

    if (distanceCells === 0) {
      return this.isPassable(x0, y0, shipDraft, requiredClearanceMeters);
    }

    // Step size: 0.35 cells ensures no corner leakage or skipped obstacles
    const steps = Math.ceil(distanceCells / 0.35);
    for (let i = 0; i <= steps; i++) {
      const t = i / steps;
      const x = x0 + dx * t;
      const y = y0 + dy * t;
      if (!this.isPassable(x, y, shipDraft, requiredClearanceMeters)) {
        return false;
      }
    }
    return true;
  }

  /**
   * Calculates straight-line distance between Start and Destination in Nautical Miles and km
   */
  getStraightLineDistance() {
    const dx = (this.destination.x - this.start.x) * this.cellSizeMeters;
    const dy = (this.destination.y - this.start.y) * this.cellSizeMeters;
    const meters = Math.hypot(dx, dy);
    const km = meters / 1000;
    const nauticalMiles = meters / 1852;
    return { meters, km, nauticalMiles };
  }

  /**
   * Preset Maps
   */
  loadPreset(name) {
    this.init();

    if (name === 'archipelago') {
      // Natural island chain with navigable straits
      this.start = { x: 4, y: 45 };
      this.destination = { x: 45, y: 4 };

      // Island 1 (Central-West)
      this.drawCircularIsland(14, 28, 6, 3);
      // Island 2 (Central-East)
      this.drawCircularIsland(34, 20, 7, 3);
      // Island 3 (North-West reef)
      this.drawCircularIsland(20, 10, 5, 2);
      // Island 4 (South-East spit)
      this.drawCircularIsland(38, 38, 5, 2);
      // Ridge in center
      for (let x = 22; x <= 28; x++) {
        this.setCell(x, 26, this.TYPE_LAND);
        this.setCell(x, 27, this.TYPE_LAND);
        this.setCell(x, 28, this.TYPE_SHALLOW, 6.0);
        this.setCell(x, 25, this.TYPE_SHALLOW, 7.5);
      }
    } else if (name === 'singapore') {
      // Singapore Strait channel scenario: mainland north, archipelago south, narrow fairway
      this.start = { x: 3, y: 25 };
      this.destination = { x: 46, y: 24 };

      // Northern coast and port shallows
      for (let x = 0; x < this.cols; x++) {
        const landH = Math.floor(10 + Math.sin(x * 0.25) * 3);
        for (let y = 0; y < landH; y++) {
          this.setCell(x, y, this.TYPE_LAND);
        }
        for (let y = landH; y < landH + 3; y++) {
          this.setCell(x, y, this.TYPE_SHALLOW, 7.0);
        }
      }

      // Southern islands and shallow banks
      for (let x = 0; x < this.cols; x++) {
        const islandTop = Math.floor(38 - Math.sin(x * 0.2) * 4);
        for (let y = islandTop; y < this.rows; y++) {
          this.setCell(x, y, this.TYPE_LAND);
        }
        for (let y = islandTop - 3; y < islandTop; y++) {
          this.setCell(x, y, this.TYPE_SHALLOW, 7.5);
        }
      }

      // Channel navigation hazards (shoal bank in mid-channel)
      this.drawCircularIsland(25, 22, 3, 2);
      this.drawCircularIsland(15, 29, 2.5, 2);
      this.drawCircularIsland(36, 18, 3, 2);
    } else if (name === 'ice_alley') {
      // Scattered iceberg fields and narrow navigation corridors
      this.start = { x: 4, y: 44 };
      this.destination = { x: 44, y: 5 };

      // Cluster 1
      this.drawCircularIsland(12, 36, 4, 2);
      this.drawCircularIsland(18, 32, 3, 2);
      // Cluster 2
      this.drawCircularIsland(25, 25, 5, 2);
      this.drawCircularIsland(22, 18, 4, 2);
      this.drawCircularIsland(30, 20, 3, 2);
      // Cluster 3
      this.drawCircularIsland(36, 12, 4, 2);
      this.drawCircularIsland(42, 18, 3, 2);
      // Shallow sandbars
      for (let y = 10; y <= 40; y += 4) {
        this.setCell(10, y, this.TYPE_SHALLOW, 7.0);
        this.setCell(38, y, this.TYPE_SHALLOW, 7.0);
      }
    } else if (name === 'canal') {
      // Approach to a narrow dredged canal
      this.start = { x: 5, y: 42 };
      this.destination = { x: 42, y: 8 };

      // Massive breakwater barriers creating a funnel
      for (let x = 0; x < 35; x++) {
        this.setCell(x, 20, this.TYPE_LAND);
        this.setCell(x, 19, this.TYPE_SHALLOW, 6.0);
      }
      for (let x = 15; x < this.cols; x++) {
        this.setCell(x, 30, this.TYPE_LAND);
        this.setCell(x, 31, this.TYPE_SHALLOW, 6.0);
      }
      // Shoal island in outer approach
      this.drawCircularIsland(12, 36, 3, 2);
      this.drawCircularIsland(38, 14, 3, 2);
    } else {
      // Open Sea with random small hazards
      this.generateRandomMap();
      return;
    }

    // Keep start and dest clear
    this.setCell(this.start.x, this.start.y, this.TYPE_SEA);
    this.setCell(this.destination.x, this.destination.y, this.TYPE_SEA);

    this.computeDistanceField();
  }

  /**
   * Helper to draw organic circular island with surrounding shallow reef
   */
  drawCircularIsland(cx, cy, radiusLand, radiusShallow) {
    const totalR = radiusLand + radiusShallow;
    for (let dy = -totalR; dy <= totalR; dy++) {
      for (let dx = -totalR; dx <= totalR; dx++) {
        const x = Math.round(cx + dx);
        const y = Math.round(cy + dy);
        if (!this.inBounds(x, y)) continue;

        const d = Math.hypot(dx, dy);
        if (d <= radiusLand) {
          this.cells[y][x] = this.TYPE_LAND;
          this.depths[y][x] = 0.0;
        } else if (d <= totalR && this.cells[y][x] !== this.TYPE_LAND) {
          this.cells[y][x] = this.TYPE_SHALLOW;
          this.depths[y][x] = 7.5; // Shallow shoal
        }
      }
    }
  }

  /**
   * Generates randomized archipelago
   */
  generateRandomMap() {
    this.init();
    this.start = { x: 4, y: 44 };
    this.destination = { x: 44, y: 5 };

    const islandCount = 5 + Math.floor(Math.random() * 4);
    for (let i = 0; i < islandCount; i++) {
      const cx = 10 + Math.random() * 30;
      const cy = 10 + Math.random() * 30;
      // Keep clear of start and dest
      if (Math.hypot(cx - this.start.x, cy - this.start.y) < 9 ||
          Math.hypot(cx - this.destination.x, cy - this.destination.y) < 9) {
        continue;
      }
      const rLand = 2.5 + Math.random() * 3.5;
      const rShallow = 1.5 + Math.random() * 2.0;
      this.drawCircularIsland(cx, cy, rLand, rShallow);
    }

    // Ensure start & dest are sea
    for (let dy = -2; dy <= 2; dy++) {
      for (let dx = -2; dx <= 2; dx++) {
        this.setCell(this.start.x + dx, this.start.y + dy, this.TYPE_SEA);
        this.setCell(this.destination.x + dx, this.destination.y + dy, this.TYPE_SEA);
      }
    }

    this.computeDistanceField();
  }
}

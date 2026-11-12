/**
 * NavPilot - Cargo Ship Real-Time Path-Planning Simulator
 * Intelligent Path Map Image & JSON Auto-Recognizer (mapImporter.js)
 *
 * Parses uploaded map images (nautical charts, sketches, occupancy maps, screenshots)
 * completely offline in vanilla HTML5 Canvas 2D:
 * - Auto-detects sea vs land obstacles vs shallow shoal zones
 * - Supports Dark-on-Light, Light-on-Dark, and Maritime Color Chart heuristics
 * - Automatically locates green Start (Port M) & red Destination (Port N) markers
 * - Live threshold calibration & visual side-by-side preview
 */

class MapImageImporter {
  constructor() {
    this.loadedImage = null;
    this.loadedFileName = '';
    this.sampleSize = 250; // 50x50 cells * 5x5 subpixels per cell
    this.offCanvas = document.createElement('canvas');
    this.offCanvas.width = this.sampleSize;
    this.offCanvas.height = this.sampleSize;
    this.offCtx = this.offCanvas.getContext('2d', { willReadFrequently: true });
    this.detectedStyle = 'auto';
  }

  hasImage() {
    return this.loadedImage !== null;
  }

  /**
   * Loads an uploaded File object (Image or JSON map).
   */
  loadFile(file, onImageLoaded, onJsonLoaded, onError) {
    if (!file) return;
    this.loadedFileName = file.name || 'uploaded_map';

    if (file.name.toLowerCase().endsWith('.json') || file.type === 'application/json') {
      const reader = new FileReader();
      reader.onload = (e) => {
        try {
          const parsed = JSON.parse(e.target.result);
          if (onJsonLoaded) onJsonLoaded(parsed, this.loadedFileName);
        } catch (err) {
          if (onError) onError('Invalid JSON map file: ' + err.message);
        }
      };
      reader.readAsText(file);
      return;
    }

    if (!file.type.startsWith('image/')) {
      if (onError) onError('Unsupported file format. Please upload PNG, JPG, WebP, SVG, BMP, or JSON.');
      return;
    }

    const reader = new FileReader();
    reader.onload = (e) => {
      const img = new Image();
      img.onload = () => {
        this.loadedImage = img;
        if (onImageLoaded) onImageLoaded(img, this.loadedFileName);
      };
      img.onerror = () => {
        if (onError) onError('Failed to decode uploaded image.');
      };
      img.src = e.target.result;
    };
    reader.readAsDataURL(file);
  }

  /**
   * Computes pixel HSV & Luminance helper
   */
  rgbToStats(r, g, b) {
    const max = Math.max(r, g, b);
    const min = Math.min(r, g, b);
    const d = max - min;
    const sat = max === 0 ? 0 : d / max;
    const lum = 0.299 * r + 0.587 * g + 0.114 * b;
    return { r, g, b, lum, sat, max, min };
  }

  /**
   * Processes the loaded image into a 50x50 SeaGrid matrix & returns recognition summary.
   */
  recognizeAndApply(grid, shipDraft = 12.0, options = {}) {
    if (!this.loadedImage) return null;

    const cols = grid.cols;
    const rows = grid.rows;
    const sub = 5; // 5x5 subpixels per cell -> 250x250 canvas
    const width = cols * sub;
    const height = rows * sub;

    this.offCanvas.width = width;
    this.offCanvas.height = height;
    this.offCtx.clearRect(0, 0, width, height);
    this.offCtx.drawImage(this.loadedImage, 0, 0, width, height);

    const imgData = this.offCtx.getImageData(0, 0, width, height).data;

    const mode = options.mode || 'auto';
    const sensitivity = options.sensitivity !== undefined ? Number(options.sensitivity) : 0.52;
    const invert = Boolean(options.invert);
    const detectPorts = options.detectPorts !== undefined ? Boolean(options.detectPorts) : true;
    const detectShoals = options.detectShoals !== undefined ? Boolean(options.detectShoals) : true;

    // Pass 1: Global histogram to infer dominant water tone when in 'auto' mode
    let effectiveMode = mode;
    let blueCount = 0;
    let lightCount = 0;
    let darkCount = 0;
    const totalSamples = width * height;

    for (let i = 0; i < imgData.length; i += 4) {
      const a = imgData[i + 3];
      if (a < 25) {
        lightCount++;
        continue;
      }
      const r = imgData[i], g = imgData[i + 1], b = imgData[i + 2];
      const lum = 0.299 * r + 0.587 * g + 0.114 * b;
      const sat = Math.max(r, g, b) > 0 ? (Math.max(r, g, b) - Math.min(r, g, b)) / Math.max(r, g, b) : 0;

      if (b >= r + 14 && (b >= g + 2 || (b - r >= 65 && b >= g * 0.88 && lum >= 115)) && lum >= 45) {
        blueCount++;
      } else if (lum >= 195 && sat < 0.25) {
        lightCount++;
      } else if (lum <= 90) {
        darkCount++;
      }
    }

    if (mode === 'auto') {
      if (blueCount / totalSamples >= 0.18) {
        effectiveMode = 'maritime_chart';
      } else if (darkCount > lightCount * 1.25) {
        effectiveMode = 'light_obstacles'; // Dark background is water, bright shapes are walls/land
      } else {
        effectiveMode = 'dark_obstacles'; // White/light background is water, dark/colored shapes are walls/land
      }
    }
    this.detectedStyle = effectiveMode;

    // Dynamic thresholds tuned by sensitivity (0.20 .. 0.90)
    const lumCutoffDark = 115 + (sensitivity - 0.5) * 110; // Higher sensitivity catches lighter grey obstacles
    const lumCutoffLight = 145 - (sensitivity - 0.5) * 90;
    const minSubpixelVotes = Math.max(5, Math.min(20, Math.round(14 - sensitivity * 9)));

    const cellTypes = Array.from({ length: rows }, () => new Array(cols).fill(grid.TYPE_SEA));
    const greenCells = [];
    const redCells = [];

    // Pass 2: Evaluate each cell's 5x5 pixel footprint
    for (let gy = 0; gy < rows; gy++) {
      for (let gx = 0; gx < cols; gx++) {
        let landVotes = 0;
        let shoalVotes = 0;
        let greenMarkerVotes = 0;
        let redMarkerVotes = 0;
        let waterVotes = 0;

        for (let sy = 0; sy < sub; sy++) {
          for (let sx = 0; sx < sub; sx++) {
            const px = gx * sub + sx;
            const py = gy * sub + sy;
            const idx = (py * width + px) * 4;
            const r = imgData[idx];
            const g = imgData[idx + 1];
            const b = imgData[idx + 2];
            const a = imgData[idx + 3];

            if (a < 25) {
              waterVotes++;
              continue;
            }

            const stats = this.rgbToStats(r, g, b);

            // Detect distinct bright green / red markers (Port M & Port N)
            const isVividGreen = (g >= 125 && g - r >= 42 && g - b >= 42 && stats.sat >= 0.35);
            const isVividRed = (r >= 135 && r - g >= 45 && r - b >= 45 && stats.sat >= 0.35);

            if (isVividGreen) greenMarkerVotes++;
            if (isVividRed) redMarkerVotes++;

            let isLandPixel = false;
            let isShoalPixel = false;

            if (effectiveMode === 'maritime_chart') {
              // Water = azure/ocean blue (B > G & B - R >= 14), strong turquoise (B - R >= 65), or crisp white sea
              const isBlueSea = (b >= r + 14 && b >= g + 2 && stats.lum >= 45) ||
                                (b - r >= 65 && b >= g * 0.88 && stats.lum >= 115) ||
                                (stats.lum >= 225 && stats.sat < 0.16 && b >= r - 6);
              // Shoal = warm sand / peach / tan / pale yellow beach border
              const isSandShoal = (r >= 180 && g >= 145 && b >= 80 && g > b + 10 && (r - b) >= 18 && stats.lum >= 158);

              if (isSandShoal && !isVividRed) {
                isShoalPixel = true;
              } else if (!isBlueSea && !isVividGreen && !isVividRed) {
                // Dark rocks, grey/slate islands, forest green landmasses, brown continents
                if (stats.lum < 215 || (g > b) || (r > b + 12)) {
                  isLandPixel = true;
                }
              }
            } else if (effectiveMode === 'light_obstacles') {
              // Bright walls / bright landmasses on dark water background
              const isShoal = detectShoals && (r > 170 && g > 130 && b < 110 && stats.lum < 210);
              if (isShoal) isShoalPixel = true;
              else if (stats.lum > lumCutoffLight) isLandPixel = true;
            } else {
              // 'dark_obstacles': White/light canvas = Sea, dark/saturated shapes = Obstacles
              const isShoal = detectShoals && (r >= 185 && g >= 150 && b >= 80 && g > b + 15 && stats.lum >= 160);
              if (isShoal && !isVividRed) {
                isShoalPixel = true;
              } else if (stats.lum < lumCutoffDark || (stats.sat > 0.28 && stats.lum < 205)) {
                isLandPixel = true;
              }
            }

            if (isLandPixel) landVotes++;
            else if (isShoalPixel) shoalVotes++;
            else waterVotes++;
          }
        }

        if (greenMarkerVotes >= 7) {
          greenCells.push({ x: gx, y: gy });
        }
        if (redMarkerVotes >= 7) {
          redCells.push({ x: gx, y: gy });
        }

        let cellType = grid.TYPE_SEA;
        if (landVotes >= minSubpixelVotes) {
          cellType = grid.TYPE_LAND;
        } else if (detectShoals && shoalVotes >= minSubpixelVotes) {
          cellType = grid.TYPE_SHALLOW;
        }

        if (invert) {
          cellType = (cellType === grid.TYPE_LAND) ? grid.TYPE_SEA : (cellType === grid.TYPE_SEA ? grid.TYPE_LAND : cellType);
        }

        cellTypes[gy][gx] = cellType;
      }
    }

    // Apply classified cells into grid
    grid.init();
    for (let y = 0; y < rows; y++) {
      for (let x = 0; x < cols; x++) {
        const t = cellTypes[y][x];
        if (t === grid.TYPE_LAND) {
          grid.setCell(x, y, grid.TYPE_LAND, 0.0);
        } else if (t === grid.TYPE_SHALLOW) {
          grid.setCell(x, y, grid.TYPE_SHALLOW, 7.5);
        } else {
          grid.setCell(x, y, grid.TYPE_SEA, 28.0);
        }
      }
    }

    // Determine Start (Port M) & Destination (Port N)
    let detectedM = null;
    let detectedN = null;

    // Only treat vivid green/red blobs as port pins if they are compact marker pins (< 4.5% of total grid)
    if (detectPorts) {
      if (greenCells.length > 0 && greenCells.length <= Math.floor(cols * rows * 0.045)) {
        const cx = Math.round(greenCells.reduce((s, p) => s + p.x, 0) / greenCells.length);
        const cy = Math.round(greenCells.reduce((s, p) => s + p.y, 0) / greenCells.length);
        detectedM = { x: cx, y: cy };
        // Clear the marker blob from obstacle layer so vessel can spawn at the port
        greenCells.forEach(p => grid.setCell(p.x, p.y, grid.TYPE_SEA, 28.0));
      }

      if (redCells.length > 0 && redCells.length <= Math.floor(cols * rows * 0.045)) {
        const cx = Math.round(redCells.reduce((s, p) => s + p.x, 0) / redCells.length);
        const cy = Math.round(redCells.reduce((s, p) => s + p.y, 0) / redCells.length);
        detectedN = { x: cx, y: cy };
        redCells.forEach(p => grid.setCell(p.x, p.y, grid.TYPE_SEA, 28.0));
      }
    }

    grid.computeDistanceField(shipDraft);

    if (detectedM) {
      const safeM = grid.findNearestPassableCell(detectedM.x, detectedM.y, shipDraft, 10.0);
      grid.setStart(safeM.x, safeM.y);
    } else {
      const safeM = grid.findNearestPassableCell(4, rows - 5, shipDraft, 18.0);
      grid.setStart(safeM.x, safeM.y);
    }

    if (detectedN) {
      const safeN = grid.findNearestPassableCell(detectedN.x, detectedN.y, shipDraft, 10.0);
      grid.setDestination(safeN.x, safeN.y);
    } else {
      const safeN = grid.findNearestPassableCell(cols - 5, 4, shipDraft, 18.0);
      grid.setDestination(safeN.x, safeN.y);
    }

    grid.computeDistanceField(shipDraft);

    // Collect recognition statistics
    let landCount = 0;
    let shoalCount = 0;
    let seaCount = 0;
    for (let y = 0; y < rows; y++) {
      for (let x = 0; x < cols; x++) {
        if (grid.cells[y][x] === grid.TYPE_LAND) landCount++;
        else if (grid.cells[y][x] === grid.TYPE_SHALLOW) shoalCount++;
        else seaCount++;
      }
    }

    return {
      fileName: this.loadedFileName,
      detectedStyle: effectiveMode,
      landCount,
      shoalCount,
      seaCount,
      portsDetected: {
        start: Boolean(detectedM),
        destination: Boolean(detectedN)
      },
      start: { ...grid.start },
      destination: { ...grid.destination }
    };
  }

  /**
   * Renders a crisp miniature preview of the recognized 50x50 grid into a target preview <canvas>
   */
  renderPreview(previewCanvas, grid) {
    if (!previewCanvas) return;
    const ctx = previewCanvas.getContext('2d');
    const w = previewCanvas.width;
    const h = previewCanvas.height;
    const cw = w / grid.cols;
    const ch = h / grid.rows;

    ctx.clearRect(0, 0, w, h);
    for (let y = 0; y < grid.rows; y++) {
      for (let x = 0; x < grid.cols; x++) {
        const t = grid.cells[y][x];
        if (t === grid.TYPE_LAND) ctx.fillStyle = '#334155';
        else if (t === grid.TYPE_SHALLOW) ctx.fillStyle = '#FFEED9';
        else ctx.fillStyle = '#E0F4FF';
        ctx.fillRect(x * cw, y * ch, Math.ceil(cw), Math.ceil(ch));
      }
    }

    // Highlight Start M & Dest N on thumbnail
    ctx.fillStyle = '#16A34A';
    ctx.beginPath();
    ctx.arc((grid.start.x + 0.5) * cw, (grid.start.y + 0.5) * ch, Math.max(3, cw * 1.4), 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = '#DC2626';
    ctx.beginPath();
    ctx.arc((grid.destination.x + 0.5) * cw, (grid.destination.y + 0.5) * ch, Math.max(3, cw * 1.4), 0, Math.PI * 2);
    ctx.fill();
  }
}

/**
 * NavPilot - Cargo Ship Real-Time Path-Planning Simulator
 * Canvas Maritime Renderer (renderer.js)
 *
 * Implements 500x500 high-DPI canvas graphics:
 * - Bathymetric depth shading (deep sea, navigable fairway, shallow reef, land)
 * - Start/Destination flags with coordinates & range vector
 * - Search wavefront (explored nodes animation)
 * - Path A (original, dashed when replanned) vs Path B (active avoidance)
 * - Scaled cargo ship hull with heading, rudder, wake, swept footprint & safety buffer
 * - Dynamic obstacles with velocity vectors and predicted trajectory tracks
 * - REPLANNING, HOLD & WAIT, and ARRIVAL banners
 * - Palette adherence to #FFCED8, #FFDDDD, #FFF4DE, #FFE8C4
 */

class MaritimeRenderer {
  constructor(canvas, grid, ship, obstacleManager) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.grid = grid;
    this.ship = ship;
    this.obstacleManager = obstacleManager;

    this.cellSize = 10; // 500px / 50 cells = 10px per cell
    this.themeMode = 'strict'; // 'strict' (using user's 4 colors) or 'nautical'

    // Explored nodes animation state
    this.exploredAnimationQueue = [];
    this.animatedExploredIndex = 0;
    this.isAnimatingExploration = false;

    // Coordinate hover HUD
    this.hoverCoord = null;

    this.setupHighDPI();
  }

  setupHighDPI() {
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = 500 * dpr;
    this.canvas.height = 500 * dpr;
    this.ctx.scale(dpr, dpr);
    this.canvas.style.width = '500px';
    this.canvas.style.height = '500px';
  }

  setTheme(theme) {
    this.themeMode = theme;
  }

  setHoverCoord(x, y) {
    this.hoverCoord = (x >= 0 && x < this.grid.cols && y >= 0 && y < this.grid.rows) ? { x, y } : null;
  }

  startExplorationAnimation(exploredNodes, onComplete) {
    this.exploredAnimationQueue = exploredNodes || [];
    this.animatedExploredIndex = 0;
    this.isAnimatingExploration = true;
    this.onExplorationComplete = onComplete;
  }

  render(deltaTime = 0.016) {
    const ctx = this.ctx;
    ctx.clearRect(0, 0, 500, 500);

    // 1. Draw Bathymetric Grid & Static Obstacles
    this.drawBathymetricGrid(ctx);

    // 2. Draw Distance / Straight-Line Vector between Start & Dest
    this.drawStartDestVector(ctx);

    // 3. Draw Explored Nodes (Frontier wavefront)
    this.drawExploredNodes(ctx);

    // 4. Draw Planned Routes (Path A & Path B)
    this.drawPaths(ctx);

    // 5. Draw Dynamic Obstacles & Forward Trajectories
    this.drawDynamicObstacles(ctx);

    // 6. Draw Ship Hull, Heading, Rudder, Swept Footprint & Safety Buffer
    this.drawShip(ctx);

    // 7. Draw Start & Destination Markers (Flags + Coordinates)
    this.drawFlags(ctx);

    // 8. Draw Alert Banners (Replanning, Emergency Stop, Arrived)
    this.drawBanners(ctx);

    // 9. Draw Mouse Hover HUD
    this.drawHoverHUD(ctx);
  }

  /**
   * Draws bathymetric sea map with depth contours
   */
  drawBathymetricGrid(ctx) {
    const isStrict = this.themeMode === 'strict';

    for (let y = 0; y < this.grid.rows; y++) {
      for (let x = 0; x < this.grid.cols; x++) {
        const type = this.grid.cells[y][x];
        const depth = this.grid.depths[y][x];
        const px = x * this.cellSize;
        const py = y * this.cellSize;

        if (type === this.grid.TYPE_LAND) {
          // Land / Island obstacle
          ctx.fillStyle = isStrict ? '#7C5865' : '#475569';
          ctx.fillRect(px, py, this.cellSize, this.cellSize);

          // Rocky texture highlight
          ctx.fillStyle = isStrict ? '#FFCED8' : '#64748B';
          ctx.fillRect(px + 1, py + 1, this.cellSize - 2, 2);
        } else if (type === this.grid.TYPE_SHALLOW || depth < this.ship.draft) {
          // Shallow reef/shoal impassable for current vessel draft
          ctx.fillStyle = isStrict ? '#FFE8C4' : '#64A5B5';
          ctx.fillRect(px, py, this.cellSize, this.cellSize);

          // Stipple reef texture
          ctx.fillStyle = isStrict ? '#E2B888' : '#4E8A98';
          ctx.fillRect(px + 3, py + 3, 2, 2);
          ctx.fillRect(px + 7, py + 6, 2, 2);
        } else {
          // Open Sea / Navigable Fairway
          ctx.fillStyle = isStrict ? '#FFF4DE' : '#1D3557';
          ctx.fillRect(px, py, this.cellSize, this.cellSize);
        }
      }
    }

    // Grid mesh lines
    ctx.strokeStyle = isStrict ? 'rgba(215, 185, 175, 0.28)' : 'rgba(255, 255, 255, 0.06)';
    ctx.lineWidth = 0.5;
    ctx.beginPath();
    for (let i = 0; i <= 50; i++) {
      ctx.moveTo(i * this.cellSize, 0);
      ctx.lineTo(i * this.cellSize, 500);
      ctx.moveTo(0, i * this.cellSize);
      ctx.lineTo(500, i * this.cellSize);
    }
    ctx.stroke();
  }

  /**
   * Straight-line distance range vector between Start and Destination
   */
  drawStartDestVector(ctx) {
    const sx = (this.grid.start.x + 0.5) * this.cellSize;
    const sy = (this.grid.start.y + 0.5) * this.cellSize;
    const dx = (this.grid.destination.x + 0.5) * this.cellSize;
    const dy = (this.grid.destination.y + 0.5) * this.cellSize;

    ctx.save();
    ctx.setLineDash([4, 4]);
    ctx.strokeStyle = 'rgba(120, 90, 80, 0.4)';
    ctx.lineWidth = 1.0;
    ctx.beginPath();
    ctx.moveTo(sx, sy);
    ctx.lineTo(dx, dy);
    ctx.stroke();
    ctx.restore();

    // Range distance label along mid-line
    const midX = (sx + dx) / 2;
    const midY = (sy + dy) / 2;
    const distInfo = this.grid.getStraightLineDistance();

    ctx.save();
    ctx.fillStyle = 'rgba(255, 244, 222, 0.9)';
    ctx.strokeStyle = '#FFCED8';
    ctx.lineWidth = 1;
    const labelText = `Range: ${distInfo.nauticalMiles.toFixed(2)} NM (${distInfo.km.toFixed(2)} km)`;
    ctx.font = '10px "Inter", sans-serif';
    const textWidth = ctx.measureText(labelText).width;

    ctx.fillRect(midX - textWidth / 2 - 4, midY - 8, textWidth + 8, 16);
    ctx.strokeRect(midX - textWidth / 2 - 4, midY - 8, textWidth + 8, 16);

    ctx.fillStyle = '#2A1A1E';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(labelText, midX, midY);
    ctx.restore();
  }

  /**
   * Draws faint dots of explored nodes (search wavefront)
   */
  drawExploredNodes(ctx) {
    if (!this.exploredAnimationQueue || this.exploredAnimationQueue.length === 0) return;

    if (this.isAnimatingExploration) {
      // Advance animation batch
      this.animatedExploredIndex = Math.min(
        this.exploredAnimationQueue.length,
        this.animatedExploredIndex + 25
      );
      if (this.animatedExploredIndex >= this.exploredAnimationQueue.length) {
        this.isAnimatingExploration = false;
        if (this.onExplorationComplete) this.onExplorationComplete();
      }
    }

    const count = this.isAnimatingExploration ? this.animatedExploredIndex : this.exploredAnimationQueue.length;
    ctx.fillStyle = 'rgba(217, 83, 111, 0.35)'; // faint glowing dot

    for (let i = 0; i < count; i++) {
      const node = this.exploredAnimationQueue[i];
      const cx = (node.x + 0.5) * this.cellSize;
      const cy = (node.y + 0.5) * this.cellSize;

      ctx.beginPath();
      ctx.arc(cx, cy, 1.8, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  /**
   * Draws Path A (original) and Path B (replan)
   */
  drawPaths(ctx) {
    // 1. Draw Original Path (Path A)
    // If ship has replanned, draw Path A as dashed muted line
    if (this.ship.originalPath && this.ship.originalPath.length > 1) {
      ctx.save();
      if (this.ship.hasReplanned) {
        ctx.setLineDash([5, 5]);
        ctx.strokeStyle = 'rgba(140, 110, 115, 0.6)';
        ctx.lineWidth = 2.0;
      } else {
        ctx.strokeStyle = '#D9536F'; // bright vivid line
        ctx.lineWidth = 3.0;
        ctx.shadowColor = 'rgba(217, 83, 111, 0.4)';
        ctx.shadowBlur = 6;
      }

      ctx.beginPath();
      ctx.moveTo(
        (this.ship.originalPath[0].x + 0.5) * this.cellSize,
        (this.ship.originalPath[0].y + 0.5) * this.cellSize
      );
      for (let i = 1; i < this.ship.originalPath.length; i++) {
        ctx.lineTo(
          (this.ship.originalPath[i].x + 0.5) * this.cellSize,
          (this.ship.originalPath[i].y + 0.5) * this.cellSize
        );
      }
      ctx.stroke();
      ctx.restore();
    }

    // 2. Draw Replanned Path (Path B) if active
    if (this.ship.hasReplanned && this.ship.replanPath && this.ship.replanPath.length > 1) {
      ctx.save();
      ctx.strokeStyle = '#2A9D8F'; // distinct bright turquoise / green avoidance path
      ctx.lineWidth = 3.2;
      ctx.shadowColor = 'rgba(42, 157, 143, 0.5)';
      ctx.shadowBlur = 8;

      ctx.beginPath();
      ctx.moveTo(
        (this.ship.replanPath[0].x + 0.5) * this.cellSize,
        (this.ship.replanPath[0].y + 0.5) * this.cellSize
      );
      for (let i = 1; i < this.ship.replanPath.length; i++) {
        ctx.lineTo(
          (this.ship.replanPath[i].x + 0.5) * this.cellSize,
          (this.ship.replanPath[i].y + 0.5) * this.cellSize
        );
      }
      ctx.stroke();
      ctx.restore();
    }
  }

  /**
   * Draws ship hull, heading vector, rudder angle, wake and safety buffer
   */
  drawShip(ctx) {
    const ship = this.ship;
    const px = (ship.x + 0.5) * this.cellSize;
    const py = (ship.y + 0.5) * this.cellSize;

    // 1. Draw Wake Trail
    if (ship.trail && ship.trail.length > 1) {
      ctx.save();
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.4)';
      ctx.lineWidth = 2.0;
      ctx.beginPath();
      for (let i = 0; i < ship.trail.length; i++) {
        const tx = (ship.trail[i].x + 0.5) * this.cellSize;
        const ty = (ship.trail[i].y + 0.5) * this.cellSize;
        if (i === 0) ctx.moveTo(tx, ty);
        else ctx.lineTo(tx, ty);
      }
      ctx.stroke();
      ctx.restore();
    }

    // 2. Draw Dynamic Safety Buffer (Capsule / Ellipse around ship)
    const clearanceMeters = ship.computeRequiredClearanceMeters();
    const clearanceCells = clearanceMeters / 50.0;
    const bufferRadiusPx = clearanceCells * this.cellSize;

    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(ship.heading);

    // Safety buffer fill & stroke (#FFDDDD and #FFCED8)
    ctx.fillStyle = 'rgba(255, 221, 221, 0.28)';
    ctx.strokeStyle = '#FFCED8';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([3, 3]);

    const halfLenPx = ((ship.length / 2.0) / 50.0) * this.cellSize;
    const halfBmPx = ((ship.beam / 2.0) / 50.0) * this.cellSize;

    // Draw safety capsule
    ctx.beginPath();
    ctx.ellipse(0, 0, halfLenPx + bufferRadiusPx, halfBmPx + bufferRadiusPx, 0, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // 3. Draw Swept Hull Footprint (Oriented Bounding Box)
    ctx.setLineDash([]);
    ctx.fillStyle = '#2C1810'; // Deep dark ship hull
    ctx.strokeStyle = '#FFCED8';
    ctx.lineWidth = 1.2;

    // Draw pointed cargo ship hull
    ctx.beginPath();
    ctx.moveTo(halfLenPx * 1.15, 0); // Bow tip
    ctx.lineTo(halfLenPx * 0.8, halfBmPx);
    ctx.lineTo(-halfLenPx, halfBmPx);
    ctx.lineTo(-halfLenPx, -halfBmPx);
    ctx.lineTo(halfLenPx * 0.8, -halfBmPx);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // Cargo container stacks (colorful bays)
    ctx.fillStyle = '#FFE8C4';
    ctx.fillRect(-halfLenPx * 0.5, -halfBmPx * 0.7, halfLenPx * 0.9, halfBmPx * 1.4);

    // Navigation Bridge / Deckhouse (at stern)
    ctx.fillStyle = '#FFFFFF';
    ctx.fillRect(-halfLenPx * 0.85, -halfBmPx * 0.5, halfLenPx * 0.25, halfBmPx * 1.0);

    // 4. Rudder angle indicator at stern
    ctx.strokeStyle = '#D9536F';
    ctx.lineWidth = 2.0;
    ctx.beginPath();
    ctx.moveTo(-halfLenPx, 0);
    const rudderLen = halfBmPx * 1.1;
    ctx.lineTo(-halfLenPx - Math.cos(ship.rudderAngle) * rudderLen, -Math.sin(ship.rudderAngle) * rudderLen);
    ctx.stroke();

    // 5. Heading vector pointer (bow arrow)
    ctx.strokeStyle = '#2A9D8F';
    ctx.lineWidth = 2.2;
    ctx.beginPath();
    ctx.moveTo(halfLenPx * 1.15, 0);
    ctx.lineTo(halfLenPx * 1.15 + 20, 0);
    ctx.stroke();

    // Turning radius arc indicator
    if (Math.abs(ship.rudderAngle) > 0.05) {
      const turnRadiusPx = (ship.minTurningRadius / 50.0) * this.cellSize;
      ctx.strokeStyle = 'rgba(217, 83, 111, 0.45)';
      ctx.lineWidth = 1.0;
      ctx.setLineDash([2, 2]);
      ctx.beginPath();
      const arcCenterY = ship.rudderAngle > 0 ? turnRadiusPx : -turnRadiusPx;
      ctx.arc(0, arcCenterY, turnRadiusPx, 0, Math.PI * 2);
      ctx.stroke();
    }

    ctx.restore();
  }

  /**
   * Draws dynamic obstacles with predicted future trajectories
   */
  drawDynamicObstacles(ctx) {
    const obstacles = this.obstacleManager.obstacles;

    for (let i = 0; i < obstacles.length; i++) {
      const obs = obstacles[i];
      const px = (obs.x + 0.5) * this.cellSize;
      const py = (obs.y + 0.5) * this.cellSize;
      const rPx = (obs.radiusMeters / 50.0) * this.cellSize;

      // 1. Draw Predicted Trajectory (Forward Lookahead Horizon Track)
      ctx.save();
      ctx.setLineDash([3, 3]);
      ctx.strokeStyle = 'rgba(224, 90, 109, 0.6)';
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(px, py);
      for (let t = 1; t <= 15; t += 2) {
        const pred = obs.predictPosition(t);
        ctx.lineTo((pred.x + 0.5) * this.cellSize, (pred.y + 0.5) * this.cellSize);
      }
      ctx.stroke();
      ctx.restore();

      // 2. Draw Obstacle Body
      ctx.save();
      ctx.translate(px, py);

      if (obs.type === 'vessel') {
        // Dynamic ship
        ctx.rotate(obs.heading);
        ctx.fillStyle = '#E05A6D'; // red/coral vessel
        ctx.strokeStyle = '#2C1810';
        ctx.lineWidth = 1.2;

        ctx.beginPath();
        ctx.moveTo(rPx * 1.2, 0);
        ctx.lineTo(rPx * 0.7, rPx * 0.6);
        ctx.lineTo(-rPx * 0.9, rPx * 0.6);
        ctx.lineTo(-rPx * 0.9, -rPx * 0.6);
        ctx.lineTo(rPx * 0.7, -rPx * 0.6);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();

        // Velocity vector arrow
        ctx.strokeStyle = '#FFFFFF';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(rPx * 1.8, 0);
        ctx.stroke();
      } else if (obs.type === 'iceberg') {
        // Jagged drifting iceberg
        ctx.fillStyle = '#E8F4F8';
        ctx.strokeStyle = '#64A5B5';
        ctx.lineWidth = 1.5;

        ctx.beginPath();
        ctx.moveTo(0, -rPx);
        ctx.lineTo(rPx * 0.9, -rPx * 0.4);
        ctx.lineTo(rPx * 0.8, rPx * 0.8);
        ctx.lineTo(-rPx * 0.3, rPx);
        ctx.lineTo(-rPx * 0.9, rPx * 0.2);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
      } else {
        // Drifting cargo container / debris
        ctx.fillStyle = '#F4A261';
        ctx.strokeStyle = '#2C1810';
        ctx.lineWidth = 1.0;
        ctx.fillRect(-rPx * 0.7, -rPx * 0.4, rPx * 1.4, rPx * 0.8);
        ctx.strokeRect(-rPx * 0.7, -rPx * 0.4, rPx * 1.4, rPx * 0.8);
      }

      // Safety boundary circle
      ctx.strokeStyle = 'rgba(224, 90, 109, 0.4)';
      ctx.lineWidth = 1.0;
      ctx.beginPath();
      ctx.arc(0, 0, rPx, 0, Math.PI * 2);
      ctx.stroke();

      ctx.restore();
    }
  }

  /**
   * Draws START (green flag) and DESTINATION (red flag) with coordinate tags
   */
  drawFlags(ctx) {
    const sx = (this.grid.start.x + 0.5) * this.cellSize;
    const sy = (this.grid.start.y + 0.5) * this.cellSize;
    const dx = (this.grid.destination.x + 0.5) * this.cellSize;
    const dy = (this.grid.destination.y + 0.5) * this.cellSize;

    // START (Green Flag)
    this.drawFlagItem(ctx, sx, sy, '#2E7D32', `START (${this.grid.start.x}, ${this.grid.start.y})`);

    // DESTINATION (Red Flag)
    this.drawFlagItem(ctx, dx, dy, '#C62828', `DEST (${this.grid.destination.x}, ${this.grid.destination.y})`);
  }

  drawFlagItem(ctx, x, y, color, label) {
    ctx.save();
    // Flag pole
    ctx.strokeStyle = '#2A1A1E';
    ctx.lineWidth = 2.0;
    ctx.beginPath();
    ctx.moveTo(x, y + 6);
    ctx.lineTo(x, y - 16);
    ctx.stroke();

    // Flag banner
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(x, y - 16);
    ctx.lineTo(x + 12, y - 11);
    ctx.lineTo(x, y - 6);
    ctx.closePath();
    ctx.fill();

    // Base circle
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(x, y + 6, 3, 0, Math.PI * 2);
    ctx.fill();

    // Coordinate tag pill
    ctx.font = 'bold 9px "Inter", sans-serif';
    const textW = ctx.measureText(label).width;
    ctx.fillStyle = 'rgba(255, 244, 222, 0.95)';
    ctx.strokeStyle = '#FFCED8';
    ctx.lineWidth = 1;
    ctx.fillRect(x - textW / 2 - 3, y - 28, textW + 6, 13);
    ctx.strokeRect(x - textW / 2 - 3, y - 28, textW + 6, 13);

    ctx.fillStyle = '#2A1A1E';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(label, x, y - 22);
    ctx.restore();
  }

  /**
   * Draws banner overlays (REPLANNING, HOLD & WAIT, SAFE STOP, ARRIVED)
   */
  drawBanners(ctx) {
    if (this.obstacleManager.isReplanning) {
      this.renderBanner(ctx, '⚠️ REPLANNING IN PROGRESS (PATH B ACTIVE)', '#FFDDDD', '#D9536F');
    } else if (this.ship.state === 'HOLD_WAIT') {
      this.renderBanner(ctx, '⏸ HOLD & WAIT MANOEUVRE (REVERSE THRUST)', '#FFE8C4', '#D97706');
    } else if (this.ship.state === 'EMERGENCY_STOP') {
      this.renderBanner(ctx, '⛔ EMERGENCY SAFE STOP (PASSAGE BLOCKED)', '#FFDDDD', '#B91C1C');
    } else if (this.ship.state === 'ARRIVED') {
      this.renderBanner(ctx, '🏁 DESTINATION REACHED SAFELY', '#FFF4DE', '#15803D');
    }
  }

  renderBanner(ctx, text, bgColor, textColor) {
    ctx.save();
    ctx.font = 'bold 12px "Inter", sans-serif';
    const textW = ctx.measureText(text).width;
    const bannerW = textW + 32;
    const bannerH = 28;
    const bx = (500 - bannerW) / 2;
    const by = 16;

    ctx.fillStyle = bgColor;
    ctx.strokeStyle = '#FFCED8';
    ctx.lineWidth = 2;
    ctx.shadowColor = 'rgba(0, 0, 0, 0.15)';
    ctx.shadowBlur = 8;
    ctx.fillRect(bx, by, bannerW, bannerH);
    ctx.strokeRect(bx, by, bannerW, bannerH);

    ctx.shadowColor = 'transparent';
    ctx.fillStyle = textColor;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, 250, by + bannerH / 2);
    ctx.restore();
  }

  /**
   * Mouse Hover coordinate & bathymetry info HUD
   */
  drawHoverHUD(ctx) {
    if (!this.hoverCoord) return;
    const { x, y } = this.hoverCoord;
    const type = this.grid.cells[y][x];
    const depth = this.grid.depths[y][x];
    const clearance = this.grid.distanceField[y][x];

    let typeStr = 'Deep Sea';
    if (type === this.grid.TYPE_LAND) typeStr = 'Land Obstacle';
    else if (type === this.grid.TYPE_SHALLOW) typeStr = 'Shallow Shoal';

    const text = `Cell [${x}, ${y}] | Depth: ${depth.toFixed(1)}m | Hazard Dist: ${clearance.toFixed(0)}m | ${typeStr}`;

    ctx.save();
    ctx.font = '10px "Inter", sans-serif';
    ctx.fillStyle = 'rgba(42, 26, 30, 0.85)';
    ctx.fillRect(8, 476, ctx.measureText(text).width + 16, 18);
    ctx.fillStyle = '#FFF4DE';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, 16, 485);
    ctx.restore();
  }
}

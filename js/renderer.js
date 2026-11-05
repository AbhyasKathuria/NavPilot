/**
 * NavPilot - Cargo Ship Real-Time Path-Planning Simulator
 * Canvas Maritime Renderer (renderer.js)
 *
 * Implements 500x500 high-DPI canvas graphics:
 * - Bathymetric depth shading (deep sea, navigable fairway, shallow reef, land)
 * - Port M (Departure Port) and Port N (Destination Port) markers & range vector
 * - Stage 1 (A* coarse route), Stage 2 (RRT* kinematic tree & path), Stage 3 (Somersault Monkey final path)
 * - Ship Domain (SS = L * B * D) dynamic 2D/3D safety envelope
 * - AIS Vessel traffic with MMSI tags, SOG/COG vectors, and collision status
 * - Reactive Avoidance Path B & Real-Time Alert Banners
 * - Palette adherence to #FFCED8, #FFDDDD, #FFF4DE, #FFE8C4
 */

class MaritimeRenderer {
  constructor(canvas, grid, ship, aisSystem, obstacleManager = null) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.grid = grid;
    this.ship = ship;
    this.aisSystem = aisSystem;
    this.obstacleManager = obstacleManager;

    this.cellSize = 10; // 500px / 50 cells = 10px per cell
    this.themeMode = 'strict';

    // Layer visibility toggles
    this.showStage1 = true;      // Stage 1 A* Coarse Route
    this.showStage2 = true;      // Stage 2 RRT* Kinematic Feasible Route & Tree
    this.showStage3 = true;      // Stage 3 Somersault Monkey Optimized Route
    this.showShipDomain = true;   // Ship Domain SS = L * B * D
    this.showAISTraffic = true;   // AIS Vessels & Broadcast Tags

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
    this.canvas.style.width = '500px';
    this.canvas.style.height = '500px';
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
    const dpr = window.devicePixelRatio || 1;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, 500, 500);

    // 1. Bathymetric Grid & Static Chart Obstacles
    this.drawBathymetricGrid(ctx);

    // 2. Straight-Line Range Vector between Port M & Port N
    this.drawPortRangeVector(ctx);

    // 3. Explored Nodes Wavefront (Stage 1 A* animation)
    this.drawExploredNodes(ctx);

    // 4. Three-Stage Hybrid Pipeline Routes
    this.drawThreeStageRoutes(ctx);

    // 5. AIS Marine Traffic (Other Vessels)
    if (this.showAISTraffic && this.aisSystem) {
      this.drawAISTraffic(ctx);
    }

    // 6. Dynamic Obstacles (if any legacy debris/icebergs)
    if (this.obstacleManager) {
      this.drawDynamicObstacles(ctx);
    }

    // 7. Cargo ASV Ship, Domain (SS = L * B * D), Rudder & Heading
    this.drawShipAndDomain(ctx);

    // 8. Port M (Departure) & Port N (Destination) Markers
    this.drawPorts(ctx);

    // 9. Alert Banners (Replanning, Collision Alert, Arrived)
    this.drawBanners(ctx);

    // 10. Hover Bathymetry / AIS Inspector HUD
    this.drawHoverHUD(ctx);
  }

  /**
   * Draws bathymetric sea map with depth contours
   */
  drawBathymetricGrid(ctx) {
    for (let y = 0; y < this.grid.rows; y++) {
      for (let x = 0; x < this.grid.cols; x++) {
        const type = this.grid.cells[y][x];
        const depth = this.grid.depths[y][x];
        const px = x * this.cellSize;
        const py = y * this.cellSize;

        if (type === this.grid.TYPE_LAND) {
          // Land / Island obstacle
          ctx.fillStyle = '#334155'; // Dark bedrock / slate
          ctx.fillRect(px, py, this.cellSize, this.cellSize);
          ctx.fillStyle = '#87C4FF'; // Coastal highlight crest
          ctx.fillRect(px + 1, py + 1, this.cellSize - 2, 2);
        } else if (type === this.grid.TYPE_SHALLOW || depth < this.ship.draft) {
          // Shallow reef/shoal impassable for vessel draft D
          ctx.fillStyle = '#FFEED9'; // Warm Sand / Shallow Reef
          ctx.fillRect(px, py, this.cellSize, this.cellSize);
          ctx.fillStyle = '#E5D3BD';
          ctx.fillRect(px + 3, py + 3, 2, 2);
          ctx.fillRect(px + 7, py + 6, 2, 2);
        } else {
          // Deep Navigable Sea
          ctx.fillStyle = '#E0F4FF'; // Crisp Soft Azure Deep Sea
          ctx.fillRect(px, py, this.cellSize, this.cellSize);
        }
      }
    }

    // Nautical grid coordinate lines
    ctx.strokeStyle = 'rgba(135, 196, 255, 0.25)';
    ctx.lineWidth = 0.5;
    ctx.beginPath();
    for (let i = 0; i <= 50; i += 5) {
      ctx.moveTo(i * this.cellSize, 0);
      ctx.lineTo(i * this.cellSize, 500);
      ctx.moveTo(0, i * this.cellSize);
      ctx.lineTo(500, i * this.cellSize);
    }
    ctx.stroke();
  }

  /**
   * Vector line connecting Port M and Port N
   */
  drawPortRangeVector(ctx) {
    const sx = (this.grid.start.x + 0.5) * this.cellSize;
    const sy = (this.grid.start.y + 0.5) * this.cellSize;
    const dx = (this.grid.destination.x + 0.5) * this.cellSize;
    const dy = (this.grid.destination.y + 0.5) * this.cellSize;

    ctx.save();
    ctx.setLineDash([4, 4]);
    ctx.strokeStyle = 'rgba(10, 37, 64, 0.35)';
    ctx.lineWidth = 1.0;
    ctx.beginPath();
    ctx.moveTo(sx, sy);
    ctx.lineTo(dx, dy);
    ctx.stroke();
    ctx.restore();

    // Range distance label
    const midX = (sx + dx) / 2;
    const midY = (sy + dy) / 2;
    const distInfo = this.grid.getStraightLineDistance();

    ctx.save();
    ctx.fillStyle = '#FFEED9';
    ctx.strokeStyle = '#39A7FF';
    ctx.lineWidth = 1.2;
    const labelText = `Range: ${distInfo.nauticalMiles.toFixed(2)} NM (${distInfo.km.toFixed(2)} km)`;
    ctx.font = '10px "Inter", sans-serif';
    const textWidth = ctx.measureText(labelText).width;

    ctx.fillRect(midX - textWidth / 2 - 4, midY - 8, textWidth + 8, 16);
    ctx.strokeRect(midX - textWidth / 2 - 4, midY - 8, textWidth + 8, 16);

    ctx.fillStyle = '#0A2540';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(labelText, midX, midY);
    ctx.restore();
  }

  drawExploredNodes(ctx) {
    if (!this.exploredAnimationQueue || this.exploredAnimationQueue.length === 0) return;

    if (this.isAnimatingExploration) {
      this.animatedExploredIndex = Math.min(
        this.exploredAnimationQueue.length,
        this.animatedExploredIndex + 30
      );
      if (this.animatedExploredIndex >= this.exploredAnimationQueue.length) {
        this.isAnimatingExploration = false;
        if (this.onExplorationComplete) this.onExplorationComplete();
      }
    }

    const count = this.isAnimatingExploration ? this.animatedExploredIndex : this.exploredAnimationQueue.length;
    ctx.fillStyle = 'rgba(57, 167, 255, 0.35)';

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
   * Draws the Three-Stage Hybrid Pipeline routes
   */
  drawThreeStageRoutes(ctx) {
    // -------------------------------------------------------------
    // STAGE 1: A* Global Coarse Waypoint Route (dashed amber/gold)
    // -------------------------------------------------------------
    if (this.showStage1 && this.ship.stage1Path && this.ship.stage1Path.length > 1) {
      ctx.save();
      ctx.setLineDash([4, 4]);
      ctx.strokeStyle = '#D97706'; // Amber Stage 1
      ctx.lineWidth = 1.6;
      ctx.beginPath();
      ctx.moveTo(
        (this.ship.stage1Path[0].x + 0.5) * this.cellSize,
        (this.ship.stage1Path[0].y + 0.5) * this.cellSize
      );
      for (let i = 1; i < this.ship.stage1Path.length; i++) {
        ctx.lineTo(
          (this.ship.stage1Path[i].x + 0.5) * this.cellSize,
          (this.ship.stage1Path[i].y + 0.5) * this.cellSize
        );
      }
      ctx.stroke();

      // Waypoint diamond markers
      ctx.fillStyle = '#D97706';
      for (let i = 0; i < this.ship.stage1Path.length; i += 6) {
        const wx = (this.ship.stage1Path[i].x + 0.5) * this.cellSize;
        const wy = (this.ship.stage1Path[i].y + 0.5) * this.cellSize;
        ctx.fillRect(wx - 2, wy - 2, 4, 4);
      }
      ctx.restore();
    }

    // -------------------------------------------------------------
    // STAGE 2: RRT* Kinematic Feasible Route (cyan/teal)
    // -------------------------------------------------------------
    if (this.showStage2 && this.ship.stage2Path && this.ship.stage2Path.length > 1) {
      ctx.save();
      ctx.strokeStyle = '#0284C7'; // Light ocean blue / cyan
      ctx.lineWidth = 2.0;
      ctx.beginPath();
      ctx.moveTo(
        (this.ship.stage2Path[0].x + 0.5) * this.cellSize,
        (this.ship.stage2Path[0].y + 0.5) * this.cellSize
      );
      for (let i = 1; i < this.ship.stage2Path.length; i++) {
        ctx.lineTo(
          (this.ship.stage2Path[i].x + 0.5) * this.cellSize,
          (this.ship.stage2Path[i].y + 0.5) * this.cellSize
        );
      }
      ctx.stroke();
      ctx.restore();
    }

    // -------------------------------------------------------------
    // STAGE 3: Somersault Monkey Search Final Route (Vivid Ocean Blue)
    // -------------------------------------------------------------
    const finalRoute = this.ship.stage3Path || this.ship.originalPath;
    if (this.showStage3 && finalRoute && finalRoute.length > 1) {
      ctx.save();
      if (this.ship.hasReplanned) {
        ctx.setLineDash([5, 5]);
        ctx.strokeStyle = 'rgba(72, 101, 129, 0.6)';
        ctx.lineWidth = 2.0;
      } else {
        ctx.strokeStyle = '#39A7FF'; // Vivid Ocean Blue Accent
        ctx.lineWidth = 3.2;
        ctx.shadowColor = 'rgba(57, 167, 255, 0.55)';
        ctx.shadowBlur = 8;
      }

      ctx.beginPath();
      ctx.moveTo(
        (finalRoute[0].x + 0.5) * this.cellSize,
        (finalRoute[0].y + 0.5) * this.cellSize
      );
      for (let i = 1; i < finalRoute.length; i++) {
        ctx.lineTo(
          (finalRoute[i].x + 0.5) * this.cellSize,
          (finalRoute[i].y + 0.5) * this.cellSize
        );
      }
      ctx.stroke();
      ctx.restore();
    }

    // -------------------------------------------------------------
    // PATH B: Active Dynamic Collision Avoidance Route (Emerald Green)
    // -------------------------------------------------------------
    if (this.ship.hasReplanned && this.ship.replanPath && this.ship.replanPath.length > 1) {
      ctx.save();
      ctx.strokeStyle = '#15803D'; // Vivid Emerald Green
      ctx.lineWidth = 3.4;
      ctx.shadowColor = 'rgba(21, 128, 61, 0.5)';
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
   * Draws Cargo Ship and its Ship Domain SS = L * B * D
   */
  drawShipAndDomain(ctx) {
    const ship = this.ship;
    const px = (ship.x + 0.5) * this.cellSize;
    const py = (ship.y + 0.5) * this.cellSize;

    // Wake trail
    if (ship.trail && ship.trail.length > 1) {
      ctx.save();
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.6)';
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

    const halfLenPx = ((ship.length / 2.0) / 50.0) * this.cellSize;
    const halfBmPx = ((ship.beam / 2.0) / 50.0) * this.cellSize;
    const domainRadiusMeters = ship.domainRadiusMeters;
    const domainRadiusPx = (domainRadiusMeters / 50.0) * this.cellSize;

    ctx.save();
    ctx.translate(px, py);
    ctx.rotate(ship.heading);

    // -------------------------------------------------------------
    // SHIP DOMAIN (SS = L * B * D) Envelope
    // -------------------------------------------------------------
    if (this.showShipDomain) {
      ctx.save();
      ctx.fillStyle = 'rgba(135, 196, 255, 0.35)'; // #87C4FF domain zone
      ctx.strokeStyle = '#39A7FF';                 // #39A7FF domain boundary
      ctx.lineWidth = 1.5;
      ctx.setLineDash([4, 4]);

      ctx.beginPath();
      ctx.ellipse(
        halfLenPx * 0.4, 0,
        halfLenPx * 1.5 + domainRadiusPx * 0.8,
        halfBmPx + domainRadiusPx,
        0, 0, Math.PI * 2
      );
      ctx.fill();
      ctx.stroke();

      // Ship Domain Tag
      ctx.fillStyle = '#0A2540';
      ctx.font = 'bold 8px "Inter", sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(`SS: ${(ship.shipDomainVolume / 1000).toFixed(0)}k m³`, 0, -halfBmPx - domainRadiusPx - 3);
      ctx.restore();
    }

    // -------------------------------------------------------------
    // SHIP HULL (Length L, Beam B, Pointed Bow, Bridge, Containers)
    // -------------------------------------------------------------
    ctx.fillStyle = '#0A2540'; // Deep maritime navy hull
    ctx.strokeStyle = '#87C4FF';
    ctx.lineWidth = 1.2;

    ctx.beginPath();
    ctx.moveTo(halfLenPx * 1.15, 0); // Bow tip
    ctx.lineTo(halfLenPx * 0.8, halfBmPx);
    ctx.lineTo(-halfLenPx, halfBmPx);
    ctx.lineTo(-halfLenPx, -halfBmPx);
    ctx.lineTo(halfLenPx * 0.8, -halfBmPx);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // Cargo Container Bay
    ctx.fillStyle = '#FFEED9'; // Warm Sand cargo crates
    ctx.fillRect(-halfLenPx * 0.5, -halfBmPx * 0.7, halfLenPx * 0.9, halfBmPx * 1.4);

    // Navigation Bridge
    ctx.fillStyle = '#FFFFFF';
    ctx.fillRect(-halfLenPx * 0.85, -halfBmPx * 0.5, halfLenPx * 0.25, halfBmPx * 1.0);

    // Rudder Angle Deflection
    ctx.strokeStyle = '#39A7FF';
    ctx.lineWidth = 2.0;
    ctx.beginPath();
    ctx.moveTo(-halfLenPx, 0);
    const rudderLen = halfBmPx * 1.1;
    ctx.lineTo(-halfLenPx - Math.cos(ship.rudderAngle) * rudderLen, -Math.sin(ship.rudderAngle) * rudderLen);
    ctx.stroke();

    // Heading Vector Arrow
    ctx.strokeStyle = '#0284C7';
    ctx.lineWidth = 2.0;
    ctx.beginPath();
    ctx.moveTo(halfLenPx * 1.15, 0);
    ctx.lineTo(halfLenPx * 1.15 + 18, 0);
    ctx.stroke();

    ctx.restore();
  }

  /**
   * Draws AIS marine traffic
   */
  drawAISTraffic(ctx) {
    const vessels = this.aisSystem.vessels;

    for (let i = 0; i < vessels.length; i++) {
      const v = vessels[i];
      const px = (v.x + 0.5) * this.cellSize;
      const py = (v.y + 0.5) * this.cellSize;
      const halfLenPx = ((v.length / 2.0) / 50.0) * this.cellSize;
      const halfBmPx = ((v.beam / 2.0) / 50.0) * this.cellSize;

      // Predicted AIS forward track
      ctx.save();
      ctx.setLineDash([3, 3]);
      ctx.strokeStyle = v.riskLevel === 'COLLISION_ALERT' ? '#DC2626' : 'rgba(57, 167, 255, 0.6)';
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(px, py);
      for (let t = 2; t <= 18; t += 3) {
        const pred = v.predictPosition(t);
        ctx.lineTo((pred.x + 0.5) * this.cellSize, (pred.y + 0.5) * this.cellSize);
      }
      ctx.stroke();
      ctx.restore();

      // AIS Vessel Hull
      ctx.save();
      ctx.translate(px, py);
      ctx.rotate((v.cog * Math.PI) / 180.0);

      // Color code by risk
      if (v.riskLevel === 'COLLISION_ALERT') {
        ctx.fillStyle = '#EF4444';
        ctx.strokeStyle = '#991B1B';
      } else if (v.riskLevel === 'WARNING') {
        ctx.fillStyle = '#F59E0B';
        ctx.strokeStyle = '#78350F';
      } else {
        ctx.fillStyle = '#39A7FF'; // Vibrant Ocean Blue AIS vessel
        ctx.strokeStyle = '#0A2540';
      }

      ctx.beginPath();
      ctx.moveTo(halfLenPx * 1.2, 0);
      ctx.lineTo(halfLenPx * 0.7, halfBmPx);
      ctx.lineTo(-halfLenPx, halfBmPx);
      ctx.lineTo(-halfLenPx, -halfBmPx);
      ctx.lineTo(halfLenPx * 0.7, -halfBmPx);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();

      // Velocity arrow
      ctx.strokeStyle = '#FFFFFF';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(halfLenPx * 1.8, 0);
      ctx.stroke();

      ctx.restore();

      // AIS Broadcast Tag Pill (MMSI & SOG)
      ctx.save();
      ctx.font = 'bold 8px "Inter", sans-serif';
      const label = `${v.name.split(' ')[0]} | ${v.sog.toFixed(1)}kn`;
      const textW = ctx.measureText(label).width;
      ctx.fillStyle = '#FFEED9';
      ctx.strokeStyle = '#87C4FF';
      ctx.lineWidth = 1.2;
      ctx.fillRect(px - textW / 2 - 3, py - 18, textW + 6, 12);
      ctx.strokeRect(px - textW / 2 - 3, py - 18, textW + 6, 12);

      ctx.fillStyle = '#0A2540';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(label, px, py - 12);
      ctx.restore();
    }
  }

  drawDynamicObstacles(ctx) {
    if (!this.obstacleManager || !this.obstacleManager.obstacles) return;
    const obstacles = this.obstacleManager.obstacles;

    for (let i = 0; i < obstacles.length; i++) {
      const obs = obstacles[i];
      const px = (obs.x + 0.5) * this.cellSize;
      const py = (obs.y + 0.5) * this.cellSize;
      const rPx = (obs.radiusMeters / 50.0) * this.cellSize;

      ctx.save();
      ctx.translate(px, py);
      if (obs.type === 'iceberg') {
        ctx.fillStyle = '#E0F4FF';
        ctx.strokeStyle = '#39A7FF';
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
      } else if (obs.type === 'debris') {
        ctx.fillStyle = '#FFEED9';
        ctx.strokeStyle = '#0A2540';
        ctx.fillRect(-rPx * 0.7, -rPx * 0.4, rPx * 1.4, rPx * 0.8);
        ctx.strokeRect(-rPx * 0.7, -rPx * 0.4, rPx * 1.4, rPx * 0.8);
      }
      ctx.restore();
    }
  }

  /**
   * Draws Port M (Departure) and Port N (Destination)
   */
  drawPorts(ctx) {
    const sx = (this.grid.start.x + 0.5) * this.cellSize;
    const sy = (this.grid.start.y + 0.5) * this.cellSize;
    const dx = (this.grid.destination.x + 0.5) * this.cellSize;
    const dy = (this.grid.destination.y + 0.5) * this.cellSize;

    // Port M: Departure Port (Green Harbor Quay)
    this.drawPortMarker(ctx, sx, sy, '#16A34A', 'PORT M (Departure)', `(${this.grid.start.x}, ${this.grid.start.y})`);

    // Port N: Destination Port (Red Harbor Quay)
    this.drawPortMarker(ctx, dx, dy, '#DC2626', 'PORT N (Destination)', `(${this.grid.destination.x}, ${this.grid.destination.y})`);
  }

  drawPortMarker(ctx, x, y, color, title, coords) {
    ctx.save();
    // Harbor Pier Structure
    ctx.fillStyle = '#334155';
    ctx.fillRect(x - 6, y + 4, 12, 4);

    // Flag Pole
    ctx.strokeStyle = '#0A2540';
    ctx.lineWidth = 2.0;
    ctx.beginPath();
    ctx.moveTo(x, y + 6);
    ctx.lineTo(x, y - 18);
    ctx.stroke();

    // Port Banner Flag
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(x, y - 18);
    ctx.lineTo(x + 14, y - 12);
    ctx.lineTo(x, y - 6);
    ctx.closePath();
    ctx.fill();

    // Port Label Badge
    ctx.font = 'bold 9px "Inter", sans-serif';
    const label = `${title} ${coords}`;
    const textW = ctx.measureText(label).width;

    ctx.fillStyle = '#FFEED9';
    ctx.strokeStyle = '#39A7FF';
    ctx.lineWidth = 1.2;
    ctx.fillRect(x - textW / 2 - 4, y - 30, textW + 8, 14);
    ctx.strokeRect(x - textW / 2 - 4, y - 30, textW + 8, 14);

    ctx.fillStyle = '#0A2540';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(label, x, y - 23);
    ctx.restore();
  }

  drawBanners(ctx) {
    if (this.ship.hasReplanned && this.obstacleManager && this.obstacleManager.isReplanning) {
      this.renderBanner(ctx, '⚠️ DYNAMIC REROUTE ACTIVE: AVOIDING AIS TRAFFIC (PATH B)', '#FFEED9', '#B91C1C');
    } else if (this.ship.state === 'HOLD_WAIT') {
      this.renderBanner(ctx, '⏸ HOLD & WAIT MANOEUVRE (REVERSE THRUST)', '#FFEED9', '#B45309');
    } else if (this.ship.state === 'EMERGENCY_STOP') {
      this.renderBanner(ctx, '⛔ EMERGENCY SAFE STOP (PASSAGE BLOCKED)', '#FEE2E2', '#991B1B');
    } else if (this.ship.state === 'ARRIVED') {
      this.renderBanner(ctx, '🏁 DESTINATION PORT N REACHED SAFELY', '#E0F4FF', '#15803D');
    }
  }

  renderBanner(ctx, text, bgColor, textColor) {
    ctx.save();
    ctx.font = 'bold 11px "Inter", sans-serif';
    const textW = ctx.measureText(text).width;
    const bannerW = textW + 28;
    const bannerH = 26;
    const bx = (500 - bannerW) / 2;
    const by = 14;

    ctx.fillStyle = bgColor;
    ctx.strokeStyle = '#39A7FF';
    ctx.lineWidth = 1.8;
    ctx.fillRect(bx, by, bannerW, bannerH);
    ctx.strokeRect(bx, by, bannerW, bannerH);

    ctx.fillStyle = textColor;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, 250, by + bannerH / 2);
    ctx.restore();
  }

  drawHoverHUD(ctx) {
    if (!this.hoverCoord) return;
    const { x, y } = this.hoverCoord;
    const type = this.grid.cells[y][x];
    const depth = this.grid.depths[y][x];
    const clearance = this.grid.distanceField[y][x];

    let typeStr = 'Deep Sea';
    if (type === this.grid.TYPE_LAND) typeStr = 'Land Obstacle';
    else if (type === this.grid.TYPE_SHALLOW) typeStr = 'Shoal Reef';

    const text = `Cell [${x}, ${y}] | Depth: ${depth.toFixed(1)}m | Hazard Dist: ${clearance.toFixed(0)}m | ${typeStr}`;

    ctx.save();
    ctx.font = '10px "Inter", sans-serif';
    ctx.fillStyle = 'rgba(10, 37, 64, 0.9)';
    ctx.fillRect(8, 478, ctx.measureText(text).width + 16, 17);
    ctx.fillStyle = '#E0F4FF';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, 16, 486);
    ctx.restore();
  }
}

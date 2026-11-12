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

    // Sequential Solver Animation States
    this.exploredAnimationQueue = [];
    this.animatedExploredIndex = 0;
    this.isAnimatingExploration = false;

    this.rrtTreeEdges = [];
    this.animatedRrtIndex = 0;
    this.isAnimatingRrt = false;

    this.monkeyAnimPaths = [];
    this.monkeyAnimIdx = 0;
    this.isAnimatingMonkey = false;

    this.solverStatusBanner = ''; // e.g., 'Computing 1/3: A*...', 'Computing 2/3: RRT*...'
    this.placementPreview = null; // { startX, startY, endX, endY, tool }

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

  setPlacementPreview(preview) {
    this.placementPreview = preview;
  }

  clearAllAnimations() {
    this.exploredAnimationQueue = [];
    this.animatedExploredIndex = 0;
    this.isAnimatingExploration = false;
    this.rrtTreeEdges = [];
    this.animatedRrtIndex = 0;
    this.isAnimatingRrt = false;
    this.monkeyAnimPaths = [];
    this.monkeyAnimIdx = 0;
    this.isAnimatingMonkey = false;
    this.solverStatusBanner = '';
    this.placementPreview = null;
  }

  startExplorationAnimation(exploredNodes, onComplete) {
    this.exploredAnimationQueue = exploredNodes || [];
    this.animatedExploredIndex = 0;
    this.isAnimatingExploration = true;
    this.onExplorationComplete = onComplete;
  }

  startRrtTreeAnimation(treeEdges) {
    this.rrtTreeEdges = treeEdges || [];
    this.animatedRrtIndex = 0;
    this.isAnimatingRrt = true;
  }

  startMonkeyAnimation(iterationHistory) {
    this.monkeyAnimPaths = iterationHistory || [];
    this.monkeyAnimIdx = 0;
    this.isAnimatingMonkey = true;
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

    // 3. Stage 1 A* Explored Nodes Wavefront & Stage 2 RRT* Tree Branches
    this.drawExploredNodes(ctx);
    this.drawRrtTree(ctx);

    // 4. Three-Stage Hybrid Pipeline Routes & Stage 3 Monkey Convergence
    this.drawThreeStageRoutes(ctx);
    this.drawMonkeyAnimation(ctx);

    // 5. AIS Marine Traffic (Cargo Ships & Mini Ships)
    if (this.showAISTraffic && this.aisSystem) {
      this.drawAISTraffic(ctx);
    }

    // 6. Dynamic Obstacles (Icebergs / Drifting Debris)
    if (this.obstacleManager) {
      this.drawDynamicObstacles(ctx);
    }

    // 7. Interactive Placement Aim Vector Preview
    if (this.placementPreview) {
      this.drawPlacementPreview(ctx);
    }

    // 8. Cargo ASV Ship, Domain (SS = L * B * D), Rudder & Heading
    this.drawShipAndDomain(ctx);

    // 9. Port M (Departure) & Port N (Destination) Markers
    this.drawPorts(ctx);

    // 10. Alert Banners & Sequential Solver Status
    this.drawBanners(ctx);

    // 11. Hover Bathymetry Inspector HUD
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
   * Vector line connecting Port M and Port N (Straight-line range display)
   * If line of sight intersects land/shoals, avoids cutting through obstacles!
   */
  drawPortRangeVector(ctx) {
    const sx = (this.grid.start.x + 0.5) * this.cellSize;
    const sy = (this.grid.start.y + 0.5) * this.cellSize;
    const dx = (this.grid.destination.x + 0.5) * this.cellSize;
    const dy = (this.grid.destination.y + 0.5) * this.cellSize;
    const distInfo = this.grid.getStraightLineDistance();

    // Check if direct line-of-sight has obstacle obstruction
    const isDirectLOSClear = this.grid.lineOfSight(
      this.grid.start.x, this.grid.start.y,
      this.grid.destination.x, this.grid.destination.y,
      this.ship.draft, 0.0
    );

    ctx.save();
    if (isDirectLOSClear) {
      // Draw faint direct line only when clear water
      ctx.setLineDash([4, 4]);
      ctx.strokeStyle = 'rgba(57, 167, 255, 0.28)';
      ctx.lineWidth = 1.0;
      ctx.beginPath();
      ctx.moveTo(sx, sy);
      ctx.lineTo(dx, dy);
      ctx.stroke();

      // Range distance label at midpoint
      const midX = (sx + dx) / 2;
      const midY = (sy + dy) / 2;
      const labelText = `Range: ${distInfo.nauticalMiles.toFixed(2)} NM (${distInfo.km.toFixed(2)} km)`;
      ctx.font = '10px "Inter", sans-serif';
      const textWidth = ctx.measureText(labelText).width;

      ctx.fillStyle = '#FFEED9';
      ctx.strokeStyle = '#39A7FF';
      ctx.lineWidth = 1.2;
      ctx.fillRect(midX - textWidth / 2 - 4, midY - 8, textWidth + 8, 16);
      ctx.strokeRect(midX - textWidth / 2 - 4, midY - 8, textWidth + 8, 16);

      ctx.fillStyle = '#0A2540';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(labelText, midX, midY);
    } else {
      // Obstacle in between: do NOT draw a line cutting through land!
      // Display range badge neatly in open water near Port N
      const labelText = `Direct Range: ${distInfo.nauticalMiles.toFixed(2)} NM (${distInfo.km.toFixed(2)} km)`;
      ctx.font = '10px "Inter", sans-serif';
      const textWidth = ctx.measureText(labelText).width;
      const badgeX = Math.max(textWidth / 2 + 10, Math.min(500 - textWidth / 2 - 10, dx));
      const badgeY = dy < 40 ? dy + 38 : dy - 44;

      ctx.fillStyle = '#FFEED9';
      ctx.strokeStyle = '#39A7FF';
      ctx.lineWidth = 1.2;
      ctx.fillRect(badgeX - textWidth / 2 - 4, badgeY - 8, textWidth + 8, 16);
      ctx.strokeRect(badgeX - textWidth / 2 - 4, badgeY - 8, textWidth + 8, 16);

      ctx.fillStyle = '#0A2540';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(labelText, badgeX, badgeY);
    }
    ctx.restore();
  }

  drawExploredNodes(ctx) {
    if (!this.exploredAnimationQueue || this.exploredAnimationQueue.length === 0) return;

    if (this.isAnimatingExploration) {
      this.animatedExploredIndex = Math.min(
        this.exploredAnimationQueue.length,
        this.animatedExploredIndex + 36
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
      ctx.arc(cx, cy, 1.6, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  /**
   * Animates Algorithm 2 (RRT* Kinematic Tree Expansion)
   */
  drawRrtTree(ctx) {
    if (!this.rrtTreeEdges || this.rrtTreeEdges.length === 0) return;

    if (this.isAnimatingRrt) {
      this.animatedRrtIndex = Math.min(this.rrtTreeEdges.length, this.animatedRrtIndex + 6);
      if (this.animatedRrtIndex >= this.rrtTreeEdges.length) {
        this.isAnimatingRrt = false;
      }
    }

    const count = this.isAnimatingRrt ? this.animatedRrtIndex : this.rrtTreeEdges.length;
    ctx.save();
    ctx.strokeStyle = 'rgba(2, 132, 199, 0.42)';
    ctx.lineWidth = 1.0;
    ctx.beginPath();
    for (let i = 0; i < count; i++) {
      const edge = this.rrtTreeEdges[i];
      ctx.moveTo((edge.from.x + 0.5) * this.cellSize, (edge.from.y + 0.5) * this.cellSize);
      ctx.lineTo((edge.to.x + 0.5) * this.cellSize, (edge.to.y + 0.5) * this.cellSize);
    }
    ctx.stroke();

    ctx.fillStyle = 'rgba(2, 132, 199, 0.65)';
    for (let i = 0; i < count; i++) {
      const edge = this.rrtTreeEdges[i];
      ctx.fillRect((edge.to.x + 0.5) * this.cellSize - 1, (edge.to.y + 0.5) * this.cellSize - 1, 2, 2);
    }
    ctx.restore();
  }

  /**
   * Animates Algorithm 3 (Somersault Monkey Search Candidate Convergence)
   */
  drawMonkeyAnimation(ctx) {
    if (!this.isAnimatingMonkey || !this.monkeyAnimPaths || this.monkeyAnimPaths.length === 0) return;

    this.monkeyAnimIdx = Math.min(this.monkeyAnimPaths.length - 1, this.monkeyAnimIdx + 0.25);
    const frameIdx = Math.floor(this.monkeyAnimIdx);
    const activeWps = this.monkeyAnimPaths[frameIdx];

    if (activeWps && activeWps.length > 1) {
      ctx.save();
      ctx.strokeStyle = 'rgba(57, 167, 255, 0.85)';
      ctx.lineWidth = 2.6;
      ctx.setLineDash([6, 3]);
      ctx.beginPath();
      ctx.moveTo((activeWps[0].x + 0.5) * this.cellSize, (activeWps[0].y + 0.5) * this.cellSize);
      for (let i = 1; i < activeWps.length; i++) {
        ctx.lineTo((activeWps[i].x + 0.5) * this.cellSize, (activeWps[i].y + 0.5) * this.cellSize);
      }
      ctx.stroke();

      ctx.fillStyle = '#FFEED9';
      ctx.strokeStyle = '#0A2540';
      ctx.lineWidth = 1.2;
      for (let i = 0; i < activeWps.length; i++) {
        ctx.beginPath();
        ctx.arc((activeWps[i].x + 0.5) * this.cellSize, (activeWps[i].y + 0.5) * this.cellSize, 3.2, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();
      }
      ctx.restore();
    }

    if (this.monkeyAnimIdx >= this.monkeyAnimPaths.length - 1) {
      this.isAnimatingMonkey = false;
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
    // OPTIMAL / STAGE 3 ROUTE: Verified Best Optimal Path (Vivid Ocean Blue)
    // -------------------------------------------------------------
    const finalRoute = this.ship.originalPath || this.ship.stage3Path;
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
   * Draws AIS marine traffic (Cargo Ships & Mini Ships)
   */
  drawAISTraffic(ctx) {
    const vessels = this.aisSystem.vessels;

    for (let i = 0; i < vessels.length; i++) {
      const v = vessels[i];
      const px = (v.x + 0.5) * this.cellSize;
      const py = (v.y + 0.5) * this.cellSize;
      const isMini = v.vesselCategory === 'mini';
      const halfLenPx = Math.max(isMini ? 4.5 : 8.5, ((v.length / 2.0) / 50.0) * this.cellSize);
      const halfBmPx = Math.max(isMini ? 2.2 : 3.2, ((v.beam / 2.0) / 50.0) * this.cellSize);

      // Wake trail for moving ships
      if (v.trail && v.trail.length > 1) {
        ctx.save();
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.45)';
        ctx.lineWidth = isMini ? 1.0 : 1.4;
        ctx.beginPath();
        for (let t = 0; t < v.trail.length; t++) {
          const tx = (v.trail[t].x + 0.5) * this.cellSize;
          const ty = (v.trail[t].y + 0.5) * this.cellSize;
          if (t === 0) ctx.moveTo(tx, ty);
          else ctx.lineTo(tx, ty);
        }
        ctx.stroke();
        ctx.restore();
      }

      // Predicted AIS forward track
      if (v.sog > 0.2) {
        ctx.save();
        ctx.setLineDash([3, 3]);
        ctx.strokeStyle = v.riskLevel === 'COLLISION_ALERT' ? '#DC2626' : (isMini ? 'rgba(2, 132, 199, 0.55)' : 'rgba(57, 167, 255, 0.65)');
        ctx.lineWidth = 1.2;
        ctx.beginPath();
        ctx.moveTo(px, py);
        for (let t = 2; t <= 18; t += 3) {
          const pred = v.predictPosition(t, this.grid);
          ctx.lineTo((pred.x + 0.5) * this.cellSize, (pred.y + 0.5) * this.cellSize);
        }
        ctx.stroke();
        ctx.restore();
      }

      // Vessel Hull oriented accurately to compass heading (0=North=-PI/2 in canvas)
      ctx.save();
      ctx.translate(px, py);
      ctx.rotate(((v.cog - 90) * Math.PI) / 180.0);

      if (v.riskLevel === 'COLLISION_ALERT') {
        ctx.fillStyle = '#EF4444';
        ctx.strokeStyle = '#991B1B';
      } else if (v.riskLevel === 'WARNING') {
        ctx.fillStyle = '#F59E0B';
        ctx.strokeStyle = '#78350F';
      } else if (isMini) {
        ctx.fillStyle = '#FFEED9';
        ctx.strokeStyle = '#0284C7';
      } else {
        ctx.fillStyle = '#39A7FF';
        ctx.strokeStyle = '#0A2540';
      }

      ctx.lineWidth = 1.3;
      ctx.beginPath();
      if (isMini) {
        // Pointed speed/patrol mini-vessel silhouette
        ctx.moveTo(halfLenPx * 1.3, 0);
        ctx.lineTo(-halfLenPx * 0.8, halfBmPx);
        ctx.lineTo(-halfLenPx, 0);
        ctx.lineTo(-halfLenPx * 0.8, -halfBmPx);
      } else {
        // Full cargo vessel hull
        ctx.moveTo(halfLenPx * 1.2, 0);
        ctx.lineTo(halfLenPx * 0.7, halfBmPx);
        ctx.lineTo(-halfLenPx, halfBmPx);
        ctx.lineTo(-halfLenPx, -halfBmPx);
        ctx.lineTo(halfLenPx * 0.7, -halfBmPx);
      }
      ctx.closePath();
      ctx.fill();
      ctx.stroke();

      // Cabin / cargo detail
      if (!isMini) {
        ctx.fillStyle = '#FFEED9';
        ctx.fillRect(-halfLenPx * 0.45, -halfBmPx * 0.55, halfLenPx * 0.8, halfBmPx * 1.1);
      }

      // Bow heading vector if underway
      if (v.sog > 0.2) {
        ctx.strokeStyle = isMini ? '#0284C7' : '#FFFFFF';
        ctx.lineWidth = 1.4;
        ctx.beginPath();
        ctx.moveTo(halfLenPx * 0.5, 0);
        ctx.lineTo(halfLenPx * 1.9, 0);
        ctx.stroke();
      }

      ctx.restore();

      // Tag Pill (Name / Type & SOG)
      ctx.save();
      ctx.font = 'bold 7.5px "Inter", sans-serif';
      const prefix = isMini ? '🚤 ' : '';
      const shortName = v.name.split(' ')[0];
      const sogTag = v.sog <= 0.1 ? 'ANCHOR' : `${v.sog.toFixed(0)}kn`;
      const label = `${prefix}${shortName} | ${sogTag}`;
      const textW = ctx.measureText(label).width;
      ctx.fillStyle = '#FFEED9';
      ctx.strokeStyle = v.riskLevel === 'COLLISION_ALERT' ? '#DC2626' : '#87C4FF';
      ctx.lineWidth = 1.1;
      ctx.fillRect(px - textW / 2 - 3, py - 18, textW + 6, 11);
      ctx.strokeRect(px - textW / 2 - 3, py - 18, textW + 6, 11);

      ctx.fillStyle = '#0A2540';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(label, px, py - 12.5);
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
      const rPx = Math.max(5.5, (obs.radiusMeters / 50.0) * this.cellSize);

      // Drift vector prediction line
      if (Math.hypot(obs.vx, obs.vy) > 0.01) {
        ctx.save();
        ctx.setLineDash([2, 3]);
        ctx.strokeStyle = 'rgba(220, 38, 38, 0.55)';
        ctx.lineWidth = 1.1;
        ctx.beginPath();
        ctx.moveTo(px, py);
        const pred = obs.predictPosition(14, this.grid);
        ctx.lineTo((pred.x + 0.5) * this.cellSize, (pred.y + 0.5) * this.cellSize);
        ctx.stroke();
        ctx.restore();
      }

      ctx.save();
      ctx.translate(px, py);
      if (obs.type === 'iceberg') {
        ctx.fillStyle = '#FFFFFF';
        ctx.strokeStyle = '#39A7FF';
        ctx.lineWidth = 1.6;
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
        ctx.fillStyle = '#FFEED9';
        ctx.strokeStyle = '#DC2626';
        ctx.lineWidth = 1.5;
        ctx.fillRect(-rPx * 0.7, -rPx * 0.45, rPx * 1.4, rPx * 0.9);
        ctx.strokeRect(-rPx * 0.7, -rPx * 0.45, rPx * 1.4, rPx * 0.9);
      }
      ctx.restore();
    }
  }

  /**
   * Renders live heading / vector preview when click-dragging to place ships/hazards
   */
  drawPlacementPreview(ctx) {
    const p = this.placementPreview;
    if (!p) return;

    const sx = (p.startX + 0.5) * this.cellSize;
    const sy = (p.startY + 0.5) * this.cellSize;
    const ex = (p.endX + 0.5) * this.cellSize;
    const ey = (p.endY + 0.5) * this.cellSize;

    ctx.save();
    ctx.strokeStyle = '#0A2540';
    ctx.fillStyle = 'rgba(57, 167, 255, 0.35)';
    ctx.lineWidth = 2.0;
    ctx.setLineDash([4, 3]);
    ctx.beginPath();
    ctx.moveTo(sx, sy);
    ctx.lineTo(ex, ey);
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(sx, sy, 8, 0, Math.PI * 2);
    ctx.fill();
    ctx.setLineDash([]);
    ctx.stroke();

    const dx = ex - sx;
    const dy = ey - sy;
    if (Math.hypot(dx, dy) > 8) {
      const cog = ((Math.atan2(dx, -dy) * 180) / Math.PI + 360) % 360;
      ctx.font = 'bold 9.5px "Inter", sans-serif';
      ctx.fillStyle = '#0A2540';
      ctx.fillText(`Course: ${cog.toFixed(0)}°`, ex + 8, ey - 6);
    }
    ctx.restore();
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
    this.drawPortMarker(ctx, sx, sy, '#16A34A', 'PORT M', `(${this.grid.start.x}, ${this.grid.start.y})`);

    // Port N: Destination Port (Red Harbor Quay)
    this.drawPortMarker(ctx, dx, dy, '#DC2626', 'PORT N', `(${this.grid.destination.x}, ${this.grid.destination.y})`);
  }

  drawPortMarker(ctx, x, y, color, title, coords) {
    ctx.save();
    ctx.fillStyle = '#334155';
    ctx.fillRect(x - 6, y + 4, 12, 4);

    ctx.strokeStyle = '#0A2540';
    ctx.lineWidth = 2.0;
    ctx.beginPath();
    ctx.moveTo(x, y + 6);
    ctx.lineTo(x, y - 18);
    ctx.stroke();

    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(x, y - 18);
    ctx.lineTo(x + 14, y - 12);
    ctx.lineTo(x, y - 6);
    ctx.closePath();
    ctx.fill();

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
    if (this.solverStatusBanner) {
      this.renderBanner(ctx, this.solverStatusBanner, '#FFEED9', '#0A2540');
    } else if (this.ship.hasReplanned && this.obstacleManager && this.obstacleManager.isReplanning) {
      this.renderBanner(ctx, '⚠️ DYNAMIC REROUTE ACTIVE: AVOIDING TRAFFIC (PATH B)', '#FFEED9', '#B91C1C');
    } else if (this.ship.state === 'HOLD_WAIT') {
      this.renderBanner(ctx, '⏸ HOLD & WAIT MANOEUVRE (PROXIMITY SHIELD)', '#FFEED9', '#B45309');
    } else if (this.ship.state === 'EMERGENCY_STOP') {
      this.renderBanner(ctx, '⛔ EMERGENCY SAFE STOP (NO CLEAR PASSAGE)', '#FEE2E2', '#991B1B');
    } else if (this.ship.state === 'ARRIVED') {
      this.renderBanner(ctx, '🏁 DESTINATION PORT N REACHED SAFELY (0 COLLISIONS)', '#E0F4FF', '#15803D');
    }
  }

  renderBanner(ctx, text, bgColor, textColor) {
    ctx.save();
    ctx.font = 'bold 10.5px "Inter", sans-serif';
    const textW = ctx.measureText(text).width;
    const bannerW = Math.min(480, textW + 28);
    const bannerH = 26;
    const bx = (500 - bannerW) / 2;
    const by = 12;

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
    const edgeDist = this.grid.getExactHazardDistanceMeters(x, y, this.ship.draft);

    let typeStr = 'Deep Sea';
    if (type === this.grid.TYPE_LAND) typeStr = 'Land Obstacle';
    else if (type === this.grid.TYPE_SHALLOW) typeStr = 'Shoal Reef';

    const text = `Cell [${x}, ${y}] | Depth: ${depth.toFixed(1)}m | Edge Clear: ${edgeDist.toFixed(0)}m | ${typeStr}`;

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

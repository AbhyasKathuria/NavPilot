/**
 * NavPilot - Cargo Ship Real-Time Path-Planning Simulator
 * Main Application Orchestrator (main.js)
 *
 * Implements Three-Stage Hybrid Planning Pipeline:
 * Stage 1: A* Global Path Search (grid-based)
 * Stage 2: RRT* Kinematic Refinement (curvature-constrained)
 * Stage 3: Somersault Monkey Search Optimisation (multi-objective)
 * AIS Data simulation, Ship Domain (SS = L * B * D), Ports M and N.
 */

window.addEventListener('DOMContentLoaded', () => {
  // DOM Elements
  const canvas = document.getElementById('mapCanvas');
  const speedSlider = document.getElementById('speedSlider');
  const speedDisplay = document.getElementById('speedDisplay');

  // Action Buttons
  const btnRun = document.getElementById('btnRun');
  const btnPause = document.getElementById('btnPause');
  const btnReset = document.getElementById('btnReset');
  const btnRunPipeline = document.getElementById('btnRunPipeline');
  const btnBenchmark = document.getElementById('btnBenchmark');
  const btnRandomMap = document.getElementById('btnRandomMap');
  const btnClearObstacles = document.getElementById('btnClearObstacles');
  const btnAddAIS = document.getElementById('btnAddAIS');
  const btnSpawnAIS = document.getElementById('btnSpawnAIS');

  // Visualization Layer Checkboxes
  const chkStage1 = document.getElementById('chkStage1');
  const chkStage2 = document.getElementById('chkStage2');
  const chkStage3 = document.getElementById('chkStage3');
  const chkShipDomain = document.getElementById('chkShipDomain');
  const chkAISTraffic = document.getElementById('chkAISTraffic');

  // Interactive Tools
  const toolButtons = {
    set_start: document.getElementById('toolSetStart'),
    set_dest: document.getElementById('toolSetDest'),
    paint_obstacle: document.getElementById('toolAddLand'),
    paint_clear: document.getElementById('toolRemoveLand'),
    paint_shallow: document.getElementById('toolAddShallow'),
    add_dynamic: document.getElementById('toolAddDynamic')
  };

  // Ship Parameter Inputs
  const shipInputs = {
    length: document.getElementById('paramLength'),
    beam: document.getElementById('paramBeam'),
    draft: document.getElementById('paramDraft'),
    displacement: document.getElementById('paramDisplacement'),
    maxSpeed: document.getElementById('paramSpeed'),
    minTurningRadius: document.getElementById('paramTurnRadius')
  };

  // Ship Domain Displays
  const domainVolDisplay = document.getElementById('domainVolDisplay');
  const domainCalcText = document.getElementById('domainCalcText');
  const lblPortM = document.getElementById('lblPortM');
  const lblPortN = document.getElementById('lblPortN');

  // Live Telemetry Elements
  const statAlg = document.getElementById('statAlg');
  const statNodes = document.getElementById('statNodes');
  const statLength = document.getElementById('statLength');
  const statSmoothness = document.getElementById('statSmoothness');
  const statEnergy = document.getElementById('statEnergy');
  const statETA = document.getElementById('statETA');
  const statClearance = document.getElementById('statClearance');
  const statStopping = document.getElementById('statStopping');
  const statSpeed = document.getElementById('statSpeed');
  const statReplans = document.getElementById('statReplans');

  // Log and Tables
  const aisTrafficTbody = document.getElementById('aisTrafficTbody');
  const comparisonTbody = document.getElementById('comparisonTbody');
  const eventLogList = document.getElementById('eventLogList');
  const btnClearLog = document.getElementById('btnClearLog');

  // State Management
  let activeTool = 'paint_obstacle';
  let isSimRunning = false;
  let simulationSpeed = 1.0;
  let isMouseDown = false;
  let lastFrameTime = performance.now();
  let lastAisTableUpdate = 0;

  // Core Simulation Model Instances
  const grid = new SeaGrid(50, 50, 50); // 50x50 cells, 50m per cell
  const ship = new CargoShip({
    length: 200,
    beam: 32,
    draft: 12,
    displacement: 50000,
    maxSpeed: 18,
    minTurningRadius: 400
  });

  const aisSystem = new AISSystem(grid);
  const obstacleManager = new DynamicObstacleManager(grid, aisSystem);
  const renderer = new MaritimeRenderer(canvas, grid, ship, aisSystem, obstacleManager);

  // Planners for the Three-Stage Architecture
  const aStarPlanner = new AStarPlanner(grid);
  const rrtStarPlanner = new RRTStarPlanner(grid);
  const monkeySearchPlanner = new SomersaultMonkeySearchPlanner(grid);
  const threeStagePlanner = new ThreeStageHybridPlanner(grid);

  // Logging Helper
  function logEvent(message, type = 'info') {
    if (!eventLogList) return;
    const li = document.createElement('li');
    const now = new Date();
    const timeStr = now.toTimeString().split(' ')[0];
    li.className = `log-item log-${type}`;
    li.innerHTML = `<span class="log-time">[${timeStr}]</span> <span class="log-msg">${message}</span>`;
    eventLogList.appendChild(li);
    eventLogList.scrollTop = eventLogList.scrollHeight;
  }

  logEvent('NavPilot Three-Stage Hybrid maritime system initialized.', 'info');
  logEvent('Ship Domain formula: SS = L * B * D. Initial ports: Port M (5, 44) -> Port N (44, 6).', 'info');
  logEvent('AIS System broadcasting live marine traffic for 4 commercial vessels.', 'info');

  // Layer Checkbox Listeners
  if (chkStage1) chkStage1.addEventListener('change', () => { renderer.showStage1 = chkStage1.checked; });
  if (chkStage2) chkStage2.addEventListener('change', () => { renderer.showStage2 = chkStage2.checked; });
  if (chkStage3) chkStage3.addEventListener('change', () => { renderer.showStage3 = chkStage3.checked; });
  if (chkShipDomain) chkShipDomain.addEventListener('change', () => { renderer.showShipDomain = chkShipDomain.checked; });
  if (chkAISTraffic) chkAISTraffic.addEventListener('change', () => { renderer.showAISTraffic = chkAISTraffic.checked; });

  // Update Ship Domain Display
  function updateDomainDisplay() {
    const vol = ship.shipDomainVolume;
    if (domainVolDisplay) domainVolDisplay.textContent = `${(vol / 1000).toFixed(1)}k m³`;
    if (domainCalcText) {
      domainCalcText.textContent = `${ship.length}m × ${ship.beam}m × ${ship.draft}m = ${vol.toLocaleString()} m³ (Domain radius: ${ship.domainRadiusMeters.toFixed(1)}m)`;
    }
    if (lblPortM) lblPortM.textContent = `Port M (${grid.start.x}, ${grid.start.y})`;
    if (lblPortN) lblPortN.textContent = `Port N (${grid.destination.x}, ${grid.destination.y})`;
    if (statStopping) statStopping.textContent = `${ship.computeStoppingDistanceMeters().toFixed(0)} m`;
  }

  // Update AIS Marine Traffic Table
  function updateAISTrafficTable() {
    if (!aisTrafficTbody) return;
    aisTrafficTbody.innerHTML = '';

    aisSystem.vessels.forEach(v => {
      const tr = document.createElement('tr');
      if (v.riskLevel === 'COLLISION_ALERT') tr.className = 'risk-alert';
      else if (v.riskLevel === 'WARNING') tr.className = 'risk-warning';

      let riskBadge = '<span class="badge badge-success">Clear</span>';
      if (v.riskLevel === 'COLLISION_ALERT') riskBadge = '<span class="badge badge-danger">Alert</span>';
      else if (v.riskLevel === 'WARNING') riskBadge = '<span class="badge badge-warning">Caution</span>';

      const cpaStr = v.cpaMeters !== Infinity ? `${v.cpaMeters.toFixed(0)}m` : '—';
      const tcpaStr = v.tcpaSeconds !== Infinity ? `${v.tcpaSeconds.toFixed(1)}s` : '—';

      tr.innerHTML = `
        <td>${v.mmsi}</td>
        <td><strong>${v.name}</strong></td>
        <td>${v.type}</td>
        <td>${v.sog.toFixed(1)} kn</td>
        <td>${v.cog.toFixed(0)}°</td>
        <td>${cpaStr}</td>
        <td>${tcpaStr}</td>
        <td>${riskBadge}</td>
      `;
      aisTrafficTbody.appendChild(tr);
    });
  }

  // Execute Three-Stage Hybrid Pipeline
  function executeThreeStagePipeline(autoStartSimulation = true) {
    logEvent(`Stage 1: A* Global Search from Port M (${grid.start.x}, ${grid.start.y}) to Port N (${grid.destination.x}, ${grid.destination.y})...`, 'info');

    const result = threeStagePlanner.plan(grid.start, grid.destination, ship, aisSystem);

    if (!result.success || !result.path || result.path.length === 0) {
      logEvent(`Three-Stage Pipeline failed: ${result.message}`, 'error');
      if (autoStartSimulation) {
        alert(`Path Planning Failed: ${result.message}`);
      }
      return;
    }

    logEvent(`Stage 1 (A*) complete: coarse waypoint route (${result.stage1.lengthMeters.toFixed(0)}m, ${result.stage1.computeTimeMs.toFixed(1)}ms).`, 'success');
    logEvent(`Stage 2 (RRT*) complete: curvature-constrained feasible path (${result.stage2.computeTimeMs.toFixed(1)}ms, R >= ${ship.minTurningRadius}m).`, 'success');
    logEvent(`Stage 3 (Somersault Monkey Search) complete: final multi-objective ASV route (${result.stage3.computeTimeMs.toFixed(1)}ms, Energy Index: ${result.stage3.energyIndex}).`, 'success');

    // Update ship paths for rendering
    ship.setStagePaths(result.stage1.path, result.stage2.path, result.stage3.path);

    // Update Live Telemetry
    if (statAlg) statAlg.textContent = 'HYBRID 3-STAGE';
    if (statNodes) statNodes.textContent = `${result.stage1.nodesExpanded} (A*) | ${result.stage2.tree.length} (RRT*)`;
    if (statLength) statLength.textContent = `${result.lengthMeters.toFixed(0)} m (${(result.lengthMeters / 1852).toFixed(2)} NM)`;
    if (statSmoothness) statSmoothness.textContent = `${result.smoothnessDeg.toFixed(1)}°`;
    if (statEnergy) statEnergy.textContent = `${result.energyIndex} (Optimized)`;
    if (statClearance) statClearance.textContent = `${result.minClearanceMeters.toFixed(1)} m`;
    if (statReplans) statReplans.textContent = '0';

    const mins = Math.floor(result.transitTimeSec / 60);
    const secs = Math.floor(result.transitTimeSec % 60);
    if (statETA) statETA.textContent = `${mins}m ${secs}s`;

    if (autoStartSimulation) {
      if (result.stage1 && result.stage1.path) {
        renderer.startExplorationAnimation(result.stage1.path, () => {
          isSimRunning = true;
          if (btnRun) {
            btnRun.textContent = 'Running...';
            btnRun.classList.add('active');
          }
        });
      } else {
        isSimRunning = true;
        if (btnRun) {
          btnRun.textContent = 'Running...';
          btnRun.classList.add('active');
        }
      }
    }

    updateBenchmarkTable();
  }

  // Update Benchmark Comparison Table
  function updateBenchmarkTable() {
    if (!comparisonTbody) return;
    comparisonTbody.innerHTML = '';

    // Run evaluations
    const s1 = aStarPlanner.plan(grid.start, grid.destination, ship.draft, ship.computeRequiredClearanceMeters());
    const coarse = threeStagePlanner.extractCoarseWaypoints(s1.path);
    const s2 = rrtStarPlanner.refine(coarse, ship, aisSystem);
    const s3 = monkeySearchPlanner.optimize(s2.waypoints, ship, aisSystem, 25);
    const sFull = threeStagePlanner.lastResult || threeStagePlanner.plan(grid.start, grid.destination, ship, aisSystem);

    const rows = [
      { name: 'Stage 1: A* Global (Grid Baseline)', res: s1, id: 's1' },
      { name: 'Stage 2: RRT* Kinematic Refinement', res: s2, id: 's2' },
      { name: 'Stage 3: Somersault Monkey Search', res: s3, id: 's3' },
      { name: 'Full 3-Stage Hybrid Architecture (Fig 3.2)', res: sFull, id: 'hybrid' }
    ];

    rows.forEach(item => {
      const r = item.res;
      const tr = document.createElement('tr');
      if (item.id === 'hybrid') tr.className = 'highlight-row';

      const isSuccess = r && r.success;
      tr.innerHTML = `
        <td><strong>${item.name}</strong></td>
        <td>${isSuccess ? `${r.computeTimeMs.toFixed(1)} ms` : 'Failed'}</td>
        <td>${isSuccess ? (r.nodesExpanded || r.nodesSampled || '16 pop') : '—'}</td>
        <td>${isSuccess ? `${r.lengthMeters.toFixed(0)} m` : '—'}</td>
        <td>${isSuccess ? `${r.smoothnessDeg.toFixed(1)}°` : '—'}</td>
        <td>${isSuccess ? `${r.minClearanceMeters.toFixed(1)} m` : '—'}</td>
        <td>${isSuccess ? (r.curvatureViolations || 0) : '—'}</td>
        <td><span class="badge ${isSuccess ? 'badge-success' : 'badge-danger'}">${isSuccess ? 'Feasible' : 'Blocked'}</span></td>
      `;
      comparisonTbody.appendChild(tr);
    });
  }

  // Interactive Tools Implementation
  function applyToolAtPixel(clientX, clientY) {
    const rect = canvas.getBoundingClientRect();
    const scaleX = 500 / rect.width;
    const scaleY = 500 / rect.height;
    const px = (clientX - rect.left) * scaleX;
    const py = (clientY - rect.top) * scaleY;

    const gx = Math.floor(px / renderer.cellSize);
    const gy = Math.floor(py / renderer.cellSize);

    if (!grid.inBounds(gx, gy)) return;

    if (activeTool === 'set_start') {
      isSimRunning = false;
      if (btnRun) {
        btnRun.textContent = '▶ Run';
        btnRun.classList.remove('active');
      }
      grid.setStart(gx, gy);
      ship.resetToStart(grid.start);
      updateDomainDisplay();
      executeThreeStagePipeline(false);
      logEvent(`Port M (Departure) relocated to (${gx}, ${gy}).`, 'info');
    } else if (activeTool === 'set_dest') {
      isSimRunning = false;
      if (btnRun) {
        btnRun.textContent = '▶ Run';
        btnRun.classList.remove('active');
      }
      grid.setDestination(gx, gy);
      ship.resetToStart(grid.start);
      updateDomainDisplay();
      executeThreeStagePipeline(false);
      logEvent(`Port N (Destination) relocated to (${gx}, ${gy}).`, 'info');
    } else if (activeTool === 'paint_obstacle') {
      if (!grid.isStart(gx, gy) && !grid.isDestination(gx, gy)) {
        grid.setCell(gx, gy, grid.TYPE_LAND);
        grid.computeDistanceField(ship.draft);
      }
    } else if (activeTool === 'paint_clear') {
      grid.setCell(gx, gy, grid.TYPE_SEA);
      grid.computeDistanceField(ship.draft);
    } else if (activeTool === 'paint_shallow') {
      if (!grid.isStart(gx, gy) && !grid.isDestination(gx, gy)) {
        grid.setCell(gx, gy, grid.TYPE_SHALLOW, 8.0);
        grid.computeDistanceField(ship.draft);
      }
    } else if (activeTool === 'add_dynamic') {
      obstacleManager.addObstacle(gx, gy, (Math.random() - 0.5) * 0.4, (Math.random() - 0.5) * 0.4, 'iceberg', 30);
      logEvent(`Spawned drifting hazard at (${gx}, ${gy}).`, 'info');
    }
  }

  // Canvas Mouse Listeners
  if (canvas) {
    canvas.addEventListener('mousedown', (e) => {
      isMouseDown = true;
      applyToolAtPixel(e.clientX, e.clientY);
    });

    window.addEventListener('mouseup', () => {
      isMouseDown = false;
    });

    canvas.addEventListener('mousemove', (e) => {
      const rect = canvas.getBoundingClientRect();
      const scaleX = 500 / rect.width;
      const scaleY = 500 / rect.height;
      const px = (e.clientX - rect.left) * scaleX;
      const py = (e.clientY - rect.top) * scaleY;
      const gx = Math.floor(px / renderer.cellSize);
      const gy = Math.floor(py / renderer.cellSize);

      renderer.setHoverCoord(gx, gy);

      if (isMouseDown && (activeTool === 'paint_obstacle' || activeTool === 'paint_clear' || activeTool === 'paint_shallow')) {
        applyToolAtPixel(e.clientX, e.clientY);
      }
    });

    canvas.addEventListener('mouseleave', () => {
      renderer.setHoverCoord(-1, -1);
    });
  }

  // Tool Buttons Setup
  Object.keys(toolButtons).forEach(tool => {
    const btn = toolButtons[tool];
    if (!btn) return;
    btn.addEventListener('click', () => {
      Object.values(toolButtons).forEach(b => b && b.classList.remove('active'));
      btn.classList.add('active');
      activeTool = tool;
      logEvent(`Tool selected: ${btn.textContent.trim()}`, 'info');
    });
  });

  // Parameter Change Listeners
  function readShipInputs() {
    if (!shipInputs.length) return;
    ship.updateParameters({
      length: shipInputs.length.value,
      beam: shipInputs.beam.value,
      draft: shipInputs.draft.value,
      displacement: shipInputs.displacement.value,
      maxSpeed: shipInputs.maxSpeed.value,
      minTurningRadius: shipInputs.minTurningRadius.value
    });
    grid.computeDistanceField(ship.draft);
    updateDomainDisplay();
  }

  Object.values(shipInputs).forEach(input => {
    if (!input) return;
    input.addEventListener('change', () => {
      readShipInputs();
      logEvent(`Ship domain parameter updated: ${input.id.replace('param', '')} = ${input.value}`, 'info');
      executeThreeStagePipeline(false);
    });
  });

  // Speed Slider
  if (speedSlider) {
    speedSlider.addEventListener('input', () => {
      simulationSpeed = parseFloat(speedSlider.value);
      if (speedDisplay) speedDisplay.textContent = `${simulationSpeed.toFixed(1)}x`;
    });
  }

  // Simulation Action Buttons
  if (btnRun) {
    btnRun.addEventListener('click', () => {
      if (!ship.activePath || ship.state === 'IDLE' || ship.state === 'ARRIVED') {
        executeThreeStagePipeline(true);
      } else {
        isSimRunning = true;
        btnRun.textContent = 'Running...';
        btnRun.classList.add('active');
        logEvent('Simulation resumed along active ASV path.', 'info');
      }
    });
  }

  if (btnPause) {
    btnPause.addEventListener('click', () => {
      isSimRunning = false;
      if (btnRun) {
        btnRun.textContent = 'Run';
        btnRun.classList.remove('active');
      }
      logEvent('Simulation paused.', 'info');
    });
  }

  if (btnReset) {
    btnReset.addEventListener('click', () => {
      isSimRunning = false;
      if (btnRun) {
        btnRun.textContent = 'Run';
        btnRun.classList.remove('active');
      }
      ship.resetToStart(grid.start);
      executeThreeStagePipeline(false);
      if (statReplans) statReplans.textContent = '0';
      logEvent('ASV reset to Port M (Departure).', 'info');
    });
  }

  if (btnRunPipeline) {
    btnRunPipeline.addEventListener('click', () => {
      executeThreeStagePipeline(false);
    });
  }

  if (btnBenchmark) {
    btnBenchmark.addEventListener('click', () => {
      updateBenchmarkTable();
      logEvent('Ran full multi-algorithm benchmark comparison.', 'info');
    });
  }

  if (btnRandomMap) {
    btnRandomMap.addEventListener('click', () => {
      grid.generateRandomMap();
      ship.resetToStart(grid.start);
      logEvent('Generated random archipelago map with navigational channels.', 'info');
      executeThreeStagePipeline(false);
    });
  }

  if (btnClearObstacles) {
    btnClearObstacles.addEventListener('click', () => {
      grid.init();
      grid.computeDistanceField(ship.draft);
      ship.resetToStart(grid.start);
      logEvent('Cleared all static chart obstacles.', 'info');
      executeThreeStagePipeline(false);
    });
  }

  function handleSpawnAIS() {
    const v = aisSystem.addRandomVessel();
    if (v) {
      logEvent(`AIS Broadcast Registered: ${v.name} (MMSI: ${v.mmsi}) SOG: ${v.sog.toFixed(1)}kn COG: ${v.cog.toFixed(0)}°.`, 'info');
      updateAISTrafficTable();
    }
  }

  if (btnAddAIS) btnAddAIS.addEventListener('click', handleSpawnAIS);
  if (btnSpawnAIS) btnSpawnAIS.addEventListener('click', handleSpawnAIS);

  if (btnClearLog) {
    btnClearLog.addEventListener('click', () => {
      if (eventLogList) eventLogList.innerHTML = '';
      logEvent('Navigation log cleared.', 'info');
    });
  }

  // Initial Startup Run
  readShipInputs();
  executeThreeStagePipeline(false);
  updateAISTrafficTable();
  updateBenchmarkTable();

  // =========================================================================
  // Main Real-Time 60 FPS Loop
  // Dynamic obstacles and AIS vessels update continuously before, during,
  // and after path planning.
  // =========================================================================
  function mainLoop(timestamp) {
    const rawDt = (timestamp - lastFrameTime) / 1000.0;
    lastFrameTime = timestamp;
    const dt = Math.min(0.1, rawDt) * simulationSpeed;

    // 1. AIS traffic and dynamic obstacles move continuously
    aisSystem.update(dt);
    obstacleManager.update(dt);

    // 2. Periodic AIS table refresh (every 0.5s)
    if (timestamp - lastAisTableUpdate > 500) {
      lastAisTableUpdate = timestamp;
      updateAISTrafficTable();
    }

    if (isSimRunning) {
      // 3. Collision risk evaluation & dynamic rerouting
      obstacleManager.checkAndReroute(ship, threeStagePlanner, logEvent);

      // 4. Update ASV kinematics & energy accumulation (with hard grid collision prevention)
      ship.update(dt, grid, grid.cellSizeMeters);

      // 5. Update Live Telemetry
      if (statSpeed) statSpeed.textContent = `${(ship.speed / 0.514444).toFixed(1)} kn`;
      if (statReplans) statReplans.textContent = `${ship.totalReplans}`;

      if (ship.speed > 0.1 && ship.activePath && ship.pathIndex < ship.activePath.length) {
        let remainingCells = 0;
        let lastPt = { x: ship.x, y: ship.y };
        for (let i = ship.pathIndex; i < ship.activePath.length; i++) {
          const pt = ship.activePath[i];
          remainingCells += Math.hypot(pt.x - lastPt.x, pt.y - lastPt.y);
          lastPt = pt;
        }
        const remainingMeters = remainingCells * grid.cellSizeMeters;
        const etaSeconds = remainingMeters / ship.speed;
        const mins = Math.floor(etaSeconds / 60);
        const secs = Math.floor(etaSeconds % 60);
        if (statETA) statETA.textContent = `${mins}m ${secs}s`;
      } else if (ship.state === 'ARRIVED') {
        if (statETA) statETA.textContent = 'PORT N REACHED';
      } else {
        if (statETA) statETA.textContent = '—';
      }

      if (ship.state === 'ARRIVED') {
        isSimRunning = false;
        if (btnRun) {
          btnRun.textContent = 'Run';
          btnRun.classList.remove('active');
        }
        logEvent('Voyage complete: ASV arrived safely at Port N.', 'success');
      }
    }

    // Render Canvas
    renderer.render(dt);

    requestAnimationFrame(mainLoop);
  }

  requestAnimationFrame(mainLoop);
});

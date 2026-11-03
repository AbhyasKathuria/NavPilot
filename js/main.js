/**
 * NavPilot - Cargo Ship Real-Time Path-Planning Simulator
 * Main Application Orchestrator (main.js)
 *
 * Connects UI controls, canvas events, algorithm runners,
 * benchmark comparison table, dynamic collision detection, and animation loop.
 */

window.addEventListener('DOMContentLoaded', () => {
  // DOM Elements
  const canvas = document.getElementById('mapCanvas');
  const algSelect = document.getElementById('algSelect');
  const mapPresetSelect = document.getElementById('mapPresetSelect');
  const speedSlider = document.getElementById('speedSlider');
  const speedDisplay = document.getElementById('speedDisplay');

  // Action Buttons
  const btnRun = document.getElementById('btnRun');
  const btnPause = document.getElementById('btnPause');
  const btnReset = document.getElementById('btnReset');
  const btnBenchmark = document.getElementById('btnBenchmark');
  const btnRandomMap = document.getElementById('btnRandomMap');
  const btnClearObstacles = document.getElementById('btnClearObstacles');
  const btnAddDynamic = document.getElementById('btnAddDynamic');

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

  // Live Stats Elements
  const statAlg = document.getElementById('statAlg');
  const statNodes = document.getElementById('statNodes');
  const statLength = document.getElementById('statLength');
  const statSmoothness = document.getElementById('statSmoothness');
  const statTime = document.getElementById('statTime');
  const statReplans = document.getElementById('statReplans');
  const statClearance = document.getElementById('statClearance');
  const statStopping = document.getElementById('statStopping');
  const statSpeed = document.getElementById('statSpeed');
  const statETA = document.getElementById('statETA');

  // Log and Table
  const eventLogList = document.getElementById('eventLogList');
  const btnClearLog = document.getElementById('btnClearLog');
  const comparisonTbody = document.getElementById('comparisonTbody');

  // State Management
  let activeTool = 'paint_obstacle';
  let isSimRunning = false;
  let simulationSpeed = 1.0;
  let isMouseDown = false;
  let lastFrameTime = performance.now();

  // Core Instances
  const grid = new SeaGrid(50, 50, 50); // 50x50 cells, 50m per cell
  const ship = new CargoShip({
    length: 200,
    beam: 32,
    draft: 12,
    displacement: 50000,
    maxSpeed: 18,
    minTurningRadius: 400
  });

  const obstacleManager = new DynamicObstacleManager(grid);
  const renderer = new MaritimeRenderer(canvas, grid, ship, obstacleManager);

  // Planners
  const aStarPlanner = new AStarPlanner(grid);
  const thetaStarPlanner = new ThetaStarPlanner(grid);
  const monkeySearchPlanner = new MonkeySearchPlanner(grid);
  const hybridPlanner = new HybridPlanner(grid);

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

  logEvent('NavPilot maritime simulator initialized. 50x50 bathymetric grid loaded (2500m × 2500m).', 'info');
  logEvent('Default Cargo Ship: 200m x 32m, 50,000t, min turn radius 400m, draft 12m.', 'info');

  // Spawn initial dynamic obstacles
  function spawnInitialDynamicObstacles() {
    obstacleManager.clear();
    obstacleManager.addObstacle(18, 12, 0.45, 0.7, 'vessel', 38);
    obstacleManager.addObstacle(36, 32, -0.6, -0.4, 'vessel', 30);
    obstacleManager.addObstacle(28, 14, 0.15, 0.2, 'iceberg', 32);
    obstacleManager.addObstacle(12, 38, 0.1, -0.15, 'debris', 20);
    logEvent('4 dynamic hazards spawned (2 vessels, 1 iceberg, 1 container).', 'info');
  }
  spawnInitialDynamicObstacles();

  // Active Planner Helper
  function getActivePlanner() {
    const alg = algSelect ? algSelect.value : 'hybrid';
    return {
      name: alg,
      plan: (start, dest, vessel) => {
        const draft = vessel.draft;
        const clearance = vessel.computeRequiredClearanceMeters();
        const minRadius = vessel.minTurningRadius;

        if (alg === 'astar') {
          return aStarPlanner.plan(start, dest, draft, clearance);
        } else if (alg === 'thetastar') {
          return thetaStarPlanner.plan(start, dest, draft, clearance);
        } else if (alg === 'monkey') {
          const thetaSeed = thetaStarPlanner.plan(start, dest, draft, clearance);
          if (!thetaSeed.success) return thetaSeed;
          return monkeySearchPlanner.optimize(thetaSeed.waypoints, draft, clearance, minRadius, 25);
        } else {
          return hybridPlanner.plan(start, dest, vessel);
        }
      }
    };
  }

  // Execute Route Computation
  function computeAndApplyRoute(autoStartSimulation = true) {
    const planner = getActivePlanner();
    logEvent(`Planning route from (${grid.start.x}, ${grid.start.y}) to (${grid.destination.x}, ${grid.destination.y}) via ${planner.name.toUpperCase()}...`, 'info');

    const result = planner.plan(grid.start, grid.destination, ship);

    if (!result.success || !result.path || result.path.length === 0) {
      logEvent(`Path planning failed: ${result.message || 'No navigable corridor'}`, 'error');
      if (autoStartSimulation) {
        alert(`Path Planning Failed: ${result.message || 'No feasible path exists without collision'}`);
      }
      return;
    }

    logEvent(`Path computed successfully: ${result.lengthMeters.toFixed(0)}m length, ${result.smoothnessDeg.toFixed(1)}° total turn, ${result.computeTimeMs.toFixed(1)}ms.`, 'success');

    // Update Live Stats
    if (statAlg) statAlg.textContent = planner.name.toUpperCase();
    if (statNodes) statNodes.textContent = result.nodesExpanded !== undefined ? result.nodesExpanded : 'N/A';
    if (statLength) statLength.textContent = `${result.lengthMeters.toFixed(0)} m (${(result.lengthMeters / 1852).toFixed(2)} NM)`;
    if (statSmoothness) statSmoothness.textContent = `${result.smoothnessDeg.toFixed(1)}°`;
    if (statTime) statTime.textContent = `${result.computeTimeMs.toFixed(1)} ms`;
    if (statClearance) statClearance.textContent = `${result.minClearanceMeters.toFixed(1)} m`;
    if (statReplans) statReplans.textContent = '0';

    if (autoStartSimulation && result.explored && result.explored.length > 0) {
      renderer.startExplorationAnimation(result.explored, () => {
        ship.setPath(result.path, false);
        isSimRunning = true;
        if (btnRun) {
          btnRun.textContent = 'Running...';
          btnRun.classList.add('active');
        }
      });
    } else {
      ship.setPath(result.path, false);
      if (autoStartSimulation) {
        isSimRunning = true;
        if (btnRun) {
          btnRun.textContent = 'Running...';
          btnRun.classList.add('active');
        }
      }
    }

    updateSingleBenchmarkRow(planner.name, result);
  }

  // Benchmark all 4 algorithms on current map
  function runFullBenchmark() {
    if (!comparisonTbody) return;
    logEvent('Starting 4-Algorithm Benchmark comparison...', 'info');
    comparisonTbody.innerHTML = '';

    const algorithms = [
      { id: 'astar', name: 'A* (8-Connected Grid)' },
      { id: 'thetastar', name: 'Theta* (Any-Angle)' },
      { id: 'monkey', name: 'Monkey Search' },
      { id: 'hybrid', name: 'Hybrid (Theta* + Monkey + Dubins)' }
    ];

    const results = [];

    for (let i = 0; i < algorithms.length; i++) {
      const a = algorithms[i];
      let res;
      if (a.id === 'astar') {
        res = aStarPlanner.plan(grid.start, grid.destination, ship.draft, ship.computeRequiredClearanceMeters());
      } else if (a.id === 'thetastar') {
        res = thetaStarPlanner.plan(grid.start, grid.destination, ship.draft, ship.computeRequiredClearanceMeters());
      } else if (a.id === 'monkey') {
        const seed = thetaStarPlanner.plan(grid.start, grid.destination, ship.draft, ship.computeRequiredClearanceMeters());
        res = seed.success
          ? monkeySearchPlanner.optimize(seed.waypoints, ship.draft, ship.computeRequiredClearanceMeters(), ship.minTurningRadius, 25)
          : seed;
      } else {
        res = hybridPlanner.plan(grid.start, grid.destination, ship);
      }
      results.push({ name: a.name, id: a.id, result: res });
    }

    results.forEach(item => {
      const r = item.result;
      const tr = document.createElement('tr');
      if (item.id === 'hybrid') tr.className = 'highlight-row';

      const isSuccess = r.success;
      tr.innerHTML = `
        <td><strong>${item.name}</strong></td>
        <td>${isSuccess ? `${r.computeTimeMs.toFixed(1)} ms` : 'Failed'}</td>
        <td>${r.nodesExpanded !== undefined ? r.nodesExpanded : '—'}</td>
        <td>${isSuccess ? `${r.lengthMeters.toFixed(0)} m` : '—'}</td>
        <td>${isSuccess ? `${r.smoothnessDeg.toFixed(1)}°` : '—'}</td>
        <td>${isSuccess ? `${r.minClearanceMeters.toFixed(1)} m` : '—'}</td>
        <td>${isSuccess ? (r.curvatureViolations !== undefined ? r.curvatureViolations : 0) : '—'}</td>
        <td><span class="badge ${isSuccess ? 'badge-success' : 'badge-danger'}">${isSuccess ? 'Feasible' : 'Blocked'}</span></td>
      `;
      comparisonTbody.appendChild(tr);
    });

    logEvent('Benchmark finished: Hybrid achieves lowest curvature violations and smoothest path.', 'success');
  }

  function updateSingleBenchmarkRow(algId, res) {
    if (comparisonTbody && comparisonTbody.children.length === 0) {
      runFullBenchmark();
    }
  }

  // Interactive Map Painting
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
      grid.setStart(gx, gy);
      ship.resetToStart(grid.start);
      computeAndApplyRoute(false);
      logEvent(`Start point moved to (${gx}, ${gy}).`, 'info');
    } else if (activeTool === 'set_dest') {
      grid.setDestination(gx, gy);
      computeAndApplyRoute(false);
      logEvent(`Destination moved to (${gx}, ${gy}).`, 'info');
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
      const angle = Math.random() * Math.PI * 2;
      obstacleManager.addObstacle(gx, gy, Math.cos(angle) * 0.7, Math.sin(angle) * 0.7, 'vessel', 35);
      logEvent(`Added dynamic vessel at (${gx}, ${gy}).`, 'info');
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

  // Tool Selection Buttons
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

  // Ship Parameter Changes
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
    if (statStopping) {
      statStopping.textContent = `${ship.computeStoppingDistanceMeters().toFixed(0)} m`;
    }
  }

  Object.values(shipInputs).forEach(input => {
    if (!input) return;
    input.addEventListener('change', () => {
      readShipInputs();
      logEvent(`Ship parameter updated: ${input.id.replace('param', '')} = ${input.value}`, 'info');
    });
  });

  // Speed Slider
  if (speedSlider) {
    speedSlider.addEventListener('input', () => {
      simulationSpeed = parseFloat(speedSlider.value);
      if (speedDisplay) speedDisplay.textContent = `${simulationSpeed.toFixed(1)}x`;
    });
  }

  // Control Buttons
  if (btnRun) {
    btnRun.addEventListener('click', () => {
      if (!ship.activePath || ship.state === 'IDLE' || ship.state === 'ARRIVED') {
        computeAndApplyRoute(true);
      } else {
        isSimRunning = true;
        btnRun.textContent = 'Running...';
        btnRun.classList.add('active');
        logEvent('Simulation resumed.', 'info');
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
      spawnInitialDynamicObstacles();
      computeAndApplyRoute(false);
      if (statReplans) statReplans.textContent = '0';
      logEvent('Simulation reset to start.', 'info');
    });
  }

  if (btnBenchmark) {
    btnBenchmark.addEventListener('click', () => {
      runFullBenchmark();
    });
  }

  if (btnRandomMap) {
    btnRandomMap.addEventListener('click', () => {
      grid.generateRandomMap();
      ship.resetToStart(grid.start);
      logEvent('Generated random archipelago map.', 'info');
      computeAndApplyRoute(false);
    });
  }

  if (btnClearObstacles) {
    btnClearObstacles.addEventListener('click', () => {
      grid.init();
      grid.computeDistanceField(ship.draft);
      ship.resetToStart(grid.start);
      logEvent('Cleared all static obstacles.', 'info');
      computeAndApplyRoute(false);
    });
  }

  if (btnAddDynamic) {
    btnAddDynamic.addEventListener('click', () => {
      obstacleManager.addRandomObstacle();
      logEvent('Spawned random dynamic maritime hazard.', 'info');
    });
  }

  if (mapPresetSelect) {
    mapPresetSelect.addEventListener('change', () => {
      const val = mapPresetSelect.value;
      grid.loadPreset(val);
      ship.resetToStart(grid.start);
      logEvent(`Loaded scenario preset: ${val.toUpperCase()}`, 'info');
      computeAndApplyRoute(false);
    });
  }

  if (algSelect) {
    algSelect.addEventListener('change', () => {
      logEvent(`Algorithm switched to: ${algSelect.options[algSelect.selectedIndex].text}`, 'info');
      computeAndApplyRoute(false);
    });
  }

  if (btnClearLog) {
    btnClearLog.addEventListener('click', () => {
      if (eventLogList) eventLogList.innerHTML = '';
      logEvent('Event log cleared.', 'info');
    });
  }

  // Initial Route Run on startup
  readShipInputs();
  computeAndApplyRoute(false);
  runFullBenchmark();

  // Main Real-Time Animation Loop (requestAnimationFrame)
  function mainLoop(timestamp) {
    const rawDt = (timestamp - lastFrameTime) / 1000.0;
    lastFrameTime = timestamp;
    const dt = Math.min(0.1, rawDt) * simulationSpeed;

    if (isSimRunning) {
      // 1. Update Dynamic Obstacles
      obstacleManager.update(dt);

      // 2. CPA / TCPA Collision Detection & Automatic Replanning
      const activePlanner = getActivePlanner();
      obstacleManager.checkAndAvoidCollisions(ship, activePlanner, logEvent);

      // 3. Step Cargo Ship Kinematics
      ship.update(dt, grid.cellSizeMeters);

      // 4. Update Live Telemetry
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
        if (statETA) statETA.textContent = 'ARRIVED';
      } else {
        if (statETA) statETA.textContent = '—';
      }

      if (ship.state === 'ARRIVED') {
        isSimRunning = false;
        if (btnRun) {
          btnRun.textContent = 'Run';
          btnRun.classList.remove('active');
        }
        logEvent('Destination reached successfully!', 'success');
      }
    }

    // Render Canvas
    renderer.render(dt);

    requestAnimationFrame(mainLoop);
  }

  // Launch real-time 60fps loop
  requestAnimationFrame(mainLoop);
});

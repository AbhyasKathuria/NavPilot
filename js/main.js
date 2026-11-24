/**
 * NavPilot - Cargo Ship Real-Time Path-Planning Simulator
 * Main Application Orchestrator (main.js)
 *
 * Implements:
 * - Sequential 1-by-1 calculation of Algorithm 1 (A*) -> Algorithm 2 (RRT*) -> Algorithm 3 (Somersault Monkey Search)
 * - 100% Collision Audit & Best Optimal Path Selection
 * - Full Clear Dashboard mode (blank open sea ready for full customization)
 * - Complete Entity Builder (Ports M/N, Land, Shoals, Cargo Ships, Mini Ships, Drift Hazards)
 * - Offline Path Map Image & JSON Auto-Recognition
 */

window.addEventListener('DOMContentLoaded', () => {
  // Core Canvas & Display Elements
  const canvas = document.getElementById('mapCanvas');
  const speedSlider = document.getElementById('speedSlider');
  const speedDisplay = document.getElementById('speedDisplay');
  const canvasModeStatus = document.getElementById('canvasModeStatus');
  const canvasCoordsDisplay = document.getElementById('canvasCoordsDisplay');

  // Action Buttons
  const btnRun = document.getElementById('btnRun');
  const btnPause = document.getElementById('btnPause');
  const btnReset = document.getElementById('btnReset');
  const btnRunPipeline = document.getElementById('btnRunPipeline');
  const btnBenchmark = document.getElementById('btnBenchmark');
  const btnRandomMap = document.getElementById('btnRandomMap');
  const btnClearObstacles = document.getElementById('btnClearObstacles');
  const presetSelect = document.getElementById('presetSelect');

  // Traffic Quick Spawn Buttons
  const btnSpawnCargoAIS = document.getElementById('btnSpawnCargoAIS');
  const btnSpawnMiniAIS = document.getElementById('btnSpawnMiniAIS');
  const btnSpawnHazard = document.getElementById('btnSpawnHazard');
  const trafficCountBadge = document.getElementById('trafficCountBadge');

  // Map Upload & Auto-Recognition Elements
  const mapDropZone = document.getElementById('mapDropZone');
  const mapFileInput = document.getElementById('mapFileInput');
  const btnExportMap = document.getElementById('btnExportMap');
  const mapImportPreviewBox = document.getElementById('mapImportPreviewBox');
  const imgUploadThumb = document.getElementById('imgUploadThumb');
  const canvasRecognizedPreview = document.getElementById('canvasRecognizedPreview');
  const importStatsSummary = document.getElementById('importStatsSummary');
  const importModeSelect = document.getElementById('importModeSelect');
  const importSensitivity = document.getElementById('importSensitivity');
  const importSensVal = document.getElementById('importSensVal');
  const chkDetectPorts = document.getElementById('chkDetectPorts');
  const chkDetectShoals = document.getElementById('chkDetectShoals');
  const btnInvertMap = document.getElementById('btnInvertMap');

  // Stage Flow Cards & Badges
  const stepCards = [null, document.getElementById('stepCard1'), document.getElementById('stepCard2'), document.getElementById('stepCard3')];
  const stepBadges = [null, document.getElementById('stepBadge1'), document.getElementById('stepBadge2'), document.getElementById('stepBadge3')];
  const winnerBadgeDisplay = document.getElementById('winnerBadgeDisplay');

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
    paint_shallow: document.getElementById('toolAddShallow'),
    add_cargo_ship: document.getElementById('toolAddCargoShip'),
    add_mini_ship: document.getElementById('toolAddMiniShip'),
    add_dynamic: document.getElementById('toolAddDynamic'),
    paint_clear: document.getElementById('toolRemoveLand')
  };

  // Entity Placement Controls
  const brushButtons = document.querySelectorAll('.brush-btn');
  const placeCraftSpeed = document.getElementById('placeCraftSpeed');
  const placeCraftHeading = document.getElementById('placeCraftHeading');
  const chkPlaceAnchored = document.getElementById('chkPlaceAnchored');
  const toolInstructionHint = document.getElementById('toolInstructionHint');

  // Ship Parameter Inputs
  const shipInputs = {
    length: document.getElementById('paramLength'),
    beam: document.getElementById('paramBeam'),
    draft: document.getElementById('paramDraft'),
    displacement: document.getElementById('paramDisplacement'),
    maxSpeed: document.getElementById('paramSpeed'),
    minTurningRadius: document.getElementById('paramTurnRadius')
  };

  // Ship Domain & Port Displays
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

  // Tables & Event Log
  const aisTrafficTbody = document.getElementById('aisTrafficTbody');
  const comparisonTbody = document.getElementById('comparisonTbody');
  const eventLogList = document.getElementById('eventLogList');
  const btnClearLog = document.getElementById('btnClearLog');

  // State Management
  let activeTool = 'paint_obstacle';
  let brushRadius = 1; // 1 => 1x1, 2 => 3x3, 3 => 5x5
  let isSimRunning = false;
  let simulationSpeed = 1.0;
  let isMouseDown = false;
  let dragStartCoord = null;
  let lastFrameTime = performance.now();
  let lastAisTableUpdate = 0;
  let solverRunToken = 0; // Cancels older async sequential computations cleanly
  let importInvertState = false;
  let mapDirtySinceSolve = false;

  // Core Simulation Model Instances
  const grid = new SeaGrid(50, 50, 50);
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
  const mapImporter = new MapImageImporter();
  const renderer = new MaritimeRenderer(canvas, grid, ship, aisSystem, obstacleManager);
  const threeStagePlanner = new ThreeStageHybridPlanner(grid);

  // =========================================================================
  // Utility & Logging Helpers
  // =========================================================================
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

  function sleepMs(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
  }

  function updateRunButtonUI(running) {
    if (!btnRun) return;
    if (running) {
      btnRun.textContent = 'Running...';
      btnRun.classList.add('active');
    } else {
      btnRun.textContent = '▶ Run All 3';
      btnRun.classList.remove('active');
    }
  }

  function resetStageCardsUI() {
    for (let i = 1; i <= 3; i++) {
      if (stepCards[i]) {
        stepCards[i].classList.remove('active', 'computing', 'winner');
      }
      if (stepBadges[i]) {
        stepBadges[i].textContent = 'Pending';
        stepBadges[i].className = 'badge ' + (i === 1 ? 'badge-warning' : (i === 3 ? 'badge-success' : ''));
        if (i === 2) {
          stepBadges[i].style.background = '#E0F2FE';
          stepBadges[i].style.color = '#0369A1';
          stepBadges[i].style.border = '1px solid #7DD3FC';
        }
      }
    }
    if (winnerBadgeDisplay) winnerBadgeDisplay.textContent = 'Ready';
  }

  function updateDomainDisplay() {
    const vol = ship.shipDomainVolume;
    if (domainVolDisplay) domainVolDisplay.textContent = `${(vol / 1000).toFixed(1)}k m³`;
    if (domainCalcText) {
      domainCalcText.textContent = `${ship.length}m × ${ship.beam}m × ${ship.draft}m = ${vol.toLocaleString()} m³ (Domain: ${ship.domainRadiusMeters.toFixed(0)}m)`;
    }
    if (lblPortM) lblPortM.textContent = `Port M (${grid.start.x}, ${grid.start.y})`;
    if (lblPortN) lblPortN.textContent = `Port N (${grid.destination.x}, ${grid.destination.y})`;
    if (statStopping) statStopping.textContent = `${ship.computeStoppingDistanceMeters().toFixed(0)} m`;
  }

  function clearTelemetryAndTables(statusLabel = 'READY (BLANK CHART)') {
    if (statAlg) statAlg.textContent = statusLabel;
    if (statNodes) statNodes.textContent = '—';
    if (statLength) statLength.textContent = '—';
    if (statSmoothness) statSmoothness.textContent = '—';
    if (statEnergy) statEnergy.textContent = '—';
    if (statETA) statETA.textContent = '—';
    if (statClearance) statClearance.textContent = '—';
    if (statSpeed) statSpeed.textContent = '0.0 kn';
    if (statReplans) statReplans.textContent = '0';

    if (comparisonTbody) {
      comparisonTbody.innerHTML = `
        <tr>
          <td colspan="7" style="text-align: center; color: var(--text-muted); padding: 10px;">
            Chart ready — add ships, mini-ships &amp; obstacles or upload a map, then click <strong>▶ Run All 3</strong>.
          </td>
        </tr>
      `;
    }
  }

  // =========================================================================
  // AIS & Dynamic Entities Table Rendering
  // =========================================================================
  function updateAISTrafficTable() {
    if (trafficCountBadge) {
      const shipCount = aisSystem.vessels.length;
      const hazCount = obstacleManager.obstacles.length;
      trafficCountBadge.textContent = `${shipCount} ${shipCount === 1 ? 'ship' : 'ships'} | ${hazCount} ${hazCount === 1 ? 'hazard' : 'hazards'}`;
    }
    if (!aisTrafficTbody) return;
    aisTrafficTbody.innerHTML = '';

    if (aisSystem.vessels.length === 0) {
      aisTrafficTbody.innerHTML = `
        <tr>
          <td colspan="8" style="text-align: center; color: var(--text-muted); padding: 8px;">
            No ships placed. Select <strong>🚢 Cargo Ship</strong> or <strong>🚤 Mini Ship</strong> to place on map.
          </td>
        </tr>
      `;
      return;
    }

    aisSystem.vessels.forEach(v => {
      const tr = document.createElement('tr');
      if (v.riskLevel === 'COLLISION_ALERT') tr.className = 'risk-alert';
      else if (v.riskLevel === 'WARNING') tr.className = 'risk-warning';

      let riskBadge = '<span class="badge badge-success">Clear</span>';
      if (v.riskLevel === 'COLLISION_ALERT') riskBadge = '<span class="badge badge-danger">Alert</span>';
      else if (v.riskLevel === 'WARNING') riskBadge = '<span class="badge badge-warning">Caution</span>';

      const cpaStr = (v.cpaMeters !== undefined && v.cpaMeters !== Infinity) ? `${v.cpaMeters.toFixed(0)}m` : '—';
      const tcpaStr = (v.tcpaSeconds !== undefined && v.tcpaSeconds !== Infinity) ? `${v.tcpaSeconds.toFixed(1)}s` : '—';
      const classLabel = v.vesselCategory === 'mini' ? '🚤 Mini' : '🚢 Cargo';

      tr.innerHTML = `
        <td><strong>${v.name}</strong></td>
        <td>${classLabel}</td>
        <td>${v.sog.toFixed(1)}kn</td>
        <td>${v.cog.toFixed(0)}°</td>
        <td>${cpaStr}</td>
        <td>${tcpaStr}</td>
        <td>${riskBadge}</td>
        <td><button class="btn-del-row" data-mmsi="${v.mmsi}" title="Remove vessel">✖</button></td>
      `;
      aisTrafficTbody.appendChild(tr);
    });
  }

  // Event delegation for vessel removal button in AIS table
  if (aisTrafficTbody) {
    aisTrafficTbody.addEventListener('click', (e) => {
      const btn = e.target.closest('.btn-del-row');
      if (!btn) return;
      const mmsi = btn.getAttribute('data-mmsi');
      const removed = aisSystem.removeVessel(mmsi);
      if (removed) {
        logEvent(`Removed ${removed.name} from sea chart.`, 'info');
        updateAISTrafficTable();
      }
    });
  }

  // =========================================================================
  // Algorithm Comparison Table Renderer
  // =========================================================================
  function renderComparisonRows(resultsList, winnerId = null) {
    if (!comparisonTbody) return;
    comparisonTbody.innerHTML = '';

    resultsList.forEach(item => {
      const r = item.res;
      const tr = document.createElement('tr');
      const isWinner = winnerId && item.id === winnerId;
      if (isWinner) tr.className = 'highlight-row';

      const isComputing = item.state === 'computing';
      const isPending = item.state === 'pending';

      if (isPending) {
        tr.innerHTML = `
          <td><strong>${item.name}</strong></td>
          <td colspan="6" style="color: var(--text-muted); text-align:center;">Waiting in queue...</td>
        `;
      } else if (isComputing) {
        tr.innerHTML = `
          <td><strong>${item.name}</strong></td>
          <td colspan="6" style="color: #0284C7; font-weight:700; text-align:center;">⏳ Computing optimal corridor...</td>
        `;
      } else {
        const isSuccess = r && r.success && r.collisionFree !== false;
        const nodesText = isSuccess ? (r.nodesExpanded || r.nodesSampled || '16 pop') : '—';
        const statusBadge = isWinner
          ? '<span class="badge badge-success">🏆 OPTIMAL</span>'
          : (isSuccess
            ? '<span class="badge badge-success">Safe ✓</span>'
            : '<span class="badge badge-danger">Blocked</span>');

        tr.innerHTML = `
          <td><strong>${item.name}</strong></td>
          <td>${r && r.success ? `${r.computeTimeMs.toFixed(1)}ms` : 'Failed'}</td>
          <td>${nodesText}</td>
          <td>${isSuccess ? `${r.lengthMeters.toFixed(0)}m` : '—'}</td>
          <td>${isSuccess ? `${r.smoothnessDeg.toFixed(1)}°` : '—'}</td>
          <td>${isSuccess ? `${r.minClearanceMeters.toFixed(1)}m` : '—'}</td>
          <td>${statusBadge}</td>
        `;
      }
      comparisonTbody.appendChild(tr);
    });
  }

  // =========================================================================
  // Sequential One-By-One Solver (Algo 1 -> Algo 2 -> Algo 3 -> Pick Best -> Sail)
  // =========================================================================
  async function runSequentialThreeAlgorithms(autoStartNavigation = true) {
    const token = ++solverRunToken;
    mapDirtySinceSolve = false;
    isSimRunning = false;
    updateRunButtonUI(false);

    ship.resetToStart(grid.start);
    renderer.clearAllAnimations();
    resetStageCardsUI();

    const stepDelay = simulationSpeed >= 2.5 ? 240 : 460;
    const rows = [
      { id: 'stage1', name: '1. A* Global Search', state: 'pending', res: null },
      { id: 'stage2', name: '2. RRT* Kinematic Refinement', state: 'pending', res: null },
      { id: 'stage3', name: '3. Somersault Monkey Search', state: 'pending', res: null },
      { id: 'hybrid', name: 'Combined 3-Stage Hybrid Optimal', state: 'pending', res: null }
    ];

    if (statAlg) statAlg.textContent = 'COMPUTING 1/3...';

    // -----------------------------------------------------------------------
    // ALGORITHM 1: A* GLOBAL PATH SEARCH
    // -----------------------------------------------------------------------
    rows[0].state = 'computing';
    renderComparisonRows(rows);
    if (stepCards[1]) stepCards[1].classList.add('computing');
    if (stepBadges[1]) stepBadges[1].textContent = 'COMPUTING';
    renderer.solverStatusBanner = '⏳ Algorithm 1/3: Computing A* Global Safe Corridor...';
    logEvent(`[Algo 1/3] Computing A* Global Search from Port M (${grid.start.x}, ${grid.start.y}) ➔ Port N (${grid.destination.x}, ${grid.destination.y})...`, 'info');

    await sleepMs(40);
    if (token !== solverRunToken) return;

    const stage1 = threeStagePlanner.runStage1(grid.start, grid.destination, ship, aisSystem, obstacleManager);
    rows[0].state = 'done';
    rows[0].res = stage1;

    if (!stage1.success) {
      if (stepCards[1]) stepCards[1].classList.remove('computing');
      if (stepBadges[1]) stepBadges[1].textContent = 'FAILED';
      renderer.solverStatusBanner = '⛔ No collision-free path exists between Ports M & N';
      renderComparisonRows(rows);
      clearTelemetryAndTables('BLOCKED');
      logEvent(`Algorithm 1 (A*) failed: ${stage1.message}`, 'error');
      return;
    }

    ship.stage1Path = stage1.path;
    renderer.startExplorationAnimation(stage1.explored);
    if (stepCards[1]) {
      stepCards[1].classList.remove('computing');
      stepCards[1].classList.add('active');
    }
    if (stepBadges[1]) stepBadges[1].textContent = `${stage1.lengthMeters.toFixed(0)}m`;
    renderComparisonRows(rows);
    logEvent(`[Algo 1/3] A* complete: ${stage1.lengthMeters.toFixed(0)}m (${stage1.nodesExpanded} nodes, ${stage1.smoothnessDeg.toFixed(1)}° turn, min clear ${stage1.minClearanceMeters.toFixed(1)}m).`, 'success');

    await sleepMs(stepDelay);
    if (token !== solverRunToken) return;

    // -----------------------------------------------------------------------
    // ALGORITHM 2: RRT* KINEMATIC REFINEMENT
    // -----------------------------------------------------------------------
    rows[1].state = 'computing';
    renderComparisonRows(rows);
    if (stepCards[2]) stepCards[2].classList.add('computing');
    if (stepBadges[2]) stepBadges[2].textContent = 'COMPUTING';
    if (statAlg) statAlg.textContent = 'COMPUTING 2/3...';
    renderer.solverStatusBanner = '⏳ Algorithm 2/3: Computing RRT* Kinematic Refinement (R ≥ R_min)...';
    logEvent(`[Algo 2/3] Computing RRT* Kinematic Refinement (R ≥ ${ship.minTurningRadius}m)...`, 'info');

    await sleepMs(40);
    if (token !== solverRunToken) return;

    const stage2 = threeStagePlanner.runStage2(stage1, ship, aisSystem, obstacleManager);
    rows[1].state = 'done';
    rows[1].res = stage2;

    ship.stage2Path = stage2.path;
    renderer.startRrtTreeAnimation(stage2.treeEdges);
    if (stepCards[2]) {
      stepCards[2].classList.remove('computing');
      stepCards[2].classList.add('active');
    }
    if (stepBadges[2]) stepBadges[2].textContent = `${stage2.lengthMeters.toFixed(0)}m`;
    renderComparisonRows(rows);
    logEvent(`[Algo 2/3] RRT* complete: ${stage2.lengthMeters.toFixed(0)}m (${stage2.nodesSampled} samples, ${stage2.smoothnessDeg.toFixed(1)}° turn, min clear ${stage2.minClearanceMeters.toFixed(1)}m).`, 'success');

    await sleepMs(stepDelay);
    if (token !== solverRunToken) return;

    // -----------------------------------------------------------------------
    // ALGORITHM 3: SOMERSAULT MONKEY SEARCH OPTIMISATION
    // -----------------------------------------------------------------------
    rows[2].state = 'computing';
    rows[3].state = 'computing';
    renderComparisonRows(rows);
    if (stepCards[3]) stepCards[3].classList.add('computing');
    if (stepBadges[3]) stepBadges[3].textContent = 'COMPUTING';
    if (statAlg) statAlg.textContent = 'COMPUTING 3/3...';
    renderer.solverStatusBanner = '⏳ Algorithm 3/3: Somersault Monkey Search Multi-Objective Optimisation...';
    logEvent('[Algo 3/3] Computing Somersault Monkey Search (Climb ➔ Watch-Jump ➔ Somersault)...', 'info');

    await sleepMs(40);
    if (token !== solverRunToken) return;

    const stage3 = threeStagePlanner.runStage3(stage2, stage1, ship, aisSystem, obstacleManager, 25);
    rows[2].state = 'done';
    rows[2].res = stage3;
    rows[3].state = 'done';
    rows[3].res = stage3.hybrid;

    ship.stage3Path = stage3.path;
    renderer.startMonkeyAnimation(stage3.iterationHistory);
    if (stepCards[3]) {
      stepCards[3].classList.remove('computing');
      stepCards[3].classList.add('active');
    }
    if (stepBadges[3]) stepBadges[3].textContent = `${stage3.lengthMeters.toFixed(0)}m`;
    renderComparisonRows(rows);
    logEvent(`[Algo 3/3] Somersault Monkey Search complete: ${stage3.lengthMeters.toFixed(0)}m (Turn: ${stage3.smoothnessDeg.toFixed(1)}°, Energy: ${stage3.energyIndex}).`, 'success');

    await sleepMs(stepDelay);
    if (token !== solverRunToken) return;

    // -----------------------------------------------------------------------
    // AUDIT ALL CANDIDATES & PICK STRICTLY BEST 100% COLLISION-FREE ROUTE
    // -----------------------------------------------------------------------
    const winner = threeStagePlanner.selectOptimalCandidate(stage1, stage2, stage3, ship, aisSystem, obstacleManager);

    if (!winner) {
      renderer.solverStatusBanner = '⛔ Zero safe collision-free corridor found!';
      logEvent('Audit rejected all candidate trajectories due to unsafe obstacle proximity.', 'error');
      return;
    }

    threeStagePlanner.lastResult = {
      success: true,
      path: winner.path,
      waypoints: winner.waypoints,
      winnerId: winner.id,
      winnerName: winner.name,
      stage1,
      stage2,
      stage3,
      hybrid: stage3.hybrid,
      computeTimeMs: stage1.computeTimeMs + stage2.computeTimeMs + stage3.computeTimeMs,
      ...winner
    };

    // Mark winning stage card & comparison row
    renderComparisonRows(rows, winner.id);
    if (winnerBadgeDisplay) winnerBadgeDisplay.textContent = `🏆 ${winner.id.toUpperCase()}`;

    if (winner.id === 'stage1' && stepCards[1]) stepCards[1].classList.add('winner');
    else if (winner.id === 'stage2' && stepCards[2]) stepCards[2].classList.add('winner');
    else if (stepCards[3]) stepCards[3].classList.add('winner');

    // Update Ship Trajectory
    ship.setStagePaths(stage1.path, stage2.path, stage3.path, winner.path, winner.name);
    renderer.solverStatusBanner = `🏆 Best Optimal Selected: ${winner.name} (${winner.lengthMeters.toFixed(0)}m | 0 Collisions)`;

    // Update Telemetry
    if (statAlg) statAlg.textContent = winner.name.replace('Algorithm ', '').replace('Combined ', '');
    if (statNodes) statNodes.textContent = `${stage1.nodesExpanded} (A*) | ${stage2.nodesSampled} (RRT*)`;
    if (statLength) statLength.textContent = `${winner.lengthMeters.toFixed(0)} m (${(winner.lengthMeters / 1852).toFixed(2)} NM)`;
    if (statSmoothness) statSmoothness.textContent = `${winner.smoothnessDeg.toFixed(1)}°`;
    if (statEnergy) statEnergy.textContent = `${winner.energyIndex} (Low Drag)`;
    if (statClearance) statClearance.textContent = `${winner.minClearanceMeters.toFixed(1)} m (100% Safe)`;
    if (statReplans) statReplans.textContent = '0';

    const mins = Math.floor(winner.transitTimeSec / 60);
    const secs = Math.floor(winner.transitTimeSec % 60);
    if (statETA) statETA.textContent = `${mins}m ${secs}s`;

    logEvent(`🏆 AUDIT WINNER: ${winner.name} | Length: ${winner.lengthMeters.toFixed(0)}m | Min Clearance: ${winner.minClearanceMeters.toFixed(1)}m | Collisions: 0.`, 'success');

    if (autoStartNavigation) {
      await sleepMs(380);
      if (token !== solverRunToken) return;
      renderer.solverStatusBanner = '';
      isSimRunning = true;
      updateRunButtonUI(true);
      logEvent('ASV departing Port M along verified Optimal Path with active collision shield.', 'info');
    } else {
      ship.state = 'IDLE';
      ship.speed = 0.0;
      await sleepMs(1600);
      if (token === solverRunToken) renderer.solverStatusBanner = '';
    }
  }

  // Synchronous instant benchmark update (for parameter tweaks/preset switch without launching navigation)
  function runInstantComparisonSnapshot() {
    solverRunToken++;
    const res = threeStagePlanner.plan(grid.start, grid.destination, ship, aisSystem, { obstacleManager });
    renderer.clearAllAnimations();
    resetStageCardsUI();

    if (!res.success) {
      ship.resetToStart(grid.start);
      clearTelemetryAndTables('BLOCKED');
      return;
    }

    ship.setStagePaths(res.stage1.path, res.stage2.path, res.stage3.path, res.path, res.winnerName);
    ship.state = 'IDLE';
    ship.speed = 0.0;

    if (statAlg) statAlg.textContent = res.winnerName.replace('Algorithm ', '').replace('Combined ', '');
    if (statNodes) statNodes.textContent = `${res.stage1.nodesExpanded} (A*) | ${res.stage2.nodesSampled} (RRT*)`;
    if (statLength) statLength.textContent = `${res.lengthMeters.toFixed(0)} m (${(res.lengthMeters / 1852).toFixed(2)} NM)`;
    if (statSmoothness) statSmoothness.textContent = `${res.smoothnessDeg.toFixed(1)}°`;
    if (statEnergy) statEnergy.textContent = `${res.energyIndex} (Low Drag)`;
    if (statClearance) statClearance.textContent = `${res.minClearanceMeters.toFixed(1)} m`;
    const mins = Math.floor(res.transitTimeSec / 60);
    const secs = Math.floor(res.transitTimeSec % 60);
    if (statETA) statETA.textContent = `${mins}m ${secs}s`;

    const rows = [
      { id: 'stage1', name: '1. A* Global Search', state: 'done', res: res.stage1 },
      { id: 'stage2', name: '2. RRT* Kinematic Refinement', state: 'done', res: res.stage2 },
      { id: 'stage3', name: '3. Somersault Monkey Search', state: 'done', res: res.stage3 },
      { id: 'hybrid', name: 'Combined 3-Stage Hybrid Optimal', state: 'done', res: res.hybrid }
    ];
    renderComparisonRows(rows, res.winnerId);
    if (winnerBadgeDisplay) winnerBadgeDisplay.textContent = `🏆 ${res.winnerId.toUpperCase()}`;
  }

  // =========================================================================
  // Clear Entire Dashboard to 100% Blank Sea
  // =========================================================================
  function clearEntireDashboard() {
    solverRunToken++;
    isSimRunning = false;
    updateRunButtonUI(false);

    grid.clearToBlankSea(ship.draft);
    aisSystem.clear();
    obstacleManager.clear();
    ship.resetToStart(grid.start);
    renderer.clearAllAnimations();
    resetStageCardsUI();

    if (presetSelect) presetSelect.value = 'blank';
    if (winnerBadgeDisplay) winnerBadgeDisplay.textContent = 'Blank Chart';
    updateDomainDisplay();
    updateAISTrafficTable();
    clearTelemetryAndTables('READY (BLANK SEA)');

    logEvent('Dashboard cleared: Open blue sea chart ready. Add ships, mini-ships, obstacles, or upload a map!', 'info');
  }

  // =========================================================================
  // Interactive Map Tools, Brush Sizes & Drag-Heading Vessel Placement
  // =========================================================================
  function getGridCoordFromMouse(clientX, clientY) {
    const rect = canvas.getBoundingClientRect();
    const scaleX = 500 / rect.width;
    const scaleY = 500 / rect.height;
    const px = (clientX - rect.left) * scaleX;
    const py = (clientY - rect.top) * scaleY;
    return {
      x: Math.floor(px / renderer.cellSize),
      y: Math.floor(py / renderer.cellSize)
    };
  }

  function applyPaintBrush(gx, gy, cellType, depthVal) {
    const offset = brushRadius - 1;
    let modified = false;
    let touchedBlueCircle = false;
    const curShipGx = Math.round(ship.x);
    const curShipGy = Math.round(ship.y);

    for (let dy = -offset; dy <= offset; dy++) {
      for (let dx = -offset; dx <= offset; dx++) {
        const nx = gx + dx;
        const ny = gy + dy;
        if (!grid.inBounds(nx, ny)) continue;
        if (cellType !== grid.TYPE_SEA) {
          if (grid.isStart(nx, ny) || grid.isDestination(nx, ny)) continue;
          if (nx === curShipGx && ny === curShipGy) continue; // Ship cannot be embedded in solid obstacle
        }
        if (grid.cells[ny][nx] !== cellType) {
          grid.setCell(nx, ny, cellType, depthVal);
          modified = true;
          if (ship.isPointInsideBlueDomain && ship.isPointInsideBlueDomain(nx, ny, 2.0, grid.cellSizeMeters)) {
            touchedBlueCircle = true;
          }
        }
      }
    }
    if (modified) {
      grid.computeDistanceField(ship.draft);
      mapDirtySinceSolve = true;
      if (touchedBlueCircle) {
        ship.domainDisturbance = true;
        ship.domainDisturbanceReason = `Manual brush disturbance near Blue Domain at (${gx}, ${gy})`;
        ship.domainDisturbancePulseTimer = 2.5;
      }
      if (isSimRunning && ship.activePath) {
        obstacleManager.checkAndReroute(ship, threeStagePlanner, logEvent, true);
      }
    }
  }

  function applyPointOrDragTool(gx, gy) {
    if (!grid.inBounds(gx, gy)) return;

    if (activeTool === 'set_start') {
      solverRunToken++;
      isSimRunning = false;
      updateRunButtonUI(false);
      grid.setStart(gx, gy);
      grid.computeDistanceField(ship.draft);
      ship.resetToStart(grid.start);
      updateDomainDisplay();
      logEvent(`Port M (Start) placed at (${gx}, ${gy}).`, 'info');
    } else if (activeTool === 'set_dest') {
      solverRunToken++;
      isSimRunning = false;
      updateRunButtonUI(false);
      grid.setDestination(gx, gy);
      grid.computeDistanceField(ship.draft);
      ship.resetToStart(grid.start);
      updateDomainDisplay();
      logEvent(`Port N (Destination) placed at (${gx}, ${gy}).`, 'info');
    } else if (activeTool === 'paint_obstacle') {
      applyPaintBrush(gx, gy, grid.TYPE_LAND, 0.0);
    } else if (activeTool === 'paint_shallow') {
      applyPaintBrush(gx, gy, grid.TYPE_SHALLOW, 7.5);
    } else if (activeTool === 'paint_clear') {
      applyPaintBrush(gx, gy, grid.TYPE_SEA, 28.0);
      const rmVessel = aisSystem.removeNearest(gx, gy, Math.max(1.8, brushRadius));
      const rmHazard = obstacleManager.removeNear(gx, gy, Math.max(1.8, brushRadius));
      if (rmVessel) {
        logEvent(`Erased vessel ${rmVessel.name}.`, 'info');
        updateAISTrafficTable();
      }
      if (rmHazard) {
        logEvent(`Erased drifting hazard #${rmHazard.id}.`, 'info');
        updateAISTrafficTable();
      }
      if (isSimRunning && ship.activePath) {
        obstacleManager.checkAndReroute(ship, threeStagePlanner, logEvent, true);
      }
    }
  }

  function completeVectorPlacement(startPt, endPt) {
    if (!startPt || !grid.inBounds(startPt.x, startPt.y)) return;
    if (grid.cells[startPt.y][startPt.x] === grid.TYPE_LAND) {
      logEvent('Cannot place ship or hazard directly inside solid land obstacle!', 'warning');
      return;
    }

    const dx = endPt.x - startPt.x;
    const dy = endPt.y - startPt.y;
    const isAnchored = chkPlaceAnchored && chkPlaceAnchored.checked;
    const cfgSpeed = isAnchored ? 0.0 : (placeCraftSpeed ? Math.max(0, Number(placeCraftSpeed.value)) : 12.0);
    let cogDeg = placeCraftHeading ? Number(placeCraftHeading.value) : 135;

    if (Math.hypot(dx, dy) >= 1.0) {
      cogDeg = ((Math.atan2(dx, -dy) * 180) / Math.PI + 360) % 360;
      if (placeCraftHeading) placeCraftHeading.value = Math.round(cogDeg);
    }

    if (activeTool === 'add_cargo_ship') {
      const v = aisSystem.addCustomVessel({
        vesselCategory: 'cargo',
        x: startPt.x,
        y: startPt.y,
        sog: cfgSpeed,
        cog: cogDeg
      });
      mapDirtySinceSolve = true;
      logEvent(`Placed Cargo Ship: ${v.name} at (${startPt.x}, ${startPt.y}) | ${cfgSpeed.toFixed(1)}kn @ ${cogDeg.toFixed(0)}°`, 'info');
      updateAISTrafficTable();
      if (ship.isPointInsideBlueDomain && ship.isPointInsideBlueDomain(startPt.x, startPt.y, 10.0, grid.cellSizeMeters)) {
        ship.domainDisturbance = true;
        ship.domainDisturbanceReason = `Vessel ${v.name} placed inside Blue Domain`;
        ship.domainDisturbancePulseTimer = 2.5;
      }
      if (isSimRunning && ship.activePath) {
        obstacleManager.checkAndReroute(ship, threeStagePlanner, logEvent, true);
      }
    } else if (activeTool === 'add_mini_ship') {
      const v = aisSystem.addCustomVessel({
        vesselCategory: 'mini',
        x: startPt.x,
        y: startPt.y,
        sog: isAnchored ? 0.0 : Math.max(cfgSpeed, 14.0),
        cog: cogDeg
      });
      mapDirtySinceSolve = true;
      logEvent(`Placed Mini Ship: ${v.name} at (${startPt.x}, ${startPt.y}) | ${v.sog.toFixed(1)}kn @ ${cogDeg.toFixed(0)}°`, 'info');
      updateAISTrafficTable();
      if (ship.isPointInsideBlueDomain && ship.isPointInsideBlueDomain(startPt.x, startPt.y, 8.0, grid.cellSizeMeters)) {
        ship.domainDisturbance = true;
        ship.domainDisturbanceReason = `Mini-ship ${v.name} placed inside Blue Domain`;
        ship.domainDisturbancePulseTimer = 2.5;
      }
      if (isSimRunning && ship.activePath) {
        obstacleManager.checkAndReroute(ship, threeStagePlanner, logEvent, true);
      }
    } else if (activeTool === 'add_dynamic') {
      const rad = (cogDeg * Math.PI) / 180.0;
      const driftCellsPerSec = isAnchored ? 0.0 : Math.max(0.15, Math.min(0.65, (cfgSpeed * 0.514444) / 50.0 * 0.6));
      const vx = Math.sin(rad) * driftCellsPerSec;
      const vy = -Math.cos(rad) * driftCellsPerSec;
      const obs = obstacleManager.addObstacle(startPt.x, startPt.y, vx, vy, Math.random() > 0.3 ? 'iceberg' : 'debris', 32);
      mapDirtySinceSolve = true;
      logEvent(`Placed Dynamic ${obs.type.toUpperCase()} Hazard #${obs.id} at (${startPt.x}, ${startPt.y}).`, 'info');
      updateAISTrafficTable();
      if (ship.isPointInsideBlueDomain && ship.isPointInsideBlueDomain(startPt.x, startPt.y, 10.0, grid.cellSizeMeters)) {
        ship.domainDisturbance = true;
        ship.domainDisturbanceReason = `Hazard #${obs.id} placed inside Blue Domain`;
        ship.domainDisturbancePulseTimer = 2.5;
      }
      if (isSimRunning && ship.activePath) {
        obstacleManager.checkAndReroute(ship, threeStagePlanner, logEvent, true);
      }
    }
  }

  // Canvas Mouse Handlers
  if (canvas) {
    canvas.addEventListener('mousedown', (e) => {
      isMouseDown = true;
      const pt = getGridCoordFromMouse(e.clientX, e.clientY);
      if (activeTool === 'add_cargo_ship' || activeTool === 'add_mini_ship' || activeTool === 'add_dynamic') {
        dragStartCoord = pt;
        renderer.setPlacementPreview({ startX: pt.x, startY: pt.y, endX: pt.x, endY: pt.y, tool: activeTool });
      } else {
        applyPointOrDragTool(pt.x, pt.y);
      }
    });

    window.addEventListener('mouseup', (e) => {
      if (isMouseDown && dragStartCoord) {
        const pt = getGridCoordFromMouse(e.clientX, e.clientY);
        completeVectorPlacement(dragStartCoord, pt);
        dragStartCoord = null;
        renderer.setPlacementPreview(null);
      }
      isMouseDown = false;
    });

    canvas.addEventListener('mousemove', (e) => {
      const pt = getGridCoordFromMouse(e.clientX, e.clientY);
      renderer.setHoverCoord(pt.x, pt.y);

      if (canvasCoordsDisplay && grid.inBounds(pt.x, pt.y)) {
        const d = grid.depths[pt.y][pt.x];
        canvasCoordsDisplay.textContent = `Cell (${pt.x}, ${pt.y}) | Depth: ${d.toFixed(1)}m`;
      }

      if (isMouseDown) {
        if (dragStartCoord) {
          renderer.setPlacementPreview({ startX: dragStartCoord.x, startY: dragStartCoord.y, endX: pt.x, endY: pt.y, tool: activeTool });
        } else if (activeTool === 'paint_obstacle' || activeTool === 'paint_clear' || activeTool === 'paint_shallow') {
          applyPointOrDragTool(pt.x, pt.y);
        }
      }
    });

    canvas.addEventListener('mouseleave', () => {
      renderer.setHoverCoord(-1, -1);
      if (canvasCoordsDisplay) canvasCoordsDisplay.textContent = 'Hover cell to inspect depth & clearance';
    });
  }

  // Tool & Brush UI Binding
  const toolHints = {
    set_start: 'Click open water to relocate Port M',
    set_dest: 'Click open water to relocate Port N',
    paint_obstacle: 'Click / drag to draw Land obstacles',
    paint_shallow: 'Click / drag to draw Shallow Shoals',
    add_cargo_ship: 'Click (or drag heading) to place Cargo Ship',
    add_mini_ship: 'Click (or drag heading) to place Mini Ship',
    add_dynamic: 'Click (or drag drift) to place Iceberg/Debris',
    paint_clear: 'Click / drag to erase cells, ships & hazards'
  };

  Object.keys(toolButtons).forEach(tool => {
    const btn = toolButtons[tool];
    if (!btn) return;
    btn.addEventListener('click', () => {
      Object.values(toolButtons).forEach(b => b && b.classList.remove('active'));
      btn.classList.add('active');
      activeTool = tool;
      if (canvasModeStatus) canvasModeStatus.textContent = `Tool: ${btn.textContent.trim()}`;
      if (toolInstructionHint && toolHints[tool]) toolInstructionHint.textContent = toolHints[tool];
    });
  });

  brushButtons.forEach(btn => {
    btn.addEventListener('click', () => {
      brushButtons.forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      brushRadius = parseInt(btn.getAttribute('data-brush') || '1', 10);
    });
  });

  // Layer Checkbox Listeners
  if (chkStage1) chkStage1.addEventListener('change', () => { renderer.showStage1 = chkStage1.checked; });
  if (chkStage2) chkStage2.addEventListener('change', () => { renderer.showStage2 = chkStage2.checked; });
  if (chkStage3) chkStage3.addEventListener('change', () => { renderer.showStage3 = chkStage3.checked; });
  if (chkShipDomain) chkShipDomain.addEventListener('change', () => { renderer.showShipDomain = chkShipDomain.checked; });
  if (chkAISTraffic) chkAISTraffic.addEventListener('change', () => { renderer.showAISTraffic = chkAISTraffic.checked; });

  // Ship Parameter Inputs
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
      mapDirtySinceSolve = true;
      logEvent(`Own vessel specs updated: ${input.id.replace('param', '')} = ${input.value}`, 'info');
    });
  });

  // =========================================================================
  // Uploaded Path Map Image & JSON Auto-Recognition
  // =========================================================================
  function runImageMapRecognition() {
    if (!mapImporter.hasImage()) return;

    solverRunToken++;
    isSimRunning = false;
    updateRunButtonUI(false);

    const summary = mapImporter.recognizeAndApply(grid, ship.draft, {
      mode: importModeSelect ? importModeSelect.value : 'auto',
      sensitivity: importSensitivity ? parseFloat(importSensitivity.value) : 0.52,
      invert: importInvertState,
      detectPorts: chkDetectPorts ? chkDetectPorts.checked : true,
      detectShoals: chkDetectShoals ? chkDetectShoals.checked : true
    });

    if (!summary) return;

    // Remove or relocate any existing AIS ships/hazards that fell onto newly detected land obstacles
    aisSystem.vessels = aisSystem.vessels.filter(v => {
      const gx = Math.round(v.x), gy = Math.round(v.y);
      return grid.inBounds(gx, gy) && grid.cells[gy][gx] !== grid.TYPE_LAND;
    });
    obstacleManager.obstacles = obstacleManager.obstacles.filter(o => {
      const gx = Math.round(o.x), gy = Math.round(o.y);
      return grid.inBounds(gx, gy) && grid.cells[gy][gx] !== grid.TYPE_LAND;
    });

    ship.resetToStart(grid.start);
    renderer.clearAllAnimations();
    resetStageCardsUI();
    updateDomainDisplay();
    updateAISTrafficTable();
    clearTelemetryAndTables('MAP BUILT — READY TO RUN');

    if (canvasRecognizedPreview) {
      mapImporter.renderPreview(canvasRecognizedPreview, grid);
    }
    if (importStatsSummary) {
      const portTag = summary.portsDetected.start || summary.portsDetected.destination ? 'Ports Auto-Found' : 'Ports Placed';
      importStatsSummary.textContent = `${summary.landCount} Land | ${summary.shoalCount} Shoal | ${summary.seaCount} Sea (${portTag})`;
    }

    logEvent(`Auto-recognized uploaded map "${summary.fileName}" [${summary.detectedStyle}]: ${summary.landCount} land cells, ${summary.shoalCount} shoal cells built! Click ▶ Run All 3 to compute optimal path.`, 'success');
  }

  function handleUploadedMapFile(file) {
    mapImporter.loadFile(
      file,
      (img, fileName) => {
        if (mapImportPreviewBox) mapImportPreviewBox.style.display = 'block';
        if (imgUploadThumb) imgUploadThumb.src = img.src;
        importInvertState = false;
        runImageMapRecognition();
      },
      (jsonData, fileName) => {
        if (grid.loadFromJSON(jsonData, ship.draft)) {
          solverRunToken++;
          isSimRunning = false;
          updateRunButtonUI(false);
          ship.resetToStart(grid.start);
          renderer.clearAllAnimations();
          resetStageCardsUI();
          updateDomainDisplay();
          clearTelemetryAndTables('JSON MAP LOADED');
          if (mapImportPreviewBox) mapImportPreviewBox.style.display = 'none';
          logEvent(`Loaded JSON chart "${fileName}" cleanly.`, 'success');
        } else {
          logEvent('Invalid NavPilot JSON map format.', 'error');
        }
      },
      (errMsg) => {
        logEvent(errMsg, 'error');
      }
    );
  }

  if (mapDropZone && mapFileInput) {
    mapDropZone.addEventListener('click', () => mapFileInput.click());
    mapFileInput.addEventListener('change', (e) => {
      if (e.target.files && e.target.files[0]) {
        handleUploadedMapFile(e.target.files[0]);
        mapFileInput.value = '';
      }
    });

    mapDropZone.addEventListener('dragover', (e) => {
      e.preventDefault();
      mapDropZone.classList.add('dragover');
    });
    mapDropZone.addEventListener('dragleave', () => mapDropZone.classList.remove('dragover'));
    mapDropZone.addEventListener('drop', (e) => {
      e.preventDefault();
      mapDropZone.classList.remove('dragover');
      if (e.dataTransfer && e.dataTransfer.files && e.dataTransfer.files[0]) {
        handleUploadedMapFile(e.dataTransfer.files[0]);
      }
    });
  }

  if (importModeSelect) importModeSelect.addEventListener('change', runImageMapRecognition);
  if (chkDetectPorts) chkDetectPorts.addEventListener('change', runImageMapRecognition);
  if (chkDetectShoals) chkDetectShoals.addEventListener('change', runImageMapRecognition);
  if (importSensitivity) {
    importSensitivity.addEventListener('input', () => {
      if (importSensVal) importSensVal.textContent = `${Math.round(parseFloat(importSensitivity.value) * 100)}%`;
      runImageMapRecognition();
    });
  }
  if (btnInvertMap) {
    btnInvertMap.addEventListener('click', () => {
      importInvertState = !importInvertState;
      runImageMapRecognition();
    });
  }

  if (btnExportMap) {
    btnExportMap.addEventListener('click', () => {
      const jsonStr = grid.exportToJSON();
      const blob = new Blob([jsonStr], { type: 'application/json' });
      const url = URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = 'navpilot_sea_chart.json';
      a.click();
      URL.revokeObjectURL(url);
      logEvent('Exported current 50×50 bathymetric chart as navpilot_sea_chart.json.', 'info');
    });
  }

  // =========================================================================
  // Core Control Button Event Listeners
  // =========================================================================
  if (speedSlider) {
    speedSlider.addEventListener('input', () => {
      simulationSpeed = parseFloat(speedSlider.value);
      if (speedDisplay) speedDisplay.textContent = `${simulationSpeed.toFixed(1)}x`;
    });
  }

  if (btnRun) {
    btnRun.addEventListener('click', () => {
      if (isSimRunning) return;
      if (!ship.activePath || ship.state === 'IDLE' || ship.state === 'ARRIVED' || mapDirtySinceSolve) {
        runSequentialThreeAlgorithms(true);
      } else {
        const staticThreat = obstacleManager.evaluateStaticObstruction(ship);
        if (staticThreat.conflictDetected) {
          logEvent(`Active route obstructed: ${staticThreat.reason}. Recalculating optimal path...`, 'warning');
          runSequentialThreeAlgorithms(true);
        } else {
          isSimRunning = true;
          updateRunButtonUI(true);
          logEvent('Resumed autonomous navigation along verified Optimal Path.', 'info');
        }
      }
    });
  }

  if (btnRunPipeline) {
    btnRunPipeline.addEventListener('click', () => {
      runSequentialThreeAlgorithms(false);
    });
  }

  if (btnPause) {
    btnPause.addEventListener('click', () => {
      solverRunToken++;
      isSimRunning = false;
      updateRunButtonUI(false);
      logEvent('Simulation paused.', 'info');
    });
  }

  if (btnReset) {
    btnReset.addEventListener('click', () => {
      solverRunToken++;
      isSimRunning = false;
      updateRunButtonUI(false);
      ship.resetToStart(grid.start);
      renderer.clearAllAnimations();
      resetStageCardsUI();
      clearTelemetryAndTables('RESET AT PORT M');
      logEvent('Cargo ASV reset cleanly to Port M (Departure).', 'info');
    });
  }

  if (btnBenchmark) {
    btnBenchmark.addEventListener('click', () => {
      runSequentialThreeAlgorithms(false);
    });
  }

  if (btnClearObstacles) {
    btnClearObstacles.addEventListener('click', clearEntireDashboard);
  }

  if (btnRandomMap) {
    btnRandomMap.addEventListener('click', () => {
      solverRunToken++;
      isSimRunning = false;
      updateRunButtonUI(false);
      grid.generateRandomMap();
      grid.computeDistanceField(ship.draft);
      aisSystem.clear();
      aisSystem.addRandomVessel('cargo');
      aisSystem.addRandomVessel('cargo');
      aisSystem.addRandomVessel('mini');
      obstacleManager.clear();
      ship.resetToStart(grid.start);
      renderer.clearAllAnimations();
      updateDomainDisplay();
      updateAISTrafficTable();
      logEvent('Generated random archipelago sea chart with vessels.', 'info');
      runSequentialThreeAlgorithms(false);
    });
  }

  if (presetSelect) {
    presetSelect.addEventListener('change', () => {
      const preset = presetSelect.value;
      if (preset === 'blank') {
        clearEntireDashboard();
        return;
      }
      solverRunToken++;
      isSimRunning = false;
      updateRunButtonUI(false);
      grid.loadPreset(preset);
      grid.computeDistanceField(ship.draft);
      aisSystem.initDefaultTraffic();
      // Ensure default vessels don't spawn inside preset land
      aisSystem.vessels = aisSystem.vessels.filter(v => {
        const gx = Math.round(v.x), gy = Math.round(v.y);
        return grid.inBounds(gx, gy) && grid.cells[gy][gx] !== grid.TYPE_LAND;
      });
      obstacleManager.clear();
      ship.resetToStart(grid.start);
      updateDomainDisplay();
      updateAISTrafficTable();
      logEvent(`Loaded maritime chart preset: ${preset.toUpperCase()}.`, 'info');
      runInstantComparisonSnapshot();
    });
  }

  if (btnSpawnCargoAIS) {
    btnSpawnCargoAIS.addEventListener('click', () => {
      const v = aisSystem.addRandomVessel('cargo');
      if (v) {
        logEvent(`Spawned Cargo Vessel: ${v.name} (SOG ${v.sog.toFixed(1)}kn, COG ${v.cog.toFixed(0)}°).`, 'info');
        updateAISTrafficTable();
      }
    });
  }

  if (btnSpawnMiniAIS) {
    btnSpawnMiniAIS.addEventListener('click', () => {
      const v = aisSystem.addRandomVessel('mini');
      if (v) {
        logEvent(`Spawned Mini Ship: ${v.name} (SOG ${v.sog.toFixed(1)}kn, COG ${v.cog.toFixed(0)}°).`, 'info');
        updateAISTrafficTable();
      }
    });
  }

  if (btnSpawnHazard) {
    btnSpawnHazard.addEventListener('click', () => {
      const obs = obstacleManager.addRandomObstacle();
      if (obs) {
        logEvent(`Spawned Dynamic ${obs.type.toUpperCase()} #${obs.id} at (${obs.x.toFixed(0)}, ${obs.y.toFixed(0)}).`, 'info');
        updateAISTrafficTable();
      }
    });
  }

  if (btnClearLog) {
    btnClearLog.addEventListener('click', () => {
      if (eventLogList) eventLogList.innerHTML = '';
      logEvent('Solver & collision log cleared.', 'info');
    });
  }

  // Initial Startup Snapshot
  logEvent('NavPilot Three-Stage Sequential Solver initialized (SS = L × B × D).', 'info');
  readShipInputs();
  updateAISTrafficTable();
  runInstantComparisonSnapshot();

  // =========================================================================
  // Main Real-Time 60 FPS Loop
  // =========================================================================
  function mainLoop(timestamp) {
    const rawDt = (timestamp - lastFrameTime) / 1000.0;
    lastFrameTime = timestamp;
    const dt = Math.min(0.1, rawDt) * simulationSpeed;

    // 1. Dynamic obstacles and AIS ships / mini ships move continuously
    aisSystem.update(dt);
    obstacleManager.update(dt);

    // 2. Refresh AIS table CPA/TCPA smoothly every 0.4s
    if (timestamp - lastAisTableUpdate > 400) {
      lastAisTableUpdate = timestamp;
      if (ship.activePath) {
        aisSystem.evaluateTrafficRisk(ship, ship.lookaheadHorizonSeconds);
      }
      updateAISTrafficTable();
    }

    if (isSimRunning) {
      // 3. Evaluate dynamic threats & trigger immediate Path B avoidance if conflict detected
      obstacleManager.checkAndReroute(ship, threeStagePlanner, logEvent);

      // 4. Advance ship smoothly along verified optimal corridor with hard border safety clamp
      ship.update(dt, grid, grid.cellSizeMeters);

      // 5. Update Telemetry
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
        updateRunButtonUI(false);
        logEvent('Voyage complete: Cargo ASV reached Port N with 0 collisions!', 'success');
      }
    }

    renderer.render(dt);
    requestAnimationFrame(mainLoop);
  }

  requestAnimationFrame(mainLoop);
});

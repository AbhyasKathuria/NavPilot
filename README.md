# NavPilot: Three-Stage Hybrid Cargo-ASV Path-Planning Simulator

[![GitHub Repository](https://img.shields.io/badge/GitHub-AbhyasKathuria%2FNavPilot-181717?style=flat&logo=github)](https://github.com/AbhyasKathuria/NavPilot)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Dependencies](https://img.shields.io/badge/Dependencies-Zero%20(Vanilla%20JS)-success.svg)](#)
[![Architecture](https://img.shields.io/badge/Architecture-Three--Stage%20Hybrid%20(Fig%203.2)-blueviolet.svg)](#)
[![AIS Traffic](https://img.shields.io/badge/AIS-Real--Time%20Marine%20Feed-success.svg)](#)

A complete, runnable, real-time cargo ship and Autonomous Surface Vessel (ASV) path-planning simulator built as a single-page web app with HTML5, CSS3, and vanilla JavaScript (zero external dependencies). 

NavPilot implements the **Three-Stage Hybrid Planning Architecture** (Figure 3.2) combining **A\* Global Search**, **RRT\* Kinematic Refinement**, and **Somersault Monkey Search Multi-Objective Optimization**, integrated with real-time **AIS marine traffic data**, dynamic continuous vessel motion, and the physical **Ship Domain formula: $SS = L \times B \times D$** from Departure Port $M$ to Destination Port $N$.

---

## 🎨 Strict Color Palette

The entire user interface, navigation charts, tables, telemetry badges, and safety buffers strictly use the requested color theme:
- **`#E0F4FF`** — Soft Azure Sea Base (Deep Navigable Sea, Main Application Canvas Base, Primary Surface)
- **`#87C4FF`** — Vibrant Sky Blue (Ship Domain $SS$ Safety Envelopes, Card Borders, Secondary Accents)
- **`#39A7FF`** — Vivid Ocean Blue (Primary Buttons, Active Highlights, Optimized Path Route, Key Boundaries)
- **`#FFEED9`** — Warm Cream Sand (Control Panels, Headers, Shallow Reef Shoals, Telemetry Badges)

High-contrast maritime midnight navy (`#0A2540`) is utilized for typography and hull contours to ensure WCAG AAA accessibility and crisp readability on maritime bridge displays.

---

## 🏗️ Three-Stage Hybrid Planning Architecture (Figure 3.2)

```
       ┌─────────────────────────────────────────────────────────────┐
       │                       INPUT DATA                            │
       │  • Chart & Bathymetric Data                                │
       │  • Real-Time AIS Marine Traffic Broadcasts                  │
       │  • Ship Domain (SS = L × B × D)                             │
       │  • Start / End Ports M, N                                   │
       └──────────────────────────────┬──────────────────────────────┘
                                      │
                                      ▼
       ┌─────────────────────────────────────────────────────────────┐
       │         STAGE 1: A* Global Path Search (Grid-Based)         │
       │  • Discretized 50×50 bathymetric marine environment         │
       │  • Octile heuristic distance function                       │
       │  • Avoids shallow shoals (depth < D) and static hazards     │
       └──────────────────────────────┬──────────────────────────────┘
                                      │ (coarse waypoint route)
                                      ▼
       ┌─────────────────────────────────────────────────────────────┐
       │     STAGE 2: RRT* Kinematic Refinement (Curvature-Constrained)│
       │  • Informed corridor sampling seeded from Stage 1           │
       │  • Non-holonomic minimum turning radius (R ≥ R_min)         │
       │  • Cost-to-come optimization and neighbor rewiring          │
       └──────────────────────────────┬──────────────────────────────┘
                                      │ (kinematically feasible path)
                                      ▼
       ┌─────────────────────────────────────────────────────────────┐
       │  STAGE 3: Somersault Monkey Search Optimisation (Multi-Obj) │
       │  • Climb Process: fine local perturbation for energy savings│
       │  • Watch-Jump Process: long-range leaps away from hazards   │
       │  • Somersault Process: movement toward population barycenter│
       │  • Multi-Objective Fitness (Energy + Time + Collision Free) │
       └──────────────────────────────┬──────────────────────────────┘
                                      │
                                      ▼
       ┌─────────────────────────────────────────────────────────────┐
       │ Optimised, Collision-Free Energy- and Time-Efficient ASV Path│
       └─────────────────────────────────────────────────────────────┘
```

### 1. Stage 1: A* Global Path Search (Grid-Based)
- Finds the initial rough path through the discretized bathymetric grid from **Port $M$** to **Port $N$**.
- Employs an octile heuristic function to achieve global exploration without expanding the whole map.
- Strictly validates draft: cells with depth $< D$ are impassable.
- Outputs the **coarse waypoint route**.

### 2. Stage 2: RRT* Kinematic Refinement (Curvature-Constrained)
- Refines the coarse waypoint route into a **kinematically feasible path**.
- Constrains all steering actions by the ship's physical minimum turning radius:
  $$\kappa \le \kappa_{\text{max}} = \frac{1}{R_{\text{min}}}$$
- Employs informed corridor sampling around Stage 1's coarse waypoints.
- Continuously rewires near neighbors to optimize path smoothness and eliminate zig-zag grid artifacts.

### 3. Stage 3: Somersault Monkey Search Optimization (Multi-Objective)
- Seeded from the kinematically feasible path generated by Stage 2.
- Bio-inspired metaheuristic maintaining a population of candidate paths ($M=16$ monkeys):
  - **Climb Process**: Fine-grained local perturbations of intermediate waypoints to minimize rudder movement and hydrodynamic drag.
  - **Watch-Jump Process**: Large exploratory leaps away from coastlines and oncoming AIS traffic.
  - **Somersault Process**: Monkeys somersault toward the population's center of gravity (barycenter of elite routes).
- **Multi-Objective Cost Function**:
  $$F = w_{\text{time}} \cdot T_{\text{transit}} + w_{\text{energy}} \cdot E_{\text{propulsion}} + w_{\text{safety}} \cdot P_{\text{collision}} + w_{\text{curv}} \cdot P_{\text{curvature}}$$
  - **Time Efficiency ($T_{\text{transit}}$)**: Minimizes overall voyage distance and transit duration at cruise speed.
  - **Energy Efficiency ($E_{\text{propulsion}}$)**: Minimizes quadratic rudder deflection energy ($\sum |\Delta \delta_r|^2$) and resistance caused by sharp turning rates ($\dot{\psi}^2$).
  - **Collision Safety ($P_{\text{collision}}$)**: Maximizes clearance from coastal reefs and dynamic AIS ship domains.
  - **Curvature Feasibility ($P_{\text{curvature}}$)**: Zero curvature violations guaranteed.

---

## 📡 Live AIS Marine Traffic & Dynamic Rerouting

- **Continuous Dynamic Motion**: Other marine vehicles (container vessels, tankers, bulk carriers, patrol craft) navigate the sea map **continuously before, during, and after path planning**.
- **Real-Time AIS Data Broadcasts**:
  - `MMSI` (Maritime Mobile Service Identity, e.g. 563042000)
  - `Vessel Name` (e.g. *MV PACIFIC PIONEER*, *STENA POLARIS*, *COSCO SHIPPING ASIA*)
  - `Vessel Type` (Container, Tanker, Bulk Carrier, Patrol Craft)
  - `SOG` (Speed Over Ground in knots)
  - `COG` (Course Over Ground in degrees)
  - `Length` ($L_{\text{obs}}$) and `Beam` ($B_{\text{obs}}$)
- **CPA & TCPA Collision Detection**:
  - Continuously projects trajectories over a forward lookahead horizon ($t \in [0, 30\text{s}]$).
  - Evaluates Closest Point of Approach ($\text{CPA}$) and Time to CPA ($\text{TCPA}$).
- **Automatic Dynamic Rerouting**:
  - When an AIS vessel or drifting obstacle breaches the Ship Domain or is projected on an intercept course, NavPilot triggers an instant dynamic reroute.
  - The 3-Stage Pipeline re-computes an avoidance route from the ship's live $(x, y, \theta)$ to Port $N$.
  - The previous path becomes dashed (**Path A**), and the new avoidance route (**Path B**) is activated.
  - If a narrow channel is temporarily obstructed, the vessel performs a **"HOLD & WAIT"** reverse-thrust maneuver until the channel is clear.

---

## 📐 Ship Domain Formula: $SS = L \times B \times D$

In modern Autonomous Surface Vessel navigation, the safety domain is determined by the vessel's primary physical dimensions:
$$\text{Ship Domain } SS = L \times B \times D$$
- **Length ($L$)**: Longitudinal dimension ($200\text{ m}$ default).
- **Beam ($B$)**: Transverse hull breadth ($32\text{ m}$ default).
- **Draft ($D$)**: Vertical keel depth ($12\text{ m}$ default).
- **Domain Volumetric Metric**: $SS = 200 \times 32 \times 12 = 76,800\text{ m}^3$.
- **2D Planar Safety Domain Envelope**:
  $$\text{Domain Radius} = \frac{B}{2} + 15\text{m} + \left(\frac{\text{Displacement}}{50,000}\right) \times 12\text{m} + \left(\frac{v}{v_{\text{max}}}\right) \times 8\text{m}$$
  Displayed as a safety capsule around the hull with real-time domain volume annotation.

---

## 🚀 Quickstart

### Option 1: Web Application (Single-Page Browser Simulator)
- **1-Click Launch**: Double-click **`start.bat`** to open NavPilot instantly in your default web browser.
- **Manual Launch**: Double-click **`index.html`** or extract and open from **`NavPilot.zip`**.
- *Zero installation required* — runs offline with pure HTML5, CSS3, and vanilla JavaScript!

### Option 2: Python Architecture & Benchmark Suite
- **1-Click Launch**: Double-click **`run_python.bat`** on Windows.
- **CLI Execution**:
  ```bash
  # Install dependencies (numpy & matplotlib)
  pip install -r python/requirements.txt

  # Run full Three-Stage Hybrid Pipeline & Benchmark
  python run_simulation.py --benchmark

  # Interactive visualization with custom preset
  python run_simulation.py --preset singapore --iterations 25 --show

  # Auto-recognize an uploaded AI/chart image map or JSON chart directly
  python run_simulation.py --map sample_sea_map.png --benchmark
  ```
- **CLI Options**:
  - `--preset`: Choose maritime chart (`archipelago`, `singapore`, `ice_alley`, `canal`).
  - `--map`: Path to an image chart (`.png`/`.jpg`) or exported `.json` chart to auto-recognize obstacles, shoals & ports.
  - `--sensitivity`: Image obstacle recognition sensitivity threshold (default: `0.52`).
  - `--benchmark`: Runs performance benchmark across all algorithms and selects optimal path.
  - `--iterations`: Number of iterations for Somersault Monkey Search (default: `20`).
  - `--show`: Displays interactive matplotlib visualization window.
  - `--save`: Destination file for the high-res plot (default: `navpilot_simulation.png`).

---

## 🛠️ New Interactive Capabilities

1. **🗑 Full Blank Chart Clear (`Clear All`)**:
   - Wipes land, shallow shoals, AIS cargo/mini ships, dynamic drifting hazards, and previous planned trajectories cleanly back to open blue water so you can compose every entity from scratch.
2. **🚢 Full Entity Builder (`Port M/N`, `Land`, `Shoal`, `Cargo Ship`, `Mini Ship`, `Drift Hazard`, `Erase`)**:
   - Supports selectable paint brushes (`1×1`, `3×3`, `5×5`) and **Click-and-Drag Direction Vectors** on the sea grid to aim headings (`COG °`) and speeds (`SOG kn`, or `0 kn` static anchorage) for placed ships, mini ships, and drifting icebergs/debris.
3. **⚡ Sequential 3-Algorithm Solver & Optimal Zero-Collision Audit**:
   - Executes **Algorithm 1: A\*** $\to$ **Algorithm 2: RRT\*** $\to$ **Algorithm 3: Somersault Monkey Search** step-by-step visually on canvas, scores all candidates across transit time, rudder drag energy, curvature feasibility, and continuous hull-border obstacle clearance (`getExactHazardDistanceMeters`), highlights the **🏆 OPTIMAL** winner, and navigates cleanly around all static and moving entities.
4. **🗺️ Path Map Image Upload & Offline Auto-Recognition**:
   - Drag & drop or upload any nautical chart image, black-on-white maze sketch, or AI-generated map photo (including the included `sample_sea_map.png`). Auto-recognizes land obstacles, shallow sandbar reefs, deep water channels, and green/red Port M & Port N markers cleanly offline using Canvas 2D with live sensitivity & detection-mode tuning.

---

## 🛠 File Structure

```
NavPilot/
├── start.bat                   # 1-click Windows launcher for Web Simulator
├── run_python.bat              # 1-click Windows launcher for Python Simulation
├── run_simulation.py           # Python simulation & benchmark entry point
├── index.html                  # Single-page interface with 3-stage UI, uploader & telemetry
├── style.css                   # Strict 4-color palette CSS stylesheet (#E0F4FF/#87C4FF/#39A7FF/#FFEED9)
├── README.md                   # Technical documentation and operational guide
├── sample_sea_map.png          # Ready-to-test sample top-down nautical sea map image
├── NavPilot.zip                # Offline distribution package
├── navpilot_simulation.png     # High-resolution simulation plot output
├── js/                         # JavaScript Implementation (Browser Simulator)
│   ├── grid.js                 # 50×50 bathymetric grid, exact continuous clearance, Port M & N
│   ├── ship.js                 # Ship Domain SS = L * B * D, ASV hydrodynamics, stopping distance
│   ├── ais.js                  # Live AIS Cargo & Mini Ship traffic, lookahead bounce, CPA/TCPA
│   ├── astar.js                # Stage 1: A* Global Path Search (static + spatiotemporal avoidance)
│   ├── rrtstar.js              # Stage 2: RRT* Kinematic Refinement (curvature-constrained)
│   ├── monkeysearch.js         # Stage 3: Somersault Monkey Search (multi-objective SMSO)
│   ├── curvature.js            # Dubins arc fillet rounding utilities
│   ├── pipeline.js             # Sequential 3-stage orchestrator + zero-collision winner audit
│   ├── dynamic.js              # Dynamic hazard manager and real-time Path B avoidance rerouting
│   ├── mapImporter.js          # Offline image & JSON map auto-recognizer with live calibration
│   ├── renderer.js             # 500×500 high-DPI canvas renderer with live algorithm animations
│   └── main.js                 # Application bus, entity builder, telemetry, and benchmark runner
└── python/                     # Python Implementation (Same 3-Stage Architecture)
    ├── requirements.txt        # Python dependencies (numpy, matplotlib, pillow)
    ├── grid.py                 # Bathymetric grid with EDT, exact clearance, presets & image loader
    ├── ship.py                 # CargoASV model and Ship Domain SS = L * B * D
    ├── ais.py                  # Live AIS traffic broadcasting & CPA/TCPA checks
    ├── astar.py                # Stage 1: A* Global Path Search
    ├── rrtstar.py              # Stage 2: Curvature-constrained RRT* Refinement
    ├── monkeysearch.py         # Stage 3: Somersault Monkey Search Optimization
    ├── curvature.py            # Dubins fillet arc smoothing
    ├── pipeline.py             # ThreeStageHybridPlanner orchestrator & winner selection
    ├── visualizer.py           # Matplotlib visualization engine
    └── main.py                 # Python benchmark & CLI simulation runner (--map supported)
```

---

## 🎮 Interactive Controls

- **▶ Run All 3**: Runs Algorithm 1 (A*), Algorithm 2 (RRT*), and Algorithm 3 (Somersault Monkey Search) sequentially one by one, picks the verified **🏆 OPTIMAL** zero-collision route, and sails from Port M to Port N.
- **🗑 Clear All**: Clears the entire map, all cargo/mini ships, all dynamic hazards, and all paths back to a blank open sea.
- **Layer Checkboxes**:
  - `[x] Algo 1 (A*)`: Toggles dashed amber coarse global route.
  - `[x] Algo 2 (RRT*)`: Toggles cyan curvature-constrained kinematic route.
  - `[x] Best Optimal`: Toggles glowing vivid ocean blue (`#39A7FF`) selected optimal path.
  - `[x] Ship Domain (SS)`: Toggles $SS = L \times B \times D$ dynamic safety envelope.
  - `[x] AIS Ships, Mini-Ships & Hazards`: Toggles live marine traffic & drifting obstacles.
- **Upload Map**: Drag/drop a PNG/JPG/JSON chart onto the upload panel to auto-build obstacles, reefs, and ports automatically.

---

## 📜 License & Credits

Developed by **Abhyas Kathuria**. Open-sourced under the MIT License.  
Repository: [https://github.com/AbhyasKathuria/NavPilot](https://github.com/AbhyasKathuria/NavPilot)

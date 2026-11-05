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
- **`#FFCED8`** — Dusty Rose Accent (Headers, Primary Buttons, Action Highlights, Obstacle Accents)
- **`#FFDDDD`** — Blush / Warning / Safety Buffer (Ship Domain $SS$ Envelopes, Dynamic Collision Alerts)
- **`#FFF4DE`** — Ivory Sea Base (Deep Navigable Sea, Main Application Base, Document Surface)
- **`#FFE8C4`** — Warm Peach Fairway (Sidebar Panels, Shallow Reef Depths, Table Headers, Control Accents)

High-contrast espresso dark mahogany (`#2C1810`) is utilized for typography and hull contours to ensure WCAG AAA accessibility and crisp readability on maritime bridge displays.

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
  ```
- **CLI Options**:
  - `--preset`: Choose maritime chart (`archipelago`, `singapore`, `ice_alley`, `canal`).
  - `--benchmark`: Runs performance benchmark across all 4 algorithms.
  - `--iterations`: Number of iterations for Somersault Monkey Search (default: 20).
  - `--show`: Displays interactive matplotlib visualization window.
  - `--save`: Destination file for the high-res plot (default: `navpilot_simulation.png`).

---

## 🛠 File Structure

```
NavPilot/
├── start.bat                   # 1-click Windows launcher for Web Simulator
├── run_python.bat              # 1-click Windows launcher for Python Simulation
├── run_simulation.py           # Python simulation & benchmark entry point
├── index.html                  # Single-page interface with 3-stage UI, AIS table & telemetry
├── style.css                   # Strict 4-color palette CSS stylesheet & bridge theme
├── README.md                   # Technical documentation and operational guide
├── NavPilot.zip                # Offline distribution package
├── navpilot_simulation.png     # High-resolution simulation plot output
├── js/                         # JavaScript Implementation (Browser Simulator)
│   ├── grid.js                 # 50×50 bathymetric grid, EDT, Port M & N
│   ├── ship.js                 # Ship Domain SS = L * B * D, ASV hydrodynamics, stopping distance
│   ├── ais.js                  # Live AIS data broadcasting, vessel simulation, CPA/TCPA
│   ├── astar.js                # Stage 1: A* Global Path Search (coarse waypoint route)
│   ├── rrtstar.js              # Stage 2: RRT* Kinematic Refinement (curvature-constrained)
│   ├── monkeysearch.js         # Stage 3: Somersault Monkey Search (multi-objective SMSO)
│   ├── curvature.js            # Dubins arc fillet rounding utilities
│   ├── pipeline.js             # Three-Stage Hybrid Planning Pipeline orchestrator (Fig 3.2)
│   ├── dynamic.js              # Dynamic hazard manager and reactive collision rerouting
│   ├── renderer.js             # 500×500 high-DPI canvas renderer with multi-stage layer toggles
│   └── main.js                 # Application bus, telemetry updater, and benchmark runner
└── python/                     # Python Implementation (Same 3-Stage Architecture)
    ├── requirements.txt        # Python dependencies (numpy, matplotlib)
    ├── grid.py                 # Bathymetric grid with EDT and presets
    ├── ship.py                 # CargoASV model and Ship Domain SS = L * B * D
    ├── ais.py                  # Live AIS traffic broadcasting & CPA/TCPA checks
    ├── astar.py                # Stage 1: A* Global Path Search
    ├── rrtstar.py              # Stage 2: Curvature-constrained RRT* Refinement
    ├── monkeysearch.py         # Stage 3: Somersault Monkey Search Optimization
    ├── curvature.py            # Dubins fillet arc smoothing
    ├── pipeline.py             # ThreeStageHybridPlanner orchestrator
    ├── visualizer.py           # Matplotlib visualization engine
    └── main.py                 # Python benchmark & CLI simulation runner
```

---

## 🎮 Interactive Controls

- **Run 3-Stage Pipeline (⚡)**: Computes Stage 1 A* $\to$ Stage 2 RRT* $\to$ Stage 3 Somersault Monkey Search from Port M to Port N.
- **Layer Checkboxes**:
  - `[x] Stage 1 (A*)`: Toggles dashed amber coarse waypoint route.
  - `[x] Stage 2 (RRT*)`: Toggles cyan curvature-constrained kinematic route & tree.
  - `[x] Stage 3 (Monkey)`: Toggles vivid coral final multi-objective ASV path.
  - `[x] Ship Domain (SS)`: Toggles $SS = L \times B \times D$ dynamic safety envelope.
  - `[x] AIS Marine Traffic`: Toggles live other-vessel AIS broadcast labels.
- **Ports**: Click **🚩 Set Port M** (Departure) or **🎯 Set Port N** (Destination) to reposition terminals.
- **Spawn AIS Vessel (🚢)**: Injects active commercial traffic with unique MMSI, SOG, and COG into the marine environment.
- **Benchmark (📊)**: Compares Stage 1 A*, Stage 2 RRT*, Stage 3 SMSO, and the Full 3-Stage Hybrid Pipeline side-by-side.

---

## 📜 License & Credits

Developed by **Abhyas Kathuria**. Open-sourced under the MIT License.  
Repository: [https://github.com/AbhyasKathuria/NavPilot](https://github.com/AbhyasKathuria/NavPilot)

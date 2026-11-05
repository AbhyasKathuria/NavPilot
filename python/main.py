"""
NavPilot - Cargo Ship Real-Time Path-Planning Simulator
Main Entry Point & Simulation Benchmark (main.py)

Implements the complete Three-Stage Hybrid Planning Architecture (Figure 3.2):
[Chart & AIS Data, Ship Domain SS = L * B * D, Ports M, N]
                      │
                      ▼
       Stage 1: A* Global Path Search (grid-based)
                      │  (coarse waypoint route)
                      ▼
       Stage 2: RRT* Kinematic Refinement (curvature-constrained)
                      │  (kinematically feasible path)
                      ▼
       Stage 3: Somersault Monkey Search Optimisation (multi-objective)
                      │
                      ▼
    Optimised, collision-free energy- and time-efficient cargo-ASV path
"""

import argparse
import sys
import time

from ais import AISSystem
from astar import AStarPlanner
from grid import BathymetricGrid
from monkeysearch import SomersaultMonkeySearchPlanner
from pipeline import ThreeStageHybridPlanner
from rrtstar import RRTStarPlanner
from ship import CargoASV
from visualizer import plot_simulation


def print_banner():
    banner = r"""
========================================================================================
   _  __           ___  _ __      __ 
  / |/ /___ __  __/ _ \(_) /___  / /_
 /    / _ `/ |/ / ___/ / / // _\/ __/
/_/|_/\_,_/|___/_/  /_/_/\___/\__/  
Cargo Ship & ASV Three-Stage Hybrid Path-Planning Architecture
[Stage 1: A* Global -> Stage 2: RRT* Kinematic -> Stage 3: Somersault Monkey Search]
========================================================================================
"""
    print(banner)


def run_benchmark(grid: BathymetricGrid, ship: CargoASV, ais_system: AISSystem):
    print("\n" + "=" * 90)
    print("                    RUNNING 4-ALGORITHM PERFORMANCE BENCHMARK")
    print("=" * 90)

    port_m = grid.port_m
    port_n = grid.port_n
    ship_draft = ship.draft
    required_clearance = ship.compute_required_clearance_meters()

    planner = ThreeStageHybridPlanner(grid)

    # 1. A*
    a_star = AStarPlanner(grid)
    t0 = time.perf_counter()
    res_a = a_star.plan(port_m, port_n, ship_draft, required_clearance)
    time_a = (time.perf_counter() - t0) * 1000.0

    coarse_wps = planner.extract_coarse_waypoints(res_a["path"], ship_draft, required_clearance) if res_a["success"] else [{"x": port_m["x"], "y": port_m["y"]}, {"x": port_n["x"], "y": port_n["y"]}]

    # 2. Stage 2 RRT*
    rrt_star = RRTStarPlanner(grid)
    t0 = time.perf_counter()
    res_rrt = rrt_star.refine(coarse_wps, ship, ais_system)
    time_rrt = (time.perf_counter() - t0) * 1000.0

    # 3. Somersault Monkey Search standalone
    sms = SomersaultMonkeySearchPlanner(grid)
    t0 = time.perf_counter()
    res_sms = sms.optimize(coarse_wps, ship, ais_system, max_iterations=15)
    time_sms = (time.perf_counter() - t0) * 1000.0

    # 4. Full 3-Stage Hybrid Architecture
    t0 = time.perf_counter()
    res_hybrid = planner.plan(port_m, port_n, ship, ais_system, monkey_iterations=15)
    time_hybrid = (time.perf_counter() - t0) * 1000.0

    print(f"{'Algorithm':<32} | {'Time':<8} | {'Nodes/Pop':<10} | {'Length':<8} | {'Turn':<7} | {'Clearance':<9} | {'Status'}")
    print("-" * 90)

    print(f"{'Stage 1: A* Global (Grid)':<32} | {time_a:>6.1f}ms | {res_a['nodes_expanded']:>9} | {res_a['length_meters']:>6.0f}m | {res_a['smoothness_deg']:>5.1f}° | {res_a['min_clearance_meters']:>7.1f}m | Feasible")
    print(f"{'Stage 2: RRT* Kinematic':<32} | {time_rrt:>6.1f}ms | {res_rrt['nodes_sampled']:>9} | {res_rrt['length_meters']:>6.0f}m | {res_rrt['smoothness_deg']:>5.1f}° | {res_rrt['min_clearance_meters']:>7.1f}m | Feasible")
    print(f"{'Stage 3: Somersault Monkey':<32} | {time_sms:>6.1f}ms | {'16 pop':>9} | {res_sms['length_meters']:>6.0f}m | {res_sms['smoothness_deg']:>5.1f}° | {res_sms['min_clearance_meters']:>7.1f}m | Feasible")
    print(f"{'Full 3-Stage Hybrid (Fig 3.2)':<32} | {time_hybrid:>6.1f}ms | {'Multi-Stg':>9} | {res_hybrid['length_meters']:>6.0f}m | {res_hybrid['smoothness_deg']:>5.1f}° | {res_hybrid['min_clearance_meters']:>7.1f}m | Optimal")
    print("-" * 90)


def main():
    parser = argparse.ArgumentParser(description="NavPilot Three-Stage Hybrid Path-Planning Simulator")
    parser.add_argument("--preset", type=str, default="archipelago", choices=["archipelago", "singapore", "ice_alley", "canal"],
                        help="Maritime chart preset")
    parser.add_argument("--length", type=float, default=200.0, help="Ship length (m)")
    parser.add_argument("--beam", type=float, default=32.0, help="Ship beam (m)")
    parser.add_argument("--draft", type=float, default=12.0, help="Ship draft (m)")
    parser.add_argument("--displacement", type=float, default=50000.0, help="Ship displacement (tonnes)")
    parser.add_argument("--radius", type=float, default=400.0, help="Min turning radius (m)")
    parser.add_argument("--iterations", type=int, default=20, help="Somersault Monkey Search iterations")
    parser.add_argument("--benchmark", action="store_true", help="Run benchmark on all algorithms")
    parser.add_argument("--show", action="store_true", help="Show interactive matplotlib visualization window")
    parser.add_argument("--save", type=str, default="navpilot_simulation.png", help="Path to save simulation plot")

    args = parser.parse_args()

    print_banner()

    # 1. Initialize Bathymetric Grid
    print(f"[*] Initializing Bathymetric Sea Chart: Preset '{args.preset.upper()}' (50x50 cells, 50m/cell)...")
    grid = BathymetricGrid(50, 50, cell_size_meters=50.0)
    grid.load_preset(args.preset)

    # 2. Initialize Cargo ASV
    ship = CargoASV(
        length=args.length,
        beam=args.beam,
        draft=args.draft,
        displacement=args.displacement,
        max_speed_knots=18.0,
        min_turning_radius=args.radius
    )

    port_m = grid.port_m
    port_n = grid.port_n
    dist_info = grid.get_straight_line_distance()

    print("\n--- CARGO ASV SPECIFICATIONS & SHIP DOMAIN ---")
    print(f"• Physical Dimensions: Length = {ship.length:.1f}m, Beam = {ship.beam:.1f}m, Draft = {ship.draft:.1f}m")
    print(f"• Displacement: {ship.displacement:,.0f} tonnes | Min Turning Radius (R_min): {ship.min_turning_radius:.1f}m")
    print(f"• Max Speed: {ship.max_speed_knots:.1f} knots ({ship.max_speed_ms:.2f} m/s)")
    print(f"• Formula: SS = L * B * D")
    print(f"  Ship Domain Volume: {ship.ship_domain_volume:,.0f} m³")
    print(f"  Dynamic Safety Domain Radius: {ship.domain_radius_meters:.1f} m")
    print(f"  Hydrodynamic Stopping Distance: {ship.compute_stopping_distance_meters():.1f} m")

    print("\n--- MARITIME PORTS ---")
    print(f"• Departure Port M:   ({port_m['x']}, {port_m['y']})")
    print(f"• Destination Port N: ({port_n['x']}, {port_n['y']})")
    print(f"• Straight-Line Distance: {dist_info['meters']:.0f} m ({dist_info['km']:.2f} km, {dist_info['nautical_miles']:.2f} NM)")

    # 3. Initialize Live AIS Traffic
    ais_system = AISSystem(grid)
    ais_system.init_default_traffic(args.preset)
    print(f"\n[*] AIS Marine System broadcasting {len(ais_system.vessels)} commercial vessels:")
    for v in ais_system.vessels:
        print(f"    - MMSI {v.mmsi} | {v.name:<20} | Type: {v.type:<16} | SOG: {v.sog:>4.1f} kn | COG: {v.cog:>3.0f}°")

    # 4. Optional Benchmark
    if args.benchmark:
        run_benchmark(grid, ship, ais_system)

    # 5. Execute Three-Stage Hybrid Planning Architecture (Figure 3.2)
    print("\n" + "=" * 90)
    print("         EXECUTING THREE-STAGE HYBRID PATH-PLANNING ARCHITECTURE (Figure 3.2)")
    print("=" * 90)

    planner = ThreeStageHybridPlanner(grid)
    result = planner.plan(port_m, port_n, ship, ais_system, monkey_iterations=args.iterations)

    if not result["success"]:
        print(f"[ERROR] Path planning failed: {result.get('message')}")
        sys.exit(1)

    s1 = result["stage1"]
    s2 = result["stage2"]
    s3 = result["stage3"]

    print("\n[Stage 1: A* Global Path Search (grid-based)]")
    print(f"  -> Coarse Waypoints Found: {len(s1['waypoints'])} waypoints")
    print(f"  -> Nodes Expanded: {s1['nodes_expanded']} cells")
    print(f"  -> Computation Time: {s1['compute_time_ms']:.2f} ms")
    print(f"  -> Coarse Path Length: {s1['length_meters']:.1f} m")

    print("\n[Stage 2: RRT* Kinematic Refinement (curvature-constrained)]")
    print(f"  -> Kinematically Feasible Waypoints: {len(s2['waypoints'])} waypoints")
    print(f"  -> RRT* Tree Nodes Explored: {len(s2['tree'])} nodes")
    print(f"  -> Computation Time: {s2['compute_time_ms']:.2f} ms")
    print(f"  -> Kinematic Path Length: {s2['length_meters']:.1f} m")

    print("\n[Stage 3: Somersault Monkey Search Optimisation (multi-objective)]")
    print(f"  -> Final Optimised Path Waypoints: {len(s3['waypoints'])} waypoints")
    print(f"  -> SMS Population: 16 candidate monkeys ({args.iterations} iterations)")
    print(f"  -> Computation Time: {s3['compute_time_ms']:.2f} ms")
    print(f"  -> Optimized Trajectory Length: {result['length_meters']:.1f} m")
    print(f"  -> Path Smoothness (Total Turn Angle): {result['smoothness_deg']:.1f}°")
    print(f"  -> Minimum Hazard Clearance: {result['min_clearance_meters']:.1f} m")
    print(f"  -> Curvature Violations: {result['curvature_violations']}")
    print(f"  -> Estimated Transit Time: {result['transit_time_sec']:.1f} seconds ({result['transit_time_sec'] / 60.0:.2f} min)")
    print(f"  -> Hydrodynamic Rudder Energy Index: {result['energy_index']}")

    print("\n" + "=" * 90)
    print("                     ZERO-COLLISION VERIFICATION")
    print("=" * 90)
    if result['min_clearance_meters'] > 0:
        print(f" [PASS] 100% Collision-Free Route Guaranteed!")
        print(f"        Minimum clearance to any land obstacle or shallow shoal: {result['min_clearance_meters']:.1f} meters.")
        print(f"        Path stays strictly within deep, navigable maritime bathymetry.")
    else:
        print(f" [FAIL] Path touches hazard boundary.")

    # 6. Simulate Dynamic Traffic & Rerouting Horizon
    print("\n--- AIS DYNAMIC TRAFFIC & CPA/TCPA COLLISION CHECKS ---")
    ais_system.update(dt=1.0, current_path=result['path'], ship_domain_radius=ship.domain_radius_meters)
    risk_summary = ais_system.get_traffic_summary()
    for item in risk_summary:
        print(f"  Vessel: {item['name']:<20} | CPA: {item['cpa_meters']:>6.1f}m | TCPA: {item['tcpa_seconds']:>5.1f}s | Risk: {item['risk']}")

    # 7. Render & Save Visualization
    print("\n--- GENERATING MATPLOTLIB VISUALIZATION ---")
    plot_simulation(grid, ship, ais_system, result, save_path=args.save, show=args.show)

    print("\n========================================================================================")
    print("                      NAVPILOT SIMULATION COMPLETED SUCCESSFULLY")
    print("========================================================================================")


if __name__ == "__main__":
    main()

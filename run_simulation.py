#!/usr/bin/env python3
"""
NavPilot - Cargo Ship Real-Time Path-Planning Simulator
Entry Point: run_simulation.py

Runs the complete Three-Stage Hybrid Planning Architecture (Figure 3.2):
Stage 1: A* Global Path Search (grid-based)
Stage 2: RRT* Kinematic Refinement (curvature-constrained)
Stage 3: Somersault Monkey Search Optimisation (multi-objective)
"""

import os
import sys

# Add python directory to module path
current_dir = os.path.dirname(os.path.abspath(__file__))
python_dir = os.path.join(current_dir, "python")
if python_dir not in sys.path:
    sys.path.insert(0, python_dir)

from main import main

if __name__ == "__main__":
    main()

@echo off
title NavPilot - Python Three-Stage Hybrid Path-Planning Architecture
echo ===================================================================
echo   NavPilot: Cargo Ship Three-Stage Hybrid Path-Planning Architecture
echo   Figure 3.2: A* Global -^> RRT* Kinematic -^> Somersault Monkey Search
echo ===================================================================
cd /d "%~dp0"
python run_simulation.py --benchmark
echo.
echo Simulation plot saved to navpilot_simulation.png.
pause

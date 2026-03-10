# OMPL Planner Comparison (4-Room Tour)

This project benchmarks and visualizes OMPL motion planners in a 3D 4-room environment.

The task is a tour mission:
`S -> W1 -> W2 -> W3 -> W4 -> S`

Main goals:
- Compare planners such as `RRTConnect`, `PRMstar`, `RRTstar`, and `BITstar`
- Measure success, runtime, and path quality
- Visualize trajectories in 2D/3D, including an interactive web viewer for presentations

Key folders:
- `ompl_cpp_bench/4roomversion/`: C++ benchmark runner, scripts, and analysis tools
- `ompl_cpp_bench/4roomversion/results/`: generated benchmark outputs
- `docs/`: GitHub Pages interactive viewer

# forklift_move_simulation

A Python-based simulation tool for autonomous forklift navigation and motion control, featuring kinematic modeling, Bézier curve path generation, PID control, and animated trajectory visualization.

---

## Features

- **Kinematic Model** — Steering-wheel (single-wheel drive) forklift kinematics with configurable physical parameters
- **Bézier Path Generation** — Arbitrary-order Bézier curves with derivative and curvature support for smooth trajectory planning
- **PID Controller** — Proportional-Integral-Derivative controller for heading and position tracking
- **Multi-threaded Architecture** — Decoupled control thread and simulation thread communicating via queues
- **Visualization** — Static trajectory plots and animated GIF output via Matplotlib
- **YAML Configuration** — All physical, simulation, and visualization parameters managed in a single `config.yaml`

---

## Project Structure

```
forklift_move_simulation/
├── control_sim_monitor.py    # Entry point — launches control & simulation threads
├── forklift_simulator.py     # Kinematics model, simulator core, and visualizer
├── algorithm.py              # PID controller
├── bezier_curve_generator.py # Bézier curve path generator
├── config_manager.py         # YAML config loader and typed config dataclasses
├── config.yaml               # All configurable parameters
├── run.sh                    # Shell script to activate venv and run simulation
├── requirements.txt
└── output/                   # Generated trajectory images and animations
```

---

## Requirements

- Python 3.8+
- numpy >= 1.20.0
- matplotlib >= 3.3.0
- pyyaml >= 5.4.0

---

## Getting Started

### 1. Clone the repository

```bash
git clone https://github.com/your-username/forklift_move_simulation.git
cd forklift_move_simulation
```

### 2. Create and activate a virtual environment

```bash
python3 -m venv venv
source venv/bin/activate
```

### 3. Install dependencies

```bash
pip install -r requirements.txt
```

### 4. Run the simulation

```bash
bash run.sh
```

Or directly:

```bash
python control_sim_monitor.py
```

Output images and animations are saved to the `output/` directory.

---

## Configuration

All parameters are defined in [`config.yaml`](config.yaml):

| Section | Key Parameters |
|---|---|
| `forklift` | `length`, `width`, `steer_dist`, `track_width`, `path_width` |
| `simulation` | `dt` (time step in seconds) |
| `visualization` | figure size, DPI, animation FPS, colors, line widths, pallet dimensions |

---

## Animation Performance Tuning

Generating animated GIFs can be slow for long trajectories. Key knobs:

| Parameter | Effect |
|---|---|
| `sample_step` | Frames sampled per trajectory point — larger = faster, lower quality |
| `dpi` | Render resolution — lower = faster |
| `fps` | Output frame rate |

Recommended presets:

```python
# Fast preview
visualizer.visualize_animated(save_path='output.gif', sample_step=16, dpi=80)

# Balanced (recommended)
visualizer.visualize_animated(save_path='output.gif', sample_step=8, dpi=100)

# High quality
visualizer.visualize_animated(save_path='output.gif', sample_step=2, dpi=150)
```

---

## Architecture Overview

```
control_sim_monitor.py
├── ControlThread          — reads robot state, computes steering speed & angle via PID
│   └── BezierCurveGenerator — generates smooth reference paths
└── SimulationThread       — integrates kinematics, records states, drives visualizer
    ├── ForkliftKinematics — kinematic update equations
    ├── ForkliftSimulator  — state history management
    └── ForkliftVisualizer — static and animated output
```

---

## License

This project is licensed under the terms of the [LICENSE](LICENSE) file.

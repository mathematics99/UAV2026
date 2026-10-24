# Risk-Adaptive EGO-Swarm

A ROS 2 extension of [EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)
that adapts planning parameters online to the geometric risk perceived ahead of
the UAV. High risk reduces speed; narrow passages are detected separately so that
the safety distance is not blindly increased where it would make planning
infeasible.

## Features

- **Online risk evaluation** from forward point clouds and flight state,
  combining obstacle density, braking margin, and passage contraction.
- **Risk smoothing** with EMA and rate limiting to avoid parameter jitter.
- **Decoupled adaptation** of `v_max`, `d_safe`, and `λ_obs` according to each
  parameter's physical role.
- **Runtime injection** into the EGO B-spline optimizer through `/adaptive/*`
  topics — no planner restart required.
- **Experiment tooling**: a forest–funnel–forest map, CSV logging, and metric
  analysis scripts.
- At zero risk, all adaptive parameters reduce exactly to the original
  EGO-Swarm defaults, giving a fair baseline.

## Architecture

```
map_generator / mockamap / narrow_corridor_map.py
        │  /map_generator/global_cloud
        ▼
    uav_simulator  (pcl_render_node, poscmd_2_odom, so3_control, ...)
        │  drone_0_visual_slam/odom
        │  drone_0_pcl_render_node/cloud
        ▼
 sensor_adapter ──► /uav/odom , /uav/pointcloud
        ▼
 risk_evaluator ──► /risk/risk_score , /risk/contraction_risk
        ▼
 adaptive_planner ──► /adaptive/max_velocity
        │             /adaptive/safe_distance
        │             /adaptive/collision_weight
        ▼
 ego_planner (EGOReplanFSM → BsplineOptimizer)
        │  planning/bspline , planner/replan_event
        ▼
 traj_server ──► position_cmd ──► uav_simulator
```

The three new packages live in `src/my_navigation/`:

| Package | Role |
| --- | --- |
| `sensor_adapter` | Unifies simulator or robot topics into `/uav/odom` and `/uav/pointcloud` |
| `risk_evaluator` | Computes the composite risk `R_g` and its components |
| `adaptive_planner` | Smooths risk and maps it to planner parameters |
| `data_logger` | Records time-series and replan-event CSVs; analyzes results |

## Requirements

- Ubuntu 22.04 + ROS 2 Humble
- PCL, VTK (with Qt if building PCL from source), Eigen3, OpenCV, Boost, Armadillo
- Python 3 (`rclpy`)

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop python3-colcon-common-extensions \
  ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-cv-bridge \
  libeigen3-dev libboost-all-dev libopencv-dev libpcl-dev libarmadillo-dev
```

CycloneDDS is strongly recommended; FastDDS may cause noticeable lag:

```bash
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

## Build

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this-repository> ego-planner-swarm

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`scripts/run_experiment.sh` assumes the workspace is `~/ros2_ws` and the source
is at `~/ros2_ws/src/ego-planner-swarm`. Update `WORK` in the script if yours
differs.

## Usage

### Original EGO-Swarm (no risk adaptation)

```bash
# Terminal A
ros2 launch ego_planner rviz.launch.py

# Terminal B
ros2 launch ego_planner single_run_in_sim.launch.py
ros2 launch ego_planner swarm.launch.py
ros2 launch ego_planner swarm_large.launch.py
```

### Risk-adaptive planning with the narrow-corridor map

```bash
ros2 launch ego_planner risk_data_collect.launch.py
```

This starts the simulation, `sensor_adapter`, `risk_evaluator`, and
`adaptive_planner` together. The UAV takes off at `(-15, 0, 0.1)` and flies to
`(15, 0, 1.0)` through a forest–funnel–forest scene.

Record data in a second terminal (from the repository root, after sourcing the
workspace):

```bash
python3 src/my_navigation/data_logger/data_logger.py \
    --out_dir ~/experiments --rate 50 --seed 42 \
    --stop_x 14.5 --settle_dur 2.0
```

The logger stops automatically once the UAV reaches `stop_x` and settles.

### One-click experiment

```bash
cd ~/ros2_ws/src/ego-planner-swarm
chmod +x scripts/run_experiment.sh
OUT_DIR=~/experiments ./scripts/run_experiment.sh
```

The script starts the logger, launches the full stack, waits for completion or
a timeout (default 120 s), shuts everything down, and prints the latest logs and
output files.

### Useful options

- `use_wall_map:=true` selects the forest–funnel–forest map.
- `gap:=0.8` narrows the funnel throat.
- `max_vel:=1.5` changes the entry speed.
- `init_y:=0.3` perturbs the start position for repeated runs.

All node parameters have sensible defaults; tune them in the source or in the
launch file.

## Data and analysis

Each run produces two CSV files:

- `risk_run_<timestamp>_timeseries.csv` — position, velocity, risk components,
  adaptive parameters, and scene region at a fixed rate.
- `risk_run_<timestamp>_replan_events.csv` — one row per replan, with planning
  time and success flag.

Analyze the latest run:

```bash
python3 src/my_navigation/data_logger/analyze_csv.py ~/experiments
```

It writes `run_summary.csv` and prints global and per-region metrics: travel
time, path length, mean/max speed, minimum clearance, risk statistics, adaptive
activation ratio, replan count, and planning time.

## Project structure

```
.
├── src/
│   ├── my_navigation/        # sensor_adapter, risk_evaluator,
│   │                         # adaptive_planner, data_logger
│   ├── planner/              # EGO-Swarm core and launch files
│   └── uav_simulator/        # simulator, map generation, control
├── scripts/run_experiment.sh
└── Readme.md
```

## Troubleshooting

- **Simulation is sluggish** — switch to CycloneDDS and re-source the workspace.
- **No `/risk/*` data** — check that `/uav/odom` and `/uav/pointcloud` are
  publishing, and that `sensor_adapter` input topics match the simulator.
- **No `/adaptive/*` data** — `adaptive_planner` waits for `/risk/risk_score`;
  make sure `risk_evaluator` is running.
- **CSV is empty** — the logger needs `/uav/odom`; also check that `stop_x` is
  not reached too early.

## License and citation

Released under GPL-3.0, inherited from EGO-Swarm. If you use this code in
academic work, please cite the original EGO-Planner / EGO-Swarm papers.

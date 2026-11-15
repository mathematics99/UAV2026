# Historical acquisition entry points

These files preserve the WSL acquisition paths and launch configurations used in the corridor and fixed-throat experiments. The directory layout mirrors their original paths under `/home/ubuntu`. The active ROS stack was ROS 2 Humble on Ubuntu 22.04.5 with CycloneDDS. The core source and modern package entry point are described in [src/README.md](../src/README.md).

| Task | Archived source |
| --- | --- |
| Early corridor sweep | `run_all_experiments.sh` calls the `run_exp.sh` provided under the [corridor directory](<../5.1&5.2_ego_risk_experiments/README.md>). The archived script expects map publishers and an installed ROS overlay at its original paths. |
| Fixed-throat seeds 1–10 | `ros2_ws/src/ego-planner-swarm/src/my_navigation/data_logger/run_composite_multi.sh` |
| Fixed-throat seeds 11–30 | `ros2_ws/src/ego-planner-swarm/src/my_navigation/data_logger/run_composite_seeds_11_30.sh` |
| Per-flight collection | The `data_logger/run_experiment.sh` under the mirrored ROS path and the launch files under `planner/plan_manage/launch/`. |

The historical runners retain their original workspace and output paths. Use the offline recipes for reproducing the released results. To attempt live acquisition, build the corresponding overlay and run in an isolated Ubuntu/WSL instance because the older runners stop processes by executable name.

# 参数敏感性：ROS 2 闭环仿真说明（当前容器未执行）

你的 EGO-Swarm 完整环境需要 ROS 2 Humble / colcon 编译的包，无法在此处的容器中直接运行。这里提供新增的 `risk_sensitivity.launch.py`，不修改原有风险算法，只为启动时传递参数。此文件**仅通过 Python 语法检查，尚未在 ROS 2 实机环境验证**。

## 1. 放入原工程

将 `risk_sensitivity.launch.py` 复制到：

`~/ros2_x1_ws/ros2_ws/src/ego-planner-swarm/src/planner/plan_manage/launch/`

如果你的源码位置不同，以你实际的 `src/planner/plan_manage/launch/` 为准。原工程 `risk_data_collect.launch.py` 与此新文件使用相同的 4 个 ROS 节点。使用自己已有的编译和 source 命令（如 `colcon build --packages-select ego_planner && source install/setup.bash`）。

## 2. 启动同一张离线地图下的某组参数

```bash
ros2 launch ego_planner risk_sensitivity.launch.py \
  gap:=1.4 w_density:=0.20 w_braking:=0.55 w_contraction:=0.25 ema:=0.90
```

另一个终端在启动前开启原数据记录器，保留原始 CSV：

```bash
python3 ~/ros2_x1_ws/ros2_ws/src/ego-planner-swarm/src/my_navigation/data_logger/data_logger.py \
  --out_dir "$HOME/uav_sensitivity_runs/w055_ema090_g14" \
  --rate 50 --seed 42 --stop_x 14.5 --settle_dur 2.0
```

改变 `--out_dir` 使每组参数数据独立保存。各组起始状态、地图宽度、障碍物随机种子、最大允许速度和传感设置务必保持一致；之后再做不同地图的重复实验。

## 3. 建议的参数组合

仅改变制动权重（保持 `w_density=0.20`, `ema=0.90`，同时 `w_contraction=0.80-w_braking`）：

- `(w_braking, w_contraction)=(0.40,0.40)`
- `(0.475,0.325)`
- `(0.55,0.25)`，原文设置
- `(0.625,0.175)`
- `(0.70,0.10)`

仅改变 EMA（保持 `w_density=0.20`, `w_braking=0.55`, `w_contraction=0.25`）：

- `ema=0.70`, `0.80`, `0.90`, `0.95`

建议先在 `gap=1.4` 验证流程，之后在 `gap=1.0` 重复，并检查真实飞行轨迹、完成时间、碰撞、最小距离、速度变化和规划耗时。

## 4. 参数是否正确生效

启动日志 `risk_evaluator` 应显示三项新权重；检查 ROS 2 参数，例如：

```bash
ros2 param get /risk_evaluator weight/braking
ros2 param get /adaptive_planner smoothing/lambda
```

如果运行中参数无法覆盖或 ROS 2 报 `parameter` 名称非法，先核查原工程中的带斜杠参数命名及 ROS 2 版本兼容性；不要默默接受默认参数。

## 5. 对论文的证据分级

现有两张敏感性图是**固定历史风险输入的离线重放**。重新运行上述启动文件并得到各组独立 CSV 后，才可以称为**改变参数后的全流程闭环飞行敏感性试验**。不要将两者混称。

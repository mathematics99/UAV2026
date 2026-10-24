# 风险自适应 EGO-Swarm 无人机路径规划

> Risk-Adaptive Planning for UAVs in Complex Environments — ROS 2 Humble

本项目在 [EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)（ROS 2 版本）的基础上，
增加了一条 **风险感知 → 参数自适应 → 在线注入规划器** 的完整闭环：

1. **风险评估**：根据前向点云与飞行状态，在线计算几何运动风险 `R_g`；
2. **风险平滑**：对 `R_g` 做 EMA 平滑与变化率限制，避免规划参数抖动；
3. **解耦映射**：按参数的物理角色分别调节安全距离 `d_safe`、最大速度 `v_max`、碰撞权重 `λ_obs`；
4. **在线注入**：通过 `/adaptive/*` 话题实时修改 EGO 的 B 样条优化器参数；
5. **实验支撑**：提供"森林—漏斗窄通道—森林"混合地图、CSV 数据记录与指标分析脚本。

当平滑风险 `R̄ = 0` 时，自适应参数会精确退化为原版 EGO-Swarm 的默认值
（`v_max = 2.0`、`dist0 = 0.5`、`lambda_collision = 0.5`），因此可直接作为基线对照。

[中文](#中文文档) | [English](#english)

---

# 中文文档

## 1. 系统结构

| 节点 | 所属包 | 作用 |
| --- | --- | --- |
| `random_forest` / `mockamap_node` / `narrow_corridor_map.py` | `map_generator` / `mockamap` / `ego_planner` | 发布全局地图 `/map_generator/global_cloud` |
| `pcl_render_node`、`poscmd_2_odom`、`so3_control` 等 | `uav_simulator` | 点云渲染、里程计与动力学仿真 |
| `sensor_adapter` | `sensor_adapter` | 把仿真话题统一转发为 `/uav/odom`、`/uav/pointcloud` |
| `risk_evaluator` | `risk_evaluator` | 计算体素占用、制动裕度、通行空间收缩，输出 `R_g` |
| `adaptive_planner` | `adaptive_planner` | 平滑风险并解耦映射为 `v_max`、`d_safe`、`λ_obs` |
| `ego_planner_node` | `ego_planner` | EGO-Swarm 规划器，订阅自适应参数并注入优化器 |
| `traj_server` | `ego_planner` | 把 B 样条轨迹转换为位置指令 |
| `data_logger.py` | `my_navigation/data_logger` | 记录时序数据与重规划事件 CSV |

数据流：

```
 map_generator / mockamap / narrow_corridor_map.py
            │  /map_generator/global_cloud
            ▼
        uav_simulator  (pcl_render_node / poscmd_2_odom / so3_control)
            │  drone_0_visual_slam/odom
            │  drone_0_pcl_render_node/cloud
            ▼
     sensor_adapter ──► /uav/odom , /uav/pointcloud
            ▼
     risk_evaluator ──► /risk/risk_score
            │            /risk/contraction_risk
            ▼
    adaptive_planner ──► /adaptive/max_velocity
            │            /adaptive/safe_distance
            │            /adaptive/collision_weight
            ▼
      ego_planner (EGOReplanFSM → BsplineOptimizer)
            │  planning/bspline , planner/replan_event
            ▼
        traj_server ──► position_cmd ──► uav_simulator
```

> `sensor_adapter` 的输入话题默认是 `/visual/odom` 与 `/lidar/points`；
> 在 `risk_data_collect.launch.py` 中已被改为仿真的
> `drone_0_visual_slam/odom` 与 `drone_0_pcl_render_node/cloud`。
> 接入实机时只需修改这两个输入话题。

## 2. 风险模型

### 2.1 综合几何风险

```
R_g = w_ρ · ρ_occ + w_b · R_b + w_c · R_c      ∈ [0, 1]
```

| 分量 | 含义 | 计算方式 |
| --- | --- | --- |
| `ρ_occ` | 前向 ROI 体素占用率 | ROI：机体前向 `x∈[0,5] m`、`y=±2 m`、`z=±1.5 m`，体素 `0.2 m` |
| `R_b` | 动态制动裕度风险 | `d_brake = (use_d0 ? d0 : 0) + v·τ + v²/(2a_b)`，`R_b = clip(d_brake/(d_min+ε), 0, 1)` |
| `R_c` | 前向通行空间收缩风险 | 沿 X 分 10 片，片内开阔度 `F_i = 1 - O_i`；整体收缩 `C_global` + 局部突变 `G_c`；`R_c = η·C_global + (1-η)·G_c` |

默认权重：`w_ρ = 0.20`、`w_b = 0.55`、`w_c = 0.25`。

`C_global` 用近端 3 片与远端 3 片的平均开阔度计算：

```
C_global = clip((F_near - F_far) / (F_near + ε), 0, 1)
G_c      = max_i clip((F_i - F_{i+1}) / (F_i + ε), 0, 1)
```

### 2.2 风险平滑与解耦映射

`adaptive_planner` 对 `R_g` 与 `R_c` 分别做 EMA 平滑：

```
R̄  = λ · R̄_prev + (1-λ) · R_g
|ΔR̄| ≤ max_delta          (逐周期变化率限制)
```

收缩门控量 `G_gate = 1 - R̄_c`（开阔区接近 1，窄道接近 0），随后按参数角色分别映射：

```
v_max   = v_high - R̄ · (v_high - v_low)                                  # 始终随风险降速
d_safe  = d_base + G_gate · R̄ · (d_max - d_base)                         # 受收缩风险门控
λ_obs   = λ_min + (1 - c_gain · R̄_c) · R̄^γ · (λ_max - λ_min)             # 压缩范围 + 收缩门控
```

设计要点：同一个风险不应统一驱动所有参数向"更保守"变化。
高速时降速始终成立；而安全距离与碰撞权重在窄道中会被收缩门控抑制，
避免"窄道里盲目增大安全距离导致无解"。

## 3. 环境与依赖

- Ubuntu 22.04 + ROS 2 Humble
- PCL、VTK（从源码编译 PCL 时需勾选 Qt）、Eigen3、OpenCV、Boost、Armadillo
- CycloneDDS（强烈建议，见 §3.2）
- Python 3（`rclpy`，用于数据记录脚本）

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop python3-colcon-common-extensions \
  ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-cv-bridge \
  libeigen3-dev libboost-all-dev libopencv-dev libpcl-dev libarmadillo-dev
```

### 3.1 编译

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <this-repository> ego-planner-swarm

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

> `scripts/run_experiment.sh` 假定工作空间为 `~/ros2_ws`、源码位于
> `~/ros2_ws/src/ego-planner-swarm`。若路径不同，请修改脚本顶部的 `WORK` 变量。

### 3.2 切换到 CycloneDDS

原版 EGO-Swarm 在 FastDDS 下可能出现明显卡顿，建议切换：

```bash
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
```

验证：

```bash
ros2 doctor --report | grep "RMW middleware"
```

输出包含 `rmw_cyclonedds_cpp` 即成功。

## 4. 运行

### 4.1 原版 EGO-Swarm（不含风险自适应）

终端 A：

```bash
ros2 launch ego_planner rviz.launch.py
```

终端 B：

```bash
# 单机
ros2 launch ego_planner single_run_in_sim.launch.py

# 集群
ros2 launch ego_planner swarm.launch.py

# 大规模集群
ros2 launch ego_planner swarm_large.launch.py
```

### 4.2 风险自适应 + 窄通道实验

`risk_data_collect.launch.py` 会一次性启动：
`single_run_in_sim`（含窄通道地图）+ `sensor_adapter` + `risk_evaluator` + `adaptive_planner`。

```bash
ros2 launch ego_planner risk_data_collect.launch.py
```

默认参数：`point_num=1`、`use_wall_map=true`。
无人机从 `(-15, 0, 0.1)` 起飞，飞向 `(15, 0, 1.0)`。

另开终端启动数据记录（在仓库根目录执行；需先 `source ~/ros2_ws/install/setup.bash`，
因为脚本依赖 `traj_utils` 的 Python 消息）：

```bash
python3 src/my_navigation/data_logger/data_logger.py \
    --out_dir ~/experiments --rate 50 --seed 42 \
    --stop_x 14.5 --settle_dur 2.0
```

记录器在无人机到达 `stop_x` 并稳定 `settle_dur` 秒后自动退出，
输出：

- `risk_run_<时间戳>_timeseries.csv`
- `risk_run_<时间戳>_replan_events.csv`

### 4.3 一键实验脚本

```bash
cd ~/ros2_ws/src/ego-planner-swarm
chmod +x scripts/run_experiment.sh
OUT_DIR=~/experiments ./scripts/run_experiment.sh
```

脚本流程：启动记录器 → 等待 2 s → 启动完整规划栈 → 等待记录器自动结束
（最长 `RUN_DURATION` 秒，默认 120）→ 关闭所有进程 → 打印日志尾部与输出文件。

可用环境变量：

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `OUT_DIR` | `/mnt/c/Users/lenovo/Desktop` | CSV 输出目录 |
| `RUN_DURATION` | `120` | 硬超时（秒） |

### 4.4 常用启动参数

`single_run_in_sim.launch.py` 支持的参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_wall_map` | `False` | `True` 时使用混合森林 + 漏斗窄通道地图 |
| `gap` | `1.0` | 漏斗喉部宽度（m），仅 `use_wall_map=True` 生效 |
| `wide` | `5.0` | 漏斗开口宽度（m），收缩强度约为 `wide/gap` |
| `max_vel` | `2.0` | 规划器最大速度（入口速度扫参用） |
| `init_y` | `0.0` | 起始横向偏移（种子扰动/重复实验用） |
| `point_num` | `4` | 预设航点数量；风险实验建议设为 `1` |
| `use_mockamap` | `False` | `True` 使用 mockamap；`False` 使用 random forest |
| `use_dynamic` | `False` | 是否启用动力学仿真 |
| `map_size_x/y/z` | `50/25/2` | 地图尺寸 |
| `odom_topic` | `visual_slam/odom` | 里程计话题 |

示例：

```bash
# 更窄的喉部
ros2 launch ego_planner risk_data_collect.launch.py point_num:=1 use_wall_map:=true gap:=0.8

# 入口速度扫描
ros2 launch ego_planner risk_data_collect.launch.py point_num:=1 use_wall_map:=true max_vel:=1.5

# 使用原版随机森林地图（不含风险自适应节点）
ros2 launch ego_planner single_run_in_sim.launch.py use_wall_map:=false use_mockamap:=false
```

### 4.5 消融实验

**基线（无风险自适应）**：只启动原版规划栈，不启动三个新增节点：

```bash
ros2 launch ego_planner single_run_in_sim.launch.py \
    use_wall_map:=true point_num:=1 max_vel:=2.0
```

此时 `v_max=2.0`、`dist0=0.5`、`lambda_collision=0.5`，与 `R̄=0` 时的自适应参数一致。

**C2：关闭收缩门控**：给 `adaptive_planner` 传入 `gate/enabled:=false`。
`risk_data_collect.launch.py` 未暴露该参数，可在其中 `adaptive_planner` 节点上添加：

```python
adaptive_planner = Node(
    package='adaptive_planner', executable='adaptive_planner_node',
    name='adaptive_planner', output='screen',
    parameters=[{'gate/enabled': False}],   # ← 新增
)
```

效果：`G_gate ≡ 1`、`c_gain` 置 0，即安全距离与碰撞权重不再受收缩风险门控。

**D2：恢复旧制动公式**：给 `risk_evaluator` 传入 `braking/use_d0:=true`：

```python
risk_evaluator = Node(
    package='risk_evaluator', executable='risk_evaluator_node',
    name='risk_evaluator', output='screen',
    parameters=[{'braking/use_d0': True}],  # ← 新增
)
```

效果：`d_brake = d0 + v·τ + v²/(2a_b)`，而非 V4.1 的 `v·τ + v²/(2a_b)`。

## 5. 数据与指标

### 5.1 时序 CSV 字段

| 字段 | 来源话题 | 含义 |
| --- | --- | --- |
| `t_s, x, y, z, vx, vy, vz, v` | `/uav/odom` | 时间与无人机状态 |
| `d_min` | `/risk/min_distance` | 前向 ROI 内最近障碍距离 |
| `rho` | `/risk/obstacle_density` | 体素占用率 `ρ_occ` |
| `C_global` | `/risk/contraction_global` | 整体通行空间收缩 |
| `G_c` | `/risk/contraction_gradient` | 局部瓶颈突变梯度 |
| `R_c` | `/risk/contraction_risk` | 收缩风险 |
| `R_b` | `/risk/braking_risk` | 制动裕度风险 |
| `R_g` | `/risk/risk_score` | 综合几何风险 |
| `r_bar, rc_bar` | `/adaptive/smoothed_risk`、`/adaptive/smoothed_contraction` | 平滑后的风险 |
| `v_max` | `/adaptive/max_velocity` | 自适应最大速度 |
| `d_safe` | `/adaptive/safe_distance` | 自适应安全距离 |
| `lambda_obs` | `/adaptive/collision_weight` | 自适应碰撞权重 |
| `gate` | `/adaptive/contraction_gate` | 收缩门控量 `G_gate` |
| `scene_region` | 由 `x` 判定 | 场景分区（见下） |
| `seed` | 命令行参数 | 实验种子 |

重规划事件 CSV：`timestamp_s, planning_time_ms, success, count`，
来自 `/planner/replan_event`。

### 5.2 场景分区

以 `corridor_x=0`、`corridor_half_len=5`、`throat_half_len=1` 为例：

| 分区 | X 范围 | 含义 |
| --- | --- | --- |
| `forest_before` | `x < -5` | 进入通道前的森林 |
| `contraction` | `-5 ≤ x < -1` | 漏斗收缩段 |
| `throat` | `-1 ≤ x ≤ 1` | 喉部 |
| `expansion` | `1 < x ≤ 5` | 漏斗扩张段 |
| `forest_after` | `x > 5` | 出通道后的森林 |

可通过 `data_logger.py` 的 `--corridor_x`、`--corridor_half_len`、`--throat_half_len` 调整。

### 5.3 指标分析

```bash
python3 src/my_navigation/data_logger/analyze_csv.py ~/experiments
```

脚本读取目录中**最新**的一组 CSV，输出 `run_summary.csv`，并在终端打印：

- 全局：通行时间、路径长度、平均/最大速度、最小净空；
- 风险：`R_b`/`R_g` 的均值、P95、最大值，`R_b` 饱和比例（`R_b > 0.8`）；
- 自适应：`adaptive_active_ratio`（`v_max < 0.95·v_high` 的时间占比）；
- 规划：重规划次数、规划耗时均值/P95/最大值；
- 分区表：各场景分区的样本数、速度、风险、净空、自适应激活比例。

> `analyze_csv.py` 中的 `V_HIGH = 2.0` 是硬编码的。若修改了
> `adaptive_planner` 的 `speed/v_high`，请同步修改该常量，
> 否则 `adaptive_active_ratio` 会失真。

## 6. 话题一览

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/uav/odom` | `nav_msgs/Odometry` | 发布（sensor_adapter） | 统一后的里程计 |
| `/uav/pointcloud` | `sensor_msgs/PointCloud2` | 发布（sensor_adapter） | 统一后的点云 |
| `/risk/obstacle_density` | `std_msgs/Float64` | 发布（risk_evaluator） | `ρ_occ` |
| `/risk/min_distance` | `std_msgs/Float64` | 发布 | 最近障碍距离 |
| `/risk/current_velocity` | `std_msgs/Float64` | 发布 | 当前速度 |
| `/risk/braking_distance` | `std_msgs/Float64` | 发布 | 所需制动距离（调试） |
| `/risk/braking_risk` | `std_msgs/Float64` | 发布 | `R_b` |
| `/risk/contraction_global` | `std_msgs/Float64` | 发布 | `C_global`（调试） |
| `/risk/contraction_gradient` | `std_msgs/Float64` | 发布 | `G_c`（调试） |
| `/risk/contraction_risk` | `std_msgs/Float64` | 发布 | `R_c` |
| `/risk/risk_score` | `std_msgs/Float64` | 发布 | `R_g` |
| `/risk/safe_distance` | `std_msgs/Float64` | 发布 | 风险侧安全距离（仅供参考，规划器不使用） |
| `/risk/timing_ms` | `std_msgs/Float64` | 发布 | 风险计算耗时 |
| `/adaptive/safe_distance` | `std_msgs/Float64` | 发布（adaptive_planner） | 注入 `dist0` |
| `/adaptive/max_velocity` | `std_msgs/Float64` | 发布 | 注入 `max_vel` |
| `/adaptive/collision_weight` | `std_msgs/Float64` | 发布 | 注入 `lambda_collision` |
| `/adaptive/smoothed_risk` | `std_msgs/Float64` | 发布 | `R̄` |
| `/adaptive/smoothed_contraction` | `std_msgs/Float64` | 发布 | `R̄_c` |
| `/adaptive/contraction_gate` | `std_msgs/Float64` | 发布 | `G_gate` |
| `/adaptive/timing_ms` | `std_msgs/Float64` | 发布 | 自适应计算耗时 |
| `/planner/replan_event` | `traj_utils/ReplanEvent` | 发布（ego_planner） | 重规划事件 |

## 7. 关键参数

### 7.1 `risk_evaluator`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `input_odom_topic` | `/uav/odom` | 输入里程计 |
| `input_cloud_topic` | `/uav/pointcloud` | 输入点云 |
| `roi/x_min, x_max` | `0.0, 5.0` | 机体系前向 ROI |
| `roi/y_half, z_half` | `2.0, 1.5` | ROI 横向/垂向半宽 |
| `voxel_size` | `0.2` | 体素边长（m） |
| `braking/d0` | `0.3` | 基础制动距离（`use_d0` 时才计入） |
| `braking/tau` | `0.3` | 反应时间（s） |
| `braking/a_b` | `2.0` | 制动减速度（m/s²） |
| `braking/epsilon` | `0.1` | 防止除零 |
| `braking/use_d0` | `false` | 消融 D2：恢复 `d0 + v·τ + v²/(2a_b)` |
| `contraction/slices` | `10` | 前向切片数 |
| `contraction/eta` | `0.6` | `R_c = η·C_global + (1-η)·G_c` |
| `contraction/epsilon` | `0.1` | 防止除零 |
| `weight/density, braking, contraction` | `0.20, 0.55, 0.25` | `R_g` 权重（和为 1） |
| `safe/min, safe/max` | `0.5, 1.5` | 风险侧 `d_safe` 映射范围（仅供观察） |

### 7.2 `adaptive_planner`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `smoothing/lambda` | `0.9` | EMA 系数，越大越平滑 |
| `smoothing/max_delta` | `0.05` | `R̄` 单周期最大变化量 |
| `safe/d_base` | `0.5` | 基础安全距离（= EGO `dist0`） |
| `safe/d_max` | `1.0` | 安全距离上限 |
| `speed/v_low` | `0.8` | 高风险时的速度下界 |
| `speed/v_high` | `2.0` | 低风险速度（= EGO `max_vel`） |
| `weight/lambda_min` | `0.5` | 碰撞权重下界（= EGO `lambda_collision`） |
| `weight/lambda_max` | `2.0` | 碰撞权重上界 |
| `weight/gamma` | `2.0` | 风险幂次 |
| `weight/contraction_gain` | `0.5` | 收缩门控对权重的抑制强度 |
| `gate/enabled` | `true` | 消融 C2：设为 `false` 关闭收缩门控 |

## 8. 目录结构

```
src/
├── my_navigation/
│   ├── sensor_adapter/      # 话题适配：仿真/实机 → /uav/*
│   ├── risk_evaluator/      # 几何运动风险评估 → /risk/*
│   ├── adaptive_planner/    # 风险平滑与解耦映射 → /adaptive/*
│   └── data_logger/         # 数据记录与离线分析（Python 脚本）
├── planner/                 # EGO-Swarm 核心（plan_manage / plan_env / bspline_opt ...）
│   └── plan_manage/launch/
│       ├── single_run_in_sim.launch.py   # 单机仿真（含窄通道地图参数）
│       ├── risk_data_collect.launch.py   # 风险自适应完整栈
│       ├── narrow_corridor_map.py        # 森林 + 漏斗窄通道地图发布器
│       ├── swarm.launch.py               # 集群
│       └── swarm_large.launch.py         # 大规模集群
└── uav_simulator/           # 仿真器、地图生成、控制器
scripts/
└── run_experiment.sh        # 一键实验：记录器 + 完整栈 + 收尾
```

## 9. 常见问题

**Q1：仿真明显卡顿。**
按 §3.2 切换到 CycloneDDS，并确认每次新终端都 `source` 了工作空间。

**Q2：`/risk/*` 没有数据。**
先确认 `/uav/odom` 与 `/uav/pointcloud` 有消息：

```bash
ros2 topic hz /uav/odom
ros2 topic hz /uav/pointcloud
```

若为空，检查 `sensor_adapter` 的输入话题是否与仿真一致
（默认 `drone_0_visual_slam/odom`、`drone_0_pcl_render_node/cloud`）。

**Q3：`/adaptive/*` 没有数据。**
`adaptive_planner` 必须先收到 `/risk/risk_score` 才会开始输出；
确认 `risk_evaluator` 已正常启动，且 `/risk/risk_score` 有数据。

**Q4：CSV 一直不生成或立即结束。**
记录器需要 `/uav/odom` 才会写数据；若 `--stop_x` 设得过小，
无人机会很快触发自动停止。可增大 `--stop_x` 或去掉该参数。

**Q5：编译找不到 PCL / VTK / OpenCV。**
确认已安装 §3 中的依赖；若 PCL 为源码编译，需启用 VTK 与 Qt 支持。

## 10. 说明

- 本项目基于 [ZJU-FAST-Lab/ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm)，
  遵循 GPL-3.0 许可证（见 `LICENSE`）。
- 若用于学术工作，请同时引用 EGO-Swarm / EGO-Planner 的原始论文。
- 仓库中的 `build/`、`install/`、`log/` 为构建产物，更新代码后建议重新
  `colcon build`，不要直接复用旧产物。

---

# English

## Overview

This repository extends the ROS 2 version of
[EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) with a closed-loop
**risk-aware adaptive planning** pipeline:

1. **Risk evaluation** — computes a geometric-motion risk `R_g` from forward
   point clouds and flight state;
2. **Risk smoothing** — EMA smoothing plus rate limiting to avoid parameter jitter;
3. **Decoupled mapping** — maps risk to `d_safe`, `v_max`, and `λ_obs` according to
   each parameter's physical role;
4. **Online injection** — updates the EGO B-spline optimizer at runtime via
   `/adaptive/*` topics;
5. **Experiment tooling** — a forest–funnel–forest map, CSV logging, and metric
   analysis scripts.

At `R̄ = 0`, the adaptive parameters degrade exactly to the original EGO-Swarm
defaults (`v_max = 2.0`, `dist0 = 0.5`, `lambda_collision = 0.5`), so a fair
baseline is obtained by simply not launching the three new nodes.

## Risk model

```
R_g = w_ρ · ρ_occ + w_b · R_b + w_c · R_c
```

- `ρ_occ`: voxel occupancy in the forward body-frame ROI
  (`x∈[0,5] m`, `y=±2 m`, `z=±1.5 m`, voxel `0.2 m`).
- `R_b`: braking-margin risk,
  `d_brake = (use_d0 ? d0 : 0) + v·τ + v²/(2a_b)`,
  `R_b = clip(d_brake/(d_min+ε), 0, 1)`.
- `R_c`: forward passage contraction, computed over 10 X-slices:
  `F_i = 1 - O_i`, `C_global = clip((F_near-F_far)/(F_near+ε),0,1)`,
  `G_c = max_i clip((F_i-F_{i+1})/(F_i+ε),0,1)`,
  `R_c = η·C_global + (1-η)·G_c`.

Default weights: `0.20 / 0.55 / 0.25`.

## Adaptive mapping

```
R̄      = λ·R̄_prev + (1-λ)·R_g ,  |ΔR̄| ≤ max_delta
G_gate = 1 - R̄_c
v_max  = v_high - R̄·(v_high - v_low)
d_safe = d_base + G_gate·R̄·(d_max - d_base)
λ_obs  = λ_min + (1 - c_gain·R̄_c)·R̄^γ·(λ_max - λ_min)
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

Dependencies: ROS 2 Humble, PCL, VTK (with Qt when building PCL from source),
Eigen3, OpenCV, Boost, Armadillo.

CycloneDDS is recommended:

```bash
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Run

Baseline EGO-Swarm:

```bash
ros2 launch ego_planner rviz.launch.py
ros2 launch ego_planner single_run_in_sim.launch.py
ros2 launch ego_planner swarm.launch.py
ros2 launch ego_planner swarm_large.launch.py
```

Risk-adaptive stack with the forest–funnel–forest map:

```bash
ros2 launch ego_planner risk_data_collect.launch.py
```

This starts the simulation, `sensor_adapter`, `risk_evaluator`, and
`adaptive_planner`. Defaults: `point_num=1`, `use_wall_map=true`.

Log data in another terminal (run from the repository root, after
`source ~/ros2_ws/install/setup.bash`):

```bash
python3 src/my_navigation/data_logger/data_logger.py \
    --out_dir ~/experiments --rate 50 --seed 42 \
    --stop_x 14.5 --settle_dur 2.0
```

Or run the full experiment non-interactively:

```bash
OUT_DIR=~/experiments ./scripts/run_experiment.sh
```

Useful launch arguments: `use_wall_map`, `gap`, `wide`, `max_vel`, `init_y`,
`point_num`, `use_mockamap`, `use_dynamic`.

## Ablations

- **Baseline** — launch only `single_run_in_sim.launch.py` (no risk-adaptive nodes).
- **C2 (no contraction gate)** — set `gate/enabled := false` on `adaptive_planner`
  (add `parameters=[{'gate/enabled': False}]` to the node in
  `risk_data_collect.launch.py`).
- **D2 (legacy braking formula)** — set `braking/use_d0 := true` on `risk_evaluator`.

## Analyze

```bash
python3 src/my_navigation/data_logger/analyze_csv.py ~/experiments
```

Produces `run_summary.csv` and prints global and per-region metrics:
travel time, path length, mean/max speed, minimum clearance, mean/P95/max
`R_b`/`R_g`, `R_b` saturation ratio, adaptive-active ratio, replan count, and
planning-time statistics.

Note: `V_HIGH = 2.0` is hard-coded in `analyze_csv.py`; update it if you change
`speed/v_high`.

## License

GPL-3.0, inherited from EGO-Swarm. Please cite the original EGO-Planner /
EGO-Swarm papers when using this code in academic work.

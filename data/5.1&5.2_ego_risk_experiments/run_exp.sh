#!/bin/bash
# 统一实验运行器 (窄通道论文实验 1/2/3/4)
# 用法: run_exp.sh <label> <map_mode> <gap> <gate> <max_vel> <adaptive> [duration_s]
#   map_mode : narrow | funnel
#   gap      : 窄通道总宽(米, narrow 用; funnel 忽略, 用默认 口3.0/喉1.2)
#   gate     : 1=收缩门控开启(C1/完整V4.1), 0=关闭(C2 消融)
#   max_vel  : EGO 最大速度(入口速度扫参用; 2.0 为原始 EGO 基线)
#   adaptive : 1=启动 adaptive_planner(本文方法), 0=不启动(原始 EGO, 仅被动 risk 测量)
set -x
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source /home/ubuntu/ego-planner-swarm/install/setup.bash

LABEL=${1:-exp}
MAP_MODE=${2:-narrow}
GAP=${3:-1.4}
GATE=${4:-1}
MAX_VEL=${5:-2.0}
ADAPTIVE=${6:-1}
DURATION=${7:-120}

cd /home/ubuntu

# 0) 预清理: 杀掉上一次实验可能残留的节点进程 (防止组件加载/命名冲突导致控制器加载失败)
cleanup_nodes() {
  pkill -f "component_container" 2>/dev/null
  pkill -f "so3_quadrotor_simulator" 2>/dev/null
  pkill -f "ego_planner_node" 2>/dev/null
  pkill -f "traj_server" 2>/dev/null
  pkill -f "pcl_render_node" 2>/dev/null
  pkill -f "odom_visualization" 2>/dev/null
  pkill -f "poscmd_2_odom" 2>/dev/null
  pkill -f "risk_evaluator" 2>/dev/null
  pkill -f "sensor_adapter" 2>/dev/null
  pkill -f "adaptive_planner" 2>/dev/null
  pkill -f "narrow_channel_map.py" 2>/dev/null
  pkill -f "funnel_channel_map.py" 2>/dev/null
  sleep 1
}
cleanup_nodes

# 1) 地图发布器 (transient_local latch)
if [ "$MAP_MODE" = "funnel" ]; then
  python3 /home/ubuntu/funnel_channel_map.py > map_${LABEL}.log 2>&1 &
else
  python3 /home/ubuntu/narrow_channel_map.py "$GAP" > map_${LABEL}.log 2>&1 &
fi
MAP_PID=$!
sleep 2

# 2) sim: use_wall_map -> 跳过内置地图, 用外部通道地图; 动力学仿真
ros2 launch ego_planner single_run_in_sim.launch.py \
  use_wall_map:=True use_dynamic:=True max_vel:=${MAX_VEL} > sim_${LABEL}.log 2>&1 &
SIM_PID=$!
sleep 8

# 3) 感知/风险栈 (sensor_adapter + risk_evaluator, 始终启动用于被动测量)
ros2 run sensor_adapter sensor_adapter_node --ros-args \
  -p input_odom_topic:=/drone_0_visual_slam/odom \
  -p input_cloud_topic:=/drone_0_pcl_render_node/cloud > sensor_${LABEL}.log 2>&1 &
SENSOR_PID=$!
ros2 run risk_evaluator risk_evaluator_node > risk_${LABEL}.log 2>&1 &
RISK_PID=$!

# 4) adaptive_planner (本文方法才启动)
if [ "$ADAPTIVE" = "1" ]; then
  if [ "$GATE" = "1" ]; then
    ros2 run adaptive_planner adaptive_planner_node > adaptive_${LABEL}.log 2>&1 &
  else
    ros2 run adaptive_planner adaptive_planner_node --ros-args -p gate/enabled:=false > adaptive_${LABEL}.log 2>&1 &
  fi
  ADAPT_PID=$!
fi
sleep 3

# 5) bag record (先于 trigger, 捕获起飞全过程; 补录全部 risk 分量)
rm -rf /home/ubuntu/${LABEL}_bag
ros2 bag record -o /home/ubuntu/${LABEL}_bag \
  /drone_0_visual_slam/odom \
  /map_generator/global_cloud \
  /risk/risk_score /risk/contraction_risk /risk/min_distance \
  /risk/braking_risk /risk/braking_distance \
  /risk/contraction_global /risk/contraction_gradient \
  /risk/current_velocity /risk/obstacle_density \
  /adaptive/smoothed_risk /adaptive/smoothed_contraction \
  /adaptive/contraction_gate /adaptive/safe_distance \
  /adaptive/max_velocity /adaptive/collision_weight \
  > bag_${LABEL}.log 2>&1 &
BAG_PID=$!
sleep 6

# 6) trigger (起飞)
ros2 topic pub -r 1 /traj_start_trigger geometry_msgs/msg/PoseStamped "{}" > trigger_${LABEL}.log 2>&1 &
TRIG_PID=$!

# 7) 记录 DURATION 秒
sleep $((DURATION + 5))

# 8) 清理 (先 SIGTERM 再 SIGKILL)
if [ "$ADAPTIVE" = "1" ]; then
  kill $BAG_PID $TRIG_PID $ADAPT_PID $RISK_PID $SENSOR_PID $SIM_PID $MAP_PID 2>/dev/null
  sleep 3
  kill -9 $BAG_PID $TRIG_PID $ADAPT_PID $RISK_PID $SENSOR_PID $SIM_PID $MAP_PID 2>/dev/null
else
  kill $BAG_PID $TRIG_PID $RISK_PID $SENSOR_PID $SIM_PID $MAP_PID 2>/dev/null
  sleep 3
  kill -9 $BAG_PID $TRIG_PID $RISK_PID $SENSOR_PID $SIM_PID $MAP_PID 2>/dev/null
fi
cleanup_nodes
echo "DONE ${LABEL}"

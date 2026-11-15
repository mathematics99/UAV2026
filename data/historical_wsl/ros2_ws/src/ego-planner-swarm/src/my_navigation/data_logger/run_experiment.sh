#!/usr/bin/env bash
# Run one §5.3 full-flight experiment (baseline or adaptive) and post-process it.
#
# Usage: bash run_experiment.sh <baseline|adaptive> <out_root> [gap]
#
# The data logger is started FIRST so it captures the drone from its initial
# pose (x = -15) before takeoff; the simulator is launched a moment later.
MODE="${1:-adaptive}"
OUT_ROOT="${2:-/mnt/d/0A大学资料/大三/无人机论文/exp_5_3_data}"
GAP="${3:-1.4}"
SEED="${4:-42}"

source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
set -e

DL=/home/ubuntu/ros2_ws/src/ego-planner-swarm/src/my_navigation/data_logger
OUT_DIR="$OUT_ROOT/$MODE/seed$(printf '%02d' "$SEED")"
mkdir -p "$OUT_DIR"

case "$MODE" in
  baseline)   LAUNCH=baseline_collect.launch.py;  ABLATION_ARGS="" ;;
  adaptive)   LAUNCH=risk_data_collect.launch.py; ABLATION_ARGS="" ;;
  speed_only) LAUNCH=risk_data_collect.launch.py; ABLATION_ARGS="safe_d_max:=0.5 lambda_max:=0.5" ;;
  *) echo "unknown mode $MODE"; exit 1 ;;
esac

cleanup() {
  set +e
  [ -n "${LAUNCH_PID:-}" ] && kill "$LAUNCH_PID" 2>/dev/null
  pkill -f "ego_planner_node" 2>/dev/null
  pkill -f "traj_server" 2>/dev/null
  pkill -f "so3_quadrotor_simulator" 2>/dev/null
  pkill -f "component_container" 2>/dev/null
  pkill -f "pcl_render_node" 2>/dev/null
  pkill -f "odom_visualization" 2>/dev/null
  pkill -f "sensor_adapter_node" 2>/dev/null
  pkill -f "risk_evaluator_node" 2>/dev/null
  pkill -f "adaptive_planner_node" 2>/dev/null
  pkill -f "narrow_corridor_map" 2>/dev/null
  pkill -f "ros2 launch" 2>/dev/null
  sleep 2
  set -e
  return 0
}
trap cleanup EXIT

LOGGER_LOG=/tmp/${MODE}_logger.log
LAUNCH_LOG=/tmp/${MODE}_launch.log
rm -f "$LOGGER_LOG" "$LAUNCH_LOG"

# 1) start the data logger first so it captures the very first odometry sample
echo "[$MODE] starting data logger (odom=/drone_0_visual_slam/odom) ..."
python3 "$DL/data_logger.py" \
  --out_dir "$OUT_DIR" --odom /drone_0_visual_slam/odom \
  --cmd_topic /drone_0_planning/pos_cmd --stop_x 14.5 --settle_dur 2.0 --rate 50 \
  --seed "$SEED" \
  > "$LOGGER_LOG" 2>&1 &
LOGGER_PID=$!

sleep 1

# 2) launch the full-flight stack
echo "[$MODE] launching $LAUNCH (gap=$GAP, seed=$SEED, use_dynamic=true, ablation='$ABLATION_ARGS') ..."
ros2 launch ego_planner "$LAUNCH" use_dynamic:=true use_wall_map:=true gap:="$GAP" seed:="$SEED" $ABLATION_ARGS > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

# 3) wait for the logger to finish (max 260 s)
for i in $(seq 1 260); do
  if ! kill -0 "$LOGGER_PID" 2>/dev/null; then
    break
  fi
  sleep 1
done

# safety: ensure the logger has exited
if kill -0 "$LOGGER_PID" 2>/dev/null; then
  echo "[$MODE] logger timeout, killing ..."
  kill "$LOGGER_PID" 2>/dev/null || true
fi
wait "$LOGGER_PID" 2>/dev/null || true

# 4) stop the stack and analyze
cleanup
trap - EXIT

echo "[$MODE] analyzing ..."
python3 "$DL/analyze_csv.py" "$OUT_DIR" --label "$MODE"

echo "[$MODE] done. files:"
ls -la "$OUT_DIR"

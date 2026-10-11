#!/usr/bin/env bash
# note: no `set -u` here — ROS setup.bash references unset vars under nounset

# --- environment ------------------------------------------------------------
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORK=~/ros2_ws/src/ego-planner-swarm
LOGGER=$WORK/src/my_navigation/data_logger/data_logger.py
OUT_DIR=${OUT_DIR:-/mnt/c/Users/lenovo/Desktop}
RUN_DURATION=${RUN_DURATION:-120}   # hard safety-net timeout (seconds)

rm -f /tmp/risk_launch.log /tmp/risk_logger.log

echo "[run] starting data logger (auto-stops when drone reaches goal)"
python3 "$LOGGER" --out_dir "$OUT_DIR" --rate 50 --seed 42 \
    --stop_x 14.5 --settle_dur 2.0 > /tmp/risk_logger.log 2>&1 &
LOGGER_PID=$!

sleep 2

echo "[run] starting full stack (map + sim + planner + risk + adaptive)"
ros2 launch ego_planner risk_data_collect.launch.py \
    point_num:=1 use_wall_map:=true > /tmp/risk_launch.log 2>&1 &
LAUNCH_PID=$!

# wait for logger to finish (drone reached goal) or hard timeout
LOGGER_DONE=0
for i in $(seq 1 "$RUN_DURATION"); do
  if ! kill -0 "$LOGGER_PID" 2>/dev/null; then
    LOGGER_DONE=1
    echo "[run] logger finished after ~${i}s"
    break
  fi
  sleep 1
done

# teardown
if [ "$LOGGER_DONE" -eq 0 ]; then
  echo "[run] timeout reached, stopping logger"
fi
kill -INT "$LOGGER_PID" 2>/dev/null || true
sleep 1
kill -INT "$LAUNCH_PID" 2>/dev/null || true
sleep 2
pkill -f 'risk_data_collect' 2>/dev/null || true
pkill -f 'single_run_in_sim' 2>/dev/null || true
pkill -f 'data_logger.py' 2>/dev/null || true
sleep 1

echo "[run] logger_done=$LOGGER_DONE"
echo "[run] ----- logger log (tail) -----"
tail -n 20 /tmp/risk_logger.log 2>/dev/null || true
echo "[run] ----- launch log (tail) -----"
tail -n 20 /tmp/risk_launch.log 2>/dev/null || true
echo "[run] ----- output files -----"
ls -la "$OUT_DIR"/risk_run_* 2>/dev/null || echo "[run] no risk_run_* files in $OUT_DIR"

#!/usr/bin/env bash
# Run inside Ubuntu-22.04. Resume keeps validated attempts, including failures.
set -eo pipefail
TASK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$TASK_ROOT"
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
source "$TASK_ROOT/experiments/5.3加统计数据/install/local_setup.bash"
export PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
set -u
exec /usr/bin/python3 "$TASK_ROOT/scripts/section53_batch10/batch.py" --resume

#!/usr/bin/env bash
# Composite-environment multi-trial experiment for paper §5.3 (reviewer rebuttal).
# Runs 3 methods (adaptive / baseline / speed_only) x N seeds, one full flight each,
# then post-processes every trial with analyze_csv.py.
#
# Usage: bash run_composite_multi.sh <out_root> <gap> <"seed1 seed2 ...">
set -u

OUT_ROOT="${1:-/mnt/f/无人机暑假开发/看实验数据/data/复合环境/multi_trials}"
GAP="${2:-1.4}"
SEEDS="${3:-1 2 3 4 5 6 7 8 9 10}"

DL=/home/ubuntu/ros2_ws/src/ego-planner-swarm/src/my_navigation/data_logger
METHODS="adaptive baseline speed_only"

mkdir -p "$OUT_ROOT"
{
  echo "OUT_ROOT=$OUT_ROOT"
  echo "GAP=$GAP"
  echo "SEEDS=$SEEDS"
  echo "METHODS=$METHODS"
  echo "start=$(date '+%F %T')"
} > "$OUT_ROOT/_batch_config.txt"

for MODE in $METHODS; do
  for SEED in $SEEDS; do
    echo "======================================================"
    echo "=== MODE=$MODE SEED=$SEED  $(date '+%F %T') ==="
    echo "======================================================"
    bash "$DL/run_experiment.sh" "$MODE" "$OUT_ROOT" "$GAP" "$SEED"
    echo "=== finished MODE=$MODE SEED=$SEED rc=$? ==="
  done
done

echo "ALL_DONE $(date '+%F %T')"

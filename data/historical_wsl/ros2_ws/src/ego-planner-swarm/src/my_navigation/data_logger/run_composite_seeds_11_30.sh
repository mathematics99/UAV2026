#!/usr/bin/env bash
# Resume composite multi-trial: seeds 11-30 for all 3 methods.
# Appends to existing seed01-10; skips any seed that already has a run_summary.csv
# so the batch can be safely re-run after an interruption.
set -u

DL=/home/ubuntu/ros2_ws/src/ego-planner-swarm/src/my_navigation/data_logger
OUT_ROOT=/mnt/f/无人机暑假开发/看实验数据/data/复合环境/multi_trials
GAP=1.4
SEEDS="11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30"
METHODS="adaptive baseline speed_only"
LOG=/tmp/composite_11_30.log

: > "$LOG"
for MODE in $METHODS; do
  for SEED in $SEEDS; do
    D="$OUT_ROOT/$MODE/seed$(printf '%02d' "$SEED")"
    if [ -s "$D/run_summary.csv" ]; then
      echo "SKIP $MODE seed$(printf '%02d' "$SEED") (already done)" | tee -a "$LOG"
      continue
    fi
    echo "=== RUN $MODE seed$(printf '%02d' "$SEED") $(date '+%F %T') ===" | tee -a "$LOG"
    bash "$DL/run_experiment.sh" "$MODE" "$OUT_ROOT" "$GAP" "$SEED" >> "$LOG" 2>&1
    echo "=== DONE $MODE seed$(printf '%02d' "$SEED") rc=$? ===" | tee -a "$LOG"
  done
done
echo "ALL_DONE $(date '+%F %T')" | tee -a "$LOG"

#!/bin/bash
# 全量实验批次 (窄通道论文实验 1/2/3/4), 顺序执行。
# 每个 run_exp.sh 约 125s, 全量 ~17 个约 35 分钟。
# 用法: ./run_all_experiments.sh  (建议后台: nohup ./run_all_experiments.sh > run_all.log 2>&1 &)
set -u
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd /home/ubuntu

run() {
  echo "==================== $(date +%T) RUN $1 ===================="
  bash /home/ubuntu/run_exp.sh "$@"
  # 运行间隔, 确保残留进程被清干净
  sleep 3
}

# ---- 实验1: 收缩提前感知 (漏斗, 完整 V4.1) ----
run exp1_funnel funnel 1.4 1 2.0 1 120

# ---- 实验2: 入口速度扫参 (原始 EGO + 被动 risk, max_vel 扫) ----
for v in 0.5 1.0 1.5 2.0 2.5; do
  run exp2_v${v} narrow 1.4 1 ${v} 0 120
done

# ---- 实验3: 通道宽度扫参 (完整 V4.1, gate ON) ----
for g in 1.8 1.6 1.4 1.2 1.0; do
  run exp3_g${g} narrow ${g} 1 2.0 1 120
done

# ---- 实验3 消融: 去 gate (最窄两个宽度) ----
run exp3_g14_nogate narrow 1.4 0 2.0 1 120
run exp3_g10_nogate narrow 1.0 0 2.0 1 120

# ---- 实验4: 原始 EGO vs 本文方法 (gap 1.4 / 1.0) ----
run exp4_orig_g14 narrow 1.4 1 2.0 0 120
run exp4_prop_g14 narrow 1.4 1 2.0 1 120
run exp4_orig_g10 narrow 1.0 1 2.0 0 120
run exp4_prop_g10 narrow 1.0 1 2.0 1 120

echo "==================== ALL EXPERIMENTS DONE $(date +%T) ===================="

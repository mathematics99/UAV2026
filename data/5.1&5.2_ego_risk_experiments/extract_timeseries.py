#!/usr/bin/env python3
"""
时间序列提取: 把 bag 里的 odom + risk + adaptive 对齐成一张 CSV。

用法: python3 extract_timeseries.py <bag_dir> <label> [out_dir]

输出: <out_dir>/<label>_timeseries.csv
列: t_s, x, y, z, v, R_g, R_b, d_min, R_c, C_global, G_c,
    d_safe, v_max, lambda_obs, gate, r_bar, rc_bar

对齐: 以 odom 时间戳为主轴, 其余 topic 取最近邻值。
"""
import sys
import math
import bisect
import os

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

FLOAT_TOPICS = {
    "/risk/risk_score": "R_g",
    "/risk/braking_risk": "R_b",
    "/risk/min_distance": "d_min",
    "/risk/contraction_risk": "R_c",
    "/risk/contraction_global": "C_global",
    "/risk/contraction_gradient": "G_c",
    "/risk/current_velocity": "v_risk",
    "/adaptive/safe_distance": "d_safe",
    "/adaptive/max_velocity": "v_max",
    "/adaptive/collision_weight": "lambda_obs",
    "/adaptive/contraction_gate": "gate",
    "/adaptive/smoothed_risk": "r_bar",
    "/adaptive/smoothed_contraction": "rc_bar",
}


def nearest(series, ts_list, t):
    if not series:
        return float("nan")
    i = bisect.bisect_left(ts_list, t)
    if i == 0:
        return series[0][1]
    if i >= len(series):
        return series[-1][1]
    a, b = series[i - 1], series[i]
    return a[1] if (t - a[0]) <= (b[0] - t) else b[1]


def main():
    if len(sys.argv) < 3:
        print("用法: python3 extract_timeseries.py <bag_dir> <label> [out_dir]")
        sys.exit(1)
    bag = sys.argv[1]
    label = sys.argv[2]
    out_dir = sys.argv[3] if len(sys.argv) > 3 else "/home/ubuntu"

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag, storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr",
                         output_serialization_format="cdr"),
    )
    topic_types = {m.name: m.type for m in reader.get_all_topics_and_types()}

    series = {t: [] for t in FLOAT_TOPICS}
    odom = []  # (t, x, y, z, v)

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic in series and topic_types.get(topic) == "std_msgs/msg/Float64":
            series[topic].append((t, deserialize_message(data, Float64).data))
        elif topic == "/drone_0_visual_slam/odom":
            m = deserialize_message(data, Odometry)
            v = math.hypot(m.twist.twist.linear.x,
                           m.twist.twist.linear.y,
                           m.twist.twist.linear.z)
            odom.append((t, m.pose.pose.position.x, m.pose.pose.position.y,
                         m.pose.pose.position.z, v))

    if not odom:
        print(f"!! {label}: 无 odom 数据")
        return

    t0 = odom[0][0]
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"{label}_timeseries.csv")

    header = ["t_s", "x", "y", "z", "v"] + list(FLOAT_TOPICS.values())
    # 预计算每个 topic 的时间戳列表 (避免 nearest 里每次重建 → O(n^2) → O(n log n))
    ts_lists = {t: [x[0] for x in series[t]] for t in FLOAT_TOPICS}
    with open(out_path, "w") as f:
        f.write(",".join(header) + "\n")
        for t, x, y, z, v in odom:
            ts = (t - t0) / 1e9
            row = [f"{ts:.3f}", f"{x:.4f}", f"{y:.4f}", f"{z:.4f}", f"{v:.4f}"]
            for topic in FLOAT_TOPICS:
                row.append(f"{nearest(series[topic], ts_lists[topic], t):.4f}")
            f.write(",".join(row) + "\n")

    print(f"{label}: {len(odom)} odom 样本 -> {out_path}")


if __name__ == "__main__":
    main()

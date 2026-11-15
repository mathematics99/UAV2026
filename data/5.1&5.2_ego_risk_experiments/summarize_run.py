#!/usr/bin/env python3
"""
单次运行汇总指标 (实验3/4用): 读 bag + sim log, 输出完整论文指标。

用法: python3 summarize_run.py <bag_dir> <label> [sim_log] [results_csv]

输出: 打印汇总; 若给 results_csv, 追加一行 CSV (用于跨 run 聚合)。
指标: success / 穿越时间 / 路径 / clearance / 速度 / replan / planning time / 风险分量峰值
"""
import sys
import re
import math
import statistics
import os

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

import numpy as np


def cloud_to_xyz(pc):
    n = pc.width * pc.height
    fields = {f.name: f for f in pc.fields}
    arr = np.frombuffer(pc.data, dtype=np.uint8).reshape(n, pc.point_step)

    def col(name):
        o = fields[name].offset
        return np.ascontiguousarray(arr[:, o:o + 4]).view(np.float32).reshape(n)

    return np.column_stack([col("x"), col("y"), col("z")]).astype(np.float64)


def replan_and_time(sim_log):
    """从 sim log 提取 replan 次数与 planning time。"""
    replan = None
    pt = []
    if sim_log and os.path.exists(sim_log):
        with open(sim_log, "r", errors="ignore") as f:
            text = f.read()
        replan = len(re.findall(r"\[drone 0 replan \d+\]", text))
        for m in re.findall(r"\[planning_time\]\s+([\d.]+)\s+ms", text):
            pt.append(float(m))
    pt_mean = statistics.mean(pt) if pt else None
    pt_max = max(pt) if pt else None
    return replan, pt_mean, pt_max


def main():
    if len(sys.argv) < 3:
        print("用法: python3 summarize_run.py <bag_dir> <label> [sim_log] [results_csv]")
        sys.exit(1)
    bag = sys.argv[1]
    label = sys.argv[2]
    sim_log = sys.argv[3] if len(sys.argv) > 3 else None
    results_csv = sys.argv[4] if len(sys.argv) > 4 else None

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag, storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr",
                         output_serialization_format="cdr"),
    )
    topic_types = {m.name: m.type for m in reader.get_all_topics_and_types()}

    floats = {t: [] for t in [
        "/risk/risk_score", "/risk/braking_risk", "/risk/contraction_risk",
        "/adaptive/safe_distance", "/adaptive/max_velocity",
    ]}
    odom = []  # (t, x, y, z, v)
    cloud = None

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic in floats and topic_types.get(topic) == "std_msgs/msg/Float64":
            floats[topic].append((t, deserialize_message(data, Float64).data))
        elif topic == "/drone_0_visual_slam/odom":
            m = deserialize_message(data, Odometry)
            v = math.hypot(m.twist.twist.linear.x,
                           m.twist.twist.linear.y,
                           m.twist.twist.linear.z)
            odom.append((t, m.pose.pose.position.x, m.pose.pose.position.y,
                         m.pose.pose.position.z, v))
        elif topic == "/map_generator/global_cloud" and cloud is None:
            cloud = cloud_to_xyz(deserialize_message(data, PointCloud2))

    if not odom:
        print(f"!! {label}: 无 odom")
        return

    t0 = odom[0][0]
    max_x = max(p[1] for p in odom)
    success = max_x >= 14.0

    # 单次穿越 -14 -> +14 (时间 + 路径)
    transit_time = path_len = None
    crossed_neg = False
    t_start = None
    sub = []
    for t, x, y, z, v in odom:
        if not crossed_neg and x <= -14.0:
            crossed_neg = True
            t_start = t
        if crossed_neg and x >= 14.0:
            if t_start is not None:
                transit_time = (t - t_start) / 1e9
                sub = [(p[0], p[1], p[2], p[3]) for p in odom if t_start <= p[0] <= t]
                plen = 0.0
                for i in range(1, len(sub)):
                    dx = sub[i][1] - sub[i - 1][1]
                    dy = sub[i][2] - sub[i - 1][2]
                    dz = sub[i][3] - sub[i - 1][3]
                    plen += math.sqrt(dx * dx + dy * dy + dz * dz)
                path_len = plen
            break

    # clearance (全局点云最近邻)
    cl_min = cl_med = None
    if cloud is not None and len(cloud):
        from scipy.spatial import cKDTree
        tree = cKDTree(cloud)
        pts = np.array([(p[1], p[2], p[3]) for p in odom[::10]], dtype=np.float64)
        d, _ = tree.query(pts, k=1)
        cl_min = float(np.min(d))
        cl_med = float(np.median(d))

    # 速度
    vv = [p[4] for p in odom]
    v_mean = statistics.mean(vv)
    v_max = max(vv)

    # replan + planning time
    replan, pt_mean, pt_max = replan_and_time(sim_log)

    def fmax(name):
        s = floats[name]
        return max(x[1] for x in s) if s else None

    rc_max = fmax("/risk/contraction_risk")
    ds_max = fmax("/adaptive/safe_distance")
    rb_max = fmax("/risk/braking_risk")

    # 死锁检测: 未 success 且末段速度 ~0 (卡死)
    deadlock = (not success) and (vv[-1] < 0.1) and (len(odom) > 50)

    print("=" * 72)
    print(f"{label}")
    print("=" * 72)
    print(f"success (max x>=14)   : {'YES' if success else 'NO'}  (max_x={max_x:.2f})")
    print(f"transit time -14->+14 : {transit_time if transit_time is None else f'{transit_time:.1f} s'}")
    print(f"path length (单次)    : {path_len if path_len is None else f'{path_len:.2f} m'}")
    print(f"clearance min/med     : {cl_min if cl_min is None else f'{cl_min:.3f}'} / "
          f"{cl_med if cl_med is None else f'{cl_med:.3f}'} m")
    print(f"mean/max velocity     : {v_mean:.2f} / {v_max:.2f} m/s")
    print(f"replan count          : {replan if replan is None else replan}")
    print(f"planning time mean/max: "
          f"{pt_mean if pt_mean is None else f'{pt_mean:.1f}'} / "
          f"{pt_max if pt_max is None else f'{pt_max:.1f}'} ms")
    print(f"deadlock (卡死)       : {'YES' if deadlock else 'NO'}")
    print(f"R_c max / R_b max     : "
          f"{rc_max if rc_max is None else f'{rc_max:.3f}'} / "
          f"{rb_max if rb_max is None else f'{rb_max:.3f}'}")
    print(f"d_safe max            : {ds_max if ds_max is None else f'{ds_max:.3f}'}")

    if results_csv:
        header = ("label,success,max_x,transit_time,path_len,clearance_min,"
                  "clearance_med,mean_vel,max_vel,replan,plan_mean_ms,plan_max_ms,"
                  "deadlock,R_c_max,R_b_max,d_safe_max")
        new_file = not os.path.exists(results_csv)
        with open(results_csv, "a") as f:
            if new_file:
                f.write(header + "\n")
            f.write(",".join([
                label,
                "1" if success else "0",
                f"{max_x:.2f}",
                "" if transit_time is None else f"{transit_time:.2f}",
                "" if path_len is None else f"{path_len:.2f}",
                "" if cl_min is None else f"{cl_min:.3f}",
                "" if cl_med is None else f"{cl_med:.3f}",
                f"{v_mean:.2f}",
                f"{v_max:.2f}",
                "" if replan is None else str(replan),
                "" if pt_mean is None else f"{pt_mean:.1f}",
                "" if pt_max is None else f"{pt_max:.1f}",
                "1" if deadlock else "0",
                "" if rc_max is None else f"{rc_max:.3f}",
                "" if rb_max is None else f"{rb_max:.3f}",
                "" if ds_max is None else f"{ds_max:.3f}",
            ]) + "\n")
        print(f"(已追加 {results_csv})")


if __name__ == "__main__":
    main()

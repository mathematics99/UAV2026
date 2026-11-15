#!/usr/bin/env python3
"""
窄通道地图发布器 (窄通道压测 + 消融实验 C1/C2 用)

在 /map_generator/global_cloud 发布一个"窄通道"点云:
  两堵平行于 x 轴的墙 (y = ±gap/2), 形成一条沿 x 轴的窄通道。
  无人机沿 y=0、z=1 飞行, 必须穿过该通道。

几何参数:
  通道半宽  gap_half = 0.7  (通道总宽 1.4 m, 墙在 y = ±0.7)
  通道长度  x ∈ [-2, 2]
  墙高      z ∈ [0, 2.5]
  墙横向延伸 y ∈ [gap_half, 12] 与 y ∈ [-12, -gap_half]

发布 QoS: transient_local (latch), 使 pcl_render_node 后启动也能收到。
"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

X_MIN, X_MAX = -2.0, 2.0     # 通道沿 x 长度
# 通道总宽 gap (实验3宽度扫参用), 半宽 = gap/2, 墙在 y = ±GAP_HALF
GAP_TOTAL = float(sys.argv[1]) if len(sys.argv) > 1 else 1.4
GAP_HALF = GAP_TOTAL / 2.0
Y_EXTENT = 12.0              # 墙横向延伸范围
Z_MAX = 2.5                  # 墙高
RES = 0.1                    # 点云分辨率 (与 grid_map 分辨率一致)

def build_wall_points():
    xs = np.arange(X_MIN, X_MAX + RES, RES)
    # 负 y 侧墙: y ∈ [-Y_EXTENT, -GAP_HALF]
    y_neg = np.arange(-Y_EXTENT, -GAP_HALF + RES, RES)
    # 正 y 侧墙: y ∈ [GAP_HALF, Y_EXTENT]
    y_pos = np.arange(GAP_HALF, Y_EXTENT + RES, RES)
    zs = np.arange(0.0, Z_MAX + RES, RES)

    pts = []
    for y in y_neg:
        for x in xs:
            for z in zs:
                pts.append((x, y, z))
    for y in y_pos:
        for x in xs:
            for z in zs:
                pts.append((x, y, z))

    arr = np.array(pts, dtype=np.float32)
    print(f"[narrow_channel_map] wall points: {arr.shape[0]} "
          f"(gap={2*GAP_HALF:.1f}m, x=[{X_MIN},{X_MAX}], z=[0,{Z_MAX}])")
    return arr


def to_pointcloud2(arr, stamp):
    pc = PointCloud2()
    pc.header.frame_id = "world"
    pc.header.stamp = stamp
    pc.height = 1
    pc.width = arr.shape[0]
    pc.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    pc.is_bigendian = False
    pc.point_step = 12
    pc.row_step = pc.point_step * arr.shape[0]
    pc.is_dense = True
    pc.data = arr.tobytes()
    return pc


class NarrowChannelMap(Node):
    def __init__(self):
        super().__init__("narrow_channel_map")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # latch
        )
        self.pub = self.create_publisher(PointCloud2, "/map_generator/global_cloud", qos)
        self.arr = build_wall_points()
        # 定时重发 (1Hz), 保证任何时刻订阅都能收到
        self.timer = self.create_timer(1.0, self.publish_cloud)
        self.get_logger().info(
            f"narrow channel map ready: gap={2*GAP_HALF:.1f}m corridor x=[{X_MIN},{X_MAX}]"
        )

    def publish_cloud(self):
        self.pub.publish(to_pointcloud2(self.arr, self.get_clock().now().to_msg()))


def main():
    rclpy.init()
    node = NarrowChannelMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

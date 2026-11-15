#!/usr/bin/env python3
"""
漏斗形收缩通道地图发布器 (实验1: 前向收缩风险 R_c 提前感知)

在 /map_generator/global_cloud 发布一个"宽→窄→宽"的漏斗形通道:
  墙在 y = ±W(x), 其中 W(x) 沿 x 从两端"口"(mouth) 线性收窄到 x=0 处的"喉"(throat)。

几何:
  W(x) = throat_half + (mouth_half - throat_half) * |x| / (L/2)
  通道总宽 gap(x) = 2 * W(x)
  无人机沿 y=0、z=1 飞行, 从宽口进入、穿过窄喉再出宽口。

默认参数:
  mouth_gap  = 3.0  (口宽, x=±3 处)
  throat_gap = 1.2  (喉宽, x=0 处)
  half_len   = 3.0  (漏斗半长, x∈[-3,3])
  wall y 延伸 12.0, 墙高 z∈[0,2.5], 点云分辨率 0.1

用法:
  python3 funnel_channel_map.py [mouth_gap] [throat_gap] [half_len]

QoS: transient_local (latch), 1Hz 重发。
"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

Y_EXTENT = 12.0
Z_MAX = 2.5
RES = 0.1


def W_at_x(x, mouth_half, throat_half, half_len):
    """半宽 W(x): |x| 越大越宽, x=0 最窄。"""
    t = min(abs(x) / half_len, 1.0)
    return throat_half + (mouth_half - throat_half) * t


def build_wall_points(mouth_half, throat_half, half_len):
    xs = np.arange(-half_len, half_len + RES, RES)
    zs = np.arange(0.0, Z_MAX + RES, RES)

    pts = []
    for x in xs:
        w = W_at_x(x, mouth_half, throat_half, half_len)
        # 上墙: y ∈ [w, Y_EXTENT]; 下墙: y ∈ [-Y_EXTENT, -w]
        y_up = np.arange(w, Y_EXTENT + RES, RES)
        y_dn = np.arange(-Y_EXTENT, -w + RES, RES)
        for y in y_up:
            for z in zs:
                pts.append((x, y, z))
        for y in y_dn:
            for z in zs:
                pts.append((x, y, z))

    arr = np.array(pts, dtype=np.float32)
    print(f"[funnel_channel_map] wall points: {arr.shape[0]} "
          f"(mouth_gap={2*mouth_half:.1f}m, throat_gap={2*throat_half:.1f}m, "
          f"x=[-{half_len},{half_len}])")
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


class FunnelChannelMap(Node):
    def __init__(self, mouth_half, throat_half, half_len):
        super().__init__("funnel_channel_map")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(PointCloud2, "/map_generator/global_cloud", qos)
        self.arr = build_wall_points(mouth_half, throat_half, half_len)
        self.timer = self.create_timer(1.0, self.publish_cloud)
        self.get_logger().info(
            f"funnel channel ready: mouth={2*mouth_half:.1f}m throat={2*throat_half:.1f}m"
        )

    def publish_cloud(self):
        self.pub.publish(to_pointcloud2(self.arr, self.get_clock().now().to_msg()))


def main():
    rclpy.init()
    mouth_gap = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    throat_gap = float(sys.argv[2]) if len(sys.argv) > 2 else 1.2
    half_len = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
    node = FunnelChannelMap(mouth_gap / 2.0, throat_gap / 2.0, half_len)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

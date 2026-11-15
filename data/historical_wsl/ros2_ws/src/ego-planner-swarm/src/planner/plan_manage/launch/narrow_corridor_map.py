#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""混合实验地图发布器: 森林(官方同款混合障碍) -> 漏斗收缩窄通道 -> 森林

往 /map_generator/global_cloud 发布一张完整地图, 复现 EGO-SWARM 官方
random_forest 的那种"杂七杂八"混合森林, 中间插一段人工漏斗墙窄通道:
 - 森林障碍 = 竖直圆柱(random pillars) + 随机旋转椭球/倾斜圆盘(circle obstacles)
   两种按官方比例混合, 参数对齐官方 map_generator 包的 random_forest。
 - 只留起点 (-15,0) / 终点 (15,0) 各 2m 安全圆, 其余区域密密麻麻,
   规划器必须自己在密林中找路, 再穿窄通道, 最后出通道穿森林到终点。

布局 (沿 +X, 无人机从 x=-15 起飞):
    [  森林1  ] [   漏斗通道   ] [  森林2  ]
      密 →  穿  →  收 →  张  →  穿  →  密
"""
import argparse
import math
import random
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


# ---------------------------------------------------------------------------
# 障碍 1: 竖直实心圆柱 (官方 map/obs_num 那部分)
# ---------------------------------------------------------------------------
def _fill_cylinder(pts, cx, cy, r, h, res, base_z=0.0):
    ri = int(r / res) + 1
    for i in range(-ri, ri + 1):
        x = cx + i * res
        for j in range(-ri, ri + 1):
            y = cy + j * res
            if (i * res) ** 2 + (j * res) ** 2 > r * r:
                continue
            z = base_z
            while z <= base_z + h + 1e-9:
                pts.append((x, y, z))
                z += res


# ---------------------------------------------------------------------------
# 障碍 2: 随机旋转椭球 / 倾斜圆盘 (官方 map/circle_num 那部分, 3D 体素化)
# 官方参数对应: radius_l ~ [ObstacleShape/radius_l=0.7], radius_h ~ [0.5],
#               z_l ~ [0.7], z_h ~ [0.8], theta ~ [0.5]
# 我们这里做 3D 椭球后绕 Z 轴旋转 theta, 再平移到 (cx,cy,cz)
# ---------------------------------------------------------------------------
def _fill_ellipsoid(pts, cx, cy, cz, rx, ry, rz, theta, res):
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    ni = int(max(rx, ry) / res) + 2
    nj = int(max(rx, ry) / res) + 2
    nk = int(rz / res) + 2
    for i in range(-ni, ni + 1):
        lx = i * res
        for j in range(-nj, nj + 1):
            ly = j * res
            # 反旋转回椭球局部坐标
            ex = lx * cos_t + ly * sin_t
            ey = -lx * sin_t + ly * cos_t
            if rx <= 0 or ry <= 0:
                continue
            d2 = (ex / rx) ** 2 + (ey / ry) ** 2
            if d2 > 1.0 + 1e-6:
                continue
            z_r = rz * math.sqrt(max(0.0, 1.0 - d2))
            k = -int(z_r / res)
            while k <= int(z_r / res):
                lz = k * res
                if (ex / rx) ** 2 + (ey / ry) ** 2 + (lz / rz) ** 2 <= 1.0 + 1e-6:
                    pts.append((cx + lx, cy + ly, cz + lz))
                k += 1


# ---------------------------------------------------------------------------
# 漏斗墙: 余弦腰形收缩/张开通道, 实心有厚度, 两端 wide, 中心 gap
# ---------------------------------------------------------------------------
def _build_funnel(gap, wide, funnel_x, funnel_len, z_max, res, wall_thick=0.5):
    pts = []
    gap_half = gap / 2.0
    wide_half = wide / 2.0
    x_left = funnel_x - funnel_len
    span = 2.0 * funnel_len
    x = x_left
    while x <= funnel_x + funnel_len + 1e-9:
        t = (x - x_left) / span
        half = gap_half + (wide_half - gap_half) * math.cos(math.pi * t) ** 2
        y = half
        while y <= half + wall_thick + 1e-9:
            z = 0.0
            while z <= z_max + 1e-9:
                pts.append((x, y, z))
                pts.append((x, -y, z))
                z += res
            y += res
        x += res
    return pts


# ---------------------------------------------------------------------------
# 组合森林 = 竖直柱 + 随机椭球/倾斜圆盘, 参数对齐官方 map_generator
# 只排除指定禁区(起终点安全圆 / 漏斗墙体范围), 其余区域随机填满
# ---------------------------------------------------------------------------
def _build_hybrid_forest(rng,
                         x_min, x_max, y_half, z_max, res,
                         obs_num, circle_num,
                         pillar_rad_l, pillar_rad_h, pillar_h_l, pillar_h_h,
                         circ_rad_l, circ_rad_h, circ_z_l, circ_z_h, circ_theta_max,
                         exclude_regions=None):
    exclude_regions = exclude_regions or []
    pts = []
    placed_pillars = []
    placed_circles = []
    attempt = 0

    def _inside_exclude(cx, cy, r):
        for reg in exclude_regions:
            if reg[0] == 'rect':
                _, rx1, rx2, ry1, ry2 = reg
                if (cx + r > rx1 and cx - r < rx2 and
                        cy + r > ry1 and cy - r < ry2):
                    return True
            else:
                ex, ey, er = reg
                if (cx - ex) ** 2 + (cy - ey) ** 2 < (r + er) ** 2:
                    return True
        return False

    # --- 1) 竖直柱 (obs_num) ---
    while len(placed_pillars) < obs_num and attempt < obs_num * 50:
        attempt += 1
        cx = rng.uniform(x_min, x_max)
        cy = rng.uniform(-y_half, y_half)
        r = rng.uniform(pillar_rad_l, pillar_rad_h)
        h = rng.uniform(pillar_h_l, pillar_h_h)
        if h > z_max:
            h = z_max
        if _inside_exclude(cx, cy, r):
            continue
        if any((cx - px) ** 2 + (cy - py) ** 2 < (r + pr + 0.15) ** 2
               for px, py, pr in placed_pillars):
            continue
        # 距椭球中心也别太近, 避免两种障碍几乎重叠
        if any((cx - ox) ** 2 + (cy - oy) ** 2 < (r + max(orx, ory) + 0.1) ** 2
               for ox, oy, orx, ory in placed_circles):
            continue
        placed_pillars.append((cx, cy, r))
        _fill_cylinder(pts, cx, cy, r, h, res)

    # --- 2) 随机旋转椭球 / 倾斜圆盘 (circle_num) ---
    attempt2 = 0
    while len(placed_circles) < circle_num and attempt2 < circle_num * 50:
        attempt2 += 1
        cx = rng.uniform(x_min, x_max)
        cy = rng.uniform(-y_half, y_half)
        rx = rng.uniform(circ_rad_h, circ_rad_l)  # 官方写反 radius_l/radius_h
        ry = rng.uniform(circ_rad_h, circ_rad_l)
        rz = rng.uniform(circ_z_l, circ_z_h)
        theta = rng.uniform(-circ_theta_max, circ_theta_max)
        # 椭球中心 Z 放在 rz ~ z_max - rz 之间, 保证不会整体埋在地面下, 也不会超过顶
        if z_max - rz < rz:
            cz = rz if z_max >= 2 * rz else z_max - rz
        else:
            cz = rng.uniform(rz, z_max - rz)
        r_footprint = max(rx, ry)
        if _inside_exclude(cx, cy, r_footprint):
            continue
        if any((cx - px) ** 2 + (cy - py) ** 2 < (r_footprint + pr + 0.15) ** 2
               for px, py, pr in placed_pillars):
            continue
        if any((cx - ox) ** 2 + (cy - oy) ** 2 < (r_footprint + max(orx, ory) + 0.1) ** 2
               for ox, oy, orx, ory in placed_circles):
            continue
        placed_circles.append((cx, cy, rx, ry))
        _fill_ellipsoid(pts, cx, cy, cz, rx, ry, rz, theta, res)

    return pts, len(placed_pillars), len(placed_circles)


# ---------------------------------------------------------------------------
# 节点
# ---------------------------------------------------------------------------
class HybridMapPublisher(Node):
    def __init__(self, args):
        super().__init__('narrow_corridor_publisher')
        self.pub = self.create_publisher(
            PointCloud2, '/map_generator/global_cloud', 1)

        rng = random.Random(args.seed)
        funnel_left = args.funnel_x - args.funnel_len
        funnel_right = args.funnel_x + args.funnel_len

        # 禁区 (只留起/终点安全圆, 其他地方全部"杂七杂八"满满障碍)
        start_safe = (args.init_x, 0.0, 2.0)
        end_safe = (args.goal_x, 0.0, 2.0)
        # 漏斗墙体自身矩形范围内, 不再额外生成森林障碍 (避免墙里插柱子)
        funnel_block = ('rect',
                        funnel_left - 0.2, funnel_right + 0.2,
                        -(args.wide / 2.0 + args.wall_thick + 0.3),
                         args.wide / 2.0 + args.wall_thick + 0.3)

        exclude_forest = [start_safe, end_safe, funnel_block]

        # --- 左右两片森林: 竖直柱 + 随机椭球 ---
        n1, np1, nc1 = _build_hybrid_forest(
            rng, args.x_min, funnel_left, args.y_half, args.z_max, args.res,
            obs_num=args.forest_obs, circle_num=args.forest_obs,
            pillar_rad_l=args.pillar_rad_l, pillar_rad_h=args.pillar_rad_h,
            pillar_h_l=args.pillar_h_l, pillar_h_h=args.pillar_h_h,
            circ_rad_l=args.circ_rad_l, circ_rad_h=args.circ_rad_h,
            circ_z_l=args.circ_z_l, circ_z_h=args.circ_z_h,
            circ_theta_max=args.circ_theta,
            exclude_regions=exclude_forest)

        n2, np2, nc2 = _build_hybrid_forest(
            rng, funnel_right, args.x_max, args.y_half, args.z_max, args.res,
            obs_num=args.forest_obs, circle_num=args.forest_obs,
            pillar_rad_l=args.pillar_rad_l, pillar_rad_h=args.pillar_rad_h,
            pillar_h_l=args.pillar_h_l, pillar_h_h=args.pillar_h_h,
            circ_rad_l=args.circ_rad_l, circ_rad_h=args.circ_rad_h,
            circ_z_l=args.circ_z_l, circ_z_h=args.circ_z_h,
            circ_theta_max=args.circ_theta,
            exclude_regions=exclude_forest)

        # --- 中间漏斗墙 ---
        fun = _build_funnel(args.gap, args.wide, args.funnel_x,
                            args.funnel_len, args.z_max, args.res,
                            args.wall_thick)

        pts = n1 + fun + n2
        self.pts = pts
        n_pillars = np1 + np2
        n_circles = nc1 + nc2
        self.get_logger().info(
            '[narrow_corridor] Hybrid map (EGO-SWARM style): '
            'forest1(pillars=%d,circles=%d) + funnel=%d + '
            'forest2(pillars=%d,circles=%d) = %d total pts | '
            'funnel wide=%.1f -> throat=%.1f -> wide=%.1f, wall=%.2f | '
            'safe start(r=2m @ x=%.1f) / goal(r=2m @ x=%.1f).'
            % (np1, nc1, len(fun), np2, nc2, len(pts),
               args.wide, args.gap, args.wide, args.wall_thick,
               args.init_x, args.goal_x))
        self.timer = self.create_timer(1.0 / args.rate, self._publish)

    def _publish(self):
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.height = 1
        msg.width = len(self.pts)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        buf = bytearray()
        for x, y, z in self.pts:
            buf += struct.pack('<fff', x, y, z)
        msg.data = bytes(buf)
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(
        description='Publish EGO-SWARM style hybrid forest + funnel corridor map.')

    # 漏斗
    parser.add_argument('--gap', type=float, default=1.0)
    parser.add_argument('--wide', type=float, default=5.0)
    parser.add_argument('--funnel-x', type=float, default=0.0)
    parser.add_argument('--funnel-len', type=float, default=5.0)
    parser.add_argument('--wall-thick', type=float, default=0.5)

    # 总体地图边界 (对齐 ego_planner 的 50x25x2 origin-centered grid map)
    parser.add_argument('--x-min', type=float, default=-24.5)
    parser.add_argument('--x-max', type=float, default=24.5)
    parser.add_argument('--y-half', type=float, default=12.0)
    parser.add_argument('--z-max', type=float, default=1.9)
    parser.add_argument('--res', type=float, default=0.1)
    parser.add_argument('--rate', type=float, default=2.0)
    parser.add_argument('--seed', type=int, default=42)

    # 起终点 (只做 2m 安全圆, 不再清空整条中心线通道)
    parser.add_argument('--init-x', type=float, default=-15.0)
    parser.add_argument('--goal-x', type=float, default=15.0)

    # 森林密度: 每侧森林的 竖直柱个数 / 椭球圆盘个数
    # (两侧合计就是 2*forest_obs 柱 + 2*forest_obs 椭球, 与官方 obs_num=250/circle_num=250 量级一致)
    parser.add_argument('--forest-obs', type=int, default=120,
                        help='number of pillars AND circles per forest side')

    # --- 官方 map_generator random_forest 同款参数 ---
    # 竖直柱: ObstacleShape/lower_rad=0.5, upper_rad=0.7, lower_hei=0.0, upper_hei=3.0
    parser.add_argument('--pillar-rad-l', type=float, default=0.5)
    parser.add_argument('--pillar-rad-h', type=float, default=0.7)
    parser.add_argument('--pillar-h-l',   type=float, default=1.0)   # 不做完全埋地的, 更自然
    parser.add_argument('--pillar-h-h',   type=float, default=1.9)
    # 椭球圆盘: ObstacleShape/radius_l=0.7, radius_h=0.5, z_l=0.7, z_h=0.8, theta=0.5
    parser.add_argument('--circ-rad-l',   type=float, default=0.7)
    parser.add_argument('--circ-rad-h',   type=float, default=0.5)
    parser.add_argument('--circ-z-l',     type=float, default=0.7)
    parser.add_argument('--circ-z-h',     type=float, default=0.8)
    parser.add_argument('--circ-theta',   type=float, default=0.5)

    args = parser.parse_args()

    rclpy.init()
    node = HybridMapPublisher(args)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

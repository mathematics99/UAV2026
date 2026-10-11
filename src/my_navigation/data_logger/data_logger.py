#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Risk-adaptive planning experiment data logger.

Writes two CSVs:
  <prefix>_timeseries.csv    fixed-rate state quantities + scene_region + seed
  <prefix>_replan_events.csv one row per replan (event-driven, no polling)
"""
import argparse
import math
import os
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from traj_utils.msg import ReplanEvent

TS_HEADER = ("t_s,x,y,z,vx,vy,vz,v,"
             "d_min,rho,C_global,G_c,R_c,R_b,R_g,"
             "r_bar,rc_bar,v_max,d_safe,lambda_obs,gate,"
             "scene_region,seed")
REPLAN_HEADER = "timestamp_s,planning_time_ms,success,count"

FLOAT_KEYS = ['d_min', 'rho', 'C_global', 'G_c', 'R_c', 'R_b', 'R_g',
              'r_bar', 'rc_bar', 'v_max', 'd_safe', 'lambda_obs', 'gate']
FLOAT_TOPICS = {
    'd_min': '/risk/min_distance',
    'rho': '/risk/obstacle_density',
    'C_global': '/risk/contraction_global',
    'G_c': '/risk/contraction_gradient',
    'R_c': '/risk/contraction_risk',
    'R_b': '/risk/braking_risk',
    'R_g': '/risk/risk_score',
    'r_bar': '/adaptive/smoothed_risk',
    'rc_bar': '/adaptive/smoothed_contraction',
    'v_max': '/adaptive/max_velocity',
    'd_safe': '/adaptive/safe_distance',
    'lambda_obs': '/adaptive/collision_weight',
    'gate': '/adaptive/contraction_gate',
}


class DataLogger(Node):
    def __init__(self, args):
        super().__init__('data_logger')
        self.args = args

        os.makedirs(args.out_dir, exist_ok=True)
        prefix = time.strftime('risk_run_%Y%m%d_%H%M%S')
        self.ts_path = os.path.join(args.out_dir, prefix + '_timeseries.csv')
        self.replan_path = os.path.join(args.out_dir, prefix + '_replan_events.csv')

        self.ts_f = open(self.ts_path, 'w')
        self.ts_f.write(TS_HEADER + '\n')
        self.replan_f = open(self.replan_path, 'w')
        self.replan_f.write(REPLAN_HEADER + '\n')

        self.vals = {k: None for k in FLOAT_KEYS}
        self.have_odom = False
        self.t0 = None
        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.t_s = 0.0
        self._prev_pos = None
        self._prev_t = None
        self._settle_start = None
        self.done = False

        self.create_subscription(Odometry, args.odom, self.odom_cb, 10)
        for key, topic in FLOAT_TOPICS.items():
            self.create_subscription(Float64, topic, self._make_float_cb(key), 10)
        self.create_subscription(ReplanEvent, '/planner/replan_event', self.replan_cb, 10)

        self.timer = self.create_timer(1.0 / args.rate, self.timer_cb)

        self.get_logger().info('timeseries    -> %s' % self.ts_path)
        self.get_logger().info('replan_events -> %s' % self.replan_path)

    def _make_float_cb(self, key):
        def cb(msg):
            self.vals[key] = msg.data
        return cb

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z

        now = self._now()
        if not self.have_odom:
            self.t0 = now
            self.have_odom = True
        self.t_s = now - self.t0

        # finite-difference fallback if odom twist is absent (all zeros)
        if (vx * vx + vy * vy + vz * vz) < 1e-12 and self._prev_pos is not None and self._prev_t is not None:
            dt = self.t_s - self._prev_t
            if dt > 1e-6:
                vx = (self.x - self._prev_pos[0]) / dt
                vy = (self.y - self._prev_pos[1]) / dt
                vz = (self.z - self._prev_pos[2]) / dt
        self.vx, self.vy, self.vz = vx, vy, vz
        self._prev_pos = (self.x, self.y, self.z)
        self._prev_t = self.t_s

    def replan_cb(self, msg):
        now = self._now()
        t_rel = now - (self.t0 if self.t0 is not None else now)
        self.replan_f.write('%.6f,%.3f,%d,%d\n' % (
            t_rel, msg.planning_time_ms, int(msg.success), msg.count))
        self.replan_f.flush()

    def scene_region(self, x):
        cx = self.args.corridor_x
        hl = self.args.corridor_half_len
        th = self.args.throat_half_len
        if x < cx - hl:
            return 'forest_before'
        if x < cx - th:
            return 'contraction'
        if x <= cx + th:
            return 'throat'
        if x <= cx + hl:
            return 'expansion'
        return 'forest_after'

    def timer_cb(self):
        if not self.have_odom:
            return
        v = math.sqrt(self.vx * self.vx + self.vy * self.vy + self.vz * self.vz)
        region = self.scene_region(self.x)
        parts = ['%.6f' % self.t_s,
                 '%.4f' % self.x, '%.4f' % self.y, '%.4f' % self.z,
                 '%.4f' % self.vx, '%.4f' % self.vy, '%.4f' % self.vz,
                 '%.4f' % v]
        for key in FLOAT_KEYS:
            val = self.vals[key]
            parts.append('' if val is None else '%.6f' % val)
        parts.append(region)
        parts.append('%d' % self.args.seed)
        self.ts_f.write(','.join(parts) + '\n')
        self.ts_f.flush()

        # auto-stop once drone reaches stop_x and settles there
        if self.args.stop_x is not None:
            if self.x >= self.args.stop_x:
                if self._settle_start is None:
                    self._settle_start = time.time()
                elif time.time() - self._settle_start >= self.args.settle_dur:
                    self.get_logger().info('reached stop_x=%.1f, stopping logger' % self.args.stop_x)
                    self.done = True
            else:
                self._settle_start = None

    def close(self):
        for f in (self.ts_f, self.replan_f):
            try:
                f.flush()
                f.close()
            except Exception:
                pass


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out_dir', default='/mnt/c/Users/lenovo/Desktop')
    p.add_argument('--rate', type=float, default=50.0)
    p.add_argument('--seed', type=int, default=42)
    p.add_argument('--corridor_x', type=float, default=0.0)
    p.add_argument('--corridor_half_len', type=float, default=5.0)
    p.add_argument('--throat_half_len', type=float, default=1.0)
    p.add_argument('--odom', default='/uav/odom')
    p.add_argument('--stop_x', type=float, default=14.5)
    p.add_argument('--settle_dur', type=float, default=2.0)
    args = p.parse_args()

    rclpy.init()
    node = DataLogger(args)
    try:
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        while rclpy.ok() and not node.done:
            executor.spin_once(timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

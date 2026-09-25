#!/usr/bin/env python3
"""nav_scan_node — Mid360 /cloud_registered → /scan in base_footprint.

CRITICAL: must TF into base_link (not base_footprint) for the height slice.
  TF tree has two branches from odom:
    odom → camera_init → body → base_link   (includes +45° mount leveling)
    odom → base_footprint                   (yaw-only planar, NO pitch)

  Looking up base_footprint←camera_init skips body→base_link, so z-slice
  cuts a tilted band and wall ranges never match the pcd2pgm map (hit~10%).

  Correct flow (wheeltec-style):
    cloud camera_init → base_link (pitch leveled) → keep z∈[z_min,z_max]
    publish LaserScan frame_id=base_footprint (same yaw/xy as base_link).
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, LaserScan
from tf2_ros import Buffer, TransformListener
import sensor_msgs_py.point_cloud2 as pc2


def transform_to_mat(tf):
    q = tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )
    t = tf.transform.translation
    return R, (t.x, t.y, t.z)


def apply_Rt(R, t, px, py, pz):
    return (
        R[0][0] * px + R[0][1] * py + R[0][2] * pz + t[0],
        R[1][0] * px + R[1][1] * py + R[1][2] * pz + t[1],
        R[2][0] * px + R[2][1] * py + R[2][2] * pz + t[2],
    )


class NavScanNode(Node):
    def __init__(self):
        super().__init__('nav_scan_node')
        self.z_min = float(self.declare_parameter('z_min', 0.15).value)
        self.z_max = float(self.declare_parameter('z_max', 1.0).value)
        self.range_min = float(self.declare_parameter('range_min', 0.20).value)
        self.range_max = float(self.declare_parameter('range_max', 8.0).value)
        self.angle_increment = float(self.declare_parameter('angle_increment', 0.01).value)
        self._nbin = max(1, int(round((2.0 * math.pi) / self.angle_increment)))
        self._miss_tf = 0
        self._ok = 0

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/cloud_registered', self.cb_cloud, qos_profile_sensor_data)

        self.get_logger().info(
            'nav_scan: TF→base_link z=[%.2f,%.2f] → /scan(base_footprint)'
            % (self.z_min, self.z_max))

    def cb_cloud(self, msg: PointCloud2):
        # MUST use base_link (has mount pitch). base_footprint skips leveling.
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link', msg.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except Exception:
            self._miss_tf += 1
            if self._miss_tf % 30 == 1:
                self.get_logger().warn(
                    'waiting TF base_link←%s (miss=%d)'
                    % (msg.header.frame_id, self._miss_tf))
            return

        R, t = transform_to_mat(tf)
        bins = [self.range_max + 1.0] * self._nbin
        n_keep = 0
        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = apply_Rt(R, t, float(p[0]), float(p[1]), float(p[2]))
            if z < self.z_min or z > self.z_max:
                continue
            rng = math.hypot(x, y)
            if rng < self.range_min or rng > self.range_max:
                continue
            ang = math.atan2(y, x)
            idx = int((ang + math.pi) / self.angle_increment) % self._nbin
            if rng < bins[idx]:
                bins[idx] = rng
            n_keep += 1

        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        # Nav2 / overlay expect base_footprint; xy/yaw match base_link
        scan.header.frame_id = 'base_footprint'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = [
            float(r) if r <= self.range_max else float('inf') for r in bins]
        self.pub.publish(scan)
        self._ok += 1
        if self._ok % 50 == 1:
            valid = sum(1 for r in bins if r <= self.range_max)
            self.get_logger().info(
                'scan ok: band_pts~%d beams=%d/%d' % (n_keep, valid, self._nbin))


def main():
    rclpy.init()
    node = NavScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if 'ExternalShutdown' not in type(e).__name__:
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()

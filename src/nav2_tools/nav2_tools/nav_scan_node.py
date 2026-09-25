#!/usr/bin/env python3
"""nav_scan_node — Mid360 /cloud_registered → /scan in base_footprint.

Leveling: the lio_tf_bridge TF chain (camera_init → body → base_link, with a
static +45° "mount leveling" edge) was found to be WRONG — near-robot points
that must be floor were coming out at 0.7-2.5m in 'base_link' instead of
~0m. Root cause: FAST-LIO's 'camera_init' world frame is not gravity-vertical
here (it just pins to wherever 'body' pointed at boot, ~45° tilted with the
physical mount), and the static body→base_link correction did not actually
cancel that tilt (verified empirically on-robot, see chat history).

Fix: level the cloud ourselves using the real, IMU-measured gravity "up"
vector (same source as scripts/calib_lidar_imu_gravity.py and the web UI's
nominalUp()/levelMat()), applied on top of the *dynamic* camera_init→body TF
(so it still tracks the robot's real-time pose while walking), instead of
trusting the static body→base_link TF edge. Verified on-robot: floor lands
near z≈0 consistently from 0-6m range after this fix (was scattered
0.7-2.5m before, at every range).

Config: reads pitch_down/roll from config/lidar_extrinsic.yaml (written by
scripts/calib_lidar_imu_gravity.py --apply); falls back to the nominal 45°/0°
mount spec if the file is missing.
"""
from __future__ import annotations

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, LaserScan
from tf2_ros import Buffer, TransformListener
import sensor_msgs_py.point_cloud2 as pc2

EXTRINSIC_YAML = '/home/linaro/robot_ws/config/lidar_extrinsic.yaml'
NOMINAL_PITCH_DOWN = 0.7853981634  # 45 deg, mount spec fallback
NOMINAL_ROLL = 0.0


def load_pitch_roll(path=EXTRINSIC_YAML):
    pitch, roll = NOMINAL_PITCH_DOWN, NOMINAL_ROLL
    try:
        with open(path, 'r') as f:
            for line in f:
                line = line.split('#', 1)[0].strip()
                if not line or ':' not in line:
                    continue
                k, v = line.split(':', 1)
                k = k.strip()
                if k == 'pitch_down':
                    pitch = float(v.strip())
                elif k == 'roll':
                    roll = float(v.strip())
    except Exception:
        pass
    return pitch, roll


def nominal_up(pitch_down, roll):
    """Unit 'up' vector expressed in the tilted sensor (body) frame.
    Matches web_ui/index.html nominalUp() and calib_lidar_imu_gravity.py."""
    sp, cp = math.sin(pitch_down), math.cos(pitch_down)
    sr, cr = math.sin(roll), math.cos(roll)
    return (-sp, sr * cp, cr * cp)


def rot_align(a, b):
    """3x3 rotation matrix (row-major tuple of tuples) mapping unit vector a -> b."""
    an = math.sqrt(sum(c * c for c in a)) or 1.0
    a = tuple(c / an for c in a)
    bn = math.sqrt(sum(c * c for c in b)) or 1.0
    b = tuple(c / bn for c in b)
    vx, vy, vz = (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
    c = a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
    s2 = vx*vx + vy*vy + vz*vz
    if s2 < 1e-12:
        if c > 0:
            return ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))
        return ((-1., 0., 0.), (0., -1., 0.), (0., 0., -1.))
    k = (1.0 - c) / s2
    return (
        (1 + (-vz*vz - vy*vy) * k, -vz + vx*vy*k, vy + vx*vz*k),
        (vz + vx*vy*k, 1 + (-vz*vz - vx*vx) * k, -vx + vy*vz*k),
        (-vy + vx*vz*k, vx + vy*vz*k, 1 + (-vy*vy - vx*vx) * k),
    )


def mat_vec(R, v):
    return (
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
    )


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
        self.z_min = float(self.declare_parameter('z_min', 0.10).value)
        self.z_max = float(self.declare_parameter('z_max', 1.0).value)
        self.range_min = float(self.declare_parameter('range_min', 0.20).value)
        self.range_max = float(self.declare_parameter('range_max', 8.0).value)
        self.angle_increment = float(self.declare_parameter('angle_increment', 0.01).value)
        self.lidar_x = float(self.declare_parameter('lidar_x', 0.25).value)
        self.lidar_y = float(self.declare_parameter('lidar_y', 0.0).value)
        self.lidar_z = float(self.declare_parameter('lidar_z', 0.45).value)
        self._nbin = max(1, int(round((2.0 * math.pi) / self.angle_increment)))
        self._miss_tf = 0
        self._ok = 0

        pitch_down, roll = load_pitch_roll()
        up = nominal_up(pitch_down, roll)
        self.R_level = rot_align(up, (0.0, 0.0, 1.0))
        # body(livox) -> base_link translation, leveled (same physical offset
        # as lio_tf_bridge's body_to_base_link_static, recomputed with our
        # verified-correct rotation instead of its rotation).
        self.t_level = tuple(-c for c in mat_vec(self.R_level, (self.lidar_x, self.lidar_y, self.lidar_z)))

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/cloud_registered', self.cb_cloud, qos_profile_sensor_data)

        self.get_logger().info(
            'nav_scan: leveled(pitch_down=%.1f° roll=%.1f°) z=[%.2f,%.2f] → /scan(base_footprint)'
            % (math.degrees(pitch_down), math.degrees(roll), self.z_min, self.z_max))

    def cb_cloud(self, msg: PointCloud2):
        # Dynamic real-time sensor pose (tracks robot motion); leveling itself
        # is applied on top with our own verified-correct rotation, not the
        # (buggy) static body->base_link TF edge.
        try:
            tf = self.tf_buffer.lookup_transform(
                'body', msg.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except Exception:
            self._miss_tf += 1
            if self._miss_tf % 30 == 1:
                self.get_logger().warn(
                    'waiting TF body←%s (miss=%d)'
                    % (msg.header.frame_id, self._miss_tf))
            return

        R, t = transform_to_mat(tf)
        Rl, tl = self.R_level, self.t_level
        bins = [self.range_max + 1.0] * self._nbin
        n_keep = 0
        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            bx, by, bz = apply_Rt(R, t, float(p[0]), float(p[1]), float(p[2]))
            x, y, z = apply_Rt(Rl, tl, bx, by, bz)
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
        # Nav2 / overlay expect base_footprint; xy/yaw match our leveled frame
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

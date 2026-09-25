#!/usr/bin/env python3
"""lio_tf_bridge: FAST-LIO2 Odometry -> Nav2 odom frame.

Subscribes FAST-LIO2 /Odometry (frame_id=camera_init, child_frame_id=body),
combines with the lidar-to-base extrinsic (base_link -> livox_frame, same as
genisom_bridge), and republishes as /odom_nav (frame_id=odom,
child_frame_id=base_link) plus a dynamic TF odom -> base_link.
Also broadcasts a static map -> odom identity transform so Nav2 has a full
map -> odom -> base_link chain while using FAST-LIO2 as the SLAM source.

Coordinate convention: REP-103, base_link forward X / left Y / up Z.
Extrinsic roll/pitch/yaw are in radians, ZYX order (same as genisom_bridge).
"""
import math
import threading

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def quat_mult(q1, q2):
    """Hamilton product of quaternions (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


def rpy_to_quat(r, p, y):
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def quat_rotate(q, v):
    """Rotate vector v by quaternion q."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v); v' = v + w*t + cross(q.xyz, t)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


class LioTfBridge(Node):
    def __init__(self):
        super().__init__('lio_tf_bridge')
        self.declare_parameter('lidar_x', 0.0)
        self.declare_parameter('lidar_y', 0.0)
        self.declare_parameter('lidar_z', 0.0)
        self.declare_parameter('lidar_roll', 0.0)
        self.declare_parameter('lidar_pitch', 0.0)
        self.declare_parameter('lidar_yaw', 0.0)

        # Extrinsic: base_link -> livox_frame (= FAST-LIO2 body frame)
        self.q_bl_lf = rpy_to_quat(
            self.get_parameter('lidar_roll').value,
            self.get_parameter('lidar_pitch').value,
            self.get_parameter('lidar_yaw').value)
        self.t_bl_lf = (
            self.get_parameter('lidar_x').value,
            self.get_parameter('lidar_y').value,
            self.get_parameter('lidar_z').value)
        # Inverse: body(livox) -> base_link
        self.q_lf_bl = quat_conj(self.q_bl_lf)
        self.t_lf_bl = quat_rotate(self.q_lf_bl,
                                   (-self.t_bl_lf[0], -self.t_bl_lf[1], -self.t_bl_lf[2]))

        self.tf_bc = TransformBroadcaster(self)
        self.tf_static_bc = StaticTransformBroadcaster(self)
        # odom and camera_init are the same world frame (FAST-LIO2's origin).
        # Nav2 expects odom; FAST-LIO2 names it camera_init. Publish a static
        # identity odom -> camera_init so the whole tree is:
        #   map -> odom -> camera_init -> body -> base_link
        # This keeps base_link with a SINGLE parent (body), avoiding TF tree
        # splits between Nav2 (odom) and scan consumers (camera_init).
        self._publish_odom_camera_init()
        # NOTE: static map->odom identity is NO LONGER published here.
        # Nav2 with AMCL: amcl estimates map->odom from /scan + map, so a
        # static identity would conflict with it. Remove amcl by re-adding
        # self._publish_static_map_odom() if running without localization.
        # NOTE: body->base_link static TF is published by launch's
        # static_transform_publisher (see bridge.launch.py), not here.

        # 静态 TF 偶发丢包时会出现 "Invalid frame ID odom"；定时重发
        self.create_timer(2.0, self._publish_odom_camera_init)

        self.pub_odom = self.create_publisher(Odometry, '/odom_nav', 10)
        self.pub_odom2 = self.create_publisher(Odometry, '/odom', 10)
        self.sub = self.create_subscription(
            Odometry, '/Odometry', self.cb_odom, 10)
        self.get_logger().info(
            f'lio_tf_bridge ready: extrinsic T=({self.t_bl_lf[0]:.3f}, '
            f'{self.t_bl_lf[1]:.3f}, {self.t_bl_lf[2]:.3f}) '
            f'RPY=({self.get_parameter("lidar_roll").value:.3f}, '
            f'{self.get_parameter("lidar_pitch").value:.3f}, '
            f'{self.get_parameter("lidar_yaw").value:.3f})')

    def _publish_static_map_odom(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_static_bc.sendTransform(t)

    def _publish_odom_camera_init(self):
        # Static identity: odom == camera_init (FAST-LIO2 world frame).
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'camera_init'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_static_bc.sendTransform(t)

    def cb_odom(self, msg: Odometry):
        # FAST-LIO2 pose: camera_init -> body
        p = msg.pose.pose
        q_cb = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        t_cb = (p.position.x, p.position.y, p.position.z)

        # odom(base) -> base_link = camera_init->body * body->base_link
        q_ob = quat_mult(q_cb, self.q_lf_bl)
        t_ob = quat_rotate(q_cb, self.t_lf_bl)
        t_ob = (t_cb[0] + t_ob[0], t_cb[1] + t_ob[1], t_cb[2] + t_ob[2])

        # Publish /odom_nav
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_link'
        out.pose.pose.position.x = t_ob[0]
        out.pose.pose.position.y = t_ob[1]
        out.pose.pose.position.z = t_ob[2]
        out.pose.pose.orientation.x = q_ob[0]
        out.pose.pose.orientation.y = q_ob[1]
        out.pose.pose.orientation.z = q_ob[2]
        out.pose.pose.orientation.w = q_ob[3]
        out.twist = msg.twist
        self.pub_odom.publish(out)
        self.pub_odom2.publish(out)

        # Publish TF camera_init -> body (same as FAST-LIO2's own dynamic TF;
        # duplicated here for consistency in case FAST-LIO2's TF lags).
        # Full tree: map -> odom -> camera_init -> body -> base_link
        # (odom->camera_init static identity, body->base_link static from launch).
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'camera_init'
        t.child_frame_id = 'body'
        t.transform.translation.x = t_cb[0]
        t.transform.translation.y = t_cb[1]
        t.transform.translation.z = t_cb[2]
        t.transform.rotation.x = q_cb[0]
        t.transform.rotation.y = q_cb[1]
        t.transform.rotation.z = q_cb[2]
        t.transform.rotation.w = q_cb[3]
        self.tf_bc.sendTransform(t)

        # Planar footprint for Nav2 /scan (yaw only, z=0) — wheeltec convention.
        # Parent stays odom (parallel to camera_init→body→base_link).
        # nav_scan MUST TF into base_link for height slice (has mount pitch);
        # looking up base_footprint←camera_init skips leveling and breaks ICP.
        yaw = math.atan2(
            2.0 * (q_ob[3] * q_ob[2] + q_ob[0] * q_ob[1]),
            1.0 - 2.0 * (q_ob[1] * q_ob[1] + q_ob[2] * q_ob[2]))
        fp = TransformStamped()
        fp.header.stamp = msg.header.stamp
        fp.header.frame_id = 'odom'
        fp.child_frame_id = 'base_footprint'
        fp.transform.translation.x = t_ob[0]
        fp.transform.translation.y = t_ob[1]
        fp.transform.translation.z = 0.0
        fp.transform.rotation.z = math.sin(yaw * 0.5)
        fp.transform.rotation.w = math.cos(yaw * 0.5)
        self.tf_bc.sendTransform(fp)


def main():
    rclpy.init()
    node = LioTfBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

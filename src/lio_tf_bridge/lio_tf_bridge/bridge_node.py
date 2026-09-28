#!/usr/bin/env python3
"""lio_tf_bridge: Nav2 odom + FAST-LIO body TF.

默认 odom_source:=dog：
  - /odom_dog（机身 SDK 里程计）→ /odom_nav + /odom + TF odom→base_footprint
  - /Odometry（FAST-LIO）→ 仅 TF camera_init→body（给 nav_scan 出激光）
  - 不再把 odom 与 camera_init 绑成 identity（二者原点不同）

odom_source:=lio：旧行为，平面里程计也来自 FAST-LIO（易漂）。
"""
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def quat_mult(q1, q2):
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
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def yaw_from_quat(q):
    """Planar yaw from quaternion (x,y,z,w). Prefer projected +X when tilted."""
    fx, fy, _ = quat_rotate(q, (1.0, 0.0, 0.0))
    return math.atan2(fy, fx)


class LioTfBridge(Node):
    def __init__(self):
        super().__init__('lio_tf_bridge')
        self.declare_parameter('lidar_x', 0.0)
        self.declare_parameter('lidar_y', 0.0)
        self.declare_parameter('lidar_z', 0.0)
        self.declare_parameter('lidar_roll', 0.0)
        self.declare_parameter('lidar_pitch', 0.0)
        self.declare_parameter('lidar_yaw', 0.0)
        # dog = 机身里程计短时运动；lio = 旧纯激光里程计
        self.declare_parameter('odom_source', 'dog')
        self.odom_source = str(self.get_parameter('odom_source').value).strip().lower()

        self.q_bl_lf = rpy_to_quat(
            self.get_parameter('lidar_roll').value,
            self.get_parameter('lidar_pitch').value,
            self.get_parameter('lidar_yaw').value)
        self.t_bl_lf = (
            self.get_parameter('lidar_x').value,
            self.get_parameter('lidar_y').value,
            self.get_parameter('lidar_z').value)
        self.q_lf_bl = quat_conj(self.q_bl_lf)
        self.t_lf_bl = quat_rotate(self.q_lf_bl,
                                   (-self.t_bl_lf[0], -self.t_bl_lf[1], -self.t_bl_lf[2]))

        self.tf_bc = TransformBroadcaster(self)
        self.tf_static_bc = StaticTransformBroadcaster(self)

        # 仅 lio 模式才把 odom≡camera_init；dog 模式下二者独立，避免 LIO 炸坐标拖垮 Nav2
        if self.odom_source == 'lio':
            self._publish_odom_camera_init()
            self.create_timer(2.0, self._publish_odom_camera_init)

        self.pub_odom = self.create_publisher(Odometry, '/odom_nav', 10)
        self.pub_odom2 = self.create_publisher(Odometry, '/odom', 10)

        self.sub_lio = self.create_subscription(
            Odometry, '/Odometry', self.cb_lio, 10)
        if self.odom_source == 'dog':
            self.sub_dog = self.create_subscription(
                Odometry, '/odom_dog', self.cb_dog, 10)

        self.get_logger().info(
            f'lio_tf_bridge ready: odom_source={self.odom_source} '
            f'extrinsic T=({self.t_bl_lf[0]:.3f},{self.t_bl_lf[1]:.3f},{self.t_bl_lf[2]:.3f}) '
            f'pitch={self.get_parameter("lidar_pitch").value:.3f}')

    def _publish_odom_camera_init(self):
        t = TransformStamped()
        t.header.stamp.sec = 0
        t.header.stamp.nanosec = 0
        t.header.frame_id = 'odom'
        t.child_frame_id = 'camera_init'
        t.transform.rotation.w = 1.0
        self.tf_static_bc.sendTransform(t)

    def _publish_nav_odom(self, x, y, yaw, twist_msg, stamp):
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_link'
        out.pose.pose.position.x = float(x)
        out.pose.pose.position.y = float(y)
        out.pose.pose.position.z = 0.0
        out.pose.pose.orientation.z = math.sin(yaw * 0.5)
        out.pose.pose.orientation.w = math.cos(yaw * 0.5)
        out.twist = twist_msg
        self.pub_odom.publish(out)
        self.pub_odom2.publish(out)

        fp = TransformStamped()
        fp.header.stamp = stamp
        fp.header.frame_id = 'odom'
        fp.child_frame_id = 'base_footprint'
        fp.transform.translation.x = float(x)
        fp.transform.translation.y = float(y)
        fp.transform.translation.z = 0.0
        fp.transform.rotation.z = math.sin(yaw * 0.5)
        fp.transform.rotation.w = math.cos(yaw * 0.5)
        self.tf_bc.sendTransform(fp)

    def cb_dog(self, msg: Odometry):
        """机身 SDK 里程计 → Nav2 平面 odom。"""
        stamp = self.get_clock().now().to_msg()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat((q.x, q.y, q.z, q.w))
        # 狗里程计偶发飞数时丢弃本帧，避免箭头再次飞出地图
        if abs(p.x) > 500.0 or abs(p.y) > 500.0:
            now = self.get_clock().now().nanoseconds
            if now - getattr(self, '_dog_drop_ns', 0) > int(2e9):
                self._dog_drop_ns = now
                self.get_logger().warn(f'drop insane /odom_dog ({p.x:.1f},{p.y:.1f})')
            return
        self._publish_nav_odom(p.x, p.y, yaw, msg.twist, stamp)

    def cb_lio(self, msg: Odometry):
        """FAST-LIO → camera_init→body；仅 lio 模式才写平面里程计。"""
        p = msg.pose.pose
        q_cb = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        t_cb = (p.position.x, p.position.y, p.position.z)
        stamp = self.get_clock().now().to_msg()

        # 始终维持激光 TF（nav_scan 用）
        t = TransformStamped()
        t.header.stamp = stamp
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

        if self.odom_source != 'lio':
            return

        # 旧路径：LIO 平面里程计（易漂，仅调试）
        if abs(t_cb[0]) > 500.0 or abs(t_cb[1]) > 500.0:
            now = self.get_clock().now().nanoseconds
            if now - getattr(self, '_lio_drop_ns', 0) > int(2e9):
                self._lio_drop_ns = now
                self.get_logger().warn(f'drop insane LIO odom ({t_cb[0]:.1f},{t_cb[1]:.1f})')
            return
        q_ob = quat_mult(q_cb, self.q_lf_bl)
        t_ob = quat_rotate(q_cb, self.t_lf_bl)
        t_ob = (t_cb[0] + t_ob[0], t_cb[1] + t_ob[1], t_cb[2] + t_ob[2])
        yaw = yaw_from_quat(q_ob)
        self._publish_nav_odom(t_ob[0], t_ob[1], yaw, msg.twist, stamp)


def main():
    rclpy.init()
    node = LioTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
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

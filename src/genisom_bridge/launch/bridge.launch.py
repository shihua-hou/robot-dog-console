"""Launch genisom_bridge with optional lidar extrinsic params."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='genisom_bridge',
            executable='genisom_bridge',
            name='genisom_bridge',
            output='screen',
            parameters=[{
                'local_ip': '192.168.168.150',
                'local_port': 43988,
                'dog_ip': '192.168.168.168',
                'state_rate': 10.0,
                # Disable SDK odom/TF so Nav2 uses FAST-LIO via lio_tf_bridge.
                'publish_odom_tf': False,
                'lidar_x': 0.25,
                'lidar_y': 0.0,
                'lidar_z': 0.45,
                'lidar_roll': 0.0,
                'lidar_pitch': -0.7853981634,
                'lidar_yaw': 0.0,
            }],
        ),
    ])

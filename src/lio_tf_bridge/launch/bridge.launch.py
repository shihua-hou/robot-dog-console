"""Launch lio_tf_bridge with lidar extrinsic params (must match genisom_bridge)."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lio_tf_bridge',
            executable='lio_tf_bridge',
            name='lio_tf_bridge',
            output='screen',
            parameters=[{
                'lidar_x': 0.25,
                'lidar_y': 0.0,
                'lidar_z': 0.45,
                'lidar_roll': 0.0,
                'lidar_pitch': -0.7853981634,
                'lidar_yaw': 0.0,
            }],
        ),
        # Static body(livox) -> base_link (inverse of base_link->livox_frame).
        # FAST-LIO2 publishes camera_init->body where body == lidar frame, and
        # genisom_bridge publishes base_link->livox_frame; this closes the tree.
        # R^T * (-t) for T=(0.25,0,0.45) RPY=(0,-45deg,0) => (-0.4950, 0, -0.1414), RPY=(0, +45deg, 0).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='body_to_base_link_static',
            arguments=[
                '-0.4950', '0', '-0.1414',
                '0', '0.7853981634', '0',
                'body', 'base_link',
            ],
        ),
    ])

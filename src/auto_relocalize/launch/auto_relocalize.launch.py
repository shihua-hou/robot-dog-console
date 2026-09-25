#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='auto_relocalize',
            executable='auto_relocalize',
            name='auto_relocalize',
            output='screen',
            parameters=[{
                # 核心：红激光大部分必须贴黑墙
                'min_hit_ratio': 0.50,
                'max_unk_ratio': 0.35,
                'hit_dist': 0.25,
                'max_beam_range': 8.0,
                'local_radius': 5.0,
                # 初值不必精确：附近平移 + 旋转搜索
                'search_xy': 2.0,
                'xy_step': 0.15,
                'yaw_span_deg': 75.0,
                'yaw_step_deg': 6.0,
                'icp_max_corr': 0.55,
                'icp_max_iter': 50,
                'icp_fitness_max': 0.10,
                'watchdog_en': True,
                'watchdog_hit': 0.35,
                'watchdog_count': 6,
            }],
        ),
    ])

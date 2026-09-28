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
                # 启动不盲搜（避免开盲盒）；看门狗 + 导航结束后自动纠偏
                'auto_on_startup': False,
                'watchdog_en': True,
                'min_black_hit': 0.60,
                'reloc_trigger_hit': 0.35,
                'post_nav_hit': 0.55,
                'post_nav_settle_sec': 1.5,
                'mid_wait_sec': 15.0,
                'reloc_cooldown_sec': 45.0,
                'accept_score': 0.40,
                'sigma': 0.25,
                'watchdog_sigma': 0.08,
                'coarse_step': 0.20,
                'coarse_yaw_step_deg': 10.0,
                'max_beam_range': 8.0,
                'min_clearance': 0.12,
                'num_threads': 4,
                'watchdog_count': 3,
            }],
        ),
    ])

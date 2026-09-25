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
                # 导航启动后自动跑一次全局重定位, 不需要先点"设初始位姿"
                'auto_on_startup': True,
                'accept_score': 0.55,
                'sigma': 0.25,               # 全局搜索似然场高斯宽度(m)
                'coarse_step': 0.2,          # 粗搜位置步长(m)
                'coarse_yaw_step_deg': 10.0,
                'max_beam_range': 8.0,       # Mid360 室内有效距离, 比wheeltec的2D激光近
                'min_clearance': 0.12,
                'num_threads': 4,
                # 绑架/漂移检测看门狗: 位姿匹配分连续过低时自动再触发一次全局重定位
                'watchdog_en': True,
                'watchdog_sigma': 0.08,      # 看门狗用窄高斯严格打分
                'watchdog_score': 0.45,
                'watchdog_count': 3,
            }],
        ),
    ])

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
save_scan.launch.py —— 录制参考点云

用途：车停在起跑位，累积多帧雷达点云，体素降采样后存 CSV。
      这份 CSV 是后续 scan_match 的匹配基准。

⚠️ 运行期间车必须【静止】。

用法：
    ros2 launch my_racecar save_scan.launch.py
    ros2 launch my_racecar save_scan.launch.py accumulate_frames:=40 voxel_size:=0.05
    ros2 launch my_racecar save_scan.launch.py output_path:=/home/racecar/ref_cloud.csv
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('accumulate_frames', default_value='20',
                              description='累积多少帧（10Hz 雷达时 20 帧 = 2 秒）'),
        DeclareLaunchArgument('voxel_size', default_value='0.03',
                              description='体素降采样边长（米）。0 = 不降采样'),
        DeclareLaunchArgument('min_range', default_value='0.20',
                              description='忽略比这更近的点'),
        DeclareLaunchArgument('max_range', default_value='25.0',
                              description='忽略比这更远的点'),
        DeclareLaunchArgument('output_path', default_value='/home/racecar/src/my_racecar/include/ref_cloud.csv',
                              description='参考点云保存路径'),
    ]

    node = Node(
        package='my_racecar',
        executable='save_scan',
        name='save_scan',
        output='screen',
        parameters=[{
            'accumulate_frames': LaunchConfiguration('accumulate_frames'),
            'voxel_size':        LaunchConfiguration('voxel_size'),
            'min_range':         LaunchConfiguration('min_range'),
            'max_range':         LaunchConfiguration('max_range'),
            'output_path':       LaunchConfiguration('output_path'),
        }],
    )

    return LaunchDescription(args + [node])

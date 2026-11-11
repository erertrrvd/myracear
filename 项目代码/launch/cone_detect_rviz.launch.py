#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cone_detect_rviz.launch.py —— 一键启动：cone_detect 节点 + RViz（预设配置）

用法：
    ros2 launch my_racecar cone_detect_rviz.launch.py
    ros2 launch my_racecar cone_detect_rviz.launch.py frame_id:=laser
    ros2 launch my_racecar cone_detect_rviz.launch.py max_width:=0.45 cluster_tol:=0.30
    ros2 launch my_racecar cone_detect_rviz.launch.py use_rviz:=false

RViz 里会看到：
    🟢 绿色大球（带暗绿描边）= 选中的参考锥桶
    ⚫ 灰色半透明小球        = 被淘汰的候选
    🟡 黄色线段              = 选中锥桶之间的连线
    🟡 黄色数字              = 两两之间的距离（成对距离指纹）
    ⚪ 白色文字 "#0  1.36m"  = 编号 + 到车的距离

⚠️ 节点只累积前 N 帧就永久锁定，所以启动后的前 0.4 秒车要保持静止。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_racecar')

    candidates = [
        os.path.join(pkg_share, 'rviz', 'cone_detect.rviz'),
        '/home/racecar/src/my_racecar/rviz/cone_detect.rviz',
    ]
    rviz_cfg = None
    for c in candidates:
        if os.path.exists(c):
            rviz_cfg = c
            break
    if rviz_cfg is None:
        rviz_cfg = candidates[0]

    args = [
        DeclareLaunchArgument('min_range', default_value='0.30',
                              description='忽略比这更近的点'),
        DeclareLaunchArgument('max_range', default_value='8.00',
                              description='忽略比这更远的点'),
        DeclareLaunchArgument('accumulate_frames', default_value='20',
                              description='累积多少帧后检测一次（车需静止）'),
        DeclareLaunchArgument('cluster_tol', default_value='0.25',
                              description='聚类距离阈值'),
        DeclareLaunchArgument('min_points', default_value='4',
                              description='簇最少点数'),
        DeclareLaunchArgument('max_points', default_value='120',
                              description='簇最多点数（超过认为是墙）'),
        DeclareLaunchArgument('min_width', default_value='0.03',
                              description='簇最小宽度'),
        DeclareLaunchArgument('max_width', default_value='0.35',
                              description='簇最大宽度（超过认为是人）'),
        DeclareLaunchArgument('max_cones', default_value='4',
                              description='取最近的几个锥桶'),
        DeclareLaunchArgument('frame_id', default_value='laser_link',
                              description='★ 雷达坐标系名字。RViz 的 Fixed Frame 会跟着改'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='是否同时启动 RViz'),
    ]

    frame = LaunchConfiguration('frame_id')

    cone_detect_node = Node(
        package='my_racecar',
        executable='cone_detect',
        name='cone_detect',
        output='screen',
        parameters=[{
            'min_range':         LaunchConfiguration('min_range'),
            'max_range':         LaunchConfiguration('max_range'),
            'accumulate_frames': LaunchConfiguration('accumulate_frames'),
            'cluster_tol':       LaunchConfiguration('cluster_tol'),
            'min_points':        LaunchConfiguration('min_points'),
            'max_points':        LaunchConfiguration('max_points'),
            'min_width':         LaunchConfiguration('min_width'),
            'max_width':         LaunchConfiguration('max_width'),
            'max_cones':         LaunchConfiguration('max_cones'),
            'frame_id':          frame,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_cfg, '-f', frame],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(args + [cone_detect_node, rviz_node])

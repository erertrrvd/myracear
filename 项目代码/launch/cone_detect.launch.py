#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cone_detect.launch.py —— 锥桶检测调试 launch

用途：单独启动锥桶检测节点，方便调参和观察 RViz 效果。
      不启动其它节点，不和现有流程冲突。

行为：只累积前 N 帧（默认 20 帧）雷达数据，检测一次后【永久锁定】。
      话题用 transient_local 发布，RViz 晚开也能收到结果。

用法：
    ros2 launch my_racecar cone_detect.launch.py
    ros2 launch my_racecar cone_detect.launch.py accumulate_frames:=30
    ros2 launch my_racecar cone_detect.launch.py max_width:=0.45 cluster_tol:=0.30

RViz 观察：
    Fixed Frame: laser_link
    Add -> By topic -> /cone_detect/markers -> MarkerArray
        绿色大球 = 选中的参考锥桶
        灰色小球 = 被淘汰的候选（说明附近有多余锥桶）
        黄色线段 = 选中锥桶之间的连线
        黄色数字 = 两两之间的距离（成对距离指纹）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---------------- 可调参数 ----------------
    args = [
        # 点云预过滤
        DeclareLaunchArgument('min_range', default_value='0.30',
                              description='忽略比这更近的点（车体/地面）'),
        DeclareLaunchArgument('max_range', default_value='8.00',
                              description='忽略比这更远的点'),

        # 累积帧数
        DeclareLaunchArgument('accumulate_frames', default_value='20',
                              description='累积多少帧后检测一次（车需静止）'),

        # 聚类
        DeclareLaunchArgument('cluster_tol', default_value='0.25',
                              description='聚类距离阈值。太小→锥桶被拆开；太大→锥桶粘连'),

        # 簇过滤
        DeclareLaunchArgument('min_points', default_value='4',
                              description='簇最少点数'),
        DeclareLaunchArgument('max_points', default_value='120',
                              description='簇最多点数（超过认为是墙）'),
        DeclareLaunchArgument('min_width', default_value='0.03',
                              description='簇最小宽度'),
        DeclareLaunchArgument('max_width', default_value='0.35',
                              description='簇最大宽度 ★ 超过认为是人。建议设为"锥桶宽度×1.8"'),

        # 行为
        DeclareLaunchArgument('max_cones', default_value='4',
                              description='取最近的几个锥桶作为候选'),
        DeclareLaunchArgument('frame_id', default_value='laser_link',
                              description='RViz marker 的坐标系'),
    ]

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
            'frame_id':          LaunchConfiguration('frame_id'),
        }],
    )

    return LaunchDescription(args + [cone_detect_node])

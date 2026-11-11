#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cone_match_rviz.launch.py —— 一键启动：cone_match 节点 + RViz（预设配置）

用法：
    ros2 launch my_racecar cone_match_rviz.launch.py
    ros2 launch my_racecar cone_match_rviz.launch.py frame_id:=laser
    ros2 launch my_racecar cone_match_rviz.launch.py use_rviz:=false
    ros2 launch my_racecar cone_match_rviz.launch.py max_width:=0.45

前提：
    /home/racecar/src/my_racecar/include/ref_cloud.csv 已存在
    （用 save_scan.launch.py 录制）

RViz 里会看到：
    🔵 蓝色球 = 参考锥桶
    🟢 绿色球 = 当前锥桶（已按配准结果变换）
    🟡 黄色连线 = 配对成功的锥桶两两相连（线越短说明配得越准）
    🔴 红色箭头 = +X（雷达正前方）
    🟢 绿色箭头 = +Y（雷达左方）
    🟡 结果文字 = dx / dy / dyaw / RMS / 配对数
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
        os.path.join(pkg_share, 'rviz', 'cone_match.rviz'),
        '/home/racecar/src/my_racecar/rviz/cone_match.rviz',
    ]
    rviz_cfg = candidates[0]
    for c in candidates:
        if os.path.exists(c):
            rviz_cfg = c
            break

    args = [
        DeclareLaunchArgument('ref_path',
                              default_value='/home/racecar/src/my_racecar/include/ref_cloud.csv',
                              description='参考点云 CSV（save_scan 生成的）'),
        DeclareLaunchArgument('accumulate_frames', default_value='20',
                              description='累积多少帧当前点云'),
        # 锥桶提取参数
        DeclareLaunchArgument('cluster_tol', default_value='0.25',
                              description='聚类距离阈值'),
        DeclareLaunchArgument('min_points', default_value='4',
                              description='簇最少点数'),
        DeclareLaunchArgument('max_points', default_value='200',
                              description='簇最多点数'),
        DeclareLaunchArgument('min_width', default_value='0.03',
                              description='簇最小宽度'),
        DeclareLaunchArgument('max_width', default_value='0.35',
                              description='簇最大宽度（超过认为是人）'),
        DeclareLaunchArgument('max_range', default_value='8.00',
                              description='忽略比这更远的点'),
        DeclareLaunchArgument('max_cones', default_value='12',
                              description='最多用几个锥桶做配准'),
        # 配准判据
        DeclareLaunchArgument('min_pairs', default_value='3',
                              description='最少配对数'),
        DeclareLaunchArgument('sig_tol', default_value='0.15',
                              description='指纹误差阈值（米，三条边之和）'),
        DeclareLaunchArgument('max_rms', default_value='0.05',
                              description='RMS 残差阈值（米）'),
        DeclareLaunchArgument('max_dxy', default_value='0.60',
                              description='平移修正量上限（米）'),
        DeclareLaunchArgument('max_dyaw_deg', default_value='25.0',
                              description='航向修正量上限（度）'),
        DeclareLaunchArgument('frame_id', default_value='laser_link',
                              description='★ 雷达坐标系名，RViz 的 Fixed Frame 会跟着改'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='是否同时启动 RViz'),
    ]

    frame = LaunchConfiguration('frame_id')

    cone_match_node = Node(
        package='my_racecar',
        executable='cone_match',
        name='cone_match',
        output='screen',
        parameters=[{
            'ref_path':          LaunchConfiguration('ref_path'),
            'accumulate_frames': LaunchConfiguration('accumulate_frames'),
            'cluster_tol':       LaunchConfiguration('cluster_tol'),
            'min_points':        LaunchConfiguration('min_points'),
            'max_points':        LaunchConfiguration('max_points'),
            'min_width':         LaunchConfiguration('min_width'),
            'max_width':         LaunchConfiguration('max_width'),
            'max_range':         LaunchConfiguration('max_range'),
            'max_cones':         LaunchConfiguration('max_cones'),
            'min_pairs':         LaunchConfiguration('min_pairs'),
            'sig_tol':           LaunchConfiguration('sig_tol'),
            'max_rms':           LaunchConfiguration('max_rms'),
            'max_dxy':           LaunchConfiguration('max_dxy'),
            'max_dyaw_deg':      LaunchConfiguration('max_dyaw_deg'),
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

    return LaunchDescription(args + [cone_match_node, rviz_node])

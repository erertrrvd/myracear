#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scan_match_rviz.launch.py —— 一键启动：scan_match 节点 + RViz（预设配置）

不用手动配 RViz，直接跑这个就能看到结果。

用法：
    ros2 launch my_racecar scan_match_rviz.launch.py
    ros2 launch my_racecar scan_match_rviz.launch.py ref_path:=/home/racecar/ref_cloud.csv
    ros2 launch my_racecar scan_match_rviz.launch.py frame_id:=laser

⚠️ 如果你的雷达 frame_id 不是 laser_link，必须传 frame_id 参数，
   这样节点和 RViz 的 Fixed Frame 会一起改，不用手动调。

RViz 里会看到：
    🔴 红色箭头 = +X（雷达正前方）
    🟢 绿色箭头 = +Y（雷达左方）
    🔵 蓝色点云 = 参考点云
    🟢 绿色点云 = 当前点云（已按匹配结果变换）
    🟡 结果文字 = dx / dy / dyaw / RMS / 内点比例
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

    # 支持两种启动方式（install 后的 share 目录 / 源码目录）
    candidates = [
        os.path.join(pkg_share, 'rviz', 'scan_match.rviz'),          # colcon install 后
        '/home/racecar/src/my_racecar/rviz/scan_match.rviz',          # 源码目录
    ]
    rviz_cfg = None
    for c in candidates:
        if os.path.exists(c):
            rviz_cfg = c
            break
    if rviz_cfg is None:
        rviz_cfg = candidates[0]   # 都找不到就用第一个路径，让 RViz 报错提示

    args = [
        DeclareLaunchArgument('ref_path',
                              default_value='/home/racecar/src/my_racecar/include/ref_cloud.csv',
                              description='参考点云 CSV（save_scan 生成的）'),
        DeclareLaunchArgument('accumulate_frames', default_value='20',
                              description='累积多少帧当前点云'),
        DeclareLaunchArgument('min_range', default_value='0.20',
                              description='忽略比这更近的点'),
        DeclareLaunchArgument('max_range', default_value='25.0',
                              description='忽略比这更远的点'),
        DeclareLaunchArgument('yaw_search_deg', default_value='8.0',
                              description='yaw 初值搜索范围（±度）'),
        DeclareLaunchArgument('yaw_search_step', default_value='1.0',
                              description='yaw 搜索步长（度）'),
        DeclareLaunchArgument('cur_voxel', default_value='0.03',
                              description='当前点云降采样体素（米）'),
        DeclareLaunchArgument('search_r', default_value='0.20',
                              description='对应点搜索半径（米）'),
        DeclareLaunchArgument('max_dxy', default_value='0.50',
                              description='平移修正量上限（米），超界判失败'),
        DeclareLaunchArgument('max_dyaw_deg', default_value='20.0',
                              description='航向修正量上限（度），超界判失败'),
        DeclareLaunchArgument('max_rms', default_value='0.03',
                              description='RMS 残差阈值（米），超过判失败'),
        DeclareLaunchArgument('min_inlier_ratio', default_value='0.60',
                              description='内点比例阈值，低于判失败'),
        DeclareLaunchArgument('frame_id', default_value='laser_link',
                              description='★ 雷达坐标系名字。RViz 的 Fixed Frame 会跟着改成这个'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='是否同时启动 RViz'),
    ]

    frame = LaunchConfiguration('frame_id')

    scan_match_node = Node(
        package='my_racecar',
        executable='scan_match',
        name='scan_match',
        output='screen',
        parameters=[{
            'ref_path':          LaunchConfiguration('ref_path'),
            'accumulate_frames': LaunchConfiguration('accumulate_frames'),
            'min_range':         LaunchConfiguration('min_range'),
            'max_range':         LaunchConfiguration('max_range'),
            'yaw_search_deg':    LaunchConfiguration('yaw_search_deg'),
            'yaw_search_step':   LaunchConfiguration('yaw_search_step'),
            'cur_voxel':         LaunchConfiguration('cur_voxel'),
            'search_r':          LaunchConfiguration('search_r'),
            'max_dxy':           LaunchConfiguration('max_dxy'),
            'max_dyaw_deg':      LaunchConfiguration('max_dyaw_deg'),
            'max_rms':           LaunchConfiguration('max_rms'),
            'min_inlier_ratio':  LaunchConfiguration('min_inlier_ratio'),
            'frame_id':          frame,
        }],
    )

    # ★ 关键：把 Fixed Frame 通过 -f 参数传给 RViz，不用手动设
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_cfg, '-f', frame],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription(args + [scan_match_node, rviz_node])

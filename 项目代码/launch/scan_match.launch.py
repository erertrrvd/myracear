#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scan_match.launch.py —— 点云匹配求初始位姿

用途：读取 save_scan 录制的参考点云，与当前雷达点云配准，
      输出 (dx, dy, dyaw)，并在 RViz 中可视化。

⚠️ 运行期间车必须【静止】。
⚠️ 结果用 transient_local 发布，RViz 晚开也能收到。

用法：
    ros2 launch my_racecar scan_match.launch.py
    ros2 launch my_racecar scan_match.launch.py ref_path:=/home/racecar/ref_cloud.csv
    ros2 launch my_racecar scan_match.launch.py frame_id:=laser

RViz 观察（Fixed Frame 必须和 frame_id 一致）：
    Add -> By topic -> /scan_match/markers -> MarkerArray

    图例：
      🔵 蓝色点云  = 参考点云（录制时存下来的）
      🟢 绿色点云  = 当前点云（已按匹配结果变换）
      🔴 红色箭头  = +X 方向（雷达正前方）
      🟢 绿色箭头  = +Y 方向（雷达左方）
      ⚪ "origin (lidar)" = 雷达原点
      🟡 结果文字  = dx / dy / dyaw / RMS / 内点比例

    判断标准：
      · 蓝绿两片点云【重合】          → 匹配成功
      · 明显错开 / 旋转了             → 匹配失败或车放偏了
      · 结果文字是绿色 [OK]           → 质量指标达标
      · 结果文字是红色 [FAIL]         → 不可信，需检查
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('ref_path',
                              default_value='/home/racecar/src/my_racecar/include/ref_cloud.csv',
                              description='参考点云 CSV 路径（save_scan 生成的）'),
        DeclareLaunchArgument('accumulate_frames', default_value='20',
                              description='累积多少帧当前点云'),
        DeclareLaunchArgument('min_range', default_value='0.20',
                              description='忽略比这更近的点'),
        DeclareLaunchArgument('max_range', default_value='25.0',
                              description='忽略比这更远的点'),

        # ---- 匹配参数 ----
        DeclareLaunchArgument('yaw_search_deg', default_value='8.0',
                              description='yaw 初值搜索范围（±度）。放得越歪要越大'),
        DeclareLaunchArgument('yaw_search_step', default_value='1.0',
                              description='yaw 搜索步长（度）。越小越准但越慢'),
        DeclareLaunchArgument('cur_voxel', default_value='0.03',
                              description='当前点云降采样体素（米）。参考点云不降采样'),
        DeclareLaunchArgument('search_r', default_value='0.20',
                              description='对应点搜索半径（米）。要能罩住 yaw 初值误差'),

        # ---- 自检阈值 ----
        DeclareLaunchArgument('max_dxy', default_value='0.50',
                              description='平移修正量上限（米），超界判失败'),
        DeclareLaunchArgument('max_dyaw_deg', default_value='20.0',
                              description='航向修正量上限（度），超界判失败'),
        DeclareLaunchArgument('max_rms', default_value='0.03',
                              description='RMS 残差阈值（米）。超过判失败'),
        DeclareLaunchArgument('min_inlier_ratio', default_value='0.60',
                              description='内点比例阈值。低于判失败'),

        DeclareLaunchArgument('frame_id', default_value='laser_link',
                              description='RViz marker 坐标系（必须和 RViz Fixed Frame 一致）'),
    ]

    node = Node(
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
            'frame_id':          LaunchConfiguration('frame_id'),
        }],
    )

    return LaunchDescription(args + [node])

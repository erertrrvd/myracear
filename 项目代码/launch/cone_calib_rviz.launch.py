#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cone_calib_rviz.launch.py —— 一键启动：初始点校准节点 + RViz（预设配置）

用法：
    # 看「校准结果」（默认）—— 参考/候选/匹配/对齐/位姿/结论
    ros2 launch my_racecar cone_calib_rviz.launch.py

    # 看「录制结果」—— 选中的 4 个参考锥桶 + 指纹
    ros2 launch my_racecar cone_calib_rviz.launch.py mode:=record

    # 不启 RViz（只用终端）
    ros2 launch my_racecar cone_calib_rviz.launch.py use_rviz:=false

    # 坐标系不对时手动指定（一般不用改）
    ros2 launch my_racecar cone_calib_rviz.launch.py frame_id:=laser

RViz 里会看到什么（mode:=run 时）：
    🟢 绿色大球 = 参考基准的 4 个锥桶（R0..R3）
    ⚫ 灰色小球 = 本次检出的其它候选
    🔵 蓝色球   = 本次匹配上的 4 个锥桶
    🟡 黄色球   = 蓝球按解出的位姿变换到参考系后的位置
                  ★ 黄球和绿球几乎重合 = 匹配正确，一眼可辨
    🔷 青色线   = 对应关系（标注残差 mm）
    🔴 红色箭头 = 车的位移偏差（放大 10 倍显示方向）
    🟢/🔴 状态灯 = 绿=可以起跑，红=请重新摆放（放在锥桶下方，不会被挡住）

⚠️ 节点只累积前 20 帧就永久锁定（约 2 秒），启动后车要保持静止。
⚠️ 需要 /scan 已经在发布（先启动雷达驱动）。
⚠️ 如果 RViz 里什么都看不到，先确认坐标系对不对：
       ros2 topic echo /scan --field header.frame_id --once
   然后把它的值用 frame_id:= 传进来。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _find_rviz_config(cfg_name):
    """按优先级找 rviz 配置：install/share 目录 > 源码目录"""
    pkg_share = get_package_share_directory('my_racecar')
    candidates = [
        os.path.join(pkg_share, 'rviz', cfg_name),
        os.path.join('/home/racecar/src/my_racecar/rviz', cfg_name),
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    return candidates[0]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('mode', default_value='run',
                              description="run = 校准(步骤2) / record = 录制参考(步骤1)"),
        DeclareLaunchArgument('frame_id', default_value='laser_link',
                              description='RViz 的 Fixed Frame。节点会优先用 /scan 实际发布的 frame_id'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='是否同时启动 RViz'),
        DeclareLaunchArgument('ref_path',
                              default_value='/home/racecar/calib_logs/cones_ref.csv',
                              description='校准用的参考基准文件（mode:=run 时用）'),
        DeclareLaunchArgument('output_path',
                              default_value='/home/racecar/calib_logs/cones_ref.csv',
                              description='录制输出文件（mode:=record 时用）'),
    ]

    mode     = LaunchConfiguration('mode')
    frame    = LaunchConfiguration('frame_id')
    use_rviz = LaunchConfiguration('use_rviz')

    # 用 PythonExpression 算出 'true'/'false' 字符串，IfCondition 才认
    cond_record = PythonExpression(["'true' if '", mode, "' == 'record' else 'false'"])
    cond_run    = PythonExpression(["'true' if '", mode, "' == 'run' else 'false'"])
    cond_rviz_record = PythonExpression(
        ["'true' if ('", mode, "' == 'record' and '", use_rviz,
         "' == 'true') else 'false'"])
    cond_rviz_run = PythonExpression(
        ["'true' if ('", mode, "' == 'run' and '", use_rviz,
         "' == 'true') else 'false'"])

    record_node = Node(
        package='my_racecar',
        executable='cone_calib_record',
        name='cone_calib_record',
        output='screen',
        parameters=[{
            'frame_id':    frame,
            'output_path': LaunchConfiguration('output_path'),
        }],
        condition=IfCondition(cond_record),
    )

    run_node = Node(
        package='my_racecar',
        executable='cone_calib_run',
        name='cone_calib_run',
        output='screen',
        parameters=[{
            'frame_id': frame,
            'ref_path': LaunchConfiguration('ref_path'),
        }],
        condition=IfCondition(cond_run),
    )

    rviz_record = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', _find_rviz_config('cone_calib_record.rviz'), '-f', frame],
        condition=IfCondition(cond_rviz_record),
    )

    rviz_run = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', _find_rviz_config('cone_calib_run.rviz'), '-f', frame],
        condition=IfCondition(cond_rviz_run),
    )

    return LaunchDescription(args + [record_node, run_node, rviz_record, rviz_run])

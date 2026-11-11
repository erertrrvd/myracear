"""车端分布式建图入口：只启动 imu_encoder_mix（发 /encoder_imu_odom），雷达数据经 WiFi 由电脑端 SLAM 建图。

电脑端 scan_tf_bridge 只订阅 /scan 与 /encoder_imu_odom 并自己生成 TF，不使用车端 /tf，
所以本入口不再启动 odom_tf_adapter（它发 odom->base_footprint，分布式建图用不到，且约占 14% CPU）。
odom_tf_adapter 仍保留给 mapping_only / race_mapping（车上跑 SLAM 时需要）。

前置条件：
  1. 已运行 Run_car.sh（雷达 /scan、IMU、编码器驱动，以及 base_footprint -> base_link
     -> laser_link 静态 TF）；publish_laser_tf 默认 false，不与 Run_car.sh 重复发布；
  2. 若固定路径行驶入口已启动 imu_encoder_mix，必须 start_odom:=false，避免重复融合；
  3. 与 mapping_only.launch.py / race_mapping.launch.py 互斥，不能同时运行；
  4. 电脑端配套入口为 distributed_mapping_pc.launch.py，网络条件见 mapping/DISTRIBUTED.md。
"""
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    start_odom = LaunchConfiguration('start_odom').perform(context).lower()
    if start_odom not in ('true', 'false'):
        raise RuntimeError('start_odom 只能是 true 或 false')
    publish_tf = LaunchConfiguration('publish_laser_tf').perform(context).lower()
    if publish_tf not in ('true', 'false'):
        raise RuntimeError('publish_laser_tf 只能是 true 或 false')
    frame = LaunchConfiguration('laser_frame').perform(context).strip()
    if not frame or frame in ('map', 'odom', 'base_footprint'):
        raise RuntimeError('laser_frame 必须是 /scan 使用的独立雷达坐标系')
    names = ('laser_x', 'laser_y', 'laser_z', 'laser_roll', 'laser_pitch', 'laser_yaw')
    laser = [float(LaunchConfiguration(name).perform(context)) for name in names]
    if not all(math.isfinite(v) for v in laser):
        raise RuntimeError('雷达外参必须是有限数')

    actions = [
        LogInfo(msg='车端分布式建图：只提供 /encoder_imu_odom（imu_encoder_mix），SLAM 在电脑端运行；'
                    '不与 mapping_only / race_mapping 同时运行'),
    ]
    if start_odom == 'true':
        actions.append(Node(package='encoder_imu', executable='imu_encoder_mix',
                            name='imu_encoder_mix', output='screen'))
    # 已移除 odom_tf_adapter：电脑端 scan_tf_bridge 自己生成 TF，车端 odom->base_footprint 没人用。
    if start_odom == 'false':
        actions.append(LogInfo(msg='start_odom:=false 且不启动 odom_tf_adapter：本入口没有启动任何节点，'
                                   '请确认 /encoder_imu_odom 已由其他入口发布'))
    if publish_tf == 'true':
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['--x', str(laser[0]), '--y', str(laser[1]), '--z', str(laser[2]),
                       '--roll', str(laser[3]), '--pitch', str(laser[4]),
                       '--yaw', str(laser[5]), '--frame-id', 'base_footprint',
                       '--child-frame-id', frame],
        ))
    return actions


def generate_launch_description():
    defaults = {
        'start_odom': 'true',
        'publish_laser_tf': 'false',
        'laser_frame': 'laser_link',
        'laser_x': '0.0', 'laser_y': '0.0', 'laser_z': '0.30',
        'laser_roll': '0.0', 'laser_pitch': '0.0', 'laser_yaw': '0.0',
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default)
          for name, default in defaults.items()),
        OpaqueFunction(function=_setup),
    ])

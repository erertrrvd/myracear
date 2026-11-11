"""Three-run fixed-route mapping; starts the old controller only after SLAM is ready."""
import hashlib
import json
import math
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, Shutdown, LogInfo)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _sha256(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def _check_previous(root, run):
    prior = root / f'run{run - 1}'
    manifest = prior / 'complete.json'
    if not manifest.is_file():
        raise RuntimeError(f'上一趟没有完整保存标记：{manifest}')
    record = json.loads(manifest.read_text())
    if record.get('run_index') != run - 1:
        raise RuntimeError('上一趟地图编号不匹配')
    for name in ('map.posegraph', 'map.data', 'map.pgm', 'map.yaml', 'map.raw', 'run.jsonl'):
        path = prior / name
        expected = record.get('files', {}).get(name, {})
        if not path.is_file() or path.stat().st_size != expected.get('bytes'):
            raise RuntimeError(f'上一趟地图文件丢失或大小变化：{path}')
        if _sha256(path) != expected.get('sha256'):
            raise RuntimeError(f'上一趟地图校验失败：{path}')
    return prior / 'map'


def _setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    run = int(value('run_index'))
    if run not in (1, 2, 3):
        raise RuntimeError('run_index 只能是 1、2、3')
    root = Path(value('map_dir')).expanduser().resolve()
    if (root / f'run{run}').exists():
        raise RuntimeError(f'第 {run} 趟已有保存结果，拒绝覆盖；请先核对目录')
    path_csv = Path('/home/racecar/src/my_racecar/include/path.csv')
    if not path_csv.is_file():
        raise RuntimeError(f'原固定路径代码需要此文件：{path_csv}')
    if any(value(n) == 'unset' for n in ('laser_x', 'laser_y', 'laser_z', 'laser_yaw')):
        raise RuntimeError('必须提供实测雷达外参')
    laser = [float(value(n)) for n in ('laser_x', 'laser_y', 'laser_z', 'laser_yaw')]
    pose = [float(value(n)) for n in ('start_x', 'start_y', 'start_yaw')]
    if not all(math.isfinite(v) for v in laser + pose):
        raise RuntimeError('外参与起点位姿必须是有限数')
    if float(value('start_radius')) <= 0 or float(value('start_yaw_tolerance')) <= 0:
        raise RuntimeError('起点允许偏差必须大于零')
    if not 0 <= float(value('min_match_score')) <= 1:
        raise RuntimeError('min_match_score 必须在 0 到 1 之间')
    old_map = _check_previous(root, run) if run > 1 else None
    root.mkdir(parents=True, exist_ok=True)

    share = Path(get_package_share_directory('my_racecar'))
    config = share / 'mapping' / 'config' / 'mapper.yaml'
    original = share / 'launch' / 'move_test.launch.py'
    mapper_overrides = {}
    if old_map:
        mapper_overrides = {'map_file_name': str(old_map), 'map_start_pose': pose}
    mapper = Node(package='slam_toolbox', executable='async_slam_toolbox_node',
                  name='slam_toolbox', output='screen',
                  parameters=[str(config), mapper_overrides])
    ready = Node(package='my_racecar', executable='mapping_ready', output='screen',
                 arguments=['--run-index', str(run), '--map-dir', str(root),
                            '--laser-frame', value('laser_frame'),
                            '--start-x', str(pose[0]), '--start-y', str(pose[1]),
                            '--start-yaw', str(pose[2]),
                            '--start-radius', value('start_radius'),
                            '--start-yaw-tolerance', value('start_yaw_tolerance'),
                            '--min-match-score', value('min_match_score'),
                            '--timeout', value('ready_timeout')])

    def on_ready_exit(event, _context):
        if event.returncode != 0:
            return [Shutdown(reason='建图起跑检查失败，固定路径控制未启动')]
        return [LogInfo(msg='建图检查通过，启动原 move_test 固定路径控制'),
                IncludeLaunchDescription(PythonLaunchDescriptionSource(str(original)),
                                         launch_arguments={'start_odom': 'false'}.items())]

    return [
        LogInfo(msg=f'第 {run} 趟：固定路径 + {"续建" if old_map else "空图建图"}；保存目录 {root}'),
        Node(package='encoder_imu', executable='imu_encoder_mix',
             name='imu_encoder_mix', output='screen'),
        Node(package='my_racecar', executable='odom_tf_adapter', output='screen'),
        Node(package='my_racecar', executable='mapping_status_logger', output='screen',
             arguments=[str(root / f'.run{run}-live.jsonl'), str(run)]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', str(laser[0]), '--y', str(laser[1]), '--z', str(laser[2]),
                        '--yaw', str(laser[3]), '--pitch', '0', '--roll', '0',
                        '--frame-id', 'base_footprint', '--child-frame-id', value('laser_frame')]),
        mapper,
        ready,
        RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=on_ready_exit)),
    ]


def generate_launch_description():
    defaults = {
        'run_index': '1',
        'map_dir': '/home/racecar/maps',
        'laser_frame': 'laser_link',
        'laser_x': 'unset', 'laser_y': 'unset', 'laser_z': 'unset', 'laser_yaw': 'unset',
        'start_x': '0.0', 'start_y': '0.0', 'start_yaw': '0.0',
        'start_radius': '0.8', 'start_yaw_tolerance': '0.5',
        'min_match_score': '0.2', 'ready_timeout': '60.0',
    }
    return LaunchDescription([*(DeclareLaunchArgument(k, default_value=v)
                               for k, v in defaults.items()), OpaqueFunction(function=_setup)])

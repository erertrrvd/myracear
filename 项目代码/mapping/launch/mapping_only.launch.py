"""Map with a provisional laser transform, without starting vehicle control."""
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)
    root = Path(value('map_dir')).expanduser().resolve()
    if root == Path('/home/racecar/maps'):
        raise RuntimeError('独立建图不能写入三趟比赛地图目录，请指定单独的 map_dir')
    if (root / 'run1').exists():
        raise RuntimeError(f'地图已存在，拒绝覆盖：{root / "run1"}')

    frame = value('laser_frame').strip()
    if not frame or frame in ('map', 'odom', 'base_footprint'):
        raise RuntimeError('laser_frame 必须是 /scan 使用的独立雷达坐标系')
    publish_tf = value('publish_laser_tf').lower()
    if publish_tf not in ('true', 'false'):
        raise RuntimeError('publish_laser_tf 只能是 true 或 false')
    names = ('laser_x', 'laser_y', 'laser_z', 'laser_roll', 'laser_pitch', 'laser_yaw')
    laser = [float(value(name)) for name in names]
    timeout = float(value('ready_timeout'))
    if not all(math.isfinite(v) for v in laser) or not math.isfinite(timeout) or timeout <= 0:
        raise RuntimeError('雷达外参必须是有限数，ready_timeout 必须大于零')

    root.mkdir(parents=True, exist_ok=True)
    config = Path(get_package_share_directory('my_racecar')) / 'mapping' / 'config' / 'mapper.yaml'
    ready = Node(
        package='my_racecar', executable='mapping_ready', output='screen',
        arguments=['--run-index', '1', '--map-dir', str(root),
                   '--laser-frame', frame, '--start-x', '0', '--start-y', '0',
                   '--start-yaw', '0', '--start-radius', '0.8',
                   '--start-yaw-tolerance', '0.5', '--min-match-score', '0.2',
                   '--timeout', str(timeout)],
    )

    def on_ready_exit(event, _context):
        if event.returncode != 0:
            return [Shutdown(reason='独立建图检查失败；未启动行驶控制')]
        return [LogInfo(msg='扫描、TF 和地图初步检查通过；继续建图，不启动行驶控制')]

    actions = [
        LogInfo(msg=f'独立建图：地图目录 {root}；雷达帧 {frame}；不启动行驶控制'),
        Node(package='encoder_imu', executable='imu_encoder_mix',
             name='imu_encoder_mix', output='screen'),
        Node(package='my_racecar', executable='odom_tf_adapter', output='screen'),
        Node(package='my_racecar', executable='mapping_status_logger', output='screen',
             arguments=[str(root / '.run1-live.jsonl'), '1']),
    ]
    if publish_tf == 'true':
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=['--x', str(laser[0]), '--y', str(laser[1]), '--z', str(laser[2]),
                       '--roll', str(laser[3]), '--pitch', str(laser[4]),
                       '--yaw', str(laser[5]), '--frame-id', 'base_footprint',
                       '--child-frame-id', frame],
        ))
    actions.extend([
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', output='screen', parameters=[str(config)]),
        ready,
        RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=on_ready_exit)),
    ])
    return actions


def generate_launch_description():
    defaults = {
        'map_dir': '/home/racecar/maps/mapping_only',
        'laser_frame': 'laser_link',
        'laser_x': '0.0', 'laser_y': '0.0', 'laser_z': '0.30',
        'laser_roll': '0.0', 'laser_pitch': '0.0', 'laser_yaw': '0.0',
        'publish_laser_tf': 'true',
        'ready_timeout': '60.0',
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default)
          for name, default in defaults.items()),
        OpaqueFunction(function=_setup),
    ])

"""离线查看：已保存的地图 + path.csv / AB 轨迹，一起显示在 rviz2（不做拟合）。

  ros2 launch my_racecar view_path_on_map.launch.py \
      map_yaml:=/home/racecar/maps/mapping_only/run1/map.yaml

调试：extra_csv:=/路径/a.csv,/路径/b.csv 把任意 CSV 段高亮叠在图上（改文件会自动刷新）。
路径位置不对时，用 dx / dy / dyaw 手动平移旋转（米、弧度），例如 dx:=0.2 dyaw:=0.05。
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('my_racecar'))
    inc = '/home/racecar/src/my_racecar/include/'
    args = {
        'map_yaml': '',
        'path_csv': inc + 'path.csv',
        'ab_a_csv': inc + 'ab_a.csv',
        'ab_b_csv': inc + 'ab_b.csv',
        'ab_fork_index': '-1',
        'concat_dy': '0.0',
        'extra_csv': '',
        'inflate_cells': '0',     # 仅显示：障碍格加粗几格
        'dx': '0.0', 'dy': '0.0', 'dyaw': '0.0',
        'rviz': 'true',
        'rviz_config': str(share / 'mapping' / 'rviz' / 'path_on_map.rviz'),
    }
    params = {k: LaunchConfiguration(k) for k in
              ('map_yaml', 'path_csv', 'ab_a_csv', 'ab_b_csv', 'concat_dy', 'extra_csv', 'inflate_cells', 'dx', 'dy', 'dyaw')}
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()] + [
            Node(package='my_racecar', executable='view_path_on_map', output='screen',
                 parameters=[params, {'ab_fork_index': LaunchConfiguration('ab_fork_index')}]),
            Node(package='rviz2', executable='rviz2', output='screen',
                 arguments=['-d', LaunchConfiguration('rviz_config')],
                 condition=IfCondition(LaunchConfiguration('rviz'))),
        ])

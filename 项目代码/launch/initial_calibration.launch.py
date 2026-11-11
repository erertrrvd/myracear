from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_racecar',
            executable='initial_calibration_node',
            name='initial_calibration',
            output='screen',
            parameters=[{
                'init_scan_path': '/home/racecar/src/my_racecar/include/init_scan.csv',
                'skip_frames': 5,
                'error_threshold': 0.1
            }]
        )
    ])
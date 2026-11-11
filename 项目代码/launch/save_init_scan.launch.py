from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_racecar',
            executable='save_init_scan',
            name='save_init_scan',
            output='screen'
        )
    ])
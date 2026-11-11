# ==================== 【AB改动】整个文件为新增：启动里程计 + save_ab_path，ab:=a 或 ab:=b ====================
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('ab', default_value='a', description='录制哪一边：a 或 b'),

        # 里程计节点（定位）
        Node(
            package='encoder_imu',
            executable='imu_encoder_mix',
            name='imu_encoder_mix',
            output='screen',
        ),

        # 录制 A/B 停车轨迹 -> include/ab_a.csv 或 include/ab_b.csv
        Node(
            package='my_racecar',
            executable='save_ab_path',
            name='save_ab_path',
            output='screen',
            parameters=[{
                'ab': LaunchConfiguration('ab'),
                'save_dx': 0.1,
                'speed': 1.5,
            }]
        ),
    ])

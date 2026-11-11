import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node




# 定义所有参数
parameters = {

    'pid_kp': 45.0,
    'pid_ki': 1.4,
    'pid_kd': 20.0,
    'pid_integral_limit_min': -100.0,
    'pid_integral_limit_max': 100.0,
    'pid_output_limit_min': 1380.0,
    'pid_output_limit_max': 1620.0,
}


def generate_launch_description():
    # 获取包目录路径
    package_name = 'my_racecar'  
    
    
    return LaunchDescription([
        # 启动 GetDistance 节点
        Node(
            package=package_name,
            executable='transform_v',
            name='transform_v',
            output='screen',
            parameters=[parameters],
            
        ),
        

        
    ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_odom', default_value='true',
                              description='保持原入口默认启动里程计；组合建图入口设为 false'),
        # ==================== 【AB改动1】新增 launch 参数 ab：启动时指定停 A 还是 B ====================
        DeclareLaunchArgument('ab', default_value='a',
                              description='第二圈结束停哪个：a / b / none（不拼 AB）'),
    
      

    
       #启动里程计节点，定位
        Node(
            package='encoder_imu',
            executable='imu_encoder_mix',
            name='imu_encoder_mix',
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_odom')),
        ),
    
        #提取路径
        Node(
            package='my_racecar',  
            executable='get_local_path',  # 可执行文件名称
            name='get_local_path',
            output='screen',
            parameters=[{
                
                'front_dist': 0.5,
                'pub_global_path':0,
                'stop_point':3,  #????????????
                'slow_point':15, 
                'start_icp':0,
                # ==================== 【AB改动2】原 'ab_flag':1 改为 ab 参数，并新增 ab_speed / laps / ab_fork_index ====================
                'ab':LaunchConfiguration('ab'),   # a / b / none，启动时指定
                'concat_dy':0.0,
                'ab_speed':1.5,    # AB 段速度
                'laps':1,              # path.csv 已是完整两圈（录到分叉点为止）；若只录了 1 圈则设 2
                'ab_fork_index':-1,    # AB 分叉点在 path.csv 的序号（负数从末尾倒数，-1=最后一个点）
                # ==================== 【AB改动2】结束 ====================
                
            }]
            ),
        #横向运动控制
        Node(
            package='my_racecar',
            executable='pure_track_test',
            name='pure_track_test',
            output='screen',
            parameters=[{
                
                'linear_x': 2.0,
                'd_min':0.5,
                'k_v':0.5,
                'kp_yaw': 1.0,
                'kd_yaw': 0.0,
                'ff_kp':1.15,
                
                'theta_min':-25.0,
                'theta_max':25.0,
                
                'f_index':2,
                
                'traffic_light_time':5.2,
                'traffic_light_index': 800,  
                'first_stop_index': 800,
                'second_stop_index':800,     
                
                'v_mode':0,    #0是速度规划模式，1是匀速模式
                
                'slow_v':1.0,
                
            }]
        )
    ])
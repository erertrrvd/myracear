from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
          #启动里程计节点
        Node(
            package='my_racecar',
            executable='map_recieve_node',
            name='map_recieve',
            output='log',
        ),
    
        Node(
            package='my_racecar',  
            executable='get_local_path',  # 可执行文件名称
            name='get_local_path',
            output='screen',
            parameters=[{
                
                'front_dist': 0.5,
                'ab_flag':2,
                'concat_dy':0.0,  #左正，右负
                'stop_point':4,  #倒数第三个点
                'slow_point':20, 
                
            }]
            ),
            
        Node(
            package='my_racecar',
            executable='pure_track_test',
            name='pure_track_test',
            output='screen',
            parameters=[{
                
                'linear_x': 0.6,
                'd_min':0.5,
                'k_v':0.5,
                'kp_yaw': 1.0,
                'kd_yaw': 0.0,
                'ff_kp':1.0,
                
                'theta_min':-25.0,
                'theta_max':25.0,
                
                'light_mode': 0,   #模式0是基于yolo自动停车
                'first_stop_index': 180,
                'second_stop_index':510,     
                'traffic_light_area':1500,
                'v_mode':1,    #0是速度规划模式，1是匀速模式
                
                'slow_v':1.0,
                
            }]
        )
    ])
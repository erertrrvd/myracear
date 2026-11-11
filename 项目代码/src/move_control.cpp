//模式1：根据中点使用pid进行控制（包括直线，s，90度圆弧）
//模式2：黄色圆弧模式，固定角度和速度
//模式3：直角拐弯模式，固定角度和速度
//模式4：红绿灯停车模式，速度和角度全部清零
//模式5：进入第二圈
//模式6：ab停车前半段
//模式7：ab停车后半段
//模式8：最终停车模式
//这个是省赛的代码



#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <csignal>    // 用于 signal() 和 SIGINT
#include <cstdlib>    // 用于 std::exit()


//横向控制的pid类，一个简单的基于前视点横向误差实现的横向运动控制
class PID
{
    public:
        /***********pid参数*******/
        //比例系数
        float kp = 0;
        float ki = 0;
        float kd = 0;
        //积分限幅
        float integral_limit_min = 0.0;
        float integral_limit_max = 0.0;
        //输出限幅
        float output_limit_min = 0.0;
        float output_limit_max = 0.0;
       /***********中间参数*******/
        float last_error = 0.0;//用于计算微分项
        float integral = 0.0; //用于计算积分项
        //pid函数，输入误差，输出控制量
        float pid(float error)
        {
            //积分累计
            integral += error;
            //积分限幅
            if(integral > integral_limit_max)
            {
                integral = integral_limit_max;
            }
            if(integral < integral_limit_min)
            {
                integral = integral_limit_min;
            }
            float control = kp * error + ki * integral + kd * (error - last_error);
            last_error = error;
            //输出限幅
            if(control > output_limit_max)
            {
                control = output_limit_max;
            }
            if(control < output_limit_min)
            {
                control = output_limit_min;
            }
            return control;
        }
};












class MoveControl : public rclcpp::Node
{

public:

    /********************话题相关**********************/
    // 订阅器
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr flag_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr middle_point_sub_;
    
    // 发布器
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;


    /********************状态相关**********************/
    //当前状态
    int current_flag = 1;//默认为1，中点跟踪模式
    int circle = 1;//圈数



     /********************PID相关**********************/
    //第一圈和第二圈各自的pid
    PID pid1;//第一圈的pid
    PID pid2;//第二圈的pid
   
    //舵机终中值
    float angular_z = 88;

    //第一圈默认参数
    float linear_x1 = 0.6;
    //pid参数
    float pid_kp1 = 180;
    float pid_ki1 = 0.0;
    float pid_kd1 = 200;

    float pid_integral_limit_min1 = 0.0;
    float pid_integral_limit_max1 = 0.0;
    
    float pid_output_limit_min1 = -80;
    float pid_output_limit_max1 = 80;

    //第二圈默认参数
    float linear_x2 = 0.6;
    //pid参数
    float pid_kp2 = 150;
    float pid_ki2 = 0.0;
    float pid_kd2 = 200;

    float pid_integral_limit_min2 = 0.0;
    float pid_integral_limit_max2 = 0.0;
    
    float pid_output_limit_min2 = -80;
    float pid_output_limit_max2 = 80;

 /********************特殊模式相关**********************/
    //停止时的速度和角度,包含红绿灯和最终停车
     float stop_linear_x = 0.0;

     //黄色圆弧的角度和速度，第一圈和第二圈
     float yellow_angular_z1 = 0;
     float yellow_linear_x1 = 0.6;
     float yellow_angular_z2 = 0;
     float yellow_linear_x2 = 1.2;

    //直角拐弯的角度和速度
     float direct_angular_z1 =  0;
     float direct_linear_x1 = 0.6;
     float direct_angular_z2 = 0;
     float direct_linear_x2 = 1.2;


    //ab停车标识
    int ab_flag = 1; //1是停a，2是停b

    //a停车的参数（第一段和第二段）
     float a_linear_x1 = 0.6;
     float a_angular_z1 = 0;
     float a_linear_x2 = 0.6;
     float a_angular_z2 = 0;

    //b停车的参数（第一段和第二段）
     float b_linear_x1 = 0.6;
     float b_angular_z1 = 0;
     float b_linear_x2 = 0.6;
     float b_angular_z2 = 0;

public:
    MoveControl() : Node("move_control")
    {
        //参数声明
        // 两圈通用
        this->declare_parameter("angular_z", angular_z);//舵机中值
        this->declare_parameter("stop_linear_x", stop_linear_x);

        //ab停车
        this->declare_parameter("ab_flag", ab_flag);
        this->declare_parameter("a_linear_x1", a_linear_x1);
        this->declare_parameter("a_angular_z1", a_angular_z1);
        this->declare_parameter("a_linear_x2", a_linear_x2);
        this->declare_parameter("a_angular_z2", a_angular_z2);
        this->declare_parameter("b_linear_x1", b_linear_x1);
        this->declare_parameter("b_angular_z1", b_angular_z1);
        this->declare_parameter("b_linear_x2", b_linear_x2);
        this->declare_parameter("b_angular_z2", b_angular_z2);


        // 第一圈
        this->declare_parameter("linear_x1", linear_x1);
        this->declare_parameter("pid_kp1", pid_kp1);
        this->declare_parameter("pid_ki1", pid_ki1);
        this->declare_parameter("pid_kd1", pid_kd1);
        this->declare_parameter("pid_integral_limit_min1", pid_integral_limit_min1);
        this->declare_parameter("pid_integral_limit_max1", pid_integral_limit_max1);
        this->declare_parameter("pid_output_limit_min1", pid_output_limit_min1);
        this->declare_parameter("pid_output_limit_max1", pid_output_limit_max1);

        this->declare_parameter("yellow_angular_z1", yellow_angular_z1);
        this->declare_parameter("yellow_linear_x1", yellow_linear_x1);
        this->declare_parameter("direct_angular_z1", direct_angular_z1);
        this->declare_parameter("direct_linear_x1", direct_linear_x1);

    
        //第二圈
        this->declare_parameter("linear_x2", linear_x2);
        this->declare_parameter("pid_kp2", pid_kp2);
        this->declare_parameter("pid_ki2", pid_ki2);
        this->declare_parameter("pid_kd2", pid_kd2);
        this->declare_parameter("pid_integral_limit_min2", pid_integral_limit_min2);
        this->declare_parameter("pid_integral_limit_max2", pid_integral_limit_max2);
        this->declare_parameter("pid_output_limit_min2", pid_output_limit_min2);
        this->declare_parameter("pid_output_limit_max2", pid_output_limit_max2);

        this->declare_parameter("yellow_angular_z2", yellow_angular_z2);
        this->declare_parameter("yellow_linear_x2", yellow_linear_x2);
        this->declare_parameter("direct_angular_z2", direct_angular_z2);
        this->declare_parameter("direct_linear_x2", direct_linear_x2);


        //获取参数
        this->get_parameter("angular_z", angular_z);
        this->get_parameter("stop_linear_x", stop_linear_x);
        //ab停车
        this->get_parameter("ab_flag", ab_flag);
        this->get_parameter("a_linear_x1", a_linear_x1);
        this->get_parameter("a_angular_z1", a_angular_z1);
        this->get_parameter("a_linear_x2", a_linear_x2);
        this->get_parameter("a_angular_z2", a_angular_z2);
        this->get_parameter("b_linear_x1", b_linear_x1);
        this->get_parameter("b_angular_z1", b_angular_z1);
        this->get_parameter("b_linear_x2", b_linear_x2);
        this->get_parameter("b_angular_z2", b_angular_z2);

        this->get_parameter("linear_x1", linear_x1);
        this->get_parameter("pid_kp1", pid_kp1);
        this->get_parameter("pid_ki1", pid_ki1);
        this->get_parameter("pid_kd1", pid_kd1);
        this->get_parameter("pid_integral_limit_min1", pid_integral_limit_min1);
        this->get_parameter("pid_integral_limit_max1", pid_integral_limit_max1);
        this->get_parameter("pid_output_limit_min1", pid_output_limit_min1);
        this->get_parameter("pid_output_limit_max1", pid_output_limit_max1);

        this->get_parameter("yellow_angular_z1", yellow_angular_z1);
        this->get_parameter("yellow_linear_x1", yellow_linear_x1);
        this->get_parameter("direct_angular_z1", direct_angular_z1);
        this->get_parameter("direct_linear_x1", direct_linear_x1);

        this->get_parameter("linear_x2", linear_x2);
        this->get_parameter("pid_kp2", pid_kp2);
        this->get_parameter("pid_ki2", pid_ki2);
        this->get_parameter("pid_kd2", pid_kd2);
        this->get_parameter("pid_integral_limit_min2", pid_integral_limit_min2);
        this->get_parameter("pid_integral_limit_max2", pid_integral_limit_max2);
        this->get_parameter("pid_output_limit_min2", pid_output_limit_min2);
        this->get_parameter("pid_output_limit_max2", pid_output_limit_max2);

        this->get_parameter("yellow_angular_z2", yellow_angular_z2);
        this->get_parameter("yellow_linear_x2", yellow_linear_x2);
        this->get_parameter("direct_angular_z2", direct_angular_z2);
        this->get_parameter("direct_linear_x2", direct_linear_x2);


        //pid参数
        pid1.kp = pid_kp1;
        pid1.ki = pid_ki1;
        pid1.kd = pid_kd1;
        pid1.integral_limit_min = pid_integral_limit_min1;
        pid1.integral_limit_max = pid_integral_limit_max1;
        pid1.output_limit_min = pid_output_limit_min1;
        pid1.output_limit_max = pid_output_limit_max1;
        
        // 第二圈PID参数设置
        pid2.kp = pid_kp2;
        pid2.ki = pid_ki2;
        pid2.kd = pid_kd2;
        pid2.integral_limit_min = pid_integral_limit_min2;
        pid2.integral_limit_max = pid_integral_limit_max2;
        pid2.output_limit_min = pid_output_limit_min2;
        pid2.output_limit_max = pid_output_limit_max2;



        // 订阅flag话题
        flag_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "flag",
            10,
            std::bind(&MoveControl::flag_callback, this, std::placeholders::_1));

        // 订阅middle_point话题
        middle_point_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "middle_point",
            10,
            std::bind(&MoveControl::middle_point_callback, this, std::placeholders::_1));

        // 发布小车控制指令
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Move control node started");
    }



    //写死的控制
    void flag_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if(msg->data == 5)
        {
            circle = 2;//进入到第二圈,进入第二圈不需要改变巡线模式，加这个是允许第一圈和第二圈有不同的参数
        }
        else
        {
            current_flag = msg->data; //更新当前模式

            if(circle == 1)
            {
                if(current_flag == 2)//2：黄色圆弧模式
                {
                publish_cmd_vel(yellow_linear_x1, yellow_angular_z1);
                }
                else if(current_flag == 3)//3:直角拐弯模式
                {
                    publish_cmd_vel(direct_linear_x1, direct_angular_z1);
                }  
                else if(current_flag == 4)//4.红绿灯模式
                {
                publish_cmd_vel(stop_linear_x, angular_z);
                }
            }
            else if(circle == 2)
            {
                if(current_flag == 2)//2：黄色圆弧模式
                {
                    publish_cmd_vel(yellow_linear_x2, yellow_angular_z2);
                }
                else if(current_flag == 3)//3：直角拐弯模式
                {
                    publish_cmd_vel(direct_linear_x2, direct_angular_z2);
                }
                else if(current_flag == 4)//4:红绿灯停车模式
                {
                    publish_cmd_vel(stop_linear_x, angular_z);
                }
                else if(current_flag == 6)//6.ab停车模式第一段
                {
                    if(ab_flag == 1)
                    {
                        publish_cmd_vel(a_linear_x1, a_angular_z1);
                    }
                    else if(ab_flag == 2)
                    {

                        publish_cmd_vel(b_linear_x1, b_angular_z1);
                    }
                }
                else if(current_flag == 7)//7.ab停车第二段
                {
                    if(ab_flag == 1)
                    {
                        publish_cmd_vel(a_linear_x2, a_angular_z2);
                    }
                    else if(ab_flag == 2)
                    {

                        publish_cmd_vel(b_linear_x2, b_angular_z2);
                    }

                }
                else if(current_flag == 8)//8:停止模式
                {
                    publish_cmd_vel(stop_linear_x, angular_z);//清零当前的速度，车辆回正
                }



            }


        }
        RCLCPP_INFO(this->get_logger(), "Received flag: %d", current_flag);
    }

    void middle_point_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
       //在这里写根据不同标志进行不同控制策略的代码
       if(circle == 1)
        {
            //第一圈
            if(current_flag == 1)//1.正常寻激光雷达，直线或s
            {
                 // 计算误差
                float error = msg->data;
                float control = pid1.pid(error);
                publish_cmd_vel(linear_x1,control);//恒定速度
            }


            
        }
        else if(circle == 2)
        {
            //第二圈
             if(current_flag == 1)//1.正常寻激光雷达，直线或s
            {
                   // 计算误差
                float error = msg->data;
                float control = pid2.pid(error);
                publish_cmd_vel(linear_x2,control); 
            }
            
        }



        
    }

 
    void publish_cmd_vel(double linear_x, double angular_z)
    {
        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = linear_x;
        cmd_msg.angular.z = angular_z;
        cmd_pub_->publish(cmd_msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveControl>();
    
    // 先创建停止消息
    auto stop_msg = geometry_msgs::msg::Twist();
    stop_msg.linear.x = 0;  
    stop_msg.angular.z = 0;   
    
    // 运行节点
    rclcpp::spin(node);
    
    // spin返回后（Ctrl+C后）发布停止命令
    node->cmd_pub_->publish(stop_msg);
    RCLCPP_INFO(node->get_logger(), "Node stopped, sending stop command");
    
    // 短暂延迟确保消息发布
    rclcpp::sleep_for(std::chrono::milliseconds(50));
    
    rclcpp::shutdown();
    return 0;
}






//运动控制测试节点
//首先尝试通过使用pid进行寻找直线
//对于直线而言，默认的舵机角度就是中轴，是相对比较简单的控制
//主要是尝试写一个pid横向控制的类，进行运动控制
//至于纵向运动控制，可以先不着急
//这个节点订阅middle_point，得到中点的坐标，输出twist话题，进行运动控制
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>




//横向控制的pid类
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













class MoveControlTest : public rclcpp::Node
{
    private:
    // 订阅器
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr middle_point_sub_;
    
    // 发布器
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    PID pid;
 
    //默认速度和角度
    float linear_x = 1520;
    float angular_z = 88;
    //pid参数
    float pid_kp = 20;
    float pid_ki = 0.0;
    float pid_kd = 0.0;

    float pid_integral_limit_min = 0.0;
    float pid_integral_limit_max = 0.0;
    
    float pid_output_limit_min = -5;
    float pid_output_limit_max = 5;

public:
    MoveControlTest() : Node("move_control_test")
    {
        // 从参数服务器获取pid参数
        this->declare_parameter("linear_x", linear_x);
        this->declare_parameter("angular_z", angular_z);
        this->declare_parameter("pid_kp", pid_kp);
        this->declare_parameter("pid_ki", pid_ki);
        this->declare_parameter("pid_kd", pid_kd);

        this->declare_parameter("pid_integral_limit_min", pid_integral_limit_min);
        this->declare_parameter("pid_integral_limit_max", pid_integral_limit_max);
        this->declare_parameter("pid_output_limit_min", pid_output_limit_min);
        this->declare_parameter("pid_output_limit_max", pid_output_limit_max);
        // 从参数服务器获取pid参数
        this->get_parameter("linear_x", linear_x);
        this->get_parameter("angular_z", angular_z);
        this->get_parameter("pid_kp", pid_kp);
        this->get_parameter("pid_ki", pid_ki);
        this->get_parameter("pid_kd", pid_kd);

        this->get_parameter("pid_integral_limit_min", pid_integral_limit_min);
        this->get_parameter("pid_integral_limit_max", pid_integral_limit_max);
        this->get_parameter("pid_output_limit_min", pid_output_limit_min);
        this->get_parameter("pid_output_limit_max", pid_output_limit_max);
        

        // 初始化pid参数
        pid.kp = pid_kp;
        pid.ki = pid_ki;
        pid.kd = pid_kd;

        pid.integral_limit_min = pid_integral_limit_min;
        pid.integral_limit_max = pid_integral_limit_max;
        
        pid.output_limit_min = pid_output_limit_min;
        pid.output_limit_max = pid_output_limit_max;

        // 订阅middle_point话题
        middle_point_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "middle_point",
            10,
            std::bind(&MoveControlTest::middle_point_callback, this, std::placeholders::_1));

        // 发布小车控制指令
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("car_cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Move control node started");
    }




    void middle_point_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // 计算误差
        float error = msg->data;

        // 计算控制量
        float control = pid.pid(error);
        
        
         RCLCPP_INFO(this->get_logger(), "error: %.2f, control: %.2f ,argular.z: %.2f", 
                error, control, angular_z + control);

        // 发布控制指令
        publish_cmd_vel(linear_x, angular_z + control);

    }



        
    

    void publish_cmd_vel(double linear_x, double angular_z)
    {
        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = linear_x;
        cmd_msg.angular.z = angular_z;
        cmd_pub_->publish(cmd_msg);
    }
    
    
     ~MoveControlTest()
    {
        // 节点销毁时自动停车
        publish_cmd_vel(1500, 88);
        RCLCPP_INFO(this->get_logger(), "Car stopped safely");
    }


};






int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveControlTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    
    
    RCLCPP_INFO(node->get_logger(), "Node shutdown, car stopped");
    
    
    return 0;
}














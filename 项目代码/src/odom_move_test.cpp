//基于里程计进行导航的测试
//首先测试基于里程计走直线的效果，只需要依赖dy就可以
//还可以依赖x走指定的距离
//依旧使用纯追踪加pid的控制策略

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "math.h"



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




class odom_move_test : public rclcpp::Node
{

private:

    /*******************话题相关***************/
    // 订阅器，订阅里程计话题
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;    
    // 发布器,发布目标速度
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;


    /********************pid相关**************/
    //横向pid控制器
    PID pid;

    //纵向速度
    float linear_x = 1.0;

    //横向pid参数
    float pid_kp = 0.0;
    float pid_ki = 0.0;
    float pid_kd = 0.0;

    float pid_integral_limit_min = 0.0;
    float pid_integral_limit_max = 0.0;

    float pid_output_limit_min = 0.0;
    float pid_output_limit_max = 0.0;


   /********************纯追踪相关**********************/

    float pure_L=0.325; //前后轮轴间距
    float pure_dx = 1.0; //前视点x轴距离（前视点-原点）
    float pure_theta = 0.0; //当前角度
    float pure_theta_max = 40; //最大角度，40度
    float pure_theta_min = -40; //最小角度，-40度

     







public:
    odom_move_test() : Node("odom_move_test")
    {
        //参数声明
        this->declare_parameter("linear_x", linear_x);

        this->declare_parameter("pid_kp", pid_kp);
        this->declare_parameter("pid_ki", pid_ki);
        this->declare_parameter("pid_kd", pid_kd);
        this->declare_parameter("pid_integral_limit_min", pid_integral_limit_min);
        this->declare_parameter("pid_integral_limit_max", pid_integral_limit_max);
        this->declare_parameter("pid_output_limit_min", pid_output_limit_min);
        this->declare_parameter("pid_output_limit_max", pid_output_limit_max);

        
        this->declare_parameter("pure_L", pure_L);//前后轮轴间距
        this->declare_parameter("pure_dx", pure_dx);//前视点的距离
        this->declare_parameter("pure_theta_max", pure_theta_max);//最大角度，
        this->declare_parameter("pure_theta_min", pure_theta_min);//最小角度，


        //得到参数
        this->get_parameter("linear_x", linear_x);

        this->get_parameter("pid_kp", pid_kp);
        this->get_parameter("pid_ki", pid_ki);
        this->get_parameter("pid_kd", pid_kd);
        this->get_parameter("pid_integral_limit_min", pid_integral_limit_min);
        this->get_parameter("pid_integral_limit_max", pid_integral_limit_max);
        this->get_parameter("pid_output_limit_min", pid_output_limit_min);
        this->get_parameter("pid_output_limit_max", pid_output_limit_max);

        this->get_parameter("pure_L", pure_L);//前后轮轴间距
        this->get_parameter("pure_dx", pure_dx);//前视点的距离
        this->get_parameter("pure_theta_max", pure_theta_max);//最大角度，
        this->get_parameter("pure_theta_min", pure_theta_min);//最小角度，


        pid.kp = pid_kp;
        pid.ki = pid_ki;
        pid.kd = pid_kd;
        pid.integral_limit_min = pid_integral_limit_min;
        pid.integral_limit_max = pid_integral_limit_max;
        pid.output_limit_min = pid_output_limit_min;
        pid.output_limit_max = pid_output_limit_max;



         //订阅里程计话题    
         odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/encoder_imu_odom",
            10,
            std::bind(&odom_move_test::odom_callback, this, std::placeholders::_1));

        //发布目标速度话题
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    }

    /*回调函数*/
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        //在里程计的回调函数中进行运动控制，现在先测试一个简单的走直线的效果
        float current_x = msg->pose.pose.position.x;
        float current_y = msg->pose.pose.position.y;


        float purpose_x = current_x + pure_dx;
        float purpose_y = 0;//走直线，横向目标为0

        pure_theta = Pure_Tracking(purpose_x - current_x, purpose_y - current_y);
        float pid_theta = pid.pid( purpose_y - current_y);
        
        RCLCPP_INFO(this->get_logger(), "dx:%.2f  dy:%.2f  pure_theta:%.2f  pid_theta:%.2f out_put:%.2f\n",purpose_x - current_x, purpose_y - current_y,pure_theta,pid_theta,pure_theta + pid_theta);
        
        //基于纯追踪走直线的代码
        
        publish_cmd_vel(linear_x, pure_theta + pid_theta);


    }



    //输入(purpose_x - current_x)和(purpose_y - current_y),汽车坐标系下
    float Pure_Tracking(float dx, float dy)
    {
        float theta ;
        float L = pure_L;
        
        theta = atan2(2*L*dy,dy*dy+dx*dx) * 180.0 / M_PI;//计算理论角度
        //角度限幅
        if(theta > pure_theta_max)
        {
            theta = pure_theta_max;
        }
        if(theta < pure_theta_min)
        {
            theta = pure_theta_min;
        }

        return theta;        
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
    auto node = std::make_shared<odom_move_test>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}





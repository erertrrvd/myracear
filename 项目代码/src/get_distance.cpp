#include <rclcpp/rclcpp.hpp>  //rclcpp标准
#include <nav_msgs/msg/odometry.hpp> //odometry的消息（得到编码器给的小车合速度大小）
#include <std_msgs/msg/float32.hpp>  //float32的消息
#include "math.h"

class GetDistance : public rclcpp::Node
{
public:

    //校准系数
    double distance_k = 1; // 校准系数，根据实际情况调整

    GetDistance() : Node("get_distance")
    {
        
        //校准系数
        this->declare_parameter("distance_k", distance_k);
        this->get_parameter("distance_k", distance_k);


        distance= 0.0;//路程

        last_linear_x = 0.0;//上一次的线速度
        last_time = this->now();

        first_message = true;
        
        // 创建订阅者，订阅encoder话题
         encoder_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "encoder", 10,
            std::bind(&GetDistance::encoder_callback, this, std::placeholders::_1));
        
        // 创建发布者，发布距离话题
        distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("distance", 10);
        
        RCLCPP_INFO(this->get_logger(), "GetDistance node started");
    }

private:
    void encoder_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        //得到当前时间戳
        auto current_time = this->now();
        
        // 如果是第一条消息，只记录初始值
        if (first_message) 
        {
            last_linear_x = fabs(msg->twist.twist.linear.x);
            last_time = current_time;
            first_message = false;
            return;
        }
        
        // 计算时间差
        double dt = (current_time - last_time).seconds();
        
        // 检查时间差是否异常
        if (dt <= 0.0 || dt > 1.0)
         {  
            RCLCPP_WARN(this->get_logger(), "Abnormal time interval: %.3f s, skipping", dt);
            last_time = current_time;
            last_linear_x = fabs(msg->twist.twist.linear.x);
            return;
        }


        double average_velocity = (last_linear_x + fabs(msg->twist.twist.linear.x)) / 2.0;
        distance += average_velocity * dt;
        
        // 发布距离
        auto distance_msg = std_msgs::msg::Float32();
        distance_msg.data = distance*distance_k;
        distance_pub_->publish(distance_msg );
        
        // 更新上一次的值
        last_linear_x = fabs(msg->twist.twist.linear.x);
        last_time = current_time;
        
      
    }
    
    // 发布者和订阅者
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr encoder_sub_; 
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
    
    double distance;  // 当前累计距离
    double last_linear_x;     // 上一次的线速度
    rclcpp::Time last_time;   // 上一次的时间戳

    bool first_message;       // 是否是第一条消息

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GetDistance>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
//模式1：根据中点使用pid进行控制（包括直线，s，90度圆弧）
//模式2：黄色圆弧模式，固定角度和速度
//模式3：直角拐弯模式，固定角度和速度
//模式4：红绿灯停车模式，速度和角度全部清零
//模式5：进入第二圈
//模式6：ab停车前半段
//模式7：ab停车后半段
//模式8：最终停车模式


//要对这个的代码进行更新，订阅里程计，使用位移而不是路程进行触发条件的判断


#include <rclcpp/rclcpp.hpp>  //rclcpp标准
#include <std_msgs/msg/float32.hpp>  //float32的消息，订阅distance话题
#include <std_msgs/msg/int32.hpp>  //int32的消息，发布flag话题
#include <nav_msgs/msg/odometry.hpp> //里程计消息
#include <math.h> //数学库


class GetFlag : public rclcpp::Node
{

private:
    // 黄色圆弧模式的距离范围
    float yellow_start_x; //开始的距离,x坐标
    float yellow_start_y; //开始的距离,y坐标
    float yellow_dx = 0.10;//判读阈值
    float yellow_dy = 1.00;//判读阈值
    float yellow_length;//持续的路程
    float yellow_start_distance ;
    int yellow_flag = 0;//黄色圆弧模式是否触发过了

     // 直接拐弯模式的距离范围
    float direct_turn_start_x;//直角拐弯触发的起始点,x坐标
    float direct_turn_start_y;//直角拐弯触发的起始点,y坐标
    float direct_turn_dx = 1.00;//判读阈值
    float direct_turn_dy = 0.10;//判读阈值
    float direct_turn_length;//持续的路程
    float direct_turn_start_distance ;
    int direct_turn_flag = 0;//直接拐弯模式是否触发过了

    //回到初始点的距离,也是一圈的距离
    float return_origin_x = 0;//回到初始点的距离,x坐标
    float return_origin_y = 0;//回到初始点的距离,y坐标
    float return_origin_dx = 0.10;//判读阈值
    float return_origin_dy = 1.00;//判读阈值
    
    float fix_dx = 0;
    float fix_dy = 0;
    

    //ab停车模式的距离范围
    float ab_stop_start_x;//ab停车触发的起始点,x坐标
    float ab_stop_start_y;//ab停车触发的起始点,y坐标
    float ab_stop_dx = 0.1;//判读阈值
    float ab_stop_dy = 1.00;//判读阈值
    float ab_stop_length1;//持续的路程
    float ab_stop_length2;//持续的路程
    float ab_stop_start_distance ;
    int ab_stop_flag = 0;//ab停车模式是否触发过了


    //当前圈数
    int current_circle = 1;//开始是第一圈，回到原点后会自动进如第二圈
    float current_distance = 0.0;//当前距离

    //红绿灯逻辑
    int traffic_light = 0;
    int traffic_light_flag = 0;//红绿灯模式是否触发过了
    int traffic_light_current_flag=1;//记录红绿灯停车前的flag模式，之后恢复
    rclcpp::TimerBase::SharedPtr traffic_light_timer_; // 红绿灯定时器
    int traffic_light_area = 1500;
    

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dis_sub_;//路程
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr flag_pub_;//发布flag话题
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr light_sub_;//红绿灯订阅者
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;//里程计订阅者

  



public:
    GetFlag() : Node("get_flag")
    {
         // 声明参数
        this->declare_parameter<float>("yellow_start_x", 2.5);
        this->declare_parameter<float>("yellow_start_y", 2.5);
        this->declare_parameter<float>("yellow_length", 6.0);

        this->declare_parameter<float>("direct_turn_start_x", 18.5);
        this->declare_parameter<float>("direct_turn_start_y", 18.5);
        this->declare_parameter<float>("direct_turn_length", 20.0);
        this->declare_parameter<float>("direct_turn_dx", 0.75);
        this->declare_parameter<float>("direct_turn_dy", 0.05);

        this->declare_parameter<float>("ab_stop_start_x", 30.0);
        this->declare_parameter<float>("ab_stop_start_y", 30.0);
        this->declare_parameter<float>("ab_stop_length1", 34.0);
        this->declare_parameter<float>("ab_stop_length2", 38.0);
        
        this->declare_parameter<int>("traffic_light_area", 1500);
        
        this->declare_parameter<float>("fix_dx", 0.0);
        this->declare_parameter<float>("fix_dy", 0.0);
        // 获取参数
        this->get_parameter("yellow_start_x", yellow_start_x);
        this->get_parameter("yellow_start_y", yellow_start_y);
        this->get_parameter("yellow_length", yellow_length);

        this->get_parameter("direct_turn_start_x", direct_turn_start_x);
        this->get_parameter("direct_turn_start_y", direct_turn_start_y);
        this->get_parameter("direct_turn_length", direct_turn_length);
        this->get_parameter("direct_turn_dx", direct_turn_dx);
        this->get_parameter("direct_turn_dy", direct_turn_dy);

        this->get_parameter("ab_stop_start_x", ab_stop_start_x);
        this->get_parameter("ab_stop_start_y", ab_stop_start_y);
        this->get_parameter("ab_stop_length1", ab_stop_length1);
        this->get_parameter("ab_stop_length2", ab_stop_length2);
        
        this->get_parameter("traffic_light_area", traffic_light_area);
         
        this->get_parameter("fix_dx", fix_dx);
        this->get_parameter("fix_dy", fix_dy);
        
        
        //订阅距离话题
         dis_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "distance",
            10,
            std::bind(&GetFlag::dis_callback, this, std::placeholders::_1));

        //订阅红绿灯话题
        light_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "traffic_light",
            10,
            std::bind(&GetFlag::light_callback, this, std::placeholders::_1));            

        //订阅里程计话题    
         odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/encoder_imu_odom",
            10,
            std::bind(&GetFlag::odom_callback, this, std::placeholders::_1));



        flag_pub_ = this->create_publisher<std_msgs::msg::Int32>("flag", 10);
            
        RCLCPP_INFO(this->get_logger(), "get_flag node started");
    }


     void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        int current_flag =1;//当前的flag模式
        auto flag_msg = std_msgs::msg::Int32();
        float current_x = msg->pose.pose.position.x;
        float current_y = msg->pose.pose.position.y;

         if(current_circle ==1 )
         {
            //第一圈,判断是否触发黄色圆弧模式
            if(fabs(current_x - yellow_start_x) <= yellow_dx && fabs(current_y - yellow_start_y) <= yellow_dy && yellow_flag == 0)
            {
                //触发黄色圆弧模式
                yellow_start_distance = current_distance;//记录开始的距离
                current_flag = 2;
                yellow_flag = 1;//黄色圆弧模式初始点被触发
                RCLCPP_INFO(this->get_logger(), "yellow arc mode started");
                flag_msg.data = current_flag;
                flag_pub_->publish(flag_msg);
            }

            if(fabs(current_x - direct_turn_start_x) <= direct_turn_dx && fabs(current_y - direct_turn_start_y) <= direct_turn_dy && direct_turn_flag == 0)
            {
                //触发直接转弯模式
                direct_turn_start_distance = current_distance;//记录开始的距离
                current_flag = 3;
                direct_turn_flag = 1;//直接拐弯模式初始点被触发
                RCLCPP_INFO(this->get_logger(), "direct turn mode started");
                flag_msg.data = current_flag;
                flag_pub_->publish(flag_msg);
            }


            //判断是否触发红绿灯模式
            if(traffic_light == 1 )
            {

                traffic_light_current_flag = current_flag;//记录红绿灯停车前的flag模式，之后恢复
                current_flag = 4;
                traffic_light = 0;//重置红绿灯状态
                RCLCPP_INFO(this->get_logger(), "traffic light");
                flag_msg.data = current_flag;
                flag_pub_->publish(flag_msg);

                //创建3秒定时器
                traffic_light_timer_ = this->create_wall_timer(
                std::chrono::seconds(3),
                std::bind(&GetFlag::traffic_light_timer_callback, this));
            }

            //判断是否触发黄色圆弧模式结束点
            if(yellow_flag == 1 && current_distance - yellow_start_distance >= yellow_length)
            {
                current_flag = 1;
                yellow_flag = 2;//黄色圆弧模式结束点被触发
                RCLCPP_INFO(this->get_logger(), "yellow arc mode ended");
                flag_msg.data = current_flag;
                flag_pub_->publish(flag_msg);
            }

            //判断是否触发直接拐弯模式结束点
             if(direct_turn_flag == 1 && current_distance - direct_turn_start_distance >= direct_turn_length)
             {
                current_flag = 1;
                direct_turn_flag = 2;//直接拐弯模式结束点被触发
                RCLCPP_INFO(this->get_logger(), "direct turn mode ended");
                flag_msg.data = current_flag;
                flag_pub_->publish(flag_msg);
             }



            //判断是否进入第二圈
            if(fabs(current_x - return_origin_x) <= return_origin_dx && fabs(current_y - return_origin_y) <= return_origin_dy 
            && yellow_flag == 2 && direct_turn_flag == 2)
            {
                current_flag = 5;
                current_circle = 2;
                flag_msg.data = current_flag;
                flag_pub_->publish(flag_msg);

                //重置所有模式的触发标志
                RCLCPP_INFO(this->get_logger(), "return to origin, enter second circle");
                yellow_flag = 0;
                direct_turn_flag = 0;
                traffic_light_flag = 0;
            }


         }
         else if(current_circle ==2)
         {
             current_x += fix_dx;
             current_y += fix_dy;
            //第二圈
             //第一圈,判断是否触发黄色圆弧模式
            if(fabs(current_x - yellow_start_x) <= yellow_dx && fabs(current_y - yellow_start_y) <= yellow_dy && yellow_flag == 0)
            {
                //触发黄色圆弧模式
                yellow_start_distance = current_distance;//记录开始的距离
                current_flag = 2;
                yellow_flag = 1;//黄色圆弧模式初始点被触发
                RCLCPP_INFO(this->get_logger(), "yellow arc mode started");
                flag_msg.data = current_flag;
                flag_pub_->publish(flag_msg);
            }

            if(fabs(current_x - direct_turn_start_x) <= direct_turn_dx && fabs(current_y - direct_turn_start_y) <= direct_turn_dy && direct_turn_flag == 0)
            {
                //触发直接转弯模式
                direct_turn_start_distance = current_distance;//记录开始的距离
                current_flag = 3;
                direct_turn_flag = 1;//直接拐弯模式初始点被触发
                RCLCPP_INFO(this->get_logger(), "direct turn mode started");
                flag_msg.data = current_flag;
                flag_pub_->publish(flag_msg);
            }


            //判断是否触发红绿灯模式
            if(traffic_light == 1 )
            {

                traffic_light_current_flag = current_flag;//记录红绿灯停车前的flag模式，之后恢复
                current_flag = 4;
                traffic_light = 0;//重置红绿灯状态
                RCLCPP_INFO(this->get_logger(), "traffic light");
                flag_msg.data = current_flag;
                flag_pub_->publish(flag_msg);

                //创建3秒定时器
                traffic_light_timer_ = this->create_wall_timer(
                std::chrono::seconds(3),
                std::bind(&GetFlag::traffic_light_timer_callback, this));
            }


            //判断是否进入ab停车
             if(fabs(current_x - ab_stop_start_x) <= ab_stop_dx && fabs(current_y - ab_stop_start_y) <= ab_stop_dy && ab_stop_flag == 0)
             {
                //触发ab停车模式
                ab_stop_start_distance = current_distance;//记录开始的距离
                current_flag = 6;
                ab_stop_flag = 1;//ab停车模式初始点被触发
                RCLCPP_INFO(this->get_logger(), "ab stop mode1");
                flag_msg.data = current_flag;
                flag_pub_->publish(flag_msg);
             }

             if(ab_stop_flag == 1 && current_distance - ab_stop_start_distance >= ab_stop_length1)
             {
                current_flag = 7;
                ab_stop_flag = 2;
                RCLCPP_INFO(this->get_logger(), "ab stop mode2");
                flag_msg.data = current_flag;
                flag_pub_->publish(flag_msg);
             }

            if(ab_stop_flag == 2 && current_distance - ab_stop_start_distance >= ab_stop_length2+ab_stop_length1)
             {
                current_flag = 8;
                ab_stop_flag = 3;
                RCLCPP_INFO(this->get_logger(), "ab end");
                flag_msg.data = current_flag;
                flag_pub_->publish(flag_msg);
             }


            //判断是否触发黄色圆弧模式结束点
            if(yellow_flag == 1 && current_distance - yellow_start_distance >= yellow_length)
            {
                current_flag = 1;
                yellow_flag = 2;//黄色圆弧模式结束点被触发
                RCLCPP_INFO(this->get_logger(), "yellow arc mode ended");
                flag_msg.data = current_flag;
                flag_pub_->publish(flag_msg);
            }

            //判断是否触发直接拐弯模式结束点
             if(direct_turn_flag == 1 && current_distance - direct_turn_start_distance >= direct_turn_length)
             {
                current_flag = 1;
                direct_turn_flag = 2;//直接拐弯模式结束点被触发
                RCLCPP_INFO(this->get_logger(), "direct turn mode ended");
                flag_msg.data = current_flag;
                flag_pub_->publish(flag_msg);
             }



            


         }      
    }






      // 红绿灯回调函数
    void light_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if(msg->data >= traffic_light_area && traffic_light_flag == 0)
        {
            //只读取一次红绿灯信号
            traffic_light = 1;
            traffic_light_flag = 1; 
        }
    }

     void traffic_light_timer_callback()
    {
        // 关闭定时器
        traffic_light_timer_->cancel();
        
        // 发布正常行驶标志
        auto flag_msg = std_msgs::msg::Int32();
        flag_msg.data = traffic_light_current_flag;
        flag_pub_->publish(flag_msg);
        
        RCLCPP_INFO(this->get_logger(), "Traffic light wait completed, returning to normal mode");
    }

    //距离回调函数
    void dis_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        current_distance = msg->data;//读取当前距离    
    }
   
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);  
    auto node = std::make_shared<GetFlag>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}








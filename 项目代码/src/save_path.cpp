//这个代码订阅里程计话题，对里程计进行积分，得到实时距离，并每隔一段距离保存一次当前xyyaw坐标，并且通过markerarry可视化
//之后可能会进一步订阅flag话题，对于一些特殊点进行保存

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <std_msgs/msg/float32.hpp> //订阅路程消息
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  
#include <sensor_msgs/msg/laser_scan.hpp>

#include <fstream>
#include <string>
#include <memory>
#include <cmath>


class CSV_Saver
{
public:
    // 构造函数 - 只需要保存路径
    CSV_Saver(const std::string &save_path)
        : save_path_(save_path), first_save_(true)
    {
        // 构造函数可以留空，第一次保存时再创建文件
    }

    // 保存点的函数
    bool save_point(float x, float y, float yaw,float type)
    {
        std::ofstream file;
        
        if (first_save_)
        {
            // 第一次保存：创建文件并写入表头
            file.open(save_path_, std::ios::out);
            if (file.is_open())
            {
                file << "x,y,yaw,type\n"; // 写入表头
                first_save_ = false;
                RCLCPP_INFO(rclcpp::get_logger("CSV_Saver"), 
                           "创建文件: %s", save_path_.c_str());
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("CSV_Saver"), 
                            "无法创建文件: %s", save_path_.c_str());
                return false;
            }
        }
        else
        {
            // 非第一次保存：追加模式
            file.open(save_path_, std::ios::app);
            if (!file.is_open())
            {
                RCLCPP_ERROR(rclcpp::get_logger("CSV_Saver"), 
                            "无法打开文件: %s", save_path_.c_str());
                return false;
            }
        }

        // 写入数据
        file << x << "," << y << "," << yaw << "," << type << "\n";
        file.close();

        return true;
    }

    // 获取保存路径
    std::string get_save_path() const
    {
        return save_path_;
    }

    // 重置（可以重新开始记录）
    void reset()
    {
        first_save_ = true;
    }

private:
    std::string save_path_;  // 保存路径
    bool first_save_;        // 是否第一次保存
};






class SavePath : public rclcpp::Node
{
    private:
        /********话题相关*******/
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dis_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;  
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
        int marker_id_ = 0;  // 点的id


        /*********参数相关*********/
        float save_dx = 0.1; //距离间隔

        float x = 0.0; //当前x坐标
        float y = 0.0; //当前y坐标
        float yaw = 0.0; //当前yaw角度(弧度制)

        float last_dis = 0.0; //上一次保存的距离

        //csv文件相关
        std::string csv_path = "/home/racecar/src/my_racecar/include/path.csv";
        CSV_Saver csv_saver; // 创建 CSV_Saver 实例

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        bool init_scan_saved_ = false;
        std::string init_scan_path_ = "/home/racecar/src/my_racecar/include/init_scan.csv";

    public:
        SavePath(): Node("save_path"), csv_saver(csv_path)
        {

            //参数声明
            this->declare_parameter("save_dx", save_dx);
            //获取参数值
            this->get_parameter("save_dx", save_dx);




            //订阅距离话题
             dis_sub_ = this->create_subscription<std_msgs::msg::Float32>(
                "distance",
                10,
                std::bind(&SavePath::dis_callback, this, std::placeholders::_1));

            //订阅里程计话题 
             odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/encoder_imu_odom",
                10,
                std::bind(&SavePath::odom_callback, this, std::placeholders::_1));
            
            //发布marker话题
             marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "path_markers", 10);

            //订阅雷达话题
             scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
             "/scan",
             10,
             std::bind(&SavePath::scan_callback, this, std::placeholders::_1));;


        }



        double quaternionToYaw(double x, double y, double z, double w)
        {
            // 使用公式：yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
            double siny_cosp = 2.0 * (w * z + x * y);
            double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
            return std::atan2(siny_cosp, cosy_cosp);
        }

       //里程计回调函数
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            //获取当前xyyaw坐标
            x = msg->pose.pose.position.x;
            y = msg->pose.pose.position.y;
            
            // 使用自定义函数获取偏航角
            auto& orientation = msg->pose.pose.orientation;
            yaw = quaternionToYaw(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w
            );
                       
        }

        //距离回调函数
        void dis_callback(const std_msgs::msg::Float32::SharedPtr msg)
        {
            //获取当前距离
            float dis = msg->data;

            //判断是否需要保存当前坐标
            if(dis - last_dis >= save_dx)
            {
                //进行保存当前坐标
                csv_saver.save_point(x, y, yaw,1);

                //显示当前坐标
                show_point(x, y);

                //保存当前坐标
                last_dis = dis;
            }

        }

        void show_point(float x, float y)
        {
            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.ns = "path";
            marker.id = marker_id_++;  // ID递增
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = 0.0;
            
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            
            marker.color.r = 0.0;
            marker.color.g = 1.0;//绿色路径点
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            
            // 永久显示
            marker.lifetime = rclcpp::Duration::from_seconds(0);
            
            // 直接发布这个点
            auto marker_array = visualization_msgs::msg::MarkerArray();
            marker_array.markers.push_back(marker);
            marker_pub_->publish(marker_array);
            
            RCLCPP_INFO(this->get_logger(), "显示点 %d: (%.2f, %.2f)", marker.id, x, y);
        }


        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            if (init_scan_saved_)
                return;

            std::ofstream file(init_scan_path_);
            if (!file.is_open())
            {
                RCLCPP_ERROR(this->get_logger(), "无法创建文件: %s", init_scan_path_.c_str());
                return;
            }

            file << "index,angle,range,x,y\n";

            int valid_count = 0;
            for (size_t i = 0; i < msg->ranges.size(); i++)
            {
                float range = msg->ranges[i];
                if (std::isnan(range) || std::isinf(range))
                    continue;
                if (range < msg->range_min || range > msg->range_max)
                    continue;

                float angle = msg->angle_min + i * msg->angle_increment;
                float x = range * std::cos(angle);
                float y = range * std::sin(angle);

                file << i << "," << angle << "," << range << "," << x << "," << y << "\n";
                valid_count++;
            }

            file.close();
            init_scan_saved_ = true;

            RCLCPP_INFO(this->get_logger(), "初始雷达帧已保存，%d 个有效点", valid_count);
        }

};








int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SavePath>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


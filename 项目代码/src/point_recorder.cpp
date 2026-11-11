#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <fstream>
#include <cmath>

class PointRecorder : public rclcpp::Node
{
public:
    PointRecorder() : Node("point_recorder")
    {
        // 声明可调参数（默认3m，flag=2时0.5m）
        this->declare_parameter("record_interval", 3.0);
        this->declare_parameter("flag2_interval", 0.5);

        // 初始化TF监听器
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 订阅flag和distance话题
        flag_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "flag", 10, 
            std::bind(&PointRecorder::flag_callback, this, std::placeholders::_1)
        );
        distance_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "distance", 10, 
            std::bind(&PointRecorder::distance_callback, this, std::placeholders::_1)
        );

        // 初始化CSV文件（重启时清空）
        init_csv();

        // 记录上一次记录的距离
        last_recorded_distance_ = 0.0;
        current_flag_ = 0;
    }

private:
    // 初始化CSV（清空原有内容，写入表头）
    void init_csv()
    {
        std::string csv_path = "/home/racecar/src/my_racecar/include/point.csv";
        csv_file_.open("point.csv", std::ios::out | std::ios::trunc);
        if (csv_file_.is_open())
        {
            csv_file_ << "distance,x,y,yaw\n";
            csv_file_.flush();
            RCLCPP_INFO(this->get_logger(), "CSV文件已初始化");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "无法打开point.csv");
        }
    }

    // 获取当前位姿（x,y,yaw）
    bool get_current_pose(double &x, double &y, double &yaw)
    {
        try
        {
            // 假设base_link到map的变换（根据实际TF树修改父/子坐标系）
            auto transform = tf_buffer_->lookupTransform(
                "map", "base_link", rclcpp::Time(0)
            );

            // 位置
            x = transform.transform.translation.x;
            y = transform.transform.translation.y;

            // 四元数转yaw角（弧度）
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            );
            tf2::Matrix3x3 m(q);
            double roll, pitch;
            m.getRPY(roll, pitch, yaw);

            return true;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF查询失败: %s", ex.what());
            return false;
        }
    }

    // 记录数据到CSV
    //测试
    void record_point(double distance)
    {
        double x, y, yaw;
        if (get_current_pose(x, y, yaw))
        {
            if (csv_file_.is_open())
            {
                csv_file_ << distance << "," << x << "," << y << "," << yaw << "\n";
                csv_file_.flush();
                RCLCPP_INFO(this->get_logger(), "记录点: 距离=%.2f x=%.2f y=%.2f yaw=%.2f", 
                            distance, x, y, yaw);
            }
        }
    }

    // flag话题回调
    void flag_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        current_flag_ = msg->data;
    }

    // distance话题回调
    void distance_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        double current_distance = msg->data;
        double interval = this->get_parameter("record_interval").as_double();

        // 根据flag调整记录间隔
        if (current_flag_ == 2)
        {
            interval = this->get_parameter("flag2_interval").as_double();
        }

        // 判断是否达到记录间隔
        if (current_distance - last_recorded_distance_ >= interval)
        {
            record_point(current_distance);
            last_recorded_distance_ = current_distance;
        }
    }

    // 成员变量
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr flag_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::ofstream csv_file_;
    double last_recorded_distance_;
    int current_flag_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointRecorder>());
    rclcpp::shutdown();
    return 0;
}



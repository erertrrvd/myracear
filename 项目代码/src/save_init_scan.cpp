#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <fstream>
#include <cmath>
#include <string>

class SaveInitScan : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    bool init_scan_saved_ = false;
    std::string init_scan_path_ = "/home/racecar/src/my_racecar/include/init_scan.csv";

public:
    SaveInitScan() : Node("save_init_scan")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&SaveInitScan::scan_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "等待保存第一帧雷达数据...");
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SaveInitScan>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
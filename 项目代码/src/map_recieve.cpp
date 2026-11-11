#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rcpputils/filesystem_helper.hpp" // 替换std::filesystem
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>

// 定义CSV文件保存路径
const std::string CSV_FILE_PATH = "/home/racecar/src/my_racecar/include/map.csv";

class MapReceiveNode : public rclcpp::Node
{
public:
    MapReceiveNode() : Node("map_recieve")
    {
        // ========== 新增功能：启动时清空CSV文件 ==========
        clear_csv_file();

        // 创建/map话题订阅者，队列大小设为10
        map_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            10,
            std::bind(&MapReceiveNode::map_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Map Receive Node Started!");
        RCLCPP_INFO(this->get_logger(), "CSV File Path: %s", CSV_FILE_PATH.c_str());

        // 检查并创建目录（如果不存在）- 替换为rcpputils接口
        std::string dir_path = CSV_FILE_PATH.substr(0, CSV_FILE_PATH.find_last_of('/'));
        rcpputils::fs::path dir_path_obj(dir_path);
        rcpputils::fs::create_directories(dir_path_obj);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;

    // 新增：清空CSV文件函数
    void clear_csv_file()
    {
        try
        {
            // 以截断模式打开文件（清空原有内容），打开后立即关闭
            std::ofstream csv_file(CSV_FILE_PATH, std::ios::trunc);
            if (!csv_file.is_open())
            {
                RCLCPP_WARN(this->get_logger(), "Failed to clear CSV file (file may not exist yet): %s", CSV_FILE_PATH.c_str());
            }
            else
            {
                csv_file.close();
                RCLCPP_INFO(this->get_logger(), "Successfully cleared CSV file on startup");
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error clearing CSV file: %s", e.what());
        }
    }

    // 地图回调函数：处理接收到的地图数据并写入CSV
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        try
        {
            // 以追加模式打开文件（保持原有数据）
            std::ofstream csv_file(CSV_FILE_PATH, std::ios::app);
            if (!csv_file.is_open())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", CSV_FILE_PATH.c_str());
                return;
            }

            // ========== 第一部分：写入地图元数据（单行） ==========
            csv_file << std::fixed << std::setprecision(6); // 保持精度
            csv_file << msg->header.frame_id << ","          // frame_id
                     << msg->info.resolution << ","          // 分辨率
                     << msg->info.width << ","               // 宽度（像素）
                     << msg->info.height << ","              // 高度（像素）
                     << msg->info.origin.position.x << ","   // 原点X坐标
                     << msg->info.origin.position.y << ","   // 原点Y坐标
                     << msg->info.origin.position.z << ","   // 原点Z坐标
                     << msg->info.origin.orientation.x << ","// 原点四元数X
                     << msg->info.origin.orientation.y << ","// 原点四元数Y
                     << msg->info.origin.orientation.z << ","// 原点四元数Z
                     << msg->info.origin.orientation.w       // 原点四元数W
                     << std::endl;

            // ========== 第二部分：写入地图栅格数据（单行） ==========
            for (size_t i = 0; i < msg->data.size(); ++i)
            {
                csv_file << static_cast<int>(msg->data[i]); // 转换为int（避免char类型问题）
                if (i != msg->data.size() - 1)
                {
                    csv_file << ","; // 最后一个元素不加逗号
                }
            }
            csv_file << std::endl;

            csv_file.close();
            RCLCPP_INFO(this->get_logger(), "Successfully wrote map data to CSV (Data size: %zu)", msg->data.size());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error writing CSV file: %s", e.what());
        }
    }
};

int main(int argc, char *argv[])
{
    // 初始化ROS 2
    rclcpp::init(argc, argv);
    // 创建节点（启动时自动清空CSV）
    auto node = std::make_shared<MapReceiveNode>();
    // 自旋节点
    rclcpp::spin(node);
    // 关闭ROS 2
    rclcpp::shutdown();
    return 0;
}
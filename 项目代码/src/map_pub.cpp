#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>

// CSV文件路径
const std::string CSV_FILE_PATH = "/home/racecar/src/my_racecar/include/map.csv";
// 发布的话题名称
const std::string MAP_TOPIC_NAME = "maped";
// 订阅的路径话题名称
const std::string PATH_TOPIC_NAME = "my_local_path";

class MapPublishNode : public rclcpp::Node
{
public:
    MapPublishNode() : Node("map_pub"), 
                       current_path_index(0), 
                       current_map_index(0),
                       map_published_count(0)
    {
        this->declare_parameter("all_index", all_index);
        this->get_parameter("all_index", all_index);



        // 1. 创建/maped话题发布者
        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(MAP_TOPIC_NAME, 10);

        // 2. 订阅my_local_path话题
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            PATH_TOPIC_NAME,
            10,
            std::bind(&MapPublishNode::path_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Map Publish Node Started!");

        // 3. 读取CSV中的所有地图数据
        if (!read_map_from_csv())
        {
            RCLCPP_FATAL(this->get_logger(), "Failed to read map data from CSV! Node will exit.");
            rclcpp::shutdown();
            return;
        }
    }

private:
    int all_index = 650;
    int current_index = 0;
    //int current_map_index = 0;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

    std::vector<nav_msgs::msg::OccupancyGrid> map_data_list; // 所有地图数据
    std::vector<geometry_msgs::msg::PoseStamped> current_path_poses; // 当前路径点列表
    size_t current_path_index; // 当前经过的路径点索引
    size_t current_map_index;  // 当前要发布的地图索引
    size_t map_published_count; // 已发布的地图数量
    size_t map_total_count;     // CSV中地图总数

    rclcpp::Time last_time;    //上次的时间
    int first_time = 0;
    double all_time = 0.0;

    // 读取CSV中的所有地图数据
    bool read_map_from_csv()
    {
        std::ifstream csv_file(CSV_FILE_PATH);
        if (!csv_file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", CSV_FILE_PATH.c_str());
            return false;
        }

        std::string line;
        int line_num = 0;
        nav_msgs::msg::OccupancyGrid temp_map;

        while (std::getline(csv_file, line))
        {
            line_num++;
            std::istringstream line_stream(line);
            std::string token;
            std::vector<std::string> tokens;

            // 分割逗号字段
            while (std::getline(line_stream, token, ','))
            {
                tokens.push_back(token);
            }

            // 奇数行：地图元数据
            if (line_num % 2 == 1)
            {
                try
                {
                    temp_map = nav_msgs::msg::OccupancyGrid();
                    // 填充元数据（与之前逻辑一致）
                    temp_map.header.frame_id = tokens[0];                    
                    temp_map.info.resolution = std::stod(tokens[1]);          
                    temp_map.info.width = std::stoi(tokens[2]);               
                    temp_map.info.height = std::stoi(tokens[3]);             
                    temp_map.info.origin.position.x = std::stod(tokens[4]);   
                    temp_map.info.origin.position.y = std::stod(tokens[5]);   
                    temp_map.info.origin.position.z = std::stod(tokens[6]);   
                    temp_map.info.origin.orientation.x = std::stod(tokens[7]);
                    temp_map.info.origin.orientation.y = std::stod(tokens[8]);
                    temp_map.info.origin.orientation.z = std::stod(tokens[9]);
                    temp_map.info.origin.orientation.w = std::stod(tokens[10]);
                    temp_map.header.stamp = this->get_clock()->now();
                }
                catch (const std::exception& e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Error parsing metadata (line %d): %s", line_num, e.what());
                    continue;
                }
            }
            // 偶数行：栅格数据
            else
            {
                try
                {
                    temp_map.data.resize(tokens.size());
                    for (size_t i = 0; i < tokens.size(); ++i)
                    {
                        temp_map.data[i] = static_cast<int8_t>(std::stoi(tokens[i]));
                    }
                    map_data_list.push_back(temp_map);
                }
                catch (const std::exception& e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Error parsing grid data (line %d): %s", line_num, e.what());
                    continue;
                }
            }
        }
        csv_file.close();

        if (map_data_list.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No valid map data in CSV!");
            return false;
        }

        map_total_count = map_data_list.size();
        RCLCPP_INFO(this->get_logger(), "Successfully read %zu maps from CSV", map_total_count);
        return true;
    }


    // 订阅my_local_path的回调函数
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if(!msg->poses.empty()) 
        {
            current_index = static_cast<int>(msg->poses[0].pose.orientation.y);
        }
        
     
        if(current_index == 0)
        {
            if(first_time == 0)
            {
                first_time = 1;
                last_time = this->now();
            }
            else
            {
                //如果当前下标是0，每隔1s发一次地图
                auto current_time = this->now();
                double dt = (current_time - last_time).seconds();
                all_time += dt;
                
                if(all_time >= 1.0)
                {
                    nav_msgs::msg::OccupancyGrid map_to_pub = map_data_list[0];
                    map_to_pub.header.stamp = this->get_clock()->now();
                    map_publisher_->publish(map_to_pub);
                    RCLCPP_INFO(this->get_logger(), "first_map\n");
                    all_time = 0.0;
                }
                last_time = current_time;
            }


        }
        //判断是否发布当前地图
        else if((float)current_index/(float)all_index*(float)map_total_count >= (float)current_map_index)
        {
            

            if(current_map_index < map_total_count)
            {
                nav_msgs::msg::OccupancyGrid map_to_pub = map_data_list[current_map_index];
                map_to_pub.header.stamp = this->get_clock()->now();
                map_publisher_->publish(map_to_pub);
                RCLCPP_INFO(this->get_logger(), "map: %d\n",(int)current_map_index);
                current_map_index++;
            }


        }




     

       

    }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapPublishNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
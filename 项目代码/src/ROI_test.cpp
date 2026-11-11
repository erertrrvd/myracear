#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class ROIVisualizer : public rclcpp::Node
{
public:
    ROIVisualizer() : Node("ROI_test_node")
    {
        // 声明可调参数：正方形边长
        this->declare_parameter<double>("length", 2.0); // 默认边长2.0米
        
        // 创建发布者，发布MarkerArray消息
        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/visualization_marker_array", 10);
        
        // 创建定时器，定期发布ROI标记
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), // 每500ms发布一次
            std::bind(&ROIVisualizer::publishROI, this));
        
        RCLCPP_INFO(this->get_logger(), "ROI视觉化节点已启动");
        RCLCPP_INFO(this->get_logger(), "正方形边长可通过参数 'length' 调节");
    }

private:
    void publishROI()
    {
        // 获取当前参数值
        double length = this->get_parameter("length").as_double();
        
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // 创建DELETEALL标记来清除之前的显示
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = "laser_link";
        clear_marker.header.stamp = this->now();
        clear_marker.ns = "roi_box";
        clear_marker.id = 0;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        
        // 创建ROI正方形标记
        visualization_msgs::msg::Marker roi_marker;
        
        // 设置基本信息
        roi_marker.header.frame_id = "laser_link";
        roi_marker.header.stamp = this->now();
        roi_marker.ns = "roi_box";
        roi_marker.id = 1; // 与clear_marker不同的ID
        
        // 设置标记类型为立方体
        roi_marker.type = visualization_msgs::msg::Marker::CUBE;
        roi_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // 设置位姿：中心在坐标原点，无旋转
        roi_marker.pose.position.x = 0.0;
        roi_marker.pose.position.y = 0.0;
        roi_marker.pose.position.z = 0.0;
        roi_marker.pose.orientation.x = 0.0;
        roi_marker.pose.orientation.y = 0.0;
        roi_marker.pose.orientation.z = 0.0;
        roi_marker.pose.orientation.w = 1.0; // 无旋转
        
        // 设置尺寸：使用参数化的边长，高度设为较小值（2D效果）
        roi_marker.scale.x = length; // X轴方向长度
        roi_marker.scale.y = length; // Y轴方向长度  
        roi_marker.scale.z = 0.01;   // Z轴方向很薄，实现2D正方形效果
        
        // 设置颜色：蓝色，半透明
        roi_marker.color.r = 0.0;
        roi_marker.color.g = 0.0;
        roi_marker.color.b = 1.0; // 蓝色
        roi_marker.color.a = 0.3; // 透明度30%
        
        // 设置生命周期（可选）
        roi_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
        
        // 将ROI标记添加到数组中
        marker_array.markers.push_back(roi_marker);
        
        // 发布消息
        publisher_->publish(marker_array);
        
        // 可选：在控制台输出当前参数（调试用）
        static double last_length = 0.0;
        if (std::abs(length - last_length) > 0.001) {
            RCLCPP_INFO(this->get_logger(), "更新ROI正方形边长: %.2f 米", length);
            last_length = length;
        }
    }
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ROIVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
// ==================== 【AB改动】整个文件为新增：A/B 停车轨迹录制节点 ====================
// 录制 A/B 停车轨迹：订阅里程计，以第一帧位姿为原点，每隔 save_dx 保存一个点。
// 输出已经是 get_local_path 拼接所需的“起点相对坐标”格式（第一个点 = 0,0,0），
// 第 4 列是速度（默认 1.5），格式同 path.csv：x,y,yaw,type。
//
// 用法：车停在“分叉点”（path.csv 的最后一个点，或 ab_fork_index 对应位置）、
//       车头沿路径方向，启动本节点后开车沿 A（或 B）停车轨迹走到停车位，Ctrl+C 结束。
//   ros2 run my_racecar save_ab_path --ros-args -p ab:=a
// 只需要录一边：另一边由 get_local_path 自动镜像。

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

constexpr double AB_PI = 3.14159265358979323846;

class SaveAbPath : public rclcpp::Node
{
public:
    SaveAbPath() : Node("save_ab_path")
    {
        this->declare_parameter("ab", ab_);
        this->declare_parameter("save_dx", save_dx_);
        this->declare_parameter("speed", speed_);
        this->declare_parameter("out_dir", out_dir_);
        this->get_parameter("ab", ab_);
        this->get_parameter("save_dx", save_dx_);
        this->get_parameter("speed", speed_);
        this->get_parameter("out_dir", out_dir_);

        if (ab_ != "a" && ab_ != "b")
        {
            RCLCPP_ERROR(this->get_logger(), "参数 ab 只能是 a 或 b，当前: %s", ab_.c_str());
            return;
        }

        out_path_ = out_dir_ + "/ab_" + ab_ + ".csv";
        file_.open(out_path_, std::ios::out | std::ios::trunc);
        if (!file_.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "无法创建文件: %s", out_path_.c_str());
            return;
        }
        file_ << "x,y,yaw,type\n";
        ok_ = true;

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/encoder_imu_odom", 10,
            std::bind(&SaveAbPath::odom_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "开始录制 %s 停车轨迹 -> %s", ab_.c_str(), out_path_.c_str());
    }

    ~SaveAbPath() override
    {
        // Ctrl+C 结束时补存最后位置（停车点），否则终点可能比实际少最多 save_dx
        if (ok_ && has_origin_ && std::hypot(cur_x_ - last_x_, cur_y_ - last_y_) > 0.01)
        {
            save_relative(cur_x_, cur_y_, cur_yaw_);
        }
        if (file_.is_open())
        {
            file_.close();
            std::cout << "已保存 " << count_ << " 个点 -> " << out_path_ << std::endl;
        }
    }

    bool ok() const { return ok_; }

private:
    static double quaternionToYaw(double x, double y, double z, double w)
    {
        return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    }

    static double normalize_angle(double a)
    {
        while (a > AB_PI) a -= 2.0 * AB_PI;
        while (a < -AB_PI) a += 2.0 * AB_PI;
        return a;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        const auto &q = msg->pose.pose.orientation;
        double yaw = quaternionToYaw(q.x, q.y, q.z, q.w);
        cur_x_ = x;
        cur_y_ = y;
        cur_yaw_ = yaw;

        if (!has_origin_)
        {
            x0_ = x;
            y0_ = y;
            yaw0_ = yaw;
            has_origin_ = true;
            save_relative(x, y, yaw);   // 第一个点 = (0,0,0)
            return;
        }

        if (std::hypot(x - last_x_, y - last_y_) < save_dx_) return;
        save_relative(x, y, yaw);
    }

    // 世界系 -> 以第一帧为原点的相对坐标，再保存
    void save_relative(double x, double y, double yaw)
    {
        double dx = x - x0_, dy = y - y0_;
        double c = std::cos(-yaw0_), s = std::sin(-yaw0_);
        double rx = c * dx - s * dy;
        double ry = s * dx + c * dy;
        double ryaw = normalize_angle(yaw - yaw0_);

        file_ << rx << "," << ry << "," << ryaw << "," << speed_ << "\n";
        file_.flush();
        ++count_;
        last_x_ = x;
        last_y_ = y;
        RCLCPP_INFO(this->get_logger(), "点 %d: (%.3f, %.3f, %.3f)", count_, rx, ry, ryaw);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::ofstream file_;
    std::string ab_ = "a";
    std::string out_dir_ = "/home/racecar/src/my_racecar/include";
    std::string out_path_;
    double save_dx_ = 0.1;
    double speed_ = 1.5;
    bool ok_ = false;
    bool has_origin_ = false;
    double x0_ = 0.0, y0_ = 0.0, yaw0_ = 0.0;
    double last_x_ = 0.0, last_y_ = 0.0;
    double cur_x_ = 0.0, cur_y_ = 0.0, cur_yaw_ = 0.0;
    int count_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    {
        auto node = std::make_shared<SaveAbPath>();
        if (node->ok())
        {
            rclcpp::spin(node);
        }
    }   // node 在 shutdown 前析构，补存最后一个点（停车位）
    rclcpp::shutdown();
    return 0;
}

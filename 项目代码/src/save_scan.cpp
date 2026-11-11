// ============================================================================
//  save_scan.cpp —— 录制参考点云（第一步）
// ============================================================================
//  作用：
//      车停在起跑位，累积 N 帧雷达数据，体素降采样后存成 CSV。
//      这份点云就是后续匹配的【参考基准】。
//
//  为什么要累积多帧：
//      单帧点云稀疏、有噪声。多帧累积后：
//        · 静态结构（锥桶/墙）的点精确堆叠 → 密实
//        · 走动的人被"抹开" → 稀疏 → 后续匹配时容易被剔除
//
//  使用：
//      ros2 run my_racecar save_scan
//      ros2 run my_racecar save_scan --ros-args -p accumulate_frames:=40 -p output_path:=/home/racecar/ref_cloud.csv
//
//  输出 CSV 格式：
//      # frame_id=laser_link
//      # accumulate_frames=20
//      # raw_points=30122
//      # voxel_size=0.03
//      # saved_points=4831
//      x,y
//      1.234567,-0.456789
//      ...
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <map>
#include <string>
#include <vector>

struct Point2D
{
    double x{0.0};
    double y{0.0};
};

// ---------------------------------------------------------------------------
//  体素降采样：把空间划成固定边长的格子，每格只留一个点
//  作用：统一密度、减少点数、抑制噪声
// ---------------------------------------------------------------------------
static std::vector<Point2D> voxelDownsample(const std::vector<Point2D> & in, double voxel)
{
    std::vector<Point2D> out;
    if (voxel <= 0.0) return in;

    std::map<std::pair<int, int>, std::pair<double, double>> grid; // 格 -> 累加坐标
    std::map<std::pair<int, int>, int> count;

    for (const auto & p : in)
    {
        int ix = static_cast<int>(std::floor(p.x / voxel));
        int iy = static_cast<int>(std::floor(p.y / voxel));
        auto key = std::make_pair(ix, iy);
        grid[key].first  += p.x;
        grid[key].second += p.y;
        count[key] += 1;
    }

    out.reserve(grid.size());
    for (auto & kv : grid)
    {
        int n = count[kv.first];
        out.push_back({kv.second.first / n, kv.second.second / n});
    }
    return out;
}

// ---------------------------------------------------------------------------
class SaveScanNode : public rclcpp::Node
{
public:
    SaveScanNode() : Node("save_scan")
    {
        declare_parameter("accumulate_frames", accumulate_frames_);
        declare_parameter("voxel_size",        voxel_size_);
        declare_parameter("min_range",         min_range_);
        declare_parameter("max_range",         max_range_);
        declare_parameter("output_path",       output_path_);

        accumulate_frames_ = get_parameter("accumulate_frames").as_int();
        voxel_size_        = get_parameter("voxel_size").as_double();
        min_range_         = get_parameter("min_range").as_double();
        max_range_         = get_parameter("max_range").as_double();
        output_path_       = get_parameter("output_path").as_string();

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&SaveScanNode::scanCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "==============================================");
        RCLCPP_INFO(get_logger(), " save_scan 已启动");
        RCLCPP_INFO(get_logger(), "   累积帧数 : %d", accumulate_frames_);
        RCLCPP_INFO(get_logger(), "   体素尺寸 : %.3f m", voxel_size_);
        RCLCPP_INFO(get_logger(), "   量程     : %.2f ~ %.2f m", min_range_, max_range_);
        RCLCPP_INFO(get_logger(), "   输出文件 : %s", output_path_.c_str());
        RCLCPP_INFO(get_logger(), "==============================================");
        RCLCPP_INFO(get_logger(), ">>> 请保持车【静止】，正在累积...");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (done_) return;

        if (frame_id_.empty()) frame_id_ = msg->header.frame_id;

        // 把这一帧转成 2D 点
        size_t valid = 0;
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float r = msg->ranges[i];
            if (std::isnan(r) || std::isinf(r)) continue;
            if (r < msg->range_min || r > msg->range_max) continue;
            if (r < min_range_ || r > max_range_) continue;

            float a = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
            raw_.push_back({static_cast<double>(r) * std::cos(a),
                            static_cast<double>(r) * std::sin(a)});
            ++valid;
        }

        frames_++;

        if (frames_ % 5 == 0 && frames_ < accumulate_frames_)
        {
            RCLCPP_INFO(get_logger(), "  累积中 %d/%d 帧，本帧有效点 %zu，累计 %zu 点",
                        frames_, accumulate_frames_, valid, raw_.size());
        }

        if (frames_ < accumulate_frames_) return;

        done_ = true;
        saveToFile();

        RCLCPP_INFO(get_logger(), "★ 参考点云已保存，节点停止采集（可 Ctrl+C 退出）");
    }

    void saveToFile()
    {
        // 体素降采样
        auto down = voxelDownsample(raw_, voxel_size_);

        std::ofstream f(output_path_);
        if (!f.is_open())
        {
            RCLCPP_ERROR(get_logger(), "无法写入文件: %s", output_path_.c_str());
            return;
        }

        f << std::fixed << std::setprecision(6);

        // 头部注释（记录元信息，方便以后追溯）
        f << "# frame_id=" << frame_id_ << "\n";
        f << "# accumulate_frames=" << frames_ << "\n";
        f << "# raw_points=" << raw_.size() << "\n";
        f << "# voxel_size=" << voxel_size_ << "\n";
        f << "# min_range=" << min_range_ << "\n";
        f << "# max_range=" << max_range_ << "\n";
        f << "# saved_points=" << down.size() << "\n";
        f << "x,y\n";

        for (const auto & p : down)
            f << p.x << "," << p.y << "\n";

        f.close();

        RCLCPP_INFO(get_logger(), "==============================================");
        RCLCPP_INFO(get_logger(), " 录制完成");
        RCLCPP_INFO(get_logger(), "   frame_id   : %s", frame_id_.c_str());
        RCLCPP_INFO(get_logger(), "   累积帧数   : %d", frames_);
        RCLCPP_INFO(get_logger(), "   原始点数   : %zu", raw_.size());
        RCLCPP_INFO(get_logger(), "   降采样后   : %zu  (体素 %.3f m)", down.size(), voxel_size_);
        RCLCPP_INFO(get_logger(), "   保存到     : %s", output_path_.c_str());
        RCLCPP_INFO(get_logger(), "==============================================");
    }

    // ---- 参数 ----
    int         accumulate_frames_{20};
    double      voxel_size_{0.03};
    double      min_range_{0.20};
    double      max_range_{25.0};
    std::string output_path_{"/home/racecar/src/my_racecar/include/ref_cloud.csv"};

    // ---- 状态 ----
    std::vector<Point2D> raw_;
    int         frames_{0};
    bool        done_{false};
    std::string frame_id_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SaveScanNode>());
    rclcpp::shutdown();
    return 0;
}

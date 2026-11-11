// initial_calibration.cpp
// 功能：2D激光雷达点云匹配，校准初始位置，并在RViz中可视化
// 版本：KD-Tree + ICP（Point-to-Point + SVD）
// 说明：替换了原来的粗搜索/细搜索暴力匹配

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iomanip>

#include <Eigen/Dense>

// 2D点结构体
struct Point2D
{
    float x, y;
    Point2D(float x_ = 0, float y_ = 0) : x(x_), y(y_) {}
};

// =====================================================
// KD-Tree（2D专用，高效最近邻搜索）
// =====================================================
class KDTree2D
{
public:
    struct Node
    {
        Point2D point;
        int left = -1;
        int right = -1;
        int split_axis = 0;  // 0=x, 1=y
    };

    std::vector<Node> nodes_;
    std::vector<Point2D> points_;

    void build(const std::vector<Point2D>& points)
    {
        points_ = points;
        nodes_.clear();
        nodes_.reserve(points.size());
        if (points.empty()) return;

        std::vector<int> indices(points.size());
        std::iota(indices.begin(), indices.end(), 0);
        build_recursive(indices, 0);
    }

    // 查找最近邻，返回距离平方
    float nearest(const Point2D& query, Point2D& best_point) const
    {
        if (nodes_.empty()) return 1e10f;
        float best_dist_sq = 1e10f;
        nearest_recursive(0, query, best_point, best_dist_sq);
        return best_dist_sq;
    }

private:
    int build_recursive(std::vector<int>& indices, int depth)
    {
        if (indices.empty()) return -1;

        int axis = depth % 2;

        std::sort(indices.begin(), indices.end(),
            [&](int a, int b)
            {
                return axis == 0 ? points_[a].x < points_[b].x
                                 : points_[a].y < points_[b].y;
            });

        int mid = static_cast<int>(indices.size()) / 2;

        Node node;
        node.point = points_[indices[mid]];
        node.split_axis = axis;

        int node_idx = static_cast<int>(nodes_.size());
        nodes_.push_back(node);

        std::vector<int> left_indices(indices.begin(), indices.begin() + mid);
        std::vector<int> right_indices(indices.begin() + mid + 1, indices.end());

        nodes_[node_idx].left = build_recursive(left_indices, depth + 1);
        nodes_[node_idx].right = build_recursive(right_indices, depth + 1);

        return node_idx;
    }

    void nearest_recursive(int node_idx, const Point2D& query,
                           Point2D& best_point, float& best_dist_sq) const
    {
        if (node_idx < 0) return;

        const Node& node = nodes_[node_idx];

        float dx = query.x - node.point.x;
        float dy = query.y - node.point.y;
        float dist_sq = dx * dx + dy * dy;

        if (dist_sq < best_dist_sq)
        {
            best_dist_sq = dist_sq;
            best_point = node.point;
        }

        float diff = (node.split_axis == 0) ? dx : dy;
        int first = diff < 0 ? node.left : node.right;
        int second = diff < 0 ? node.right : node.left;

        nearest_recursive(first, query, best_point, best_dist_sq);

        if (diff * diff < best_dist_sq)
        {
            nearest_recursive(second, query, best_point, best_dist_sq);
        }
    }
};

// =====================================================
// 真正的 ICP 算法（Point-to-Point，SVD求解）
// =====================================================
class ICP2D
{
public:
    struct Result
    {
        float dx = 0;
        float dy = 0;
        float dyaw = 0;
        float error = 1e10f;
        int iterations = 0;
        bool converged = false;
    };

    struct Config
    {
        int max_iterations = 30;
        float convergence_threshold = 1e-6f;
        float max_correspondence_dist = 0.4f;
    };

    Config config_;

    Result match(const std::vector<Point2D>& source,
                 const std::vector<Point2D>& target,
                 float init_dx = 0.0f,
                 float init_dy = 0.0f,
                 float init_dyaw = 0.0f)
    {
        Result result;

        if (source.size() < 3 || target.size() < 3)
        {
            return result;
        }

        KDTree2D kdtree;
        kdtree.build(target);

        Eigen::Matrix3f T = make_transform(init_dx, init_dy, init_dyaw);

        std::vector<Eigen::Vector2f> src_points(source.size());
        for (size_t i = 0; i < source.size(); ++i)
        {
            src_points[i] = Eigen::Vector2f(source[i].x, source[i].y);
        }

        float prev_error = 1e10f;
        float max_dist_sq = config_.max_correspondence_dist * config_.max_correspondence_dist;

        for (int iter = 0; iter < config_.max_iterations; ++iter)
        {
            // 1) 变换源点
            std::vector<Eigen::Vector2f> transformed(src_points.size());
            for (size_t i = 0; i < src_points.size(); ++i)
            {
                Eigen::Vector3f p(src_points[i].x(), src_points[i].y(), 1.0f);
                Eigen::Vector3f tp = T * p;
                transformed[i] = Eigen::Vector2f(tp.x(), tp.y());
            }

            // 2) 找对应点
            std::vector<Eigen::Vector2f> matched_src;
            std::vector<Eigen::Vector2f> matched_tgt;
            matched_src.reserve(src_points.size());
            matched_tgt.reserve(src_points.size());

            float total_error = 0.0f;

            for (size_t i = 0; i < transformed.size(); ++i)
            {
                Point2D query{transformed[i].x(), transformed[i].y()};
                Point2D nearest_pt;
                float dist_sq = kdtree.nearest(query, nearest_pt);

                if (dist_sq < max_dist_sq)
                {
                    matched_src.push_back(src_points[i]);
                    matched_tgt.push_back(Eigen::Vector2f(nearest_pt.x, nearest_pt.y));
                    total_error += dist_sq;
                }
            }

            if (matched_src.size() < 3)
            {
                break;
            }

            float mean_error = total_error / static_cast<float>(matched_src.size());

            if (std::fabs(prev_error - mean_error) < config_.convergence_threshold)
            {
                result.converged = true;
                result.error = mean_error;
                result.iterations = iter + 1;
                break;
            }

            prev_error = mean_error;

            Eigen::Matrix3f delta_T = solve_svd(matched_src, matched_tgt, T);
            T = delta_T * T;

            result.error = mean_error;
            result.iterations = iter + 1;
        }

        result.dx = T(0, 2);
        result.dy = T(1, 2);
        result.dyaw = std::atan2(T(1, 0), T(0, 0));

        if (!result.converged && result.iterations == config_.max_iterations)
        {
            result.converged = (result.error < 0.01f);
        }

        return result;
    }

private:
    Eigen::Matrix3f make_transform(float dx, float dy, float dyaw)
    {
        float c = std::cos(dyaw);
        float s = std::sin(dyaw);

        Eigen::Matrix3f T;
        T << c, -s, dx,
             s,  c, dy,
             0,  0,  1;
        return T;
    }

    Eigen::Matrix3f solve_svd(const std::vector<Eigen::Vector2f>& src,
                              const std::vector<Eigen::Vector2f>& tgt,
                              const Eigen::Matrix3f& current_T)
    {
        int n = static_cast<int>(src.size());

        std::vector<Eigen::Vector2f> src_transformed(n);
        for (int i = 0; i < n; ++i)
        {
            Eigen::Vector3f p(src[i].x(), src[i].y(), 1.0f);
            Eigen::Vector3f tp = current_T * p;
            src_transformed[i] = Eigen::Vector2f(tp.x(), tp.y());
        }

        Eigen::Vector2f centroid_src = Eigen::Vector2f::Zero();
        Eigen::Vector2f centroid_tgt = Eigen::Vector2f::Zero();

        for (int i = 0; i < n; ++i)
        {
            centroid_src += src_transformed[i];
            centroid_tgt += tgt[i];
        }
        centroid_src /= static_cast<float>(n);
        centroid_tgt /= static_cast<float>(n);

        Eigen::MatrixXf S(2, n);
        Eigen::MatrixXf D(2, n);

        for (int i = 0; i < n; ++i)
        {
            S.col(i) = src_transformed[i] - centroid_src;
            D.col(i) = tgt[i] - centroid_tgt;
        }

        Eigen::Matrix2f W = D * S.transpose();
        Eigen::JacobiSVD<Eigen::Matrix2f> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Matrix2f R = svd.matrixU() * svd.matrixV().transpose();

        if (R.determinant() < 0)
        {
            Eigen::Matrix2f V = svd.matrixV();
            V.col(1) *= -1.0f;
            R = svd.matrixU() * V.transpose();
        }

        Eigen::Vector2f t = centroid_tgt - R * centroid_src;

        Eigen::Matrix3f delta_T;
        delta_T << R(0, 0), R(0, 1), t.x(),
                   R(1, 0), R(1, 1), t.y(),
                   0,       0,       1;
        return delta_T;
    }
};

class InitialCalibration : public rclcpp::Node
{
private:
    // ==================== 订阅者和发布者 ====================
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ref_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cur_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr calibration_done_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr vis_timer_;

    // ==================== 数据存储 ====================
    std::vector<Point2D> ref_points_;
    std::vector<Point2D> current_points_;
    std::vector<Point2D> aligned_points_;

    // ==================== 校准结果 ====================
    float result_dx_ = 0.0f;
    float result_dy_ = 0.0f;
    float result_dyaw_ = 0.0f;
    float match_error_ = 0.0f;
    bool calibration_done_ = false;
    int scan_count_ = 0;

    // ==================== 参数 ====================
    std::string init_scan_path_;
    int skip_frames_;
    float error_threshold_;

    // ICP对象
    ICP2D icp_;

public:
    InitialCalibration() : Node("initial_calibration")
    {
        this->declare_parameter("init_scan_path",
            "/home/racecar/src/my_racecar/include/init_scan.csv");
        this->declare_parameter("skip_frames", 5);
        this->declare_parameter("error_threshold", 0.1);

        init_scan_path_ = this->get_parameter("init_scan_path").as_string();
        skip_frames_ = this->get_parameter("skip_frames").as_int();
        error_threshold_ = this->get_parameter("error_threshold").as_double();

        if (!load_reference_scan())
        {
            RCLCPP_ERROR(this->get_logger(), "加载参考点云失败，节点退出");
            return;
        }

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&InitialCalibration::scan_callback, this, std::placeholders::_1));

        ref_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "calibration/reference_cloud", 10);
        cur_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "calibration/current_cloud", 10);
        aligned_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "calibration/aligned_cloud", 10);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "calibration/markers", 10);

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);

        calibration_done_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "calibration/done", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        vis_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&InitialCalibration::publish_visualization, this));

        RCLCPP_INFO(this->get_logger(), "2D激光雷达初始位置校准节点启动");
        RCLCPP_INFO(this->get_logger(), "参考点数量: %zu", ref_points_.size());
        RCLCPP_INFO(this->get_logger(), "跳过前 %d 帧", skip_frames_);
        RCLCPP_INFO(this->get_logger(), "等待雷达数据...");
    }

    bool load_reference_scan()
    {
        std::ifstream file(init_scan_path_);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "无法打开参考点云文件: %s", init_scan_path_.c_str());
            return false;
        }

        std::string line;
        while (std::getline(file, line))
        {
            if (line.empty() || line[0] == '#')
                continue;
            if (line.find("index") != std::string::npos || line.find("angle") != std::string::npos)
                continue;
            break;
        }

        do
        {
            if (line.empty() || line[0] == '#')
                continue;

            std::stringstream ss(line);
            std::string token;
            std::vector<float> values;

            while (std::getline(ss, token, ','))
            {
                try
                {
                    values.push_back(std::stof(token));
                }
                catch (...)
                {
                    continue;
                }
            }

            if (values.size() >= 5)
            {
                ref_points_.emplace_back(values[3], values[4]);
            }
        } while (std::getline(file, line));

        if (ref_points_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "参考点云为空！");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "成功加载 %zu 个参考点", ref_points_.size());
        return true;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan_count_++;

        if (scan_count_ <= skip_frames_)
        {
            RCLCPP_INFO(this->get_logger(), "跳过第 %d 帧...", scan_count_);
            return;
        }

        current_points_ = scan_to_points(msg);

        if (!calibration_done_)
        {
            RCLCPP_INFO(this->get_logger(), "开始ICP匹配校准...");

            if (icp_match())
            {
                calibration_done_ = true;

                aligned_points_ = transform_points(current_points_,
                                                   result_dx_, result_dy_, result_dyaw_);

                publish_initial_pose();

                auto done_msg = std_msgs::msg::Bool();
                done_msg.data = true;
                calibration_done_pub_->publish(done_msg);

                RCLCPP_INFO(this->get_logger(), "校准完成！");
                RCLCPP_INFO(this->get_logger(), "dx: %.4f m", result_dx_);
                RCLCPP_INFO(this->get_logger(), "dy: %.4f m", result_dy_);
                RCLCPP_INFO(this->get_logger(), "dyaw: %.4f deg", result_dyaw_ * 180.0 / M_PI);
                RCLCPP_INFO(this->get_logger(), "误差: %.6f", match_error_);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "匹配失败，误差: %.6f", match_error_);
            }
        }
        else
        {
            aligned_points_ = transform_points(current_points_,
                                               result_dx_, result_dy_, result_dyaw_);
        }
    }

    std::vector<Point2D> scan_to_points(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::vector<Point2D> points;
        points.reserve(msg->ranges.size());

        for (size_t i = 0; i < msg->ranges.size(); i++)
        {
            float range = msg->ranges[i];

            if (range < msg->range_min || range > msg->range_max)
                continue;
            if (std::isinf(range) || std::isnan(range))
                continue;
            if (range < 0.05f || range > 8.0f)
                continue;

            float angle = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
            points.emplace_back(range * std::cos(angle), range * std::sin(angle));
        }

        return points;
    }

    std::vector<Point2D> transform_points(const std::vector<Point2D>& points,
                                          float dx, float dy, float dyaw)
    {
        std::vector<Point2D> transformed;
        transformed.reserve(points.size());

        float cos_yaw = std::cos(dyaw);
        float sin_yaw = std::sin(dyaw);

        for (const auto& p : points)
        {
            float new_x = cos_yaw * p.x - sin_yaw * p.y + dx;
            float new_y = sin_yaw * p.x + cos_yaw * p.y + dy;
            transformed.emplace_back(new_x, new_y);
        }

        return transformed;
    }

    bool icp_match()
    {
        RCLCPP_INFO(this->get_logger(), "开始ICP匹配...");
        RCLCPP_INFO(this->get_logger(), "参考点云: %zu 个点", ref_points_.size());
        RCLCPP_INFO(this->get_logger(), "当前点云: %zu 个点", current_points_.size());

        icp_.config_.max_iterations = 30;
        icp_.config_.convergence_threshold = 1e-6f;
        icp_.config_.max_correspondence_dist = 0.4f;

        auto result = icp_.match(current_points_, ref_points_, 0.0f, 0.0f, 0.0f);

        RCLCPP_INFO(this->get_logger(), "ICP结果:");
        RCLCPP_INFO(this->get_logger(), "  dx: %.4f", result.dx);
        RCLCPP_INFO(this->get_logger(), "  dy: %.4f", result.dy);
        RCLCPP_INFO(this->get_logger(), "  dyaw: %.4f rad (%.2f deg)",
                    result.dyaw, result.dyaw * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "  error: %.6f", result.error);
        RCLCPP_INFO(this->get_logger(), "  iterations: %d", result.iterations);
        RCLCPP_INFO(this->get_logger(), "  converged: %s", result.converged ? "true" : "false");

        result_dx_ = result.dx;
        result_dy_ = result.dy;
        result_dyaw_ = result.dyaw;
        match_error_ = result.error;

        return result.converged && result.error < error_threshold_;
    }

    void publish_initial_pose()
    {
        auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = this->now();

        pose_msg.pose.pose.position.x = result_dx_;
        pose_msg.pose.pose.position.y = result_dy_;
        pose_msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, result_dyaw_);
        pose_msg.pose.pose.orientation.x = q.x();
        pose_msg.pose.pose.orientation.y = q.y();
        pose_msg.pose.pose.orientation.z = q.z();
        pose_msg.pose.pose.orientation.w = q.w();

        pose_msg.pose.covariance[0] = 0.01;
        pose_msg.pose.covariance[7] = 0.01;
        pose_msg.pose.covariance[35] = 0.01;

        pose_pub_->publish(pose_msg);
    }

    void publish_visualization()
    {
        auto now = this->now();

        if (!ref_points_.empty())
        {
            publish_pointcloud(ref_cloud_pub_, ref_points_, now, 0.0f, 1.0f, 0.0f);
        }
        if (!current_points_.empty())
        {
            publish_pointcloud(cur_cloud_pub_, current_points_, now, 1.0f, 0.0f, 0.0f);
        }
        if (!aligned_points_.empty())
        {
            publish_pointcloud(aligned_cloud_pub_, aligned_points_, now, 0.0f, 0.5f, 1.0f);
        }

        publish_markers(now);
        publish_tf(now);
    }

    void publish_pointcloud(
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
        const std::vector<Point2D>& points,
        const rclcpp::Time& stamp,
        float r, float g, float b)
    {
        auto cloud_msg = sensor_msgs::msg::PointCloud2();
        cloud_msg.header.frame_id = "map";
        cloud_msg.header.stamp = stamp;
        cloud_msg.height = 1;
        cloud_msg.width = static_cast<uint32_t>(points.size());
        cloud_msg.is_dense = true;
        cloud_msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(points.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(cloud_msg, "rgb");

        uint8_t r_byte = static_cast<uint8_t>(r * 255);
        uint8_t g_byte = static_cast<uint8_t>(g * 255);
        uint8_t b_byte = static_cast<uint8_t>(b * 255);

        for (const auto& p : points)
        {
            *iter_x = p.x;
            *iter_y = p.y;
            *iter_z = 0.0f;

            iter_rgb[0] = b_byte;
            iter_rgb[1] = g_byte;
            iter_rgb[2] = r_byte;

            ++iter_x; ++iter_y; ++iter_z; ++iter_rgb;
        }

        pub->publish(cloud_msg);
    }

    void publish_markers(const rclcpp::Time& stamp)
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();

        auto origin_marker = visualization_msgs::msg::Marker();
        origin_marker.header.frame_id = "map";
        origin_marker.header.stamp = stamp;
        origin_marker.ns = "calibration";
        origin_marker.id = 0;
        origin_marker.type = visualization_msgs::msg::Marker::SPHERE;
        origin_marker.action = visualization_msgs::msg::Marker::ADD;
        origin_marker.pose.position.x = 0;
        origin_marker.pose.position.y = 0;
        origin_marker.pose.position.z = 0;
        origin_marker.scale.x = 0.1;
        origin_marker.scale.y = 0.1;
        origin_marker.scale.z = 0.1;
        origin_marker.color.r = 1.0;
        origin_marker.color.g = 1.0;
        origin_marker.color.b = 0.0;
        origin_marker.color.a = 1.0;
        marker_array.markers.push_back(origin_marker);

        if (calibration_done_)
        {
            auto arrow_marker = visualization_msgs::msg::Marker();
            arrow_marker.header.frame_id = "map";
            arrow_marker.header.stamp = stamp;
            arrow_marker.ns = "calibration";
            arrow_marker.id = 1;
            arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
            arrow_marker.action = visualization_msgs::msg::Marker::ADD;

            geometry_msgs::msg::Point start, end;
            start.x = 0; start.y = 0; start.z = 0.1;
            end.x = result_dx_; end.y = result_dy_; end.z = 0.1;
            arrow_marker.points.push_back(start);
            arrow_marker.points.push_back(end);

            arrow_marker.scale.x = 0.02;
            arrow_marker.scale.y = 0.04;
            arrow_marker.color.r = 1.0;
            arrow_marker.color.g = 0.5;
            arrow_marker.color.b = 0.0;
            arrow_marker.color.a = 1.0;
            marker_array.markers.push_back(arrow_marker);
        }

        auto text_marker = visualization_msgs::msg::Marker();
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = stamp;
        text_marker.ns = "calibration";
        text_marker.id = 2;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position.x = 0;
        text_marker.pose.position.y = 0;
        text_marker.pose.position.z = 0.5;
        text_marker.scale.z = 0.15;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;

        std::stringstream ss;
        if (calibration_done_)
        {
            ss << "校准完成\n";
            ss << "dx: " << std::fixed << std::setprecision(3) << result_dx_ << " m\n";
            ss << "dy: " << std::fixed << std::setprecision(3) << result_dy_ << " m\n";
            ss << "dyaw: " << std::fixed << std::setprecision(2) << result_dyaw_ * 180.0 / M_PI << " deg\n";
            ss << "error: " << std::fixed << std::setprecision(4) << match_error_;
        }
        else
        {
            ss << "等待校准...\n";
            ss << "已接收 " << scan_count_ << " 帧";
        }
        text_marker.text = ss.str();
        marker_array.markers.push_back(text_marker);

        marker_pub_->publish(marker_array);
    }

    void publish_tf(const rclcpp::Time& stamp)
    {
        if (!calibration_done_)
            return;

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = stamp;
        t.header.frame_id = "map";
        t.child_frame_id = "calibrated_base";

        t.transform.translation.x = result_dx_;
        t.transform.translation.y = result_dy_;
        t.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, result_dyaw_);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
    }
};

// ==================== 主函数 ====================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitialCalibration>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
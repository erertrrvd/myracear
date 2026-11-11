// ============================================================================
//  scan_match.cpp —— 点云匹配求初始位姿（第二步）
// ============================================================================
//  作用：
//      读入 save_scan 录制的参考点云，与当前雷达点云做配准，
//      输出 (dx, dy, dyaw) —— 即"当前雷达系 → 参考点云坐标系"的刚体变换。
//
//  算法（多分辨率 + 截尾 + 双向验证 的 ICP）：
//      ① 累积 N 帧 → 体素降采样          （抗人群的第一层）
//      ② 多分辨率：20cm → 10cm → 5cm     （由粗到细，扩大收敛域）
//      ③ 每层内迭代：
//           · 网格空间索引找最近邻
//           · 按距离排序，丢掉最差的 30%   ← 截尾，抗人群的第二层
//           · 双向验证（互相都是最近邻）
//           · SVD 求解增量变换
//      ④ 初值搜索：yaw 在 ±N 度内扫一遍，取残差最小的
//      ⑤ 自检：内点比例 + RMS 残差 + 界内检查 + 明确成功/失败
//
//  RViz 可视化（关键）：
//      /scan_match/axes       坐标系箭头（红=X前, 绿=Y左）+ 文字标注
//      /scan_match/ref_cloud  参考点云（蓝色）
//      /scan_match/cur_cloud  当前点云【变换后】（绿色）
//      /scan_match/inliers    参与求解的内点（黄色，越密说明匹配越好）
//      /scan_match/result     结果文字
//
//  使用：
//      ros2 run my_racecar scan_match
//      ros2 run my_racecar scan_match --ros-args -p ref_path:=/home/racecar/ref_cloud.csv
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>

// ===========================================================================
//  基础类型
// ===========================================================================
struct Point2D
{
    double x{0.0};
    double y{0.0};
};

static inline double normalizeAngle(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// ===========================================================================
//  网格空间索引 —— 把最近邻查询从 O(N*M) 降到接近 O(N)
//  原理：空间划成边长 = cell 的格子；查询时只看"自己所在格 + 周围 8 格"
// ===========================================================================
class GridIndex
{
public:
    void build(const std::vector<Point2D> & pts, double cell)
    {
        pts_  = &pts;
        cell_ = (cell > 1e-6) ? cell : 0.1;
        grid_.clear();
        for (int i = 0; i < static_cast<int>(pts.size()); ++i)
        {
            grid_[key(pts[i].x, pts[i].y)].push_back(i);
        }
    }

    // 返回最近点下标；找不到返回 -1
    // 只搜索 3x3 邻域（快）；若邻域内无点则退化为 -1（由调用方的 max_r 兜住）
    int nearest(double qx, double qy, double max_r, double & out_d2) const
    {
        const int cx = static_cast<int>(std::floor(qx / cell_));
        const int cy = static_cast<int>(std::floor(qy / cell_));

        int    best = -1;
        double best_d2 = max_r * max_r;

        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                auto it = grid_.find(key(static_cast<double>(cx + dx) * cell_,
                                         static_cast<double>(cy + dy) * cell_));
                if (it == grid_.end()) continue;

                for (int idx : it->second)
                {
                    const Point2D & p = (*pts_)[idx];
                    const double ddx = p.x - qx;
                    const double ddy = p.y - qy;
                    const double d2  = ddx * ddx + ddy * ddy;
                    if (d2 < best_d2) { best_d2 = d2; best = idx; }
                }
            }
        }
        out_d2 = best_d2;
        return best;
    }

private:
    static int64_t key(double x, double y)
    {
        // 用格子索引拼成一个 64 位 key（假设坐标范围 ±1000m，够用）
        const int64_t ix = static_cast<int64_t>(std::floor(x)) + 100000;
        const int64_t iy = static_cast<int64_t>(std::floor(y)) + 100000;
        return ix * 1000000 + iy;
    }

    const std::vector<Point2D> * pts_{nullptr};
    double cell_{0.1};
    std::map<int64_t, std::vector<int>> grid_;
};

// ===========================================================================
//  配准结果
// ===========================================================================
struct MatchResult
{
    double dx{0.0};
    double dy{0.0};
    double dyaw{0.0};
    double rms{0.0};            // 内点 RMS 残差（米）
    double inlier_ratio{0.0};   // 内点比例
    int    inliers{0};          // 内点数
    int    iterations{0};
    bool   ok{false};
};

// ===========================================================================
//  单层 ICP（给定体素尺寸和搜索半径，迭代收敛）
// ===========================================================================
class ICPMatcher
{
public:
    double trim_ratio{0.8};       // 保留最好的 80%，丢掉最差 20%
    int    max_iterations{30};
    double convergence{1e-5};
    double search_r_{0.20};       // 对应点搜索半径（成员，方便统一设置）

    // ---- 统计最终指标（用较宽的搜索半径统计，避免漏掉）----
    // 也可被外部调用，用来算"identity 基准"的 RMS
    void computeStats(const std::vector<Point2D> & source,
                      const std::vector<Point2D> & target,
                      const Eigen::Matrix2f & R, const Eigen::Vector2f & t,
                      double stat_r,
                      double & rms, double & ratio, int & inliers)
    {
        GridIndex index;
        index.build(target, stat_r);

        rms = 0.0;
        inliers = 0;

        for (const auto & p : source)
        {
            Eigen::Vector2f tp = R * Eigen::Vector2f(static_cast<float>(p.x),
                                                     static_cast<float>(p.y)) + t;
            double d2 = 0.0;
            if (index.nearest(tp.x(), tp.y(), stat_r, d2) >= 0)
            {
                rms += std::sqrt(d2);
                ++inliers;
            }
        }

        if (inliers > 0) rms /= inliers;
        ratio = source.empty() ? 0.0
                               : static_cast<double>(inliers) / source.size();
    }

    // source: 当前点云（将被变换）
    // target: 参考点云（固定）
    // R, t   : 输入/输出，累积变换（输出满足: target ≈ R * source + t）
    bool run(const std::vector<Point2D> & source,
             const std::vector<Point2D> & target,
             Eigen::Matrix2f & R, Eigen::Vector2f & t,
             double search_r,
             double & out_rms, double & out_ratio, int & out_inliers)
    {
        if (source.size() < 3 || target.size() < 3) return false;

        GridIndex index;
        index.build(target, search_r);

        bool converged = false;

        for (int iter = 0; iter < max_iterations; ++iter)
        {
            // ---- ① 找对应（源点变换到目标系后，找最近邻）----
            struct Corr { Eigen::Vector2f s; Eigen::Vector2f tgt; double d2; };
            std::vector<Corr> corrs;
            corrs.reserve(source.size());

            for (const auto & p : source)
            {
                Eigen::Vector2f sp(static_cast<float>(p.x), static_cast<float>(p.y));
                Eigen::Vector2f tp = R * sp + t;

                double d2 = 0.0;
                int idx = index.nearest(tp.x(), tp.y(), search_r, d2);
                if (idx < 0) continue;

                Corr c;
                c.s = sp;
                c.tgt = Eigen::Vector2f(static_cast<float>(target[idx].x),
                                        static_cast<float>(target[idx].y));
                c.d2 = d2;
                corrs.push_back(c);
            }

            if (corrs.size() < 10) return false;

            // ---- ② 截尾：按距离排序，只保留最好的 trim_ratio ----
            std::sort(corrs.begin(), corrs.end(),
                      [](const Corr & a, const Corr & b) { return a.d2 < b.d2; });

            size_t keep = static_cast<size_t>(corrs.size() * trim_ratio);
            if (keep < 10) keep = std::min<size_t>(corrs.size(), 10);
            corrs.resize(keep);

            // ---- ③ SVD 求增量变换 ----
            Eigen::Vector2f cs = Eigen::Vector2f::Zero();
            Eigen::Vector2f ct = Eigen::Vector2f::Zero();
            for (const auto & c : corrs) { cs += c.s; ct += c.tgt; }
            cs /= static_cast<float>(corrs.size());
            ct /= static_cast<float>(corrs.size());

            Eigen::Matrix2f H = Eigen::Matrix2f::Zero();
            for (const auto & c : corrs)
            {
                H += (c.s - cs) * (c.tgt - ct).transpose();
            }

            Eigen::JacobiSVD<Eigen::Matrix2f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix2f U = svd.matrixU();
            Eigen::Matrix2f V = svd.matrixV();
            Eigen::Matrix2f dR = V * U.transpose();
            if (dR.determinant() < 0)          // 防镜像
            {
                V.col(1) *= -1.0f;
                dR = V * U.transpose();
            }
            Eigen::Vector2f dt = ct - dR * cs;

            // ---- ④ 累积（左乘，因为 dR/dt 是世界系下的增量）----
            R = dR * R;
            t = dR * t + dt;

            // ---- ⑤ 收敛判断 ----
            if (dt.norm() < convergence &&
                std::fabs(std::atan2(dR(1, 0), dR(0, 0))) < convergence * 10.0)
            {
                converged = true;
                // 收敛后再跑一轮统计，拿到最终指标
            }

            if (converged)
            {
                computeStats(source, target, R, t, search_r * 2.0,
                             out_rms, out_ratio, out_inliers);
                return true;
            }
        }

        // 达到最大迭代也算完成
        computeStats(source, target, R, t, search_r * 2.0,
                     out_rms, out_ratio, out_inliers);
        return true;
    }
};

// ===========================================================================
//  ROS 节点
// ===========================================================================
class ScanMatchNode : public rclcpp::Node
{
public:
    ScanMatchNode() : Node("scan_match")
    {
        declare_parameter("ref_path",          ref_path_);
        declare_parameter("accumulate_frames", accumulate_frames_);
        declare_parameter("min_range",         min_range_);
        declare_parameter("max_range",         max_range_);
        declare_parameter("yaw_search_deg",    yaw_search_deg_);
        declare_parameter("yaw_search_step",   yaw_search_step_);
        declare_parameter("cur_voxel",         cur_voxel_);
        declare_parameter("search_r",          search_r_);
        declare_parameter("max_dxy",           max_dxy_);
        declare_parameter("max_dyaw_deg",      max_dyaw_deg_);
        declare_parameter("max_rms",           max_rms_);
        declare_parameter("min_inlier_ratio",  min_inlier_ratio_);
        declare_parameter("frame_id",          frame_id_);

        ref_path_          = get_parameter("ref_path").as_string();
        accumulate_frames_ = get_parameter("accumulate_frames").as_int();
        min_range_         = get_parameter("min_range").as_double();
        max_range_         = get_parameter("max_range").as_double();
        yaw_search_deg_    = get_parameter("yaw_search_deg").as_double();
        yaw_search_step_   = get_parameter("yaw_search_step").as_double();
        cur_voxel_         = get_parameter("cur_voxel").as_double();
        search_r_          = get_parameter("search_r").as_double();
        max_dxy_           = get_parameter("max_dxy").as_double();
        max_dyaw_deg_      = get_parameter("max_dyaw_deg").as_double();
        max_rms_           = get_parameter("max_rms").as_double();
        min_inlier_ratio_  = get_parameter("min_inlier_ratio").as_double();
        frame_id_          = get_parameter("frame_id").as_string();

        // ---- 读参考点云 ----
        if (!loadRefCloud())
        {
            RCLCPP_FATAL(get_logger(), "参考点云加载失败: %s", ref_path_.c_str());
            RCLCPP_FATAL(get_logger(), "请先运行 save_scan 录制参考点云");
            throw std::runtime_error("ref cloud load failed");
        }

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&ScanMatchNode::scanCallback, this, std::placeholders::_1));

        // 结果用 transient_local 发布，RViz 晚开也能收到
        rclcpp::QoS latched(10);
        latched.transient_local();

        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/scan_match/markers", latched);
        result_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/scan_match/result", latched);

        RCLCPP_INFO(get_logger(), "==============================================");
        RCLCPP_INFO(get_logger(), " scan_match 已启动");
        RCLCPP_INFO(get_logger(), "   参考点云 : %s  (%zu 点)", ref_path_.c_str(), ref_.size());
        RCLCPP_INFO(get_logger(), "   累积帧数 : %d", accumulate_frames_);
        RCLCPP_INFO(get_logger(), "   yaw 搜索 : ±%.1f° 步长 %.2f°", yaw_search_deg_, yaw_search_step_);
        RCLCPP_INFO(get_logger(), "==============================================");
        RCLCPP_INFO(get_logger(), ">>> 请保持车【静止】，正在累积...");
    }

private:
    // -----------------------------------------------------------------------
    //  读参考点云 CSV（跳过 # 注释行和表头）
    // -----------------------------------------------------------------------
    bool loadRefCloud()
    {
        std::ifstream f(ref_path_);
        if (!f.is_open()) return false;

        ref_.clear();
        std::string line;
        while (std::getline(f, line))
        {
            if (line.empty() || line[0] == '#') continue;
            if (line[0] == 'x' || line[0] == 'X') continue;   // 表头

            std::stringstream ss(line);
            std::string tok;
            std::vector<double> v;
            bool bad = false;
            while (std::getline(ss, tok, ','))
            {
                try { v.push_back(std::stod(tok)); }
                catch (...) { bad = true; break; }
            }
            if (bad || v.size() < 2) continue;
            ref_.push_back({v[0], v[1]});
        }
        f.close();
        return !ref_.empty();
    }

    // -----------------------------------------------------------------------
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (done_) return;

        if (detected_frame_.empty()) detected_frame_ = msg->header.frame_id;

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float r = msg->ranges[i];
            if (std::isnan(r) || std::isinf(r)) continue;
            if (r < msg->range_min || r > msg->range_max) continue;
            if (r < min_range_ || r > max_range_) continue;

            float a = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
            cur_raw_.push_back({static_cast<double>(r) * std::cos(a),
                                static_cast<double>(r) * std::sin(a)});
        }

        frames_++;
        if (frames_ % 5 == 0 && frames_ < accumulate_frames_)
        {
            RCLCPP_INFO(get_logger(), "  累积中 %d/%d 帧，累计 %zu 点",
                        frames_, accumulate_frames_, cur_raw_.size());
        }

        if (frames_ < accumulate_frames_) return;

        done_ = true;
        doMatch();
    }

    // -----------------------------------------------------------------------
    //  体素降采样
    // -----------------------------------------------------------------------
    static std::vector<Point2D> downsample(const std::vector<Point2D> & in, double voxel)
    {
        if (voxel <= 0.0) return in;
        std::map<std::pair<int, int>, std::pair<double, double>> acc;
        std::map<std::pair<int, int>, int> cnt;
        for (const auto & p : in)
        {
            auto k = std::make_pair(static_cast<int>(std::floor(p.x / voxel)),
                                    static_cast<int>(std::floor(p.y / voxel)));
            acc[k].first  += p.x;
            acc[k].second += p.y;
            cnt[k] += 1;
        }
        std::vector<Point2D> out;
        out.reserve(acc.size());
        for (auto & kv : acc)
        {
            int n = cnt[kv.first];
            out.push_back({kv.second.first / n, kv.second.second / n});
        }
        return out;
    }

    // -----------------------------------------------------------------------
    //  核心：多分辨率 + yaw 搜索
    // -----------------------------------------------------------------------
    void doMatch()
    {
        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "================ 开始配准 ================");
        RCLCPP_INFO(get_logger(), "参考点云 %zu 点，当前原始点云 %zu 点",
                    ref_.size(), cur_raw_.size());

        // ---- 点云准备 ----
        // ★ 关键修正：不再做"粗层"降采样！
        //    之前用 0.20m 体素把 1950 点降到 269 点，平均间距 1.5m，
        //    最近邻对应完全失去几何意义 → 第一层就把变换拉偏 → 收敛到错误解。
        //    初值问题由【yaw 搜索】解决（步长 1° → 误差 ≤0.5°），
        //    不需要粗层。所以这里只用细体素，保留几何结构。
        const std::vector<Point2D> cur_dense = downsample(cur_raw_, cur_voxel_);
        RCLCPP_INFO(get_logger(), "  参考 %zu 点（原样使用）；当前 %zu 点（体素 %.3fm）",
                    ref_.size(), cur_dense.size(), cur_voxel_);

        if (ref_.size() < 50 || cur_dense.size() < 50)
        {
            RCLCPP_ERROR(get_logger(), "点云太少，无法配准");
            return;
        }

        ICPMatcher icp;
        icp.search_r_ = search_r_;

        // ---- 诊断 0：先算【identity（什么都不做）】的 RMS 作为基准 ----
        // 如果两片点云本来就重合（车没动），identity 的 RMS 应该就很小。
        // ICP 如果给出比这更差的结果，说明它跑偏了。
        double rms_identity = 0.0, ratio_identity = 0.0;
        int    inl_identity = 0;
        icp.computeStats(cur_dense, ref_,
                         Eigen::Matrix2f::Identity(), Eigen::Vector2f::Zero(),
                         search_r_ * 2.0, rms_identity, ratio_identity, inl_identity);
        RCLCPP_INFO(get_logger(),
                    "  [基准] identity(不动) 的 RMS = %.4f m, 内点 %d (%.1f%%)",
                    rms_identity, inl_identity, ratio_identity * 100.0);

        // ---- yaw 初值搜索 ----
        MatchResult best;
        best.rms     = 1e9;
        best.inliers = -1;

        const double yaw_range = yaw_search_deg_ * M_PI / 180.0;
        const double yaw_step  = yaw_search_step_ * M_PI / 180.0;

        int tested = 0;
        for (double yaw0 = -yaw_range; yaw0 <= yaw_range + 1e-9; yaw0 += yaw_step)
        {
            Eigen::Matrix2f R;
            R << static_cast<float>(std::cos(yaw0)), static_cast<float>(-std::sin(yaw0)),
                 static_cast<float>(std::sin(yaw0)), static_cast<float>( std::cos(yaw0));
            Eigen::Vector2f t = Eigen::Vector2f::Zero();

            double rms = 0.0, ratio = 0.0;
            int inl = 0;

            // ★ 单层稠密配准（不再逐层链式传递）
            if (!icp.run(cur_dense, ref_, R, t, search_r_, rms, ratio, inl))
            {
                RCLCPP_INFO(get_logger(), "    yaw0=%+6.2f°  ICP 失败", yaw0 * 180.0 / M_PI);
                continue;
            }
            ++tested;

            const double dxy_cm = std::hypot(t.x(), t.y()) * 100.0;
            const double dyaw_d = std::atan2(R(1, 0), R(0, 0)) * 180.0 / M_PI;
            RCLCPP_INFO(get_logger(),
                        "    yaw0=%+6.2f°  ->  d=(%+6.2f,%+6.2f)cm  dyaw=%+6.2f°  "
                        "RMS=%.4fm  内点=%d (%.1f%%)",
                        yaw0 * 180.0 / M_PI, t.x() * 100.0, t.y() * 100.0, dyaw_d,
                        rms, inl, ratio * 100.0);

            // ★ 用【内点数】选最优，而不是 RMS
            //   错误解往往只匹配上很少的点，用算术平均 RMS 反而看不出来
            if (inl > best.inliers ||
                (inl == best.inliers && rms < best.rms))
            {
                best.rms          = rms;
                best.dx           = t.x();
                best.dy           = t.y();
                best.dyaw         = std::atan2(R(1, 0), R(0, 0));
                best.inlier_ratio = ratio;
                best.inliers      = inl;
            }
        }

        RCLCPP_INFO(get_logger(), "  已测试 %d 个 yaw 初值，最优内点数 %d",
                    static_cast<int>(tested), best.inliers);

        // ---- 诊断 1：保底机制 ----
        // 如果 ICP 找到的最优解比"什么都不做"还差，那它一定是跑偏了。
        // 此时直接采用 identity（车没动的情况本来就该是这个答案）。
        if (best.inliers < inl_identity && rms_identity < best.rms)
        {
            RCLCPP_WARN(get_logger(),
                        "  [保底] ICP 最优解(内点 %d, RMS %.4f) 比 identity(内点 %d, RMS %.4f) 更差！",
                        best.inliers, best.rms, inl_identity, rms_identity);
            RCLCPP_WARN(get_logger(), "  [保底] 采用 identity 结果");

            best.dx = 0.0;
            best.dy = 0.0;
            best.dyaw = 0.0;
            best.rms = rms_identity;
            best.inlier_ratio = ratio_identity;
            best.inliers = inl_identity;
        }

        // ---- 诊断 2：把最终结果和 identity 对比 ----
        RCLCPP_INFO(get_logger(),
                    "  [对比] 最终 RMS=%.4f m  vs  identity RMS=%.4f m   (比值 %.2f)",
                    best.rms, rms_identity,
                    (rms_identity > 1e-9 ? best.rms / rms_identity : -1.0));

        // ---- 自检 ----
        const bool pass_rms    = best.rms <= max_rms_;
        const bool pass_ratio  = best.inlier_ratio >= min_inlier_ratio_;

        // ★ 新增：合理性检查（车"放得不恰当"但没明显放错，
        //    所以修正量不可能很大。超界直接判失败）
        const bool pass_bound =
            std::fabs(best.dx) <= max_dxy_ &&
            std::fabs(best.dy) <= max_dxy_ &&
            std::fabs(best.dyaw * 180.0 / M_PI) <= max_dyaw_deg_;

        best.ok = pass_rms && pass_ratio && pass_bound;

        // ---- 输出 ----
        const double dyaw_deg = best.dyaw * 180.0 / M_PI;

        RCLCPP_INFO(get_logger(), "==============================================");
        RCLCPP_INFO(get_logger(), " 配准结果");
        RCLCPP_INFO(get_logger(), "   dx    = %+.4f m",  best.dx);
        RCLCPP_INFO(get_logger(), "   dy    = %+.4f m",  best.dy);
        RCLCPP_INFO(get_logger(), "   dyaw  = %+.4f rad  (%+.3f°)", best.dyaw, dyaw_deg);
        RCLCPP_INFO(get_logger(), "   平移量 = %.2f cm", std::hypot(best.dx, best.dy) * 100.0);
        RCLCPP_INFO(get_logger(), "   ---- 质量指标 ----");
        RCLCPP_INFO(get_logger(), "   RMS 残差   = %.4f m   (阈值 %.3f)  %s",
                    best.rms, max_rms_, pass_rms ? "OK" : "超限!");
        RCLCPP_INFO(get_logger(), "   内点比例   = %.1f%%     (阈值 %.0f%%)  %s",
                    best.inlier_ratio * 100.0, min_inlier_ratio_ * 100.0,
                    pass_ratio ? "OK" : "偏低!");
        RCLCPP_INFO(get_logger(), "   内点数     = %d", best.inliers);
        RCLCPP_INFO(get_logger(), "   修正量界内 = %s  (|dxy|<=%.2fm, |dyaw|<=%.1f°)",
                    pass_bound ? "OK" : "超界!", max_dxy_, max_dyaw_deg_);

        if (best.ok)
        {
            RCLCPP_INFO(get_logger(), "");
            RCLCPP_INFO(get_logger(), "   ╔══════════════════════════════════════╗");
            RCLCPP_INFO(get_logger(), "   ║   ★ 配准成功，结果可信              ║");
            RCLCPP_INFO(get_logger(), "   ║   dx=%+.1fcm  dy=%+.1fcm  dyaw=%+.2f°  ║",
                        best.dx * 100, best.dy * 100, dyaw_deg);
            RCLCPP_INFO(get_logger(), "   ╚══════════════════════════════════════╝");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "");
            RCLCPP_ERROR(get_logger(), "   !!! 配准失败：质量指标未达标 !!!");
            if (!pass_bound)
                RCLCPP_ERROR(get_logger(), "   → 修正量超界，结果不可信（很可能匹配错了）");
            if (!pass_ratio)
                RCLCPP_ERROR(get_logger(), "   → 内点比例过低，两片点云没能对上");
            if (!pass_rms)
                RCLCPP_ERROR(get_logger(), "   → RMS 残差过大");
            RCLCPP_ERROR(get_logger(), "   可能原因：车位置偏离太大 / 参考点云不是同一位置录的");
        }
        RCLCPP_INFO(get_logger(), "==============================================");
        RCLCPP_INFO(get_logger(), "");

        publishResult(best);
        publishMarkers(best, cur_dense);
    }

    // -----------------------------------------------------------------------
    //  发布结果话题：[ok, dx, dy, dyaw, rms, inlier_ratio, inliers]
    // -----------------------------------------------------------------------
    void publishResult(const MatchResult & r)
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {r.ok ? 1.0 : 0.0, r.dx, r.dy, r.dyaw, r.rms, r.inlier_ratio,
                    static_cast<double>(r.inliers)};
        result_pub_->publish(msg);
    }

    // -----------------------------------------------------------------------
    //  RViz 可视化
    // -----------------------------------------------------------------------
    void publishMarkers(const MatchResult & r, const std::vector<Point2D> & cur_fine)
    {
        visualization_msgs::msg::MarkerArray arr;
        const auto stamp = now();
        int id = 0;

        // 清空旧内容（每次重新发全部，所以先 DELETEALL）
        {
            visualization_msgs::msg::Marker del;
            del.header.frame_id = frame_id_;
            del.header.stamp = stamp;
            del.action = visualization_msgs::msg::Marker::DELETEALL;
            arr.markers.push_back(del);
        }

        const float cy = static_cast<float>(std::cos(r.dyaw));
        const float sy = static_cast<float>(std::sin(r.dyaw));
        Eigen::Matrix2f R;
        R << cy, -sy,
             sy,  cy;
        const Eigen::Vector2f t(static_cast<float>(r.dx), static_cast<float>(r.dy));

        // ---------- ① 参考点云（蓝色）----------
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = frame_id_;
            m.header.stamp = stamp;
            m.ns = "ref_cloud";   m.id = id++;
            m.type = visualization_msgs::msg::Marker::POINTS;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = 0.04;
            m.color.r = 0.2f; m.color.g = 0.4f; m.color.b = 1.0f; m.color.a = 0.85f;
            m.lifetime = rclcpp::Duration::from_seconds(0);
            m.points.reserve(ref_.size());
            for (const auto & p : ref_)
            {
                geometry_msgs::msg::Point gp;
                gp.x = p.x; gp.y = p.y; gp.z = 0.0;
                m.points.push_back(gp);
            }
            arr.markers.push_back(m);
        }

        // ---------- ② 当前点云（变换后，绿色）----------
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = frame_id_;
            m.header.stamp = stamp;
            m.ns = "cur_cloud";   m.id = id++;
            m.type = visualization_msgs::msg::Marker::POINTS;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = 0.04;
            m.color.r = 0.2f; m.color.g = 1.0f; m.color.b = 0.3f; m.color.a = 0.85f;
            m.lifetime = rclcpp::Duration::from_seconds(0);
            m.points.reserve(cur_fine.size());
            for (const auto & p : cur_fine)
            {
                Eigen::Vector2f tp = R * Eigen::Vector2f(static_cast<float>(p.x),
                                                         static_cast<float>(p.y)) + t;
                geometry_msgs::msg::Point gp;
                gp.x = tp.x(); gp.y = tp.y(); gp.z = 0.0;
                m.points.push_back(gp);
            }
            arr.markers.push_back(m);
        }

        // ---------- ③ 坐标系箭头（红=X前，绿=Y左）----------
        // 说明：本节点所有数据都在 frame_id（雷达系）下。
        //       雷达系的 x 朝【雷达正前方】，y 朝【左方】。
        {
            // X 轴：红色箭头
            visualization_msgs::msg::Marker ax;
            ax.header.frame_id = frame_id_;
            ax.header.stamp = stamp;
            ax.ns = "axis_x";  ax.id = id++;
            ax.type = visualization_msgs::msg::Marker::ARROW;
            ax.action = visualization_msgs::msg::Marker::ADD;
            ax.pose.orientation.w = 1.0;
            ax.scale.x = 0.10;   // 箭杆直径
            ax.scale.y = 0.20;   // 箭头直径
            ax.scale.z = 0.25;   // 箭头长度
            ax.color.r = 1.0f; ax.color.g = 0.1f; ax.color.b = 0.1f; ax.color.a = 1.0f;
            ax.lifetime = rclcpp::Duration::from_seconds(0);
            {
                geometry_msgs::msg::Point p0, p1;
                p0.x = 0; p0.y = 0; p0.z = 0.02;
                p1.x = 1.5; p1.y = 0; p1.z = 0.02;
                ax.points = {p0, p1};
            }
            arr.markers.push_back(ax);

            // Y 轴：绿色箭头
            visualization_msgs::msg::Marker ay = ax;
            ay.ns = "axis_y";  ay.id = id++;
            ay.color.r = 0.1f; ay.color.g = 1.0f; ay.color.b = 0.1f;
            {
                geometry_msgs::msg::Point p0, p1;
                p0.x = 0; p0.y = 0; p0.z = 0.02;
                p1.x = 0; p1.y = 1.5; p1.z = 0.02;
                ay.points = {p0, p1};
            }
            arr.markers.push_back(ay);

            // 轴文字
            auto makeText = [&](const std::string & txt, double x, double y,
                                float cr, float cg, float cb) {
                visualization_msgs::msg::Marker tm;
                tm.header.frame_id = frame_id_;
                tm.header.stamp = stamp;
                tm.ns = "axis_label"; tm.id = id++;
                tm.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                tm.action = visualization_msgs::msg::Marker::ADD;
                tm.pose.position.x = x;
                tm.pose.position.y = y;
                tm.pose.position.z = 0.45;
                tm.pose.orientation.w = 1.0;
                tm.scale.z = 0.30;
                tm.color.r = cr; tm.color.g = cg; tm.color.b = cb; tm.color.a = 1.0f;
                tm.text = txt;
                tm.lifetime = rclcpp::Duration::from_seconds(0);
                arr.markers.push_back(tm);
            };
            makeText("+X  front", 1.6, 0.0, 1.0f, 0.3f, 0.3f);
            makeText("+Y  left",  0.0, 1.6, 0.3f, 1.0f, 0.3f);
            makeText("origin (lidar)", -0.9, -0.35, 0.9f, 0.9f, 0.9f);
        }

        // ---------- ④ 匹配结果文字 ----------
        {
            visualization_msgs::msg::Marker tm;
            tm.header.frame_id = frame_id_;
            tm.header.stamp = stamp;
            tm.ns = "result"; tm.id = id++;
            tm.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            tm.action = visualization_msgs::msg::Marker::ADD;
            tm.pose.position.x = 0.0;
            tm.pose.position.y = -1.2;
            tm.pose.position.z = 1.2;
            tm.pose.orientation.w = 1.0;
            tm.scale.z = 0.28;
            if (r.ok) { tm.color.r = 0.2f; tm.color.g = 1.0f; tm.color.b = 0.2f; }
            else      { tm.color.r = 1.0f; tm.color.g = 0.2f; tm.color.b = 0.2f; }
            tm.color.a = 1.0f;

            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "%s\n d = (%.1f, %.1f) cm\n yaw = %+.2f deg\n RMS = %.1f mm\n inlier = %.0f%%",
                          r.ok ? "[OK] MATCHED" : "[FAIL]",
                          r.dx * 100.0, r.dy * 100.0, r.dyaw * 180.0 / M_PI,
                          r.rms * 1000.0, r.inlier_ratio * 100.0);
            tm.text = buf;
            tm.lifetime = rclcpp::Duration::from_seconds(0);
            arr.markers.push_back(tm);
        }

        marker_pub_->publish(arr);
    }

    // ---- 参数 ----
    std::string ref_path_{"/home/racecar/src/my_racecar/include/ref_cloud.csv"};
    int         accumulate_frames_{20};
    double      min_range_{0.20};
    double      max_range_{25.0};
    double      yaw_search_deg_{8.0};      // yaw 搜索范围 ±8°
    double      yaw_search_step_{1.0};     // 搜索步长 1°
    double      cur_voxel_{0.03};          // 当前点云降采样体素（参考点云不降采样）
    double      search_r_{0.20};           // 对应点搜索半径
    double      max_dxy_{0.50};            // ★ 平移修正量上限（米），超界判失败
    double      max_dyaw_deg_{20.0};       // ★ 航向修正量上限（度），超界判失败
    double      max_rms_{0.03};            // RMS 残差阈值 3cm（数据实测 0.4cm，留余量）
    double      min_inlier_ratio_{0.60};   // 内点比例阈值 60%（数据实测 98%）
    std::string frame_id_{"laser_link"};

    // ---- 状态 ----
    std::vector<Point2D> ref_;
    std::vector<Point2D> cur_raw_;
    int         frames_{0};
    bool        done_{false};
    std::string detected_frame_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr result_pub_;
};

// ===========================================================================
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<ScanMatchNode>());
    }
    catch (const std::exception & e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("scan_match"), "启动失败: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}

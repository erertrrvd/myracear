// ============================================================================
//  cone_match.cpp —— 锥桶配准求初始位姿
// ============================================================================
//  为什么用锥桶而不是原始点云：
//      原始点云 ICP 的失败原因（实测验证过）：
//        ① 雷达点云是离散采样 → 两次扫描的点落在不同角度上，即使完美对齐，
//           最近邻距离也有 1~2cm 的地板
//        ② 点云里 90% 不是锥桶（地面反射、远处墙壁、人）→ 噪声主导 RMS
//        ③ 车只差 20cm 时两次点云完全"交错" → 大部分点配错
//
//      锥桶方案避开了全部三个问题：
//        ① 锥桶间距约 1m → 质心不会互相混淆
//        ② 质心是几十个点的平均 → 精度 1~2cm
//        ③ 地面/墙/人的散点不参与运算
//
//  算法：
//      ① 从参考点云 / 当前点云里各提取锥桶位置（复用 cone_detect 的聚类）
//      ② 用【成对距离指纹】建立对应关系（距离是旋转平移不变量 → 不需要初值）
//      ③ SVD 求刚体变换
//      ④ 残差检查 + 界内检查
//
//  用法：
//      # 第一步：录参考点云（车静止）
//      ros2 launch my_racecar save_scan.launch.py
//
//      # 第二步：匹配
//      ros2 launch my_racecar cone_match.launch.py
//      ros2 launch my_racecar cone_match_rviz.launch.py
//
//  RViz 会显示：
//      🔵 蓝色球 = 参考锥桶
//      🟢 绿色球 = 当前锥桶（已按结果变换）
//      🟡 黄色连线 = 对应关系（配对成功的锥桶两两相连）
//      🔴 红色箭头 = +X（雷达正前方）
//      🟢 绿色箭头 = +Y（雷达左方）
//      🟡 结果文字 = dx / dy / dyaw / 残差 / 配对数
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <numeric>
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

// 一个锥桶（位置 + 质量信息）
struct Cone
{
    double cx{0.0};
    double cy{0.0};
    double range{0.0};
    double width{0.0};
    int    n_points{0};

    double distTo(const Cone & o) const
    {
        return std::hypot(cx - o.cx, cy - o.cy);
    }
};

// ===========================================================================
//  锥桶提取（和 cone_detect.cpp 同一套聚类逻辑）
// ===========================================================================
class ConeExtractor
{
public:
    struct Config
    {
        double min_range  = 0.30;
        double max_range  = 8.00;
        double cluster_tol = 0.25;
        int    min_points = 4;
        int    max_points = 200;
        double min_width  = 0.03;
        double max_width  = 0.35;
        int    max_cones  = 12;    // ★ 最多用几个锥桶做配准（比 detect 时多，用足信息）
    };
    Config cfg;

    // 从一批点里提锥桶
    std::vector<Cone> extract(const std::vector<Point2D> & pts_in)
    {
        std::vector<Cone> result;
        if (pts_in.size() < static_cast<size_t>(cfg.min_points)) return result;

        std::vector<Point2D> pts(pts_in);
        std::sort(pts.begin(), pts.end(),
                  [](const Point2D & a, const Point2D & b) {
                      return a.x < b.x || (a.x == b.x && a.y < b.y);
                  });

        const int n = static_cast<int>(pts.size());
        std::vector<int> parent(n);
        std::iota(parent.begin(), parent.end(), 0);

        std::function<int(int)> find = [&](int i) -> int {
            while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; }
            return i;
        };
        auto unite = [&](int a, int b) {
            int ra = find(a), rb = find(b);
            if (ra != rb) parent[ra] = rb;
        };

        const double tol2 = cfg.cluster_tol * cfg.cluster_tol;
        for (int i = 0; i < n; ++i)
        {
            for (int j = i + 1; j < n; ++j)
            {
                if (pts[j].x - pts[i].x > cfg.cluster_tol) break;
                double dx = pts[j].x - pts[i].x;
                double dy = pts[j].y - pts[i].y;
                if (dx * dx + dy * dy <= tol2) unite(i, j);
            }
        }

        std::map<int, std::vector<int>> groups;
        for (int i = 0; i < n; ++i) groups[find(i)].push_back(i);

        for (auto & kv : groups)
        {
            const std::vector<int> & idxs = kv.second;
            const int np = static_cast<int>(idxs.size());
            if (np < cfg.min_points) continue;
            if (np > cfg.max_points) continue;

            Cone c;
            c.n_points = np;

            double sx = 0, sy = 0;
            for (int k : idxs) { sx += pts[k].x; sy += pts[k].y; }
            c.cx = sx / np;
            c.cy = sy / np;
            c.range = std::hypot(c.cx, c.cy);

            double maxd2 = 0;
            for (size_t a = 0; a < idxs.size(); ++a)
                for (size_t b = a + 1; b < idxs.size(); ++b)
                {
                    double dx = pts[idxs[a]].x - pts[idxs[b]].x;
                    double dy = pts[idxs[a]].y - pts[idxs[b]].y;
                    double d2 = dx * dx + dy * dy;
                    if (d2 > maxd2) maxd2 = d2;
                }
            c.width = std::sqrt(maxd2);

            if (c.width < cfg.min_width) continue;
            if (c.width > cfg.max_width) continue;

            result.push_back(c);
        }

        std::sort(result.begin(), result.end(),
                  [](const Cone & a, const Cone & b) { return a.range < b.range; });
        if (static_cast<int>(result.size()) > cfg.max_cones)
            result.resize(cfg.max_cones);

        return result;
    }
};

// ===========================================================================
//  结果
// ===========================================================================
struct MatchResult
{
    double dx{0.0}, dy{0.0}, dyaw{0.0};
    double rms{0.0};
    int    n_pairs{0};
    bool   ok{false};
    std::vector<std::pair<int, int>> pairs;   // (ref_idx, cur_idx)
};

// ===========================================================================
//  ROS 节点
// ===========================================================================
class ConeMatchNode : public rclcpp::Node
{
public:
    ConeMatchNode() : Node("cone_match")
    {
        // ---- 参数 ----
        declare_parameter("ref_path",          ref_path_);
        declare_parameter("accumulate_frames", accumulate_frames_);
        declare_parameter("cluster_tol",       cfg_.cluster_tol);
        declare_parameter("min_points",        cfg_.min_points);
        declare_parameter("max_points",        cfg_.max_points);
        declare_parameter("min_width",         cfg_.min_width);
        declare_parameter("max_width",         cfg_.max_width);
        declare_parameter("max_range",         cfg_.max_range);
        declare_parameter("max_cones",         cfg_.max_cones);
        declare_parameter("min_pairs",         min_pairs_);
        declare_parameter("sig_tol",           sig_tol_);
        declare_parameter("max_rms",           max_rms_);
        declare_parameter("max_dxy",           max_dxy_);
        declare_parameter("max_dyaw_deg",      max_dyaw_deg_);
        declare_parameter("frame_id",          frame_id_);

        ref_path_          = get_parameter("ref_path").as_string();
        accumulate_frames_ = get_parameter("accumulate_frames").as_int();
        cfg_.cluster_tol   = get_parameter("cluster_tol").as_double();
        cfg_.min_points    = get_parameter("min_points").as_int();
        cfg_.max_points    = get_parameter("max_points").as_int();
        cfg_.min_width     = get_parameter("min_width").as_double();
        cfg_.max_width     = get_parameter("max_width").as_double();
        cfg_.max_range     = get_parameter("max_range").as_double();
        cfg_.max_cones     = get_parameter("max_cones").as_int();
        min_pairs_         = get_parameter("min_pairs").as_int();
        sig_tol_           = get_parameter("sig_tol").as_double();
        max_rms_           = get_parameter("max_rms").as_double();
        max_dxy_           = get_parameter("max_dxy").as_double();
        max_dyaw_deg_      = get_parameter("max_dyaw_deg").as_double();
        frame_id_          = get_parameter("frame_id").as_string();

        extractor_.cfg = cfg_;

        // ---- 读参考点云 → 提锥桶 ----
        std::vector<Point2D> ref_pts;
        if (!loadRefCloud(ref_pts))
        {
            RCLCPP_FATAL(get_logger(), "参考点云加载失败: %s", ref_path_.c_str());
            RCLCPP_FATAL(get_logger(), "请先运行 save_scan 录制参考点云");
            throw std::runtime_error("ref cloud load failed");
        }

        ref_cones_ = extractor_.extract(ref_pts);
        RCLCPP_INFO(get_logger(), "==============================================");
        RCLCPP_INFO(get_logger(), " cone_match 已启动");
        RCLCPP_INFO(get_logger(), "   参考点云 : %s (%zu 点)", ref_path_.c_str(), ref_pts.size());
        RCLCPP_INFO(get_logger(), "   参考锥桶 : %zu 个", ref_cones_.size());
        for (size_t i = 0; i < ref_cones_.size(); ++i)
            RCLCPP_INFO(get_logger(), "      ref#%zu (%7.3f, %7.3f)  r=%.2f w=%.3f n=%d",
                        i, ref_cones_[i].cx, ref_cones_[i].cy,
                        ref_cones_[i].range, ref_cones_[i].width, ref_cones_[i].n_points);
        RCLCPP_INFO(get_logger(), "==============================================");

        if (ref_cones_.size() < 3)
        {
            RCLCPP_FATAL(get_logger(), "参考锥桶少于 3 个，无法配准");
            throw std::runtime_error("too few ref cones");
        }

        // ---- 订阅 ----
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&ConeMatchNode::scanCallback, this, std::placeholders::_1));

        rclcpp::QoS latched(10);
        latched.transient_local();
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/cone_match/markers", latched);
        result_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/cone_match/result", latched);

        RCLCPP_INFO(get_logger(), ">>> 请保持车【静止】，正在累积 %d 帧...", accumulate_frames_);
    }

private:
    // -----------------------------------------------------------------------
    bool loadRefCloud(std::vector<Point2D> & out)
    {
        std::ifstream f(ref_path_);
        if (!f.is_open()) return false;

        out.clear();
        std::string line;
        while (std::getline(f, line))
        {
            if (line.empty() || line[0] == '#' || line[0] == 'x' || line[0] == 'X') continue;
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
            out.push_back({v[0], v[1]});
        }
        f.close();
        return !out.empty();
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
            if (r < cfg_.min_range || r > cfg_.max_range) continue;

            float a = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
            cur_pts_.push_back({static_cast<double>(r) * std::cos(a),
                                static_cast<double>(r) * std::sin(a)});
        }

        frames_++;
        if (frames_ % 5 == 0 && frames_ < accumulate_frames_)
            RCLCPP_INFO(get_logger(), "  累积中 %d/%d 帧，累计 %zu 点",
                        frames_, accumulate_frames_, cur_pts_.size());

        if (frames_ < accumulate_frames_) return;

        done_ = true;
        doMatch();
    }

    // -----------------------------------------------------------------------
    //  ★ 核心：成对距离指纹 + SVD
    // -----------------------------------------------------------------------
    void doMatch()
    {
        auto cur_cones = extractor_.extract(cur_pts_);

        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "================ 锥桶配准 ================");
        RCLCPP_INFO(get_logger(), "参考锥桶 %zu 个，当前锥桶 %zu 个",
                    ref_cones_.size(), cur_cones.size());
        for (size_t i = 0; i < cur_cones.size(); ++i)
            RCLCPP_INFO(get_logger(), "      cur#%zu (%7.3f, %7.3f)  r=%.2f w=%.3f n=%d",
                        i, cur_cones[i].cx, cur_cones[i].cy,
                        cur_cones[i].range, cur_cones[i].width, cur_cones[i].n_points);

        const int N = static_cast<int>(ref_cones_.size());
        const int M = static_cast<int>(cur_cones.size());

        if (N < 3 || M < 3)
        {
            RCLCPP_ERROR(get_logger(), " 锥桶太少（参考 %d, 当前 %d），无法配准", N, M);
            publishAll(MatchResult{}, cur_cones);
            return;
        }

        // ---- 参考锥桶的成对距离（指纹基准）----
        // ref_dist[i][j]
        std::vector<std::vector<double>> ref_d(N, std::vector<double>(N, 0.0));
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j)
                ref_d[i][j] = ref_cones_[i].distTo(ref_cones_[j]);

        // ---- 枚举当前锥桶的所有 3 元组合，和参考的 3 元组合比对 ----
        // 用 3 点就能定 SE(2)（6 个约束 vs 3 个自由度）
        // 3 个点的 3 条边长构成指纹，很短很快
        struct Best
        {
            double err{1e9};
            std::vector<std::pair<int,int>> pairs;   // (ref_idx, cur_idx)
        } best;

        int tested = 0;

        // 参考的 3 元组合
        for (int ri = 0; ri < N; ++ri)
        for (int rj = ri + 1; rj < N; ++rj)
        for (int rk = rj + 1; rk < N; ++rk)
        {
            const double d_ij = ref_d[ri][rj];
            const double d_ik = ref_d[ri][rk];
            const double d_jk = ref_d[rj][rk];

            // 当前点云的 3 元组合
            for (int ci = 0; ci < M; ++ci)
            for (int cj = ci + 1; cj < M; ++cj)
            for (int ck = cj + 1; ck < M; ++ck)
            {
                ++tested;

                const double e_ij = cur_cones[ci].distTo(cur_cones[cj]);
                const double e_ik = cur_cones[ci].distTo(cur_cones[ck]);
                const double e_jk = cur_cones[cj].distTo(cur_cones[ck]);

                // 三条边的差
                double err = std::fabs(e_ij - d_ij)
                           + std::fabs(e_ik - d_ik)
                           + std::fabs(e_jk - d_jk);

                if (err < best.err)
                {
                    best.err = err;
                    best.pairs = {{ri, ci}, {rj, cj}, {rk, ck}};
                }
            }
        }

        RCLCPP_INFO(get_logger(), "  已测试 %d 个 3 元组合，最小边长误差 = %.4f m",
                    tested, best.err);

        // ---- 用指纹匹配扩展到更多点 ----
        // 把已经配上的 3 个点当作"锚"，用它们算出的变换去匹配其余的锥桶
        if (best.pairs.size() < 3)
        {
            RCLCPP_ERROR(get_logger(), "  配对失败");
            publishAll(MatchResult{}, cur_cones);
            return;
        }

        // 先用 3 个锚点求初始变换
        Eigen::Matrix2f R;
        Eigen::Vector2f t;
        if (!solveSVD(best.pairs, cur_cones, R, t))
        {
            RCLCPP_ERROR(get_logger(), "  SVD 求解失败");
            publishAll(MatchResult{}, cur_cones);
            return;
        }

        // ★ 扩展：把当前点云里其余锥桶按"变换后离哪个参考锥桶最近"补上
        std::vector<bool> ref_used(N, false);
        for (auto & p : best.pairs) ref_used[p.first] = true;

        const double ADD_TOL = 0.35;   // 补配的阈值（米）
        for (int c = 0; c < M; ++c)
        {
            bool already = false;
            for (auto & p : best.pairs) if (p.second == c) { already = true; break; }
            if (already) continue;

            Eigen::Vector2f pc(static_cast<float>(cur_cones[c].cx),
                               static_cast<float>(cur_cones[c].cy));
            Eigen::Vector2f pm = R * pc + t;

            int    best_r = -1;
            double best_d = ADD_TOL;
            for (int r = 0; r < N; ++r)
            {
                if (ref_used[r]) continue;
                double d = std::hypot(pm.x() - ref_cones_[r].cx,
                                      pm.y() - ref_cones_[r].cy);
                if (d < best_d) { best_d = d; best_r = r; }
            }
            if (best_r >= 0)
            {
                best.pairs.push_back({best_r, c});
                ref_used[best_r] = true;
            }
        }

        RCLCPP_INFO(get_logger(), "  最终配对 %zu 对", best.pairs.size());

        // ---- 用全部配对重解 ----
        if (!solveSVD(best.pairs, cur_cones, R, t))
        {
            RCLCPP_ERROR(get_logger(), "  最终 SVD 求解失败");
            publishAll(MatchResult{}, cur_cones);
            return;
        }

        // ---- 残差 ----
        double rms = 0.0;
        for (auto & p : best.pairs)
        {
            const Cone & rc = ref_cones_[p.first];
            const Cone & cc = cur_cones[p.second];
            Eigen::Vector2f pc(static_cast<float>(cc.cx), static_cast<float>(cc.cy));
            Eigen::Vector2f pm = R * pc + t;
            rms += std::pow(pm.x() - rc.cx, 2) + std::pow(pm.y() - rc.cy, 2);
        }
        rms = std::sqrt(rms / best.pairs.size());

        // ---- 结果 ----
        MatchResult res;
        res.dx   = t.x();
        res.dy   = t.y();
        res.dyaw = std::atan2(R(1, 0), R(0, 0));
        res.rms  = rms;
        res.n_pairs = static_cast<int>(best.pairs.size());
        res.pairs = best.pairs;

        const double dyaw_deg = res.dyaw * 180.0 / M_PI;
        const bool pass_pairs = res.n_pairs >= min_pairs_;
        const bool pass_rms   = res.rms <= max_rms_;
        const bool pass_sig   = best.err <= sig_tol_;
        const bool pass_bound = std::fabs(res.dx) <= max_dxy_ &&
                                std::fabs(res.dy) <= max_dxy_ &&
                                std::fabs(dyaw_deg) <= max_dyaw_deg_;
        res.ok = pass_pairs && pass_rms && pass_sig && pass_bound;

        RCLCPP_INFO(get_logger(), "==============================================");
        RCLCPP_INFO(get_logger(), " 配准结果");
        RCLCPP_INFO(get_logger(), "   dx     = %+.4f m", res.dx);
        RCLCPP_INFO(get_logger(), "   dy     = %+.4f m", res.dy);
        RCLCPP_INFO(get_logger(), "   dyaw   = %+.4f rad (%+.3f°)", res.dyaw, dyaw_deg);
        RCLCPP_INFO(get_logger(), "   平移量 = %.2f cm", std::hypot(res.dx, res.dy) * 100.0);
        RCLCPP_INFO(get_logger(), "   ---- 质量指标 ----");
        RCLCPP_INFO(get_logger(), "   配对数     = %d      (阈值 %d)   %s",
                    res.n_pairs, min_pairs_, pass_pairs ? "OK" : "太少!");
        RCLCPP_INFO(get_logger(), "   RMS 残差   = %.4f m (阈值 %.3f) %s",
                    res.rms, max_rms_, pass_rms ? "OK" : "超限!");
        RCLCPP_INFO(get_logger(), "   指纹误差   = %.4f m (阈值 %.3f) %s",
                    best.err, sig_tol_, pass_sig ? "OK" : "超限!");
        RCLCPP_INFO(get_logger(), "   修正量界内 = %s", pass_bound ? "OK" : "超界!");

        for (auto & p : res.pairs)
            RCLCPP_INFO(get_logger(), "      配对: ref#%d (%7.3f,%7.3f) <-> cur#%d (%7.3f,%7.3f)",
                        p.first, ref_cones_[p.first].cx, ref_cones_[p.first].cy,
                        p.second, cur_cones[p.second].cx, cur_cones[p.second].cy);

        if (res.ok)
        {
            RCLCPP_INFO(get_logger(), "");
            RCLCPP_INFO(get_logger(), "   ╔══════════════════════════════════════╗");
            RCLCPP_INFO(get_logger(), "   ║   ★ 配准成功，结果可信              ║");
            RCLCPP_INFO(get_logger(), "   ║   dx=%+.1fcm dy=%+.1fcm dyaw=%+.2f°  ║",
                        res.dx * 100, res.dy * 100, dyaw_deg);
            RCLCPP_INFO(get_logger(), "   ╚══════════════════════════════════════╝");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "");
            RCLCPP_ERROR(get_logger(), "   !!! 配准失败 !!!");
            if (!pass_sig)   RCLCPP_ERROR(get_logger(), "   → 指纹误差过大：两次看到的锥桶布局对不上");
            if (!pass_rms)   RCLCPP_ERROR(get_logger(), "   → RMS 残差过大");
            if (!pass_pairs) RCLCPP_ERROR(get_logger(), "   → 配对太少");
            if (!pass_bound) RCLCPP_ERROR(get_logger(), "   → 修正量超界");
        }
        RCLCPP_INFO(get_logger(), "==============================================");

        publishResult(res);
        publishAll(res, cur_cones);
    }

    // -----------------------------------------------------------------------
    //  SVD 求刚体变换：使 ref[pi] ≈ R * cur[ci] + t
    // -----------------------------------------------------------------------
    bool solveSVD(const std::vector<std::pair<int,int>> & pairs,
                  const std::vector<Cone> & cur_cones,
                  Eigen::Matrix2f & R, Eigen::Vector2f & t)
    {
        const int n = static_cast<int>(pairs.size());
        if (n < 3) return false;

        Eigen::Vector2f c_ref = Eigen::Vector2f::Zero();
        Eigen::Vector2f c_cur = Eigen::Vector2f::Zero();
        for (auto & p : pairs)
        {
            c_ref += Eigen::Vector2f(static_cast<float>(ref_cones_[p.first].cx),
                                     static_cast<float>(ref_cones_[p.first].cy));
            c_cur += Eigen::Vector2f(static_cast<float>(cur_cones[p.second].cx),
                                     static_cast<float>(cur_cones[p.second].cy));
        }
        c_ref /= static_cast<float>(n);
        c_cur /= static_cast<float>(n);

        Eigen::Matrix2f H = Eigen::Matrix2f::Zero();
        for (auto & p : pairs)
        {
            Eigen::Vector2f a = Eigen::Vector2f(static_cast<float>(cur_cones[p.second].cx),
                                                static_cast<float>(cur_cones[p.second].cy)) - c_cur;
            Eigen::Vector2f b = Eigen::Vector2f(static_cast<float>(ref_cones_[p.first].cx),
                                                static_cast<float>(ref_cones_[p.first].cy)) - c_ref;
            H += a * b.transpose();
        }

        Eigen::JacobiSVD<Eigen::Matrix2f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2f U = svd.matrixU();
        Eigen::Matrix2f V = svd.matrixV();
        R = V * U.transpose();
        if (R.determinant() < 0)     // 防镜像
        {
            V.col(1) *= -1.0f;
            R = V * U.transpose();
        }
        t = c_ref - R * c_cur;
        return true;
    }

    // -----------------------------------------------------------------------
    void publishResult(const MatchResult & r)
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {r.ok ? 1.0 : 0.0, r.dx, r.dy, r.dyaw, r.rms,
                    static_cast<double>(r.n_pairs)};
        result_pub_->publish(msg);
    }

    // -----------------------------------------------------------------------
    void publishAll(const MatchResult & r, const std::vector<Cone> & cur_cones)
    {
        visualization_msgs::msg::MarkerArray arr;
        const auto stamp = now();
        int id = 0;

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
        R << cy, -sy, sy, cy;
        const Eigen::Vector2f t(static_cast<float>(r.dx), static_cast<float>(r.dy));

        auto addSphere = [&](double x, double y, float cr, float cg, float cb,
                             double size, const std::string & ns) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = frame_id_;
            m.header.stamp = stamp;
            m.ns = ns; m.id = id++;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = x;
            m.pose.position.y = y;
            m.pose.position.z = 0.05;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = m.scale.z = size;
            m.color.r = cr; m.color.g = cg; m.color.b = cb; m.color.a = 0.9f;
            m.lifetime = rclcpp::Duration::from_seconds(0);
            arr.markers.push_back(m);
        };

        // 参考锥桶（蓝）
        for (const auto & c : ref_cones_)
            addSphere(c.cx, c.cy, 0.2f, 0.5f, 1.0f, 0.28, "ref_cones");

        // 当前锥桶（变换后，绿）
        for (const auto & c : cur_cones)
        {
            Eigen::Vector2f p = R * Eigen::Vector2f(static_cast<float>(c.cx),
                                                    static_cast<float>(c.cy)) + t;
            addSphere(p.x(), p.y(), 0.1f, 1.0f, 0.3f, 0.24, "cur_cones");
        }

        // 对应连线（黄）
        if (!r.pairs.empty())
        {
            visualization_msgs::msg::Marker line;
            line.header.frame_id = frame_id_;
            line.header.stamp = stamp;
            line.ns = "pairs"; line.id = id++;
            line.type = visualization_msgs::msg::Marker::LINE_LIST;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.scale.x = 0.02;
            line.color.r = 1.0f; line.color.g = 1.0f; line.color.b = 0.2f; line.color.a = 1.0f;
            line.lifetime = rclcpp::Duration::from_seconds(0);

            for (auto & p : r.pairs)
            {
                Eigen::Vector2f pc(static_cast<float>(cur_cones[p.second].cx),
                                   static_cast<float>(cur_cones[p.second].cy));
                Eigen::Vector2f pm = R * pc + t;

                geometry_msgs::msg::Point a, b;
                a.x = ref_cones_[p.first].cx; a.y = ref_cones_[p.first].cy; a.z = 0.05;
                b.x = pm.x();                 b.y = pm.y();                 b.z = 0.05;
                line.points.push_back(a);
                line.points.push_back(b);
            }
            arr.markers.push_back(line);
        }

        // 坐标轴（红 X / 绿 Y）
        auto addAxis = [&](double ex, double ey, float cr, float cg, float cb,
                           const std::string & ns) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = frame_id_;
            m.header.stamp = stamp;
            m.ns = ns; m.id = id++;
            m.type = visualization_msgs::msg::Marker::ARROW;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.10; m.scale.y = 0.20; m.scale.z = 0.25;
            m.color.r = cr; m.color.g = cg; m.color.b = cb; m.color.a = 1.0f;
            m.lifetime = rclcpp::Duration::from_seconds(0);
            geometry_msgs::msg::Point p0, p1;
            p0.x = 0; p0.y = 0; p0.z = 0.02;
            p1.x = ex; p1.y = ey; p1.z = 0.02;
            m.points = {p0, p1};
            arr.markers.push_back(m);
        };
        addAxis(1.5, 0.0, 1.0f, 0.1f, 0.1f, "axis_x");
        addAxis(0.0, 1.5, 0.1f, 1.0f, 0.1f, "axis_y");

        auto addText = [&](const std::string & s, double x, double y, double z,
                           float cr, float cg, float cb, double size) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = frame_id_;
            m.header.stamp = stamp;
            m.ns = "text"; m.id = id++;
            m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = x;
            m.pose.position.y = y;
            m.pose.position.z = z;
            m.pose.orientation.w = 1.0;
            m.scale.z = size;
            m.color.r = cr; m.color.g = cg; m.color.b = cb; m.color.a = 1.0f;
            m.text = s;
            m.lifetime = rclcpp::Duration::from_seconds(0);
            arr.markers.push_back(m);
        };

        addText("+X front", 1.6, 0.0, 0.45, 1.0f, 0.3f, 0.3f, 0.28);
        addText("+Y left",  0.0, 1.6, 0.45, 0.3f, 1.0f, 0.3f, 0.28);

        {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "%s\n dx = %+.1f cm\n dy = %+.1f cm\n dyaw = %+.2f deg\n"
                          " RMS = %.1f mm\n pairs = %d",
                          r.ok ? "[OK] MATCHED" : "[FAIL]",
                          r.dx * 100.0, r.dy * 100.0, r.dyaw * 180.0 / M_PI,
                          r.rms * 1000.0, r.n_pairs);
            const float tr = r.ok ? 0.2f : 1.0f;
            const float tg = r.ok ? 1.0f : 0.2f;
            addText(buf, 0.0, -1.4, 1.3, tr, tg, 0.2f, 0.26);
        }

        marker_pub_->publish(arr);
    }

    // ---- 参数 ----
    std::string ref_path_{"/home/racecar/src/my_racecar/include/ref_cloud.csv"};
    int         accumulate_frames_{20};
    ConeExtractor::Config cfg_;
    int         min_pairs_{3};
    double      sig_tol_{0.15};        // 指纹误差阈值（米，三条边之和）
    double      max_rms_{0.05};        // 残差阈值 5cm
    double      max_dxy_{0.60};        // 平移上限 60cm
    double      max_dyaw_deg_{25.0};   // 航向上限 25°
    std::string frame_id_{"laser_link"};

    // ---- 状态 ----
    ConeExtractor extractor_;
    std::vector<Cone> ref_cones_;
    std::vector<Point2D> cur_pts_;
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
        rclcpp::spin(std::make_shared<ConeMatchNode>());
    }
    catch (const std::exception & e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("cone_match"), "启动失败: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}

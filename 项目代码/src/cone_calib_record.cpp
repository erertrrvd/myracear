// ============================================================================
//  cone_calib_record.cpp —— 起跑点「参考锥桶」录制节点（初始点校准 · 步骤 1）
// ============================================================================
//
//  作用：
//      车停在【标准起跑点】静止不动，本节点累积若干帧雷达数据，
//      聚类出所有锥桶候选，再用【几何一致性】挑出真正的 4 个参考锥桶，
//      打印成对距离指纹 + 几何质量自检，并存成 cones_ref.csv（"标准答案"）。
//
//  ★ 与旧节点 cone_detect 的关键区别（这是本任务的核心修正）：
//      cone_detect 用「取最近的 N 个」（max_cones）来选参考锥桶。
//      实测这会选错 —— 起跑点附近有一个仅 5 点的噪声碎片，
//      它到车 0.960m，比真锥桶 0.996m 近了 3.6cm，
//      于是「取最近」把真锥桶挤掉、把噪声捞了进来。
//      每次重启噪声时有时无 → 选中的点集变化 → 解出的位姿变化，
//      这正是"两次停放位置对不上"的直接原因之一。
//
//      本节点改成【枚举所有 C(n,4) 组合，按几何质量打分选最优】：
//        分数 = 分散度比值 + (最小间距过近的惩罚) - (最小点数的微弱奖励)
//      几何质量最好的那一组，就是真正的参考锥桶。
//      （离线脚本 tools/cone_subset.js 已用实测数据验证过该策略有效）
//
//  ⚠️ 另外一个必须修正的参数：max_points
//      cone_detect 里 max_points 默认 120，但累积 20 帧后
//      【真锥桶的簇有 459~554 个点】，会被 120 直接全部滤掉 → 一个都检不出。
//      本节点默认改为 3000（墙/大物体靠 max_width 过滤，不靠点数）。
//
//  行为：
//      只累积【前 accumulate_frames 帧】，攒够就检测一次，然后永久锁定。
//      → 所以车必须在启动后的这 0.4 秒内保持静止。
//
//  ⚠️ 安全：本节点【只订阅雷达、只写文件】，不发布任何运动指令，
//      不涉及 /cmd_vel，不可能让车动起来。
//
//  使用：
//      ros2 run my_racecar cone_calib_record
//      ros2 run my_racecar cone_calib_record --ros-args -p accumulate_frames:=30 -p min_points:=30
//
//  产出：
//      <output_path>  默认 /home/racecar/calib_logs/cones_ref.csv
//      ★ 刻意放在仓库之外：参考基准是"当次会话数据"，每次录制都会覆盖，
//        写在仓库里会让 git 工作区变脏，导致 push 被 updateInstead 拒绝。
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "cone_viz.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
//  基础类型
// ---------------------------------------------------------------------------
struct Point2D
{
    double x{0.0};
    double y{0.0};
    Point2D() = default;
    Point2D(double x_, double y_) : x(x_), y(y_) {}
};

// 一个检测到的锥桶候选
struct Cone
{
    double cx{0.0};        // 质心 x（雷达系，米）
    double cy{0.0};        // 质心 y（雷达系，米）
    double range{0.0};     // 到雷达的距离
    double width{0.0};     // 估计宽度（簇内最大点距）
    int    n_points{0};    // 簇内点数
};

// 点集几何质量（★ 与文档 §2.4 同口径）
struct Geometry
{
    double major{0.0};     // 主方向散布（米，标准差）
    double minor{0.0};     // 次方向散布（米，标准差）
    double ratio{1e9};     // 分散度比值 = major / minor  ← 文档里的 1.19
};

// ---------------------------------------------------------------------------
//  锥桶检测器
//  （检测逻辑复制自 cone_detect.cpp，【只改默认参数，不改算法】）
// ---------------------------------------------------------------------------
class ConeDetector
{
public:
    struct Config
    {
        // ---- 点云预过滤 ----
        double min_range         = 0.30;   // 忽略太近的（车体自身/地面）
        double max_range         = 8.00;   // 忽略太远的

        // ---- 累积 ----
        int    accumulate_frames = 20;     // 累积多少帧（约 0.4s @50Hz）

        // ---- 聚类 ----
        double cluster_tol       = 0.25;   // 聚类距离阈值（米）

        // ---- 簇过滤 ----
        // ★ 已按实测数据调整：
        //    真锥桶 459~554 点 / 宽 0.157~0.174m；噪声碎片 5 点 / 宽 0.012m
        int    min_points        = 30;     // 原 cone_detect 是 4（会放进噪声）
        int    max_points        = 3000;   // 原 cone_detect 是 120（会把真锥桶全滤掉！）
        double min_width         = 0.05;   // 原 0.03
        double max_width         = 0.35;   // 不变；★ 人（0.4~0.6m）在这里被剔除
    };

    Config cfg;

    // 累积一帧
    void addScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float r = msg->ranges[i];
            if (std::isnan(r) || std::isinf(r)) continue;
            if (r < msg->range_min || r > msg->range_max) continue;
            if (r < cfg.min_range || r > cfg.max_range) continue;

            float a = msg->angle_min + static_cast<float>(i) * msg->angle_increment;
            accum_.emplace_back(static_cast<double>(r) * std::cos(a),
                                static_cast<double>(r) * std::sin(a));
        }
        frame_count_++;
    }

    bool   ready()  const { return frame_count_ >= cfg.accumulate_frames; }
    int    frames() const { return frame_count_; }
    size_t points() const { return accum_.size(); }

    // 聚类 + 过滤，返回按距离升序排列的候选锥桶
    std::vector<Cone> detect()
    {
        std::vector<Cone> result;
        if (accum_.size() < static_cast<size_t>(cfg.min_points)) return result;

        const int n = static_cast<int>(accum_.size());

        // ---- 1. 按 x 排序（扫掠用，把 O(n^2) 降下来）----
        std::vector<Point2D> pts(accum_);
        std::sort(pts.begin(), pts.end(),
                  [](const Point2D & a, const Point2D & b) {
                      return a.x < b.x || (a.x == b.x && a.y < b.y);
                  });

        // ---- 2. 并查集聚类 ----
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
                // 已按 x 排序：x 差超过阈值就不可能相邻，直接断
                if (pts[j].x - pts[i].x > cfg.cluster_tol) break;

                double dx = pts[j].x - pts[i].x;
                double dy = pts[j].y - pts[i].y;
                if (dx * dx + dy * dy <= tol2) unite(i, j);
            }
        }

        // ---- 3. 分桶 ----
        std::map<int, std::vector<int>> groups;
        for (int i = 0; i < n; ++i) groups[find(i)].push_back(i);

        // ---- 4. 提取特征 + 过滤 ----
        for (auto & kv : groups)
        {
            const std::vector<int> & idxs = kv.second;

            const int np = static_cast<int>(idxs.size());
            if (np < cfg.min_points) continue;
            if (np > cfg.max_points) continue;

            Cone c;
            c.n_points = np;

            // 质心
            double sx = 0.0, sy = 0.0;
            for (int k : idxs) { sx += pts[k].x; sy += pts[k].y; }
            c.cx = sx / np;
            c.cy = sy / np;
            c.range = std::hypot(c.cx, c.cy);

            // 宽度 = 簇内任意两点最大距离（簇通常只有几百点，暴力够快）
            double maxd2 = 0.0;
            for (size_t a = 0; a < idxs.size(); ++a)
            {
                for (size_t b = a + 1; b < idxs.size(); ++b)
                {
                    double dx = pts[idxs[a]].x - pts[idxs[b]].x;
                    double dy = pts[idxs[a]].y - pts[idxs[b]].y;
                    double d2 = dx * dx + dy * dy;
                    if (d2 > maxd2) maxd2 = d2;
                }
            }
            c.width = std::sqrt(maxd2);

            if (c.width < cfg.min_width) continue;
            if (c.width > cfg.max_width) continue;   // ★ 人在这里被剔除

            result.push_back(c);
        }

        // ---- 5. 按距离排序 ----
        std::sort(result.begin(), result.end(),
                  [](const Cone & a, const Cone & b) { return a.range < b.range; });

        return result;
    }

private:
    std::vector<Point2D> accum_;
    int frame_count_{0};
};

// ---------------------------------------------------------------------------
//  几何一致性选点
//  （tools/cone_subset.js 的 C++ 移植，评分公式保持一致）
//  抽成独立 namespace，便于离线用真实雷达数据单独测试
// ---------------------------------------------------------------------------
namespace geo
{

// 主/次方向散布与分散度比值
//   ratio = major / minor（标准差之比）★ 文档 §2.4 的 1.19 就是这个口径
Geometry spreadRatio(const std::vector<Cone> & c, const std::vector<size_t> & idx)
{
    Geometry g;
    const int m = static_cast<int>(idx.size());
    if (m < 2) return g;

    double cx = 0.0, cy = 0.0;
    for (size_t i : idx) { cx += c[i].cx; cy += c[i].cy; }
    cx /= m; cy /= m;

    double sxx = 0.0, sxy = 0.0, syy = 0.0;
    for (size_t i : idx)
    {
        double dx = c[i].cx - cx;
        double dy = c[i].cy - cy;
        sxx += dx * dx;  sxy += dx * dy;  syy += dy * dy;
    }
    sxx /= m; sxy /= m; syy /= m;

    const double tr   = sxx + syy;
    const double det  = sxx * syy - sxy * sxy;
    const double disc = std::sqrt(std::max(0.0, tr * tr / 4.0 - det));

    const double l1 = std::max(0.0, tr / 2.0 + disc);   // 大特征值
    const double l2 = std::max(0.0, tr / 2.0 - disc);   // 小特征值

    g.major = std::sqrt(l1);
    g.minor = std::sqrt(std::max(1e-18, l2));
    g.ratio = (g.minor > 1e-9) ? (g.major / g.minor) : 1e9;
    return g;
}

// 组合内最小两两间距
double minSeparation(const std::vector<Cone> & c, const std::vector<size_t> & idx)
{
    double best = 1e18;
    for (size_t a = 0; a < idx.size(); ++a)
        for (size_t b = a + 1; b < idx.size(); ++b)
            best = std::min(best, std::hypot(c[idx[a]].cx - c[idx[b]].cx,
                                             c[idx[a]].cy - c[idx[b]].cy));
    return best;
}

// 组合枚举的最优解
struct BestCombo
{
    std::vector<size_t> idx;      // 选中的候选下标
    Geometry geo;                 // 该组合的几何质量
    double   min_sep{0.0};        // 组合内最小两两间距
    int      min_pts{0};          // 组合内最少点数
    double   score{1e18};         // 评分（越小越好）
    bool     valid{false};
};

// 评估一个组合，比当前最优更好就更新
void evaluateCombo(const std::vector<Cone> & c,
                   const std::vector<size_t> & idx,
                   double min_sep, double penalty, double bonus,
                   BestCombo & best)
{
    const Geometry g  = spreadRatio(c, idx);
    const double   ms = minSeparation(c, idx);

    int min_pts = 1 << 30;
    for (size_t i : idx) min_pts = std::min(min_pts, c[i].n_points);

    // ★ 评分公式与 tools/cone_subset.js 完全一致：
    //   score = 分散度比值 + (最小间距过近 ? penalty : 0) - bonus * 最小点数
    double score = g.ratio;
    if (ms < min_sep) score += penalty;
    score -= bonus * static_cast<double>(min_pts);

    if (!best.valid || score < best.score)
    {
        best.valid   = true;
        best.idx     = idx;
        best.geo     = g;
        best.min_sep = ms;
        best.min_pts = min_pts;
        best.score   = score;
    }
}

// 递归枚举 C(n,k)，边枚举边保留最优（不占内存）
void enumerateCombos(const std::vector<Cone> & c, size_t k, size_t start,
                     std::vector<size_t> & cur,
                     double min_sep, double penalty, double bonus,
                     BestCombo & best)
{
    if (cur.size() == k)
    {
        evaluateCombo(c, cur, min_sep, penalty, bonus, best);
        return;
    }
    for (size_t i = start; i < c.size(); ++i)
    {
        // 剪枝：剩下的候选不够凑满 k 个
        if (c.size() - i < k - cur.size()) break;
        cur.push_back(i);
        enumerateCombos(c, k, i + 1, cur, min_sep, penalty, bonus, best);
        cur.pop_back();
    }
}

// 对外接口：从候选中选出几何质量最好的 k 个
BestCombo selectBest(const std::vector<Cone> & cand, size_t k,
                     double min_sep, double penalty, double bonus)
{
    BestCombo best;
    std::vector<size_t> cur;
    cur.reserve(k);
    enumerateCombos(cand, k, 0, cur, min_sep, penalty, bonus, best);
    return best;
}

}  // namespace geo

// ---------------------------------------------------------------------------
//  ROS 节点
// ---------------------------------------------------------------------------
class ConeCalibRecordNode : public rclcpp::Node
{
public:
    ConeCalibRecordNode() : Node("cone_calib_record")
    {
        // ---- 参数声明（默认值 = 实测调好的值）----
        declare_parameter("min_range",           detector_.cfg.min_range);
        declare_parameter("max_range",           detector_.cfg.max_range);
        declare_parameter("accumulate_frames",   detector_.cfg.accumulate_frames);
        declare_parameter("cluster_tol",         detector_.cfg.cluster_tol);
        declare_parameter("min_points",          detector_.cfg.min_points);
        declare_parameter("max_points",          detector_.cfg.max_points);
        declare_parameter("min_width",           detector_.cfg.min_width);
        declare_parameter("max_width",           detector_.cfg.max_width);
        declare_parameter("k",                   k_);
        declare_parameter("max_candidate_range", max_candidate_range_);
        declare_parameter("max_enum_candidates", max_enum_candidates_);
        declare_parameter("min_sep",             min_sep_);
        declare_parameter("min_sep_penalty",     min_sep_penalty_);
        declare_parameter("point_bonus",         point_bonus_);
        declare_parameter("output_path",         output_path_);
        declare_parameter("frame_id",            frame_id_);

        // ---- 读取 ----
        detector_.cfg.min_range         = get_parameter("min_range").as_double();
        detector_.cfg.max_range         = get_parameter("max_range").as_double();
        detector_.cfg.accumulate_frames = get_parameter("accumulate_frames").as_int();
        detector_.cfg.cluster_tol       = get_parameter("cluster_tol").as_double();
        detector_.cfg.min_points        = get_parameter("min_points").as_int();
        detector_.cfg.max_points        = get_parameter("max_points").as_int();
        detector_.cfg.min_width         = get_parameter("min_width").as_double();
        detector_.cfg.max_width         = get_parameter("max_width").as_double();
        k_                      = get_parameter("k").as_int();
        max_candidate_range_    = get_parameter("max_candidate_range").as_double();
        max_enum_candidates_    = get_parameter("max_enum_candidates").as_int();
        min_sep_                = get_parameter("min_sep").as_double();
        min_sep_penalty_        = get_parameter("min_sep_penalty").as_double();
        point_bonus_            = get_parameter("point_bonus").as_double();
        output_path_            = get_parameter("output_path").as_string();
        frame_id_               = get_parameter("frame_id").as_string();

        // ---- 可视化发布（RViz）----
        // 只发一次就空闲，所以必须用 transient_local（锁存），
        // 这样 RViz 晚开也能收到。RViz 那边 Durability 要设 Transient Local。
        rclcpp::QoS latched(10);
        latched.transient_local();
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/cone_calib_record/markers", latched);

        // ---- 订阅雷达 ----
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&ConeCalibRecordNode::scanCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "==============================================");
        RCLCPP_INFO(get_logger(), " cone_calib_record —— 参考锥桶录制（步骤 1）");
        RCLCPP_INFO(get_logger(), "==============================================");
        RCLCPP_INFO(get_logger(), "⚠️  请确认车已停在【标准起跑点】且保持静止");
        RCLCPP_INFO(get_logger(), "等待 %d 帧雷达数据（约 %.1f 秒）...",
                    detector_.cfg.accumulate_frames,
                    detector_.cfg.accumulate_frames / 50.0);
        RCLCPP_INFO(get_logger(),
                    "参数: cluster_tol=%.2f min_points=%d max_points=%d "
                    "width=[%.2f, %.2f] frames=%d",
                    detector_.cfg.cluster_tol, detector_.cfg.min_points,
                    detector_.cfg.max_points, detector_.cfg.min_width,
                    detector_.cfg.max_width, detector_.cfg.accumulate_frames);
        RCLCPP_INFO(get_logger(), "输出文件: %s", output_path_.c_str());
    }

private:
    // -----------------------------------------------------------------------
    //  雷达回调：只累积前 N 帧，攒够就检测一次，之后永久锁定
    // -----------------------------------------------------------------------
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (done_) return;

        // 用雷达实际发布的 frame_id 作为可视化坐标系（更稳，不依赖参数写对）
        if (!msg->header.frame_id.empty() && !frame_from_scan_)
        {
            frame_id_ = msg->header.frame_id;
            frame_from_scan_ = true;
        }

        detector_.addScan(msg);

        if (!detector_.ready() && detector_.frames() % 5 == 0)
        {
            RCLCPP_INFO(get_logger(), "累积中 %d/%d 帧，当前点数 %zu",
                        detector_.frames(), detector_.cfg.accumulate_frames,
                        detector_.points());
        }

        if (!detector_.ready()) return;

        done_ = true;
        runDetection();

        RCLCPP_INFO(get_logger(), "★ 点云已锁定：%d 帧 / %zu 个点，后续不再累积",
                    detector_.frames(), detector_.points());
        RCLCPP_INFO(get_logger(), "本节点已空闲。按 Ctrl+C 结束，或直接开始步骤 2。");
    }

    // -----------------------------------------------------------------------
    //  检测 → 选点 → 打印 → 存盘
    // -----------------------------------------------------------------------
    void runDetection()
    {
        const std::vector<Cone> cones = detector_.detect();

        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "============== 锥桶检测结果 ==============");
        RCLCPP_INFO(get_logger(), "累积 %d 帧，共 %zu 个点，检出 %zu 个候选",
                    detector_.frames(), detector_.points(), cones.size());

        if (static_cast<int>(cones.size()) < k_)
        {
            RCLCPP_ERROR(get_logger(),
                         "!! 候选锥桶只有 %zu 个，不足 %d 个，无法选点",
                         cones.size(), k_);
            printCandidates(cones, {});
            RCLCPP_ERROR(get_logger(), "排查建议：");
            RCLCPP_ERROR(get_logger(),
                         "  1) 确认 %d 个锥桶都在 %.1fm 内且未被遮挡",
                         k_, max_candidate_range_);
            RCLCPP_ERROR(get_logger(),
                         "  2) 调小 min_points（当前 %d）", detector_.cfg.min_points);
            RCLCPP_ERROR(get_logger(),
                         "  3) 调大 max_width（当前 %.2f）", detector_.cfg.max_width);
            RCLCPP_ERROR(get_logger(),
                         "  4) 跑 ros2 run my_racecar cone_detect 看原始检测结果");
            RCLCPP_ERROR(get_logger(), "==========================================");
            publishMarkers(cones, {});
            return;
        }

        // ---- 只保留最近 max_candidate_range_ 米内的候选（避免远处噪声）----
        std::vector<Cone> near;
        for (const Cone & c : cones)
            if (c.range <= max_candidate_range_) near.push_back(c);

        if (static_cast<int>(near.size()) < k_)
        {
            RCLCPP_ERROR(get_logger(),
                         "!! %.1fm 内的候选只有 %zu 个，不足 %d 个",
                         max_candidate_range_, near.size(), k_);
            printCandidates(cones, {});
            publishMarkers(cones, {});
            return;
        }

        // ---- 枚举数量保护 ----
        if (static_cast<int>(near.size()) > max_enum_candidates_)
        {
            near.resize(static_cast<size_t>(max_enum_candidates_));
            RCLCPP_WARN(get_logger(),
                        "候选项数超过 %d，只对最近的 %d 个做组合枚举",
                        max_enum_candidates_, max_enum_candidates_);
        }

        // ---- 枚举 C(n,k)，找几何质量最好的一组 ----
        const geo::BestCombo best = geo::selectBest(
            near, static_cast<size_t>(k_), min_sep_, min_sep_penalty_, point_bonus_);

        if (!best.valid)
        {
            RCLCPP_ERROR(get_logger(), "!! 组合枚举失败（内部错误）");
            return;
        }

        // ---- 选中的 k 个：按到车距离升序重新编号（与文档 §2.2 顺序一致）----
        std::vector<size_t> sel = best.idx;
        std::sort(sel.begin(), sel.end(),
                  [&](size_t a, size_t b) { return near[a].range < near[b].range; });

        std::vector<Cone> selected;
        selected.reserve(sel.size());
        for (size_t i : sel) selected.push_back(near[i]);

        RCLCPP_INFO(get_logger(), "");
        printCandidates(cones, selected);
        printSelection(selected, best);
        printOldStrategy(cones);
        printSummary(selected, best, cones, near.size());
        writeCsv(selected, best, cones.size(), near.size());
        publishMarkers(cones, selected, &best);
    }

    // -----------------------------------------------------------------------
    //  打印候选全表
    // -----------------------------------------------------------------------
    void printCandidates(const std::vector<Cone> & cones,
                         const std::vector<Cone> & selected)
    {
        // 坐标完全相同即视为同一个簇（selected 是 cones 的副本）
        auto isSelected = [&](const Cone & c) {
            for (const Cone & s : selected)
                if (std::fabs(s.cx - c.cx) < 1e-9 && std::fabs(s.cy - c.cy) < 1e-9)
                    return true;
            return false;
        };

        RCLCPP_INFO(get_logger(), "---- 全部候选（★ = 被选中）----");
        RCLCPP_INFO(get_logger(), "%-4s %-4s %-21s %-10s %-10s %-7s",
                    "id", "选中", "坐标(x, y)", "距离(m)", "宽度(m)", "点数");
        for (size_t i = 0; i < cones.size(); ++i)
        {
            RCLCPP_INFO(get_logger(), "%-4zu %-4s (%7.3f, %7.3f)  %8.3f   %8.3f   %5d",
                        i, (isSelected(cones[i]) ? "★" : "  "),
                        cones[i].cx, cones[i].cy, cones[i].range,
                        cones[i].width, cones[i].n_points);
        }
    }

    // -----------------------------------------------------------------------
    //  打印选中结果 + 指纹 + 几何自检
    // -----------------------------------------------------------------------
    void printSelection(const std::vector<Cone> & selected,
                        const geo::BestCombo & best)
    {
        char line[256];

        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "========== ★ 选中的 %d 个参考锥桶 ==========", k_);
        RCLCPP_INFO(get_logger(), "  idx      x(m)     y(m)   dist(m)  width(m)   pts");
        for (size_t i = 0; i < selected.size(); ++i)
        {
            std::snprintf(line, sizeof(line),
                          "  %3zu %9.3f %8.3f %8.3f %9.3f %5d",
                          i, selected[i].cx, selected[i].cy,
                          selected[i].range, selected[i].width, selected[i].n_points);
            RCLCPP_INFO(get_logger(), "%s", line);
        }

        // ---- 成对距离指纹（核心校验手段）----
        RCLCPP_INFO(get_logger(), "---- 成对距离指纹（旋转平移不变量）----");
        for (size_t i = 0; i < selected.size(); ++i)
            for (size_t j = i + 1; j < selected.size(); ++j)
            {
                const double d = std::hypot(selected[i].cx - selected[j].cx,
                                            selected[i].cy - selected[j].cy);
                RCLCPP_INFO(get_logger(), "  %zu-%zu : %.4f m", i, j, d);
            }

        // ---- 几何质量自检 ----
        RCLCPP_INFO(get_logger(), "---- 几何质量自检 ----");
        RCLCPP_INFO(get_logger(), "  主方向散布 = %.3f m   次方向散布 = %.3f m",
                    best.geo.major, best.geo.minor);
        RCLCPP_INFO(get_logger(), "  分散度比值 = %.2f", best.geo.ratio);
        RCLCPP_INFO(get_logger(), "  最小两两间距 = %.3f m", best.min_sep);
        RCLCPP_INFO(get_logger(), "  组合内最少点数 = %d", best.min_pts);

        const bool ok_ratio        = (best.geo.ratio < 1.5);
        const bool ok_ratio_accept = (best.geo.ratio <= 5.0);
        const bool ok_sep          = (best.min_sep >= min_sep_);

        RCLCPP_INFO(get_logger(), "  [验收] 分散度比值 < 1.5 ?  %s (实际 %.2f)",
                    ok_ratio ? "✅ 是" : (ok_ratio_accept ? "△ 可接受" : "❌ 否"),
                    best.geo.ratio);
        RCLCPP_INFO(get_logger(), "  [验收] 最小间距 >= %.2fm ? %s (实际 %.3f)",
                    min_sep_, ok_sep ? "✅ 是" : "❌ 否", best.min_sep);

        if (!ok_ratio_accept)
        {
            RCLCPP_WARN(get_logger(),
                        "!! 分散度比值 %.1f 偏大，4 点接近共线/退化，"
                        "解位姿会很不稳。建议调整锥桶摆放。", best.geo.ratio);
        }
        else if (ok_ratio && ok_sep)
        {
            RCLCPP_INFO(get_logger(),
                        "  ✅ 几何条件良好，适合解位姿（4 点分散、不共线）");
        }
        if (!ok_sep)
        {
            RCLCPP_WARN(get_logger(),
                        "!! 有两点距离小于 %.2fm，容易互相混淆，建议拉开一些", min_sep_);
        }
    }

    // -----------------------------------------------------------------------
    //  对照：旧策略「取最近 k 个」会选中谁（= cone_detect 当前行为）
    //  本次上车就能直接看到「新策略 vs 旧策略」的差别
    // -----------------------------------------------------------------------
    void printOldStrategy(const std::vector<Cone> & cones)
    {
        if (cones.empty()) return;

        // cones 已按距离升序 → 前 k 个就是「取最近 k 个」
        const size_t keep = std::min(cones.size(), static_cast<size_t>(k_));
        std::vector<size_t> idx;
        for (size_t i = 0; i < keep; ++i) idx.push_back(i);

        const Geometry g  = geo::spreadRatio(cones, idx);
        const double   ms = geo::minSeparation(cones, idx);

        RCLCPP_INFO(get_logger(), "---- 对照：旧策略「取最近 %d 个」----", k_);
        for (size_t i = 0; i < keep; ++i)
            RCLCPP_INFO(get_logger(),
                        "    %zu: (%7.3f, %7.3f)  dist=%.3f  width=%.3f  pts=%d",
                        i, cones[i].cx, cones[i].cy, cones[i].range,
                        cones[i].width, cones[i].n_points);
        RCLCPP_INFO(get_logger(),
                    "    该组合分散度比值 = %.2f（越大越接近共线，越不适合解位姿）",
                    g.ratio);
        RCLCPP_INFO(get_logger(), "    该组合最小间距 = %.3f m", ms);
    }

    // -----------------------------------------------------------------------
    //  可复制摘要：方便把关键数字一次性贴给 AI
    // -----------------------------------------------------------------------
    void printSummary(const std::vector<Cone> & selected,
                      const geo::BestCombo & best,
                      const std::vector<Cone> & cones,
                      size_t n_near)
    {
        const size_t keep = std::min(cones.size(), static_cast<size_t>(k_));
        std::vector<size_t> oidx;
        for (size_t i = 0; i < keep; ++i) oidx.push_back(i);

        std::ostringstream pts, fp, old;
        pts << std::fixed << std::setprecision(3);
        for (size_t i = 0; i < selected.size(); ++i)
        {
            if (i) pts << " ";
            pts << selected[i].cx << "," << selected[i].cy;
        }

        fp << std::fixed << std::setprecision(4);
        bool first = true;
        for (size_t i = 0; i < selected.size(); ++i)
            for (size_t j = i + 1; j < selected.size(); ++j)
            {
                if (!first) fp << " ";
                first = false;
                fp << std::hypot(selected[i].cx - selected[j].cx,
                                 selected[i].cy - selected[j].cy);
            }

        old << std::fixed << std::setprecision(3);
        for (size_t i = 0; i < keep; ++i)
        {
            if (i) old << " ";
            old << cones[i].cx << "," << cones[i].cy;
        }

        double old_ratio = 0.0;
        if (!oidx.empty()) old_ratio = geo::spreadRatio(cones, oidx).ratio;

        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(),
                    "================= 复制下面这段发给 AI =================");
        RCLCPP_INFO(get_logger(),
                    "CONES_REF k=%d ratio=%.2f minsep=%.3f cand_all=%zu cand_enum=%zu",
                    k_, best.geo.ratio, best.min_sep, cones.size(), n_near);
        RCLCPP_INFO(get_logger(), "new: %s", pts.str().c_str());
        RCLCPP_INFO(get_logger(), "fp : %s", fp.str().c_str());
        RCLCPP_INFO(get_logger(), "old: %s (ratio=%.2f)", old.str().c_str(), old_ratio);
        RCLCPP_INFO(get_logger(),
                    "======================================================");
    }

    // -----------------------------------------------------------------------
    //  写 CSV
    // -----------------------------------------------------------------------
    void writeCsv(const std::vector<Cone> & selected,
                  const geo::BestCombo & best,
                  size_t n_all, size_t n_near)
    {
        std::ofstream f(output_path_);
        if (!f.is_open())
        {
            RCLCPP_ERROR(get_logger(), "!! 无法写入文件: %s", output_path_.c_str());
            RCLCPP_ERROR(get_logger(),
                         "   请检查目录是否存在，或用 -p output_path:=<路径> 指定");
            return;
        }

        std::time_t t = std::time(nullptr);
        char tbuf[64] = {0};
        std::strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));

        f << "# cones_ref.csv —— 标准起跑点参考锥桶（初始点校准基准）\n";
        f << "# 坐标系: laser_link（雷达系, 米）, 车在原点, x 向前, y 向左\n";
        f << "# 录制时间: " << tbuf << "\n";
        f << "# 参数: accumulate_frames=" << detector_.cfg.accumulate_frames
          << " cluster_tol=" << detector_.cfg.cluster_tol
          << " min_points=" << detector_.cfg.min_points
          << " max_points=" << detector_.cfg.max_points
          << " min_width=" << detector_.cfg.min_width
          << " max_width=" << detector_.cfg.max_width
          << " k=" << k_ << "\n";
        f << "# 选点: 几何一致性枚举 C(n," << k_ << ") 最优（非「取最近」）\n";
        f << "# 几何: 分散度比值=" << std::fixed << std::setprecision(2) << best.geo.ratio
          << " 主散布=" << std::setprecision(3) << best.geo.major
          << " 次散布=" << best.geo.minor
          << " 最小间距=" << best.min_sep << "\n";
        f << "# 候选: 全部=" << n_all << " 参与枚举=" << n_near << "\n";
        f << "# 顺序: 按到车距离升序\n";
        f << "x,y\n";

        f << std::fixed << std::setprecision(6);
        for (const Cone & c : selected)
            f << c.cx << "," << c.cy << "\n";
        f.close();

        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "✅ 已写入参考基准文件: %s", output_path_.c_str());
        RCLCPP_INFO(get_logger(),
                    "   → 这是「标准答案」，只在标准起跑点录一次，之后不要随意覆盖");
    }

    // -----------------------------------------------------------------------
    //  RViz 可视化
    //    🟢 绿色大球（带暗绿描边） = 选中的参考锥桶（编号 S0..S3 + 到车距离）
    //    ⚫ 灰色半透明小球        = 未选中的候选（说明附近还有别的锥桶）
    //    🟡 黄色线段              = 选中锥桶之间的连线（显示几何形状）
    //    🟡 黄色数字              = 两两之间的距离（成对距离指纹）
    //    ⚪ 白色大字              = 几何质量结论
    // -----------------------------------------------------------------------
    void publishMarkers(const std::vector<Cone> & cones,
                        const std::vector<Cone> & selected,
                        const geo::BestCombo * best = nullptr)
    {
        viz::Painter p(frame_id_, now());
        p.deleteAll();

        auto isSelected = [&](const Cone & c) {
            for (const Cone & s : selected)
                if (std::fabs(s.cx - c.cx) < 1e-9 && std::fabs(s.cy - c.cy) < 1e-9)
                    return true;
            return false;
        };

        // ① 未选中的候选：灰色小球
        for (const Cone & c : cones)
            if (!isSelected(c))
                p.sphere("rejected", c.cx, c.cy, 0.0, 0.18, viz::kGray);

        // ② 选中的参考锥桶：暗绿描边 + 亮绿球 + 白字
        for (size_t i = 0; i < selected.size(); ++i)
        {
            p.sphere("sel_outline", selected[i].cx, selected[i].cy, 0.0, 0.44, viz::kDarkGreen);
            p.sphere("sel",         selected[i].cx, selected[i].cy, 0.0, 0.32, viz::kGreen);

            char buf[64];
            std::snprintf(buf, sizeof(buf), "S%zu  %.2fm", i, selected[i].range);
            p.text("sel_label", selected[i].cx, selected[i].cy, 0.55, 0.20,
                   viz::kWhite, buf);
        }

        // ③ 选中锥桶之间的连线 + 距离数字（就是"指纹"）
        for (size_t i = 0; i < selected.size(); ++i)
            for (size_t j = i + 1; j < selected.size(); ++j)
            {
                const double d = std::hypot(selected[i].cx - selected[j].cx,
                                            selected[i].cy - selected[j].cy);
                p.segment("edges", selected[i].cx, selected[i].cy, 0.02,
                          selected[j].cx, selected[j].cy, 0.02, 0.015, viz::kYellow);

                char buf[32];
                std::snprintf(buf, sizeof(buf), "%.2f", d);
                p.text("edge_len",
                       (selected[i].cx + selected[j].cx) / 2.0,
                       (selected[i].cy + selected[j].cy) / 2.0,
                       0.28, 0.15, viz::kYellow, buf);
            }

        // ④ 状态灯 + 结论（放在锥桶区域【下方】，俯视时不会和锥桶重叠）
        //
        //  ⚠️ 两条硬约束（踩过坑）：
        //     1) RViz 的 Marker 文字用 Liberation Sans 渲染，【没有中文/emoji 字形】
        //        —— 写了中文会整条渲染不出来（终端里的中文是好的，只有 RViz 不行）
        //     2) 【不支持换行符 \n】
        //    所以这里一律：纯 ASCII、每行一个独立 marker。
        const double bx = 0.0, by = -1.45, bz = 0.30;
        const bool ok = !selected.empty() && (best != nullptr);
        const viz::Color col = ok ? viz::kGreen : viz::kRed;
        p.sphere("status", bx, by, bz, ok ? 0.55 : 0.45, col);

        if (ok)
        {
            char l2[128], l3[128];
            std::snprintf(l2, sizeof(l2), "ratio=%.2f  minsep=%.3fm",
                          best->geo.ratio, best->min_sep);
            std::snprintf(l3, sizeof(l3), "candidates=%zu  selected=%zu",
                          cones.size(), selected.size());
            p.text("verdict1", bx, by - 0.60, bz, 0.30, col,
                   "REFERENCE CONES RECORDED");
            p.text("verdict2", bx, by - 0.95, bz, 0.22, col, l2);
            p.text("verdict3", bx, by - 1.25, bz, 0.20, col, l3);
        }
        else
        {
            char l2[128];
            std::snprintf(l2, sizeof(l2), "candidates=%zu selected=%zu",
                          cones.size(), selected.size());
            p.text("verdict1", bx, by - 0.60, bz, 0.30, col, "RECORD FAILED");
            p.text("verdict2", bx, by - 0.95, bz, 0.20, col, l2);
            p.text("verdict3", bx, by - 1.25, bz, 0.18, col,
                   "check min_points / max_width / max_range");
        }

        marker_pub_->publish(p.array());
    }

    // ---- 成员 ----
    ConeDetector detector_;
    int         k_{4};
    double      max_candidate_range_{6.0};
    int         max_enum_candidates_{60};
    double      min_sep_{0.8};
    double      min_sep_penalty_{20.0};
    double      point_bonus_{0.002};
    std::string frame_id_{"laser_link"};
    bool        frame_from_scan_{false};
    // ★ 输出路径刻意放在【仓库之外】的 calib_logs/ 下。
    //    原因：参考基准是"当次会话的数据"，每次录制都会覆盖它。
    //    如果写在仓库里（include/），每录一次 git 工作区就变脏，
    //    会导致 `git push` 被 updateInstead 拒绝
    //    （报 "Working directory has unstaged changes"）。
    //    放在仓库外，录制多少次都不影响 git 状态。
    std::string output_path_{"/home/racecar/calib_logs/cones_ref.csv"};
    bool        done_{false};

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConeCalibRecordNode>());
    rclcpp::shutdown();
    return 0;
}

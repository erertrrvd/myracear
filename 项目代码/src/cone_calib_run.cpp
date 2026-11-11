// ============================================================================
//  cone_calib_run.cpp —— 初始点校准节点（每次运行 · 步骤 2）
// ============================================================================
//
//  作用：
//      车停在起跑点（位置可能和上次不完全一样），本节点：
//        ① 累积雷达帧 → 聚类出锥桶候选（与步骤 1 完全相同的检测参数）
//        ② 读入 cones_ref.csv（标准起跑点录的"标准答案"）
//        ③ 指纹匹配：枚举「哪 4 个候选」+「24 种对应关系」，
//           用成对距离（旋转平移不变量）找出正确对应
//        ④ 2D Kabsch 解刚体变换 → 得到本次停放相对标准停放的 (dx, dy, dyaw)
//        ⑤ 自检 → 明确输出「✅ 可以起跑」或「❌ 请重新摆放」
//
//  ★ 核心原理：两个锥桶之间的距离不随平移/旋转改变
//        P^M = R · P^L + t
//        ‖A^M − B^M‖ = ‖R·(A^L − B^L)‖ = ‖A^L − B^L‖
//    4 个点有 C(4,2)=6 个两两距离；对应关系只有 4! = 24 种 → 暴力枚举。
//
//  ★ 判别力 λ = 次佳排列误差 / 最佳排列误差
//    λ 大 → 匹配唯一可信；λ≈1 → 存在并列解（布局对称/退化）→ 判失败。
//    这比只看"误差绝对值"更可靠，能识别"误差虽小但有多个解一样好"的危险情况。
//
//  ★ 关于求解方法：本节点用【2D 闭式解】而不是复制 get_local_path.cpp 的 SVD。
//    理由是 2D 情形存在解析最优解，等价于 Kabsch/SVD 但更短、无需 Eigen 依赖：
//        θ = atan2( Σ (x_i × X_i), Σ (x_i · X_i) )
//    其中 x_i 为查询点质心化坐标，X_i 为参考点质心化坐标。
//    已在离线测试中用已知 (dx,dy,dyaw) 的合成数据验证能精确还原。
//
//  ⚠️ 安全：本节点【只订阅雷达、只读文件、只打印】，
//      不发布任何运动指令，不涉及 /cmd_vel，不可能让车动起来。
//
//  使用：
//      ros2 run my_racecar cone_calib_run
//      ros2 run my_racecar cone_calib_run --ros-args -p ref_path:=/path/to/cones_ref.csv
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "cone_viz.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
//  基础类型（与 cone_calib_record.cpp 保持一致）
// ---------------------------------------------------------------------------
struct Point2D
{
    double x{0.0};
    double y{0.0};
    Point2D() = default;
    Point2D(double x_, double y_) : x(x_), y(y_) {}
};

struct Cone
{
    double cx{0.0};        // 质心 x（雷达系，米）
    double cy{0.0};        // 质心 y（雷达系，米）
    double range{0.0};     // 到雷达的距离
    double width{0.0};     // 估计宽度
    int    n_points{0};    // 簇内点数
};

// ---------------------------------------------------------------------------
//  锥桶检测器
//  ⚠️ 本段与 cone_calib_record.cpp 中的同名类【必须保持一致】。
//     两处都改了才不会出现"录制用一套参数、校准用另一套"的错配。
//     （刻意保留重复代码：让步骤 1 的文件保持已验证状态不被牵动）
// ---------------------------------------------------------------------------
class ConeDetector
{
public:
    struct Config
    {
        double min_range         = 0.30;
        double max_range         = 8.00;
        int    accumulate_frames = 20;
        double cluster_tol       = 0.25;
        int    min_points        = 30;     // 真锥桶 459~562 点 / 噪声碎片 5 点
        int    max_points        = 3000;   // 原 cone_detect 的 120 会把真锥桶全滤掉
        double min_width         = 0.05;
        double max_width         = 0.35;   // ★ 人（0.4~0.6m）在这里被剔除
    };

    Config cfg;

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

    std::vector<Cone> detect()
    {
        std::vector<Cone> result;
        if (accum_.size() < static_cast<size_t>(cfg.min_points)) return result;

        const int n = static_cast<int>(accum_.size());

        std::vector<Point2D> pts(accum_);
        std::sort(pts.begin(), pts.end(),
                  [](const Point2D & a, const Point2D & b) {
                      return a.x < b.x || (a.x == b.x && a.y < b.y);
                  });

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
            double sx = 0.0, sy = 0.0;
            for (int k : idxs) { sx += pts[k].x; sy += pts[k].y; }
            c.cx = sx / np;
            c.cy = sy / np;
            c.range = std::hypot(c.cx, c.cy);

            double maxd2 = 0.0;
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
        return result;
    }

private:
    std::vector<Point2D> accum_;
    int frame_count_{0};
};

// ===========================================================================
//  指纹匹配 + 位姿求解（抽成独立 namespace，便于离线单独测试）
// ===========================================================================
namespace calib
{

constexpr double kPi = 3.14159265358979323846;

// 组合内最多支持的锥桶数（决定排列枚举规模；4 个锥桶 = 24 种排列）
constexpr size_t kMaxK = 8;

// ---------------------------------------------------------------------------
//  位姿搜索窗口（文档 §6「位姿搜索窗口约束」）
//    车停在起跑点时，真实解必然落在"离标准位姿不远"的范围内。
//    用这个先验把不可能的解直接从搜索空间里剔除，可以消除近对称布局
//    （例如矩形发车区天然存在的 180° 旋转对称）带来的歧义。
//
//    实测依据（2026-09-23 车上数据）：本场地发车区是 1.02×1.42m 的矩形，
//    两条对角线只差 5mm —— 仅靠距离指纹，判别力 λ 只有 12（阈值 10），
//    余量很小。加上窗口约束后，180° 翻转解会被直接排除，λ 不再承担全部压力。
// ---------------------------------------------------------------------------
struct Window
{
    double max_yaw_rad{20.0 * kPi / 180.0};   // 允许的转角（弧度）
    double max_translation{0.50};             // 允许的位移（m）
    // 允许的点位残差。
    // ★ 这一条必须有！实测发现：矩形布局存在"左右镜像"的假排列，
    //   它 6 个距离几乎全对（代价只差 5 倍）、质心几乎重合（位移 8mm）、
    //   转角也小（2°）—— 只约束"转角+位移"拦不住它，但它的点位残差高达 1 米。
    //   加上这一条后假排列被直接排除。
    double max_point_residual{0.05};
};

struct ComboEval
{
    // ---- 窗口内最优（这就是最终要用的解）----
    bool   in_window{false};
    double best_cost{0.0};
    double second_cost{1e18};      // 1e18 = 窗口内只有这一个解（无歧义）
    std::vector<size_t> perm;      // perm[a] = 与组合内第 a 个点对应的参考点下标
    int    n_in_window{0};

    // ---- 全局最优（不受窗口约束，仅用于诊断）----
    double best_cost_any{1e18};
};

struct BestMatch
{
    bool   ok{false};
    std::vector<size_t> qidx;          // 选中的候选下标
    std::vector<size_t> perm;          // 对应关系
    double best_cost{0.0};             // 窗口内最佳排列的距离误差总和（m）
    double second_cost{1e18};          // 窗口内次佳排列的距离误差总和（m）
    double mean_err{0.0};              // 最佳排列的平均每对距离误差（m）
    double lambda{0.0};                // 判别力 = 次佳/最佳；1e9 = 窗口内唯一解
    double second_combo_mean{1e18};    // 次佳【组合】的平均误差（看组合区分度）
    size_t combos_tried{0};
    size_t combos_no_window{0};        // 窗口内无解的候选组合数
    double winner_cost_any{1e18};      // 获胜组合"忽略窗口"时的最小误差（诊断）
};

struct Pose2D
{
    double dx{0.0};
    double dy{0.0};
    double dyaw{0.0};      // 弧度
    double rms{0.0};       // 点位残差 RMS（m）
    double max_res{0.0};   // 单点最大残差（m）
                           // ★ 对"某一个锥桶被单独挪动"比平均误差敏感得多：
                           //   实测 1 个锥桶挪 δ，单点残差 ≈ 6δ，而平均距离误差 ≈ 0.27δ
};

// k! 个排列
inline std::vector<std::vector<size_t>> permutations(size_t k)
{
    std::vector<std::vector<size_t>> out;
    std::vector<size_t> p(k);
    std::iota(p.begin(), p.end(), 0);
    do { out.push_back(p); } while (std::next_permutation(p.begin(), p.end()));
    return out;
}

// 组合数 C(n,k)，饱和到 1e8 防止溢出
inline size_t nCk(size_t n, size_t k)
{
    if (k > n) return 0;
    if (k > n - k) k = n - k;
    unsigned long long r = 1;
    for (size_t i = 0; i < k; ++i)
    {
        r = r * static_cast<unsigned long long>(n - i) / (i + 1);
        if (r > 100000000ULL) return 100000000ULL;
    }
    return static_cast<size_t>(r);
}

// 评估一个候选组合：枚举所有排列，在【位姿搜索窗口内】取最佳/次佳
//
//   refx/refy/refcx/refcy 是参考点集的质心化坐标与质心（只算一次，所有组合共用）
//   tan_max = tan(窗口允许的最大转角)
inline ComboEval evalCombo(const std::vector<Cone> & q,
                           const std::vector<Cone> & ref,
                           const std::vector<std::vector<double>> & dref,
                           const std::vector<double> & refx,
                           const std::vector<double> & refy,
                           double refcx, double refcy,
                           const std::vector<std::vector<size_t>> & perms,
                           const std::vector<size_t> & cur,
                           const Window & win, double tan_max)
{
    const size_t k = cur.size();

    // ---- 组合内查询点的质心化坐标 + 两两距离 ----
    double dq[kMaxK][kMaxK];
    double qx[kMaxK], qy[kMaxK];
    double qcx = 0.0, qcy = 0.0;
    for (size_t a = 0; a < k; ++a) { qcx += q[cur[a]].cx; qcy += q[cur[a]].cy; }
    qcx /= k; qcy /= k;

    for (size_t a = 0; a < k; ++a)
    {
        qx[a] = q[cur[a]].cx - qcx;
        qy[a] = q[cur[a]].cy - qcy;
        dq[a][a] = 0.0;
        for (size_t b = a + 1; b < k; ++b)
        {
            const double d = std::hypot(q[cur[a]].cx - q[cur[b]].cx,
                                        q[cur[a]].cy - q[cur[b]].cy);
            dq[a][b] = dq[b][a] = d;
        }
    }

    ComboEval e;
    for (const std::vector<size_t> & p : perms)
    {
        // ---- ① 距离指纹代价 ----
        double cost = 0.0;
        for (size_t a = 0; a < k; ++a)
            for (size_t b = a + 1; b < k; ++b)
                cost += std::fabs(dq[a][b] - dref[p[a]][p[b]]);

        if (cost < e.best_cost_any) e.best_cost_any = cost;   // 诊断用

        // ---- ② 位姿窗口：先只判断【角度】，用纯比较避免 atan2 ----
        //     θ = atan2(num, den)
        //     |θ| ≤ θmax  ⟺  den > 0 且 |num| ≤ den·tan(θmax)
        double num = 0.0, den = 0.0;
        for (size_t a = 0; a < k; ++a)
        {
            const double X = refx[p[a]], Y = refy[p[a]];
            num += qx[a] * Y - qy[a] * X;      // Σ (x × X)
            den += qx[a] * X + qy[a] * Y;      // Σ (x · X)
        }
        if (!(den > 0.0) || std::fabs(num) > den * tan_max) continue;   // 转角超窗

        // ---- ③ 位姿 + 位移 / 点位残差窗口 ----
        const double th = std::atan2(num, den);
        const double c = std::cos(th), s = std::sin(th);
        const double tx = refcx - (c * qcx - s * qcy);
        const double ty = refcy - (s * qcx + c * qcy);
        if (std::hypot(tx, ty) > win.max_translation) continue;         // 位移超窗

        // 点位残差：距离指纹看不出的"镜像型假排列"在这一步会被剔除
        double mx2 = 0.0;
        for (size_t a = 0; a < k; ++a)
        {
            const double px = c * q[cur[a]].cx - s * q[cur[a]].cy + tx;
            const double py = s * q[cur[a]].cx + c * q[cur[a]].cy + ty;
            const double ex = px - ref[p[a]].cx;
            const double ey = py - ref[p[a]].cy;
            const double d2 = ex * ex + ey * ey;
            if (d2 > mx2) mx2 = d2;
        }
        if (std::sqrt(mx2) > win.max_point_residual) continue;          // 残差超窗

        // ---- ④ 通过全部窗口，参与最佳/次佳比较 ----
        e.n_in_window++;
        if (!e.in_window)
        {
            e.in_window = true;
            e.best_cost = cost;
            e.second_cost = 1e18;
            e.perm = p;
        }
        else if (cost < e.best_cost)
        {
            e.second_cost = e.best_cost;
            e.best_cost = cost;
            e.perm = p;
        }
        else if (cost < e.second_cost)
        {
            e.second_cost = cost;
        }
    }
    return e;
}

// 递归枚举 C(n,k)
inline void enumerate(const std::vector<Cone> & q,
                      const std::vector<Cone> & ref,
                      const std::vector<std::vector<double>> & dref,
                      const std::vector<double> & refx,
                      const std::vector<double> & refy,
                      double refcx, double refcy,
                      const std::vector<std::vector<size_t>> & perms,
                      size_t k, size_t start,
                      std::vector<size_t> & cur,
                      const Window & win, double tan_max,
                      BestMatch & best)
{
    if (cur.size() == k)
    {
        const ComboEval e = evalCombo(q, ref, dref, refx, refy, refcx, refcy,
                                      perms, cur, win, tan_max);
        best.combos_tried++;

        // 窗口内没有任何可行解 → 整组丢弃
        if (!e.in_window) { best.combos_no_window++; return; }

        const double npairs = static_cast<double>(k * (k - 1) / 2);
        const double mean   = e.best_cost / npairs;
        // 窗口内只有一个解 → 无歧义，λ 视为极大
        const double lambda = (e.second_cost > 1e17)
                            ? 1e9
                            : ((e.best_cost > 1e-12) ? e.second_cost / e.best_cost : 1e9);

        if (!best.ok || mean < best.mean_err)
        {
            if (best.ok) best.second_combo_mean = best.mean_err;
            best.ok               = true;
            best.qidx             = cur;
            best.perm             = e.perm;
            best.best_cost        = e.best_cost;
            best.second_cost      = e.second_cost;
            best.mean_err         = mean;
            best.lambda           = lambda;
            best.winner_cost_any  = e.best_cost_any;
        }
        else if (mean < best.second_combo_mean)
        {
            best.second_combo_mean = mean;
        }
        return;
    }

    for (size_t i = start; i < q.size(); ++i)
    {
        if (q.size() - i < k - cur.size()) break;
        cur.push_back(i);
        enumerate(q, ref, dref, refx, refy, refcx, refcy, perms, k, i + 1, cur,
                  win, tan_max, best);
        cur.pop_back();
    }
}

// 主入口：在候选里找出与参考最匹配的 k 个，并确定对应关系
inline BestMatch matchBest(const std::vector<Cone> & q,
                           const std::vector<Cone> & ref,
                           const Window & win,
                           size_t max_combos,
                           size_t * used_n,
                           bool   * trimmed)
{
    BestMatch best;
    const size_t k = ref.size();
    if (k < 3 || k > kMaxK || q.size() < k) return best;

    // 参考点：质心 + 质心化坐标 + 两两距离矩阵（所有组合共用，只算一次）
    double refcx = 0.0, refcy = 0.0;
    for (const Cone & c : ref) { refcx += c.cx; refcy += c.cy; }
    refcx /= k; refcy /= k;

    std::vector<double> refx(k), refy(k);
    std::vector<std::vector<double>> dref(k, std::vector<double>(k, 0.0));
    for (size_t a = 0; a < k; ++a)
    {
        refx[a] = ref[a].cx - refcx;
        refy[a] = ref[a].cy - refcy;
        for (size_t b = a + 1; b < k; ++b)
            dref[a][b] = dref[b][a] =
                std::hypot(ref[a].cx - ref[b].cx, ref[a].cy - ref[b].cy);
    }

    const std::vector<std::vector<size_t>> perms = permutations(k);
    const double tan_max = std::tan(win.max_yaw_rad);

    // 组合数保护：必要时只用最近的 n 个候选
    size_t n = q.size();
    while (n > k && nCk(n, k) > max_combos) --n;
    if (used_n) *used_n = n;
    if (trimmed) *trimmed = (n < q.size());

    std::vector<Cone> qq(q.begin(), q.begin() + static_cast<long>(n));
    std::vector<size_t> cur;
    cur.reserve(k);
    enumerate(qq, ref, dref, refx, refy, refcx, refcy, perms, k, 0, cur,
              win, tan_max, best);
    return best;
}

// 2D 刚体变换闭式解： 参考 = R(θ)·查询 + t
inline Pose2D solveRigid(const std::vector<Cone> & q,
                         const std::vector<size_t> & qidx,
                         const std::vector<Cone> & ref,
                         const std::vector<size_t> & perm)
{
    Pose2D p;
    const size_t k = qidx.size();
    if (k < 2) return p;

    double cqx = 0, cqy = 0, crx = 0, cry = 0;
    for (size_t i = 0; i < k; ++i)
    {
        cqx += q[qidx[i]].cx;   cqy += q[qidx[i]].cy;
        crx += ref[perm[i]].cx; cry += ref[perm[i]].cy;
    }
    cqx /= k; cqy /= k; crx /= k; cry /= k;

    double num = 0.0, den = 0.0;
    for (size_t i = 0; i < k; ++i)
    {
        const double x = q[qidx[i]].cx - cqx;
        const double y = q[qidx[i]].cy - cqy;
        const double X = ref[perm[i]].cx - crx;
        const double Y = ref[perm[i]].cy - cry;
        num += x * Y - y * X;      // Σ (x × X)
        den += x * X + y * Y;      // Σ (x · X)
    }

    const double th = std::atan2(num, den);
    const double c = std::cos(th), s = std::sin(th);

    p.dyaw = th;
    p.dx = crx - (c * cqx - s * cqy);
    p.dy = cry - (s * cqx + c * cqy);

    double se = 0.0, mx = 0.0;
    for (size_t i = 0; i < k; ++i)
    {
        const double px = c * q[qidx[i]].cx - s * q[qidx[i]].cy + p.dx;
        const double py = s * q[qidx[i]].cx + c * q[qidx[i]].cy + p.dy;
        const double ex = px - ref[perm[i]].cx;
        const double ey = py - ref[perm[i]].cy;
        const double e2 = ex * ex + ey * ey;
        se += e2;
        if (e2 > mx) mx = e2;
    }
    p.rms     = std::sqrt(se / static_cast<double>(k));
    p.max_res = std::sqrt(mx);
    return p;
}

}  // namespace calib

// ---------------------------------------------------------------------------
//  ROS 节点
// ---------------------------------------------------------------------------
class ConeCalibRunNode : public rclcpp::Node
{
public:
    ConeCalibRunNode() : Node("cone_calib_run")
    {
        declare_parameter("min_range",           detector_.cfg.min_range);
        declare_parameter("max_range",           detector_.cfg.max_range);
        declare_parameter("accumulate_frames",   detector_.cfg.accumulate_frames);
        declare_parameter("cluster_tol",         detector_.cfg.cluster_tol);
        declare_parameter("min_points",          detector_.cfg.min_points);
        declare_parameter("max_points",          detector_.cfg.max_points);
        declare_parameter("min_width",           detector_.cfg.min_width);
        declare_parameter("max_width",           detector_.cfg.max_width);
        declare_parameter("max_candidate_range", max_candidate_range_);
        declare_parameter("max_combos",          static_cast<int>(max_combos_));
        declare_parameter("ref_path",            ref_path_);
        declare_parameter("max_match_error",     max_match_error_);
        declare_parameter("fail_match_error",    fail_match_error_);
        declare_parameter("max_residual",        max_residual_);
        declare_parameter("min_lambda",          min_lambda_);
        declare_parameter("max_translation",     max_translation_);
        declare_parameter("max_yaw_deg",         max_yaw_deg_);
        declare_parameter("window_max_residual", window_max_residual_);
        declare_parameter("frame_id",            frame_id_);

        detector_.cfg.min_range         = get_parameter("min_range").as_double();
        detector_.cfg.max_range         = get_parameter("max_range").as_double();
        detector_.cfg.accumulate_frames = get_parameter("accumulate_frames").as_int();
        detector_.cfg.cluster_tol       = get_parameter("cluster_tol").as_double();
        detector_.cfg.min_points        = get_parameter("min_points").as_int();
        detector_.cfg.max_points        = get_parameter("max_points").as_int();
        detector_.cfg.min_width         = get_parameter("min_width").as_double();
        detector_.cfg.max_width         = get_parameter("max_width").as_double();
        max_candidate_range_ = get_parameter("max_candidate_range").as_double();
        max_combos_          = static_cast<size_t>(get_parameter("max_combos").as_int());
        ref_path_            = get_parameter("ref_path").as_string();
        max_match_error_     = get_parameter("max_match_error").as_double();
        fail_match_error_    = get_parameter("fail_match_error").as_double();
        max_residual_        = get_parameter("max_residual").as_double();
        min_lambda_          = get_parameter("min_lambda").as_double();
        max_translation_     = get_parameter("max_translation").as_double();
        max_yaw_deg_         = get_parameter("max_yaw_deg").as_double();
        window_max_residual_ = get_parameter("window_max_residual").as_double();
        frame_id_            = get_parameter("frame_id").as_string();

        // ---- 可视化发布（RViz）----
        // 只发一次就空闲，所以必须用 transient_local（锁存），RViz 晚开也能收到。
        rclcpp::QoS latched(10);
        latched.transient_local();
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/cone_calib_run/markers", latched);

        RCLCPP_INFO(get_logger(), "==============================================");
        RCLCPP_INFO(get_logger(), " cone_calib_run —— 初始点校准（步骤 2）");
        RCLCPP_INFO(get_logger(), "==============================================");

        if (!loadReference())
        {
            ok_ = false;
            return;
        }

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&ConeCalibRunNode::scanCallback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "参考基准: %s（%zu 个锥桶）",
                    ref_path_.c_str(), ref_.size());
        for (size_t i = 0; i < ref_.size(); ++i)
            RCLCPP_INFO(get_logger(), "    参考#%zu (%.3f, %.3f)", i, ref_[i].cx, ref_[i].cy);

        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "⚠️  请确认车已停在起跑点并保持静止");
        RCLCPP_INFO(get_logger(), "等待 %d 帧雷达数据（约 %.1f 秒）...",
                    detector_.cfg.accumulate_frames,
                    detector_.cfg.accumulate_frames / 50.0);
    }

    bool ok() const { return ok_; }

private:
    // -----------------------------------------------------------------------
    //  读参考基准文件
    // -----------------------------------------------------------------------
    bool loadReference()
    {
        std::ifstream f(ref_path_);
        if (!f.is_open())
        {
            RCLCPP_ERROR(get_logger(), "!! 打不开参考文件: %s", ref_path_.c_str());
            RCLCPP_ERROR(get_logger(),
                         "   请先运行步骤 1 生成它：ros2 run my_racecar cone_calib_record");
            return false;
        }

        std::string line;
        while (std::getline(f, line))
        {
            // 去掉行尾 \r
            while (!line.empty() && (line.back() == '\r' || line.back() == '\n'))
                line.pop_back();

            // 跳过空行与注释行（步骤 1 写的 # 头部）
            if (line.empty() || line[0] == '#') continue;

            const size_t comma = line.find(',');
            if (comma == std::string::npos) continue;      // 跳过 "x,y" 表头

            try
            {
                const double x = std::stod(line.substr(0, comma));
                const double y = std::stod(line.substr(comma + 1));
                Cone c;
                c.cx = x;
                c.cy = y;
                c.range = std::hypot(x, y);
                ref_.push_back(c);
            }
            catch (...)
            {
                continue;                                   // 非数字行，跳过
            }
        }

        if (ref_.size() < 3)
        {
            RCLCPP_ERROR(get_logger(),
                         "!! 参考文件里有效点只有 %zu 个（至少要 3 个）",
                         ref_.size());
            return false;
        }
        if (ref_.size() > calib::kMaxK)
        {
            RCLCPP_ERROR(get_logger(),
                         "!! 参考点 %zu 个太多（最多 %zu 个），排列枚举会爆炸",
                         ref_.size(), calib::kMaxK);
            return false;
        }
        return true;
    }

    // -----------------------------------------------------------------------
    //  雷达回调：累积够就校准一次，然后永久锁定
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
            RCLCPP_INFO(get_logger(), "累积中 %d/%d 帧，当前点数 %zu",
                        detector_.frames(), detector_.cfg.accumulate_frames,
                        detector_.points());

        if (!detector_.ready()) return;

        done_ = true;
        runCalibration();

        RCLCPP_INFO(get_logger(), "★ 点云已锁定：%d 帧 / %zu 个点，后续不再更新",
                    detector_.frames(), detector_.points());
        RCLCPP_INFO(get_logger(), "本节点已空闲。按 Ctrl+C 结束。");
    }

    // -----------------------------------------------------------------------
    //  校准主流程
    // -----------------------------------------------------------------------
    void runCalibration()
    {
        const std::vector<Cone> cones = detector_.detect();

        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "============== 锥桶检测结果 ==============");
        RCLCPP_INFO(get_logger(), "累积 %d 帧，共 %zu 个点，检出 %zu 个候选",
                    detector_.frames(), detector_.points(), cones.size());

        const size_t k = ref_.size();

        if (cones.size() < k)
        {
            RCLCPP_ERROR(get_logger(), "!! 只检出 %zu 个候选，不足参考的 %zu 个",
                         cones.size(), k);
            printCandidates(cones, {});
            printVerdict(false, "检出锥桶数量不足", 0, 0, calib::Pose2D{}, 0, 0);
            publishMarkers(cones, {}, nullptr, calib::Pose2D{}, false,
                           "too few cones detected");
            return;
        }

        std::vector<Cone> near;
        for (const Cone & c : cones)
            if (c.range <= max_candidate_range_) near.push_back(c);

        if (near.size() < k)
        {
            RCLCPP_ERROR(get_logger(), "!! %.1fm 内候选只有 %zu 个，不足 %zu 个",
                         max_candidate_range_, near.size(), k);
            printCandidates(cones, {});
            printVerdict(false, "有效范围内锥桶数量不足", 0, 0, calib::Pose2D{}, 0, 0);
            publishMarkers(cones, {}, nullptr, calib::Pose2D{}, false,
                           "too few cones in range");
            return;
        }

        // ---- 指纹匹配（带位姿搜索窗口）----
        calib::Window win;
        win.max_yaw_rad         = max_yaw_deg_ * calib::kPi / 180.0;
        win.max_translation     = max_translation_;
        win.max_point_residual  = window_max_residual_;

        size_t used_n = 0;
        bool   trimmed = false;
        const calib::BestMatch best =
            calib::matchBest(near, ref_, win, max_combos_, &used_n, &trimmed);

        if (!best.ok)
        {
            RCLCPP_ERROR(get_logger(),
                         "!! 匹配失败：%zu 个候选组合里，没有任何一组能在搜索窗口内"
                         "（转角 < %.1f°，位移 < %.2fm）解出合理位姿",
                         best.combos_tried, max_yaw_deg_, max_translation_);
            RCLCPP_ERROR(get_logger(),
                         "   → 可能是车放得太偏，或者锥桶摆放与参考基准不同");
            printCandidates(cones, {});
            printVerdict(false, "搜索窗口内无可行解（车放得太偏或锥桶摆放不同）",
                         0, 0, calib::Pose2D{}, near.size(), best.combos_tried);
            publishMarkers(cones, {}, nullptr, calib::Pose2D{}, false,
                           "no valid pose in search window");
            return;
        }

        if (trimmed)
            RCLCPP_WARN(get_logger(),
                        "候选过多（%zu 个），只对最近的 %zu 个做组合枚举",
                        near.size(), used_n);

        // 参与匹配的候选（按距离升序取前 used_n 个，与 matchBest 内部一致）
        std::vector<Cone> pool(near.begin(), near.begin() + static_cast<long>(used_n));

        // ---- 求解位姿 ----
        const calib::Pose2D pose =
            calib::solveRigid(pool, best.qidx, ref_, best.perm);

        // 被选中的 k 个候选（供候选表打星）
        std::vector<Cone> selected;
        selected.reserve(best.qidx.size());
        for (size_t i : best.qidx) selected.push_back(pool[i]);

        printCandidates(cones, selected);
        printMatch(pool, best, pose);
        printSelfCheck(pool, best, pose);
        printSummary(pool, best, pose, cones.size());

        // ---- RViz 可视化 ----
        // 判定条件与 printSelfCheck() 里的 5 项自检保持一致
        const double yawdeg = std::fabs(pose.dyaw * 180.0 / calib::kPi);
        const double trans  = std::hypot(pose.dx, pose.dy);
        const bool pass = (best.mean_err <= max_match_error_) &&
                          (pose.max_res <= max_residual_) &&
                          (best.lambda >= min_lambda_) &&
                          (trans < max_translation_) &&
                          (yawdeg < max_yaw_deg_);

        // 文字在 publishMarkers 内部按 ASCII 拼装（RViz 不支持中文/换行）
        publishMarkers(cones, selected, &best, pose, pass, "");
    }

    // -----------------------------------------------------------------------
    //  RViz 可视化
    //    🟢 绿色大球 = 参考基准的 4 个锥桶（R0..R3）
    //    ⚫ 灰色小球 = 本次检出的其它候选
    //    🔵 蓝色球   = 本次匹配上的 4 个锥桶（C0..C3）
    //    🟡 黄色球   = 把蓝色球按解出的位姿【变换到参考系】后的位置
    //                  ★ 黄球应该和绿球几乎重合 —— 一眼就能看出匹配对不对
    //    🔷 青色线   = 对应关系（蓝球 → 绿球）
    //    🔴 红色箭头 = 车的位移偏差（放大 10 倍显示，方便看清方向）
    //    ⚪ 白色大字 = 结论
    // -----------------------------------------------------------------------
    void publishMarkers(const std::vector<Cone> & cones,
                        const std::vector<Cone> & selected,
                        const calib::BestMatch * best,
                        const calib::Pose2D & pose,
                        bool pass,
                        const std::string & reason)
    {
        viz::Painter p(frame_id_, now());
        p.deleteAll();

        auto isSelected = [&](const Cone & c) {
            for (const Cone & s : selected)
                if (std::fabs(s.cx - c.cx) < 1e-9 && std::fabs(s.cy - c.cy) < 1e-9)
                    return true;
            return false;
        };

        // ① 未匹配上的候选：灰色小球
        for (const Cone & c : cones)
            if (!isSelected(c))
                p.sphere("cand", c.cx, c.cy, 0.0, 0.16, viz::kGray);

        // ② 参考基准：绿色大球（描边 + 编号 + 坐标）
        for (size_t i = 0; i < ref_.size(); ++i)
        {
            p.sphere("ref_outline", ref_[i].cx, ref_[i].cy, 0.0, 0.46, viz::kDarkGreen);
            p.sphere("ref",         ref_[i].cx, ref_[i].cy, 0.0, 0.34, viz::kGreen);

            char buf[96];
            // ⚠️ 必须单行、纯 ASCII —— RViz 的文字用 Liberation Sans 渲染，
            //    不支持换行，也没有中文/emoji 字形（会整条渲染不出来）
            std::snprintf(buf, sizeof(buf), "R%zu (%.3f, %.3f)",
                          i, ref_[i].cx, ref_[i].cy);
            p.text("ref_label", ref_[i].cx, ref_[i].cy, 0.62, 0.18,
                   viz::kGreen, buf);
        }

        // ③ 匹配上的本次锥桶：蓝色球 + 对齐后的黄色球 + 对应连线
        if (best != nullptr)
        {
            const double c = std::cos(pose.dyaw), s = std::sin(pose.dyaw);
            for (size_t i = 0; i < best->qidx.size(); ++i)
            {
                const Cone & qc = selected[i];
                const Cone & rc = ref_[best->perm[i]];

                // 蓝球 = 本次实际检出的位置
                p.sphere("match", qc.cx, qc.cy, 0.0, 0.26, viz::kBlue);

                // 黄球 = 按解出的位姿变换到参考系后的位置（应与绿球重合）
                const double ax = c * qc.cx - s * qc.cy + pose.dx;
                const double ay = s * qc.cx + c * qc.cy + pose.dy;
                p.sphere("aligned", ax, ay, 0.06, 0.22, viz::kYellow);

                // 青线 = 对应关系
                p.segment("link", qc.cx, qc.cy, 0.0, rc.cx, rc.cy, 0.0,
                          0.012, viz::kCyan);

                const double res = std::hypot(ax - rc.cx, ay - rc.cy);
                char buf[96];
                std::snprintf(buf, sizeof(buf), "C%zu->R%zu  %.1fmm",
                              best->qidx[i], best->perm[i], res * 1000.0);
                p.text("match_label", rc.cx, rc.cy, 0.30, 0.14, viz::kBlue, buf);
            }

            // ④ 红色箭头：车的位移偏差（放大 10 倍显示方向）
            const double mag = std::hypot(pose.dx, pose.dy);
            if (mag > 1e-6)
            {
                p.arrow("pose", 0.0, 0.0, 0.05,
                        pose.dx * 10.0, pose.dy * 10.0, 0.05,
                        0.03, 0.05, viz::kRed);
                char buf[96];
                std::snprintf(buf, sizeof(buf), "shift %.1fcm (x10)",
                              mag * 100.0);
                p.text("pose_label", pose.dx * 5.0, pose.dy * 5.0, 0.35,
                       0.15, viz::kRed, buf);
            }
        }

        // ⑤ 状态灯 + 结论（放在锥桶区域【下方】，俯视时不会和锥桶重叠）
        //
        //  ⚠️ 两条硬约束（踩过坑）：
        //     1) RViz 的 Marker 文字用 Liberation Sans 渲染，【没有中文/emoji 字形】
        //        —— 写了中文会整条渲染不出来（终端里的中文是好的，只有 RViz 不行）
        //     2) 【不支持换行符 \n】
        //    所以这里一律：纯 ASCII、每行一个独立 marker。
        const double bx = 0.0, by = -1.45, bz = 0.30;
        const viz::Color col = pass ? viz::kGreen : viz::kRed;
        p.sphere("status", bx, by, bz, pass ? 0.55 : 0.45, col);

        if (best == nullptr)
        {
            // 失败路径：没有位姿可报，只显示原因（reason 已保证是纯 ASCII）
            p.text("verdict1", bx, by - 0.60, bz, 0.30, col,
                   pass ? "OK" : "FAIL");
            p.text("verdict2", bx, by - 0.95, bz, 0.20, col,
                   reason.empty() ? "see terminal output" : reason);
        }
        else
        {
            char line2[128], line3[128];
            std::snprintf(line2, sizeof(line2), "dx=%.1fcm dy=%.1fcm dyaw=%.2fdeg",
                          pose.dx * 100.0, pose.dy * 100.0,
                          pose.dyaw * 180.0 / calib::kPi);
            std::snprintf(line3, sizeof(line3), "err=%.1fmm maxres=%.1fmm",
                          best->mean_err * 1000.0, pose.max_res * 1000.0);
            p.text("verdict1", bx, by - 0.60, bz, 0.30, col,
                   pass ? "OK  READY TO GO" : "FAIL  REPLACE CAR");
            p.text("verdict2", bx, by - 0.95, bz, 0.22, col, line2);
            p.text("verdict3", bx, by - 1.25, bz, 0.20, col, line3);
        }

        marker_pub_->publish(p.array());
    }

    // -----------------------------------------------------------------------
    void printCandidates(const std::vector<Cone> & cones,
                         const std::vector<Cone> & selected)
    {
        auto isSelected = [&](const Cone & c) {
            for (const Cone & s : selected)
                if (std::fabs(s.cx - c.cx) < 1e-9 && std::fabs(s.cy - c.cy) < 1e-9)
                    return true;
            return false;
        };

        RCLCPP_INFO(get_logger(), "---- 全部候选（★ = 参与匹配的 %zu 个）----",
                    selected.size());
        RCLCPP_INFO(get_logger(), "%-4s %-4s %-21s %-10s %-10s %-7s",
                    "id", "选中", "坐标(x, y)", "距离(m)", "宽度(m)", "点数");
        for (size_t i = 0; i < cones.size(); ++i)
            RCLCPP_INFO(get_logger(), "%-4zu %-4s (%7.3f, %7.3f)  %8.3f   %8.3f   %5d",
                        i, (isSelected(cones[i]) ? "★" : "  "),
                        cones[i].cx, cones[i].cy, cones[i].range,
                        cones[i].width, cones[i].n_points);
    }

    // -----------------------------------------------------------------------
    void printMatch(const std::vector<Cone> & pool,
                    const calib::BestMatch & best,
                    const calib::Pose2D & pose)
    {
        const size_t k = best.qidx.size();
        const double npairs = static_cast<double>(k * (k - 1) / 2);

        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "========== 指纹匹配结果 ==========");
        RCLCPP_INFO(get_logger(), "枚举组合数 = %zu（参考 %zu 个点）",
                    best.combos_tried, k);
        RCLCPP_INFO(get_logger(), "搜索窗口: 转角 < %.1f°，位移 < %.2f m",
                    max_yaw_deg_, max_translation_);
        if (best.combos_no_window > 0)
            RCLCPP_INFO(get_logger(), "  （其中有 %zu 个组合在窗口内无解，已排除）",
                        best.combos_no_window);

        RCLCPP_INFO(get_logger(), "最佳排列: 距离误差总和 = %.4f m，平均每对 = %.4f m",
                    best.best_cost, best.mean_err);
        if (best.lambda >= 1e8)
        {
            RCLCPP_INFO(get_logger(), "判别力 λ: 窗口内【唯一解】（无并列排列）✅");
        }
        else
        {
            RCLCPP_INFO(get_logger(), "次佳排列: 距离误差总和 = %.4f m（平均每对 = %.4f m）",
                        best.second_cost, best.second_cost / npairs);
            RCLCPP_INFO(get_logger(), "判别力 λ = 次佳/最佳 = %.2f", best.lambda);
        }

        // 诊断：如果"忽略窗口"的最优解比窗口内最优更好，说明窗口排除掉了一个更贴合的解
        //       —— 那通常正是对称布局的假解（例如矩形的 180° 翻转），是好事
        if (best.winner_cost_any < best.best_cost - 1e-9)
            RCLCPP_INFO(get_logger(),
                        "  （注：忽略窗口时还有一个误差更小 %.4f m 的解，"
                        "已被窗口排除 —— 通常是布局对称造成的假解）",
                        best.winner_cost_any);

        if (best.second_combo_mean < 1e17)
            RCLCPP_INFO(get_logger(), "次佳【组合】的平均每对误差 = %.4f m（越大越说明选对了组合）",
                        best.second_combo_mean);
        else
            RCLCPP_INFO(get_logger(), "次佳【组合】: 无（候选数正好等于参考数）");

        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "---- 对应关系与点位残差 ----");
        RCLCPP_INFO(get_logger(), "%-5s %-21s %-6s %-21s %-11s",
                    "候选#", "查询坐标(x, y)", "参考#", "参考坐标(x, y)", "残差(m)");

        const double c = std::cos(pose.dyaw), s = std::sin(pose.dyaw);
        for (size_t i = 0; i < k; ++i)
        {
            const Cone & qc = pool[best.qidx[i]];
            const Cone & rc = ref_[best.perm[i]];
            const double px = c * qc.cx - s * qc.cy + pose.dx;
            const double py = s * qc.cx + c * qc.cy + pose.dy;
            const double res = std::hypot(px - rc.cx, py - rc.cy);
            RCLCPP_INFO(get_logger(), "%-5zu (%7.3f, %7.3f)   %-6zu (%7.3f, %7.3f)  %8.4f",
                        best.qidx[i], qc.cx, qc.cy, best.perm[i], rc.cx, rc.cy, res);
        }

        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "---- 位姿解算（2D Kabsch 闭式解）----");
        RCLCPP_INFO(get_logger(), "  dx   = %+.4f m  (%+.2f cm)", pose.dx, pose.dx * 100.0);
        RCLCPP_INFO(get_logger(), "  dy   = %+.4f m  (%+.2f cm)", pose.dy, pose.dy * 100.0);
        RCLCPP_INFO(get_logger(), "  dyaw = %+.4f rad  (%+.2f deg)",
                    pose.dyaw, pose.dyaw * 180.0 / calib::kPi);
        RCLCPP_INFO(get_logger(), "  位移量 |t| = %.4f m", std::hypot(pose.dx, pose.dy));
        RCLCPP_INFO(get_logger(), "  点位残差 RMS = %.4f m   单点最大残差 = %.4f m",
                    pose.rms, pose.max_res);
    }

    // -----------------------------------------------------------------------
    void printSelfCheck(const std::vector<Cone> & pool,
                        const calib::BestMatch & best,
                        const calib::Pose2D & pose)
    {
        const double trans  = std::hypot(pose.dx, pose.dy);
        const double yawdeg = std::fabs(pose.dyaw * 180.0 / calib::kPi);

        const bool ok_err     = (best.mean_err <= max_match_error_);
        const bool ok_lambda  = (best.lambda >= min_lambda_);
        const bool ok_trans   = (trans < max_translation_);
        const bool ok_yaw     = (yawdeg < max_yaw_deg_);
        const bool ok_res     = (pose.max_res <= max_residual_);

        auto mark = [](bool b, bool hard_fail) {
            if (b) return "✅";
            return hard_fail ? "❌" : "⚠️";
        };

        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "---- 自检 ----");
        RCLCPP_INFO(get_logger(), "  [1] 距离匹配平均误差 %.4f m  (阈值 < %.3f, 失败 > %.3f)  %s",
                    best.mean_err, max_match_error_, fail_match_error_,
                    mark(ok_err, best.mean_err > fail_match_error_));
        RCLCPP_INFO(get_logger(), "  [2] 判别力 λ = %.2f          (阈值 > %.0f)                    %s",
                    best.lambda, min_lambda_, mark(ok_lambda, true));
        RCLCPP_INFO(get_logger(), "  [3] 位移量 |t| = %.4f m     (阈值 < %.2f)                   %s",
                    trans, max_translation_, mark(ok_trans, true));
        RCLCPP_INFO(get_logger(), "  [4] 转角 |dyaw| = %.2f°     (阈值 < %.1f°)                  %s",
                    yawdeg, max_yaw_deg_, mark(ok_yaw, true));
        RCLCPP_INFO(get_logger(), "  [5] 单点最大残差 %.4f m    (阈值 < %.3f)                  %s",
                    pose.max_res, max_residual_, mark(ok_res, true));
        RCLCPP_INFO(get_logger(), "      → [5] 专门盯\"某个锥桶被单独挪动\"：实测挪 δ 时");
        RCLCPP_INFO(get_logger(), "        单点残差 ≈ 6δ，比平均距离误差(≈0.27δ)敏感得多");

        std::string reason;
        if (!ok_err)
            reason = best.mean_err > fail_match_error_
                   ? "距离指纹对不上（误差过大）"
                   : "距离指纹误差偏大";
        else if (!ok_res)    reason = "单点残差超限：某个锥桶的位置与参考不符（锥桶被挪动了？）";
        else if (!ok_lambda) reason = "存在并列解（布局对称或检测异常）";
        else if (!ok_trans)  reason = "位移量超限，车放得太偏";
        else if (!ok_yaw)    reason = "转角超限，车放得太歪";

        printVerdict(ok_err && ok_res && ok_lambda && ok_trans && ok_yaw, reason,
                     best.mean_err, best.lambda, pose, pool.size(), best.combos_tried);
    }

    // -----------------------------------------------------------------------
    void printVerdict(bool pass, const std::string & reason,
                      double mean_err, double lambda,
                      const calib::Pose2D & pose,
                      size_t n_pool, size_t combos)
    {
        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "====================================");
        if (pass)
        {
            RCLCPP_INFO(get_logger(), "  ✅ 校准成功，可以起跑");
            RCLCPP_INFO(get_logger(), "  偏差: dx=%.1fcm  dy=%.1fcm  dyaw=%.2f°",
                        pose.dx * 100.0, pose.dy * 100.0,
                        pose.dyaw * 180.0 / calib::kPi);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "  ❌ 校准失败：%s", reason.c_str());
            RCLCPP_INFO(get_logger(), "  → 请把车重新摆放后再试");
        }
        RCLCPP_INFO(get_logger(), "====================================");
        RCLCPP_INFO(get_logger(), "");

        RCLCPP_INFO(get_logger(), "============= 复制下面这段发给 AI =============");
        RCLCPP_INFO(get_logger(),
                    "CALIB ok=%d dx=%.4f dy=%.4f dyaw=%.3f err=%.4f lambda=%.2f rms=%.4f "
                    "pool=%zu combos=%zu",
                    pass ? 1 : 0, pose.dx, pose.dy,
                    pose.dyaw * 180.0 / calib::kPi,
                    mean_err, lambda, pose.rms, n_pool, combos);
        RCLCPP_INFO(get_logger(), "reason: %s",
                    pass ? "ok" : reason.c_str());
        RCLCPP_INFO(get_logger(), "==============================================");
    }

    void printSummary(const std::vector<Cone> & pool,
                      const calib::BestMatch & best,
                      const calib::Pose2D & pose, size_t n_all)
    {
        std::ostringstream m;
        for (size_t i = 0; i < best.qidx.size(); ++i)
        {
            if (i) m << " ";
            m << "q" << best.qidx[i] << "->r" << best.perm[i];
        }
        RCLCPP_INFO(get_logger(), "match: %s", m.str().c_str());
        RCLCPP_INFO(get_logger(), "（全部候选 %zu 个，参与匹配 %zu 个）",
                    n_all, pool.size());
        RCLCPP_INFO(get_logger(),
                    "pose_mm: dx=%.2f dy=%.2f dyaw_deg=%.3f rms=%.2f maxres=%.2f",
                    pose.dx * 1000.0, pose.dy * 1000.0,
                    pose.dyaw * 180.0 / calib::kPi,
                    pose.rms * 1000.0, pose.max_res * 1000.0);
    }

    // ---- 成员 ----
    ConeDetector detector_;
    std::vector<Cone> ref_;
    double      max_candidate_range_{6.0};
    size_t      max_combos_{500000};
    // ★ 与 cone_calib_record 的默认输出路径保持一致：
    //    参考基准放【仓库之外】的 calib_logs/ 下，避免录制把 git 工作区弄脏。
    std::string ref_path_{"/home/racecar/calib_logs/cones_ref.csv"};
    double      max_match_error_{0.03};
    double      fail_match_error_{0.10};
    // 单点最大残差限值：实测正确匹配时约 1.7mm，取 25mm 有 ~15 倍余量，
    // 同时能抓出"单个锥桶被挪动 ≳4cm"（挪 δ 时单点残差 ≈ 6δ）。
    // 若实车出现误拒，先看是不是雷达噪声大，再把这里放宽到 0.03~0.04。
    double      max_residual_{0.025};
    double      min_lambda_{10.0};
    double      max_translation_{0.50};
    double      max_yaw_deg_{20.0};
    // 搜索阶段的点位残差上限。
    // ★ 这一条必须有！实测发现矩形布局存在"左右镜像"的假排列：
    //   它 6 个距离几乎全对（代价只差 5 倍）、质心几乎重合（位移 8mm）、
    //   转角也小（2°）—— 只约束"转角+位移"拦不住它，它的点位残差高达 1 米。
    //   取 0.15（宽松）：只用来剔除这种"米级"的荒谬排列，
    //   精细判别交给自检 [5]（0.025），这样"某个锥桶被挪动"的情况
    //   仍然能被选出、再由自检 [5] 判 ❌，不会误选成别的组合。
    double      window_max_residual_{0.15};
    std::string frame_id_{"laser_link"};
    bool        frame_from_scan_{false};
    bool        done_{false};
    bool        ok_{true};

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ConeCalibRunNode>();
    if (!node->ok())
    {
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

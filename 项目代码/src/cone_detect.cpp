// ============================================================================
//  cone_detect.cpp —— 锥桶检测节点（四锥桶定位第一步）
// ============================================================================
//  作用：
//      上电后车静止，累积多帧雷达点云，用 2D 欧氏聚类找出锥桶，
//      选出最近的 N 个作为候选参考锥桶，输出坐标 + 成对距离指纹，
//      并在 RViz 中用颜色区分"选中的"和"被淘汰的"候选。
//
//  为什么这么设计：
//      1. 累积多帧 —— 补漏检、压噪声、抗人群（人在动，锥桶不动）
//      2. 2D 欧氏聚类（不是角度聚类）—— 防止人和锥桶在角度上粘连
//      3. 按"簇宽度"过滤 —— 锥桶约 10~20cm，人约 40~60cm，可区分
//      4. 打印成对距离 —— 距离是旋转平移不变量，用来跨次核对检测稳定性
//      5. RViz 用颜色区分选中/淘汰 —— 防止"取最近N个"把参考锥桶挤掉
//
//  行为（重要）：
//      只累积【前 N 帧】(默认 20 帧 ≈ 0.4 秒) 的雷达数据，
//      累积够就检测一次，然后【永久锁定】——
//      点云不再更新，检测结果固定不变。
//      （所以车必须在启动后的这 0.4 秒内保持静止）
//
//      话题用 transient_local（锁存）发布，
//      所以即使 RViz 晚开，也能收到那一条结果。
//
//  使用：
//      ros2 run my_racecar cone_detect
//      ros2 run my_racecar cone_detect --ros-args -p max_cones:=4 -p accumulate_frames:=30
//
//  RViz 观察：
//      Fixed Frame: laser_link
//      Add -> By topic -> /cone_detect/markers -> MarkerArray
//         绿色大球  = 选中的参考锥桶（带编号和距离）
//         灰色小球  = 被淘汰的候选（说明附近有多余锥桶，要警惕）
//         黄色线段  = 选中锥桶之间的连线（直观显示几何形状）
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <numeric>
#include <vector>
#include <string>
#include <functional>
#include <map>
#include <array>

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

// 一个检测到的锥桶
struct Cone
{
    double cx{0.0};        // 质心 x（雷达系，米）
    double cy{0.0};        // 质心 y（雷达系，米）
    double range{0.0};     // 到雷达的距离
    double width{0.0};     // 估计宽度（簇内最大点距）
    int    n_points{0};    // 簇内点数
};

// ---------------------------------------------------------------------------
//  锥桶检测器
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
                                           // 太小：一个锥桶被拆成多簇
                                           // 太大：相邻锥桶或人粘在一起

        // ---- 簇过滤 ----
        int    min_points        = 4;      // 簇最少点数（滤噪点）
        int    max_points        = 120;    // 簇最多点数（超过可能是墙/大物体）
        double min_width         = 0.03;   // 簇最小宽度（米）
        double max_width         = 0.35;   // 簇最大宽度（米）★ 人在这里被剔除
                                           // 锥桶约 0.10~0.25，人约 0.40~0.60
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

    void reset()
    {
        accum_.clear();
        frame_count_ = 0;
    }

    // 聚类 + 过滤，返回按距离升序排列的锥桶
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

            // 宽度 = 簇内任意两点最大距离（簇通常只有几十点，暴力够快）
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
//  ROS 节点
// ---------------------------------------------------------------------------
class ConeDetectNode : public rclcpp::Node
{
public:
    ConeDetectNode() : Node("cone_detect")
    {
        // ---- 参数声明 ----
        declare_parameter("min_range",          detector_.cfg.min_range);
        declare_parameter("max_range",          detector_.cfg.max_range);
        declare_parameter("accumulate_frames",  detector_.cfg.accumulate_frames);
        declare_parameter("cluster_tol",        detector_.cfg.cluster_tol);
        declare_parameter("min_points",         detector_.cfg.min_points);
        declare_parameter("max_points",         detector_.cfg.max_points);
        declare_parameter("min_width",          detector_.cfg.min_width);
        declare_parameter("max_width",          detector_.cfg.max_width);
        declare_parameter("max_cones",          max_cones_);
        declare_parameter("frame_id",           frame_id_);

        // ---- 读取 ----
        detector_.cfg.min_range         = get_parameter("min_range").as_double();
        detector_.cfg.max_range         = get_parameter("max_range").as_double();
        detector_.cfg.accumulate_frames = get_parameter("accumulate_frames").as_int();
        detector_.cfg.cluster_tol       = get_parameter("cluster_tol").as_double();
        detector_.cfg.min_points        = get_parameter("min_points").as_int();
        detector_.cfg.max_points        = get_parameter("max_points").as_int();
        detector_.cfg.min_width         = get_parameter("min_width").as_double();
        detector_.cfg.max_width         = get_parameter("max_width").as_double();
        max_cones_                      = get_parameter("max_cones").as_int();
        frame_id_                       = get_parameter("frame_id").as_string();

        // ---- 订阅雷达 ----
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&ConeDetectNode::scanCallback, this, std::placeholders::_1));

        // ---- 发布 ----
        // 注意：本节点只发一次消息，所以用 transient_local（锁存）
        //       这样即使 RViz 晚开，也能收到那一条历史消息
        rclcpp::QoS latched_qos(10);
        latched_qos.transient_local();

        result_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/cone_detect/result", latched_qos);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/cone_detect/markers", latched_qos);

        RCLCPP_INFO(get_logger(), "cone_detect 已启动，等待 %d 帧雷达数据...",
                    detector_.cfg.accumulate_frames);
        RCLCPP_INFO(get_logger(),
                    "参数: cluster_tol=%.2f  max_width=%.2f  max_cones=%d  frame_id=%s",
                    detector_.cfg.cluster_tol, detector_.cfg.max_width,
                    max_cones_, frame_id_.c_str());
    }

private:
    // -----------------------------------------------------------------------
    //  雷达回调：只累积前 N 帧，够了就检测一次，之后永久停止
    //  点云不再更新，检测结果固定不变
    // -----------------------------------------------------------------------
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 已经完成 → 什么都不做（数据固定）
        if (done_) return;

        detector_.addScan(msg);

        // 每 5 帧打印一次进度
        if (!detector_.ready() && detector_.frames() % 5 == 0)
        {
            RCLCPP_INFO(get_logger(), "累积中 %d/%d 帧，当前点数 %zu",
                        detector_.frames(), detector_.cfg.accumulate_frames,
                        detector_.points());
        }

        if (!detector_.ready()) return;

        // ★ 攒够 N 帧 → 检测一次 → 之后永久固定
        done_ = true;
        runDetection();

        RCLCPP_INFO(get_logger(), "★ 点云已锁定：%d 帧 / %zu 个点，后续不再累积、不再更新",
                    detector_.frames(), detector_.points());
    }

    // -----------------------------------------------------------------------
    //  执行检测 + 打印结果
    // -----------------------------------------------------------------------
    void runDetection()
    {
        auto cones = detector_.detect();

        RCLCPP_INFO(get_logger(), "");
        RCLCPP_INFO(get_logger(), "================ 锥桶检测结果 ================");
        RCLCPP_INFO(get_logger(), "累积 %d 帧，共 %zu 个点，检出 %zu 个候选",
                    detector_.frames(), detector_.points(), cones.size());

        if (cones.empty())
        {
            RCLCPP_ERROR(get_logger(),
                         "!! 一个锥桶都没检出，检查 max_width / min_points / max_range");
            RCLCPP_INFO(get_logger(), "==============================================");
            publishMarkers({}, {});
            publishResult({});
            return;
        }

        // ---- 分成"选中"和"淘汰"两组 ----
        const size_t keep = std::min(cones.size(), static_cast<size_t>(max_cones_));
        std::vector<Cone> selected(cones.begin(), cones.begin() + keep);
        std::vector<Cone> rejected(cones.begin() + keep, cones.end());

        // ---- 打印候选全表 ----
        RCLCPP_INFO(get_logger(), "%-4s %-4s %-19s %-10s %-10s %-7s",
                    "id", "选中", "坐标(x, y)", "距离(m)", "宽度(m)", "点数");
        for (size_t i = 0; i < cones.size(); ++i)
        {
            RCLCPP_INFO(get_logger(), "%-4zu %-4s (%7.3f, %7.3f)  %8.3f   %8.3f   %5d",
                        i, (i < keep ? "★" : "  "),
                        cones[i].cx, cones[i].cy, cones[i].range,
                        cones[i].width, cones[i].n_points);
        }

        // ---- ★ 成对距离指纹（核心校验手段）----
        RCLCPP_INFO(get_logger(), "---- 成对距离指纹（旋转平移不变量）----");
        for (size_t i = 0; i < selected.size(); ++i)
        {
            for (size_t j = i + 1; j < selected.size(); ++j)
            {
                double d = std::hypot(selected[i].cx - selected[j].cx,
                                      selected[i].cy - selected[j].cy);
                RCLCPP_INFO(get_logger(), "  %zu-%zu : %.4f m", i, j, d);
            }
        }

        // ---- 几何分散度检查（判断解位姿的条件好不好）----
        if (selected.size() >= 3)
            checkGeometry(selected);

        // ---- 警告：有候选被淘汰 ----
        if (!rejected.empty())
        {
            RCLCPP_WARN(get_logger(),
                        "检出 %zu 个候选，只取了最近的 %zu 个，"
                        "%zu 个被淘汰（RViz 里是灰色球）",
                        cones.size(), keep, rejected.size());
            RCLCPP_WARN(get_logger(),
                        "  确认被淘汰的不是你需要的参考锥桶！");
        }

        RCLCPP_INFO(get_logger(), "==============================================");
        RCLCPP_INFO(get_logger(), "");

        // ---- 发布 ----
        publishResult(selected);
        publishMarkers(selected, rejected);
    }

    // -----------------------------------------------------------------------
    //  几何分散度检查：4 个点越接近直线，解位姿越不稳
    // -----------------------------------------------------------------------
    void checkGeometry(const std::vector<Cone> & sel)
    {
        // 用 PCA 判断"点集的形状"：
        //   主方向方差 vs 次方向方差
        //   比值越大 → 越接近直线 → 航向估计越差
        const int m = static_cast<int>(sel.size());

        double mx = 0, my = 0;
        for (const auto & c : sel) { mx += c.cx; my += c.cy; }
        mx /= m; my /= m;

        double sxx = 0, syy = 0, sxy = 0;
        for (const auto & c : sel)
        {
            double dx = c.cx - mx, dy = c.cy - my;
            sxx += dx * dx;  syy += dy * dy;  sxy += dx * dy;
        }
        sxx /= m; syy /= m; sxy /= m;

        // 2x2 对称矩阵特征值
        double tr = sxx + syy;
        double det = sxx * syy - sxy * sxy;
        double disc = std::sqrt(std::max(0.0, tr * tr / 4.0 - det));
        double l1 = tr / 2.0 + disc;     // 大特征值（主方向）
        double l2 = tr / 2.0 - disc;     // 小特征值（次方向）

        double ratio = (l2 > 1e-9) ? (l1 / l2) : 1e9;

        RCLCPP_INFO(get_logger(),
                    "几何分散度: 主方向散布 %.3f m, 次方向散布 %.3f m, 比值 %.1f",
                    std::sqrt(l1), std::sqrt(std::max(0.0, l2)), ratio);

        if (ratio > 20.0)
        {
            RCLCPP_WARN(get_logger(),
                        "!! 锥桶几乎排成一条直线（比值 %.1f > 20）"
                        "—— 航向估计会很不准，建议调整锥桶布局", ratio);
        }
        else if (ratio < 5.0)
        {
            RCLCPP_INFO(get_logger(), "    几何形状良好（分散），适合解位姿 ✓");
        }
    }

    // -----------------------------------------------------------------------
    //  发布结果话题
    //  格式: [n, x0,y0,r0,w0, x1,y1,r1,w1, ...]
    // -----------------------------------------------------------------------
    void publishResult(const std::vector<Cone> & cones)
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data.push_back(static_cast<double>(cones.size()));
        for (const auto & c : cones)
        {
            msg.data.push_back(c.cx);
            msg.data.push_back(c.cy);
            msg.data.push_back(c.range);
            msg.data.push_back(c.width);
        }
        result_pub_->publish(msg);
    }

    // -----------------------------------------------------------------------
    //  RViz 可视化
    //    绿色大球  = 选中的参考锥桶
    //    灰色小球  = 被淘汰的候选
    //    黄色线段  = 选中锥桶之间的连线（显示几何形状）
    //    白色文字  = 编号 + 距离
    // -----------------------------------------------------------------------
    void publishMarkers(const std::vector<Cone> & selected,
                        const std::vector<Cone> & rejected)
    {
        visualization_msgs::msg::MarkerArray arr;

        // 先清掉上一轮
        {
            visualization_msgs::msg::Marker del;
            del.header.frame_id = frame_id_;
            del.header.stamp = now();
            del.action = visualization_msgs::msg::Marker::DELETEALL;
            arr.markers.push_back(del);
        }

        const auto stamp = now();
        int id = 0;

        // ---- ① 被淘汰的候选：灰色半透明小球 ----
        for (size_t i = 0; i < rejected.size(); ++i)
        {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = frame_id_;
            m.header.stamp = stamp;
            m.ns = "cone_rejected";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = rejected[i].cx;
            m.pose.position.y = rejected[i].cy;
            m.pose.position.z = 0.0;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = m.scale.z = 0.28;   // 比选中的小
            m.color.r = 0.5f; m.color.g = 0.5f; m.color.b = 0.5f; m.color.a = 0.55f;
            m.lifetime = rclcpp::Duration::from_seconds(0);
            arr.markers.push_back(m);
        }

        // ---- ② 选中的参考锥桶：绿色大球（带黑边效果，用两层球）----
        for (size_t i = 0; i < selected.size(); ++i)
        {
            // 外层：稍大的暗绿球，做出"描边"效果
            {
                visualization_msgs::msg::Marker m;
                m.header.frame_id = frame_id_;
                m.header.stamp = stamp;
                m.ns = "cone_outline";
                m.id = id++;
                m.type = visualization_msgs::msg::Marker::SPHERE;
                m.action = visualization_msgs::msg::Marker::ADD;
                m.pose.position.x = selected[i].cx;
                m.pose.position.y = selected[i].cy;
                m.pose.position.z = 0.0;
                m.pose.orientation.w = 1.0;
                m.scale.x = m.scale.y = m.scale.z = 0.46;
                m.color.r = 0.0f; m.color.g = 0.25f; m.color.b = 0.0f; m.color.a = 1.0f;
                m.lifetime = rclcpp::Duration::from_seconds(0);
                arr.markers.push_back(m);
            }
            // 内层：亮绿球
            {
                visualization_msgs::msg::Marker m;
                m.header.frame_id = frame_id_;
                m.header.stamp = stamp;
                m.ns = "cone_selected";
                m.id = id++;
                m.type = visualization_msgs::msg::Marker::SPHERE;
                m.action = visualization_msgs::msg::Marker::ADD;
                m.pose.position.x = selected[i].cx;
                m.pose.position.y = selected[i].cy;
                m.pose.position.z = 0.0;
                m.pose.orientation.w = 1.0;
                m.scale.x = m.scale.y = m.scale.z = 0.34;
                m.color.r = 0.1f; m.color.g = 1.0f; m.color.b = 0.1f; m.color.a = 1.0f;
                m.lifetime = rclcpp::Duration::from_seconds(0);
                arr.markers.push_back(m);
            }

            // 文字：编号 + 距离
            {
                visualization_msgs::msg::Marker t;
                t.header.frame_id = frame_id_;
                t.header.stamp = stamp;
                t.ns = "cone_label";
                t.id = id++;
                t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                t.action = visualization_msgs::msg::Marker::ADD;
                t.pose.position.x = selected[i].cx;
                t.pose.position.y = selected[i].cy;
                t.pose.position.z = 0.60;
                t.pose.orientation.w = 1.0;
                t.scale.z = 0.22;                    // 字高
                t.color.r = 1.0f; t.color.g = 1.0f; t.color.b = 1.0f; t.color.a = 1.0f;

                char buf[64];
                std::snprintf(buf, sizeof(buf), "#%zu  %.2fm", i, selected[i].range);
                t.text = buf;
                t.lifetime = rclcpp::Duration::from_seconds(0);
                arr.markers.push_back(t);
            }
        }

        // ---- ③ 选中锥桶之间的连线（显示几何形状）----
        if (selected.size() >= 2)
        {
            visualization_msgs::msg::Marker line;
            line.header.frame_id = frame_id_;
            line.header.stamp = stamp;
            line.ns = "cone_edges";
            line.id = id++;
            line.type = visualization_msgs::msg::Marker::LINE_LIST;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.scale.x = 0.02;                    // 线宽
            line.color.r = 1.0f; line.color.g = 1.0f; line.color.b = 0.0f; line.color.a = 0.9f;
            line.lifetime = rclcpp::Duration::from_seconds(0);

            // 所有两两组合连起来（菱形/多边形的边 + 对角线）
            for (size_t i = 0; i < selected.size(); ++i)
            {
                for (size_t j = i + 1; j < selected.size(); ++j)
                {
                    geometry_msgs::msg::Point p1, p2;
                    p1.x = selected[i].cx;  p1.y = selected[i].cy;  p1.z = 0.02;
                    p2.x = selected[j].cx;  p2.y = selected[j].cy;  p2.z = 0.02;
                    line.points.push_back(p1);
                    line.points.push_back(p2);
                }
            }
            arr.markers.push_back(line);
        }

        // ---- ④ 每两个选中锥桶的距离数字（标在中点，方便人工记指纹）----
        for (size_t i = 0; i < selected.size(); ++i)
        {
            for (size_t j = i + 1; j < selected.size(); ++j)
            {
                visualization_msgs::msg::Marker t;
                t.header.frame_id = frame_id_;
                t.header.stamp = stamp;
                t.ns = "cone_edge_len";
                t.id = id++;
                t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
                t.action = visualization_msgs::msg::Marker::ADD;
                t.pose.position.x = (selected[i].cx + selected[j].cx) / 2.0;
                t.pose.position.y = (selected[i].cy + selected[j].cy) / 2.0;
                t.pose.position.z = 0.30;
                t.pose.orientation.w = 1.0;
                t.scale.z = 0.16;
                t.color.r = 1.0f; t.color.g = 0.9f; t.color.b = 0.2f; t.color.a = 0.95f;

                double d = std::hypot(selected[i].cx - selected[j].cx,
                                      selected[i].cy - selected[j].cy);
                char buf[32];
                std::snprintf(buf, sizeof(buf), "%.2f", d);
                t.text = buf;
                t.lifetime = rclcpp::Duration::from_seconds(0);
                arr.markers.push_back(t);
            }
        }

        marker_pub_->publish(arr);
    }

    // ---- 成员 ----
    ConeDetector detector_;
    int         max_cones_{4};
    bool        done_{false};
    std::string frame_id_{"laser_link"};

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr result_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ConeDetectNode>());
    rclcpp::shutdown();
    return 0;
}

// ============================================================================
//  cone_viz.hpp —— 锥桶可视化共用的 Marker 构造工具
// ============================================================================
//  给 cone_calib_record / cone_calib_run 两个节点共用，避免重复代码。
//
//  用法（在节点里）：
//      viz::Painter p(frame_id_, now());
//      p.deleteAll();                                  // 先清上一轮
//      p.sphere("cand", x, y, 0.0, 0.20, viz::kGray);  // 灰色小球
//      p.text("label", x, y, 0.6, 0.22, viz::kWhite, "R0");
//      p.segment("link", x1,y1,0, x2,y2,0, 0.02, viz::kCyan);
//      p.bigText("verdict", 0.0, 0.0, 1.8, "✅ 可以起跑");
//      marker_pub_->publish(p.array());
//
//  ⚠️ 发布时必须用 transient_local（锁存）QoS ——
//     因为节点只发一次就空闲了，RViz 晚开也要能收到。
//     RViz 那边对应的设置是 Durability Policy = Transient Local
//     （rviz/cone_calib.rviz 预设里已经写好）。
// ============================================================================
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>

namespace viz
{

// ---------------------------------------------------------------------------
//  颜色
// ---------------------------------------------------------------------------
struct Color
{
    float r{1.0f}, g{1.0f}, b{1.0f}, a{1.0f};
};

constexpr Color kGreen     {0.10f, 1.00f, 0.10f, 1.00f};   // 参考锥桶
constexpr Color kDarkGreen {0.00f, 0.30f, 0.00f, 1.00f};   // 参考锥桶描边
constexpr Color kGray      {0.60f, 0.60f, 0.60f, 0.30f};   // 未选中的候选
constexpr Color kBlue      {0.20f, 0.55f, 1.00f, 1.00f};   // 本次匹配上的锥桶
constexpr Color kYellow    {1.00f, 0.88f, 0.15f, 1.00f};   // 对齐后位置 / 连线
constexpr Color kCyan      {0.15f, 0.90f, 0.90f, 0.90f};   // 对应关系
constexpr Color kRed       {1.00f, 0.25f, 0.20f, 1.00f};   // 位姿偏差
constexpr Color kWhite     {1.00f, 1.00f, 1.00f, 1.00f};
constexpr Color kOrange    {1.00f, 0.60f, 0.10f, 1.00f};

// ---------------------------------------------------------------------------
//  Painter —— 往 MarkerArray 里累积各种图形
// ---------------------------------------------------------------------------
class Painter
{
public:
    Painter(const std::string & frame, const rclcpp::Time & stamp)
        : frame_(frame), stamp_(stamp) {}

    visualization_msgs::msg::MarkerArray & array() { return arr_; }

    // 先清掉上一轮显示（RViz 会保留旧 marker，状态变化时必须清）
    void deleteAll()
    {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame_;
        m.header.stamp = stamp_;
        m.action = visualization_msgs::msg::Marker::DELETEALL;
        arr_.markers.push_back(m);
    }

    // ---- 球 ----
    void sphere(const std::string & ns, double x, double y, double z,
                double size, const Color & c)
    {
        auto & m = add(ns, visualization_msgs::msg::Marker::SPHERE, c);
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = z;
        m.scale.x = m.scale.y = m.scale.z = size;
    }

    // ---- 文字（永远面向相机）----
    //
    //  ⚠️⚠️ RViz 的文字渲染有两条硬限制（踩过坑，务必遵守）：
    //    1) 【没有中文字形】—— RViz2 用 Liberation Sans 字体渲染 Marker 文字，
    //       字体里没有 CJK 字形。写了中文 → 整条文字渲染不出来（不是显示成方框，
    //       是【什么都不显示】）。终端里的中文没问题，只有 RViz 不行。
    //    2) 【不支持换行 \n】—— 同样会导致整条渲染失败。
    //
    //  所以本函数会【自动把非 ASCII 字符和换行替换掉】，写错了也不会整条消失。
    //  要显示多行请发多个 text() marker，各自放在不同的 y 位置上。
    void text(const std::string & ns, double x, double y, double z,
              double height, const Color & c, const std::string & s)
    {
        auto & m = add(ns, visualization_msgs::msg::Marker::TEXT_VIEW_FACING, c);
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = z;
        m.scale.z = height;
        m.text = asciiSafe(s);
    }

    // ---- 线段 ----
    void segment(const std::string & ns, double x1, double y1, double z1,
                 double x2, double y2, double z2,
                 double width, const Color & c)
    {
        auto & m = add(ns, visualization_msgs::msg::Marker::LINE_LIST, c);
        m.scale.x = width;
        m.points.push_back(pt(x1, y1, z1));
        m.points.push_back(pt(x2, y2, z2));
    }

    // ---- 箭头（ARROW：scale.x=杆径, scale.y=箭头直径, scale.z=箭头长度）----
    void arrow(const std::string & ns, double x1, double y1, double z1,
               double x2, double y2, double z2,
               double shaft, double head, const Color & c)
    {
        auto & m = add(ns, visualization_msgs::msg::Marker::ARROW, c);
        m.scale.x = shaft;
        m.scale.y = head;
        m.scale.z = head;
        // 注意：ARROW 用 points 形式时，scale.z 不起作用（长度由两点决定）
        m.points.push_back(pt(x1, y1, z1));
        m.points.push_back(pt(x2, y2, z2));
    }

    // ---- 大字（结论用，固定摆在车前方上空）----
    void bigText(const std::string & ns, double x, double y, double z,
                 const Color & c, const std::string & s)
    {
        text(ns, x, y, z, 0.34, c, s);
    }

private:
    // 把非 ASCII 字符和换行替换掉 —— RViz 的字体渲染不了它们，
    // 留着会让【整条文字】都不显示（这是踩过的坑，见 text() 的注释）
    static std::string asciiSafe(const std::string & s)
    {
        std::string out;
        out.reserve(s.size());
        for (unsigned char ch : s)
        {
            if (ch == '\n' || ch == '\r' || ch == '\t') { out += ' '; continue; }
            out += (ch < 0x80) ? static_cast<char>(ch) : '?';
        }
        return out;
    }

    static geometry_msgs::msg::Point pt(double x, double y, double z)
    {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }

    visualization_msgs::msg::Marker & add(const std::string & ns, int type,
                                          const Color & c)
    {
        arr_.markers.emplace_back();
        auto & m = arr_.markers.back();
        m.header.frame_id = frame_;
        m.header.stamp = stamp_;
        m.ns = ns;
        m.id = id_++;
        m.type = type;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.color.r = c.r;
        m.color.g = c.g;
        m.color.b = c.b;
        m.color.a = c.a;
        m.lifetime = rclcpp::Duration::from_seconds(0.0);   // 永久，不清除
        return m;
    }

    std::string frame_;
    rclcpp::Time stamp_;
    int id_{0};
    visualization_msgs::msg::MarkerArray arr_;
};

}  // namespace viz

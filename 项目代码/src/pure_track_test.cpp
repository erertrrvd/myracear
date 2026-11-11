//这个文件基于纯追踪算法对小车进行运动控制
//订阅my_local_path话题，获取局部路径
//之后进行计算，得到前轮横向转角
//之后再发布出去

//这个代码应该包含完整的运动控制，还应该包含停车的代码（红绿灯点停车的代码），ab停车的代码
//可以加个标志，进行模式切换，分为基于yolo和基于下标的两种红绿灯模式
//红绿灯，ab停车，都是基于下标就可以了


#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp> //订阅局部路径
#include <geometry_msgs/msg/twist.hpp> //发布速度命令
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32.hpp>  //int32的消息，接收红绿灯阈值
#include <vector>
#include "math.h"
#include <iostream>

// ==================== 调参日志用（纯新增，不影响控制逻辑）====================
#include <fstream>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <chrono>
#include <memory>
#include <utility>

// ============ CSV 日志类：记录全部横向控制参数 + 每周期中间量 ============
// 输出到 /home/log/lateral_<时间戳>.csv
// 参数以 # 注释写在文件头，便于"智能车调参工作台"做三源比对
class CsvLogger
{
public:
    CsvLogger(const std::string& file_path,
              const std::vector<std::pair<std::string, double>>& params)
    {
        auto now = std::time(nullptr);
        std::tm* local_tm = std::localtime(&now);

        std::ostringstream oss;
        oss << file_path.substr(0, file_path.find_last_of('.'))
            << "_" << std::put_time(local_tm, "%Y%m%d_%H%M%S") << ".csv";
        actual_filename_ = oss.str();
        file_.open(actual_filename_);

        if (file_.is_open()) {
            file_ << "# node=pure_track_test\n";
            for (const auto& kv : params) {
                file_ << "# " << kv.first << "=" << kv.second << "\n";
            }
            file_ << "timestamp,idx,target_v,current_v,dy,dyaw_deg,kappa0,kappa_ff,"
                     "ff_angle,fb_yaw_angle,fb_dy_angle,angle_raw,angle_cmd,linear_cmd,mode\n";
            file_.flush();
            RCLCPP_INFO(rclcpp::get_logger("csv_logger"),
                        "lateral log -> %s", actual_filename_.c_str());
        } else {
            RCLCPP_WARN(rclcpp::get_logger("csv_logger"),
                        "打不开日志文件: %s", actual_filename_.c_str());
        }
    }

    ~CsvLogger() { if (file_.is_open()) file_.close(); }

    bool ok() const { return file_.is_open(); }

    void writeRow(double timestamp, int idx, double target_v, double current_v,
                  double dy, double dyaw_deg, double kappa0, double kappa_ff,
                  double ff_angle, double fb_yaw_angle, double fb_dy_angle,
                  double angle_raw, double angle_cmd, double linear_cmd,
                  const std::string& mode)
    {
        if (!file_.is_open()) return;
        file_ << std::fixed << std::setprecision(6)
              << timestamp << "," << idx << "," << target_v << "," << current_v << ","
              << dy << "," << dyaw_deg << "," << kappa0 << "," << kappa_ff << ","
              << ff_angle << "," << fb_yaw_angle << "," << fb_dy_angle << ","
              << angle_raw << "," << angle_cmd << "," << linear_cmd << ","
              << mode << "\n";

        static int counter = 0;
        if (++counter % 50 == 0) file_.flush();
    }

private:
    std::ofstream file_;
    std::string actual_filename_;
};




//斯坦利控制器
class Stanely_controller
{
    public:
        /*********相关参数*********/
        float fr_l = 0.325;//轴间距

        //横向误差
        float d_min = 0.5; //最小修正距离
        float k_v =  0.1;  //速度争议

        //偏航角误差
        float kp_yaw = 1;
        float kd_yaw = 0.5;

        float last_yaw = 0.0;

        //纯追踪前馈补偿
        float ff_kp = 1;


        //角度限幅
        float theta_max = 25;
        float theta_min = -25;

        int f_index = 0;
        //对点的定义

        // === 调参日志用：暴露最后一次计算的中间量（纯新增，不参与控制）===
        float dbg_ff_angle     = 0.0f;   // 前馈角度
        float dbg_fb_yaw_angle = 0.0f;   // 航向误差反馈
        float dbg_fb_dy_angle  = 0.0f;   // 横向误差反馈
        float dbg_angle_raw    = 0.0f;   // 限幅前总角度
        float dbg_dy           = 0.0f;   // 横向误差 (m, 车身系)
        float dbg_dyaw         = 0.0f;   // 航向误差 (rad)
        float dbg_kappa0       = 0.0f;   // 当前点曲率
        float dbg_kappa_ff     = 0.0f;   // 预瞄点曲率

        struct Coord
        {
            double x;
            double y;
            double yaw;
            double k;//瞬时曲率
            double v;//速度
            int index;//下标
            
            // 构造函数
            Coord(double x_ = 0.0, double y_ = 0.0, double yaw_ = 0.0, double k_ = 0.0, double v_ = 0.0, int index_ = 0)
                : x(x_), y(y_), yaw(yaw_), k(k_), v(v_), index(index_) {}
        };



        //计算航片角反馈
        float get_yaw_angle(Coord coord)
        {
            float angle = coord.yaw * kp_yaw + kd_yaw * (coord.yaw - last_yaw);
            last_yaw = coord.yaw;
            return angle * 180 /M_PI;
        }


        //计算横向误差的反馈
        float get_dy_angle(Coord coord , float current_v)
        {
            //计算d
            float d = d_min + k_v * current_v;

            //计算angle
            float angle = atan2(coord.y,d) * 180 / M_PI;

            
            return angle;
        }

        //基于纯追踪实现前馈补偿
        float get_ff_angle(Coord coord)
        {
                
                //计算angle
                float angle = atan(coord.k * fr_l) * 180 /M_PI * ff_kp;
                return angle;
        }


        //计算最终角度
        float get_final_angle(std::vector<Coord> local_path, float current_v)
        {
            if(local_path.empty())
            {
                return 0.0;
            }


            //获取前馈角度
            int index = std::min(f_index,(int)(local_path.size()-1));
            float ff_angle = get_ff_angle(local_path[index]);

            //获取反馈角度
            float fb_yaw_angle = get_yaw_angle(local_path[0]);
            float fb_dy_angle = get_dy_angle(local_path[0],current_v);

            //计算最终角度
            float angle = ff_angle + fb_yaw_angle + fb_dy_angle;

            // === 调参日志用：记录中间量（纯新增，不改变任何控制逻辑）===
            dbg_ff_angle     = ff_angle;
            dbg_fb_yaw_angle = fb_yaw_angle;
            dbg_fb_dy_angle  = fb_dy_angle;
            dbg_angle_raw    = angle;
            dbg_dy           = (float)local_path[0].y;
            dbg_dyaw         = (float)local_path[0].yaw;
            dbg_kappa0       = (float)local_path[0].k;
            dbg_kappa_ff     = (float)local_path[index].k;

            //打印调试
            std::cout << "dy:" <<local_path[0].y<<"   d_yaw:"<< local_path[0].yaw*180/M_PI << std::endl;
            std::cout << "ff_angle:" << ff_angle << "   fb_yaw_angle:" << fb_yaw_angle << "   fb_dy_angle:" << fb_dy_angle << std::endl;


            //限制角度
            if(angle > theta_max)
            {
                angle = theta_max;
            }
            if(angle < theta_min)
            {
                angle = theta_min;
            }

            std::cout << "angle:" << angle << std::endl;



            return angle;

        }







};










class PureTracking: public rclcpp::Node
{
    private:
        /**************话题相关*************/
        //发布者
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
        //订阅者
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        //订阅编码器
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr encoder_sub_;
        //订阅红绿灯话题
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr light_sub_;
        //红绿灯识别：1 = 请 light_bridge 把 yolo 识别进程拉起来，0 = 关掉它
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr light_req_pub_;

        

        /*************参数相关*************/
        float current_v = 0.0;


        float linear_x = 1.0;
        int v_mode = 0;  //0是速度规划模式，1是匀速模式


        float d_min = 0.5;
        float k_v =  0.1;  

        float kp_yaw = 1;
        float kd_yaw = 0.5;

        float ff_kp = 1;

        float theta_max = 20;
        float theta_min = -20;



        /***********红绿灯相关参数***********/
        int traffic_light_index = 0;//红绿灯的下标
        int traffic_light_flag = 0;//
        int is_waiting_traffic_light = 0;//是否正在等待红绿灯信号
        int light_timeout_flag = 0;//1 = 超时定时器响过了，该放行了
        rclcpp::TimerBase::SharedPtr traffic_light_timer_; // 红绿灯定时器
        float traffic_light_time = 3;//红绿灯等待时间

        // ===== 第二个红绿灯（停车 / 等待 / 放行 逻辑和第一个完全一样）=====
        // 2 号灯下标 <= 1 号灯下标 时自动视为"未启用"，
        // 否则两个灯会在同一个点各停一次（会多等一整轮）。
        int   traffic_light_index2 = 0;   //2号红绿灯的下标
        float traffic_light_time2  = 3;   //2号红绿灯等待时间（<=0 表示不设超时，一直等）

        //两个灯各自"本圈已经停过"的一次性标记（end_flag 只跑一圈，不需要复位）
        int tl1_fired = 0;
        int tl2_fired = 0;
        int tl_active = -1;               //当前正在等哪个灯：-1 没在等，0 = 1号灯，1 = 2号灯
        int light_mode = 1;//红绿灯等待方案，两个方案的【停车逻辑完全一样】，只有怎么等到绿灯不同：
                           //  1 = 方案二：到点启动 yolo 认灯，认出绿灯就走；
                           //      一直认不出来就等 traffic_light_time 秒，到点当红灯处理直接走
                           //  0 = 方案一：不启用 yolo，只等 traffic_light_time 秒就走
                           //（参数值在 launch/move_test.launch.py 里改）


        //斑马线相关参数
        int first_stop_index = 0;//第一次斑马线停车
        int second_stop_index = 0;//第二次斑马线停车
        int first_stop_flag = 0;//第一圈停车
        int second_stop_flag = 0;//第二圈停车
        int is_waiting = 0;//是否正在等待斑马线停车

        rclcpp::TimerBase::SharedPtr sidewalk_timer_; // 斑马线定时器

        //自动停车
        int current_index = 0;
        int auto_first_stop_index = 0;//第一次停车的下标
        int traffic_light_area = 1500;//停车面积阈值



        //结束
        int end_flag = 0;//是否结束
        float slow_v = 1.0;//减速速度



        int f_index = 0;//前馈下标


        //阈值角度
        float angle_threshold = 8.0;//开始减速的阈值角度
        float kp_v = 0.1;//每多一度，下降速度
        float v_min = 1.5;//最小速度

        Stanely_controller controller;
        //局部路径
        std::vector<Stanely_controller::Coord> local_path;

        // === 调参日志（纯新增，不影响控制）===
        std::unique_ptr<CsvLogger> lateral_log_;
        std::chrono::system_clock::time_point start_time_;


    public:
        PureTracking() : rclcpp::Node("pure_track_test")
        {
            //参数声明
            this->declare_parameter("linear_x", linear_x);
            this->declare_parameter("d_min", d_min);
            this->declare_parameter("k_v", k_v);
            this->declare_parameter("kp_yaw", kp_yaw);
            this->declare_parameter("kd_yaw", kd_yaw);
            this->declare_parameter("ff_kp", ff_kp);
            this->declare_parameter("theta_max", theta_max);
            this->declare_parameter("theta_min", theta_min);

            this->declare_parameter("traffic_light_time", traffic_light_time);
            this->declare_parameter("traffic_light_index", traffic_light_index);
            this->declare_parameter("traffic_light_index2", traffic_light_index2);
            this->declare_parameter("traffic_light_time2", traffic_light_time2);
            this->declare_parameter("light_mode", light_mode);
            this->declare_parameter("first_stop_index", first_stop_index);
            this->declare_parameter("second_stop_index", second_stop_index);
            this->declare_parameter("v_mode", v_mode);
            this->declare_parameter("slow_v", slow_v);
            this->declare_parameter("f_index", f_index);



            //得到参数
            this->get_parameter("linear_x", linear_x);
            this->get_parameter("d_min", d_min);
            this->get_parameter("k_v", k_v);
            this->get_parameter("kp_yaw", kp_yaw);
            this->get_parameter("kd_yaw", kd_yaw);
            this->get_parameter("ff_kp", ff_kp);
            this->get_parameter("theta_max", theta_max);
            this->get_parameter("theta_min", theta_min);

            this->get_parameter("traffic_light_time", traffic_light_time);
            this->get_parameter("traffic_light_index", traffic_light_index);
            this->get_parameter("traffic_light_index2", traffic_light_index2);
            this->get_parameter("traffic_light_time2", traffic_light_time2);
            this->get_parameter("light_mode", light_mode);
            this->get_parameter("first_stop_index", first_stop_index);
            this->get_parameter("second_stop_index", second_stop_index);
            this->get_parameter("v_mode", v_mode);
            this->get_parameter("slow_v", slow_v);
            this->get_parameter("f_index", f_index);



            //参数修改
            controller.d_min = d_min;
            controller.k_v = k_v;
            controller.kp_yaw = kp_yaw;
            controller.kd_yaw = kd_yaw;
            controller.ff_kp = ff_kp;
            controller.theta_max = theta_max;
            controller.theta_min = theta_min;
            controller.f_index = f_index;

            //创建发布者
            cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            //订阅局部路径
            path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
                "my_local_path", 10,
                std::bind(&PureTracking::path_callback, this, std::placeholders::_1));

            encoder_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "encoder", 10,
            std::bind(&PureTracking::encoder_callback, this, std::placeholders::_1));
            
        
            //订阅红绿灯话题
            light_sub_ = this->create_subscription<std_msgs::msg::Int32>(
                "traffic_light",
                10,
                std::bind(&PureTracking::light_callback, this, std::placeholders::_1));

            //请求/释放红绿灯识别进程：1 = 把 yolo 拉起来，0 = 关掉它
            light_req_pub_ = this->create_publisher<std_msgs::msg::Int32>("traffic_light_req", 10);

            // === 调参日志（纯新增）：把全部横向参数写进 CSV 头部 ===
            start_time_ = std::chrono::system_clock::now();
            lateral_log_ = std::make_unique<CsvLogger>(
                "/home/log/lateral",
                std::vector<std::pair<std::string, double>>{
                    {"kp_yaw",              (double)kp_yaw},
                    {"kd_yaw",              (double)kd_yaw},
                    {"d_min",               (double)d_min},
                    {"k_v",                 (double)k_v},
                    {"ff_kp",               (double)ff_kp},
                    {"f_index",             (double)f_index},
                    {"linear_x",            (double)linear_x},
                    {"slow_v",              (double)slow_v},
                    {"v_mode",              (double)v_mode},
                    {"theta_max",           (double)theta_max},
                    {"theta_min",           (double)theta_min},
                    {"first_stop_index",    (double)first_stop_index},
                    {"second_stop_index",   (double)second_stop_index},
                    {"traffic_light_index", (double)traffic_light_index},
                    {"traffic_light_time",  (double)traffic_light_time},
                    {"traffic_light_index2",(double)traffic_light_index2},
                    {"traffic_light_time2", (double)traffic_light_time2},
                    {"light_mode",          (double)light_mode},
                    {"fr_l",                (double)controller.fr_l},
                    {"angle_threshold",     (double)angle_threshold},
                    {"kp_v",                (double)kp_v},
                    {"v_min",               (double)v_min}
                });

            RCLCPP_INFO(this->get_logger(),
                "pure_track_test started | kp_yaw=%.3f kd_yaw=%.3f d_min=%.3f k_v=%.3f ff_kp=%.3f f_index=%d",
                kp_yaw, kd_yaw, d_min, k_v, ff_kp, f_index);  

            //红绿灯自检：上电看一眼就知道两个灯配得对不对
            RCLCPP_INFO(this->get_logger(),
                "[TL1] index=%d wait=%.1fs | [TL2] index=%d wait=%.1fs%s",
                traffic_light_index, traffic_light_time,
                traffic_light_index2, traffic_light_time2,
                (traffic_light_index2 > traffic_light_index)
                    ? "" : "   (TL2 未启用: 2号灯下标必须大于1号灯)");



        }
    private:
        
        //红绿灯放行：超时、认出绿灯、网页按钮，三条路都走这里
        void release_traffic_light(const char* why)
        {
            //必须 cancel 掉定时器：否则绿灯放行后它还在跑，
            //等够时间就置 light_timeout_flag，会把【下一个灯】提前放行
            if(traffic_light_timer_)
            {
                traffic_light_timer_->cancel();
            }

            //顺手让 light_bridge 把 yolo 关掉，别占 CPU（和原来行为一致）
            if(light_mode == 1)
            {
                auto req = std_msgs::msg::Int32();
                req.data = 0;
                light_req_pub_->publish(req);
            }

            is_waiting_traffic_light = 0;
            light_timeout_flag = 0;

            int k = (tl_active >= 0) ? (tl_active + 1) : 0;
            std::cout << "TL" << k << ": go (" << why << ")" << std::endl;
            tl_active = -1;
        }

        //红绿灯回调函数：light_bridge 认出绿灯、或网页按钮，都会发 1
        //  1 号灯 / 2 号灯 共用这一条：谁在等灯就把谁放行
        void light_callback(const std_msgs::msg::Int32::SharedPtr msg)
        {
            //刻意放在 light_mode 检查之前：
            //方案一(light_mode==0)也想能用网页按钮放行，不占 CPU
            if(msg->data == 1 && is_waiting_traffic_light == 1 && is_waiting == 0)
            {
                release_traffic_light("green");
                return;
            }

            // 方案一（light_mode==0）不启用 yolo，也就不认这个话题
            if(light_mode == 0)
            {
                 return;
            }
        }




        //路径回调函数
        void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
        {
            //获取局部路径
            local_path.clear();
            for(auto &point : msg->poses)
            {
                float yaw = quaternionToYaw(
                    0.0,
                    0.0,
                    //point.pose.orientation.x,
                    //point.pose.orientation.y,
                    point.pose.orientation.z,
                    point.pose.orientation.w
                );
                local_path.push_back(Stanely_controller::Coord(point.pose.position.x,point.pose.position.y,
                 yaw, point.pose.position.z,point.pose.orientation.x,(int)point.pose.orientation.y)          );
            }
            
            current_index = local_path[0].index;
            

            //斑马线停车
            if(first_stop_flag == 0)
            {
                //判断是否达到停车点
                if(local_path[0].index >= first_stop_index)
                {
                    //publish_cmd_vel(0.0,0.0);//速度为0，回正
                    
                        //创建3秒定时器
                    sidewalk_timer_ = this->create_wall_timer(
                    std::chrono::seconds(3),
                    std::bind(&PureTracking::sidewalk_timer_callback, this));

                    is_waiting = 1;
                    first_stop_flag = 1;

                    //打印当前坐标
                    std::cout<<"first_stop_index:"<<current_index<<std::endl;


                }
            }
            if(first_stop_flag==1&&second_stop_flag==0)
            {
                //判断是否达到停车点
                if(local_path[0].index >= second_stop_index)
                {
                    //publish_cmd_vel(0.0,0.0);//速度为0，回正
                    
                        //创建3秒定时器
                    sidewalk_timer_ = this->create_wall_timer(
                    std::chrono::seconds(3),
                    std::bind(&PureTracking::sidewalk_timer_callback, this));

                    is_waiting = 1;
                    second_stop_flag = 1;

                    std::cout<<"second_stop_index:"<<current_index<<std::endl;
                }
            }
        


            // ============================================================
            //  红绿灯：两个停车点，各自只停一次，
            //  停车 / 超时 / 视觉请求 / 放行 的处理和原来一个灯时完全一样
            // ============================================================
            {
                const int   tl_idx  [2] = { traffic_light_index,
                                            (traffic_light_index2 > traffic_light_index)
                                                ? traffic_light_index2 : -1 };
                const float tl_wait [2] = { traffic_light_time, traffic_light_time2 };
                      int*  tl_fired[2] = { &tl1_fired, &tl2_fired };

                //同一个时刻只允许等一个灯
                if(tl_active < 0)
                {
                    for(int k = 0; k < 2; k++)
                    {
                        if(tl_idx[k] < 0)                    continue;  //未启用
                        if(*(tl_fired[k]))                   continue;  //本圈已停过
                        if(local_path[0].index < tl_idx[k])  continue;  //还没到

                        *(tl_fired[k]) = 1;
                        tl_active = k;
                        is_waiting_traffic_light = 1;
                        light_timeout_flag = 0;

                        //超时定时器用【这个灯自己】的等待时间
                        //   <= 0 表示不设超时：一直等视觉/按钮放行
                        if(tl_wait[k] > 0.0f)
                        {
                            traffic_light_timer_ = this->create_wall_timer(
                                std::chrono::milliseconds(static_cast<int>(tl_wait[k] * 1000)),
                                std::bind(&PureTracking::traffic_light_timer_callback, this));
                        }

                        //方案二才要开 yolo；方案一纯等时间，不用识别
                        if(light_mode == 1)
                        {
                            auto req = std_msgs::msg::Int32();
                            req.data = 1;
                            light_req_pub_->publish(req);
                            std::cout << "TL" << (k+1) << ": stop at index "
                                      << current_index << ", start vision" << std::endl;
                        }
                        else
                        {
                            std::cout << "TL" << (k+1) << ": stop at index "
                                      << current_index << ", fixed wait "
                                      << tl_wait[k] << "s" << std::endl;
                        }
                        std::cout << "traffic_light_" << (k+1) << "_index:"
                                  << current_index << std::endl;
                        break;
                    }
                }
            }

            //终点停车必须"闩锁 + 每个周期重发"，不能只发一帧：
            //万一一帧丢了（UDP 丢包 / transform_v 那一刻正在重启），
            //这个回调就再也不发 cmd_vel，车会保持最后的目标速度一直往前跑。
            if(end_flag == 1)
            {
                publish_cmd_vel(0.0, 0.0);   //油门 0 + 方向回正
                return;
            }

            if(end_flag == 0)
            {
                // 正常计算角度
                float angle = controller.get_final_angle(local_path, current_v);
            
                // 斑马线等待：速度给0，角度正常下发
                if(is_waiting == 1)
                {
                    publish_cmd_vel(0.0, angle);
                    return;
                }

                // 红绿灯等待
                if(is_waiting_traffic_light == 1)
                {
                    // 只有两种情况能走：
                    //   ① 方案二认出绿灯 -> light_callback 把 is_waiting_traffic_light 清成 0
                    //   ② 超时定时器响了 -> light_timeout_flag 置 1，这里放行
                    // 红灯、没灯、识别挂了(stale)，一律继续停着等
                    if(light_timeout_flag == 1)
                    {
                        release_traffic_light("timeout");
                    }
                    else
                    {
                        publish_cmd_vel(0.0, angle);
                        return;
                    }
                }
            
                // 正常行驶逻辑
                if(v_mode == 0)
                {
                    if(local_path[0].v >= 0)
                    {
                        publish_cmd_vel(local_path[0].v, angle);
                    }
                    else if(local_path[0].v == -1)
                    {
                        std::cout << "is slow\n";
                        angle = controller.get_dy_angle(local_path[0], current_v) + controller.get_yaw_angle(local_path[0]);
                        publish_cmd_vel(slow_v, angle);
                    }
                    else if(local_path[0].v == -2)
                    {
                        std::cout << "is stop\n";
                        publish_cmd_vel(0, 0);
                        end_flag = 1;
                    }
                }
                else
                {
                    if(local_path[0].v >= 0)
                    {
                        publish_cmd_vel(linear_x, angle);
                    }
                    else if(local_path[0].v == -1)
                    {
                        std::cout << "is slow\n";
                        angle = controller.get_dy_angle(local_path[0], current_v) + controller.get_yaw_angle(local_path[0]);
                        publish_cmd_vel(slow_v, angle);
                    }
                    else if(local_path[0].v == -2)
                    {
                        std::cout << "is stop\n";
                        publish_cmd_vel(0, 0);
                        end_flag = 1;
                    }
                }
            }
        

            

            


        }

        void sidewalk_timer_callback()
        {
            // 关闭定时器
            sidewalk_timer_->cancel();
            is_waiting = 0;

        }

        //超时定时器：到点还没等到绿灯就当红灯处理。两个方案都走这里
        void traffic_light_timer_callback()
        {
            // 关闭定时器
            if(traffic_light_timer_)
            {
                traffic_light_timer_->cancel();
            }

            if(light_mode == 1)
            {
                //方案二：先把 yolo 识别关掉，别占 CPU
                auto req = std_msgs::msg::Int32();
                req.data = 0;
                light_req_pub_->publish(req);
                std::cout<<"traffic light timeout: vision off"<<std::endl;
            }

            //置上放行标志，path_callback 下一周期就会放行
            light_timeout_flag = 1;
        }



        float quaternionToYaw(float x, float y, float z, float w)
        {
            // 使用公式：yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
            float siny_cosp = 2.0 * (w * z + x * y);
            float cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
            return std::atan2(siny_cosp, cosy_cosp);
        }


        void publish_cmd_vel(double linear_x, double angular_z)
        {
            auto cmd_msg = geometry_msgs::msg::Twist();
            cmd_msg.linear.x = linear_x;
            cmd_msg.angular.z = angular_z;
            cmd_pub_->publish(cmd_msg);

            // === 调参日志（纯新增）：在这里记录，可以覆盖所有分支（含提前 return 的等待分支）===
            logLateral(linear_x, angular_z);
        }

        // === 调参日志（纯新增）：每下发一次指令记一帧 ===
        // 记的不是"结果"而是"分解"：前馈 / 航向反馈 / 横向反馈三项分开，
        // 这样调参时能直接看出是哪一项在起作用、哪一项在打架。
        void logLateral(double linear_cmd, double angle_cmd)
        {
            if (!lateral_log_ || !lateral_log_->ok()) return;

            auto now = std::chrono::system_clock::now();
            double ts = std::chrono::duration<double>(now - start_time_).count();

            const char* mode = (is_waiting || is_waiting_traffic_light) ? "WAIT"
                             : (end_flag ? "END" : "RUN");

            double dy = controller.dbg_dy;
            double dyaw_deg = controller.dbg_dyaw * 180.0 / M_PI;
            double k0 = controller.dbg_kappa0;
            double tv = 0.0;
            if (!local_path.empty()) {
                dy       = local_path[0].y;
                dyaw_deg = local_path[0].yaw * 180.0 / M_PI;
                k0       = local_path[0].k;
                tv       = local_path[0].v;
            }

            lateral_log_->writeRow(
                ts, current_index, tv, current_v,
                dy, dyaw_deg, k0, controller.dbg_kappa_ff,
                controller.dbg_ff_angle, controller.dbg_fb_yaw_angle,
                controller.dbg_fb_dy_angle,
                controller.dbg_angle_raw, angle_cmd, linear_cmd,
                std::string(mode));
        }

        //编码器回调函数
        void encoder_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            //获取编码器数据
            current_v = msg->twist.twist.linear.x;
        }

        //根据角度输出速度
        float get_linear_x(float angle)
        {
            float v = linear_x;

            if(fabs(angle) <= angle_threshold)
            {
                return v;
            }
            else
            {
                v -= kp_v*fabs(fabs(angle)-angle_threshold);
                if(v < v_min)
                {
                    v = v_min;
                }
            }

            return v;

        }




};





int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);  
    auto node = std::make_shared<PureTracking>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}







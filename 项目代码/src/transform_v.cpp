#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <math.h>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <fstream>
#include <string>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <memory>

// ============ CSV 日志类（带时间戳文件名，记录 PID 参数）============
class CsvLogger {
public:
    // 构造函数：传入所有 PID 参数，生成带时间戳的文件
    CsvLogger(const std::string& file_path,
              float kp, float ki, float kd,
              float integral_min, float integral_max,
              float output_min, float output_max)
    {
        // 1. 生成实际文件名：路径 + 时间戳 + .csv
        auto now = std::time(nullptr);
        std::tm* local_tm = std::localtime(&now);

        std::ostringstream oss;
        oss << file_path.substr(0, file_path.find_last_of('.'))
            << "_" << std::put_time(local_tm, "%Y%m%d_%H%M%S") << ".csv";

        actual_filename_ = oss.str();

        // 2. 打开文件
        file_.open(actual_filename_);

        if (file_.is_open()) {
            // 3. 写入 PID 参数信息
            file_ << "# PID parameters\n";
            file_ << "# kp=" << kp << "\n";
            file_ << "# ki=" << ki << "\n";
            file_ << "# kd=" << kd << "\n";
            file_ << "# integral_limit_min=" << integral_min << "\n";
            file_ << "# integral_limit_max=" << integral_max << "\n";
            file_ << "# output_limit_min=" << output_min << "\n";
            file_ << "# output_limit_max=" << output_max << "\n";

            // 4. 写入数据表头
            file_ << "timestamp,target_v,current_v,linear_cmd,error\n";

            file_.flush();
        }
    }

    ~CsvLogger() {
        if (file_.is_open()) {
            file_.close();
        }
    }

    // 写入一行数据：时间戳, 目标速度, 实际速度, 纵向指令, 误差
    void writeRow(double timestamp, float target_v, float current_v,
                  float linear_cmd, float error) {
        if (file_.is_open()) {
            file_ << std::fixed << std::setprecision(6)
                  << timestamp << ","
                  << target_v << ","
                  << current_v << ","
                  << linear_cmd << ","
                  << error << "\n";

            // 每 50 行刷新一次
            static int counter = 0;
            if (++counter % 50 == 0) {
                file_.flush();
            }
        }
    }

private:
    std::ofstream file_;
    std::string actual_filename_;
};

// ============ PID 控制器 ============
class PID {
public:
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    float integral_limit_min = 0.0f;
    float integral_limit_max = 0.0f;

    float output_limit_min = 0.0f;
    float output_limit_max = 0.0f;

    float last_error = 0.0f;
    float integral = 0.0f;

    float pid(float error) {
        integral += error;

        if (integral > integral_limit_max) {
            integral = integral_limit_max;
        }

        if (integral < integral_limit_min) {
            integral = integral_limit_min;
        }

        float control = kp * error + ki * integral + kd * (error - last_error) + 1500.0f;

        last_error = error;

        if (control > output_limit_max) {
            control = output_limit_max;
        }

        if (control < output_limit_min) {
            control = output_limit_min;
        }

        return control;
    }
};

// ============ 主节点 ============
class Transform_v : public rclcpp::Node {
public:
    Transform_v()
    : Node("transform_v")
    {
        // 声明参数
        this->declare_parameter("pid_kp", 40.0f);
        this->declare_parameter("pid_ki", 0.8f);
        this->declare_parameter("pid_kd", 40.0f);
        this->declare_parameter("pid_integral_limit_min", -120.0f);
        this->declare_parameter("pid_integral_limit_max", 120.0f);
        this->declare_parameter("pid_output_limit_min", 1300.0f);
        this->declare_parameter("pid_output_limit_max", 1625.0f);

        // ===== 安全闸门参数 =====
        this->declare_parameter("require_link", true);    // 默认开：忘加参数 = 车不动（明显失败），
                                                          // 而不是"没保护"（看不见的失败）
        this->declare_parameter("link_timeout", 0.30);    // PC 心跳超时（秒）
        this->declare_parameter("require_cmd",  true);    // true = cmd_vel 断流就停车
        this->declare_parameter("cmd_timeout",  0.25);    // cmd_vel 超时（秒）

        // 获取参数
        float kp = this->get_parameter("pid_kp").as_double();
        float ki = this->get_parameter("pid_ki").as_double();
        float kd = this->get_parameter("pid_kd").as_double();

        float integral_min = this->get_parameter("pid_integral_limit_min").as_double();
        float integral_max = this->get_parameter("pid_integral_limit_max").as_double();

        float output_min = this->get_parameter("pid_output_limit_min").as_double();
        float output_max = this->get_parameter("pid_output_limit_max").as_double();

        require_link_ = this->get_parameter("require_link").as_bool();
        link_timeout_ = this->get_parameter("link_timeout").as_double();
        require_cmd_  = this->get_parameter("require_cmd").as_bool();
        cmd_timeout_  = this->get_parameter("cmd_timeout").as_double();

        // 配置 PID
        pid_.kp = kp;
        pid_.ki = ki;
        pid_.kd = kd;

        pid_.integral_limit_min = integral_min;
        pid_.integral_limit_max = integral_max;

        pid_.output_limit_min = output_min;
        pid_.output_limit_max = output_max;

        // CSV 日志：记录所有 PID 参数
        csv_logger_ = std::make_unique<CsvLogger>(
            "/home/log/pid",
            kp, ki, kd,
            integral_min, integral_max,
            output_min, output_max);

        // 订阅编码器
        encoder_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "encoder", 10,
            std::bind(&Transform_v::encoder_callback, this, std::placeholders::_1));

        // 订阅目标速度
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&Transform_v::cmd_callback, this, std::placeholders::_1));

        // PC 心跳：link_guard 只在'电脑还在发 UDP'时才发这个话题
        link_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "pc_heartbeat", 10,
            [this](std_msgs::msg::Empty::SharedPtr) {
                last_link_tp_ = std::chrono::steady_clock::now();
                have_link_ = true;
            });

        // 发布控制指令
        car_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("car_cmd_vel", 10);

        // 20ms 控制定时器
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&Transform_v::control_timer_callback, this));

        // 记录开始时间
        start_time_ = std::chrono::system_clock::now();

        // 键盘监控线程
        keyboard_thread_ = std::thread(&Transform_v::keyboard_monitor, this);

        RCLCPP_INFO(this->get_logger(), "transform_v node started");
        RCLCPP_INFO(this->get_logger(), "PID parameters:");
        RCLCPP_INFO(this->get_logger(), "kp=%.3f, ki=%.3f, kd=%.3f", kp, ki, kd);
        RCLCPP_INFO(this->get_logger(), "integral_min=%.3f, integral_max=%.3f", integral_min, integral_max);
        RCLCPP_INFO(this->get_logger(), "output_min=%.3f, output_max=%.3f", output_min, output_max);
        RCLCPP_INFO(this->get_logger(), "Press 's' to STOP, 'r' to RESUME");

        RCLCPP_INFO(this->get_logger(),
            "安全闸门: PC心跳=%s(%.2fs)  cmd_vel=%s(%.2fs)",
            require_link_ ? "开" : "关", link_timeout_,
            require_cmd_  ? "开" : "关", cmd_timeout_);
        if (require_link_) {
            RCLCPP_INFO(this->get_logger(),
                "require_link=开：没收到 PC 心跳前车不会动。"
                "现场可用: ros2 param set /transform_v require_link false 临时关掉");
        }

        // 允许运行时用 ros2 param set 直接开关闸门（不用重启节点）
        param_cb_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & params) {
                for (const auto & p : params) {
                    if (p.get_name() == "require_link")  require_link_ = p.as_bool();
                    else if (p.get_name() == "require_cmd") require_cmd_ = p.as_bool();
                    else if (p.get_name() == "link_timeout") link_timeout_ = p.as_double();
                    else if (p.get_name() == "cmd_timeout")  cmd_timeout_  = p.as_double();
                }
                RCLCPP_WARN(this->get_logger(),
                    "闸门已更新: PC心跳=%s(%.2fs)  cmd_vel=%s(%.2fs)",
                    require_link_ ? "开" : "关", link_timeout_,
                    require_cmd_  ? "开" : "关", cmd_timeout_);
                rcl_interfaces::msg::SetParametersResult r;
                r.successful = true;
                return r;
            });
    }

    ~Transform_v() {
        keyboard_running_.store(false);

        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }

        RCLCPP_INFO(this->get_logger(), "Node shutdown complete");
    }

private:
    // ===== 非阻塞键盘读取 =====
    int getch() {
        struct termios oldt, newt;
        int ch;

        tcgetattr(STDIN_FILENO, &oldt);

        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);

        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        int oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

        ch = getchar();

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        fcntl(STDIN_FILENO, F_SETFL, oldf);

        return ch;
    }

    // ===== 键盘监控线程 =====
    void keyboard_monitor() {
        while (keyboard_running_.load() && rclcpp::ok()) {
            int key = getch();

            if (key != EOF) {
                if (key == 's' || key == 'S') {
                    running_mode_.store(false);
                    RCLCPP_INFO(this->get_logger(), "Switched to STOP mode, key 's'");
                } else if (key == 'r' || key == 'R') {
                    running_mode_.store(true);
                    RCLCPP_INFO(this->get_logger(), "Switched to RUN mode, key 'r'");
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    // ===== 编码器回调 =====
    void encoder_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_v_ = msg->twist.twist.linear.x;

        if (current_v_ < 0.0f) {
            current_v_ = 0.0f;
        }
    }

    // ===== 目标速度回调 =====
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        target_v_ = msg->linear.x;
        target_angular_z_ = msg->angular.z;
        last_cmd_tp_ = std::chrono::steady_clock::now();
        have_cmd_    = true;
    }

    // ===== 控制定时器回调 =====
    void control_timer_callback() {
        auto cmd_msg = geometry_msgs::msg::Twist();

        // ====================================================================
        //  安全闸门（放在最前面：任何一道断了都立刻刹车，不看 target_v_）
        //    ① PC 心跳断了   -> 你的电脑/链路出问题了
        //    ② cmd_vel 断了  -> 车上的控制节点出问题了
        // ====================================================================
        const auto now_tp = std::chrono::steady_clock::now();
        const auto age_of = [&now_tp](bool have, const auto & tp) -> double {
            return have ? std::chrono::duration<double>(now_tp - tp).count() : 1e9;
        };

        if (require_link_) {
            const double link_age = age_of(have_link_, last_link_tp_);
            if (link_age > link_timeout_) {
                if (link_was_ok_) {
                    if (have_link_) {
                        RCLCPP_ERROR(this->get_logger(),
                            "!! PC 心跳丢失 %.0f ms (>%.0f ms) -> 刹车 !!",
                            link_age * 1000.0, link_timeout_ * 1000.0);
                    } else {
                        RCLCPP_INFO(this->get_logger(),
                            "require_link=开，还没收到 PC 心跳 -> 车不会动",
                            "（先在电脑上起 pc_heartbeat.py）");
                    }
                    link_was_ok_ = false;
                }
                pid_.integral = 0.0f;  pid_.last_error = 0.0f;
                cmd_msg.linear.x  = stop_linear_x;    // 1000 刹车
                cmd_msg.angular.z = stop_angular_z;   // 90  回正
                car_cmd_pub_->publish(cmd_msg);
                return;
            } else if (!link_was_ok_) {
                RCLCPP_WARN(this->get_logger(), "PC 心跳恢复 -> 继续");
                link_was_ok_ = true;
            }
        }

        if (require_cmd_) {
            const double cmd_age = age_of(have_cmd_, last_cmd_tp_);
            if (cmd_age > cmd_timeout_) {
                if (cmd_was_ok_) {
                    if (have_cmd_) {
                        RCLCPP_ERROR(this->get_logger(),
                            "!! cmd_vel 断流 %.0f ms (>%.0f ms) -> 刹车 !!",
                            cmd_age * 1000.0, cmd_timeout_ * 1000.0);
                    } else {
                        RCLCPP_INFO(this->get_logger(),
                            "还没收到过 cmd_vel -> 保持刹车（等 pure_track_test 发车）");
                    }
                    cmd_was_ok_ = false;
                }
                pid_.integral = 0.0f;  pid_.last_error = 0.0f;
                cmd_msg.linear.x  = stop_linear_x;
                cmd_msg.angular.z = stop_angular_z;
                car_cmd_pub_->publish(cmd_msg);
                return;
            } else if (!cmd_was_ok_) {
                RCLCPP_WARN(this->get_logger(), "cmd_vel 恢复 -> 继续");
                cmd_was_ok_ = true;
            }
        }

        // 计算速度误差
        float error = target_v_ - current_v_;

        if (running_mode_.load()) {
            // PID 控制
            float control_output = pid_.pid(error);

            // 小于 1500 时做刹车映射
            if (control_output < 1500.0f) {
                control_output = 1500.0f -
                    (1500.0f - control_output) /
                    (pid_.output_limit_max - 1500.0f) *
                    (1500.0f - 1000.0f);
            }

            cmd_msg.linear.x = control_output;

            // 转向角限幅
            float angular_cmd = target_angular_z_;

            if (angular_cmd > 25.0f) {
                angular_cmd = 25.0f;
            }

            if (angular_cmd < -25.0f) {
                angular_cmd = -25.0f;
            }

            // 转向角映射
            cmd_msg.angular.z = angular_cmd * 2.0f + 90.0f;

            // 目标速度为 0 时急停
            if (target_v_ == 0.0f) {
                pid_.integral = 0.0f;
                pid_.last_error = 0.0f;

                cmd_msg.linear.x = stop_linear_x;
                //cmd_msg.angular.z = stop_angular_z;
            }
        } else {
            // STOP 模式
            pid_.integral = 0.0f;
            pid_.last_error = 0.0f;

            cmd_msg.linear.x = stop_linear_x;
            cmd_msg.angular.z = stop_angular_z;
            
        }

        // 发布控制指令
        car_cmd_pub_->publish(cmd_msg);

        // 控制台打印，500ms 打印一次，避免刷屏
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            500,
            "target_v: %.2f, current_v: %.2f, linear_cmd: %.2f, angular_cmd: %.2f, error: %.2f, mode: %s",
            target_v_,
            current_v_,
            cmd_msg.linear.x,
            cmd_msg.angular.z,
            error,
            running_mode_.load() ? "RUN" : "STOP");

        // 记录时间戳
        auto now = std::chrono::system_clock::now();
        double timestamp = std::chrono::duration<double>(now - start_time_).count();

        // 写入 CSV
        csv_logger_->writeRow(timestamp, target_v_, current_v_, cmd_msg.linear.x, error);
    }

private:
    // ===== ROS 通信对象 =====
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr encoder_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr car_cmd_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // ===== PID =====
    PID pid_;

    // ===== CSV 日志 =====
    std::unique_ptr<CsvLogger> csv_logger_;

    // ===== 状态变量 =====
    float current_v_ = 0.0f;
    float target_v_ = 0.0f;
    float target_angular_z_ = 0.0f;

    // ===== 停车指令 =====
    const float stop_linear_x = 1000.0f;
    const float stop_angular_z = 90.0f;

    // ===== 安全闸门 =====
    bool   require_link_ = true;    // true = 必须有 PC 心跳才允许动
    double link_timeout_ = 0.30;
    bool   have_link_    = false;
    bool   link_was_ok_  = true;    // 只为边沿打日志
    std::chrono::steady_clock::time_point last_link_tp_;

    bool   require_cmd_  = true;
    double cmd_timeout_  = 0.25;
    bool   have_cmd_     = false;
    bool   cmd_was_ok_   = true;
    std::chrono::steady_clock::time_point last_cmd_tp_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr link_sub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    // ===== 模式控制 =====
    std::atomic<bool> running_mode_{true};
    std::atomic<bool> keyboard_running_{true};

    // ===== 时间和线程 =====
    std::chrono::system_clock::time_point start_time_;
    std::thread keyboard_thread_;
};

// ============ 主函数 ============
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Transform_v>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
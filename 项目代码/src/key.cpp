#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <atomic>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// 原子变量控制打印状态
std::atomic<bool> print_sss(false);
std::atomic<bool> print_rrr(false);
//检测
// 非阻塞获取键盘按键（兼容Linux终端）
int getch() {
    struct termios oldt, newt;
    int ch;
    // 保存终端原有设置
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    // 关闭规范模式（无需回车）和回显
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    // 设置非阻塞模式
    int oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    // 恢复终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    return ch;
}

// 打印任务类（作为节点的成员）
class KeyNode : public rclcpp::Node {
public:
    KeyNode() : Node("key") {
        // 打印初始化信息
        RCLCPP_INFO(this->get_logger(), "键盘监控节点启动！");
        RCLCPP_INFO(this->get_logger(), "按 s 键：持续打印 sss");
        RCLCPP_INFO(this->get_logger(), "按 r 键：持续打印 rrr");
        RCLCPP_INFO(this->get_logger(), "按其他键：停止打印");
        RCLCPP_INFO(this->get_logger(), "按 q 键：退出节点");

        // 创建打印线程（脱离主线程避免阻塞spin）
        print_thread_ = std::thread(&KeyNode::print_loop, this);
        // 创建键盘监控线程
        keyboard_thread_ = std::thread(&KeyNode::keyboard_monitor, this);
    }

    ~KeyNode() {
        // 停止所有线程
        running_.store(false);
        if (print_thread_.joinable()) {
            print_thread_.join();
        }
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
        RCLCPP_INFO(this->get_logger(), "键盘监控节点已退出");
    }

private:
    // 打印循环函数
    void print_loop() {
        while (running_.load() && rclcpp::ok()) {
            if (print_sss.load()) {
                RCLCPP_INFO(this->get_logger(), "sss");
            } else if (print_rrr.load()) {
                RCLCPP_INFO(this->get_logger(), "rrr");
            }
            // 控制打印频率（100ms/次）
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // 键盘监控函数
    void keyboard_monitor() {
        while (running_.load() && rclcpp::ok()) {
            int key = getch();
            if (key != EOF) {
                switch (key) {
                    case 's':
                        print_sss.store(true);
                        print_rrr.store(false);
                        break;
                    case 'r':
                        print_rrr.store(true);
                        print_sss.store(false);
                        break;
                    case 'q':
                        running_.store(false);
                        print_sss.store(false);
                        print_rrr.store(false);
                        rclcpp::shutdown(); // 关闭ROS 2上下文
                        break;
                    default:
                        // 其他键停止打印
                        print_sss.store(false);
                        print_rrr.store(false);
                        break;
                }
            }
            // 降低CPU占用
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    std::thread print_thread_;          // 打印线程
    std::thread keyboard_thread_;       // 键盘监控线程
    std::atomic<bool> running_{true};   // 线程运行标志
};

int main(int argc, char * argv[]) {
    // 初始化ROS 2
    rclcpp::init(argc, argv);
    // 创建节点
    auto node = std::make_shared<KeyNode>();
    // 保持节点运行（主线程spin）
    rclcpp::spin(node);
    // 清理资源
    rclcpp::shutdown();
    return 0;
}
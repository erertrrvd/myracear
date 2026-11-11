//这个节点首先从csv文件下提取全局路径，保存到vector容器中
//然后订阅odom话题，获取小车当前位置，找到对应全局路径的那个下标
//从那个下标开始选取一系列点，作为局部路径，根据odom进行坐标转换
//发布局部路径轨迹，发布到laser_link话题下

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"  //订阅里程计话题
#include <nav_msgs/msg/path.hpp>   //用于发布局部路径
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"


#include <limits>
#include <fstream>
#include <string>
#include <memory>
#include <vector>
#include <sstream>
#include <iostream>
#include "math.h"
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <ctime>
#include <filesystem>
#include <Eigen/Dense>
constexpr double ICP_PI = 3.14159265358979323846;


//这个类完成了从文件中提取路径点并保存在容器中，提取路径
class CSV_get_Path
{   
    public:
        // 对点数据类型进行定义的结构体
        struct Coord
        {
            double x;
            double y;
            double yaw;
            double k;
            double v;
            int index;
            
            // 构造函数
            Coord(double x_ = 0.0, double y_ = 0.0, double yaw_ = 0.0, double k_ = 0, double v_ = 0.0, int index_ = 0)
                : x(x_), y(y_), yaw(yaw_), k(k_), v(v_), index(index_) {}
        };

    private:
        std::vector<Coord> path_;  // 存储路径点
        std::string csv_path_;     // CSV文件路径

    public:
        // 构造函数 - 初始化时加载CSV文件
        CSV_get_Path(const std::string& csv_path) : csv_path_(csv_path)
        {
            load_from_csv();
        }

        // 返回路径向量
        std::vector<Coord> get_path() const
        {
            return path_;
        }

    private:
        // 从CSV文件加载路径
        void load_from_csv()
        {
            std::ifstream file(csv_path_);
            if (!file.is_open())
            {
                std::cerr << "无法打开CSV文件: " << csv_path_ << std::endl;
                return;
            }
            
            std::string line;
            bool is_first_line = true;
            
            while (std::getline(file, line))
            {
                // 跳过表头
                if (is_first_line && line.find("x,y,yaw,type") != std::string::npos)
                {
                    is_first_line = false;
                    continue;
                }
                
                // 跳过空行
                if (line.empty())
                {
                    continue;
                }
                
                // 使用字符串流解析CSV
                std::stringstream ss(line);
                std::string token;
                std::vector<std::string> tokens;
                
                while (std::getline(ss, token, ','))
                {
                    tokens.push_back(token);
                }
                
                // 确保有足够的字段
                if (tokens.size() >= 4)
                {
                    try
                    {
                        Coord coord;
                        coord.x = std::stod(tokens[0]);
                        coord.y = std::stod(tokens[1]);
                        coord.yaw = std::stod(tokens[2]);
                        coord.v = std::stod(tokens[3]);//第四个位置是纵向速度
                        
                        path_.push_back(coord);
                    }
                    catch (const std::exception& e)
                    {
                        std::cerr << "解析CSV行时出错: " << e.what() << std::endl;
                    }
                }
            }
            
            file.close();
            std::cout << "成功加载 " << path_.size() << " 个路径点" << std::endl;
        }
};



class PathCSVLogger
{
private:
    std::string directory_path_;     // 保存目录
    std::string base_name_;          // 基础文件名
    std::string full_file_path_;     // 最终完整文件路径
    std::ofstream file_;

private:
    // 生成时间戳字符串：YYYYMMDD_HHMMSS
    std::string generate_timestamp()
    {
        auto now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);

        std::tm tm_buf;
#ifdef _WIN32
        localtime_s(&tm_buf, &now_c);
#else
        localtime_r(&now_c, &tm_buf);
#endif

        std::ostringstream oss;
        oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
        return oss.str();
    }

    // 确保目录最后有 /
    std::string normalize_directory(const std::string& dir)
    {
        if (dir.empty()) return "./";
        if (dir.back() == '/' || dir.back() == '\\') return dir;
        return dir + "/";
    }

public:
    // 构造函数：输入保存路径 + 保存名字，自动加时间戳并创建csv文件
    PathCSVLogger(const std::string& save_dir, const std::string& save_name)
    {
        directory_path_ = normalize_directory(save_dir);
        base_name_ = save_name;

        // 创建目录
        std::filesystem::create_directories(directory_path_);

        // 生成最终文件名
        std::string timestamp = generate_timestamp();
        full_file_path_ = directory_path_ + base_name_ + "_" + timestamp + ".csv";

        // 打开文件
        file_.open(full_file_path_, std::ios::out);
        if (!file_.is_open())
        {
            std::cerr << "无法创建CSV文件: " << full_file_path_ << std::endl;
            return;
        }

        // 写表头
        file_ << "px,py,pyaw,t,index,cx,cy,cyaw\n";
        file_.flush();

        std::cout << "CSV文件创建成功: " << full_file_path_ << std::endl;
    }

    // 保存目标轨迹：输入目标轨迹vector，自动保存前三列(px,py,pyaw)
    void save_target_path(const std::vector<CSV_get_Path::Coord>& target_path)
    {
        if (!file_.is_open())
        {
            std::cerr << "CSV文件未打开，无法保存目标轨迹" << std::endl;
            return;
        }

        for (const auto& point : target_path)
        {
            file_ << point.x << ","
                  << point.y << ","
                  << point.yaw << ",,,,,\n";
        }
        file_.flush();
    }

    // 保存当前路径信息：输入 x,y,yaw,t,index，逐行追加
    void save_current_point(double x, double y, double yaw, double t, int index)
    {
        if (!file_.is_open())
        {
            std::cerr << "CSV文件未打开，无法保存当前点" << std::endl;
            return;
        }

        file_ << ",,,"
              << t << ","
              << index << ","
              << x << ","
              << y << ","
              << yaw << "\n";
        file_.flush();
    }

    ~PathCSVLogger()
    {
        if (file_.is_open())
        {
            file_.close();
        }
    }
};


//曲率计算和处理相关的类，自动计算每个点的曲率以及自动进行算法平滑
class Curvature
{
public:
    // 预定义的Savitzky-Golay滤波器系数
    // 格式：{窗口大小, 多项式阶数} -> 系数数组
    struct SG_Coeffs {
        int window_size;
        int poly_order;
        std::vector<float> coefficients;
    };
    
    std::vector<SG_Coeffs> sg_coeffs_cache;  // 缓存系数，避免重复计算

private:
    // 预计算Savitzky-Golay系数
    std::vector<float> computeSG_Coefficients(int window_size, int poly_order) {
        // 检查缓存中是否已有
        for (const auto& coeff : sg_coeffs_cache) {
            if (coeff.window_size == window_size && coeff.poly_order == poly_order) {
                return coeff.coefficients;
            }
        }
        
        // 窗口大小必须是奇数
        if (window_size % 2 == 0) window_size++;
        int half = window_size / 2;
        
        // 构建范德蒙矩阵A
        std::vector<std::vector<float>> A(window_size, std::vector<float>(poly_order + 1));
        for (int i = -half; i <= half; ++i) {
            int row = i + half;
            float power = 1.0f;
            for (int j = 0; j <= poly_order; ++j) {
                A[row][j] = power;
                power *= i;
            }
        }
        
        // 使用最小二乘法计算中心点系数
        std::vector<float> coeffs(window_size, 0.0f);
        std::vector<float> b(window_size, 0.0f);
        b[half] = 1.0f;  // 中心点为1的单位脉冲
        
        // 简化计算：使用预定义系数表（实际中最常用）
        if (window_size == 5 && poly_order == 2) {
            coeffs = {-3.0f/35.0f, 12.0f/35.0f, 17.0f/35.0f, 12.0f/35.0f, -3.0f/35.0f};
        } else if (window_size == 7 && poly_order == 2) {
            coeffs = {-2.0f/21.0f, 3.0f/21.0f, 6.0f/21.0f, 7.0f/21.0f, 6.0f/21.0f, 3.0f/21.0f, -2.0f/21.0f};
        } else if (window_size == 9 && poly_order == 2) {
            coeffs = {-21.0f/231.0f, 14.0f/231.0f, 39.0f/231.0f, 54.0f/231.0f, 59.0f/231.0f, 
                      54.0f/231.0f, 39.0f/231.0f, 14.0f/231.0f, -21.0f/231.0f};
        } else if (window_size == 5 && poly_order == 3) {
            coeffs = {-1.0f/5.0f, 2.0f/5.0f, 2.0f/5.0f, 2.0f/5.0f, -1.0f/5.0f};
        } else {
            // 默认使用窗口7，阶数2
            coeffs = {-2.0f/21.0f, 3.0f/21.0f, 6.0f/21.0f, 7.0f/21.0f, 6.0f/21.0f, 3.0f/21.0f, -2.0f/21.0f};
        }
        
        // 缓存结果
        sg_coeffs_cache.push_back({window_size, poly_order, coeffs});
        return coeffs;
    }

    // 边界处理：镜像扩展
    std::vector<float> mirror_boundary(const std::vector<float>& data, int half_window) {
        std::vector<float> extended;
        
        // 左边界镜像
        for (int i = half_window; i > 0; --i) {
            if (i < (int)data.size()) {
                extended.push_back(data[i]);
            } else {
                extended.push_back(data[0]);
            }
        }
        
        // 中间数据
        extended.insert(extended.end(), data.begin(), data.end());
        
        // 右边界镜像
        for (int i = data.size() - 2; i >= (int)data.size() - half_window - 1; --i) {
            if (i >= 0) {
                extended.push_back(data[i]);
            } else {
                extended.push_back(data.back());
            }
        }
        
        return extended;
    }

public:
    // 函数一：曲率计算
    std::vector<float> get_curvature(const std::vector<CSV_get_Path::Coord>& global_path, int dist)
    {
        std::vector<float> curvature(global_path.size(), 0.0f);
        
        if (global_path.size() < 2*dist + 1) {
            return curvature;
        }
        
        for (int i = dist; i < (int)global_path.size() - dist; i++)
        {
            float x1 = global_path[i - dist].x;
            float y1 = global_path[i - dist].y;
            float x2 = global_path[i].x;
            float y2 = global_path[i].y;
            float x3 = global_path[i + dist].x;
            float y3 = global_path[i + dist].y;
            
            float dx12 = x2 - x1;
            float dy12 = y2 - y1;
            float a = sqrtf(dx12 * dx12 + dy12 * dy12);
            
            float dx23 = x3 - x2;
            float dy23 = y3 - y2;
            float b = sqrtf(dx23 * dx23 + dy23 * dy23);
            
            float dx31 = x1 - x3;
            float dy31 = y1 - y3;
            float c = sqrtf(dx31 * dx31 + dy31 * dy31);
            
            float den = a * b * c;
            if (den < 0.0001f) {
                curvature[i] = 0.0f;
                continue;
            }
            
            float cross_product = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
            curvature[i] = 2.0f * cross_product / den;
        }
        
        // 边界填充
        for (int i = 0; i < dist; i++) {
            curvature[i] = curvature[dist];
        }
        
        for (int i = (int)global_path.size() - dist; i < (int)global_path.size(); i++) 
        {
            curvature[i] = curvature[global_path.size() - dist - 1];
        }
        
        return curvature;
    }

    // 函数二：Savitzky-Golay曲率平滑
    std::vector<float> smooth_curvature(const std::vector<float>& curvature, 
                                        int window_size = 7, 
                                        int poly_order = 2)
    {
        // 检查输入有效性
        if (curvature.empty()) return curvature;
        if (curvature.size() < 5) return curvature;  // 数据太少，不处理
        
        // 确保窗口大小为奇数
        if (window_size % 2 == 0) window_size++;
        if (window_size < 3) window_size = 3;
        if (window_size > 11) window_size = 11;  // 最大窗口限制
        
        // 确保多项式阶数合理
        if (poly_order < 1) poly_order = 1;
        if (poly_order > 4) poly_order = 4;
        
        // 确保窗口大小大于多项式阶数
        if (window_size <= poly_order) {
            window_size = poly_order + 2;
            if (window_size % 2 == 0) window_size++;
        }
        
        // 获取滤波器系数
        std::vector<float> coeffs = computeSG_Coefficients(window_size, poly_order);
        int half_window = window_size / 2;
        
        // 创建输出数组
        std::vector<float> smoothed(curvature.size(), 0.0f);
        
        // 方法1：使用镜像边界处理（效果更好）
        std::vector<float> extended_data = mirror_boundary(curvature, half_window);
        
        // 应用Savitzky-Golay滤波
        for (int i = 0; i < (int)curvature.size(); ++i) {
            float sum = 0.0f;
            for (int j = -half_window; j <= half_window; ++j) {
                sum += extended_data[i + j + half_window] * coeffs[j + half_window];
            }
            smoothed[i] = sum;
        }
        
        return smoothed;
    }

    // 函数三：一体化函数（计算+平滑）
    std::vector<float> get_smoothed_curvature(const std::vector<CSV_get_Path::Coord>& global_path, 
                                              int dist = 2,
                                              int smooth_window = 7,
                                              int smooth_poly_order = 2)
    {
        // 1. 计算原始曲率
        std::vector<float> raw_curvature = get_curvature(global_path, dist);
        
        // 2. 平滑处理
        std::vector<float> smoothed = smooth_curvature(raw_curvature, smooth_window, smooth_poly_order);
        
        return smoothed;
    }

    // 函数四：可视化辅助函数（用于调试）
    void print_curvature_stats(const std::vector<float>& curvature, const std::string& name = "曲率") {
        if (curvature.empty()) {
            std::cout << name << ": 空数组" << std::endl;
            return;
        }
        
        float max_val = curvature[0];
        float min_val = curvature[0];
        float avg_val = 0.0f;
        
        for (float val : curvature) {
            if (val > max_val) max_val = val;
            if (val < min_val) min_val = val;
            avg_val += val;
        }
        avg_val /= curvature.size();
        
        std::cout << "=== " << name << "统计 ===" << std::endl;
        std::cout << "长度: " << curvature.size() << std::endl;
        std::cout << "最大值: " << max_val << std::endl;
        std::cout << "最小值: " << min_val << std::endl;
        std::cout << "平均值: " << avg_val << std::endl;
        std::cout << "=====================" << std::endl;
    }

    // 函数五：测试不同平滑参数
    void test_smoothing_parameters(const std::vector<float>& curvature) {
        std::cout << "\n=== 曲率平滑参数测试 ===" << std::endl;
        
        // 测试不同窗口大小
        std::vector<int> test_windows = {5, 7, 9};
        for (int window : test_windows) {
            std::vector<float> smoothed = smooth_curvature(curvature, window, 2);
            
            // 计算平滑度（相邻点差值的均方根）
            float smoothness = 0.0f;
            for (size_t i = 1; i < smoothed.size(); ++i) {
                float diff = smoothed[i] - smoothed[i-1];
                smoothness += diff * diff;
            }
            smoothness = sqrtf(smoothness / (smoothed.size() - 1));
            
            std::cout << "窗口=" << window << " | 平滑度=" << smoothness << std::endl;
        }
    }
};

//这个类用于将全局路径转换为局部路径，输入全局路径和当前坐标，得到局部路径
class Path_transform
{
    public:
        /***********相关参数*******/
        int last_index = 0; //记录上一次离得最近的路径点的下标
        //int publish = 0;
        //int stop_flag = 0;

    public:
        //计算两个点的距离
        double get_distance(const CSV_get_Path::Coord& p1, const CSV_get_Path::Coord& p2)
        {
            return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
        }

        //根据当前位置，找到全局路径中最近的路径点
        //输入全局路径和当前位置，返回最近的路径点的下标
        int find_nearest_index(const std::vector<CSV_get_Path::Coord>& path, const CSV_get_Path::Coord& current_pos)
        {
            int nearest_index;
            
            
            //最后一个点退出
            // if(last_index == path.size()-2)
            // {
            //   if(get_distance(path[last_index+1],current_pos) < get_distance(path[last_index],current_pos))
            //   {
            //     return -1;
                
            //   }
            // }
            
            
            for(nearest_index = last_index;nearest_index<path.size()-1;nearest_index++)
            {
                if(get_distance(path[nearest_index],current_pos) < get_distance(path[nearest_index+1],current_pos))
                {
                    break;
                }
            }

             last_index = nearest_index;
             return nearest_index;
        }



       std::vector<CSV_get_Path::Coord> transform(const std::vector<CSV_get_Path::Coord>& global_path_segment, 
                                          const CSV_get_Path::Coord& current_pos)
        {
            std::vector<CSV_get_Path::Coord> local_path;
            
            if (global_path_segment.empty()) {
                return local_path;
            }
            
            // 计算旋转矩阵的参数（从小车朝向的逆时针旋转）
            double cos_yaw = cos(current_pos.yaw);
            double sin_yaw = sin(current_pos.yaw);
            

    
            for (const auto& global_point : global_path_segment) 
            {
                CSV_get_Path::Coord local_point;
                
                // 1. 计算相对于小车位置的偏移
                double dx = global_point.x - current_pos.x;
                double dy = global_point.y - current_pos.y;
                
                // 2. 旋转到小车的坐标系
                // 公式：x' = dx*cos(theta) + dy*sin(theta)
                //       y' = -dx*sin(theta) + dy*cos(theta)
                local_point.x = dx * cos_yaw + dy * sin_yaw;
                local_point.y = -dx * sin_yaw + dy * cos_yaw;
                
                // 3. 处理朝向（全局朝向减去小车朝向）
                local_point.yaw = global_point.yaw - current_pos.yaw;
                // 将角度归一化到 [-π, π] 范围
                while (local_point.yaw > M_PI) local_point.yaw -= 2 * M_PI;
                while (local_point.yaw < -M_PI) local_point.yaw += 2 * M_PI;
                
                // 4. 保留其他属性
                local_point.k = global_point.k;//保留曲率，纵向速度和下标
                local_point.v = global_point.v;
                local_point.index = global_point.index;
                
                local_path.push_back(local_point);
            }
            
            return local_path;
        }

        //总函数
        //输出局部路径，输入全局路径和当前位置
        std::vector<CSV_get_Path::Coord> get_local_path(const std::vector<CSV_get_Path::Coord>& global_path, 
                                          const CSV_get_Path::Coord& current_pos, float minDist,int* nearest_index_ptr)
        {
            int nearest_index = find_nearest_index(global_path,current_pos);

            *nearest_index_ptr = nearest_index;
            
            // if(nearest_index >= global_path.size()-8)
            // {
            //     publish = 1;
            // }

            float currentTotalDist = 0;
            std::vector<CSV_get_Path::Coord> global_path_segment;

            for (int i = nearest_index; i < global_path.size(); i++) {
                if (currentTotalDist > minDist) break;
                if (!global_path_segment.empty()) currentTotalDist += get_distance(global_path_segment.back(), global_path[i]);
                global_path_segment.push_back(global_path[i]);
            }

            std::vector<CSV_get_Path::Coord> local_path;
            if(!global_path_segment.empty())  local_path = transform(global_path_segment,current_pos);
            return local_path;
        }

};


class AB_stop
{
  public:
    //ab点轨迹速度
     // a_path -- 粘进 get_local_path.cpp
// ab_a_path -- 粘进 get_local_path.cpp
// ab_a_path -- 粘进 get_local_path.cpp
// ab_a_path -- 粘进 get_local_path.cpp
// ab_a_path -- 粘进 get_local_path.cpp
// ab_a_path -- 粘进 get_local_path.cpp
// ab_a_path -- 粘进 get_local_path.cpp
// ab_a_path -- 粘进 get_local_path.cpp
// ab_a_path -- 粘进 get_local_path.cpp
std::vector<CSV_get_Path::Coord> ab_a_path = {
    CSV_get_Path::Coord(0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 0.0, 2.0000, 0),
    CSV_get_Path::Coord(0.1055596390462616, 0.0025406972297311, 0.0481274425393123, 0.0, 2.0000, 1),
    CSV_get_Path::Coord(0.2108707537214608, 0.0101565129709497, 0.0932158925065429, 0.0, 2.0000, 2),
    CSV_get_Path::Coord(0.3157727038857797, 0.0221928491956051, 0.1364198004731886, 0.0, 2.0000, 3),
    CSV_get_Path::Coord(0.4200344960382689, 0.0388689264795538, 0.1815938997634118, 0.0, 2.0000, 4),
    CSV_get_Path::Coord(0.5234213367286550, 0.0603206031353670, 0.2291395934401085, 0.0, 2.0000, 5),
    CSV_get_Path::Coord(0.6256281091314599, 0.0868207518856717, 0.2700507234468673, 0.0, 2.0000, 6),
    CSV_get_Path::Coord(0.7269065886445640, 0.1166479143369741, 0.2942072468585034, 0.0, 2.0000, 7),
    CSV_get_Path::Coord(0.8277173731586428, 0.1480538951736367, 0.3056011738378612, 0.0, 2.0000, 8),
    CSV_get_Path::Coord(0.9283000125348803, 0.1801843394440915, 0.3126820983654104, 0.0, 2.0000, 9),
    CSV_get_Path::Coord(1.0286563769579822, 0.2130149358034209, 0.3190313093619150, 0.0, 2.0000, 10),
    CSV_get_Path::Coord(1.1288230066496316, 0.2464200278120615, 0.3246245884197081, 0.0, 2.0000, 11),
    CSV_get_Path::Coord(1.2288059217104454, 0.2803712171651684, 0.3302048825871317, 0.0, 2.0000, 12),
    CSV_get_Path::Coord(1.3285935726442042, 0.3148921491709367, 0.3360472041747882, 0.0, 2.0000, 13),
    CSV_get_Path::Coord(1.4281729635131415, 0.3500092589264612, 0.3418263041089624, 0.0, 2.0000, 14),
    CSV_get_Path::Coord(1.5275549813510130, 0.3856812331810187, 0.3482535290107543, 0.0, 2.0000, 15),
    CSV_get_Path::Coord(1.6266748372184587, 0.4220754778871424, 0.3549394072046205, 0.0, 2.0000, 16),
    CSV_get_Path::Coord(1.7255710223062617, 0.4590731488661733, 0.3604240104487110, 0.0, 2.0000, 17),
    CSV_get_Path::Coord(1.8242856610299514, 0.4965524309729486, 0.3657169940047348, 0.0, 2.0000, 18),
    CSV_get_Path::Coord(1.9227846357417437, 0.5345949131900971, 0.3713789426021987, 0.0, 2.0000, 19),
    CSV_get_Path::Coord(2.0210685373414470, 0.5731896007317364, 0.3773152213516062, 0.0, 2.0000, 20),
    CSV_get_Path::Coord(2.1191088950205508, 0.6123987994189631, 0.3837881860313237, 0.0, 2.0000, 21),
    CSV_get_Path::Coord(2.2168850048742410, 0.6522626236750340, 0.3899882777770941, 0.0, 2.0000, 22),
    CSV_get_Path::Coord(2.3144316982520592, 0.6926845212562136, 0.3963839080735401, 0.0, 2.0000, 23),
    CSV_get_Path::Coord(2.4116899164565058, 0.7337957305510826, 0.4034607220967459, 0.0, 2.0000, 24)
};









// ab_b_path -- 粘进 get_local_path.cpp
// ab_b_path -- 粘进 get_local_path.cpp
// ab_b_path -- 粘进 get_local_path.cpp
// ab_b_path -- 粘进 get_local_path.cpp
// ab_a_path -- 粘进 get_local_path.cpp
std::vector<CSV_get_Path::Coord> ab_b_path = {
    CSV_get_Path::Coord(0.0000000000000000, 0.0000000000000000, 0.0000000000000000, 0.0, 2.0000, 0),
    CSV_get_Path::Coord(0.1046071910718607, 0.0002824603336539, 0.0054003886409521, 0.0, 2.0000, 1),
    CSV_get_Path::Coord(0.2092114024977713, 0.0011298338651940, 0.0102445563298106, 0.0, 2.0000, 2),
    CSV_get_Path::Coord(0.3138110264484705, 0.0024257357896938, 0.0143443499700626, 0.0, 2.0000, 3),
    CSV_get_Path::Coord(0.4184048184981558, 0.0041307832640521, 0.0187921871127402, 0.0, 2.0000, 4),
    CSV_get_Path::Coord(0.5229887882887048, 0.0063571062238851, 0.0235524574037862, 0.0, 2.0000, 5),
    CSV_get_Path::Coord(0.6275616019396557, 0.0090578505751827, 0.0279391380498777, 0.0, 2.0000, 6),
    CSV_get_Path::Coord(0.7321220684645894, 0.0122016306250431, 0.0321387201823587, 0.0, 2.0000, 7),
    CSV_get_Path::Coord(0.8366684980010111, 0.0157805933896801, 0.0367482964266346, 0.0, 2.0000, 8),
    CSV_get_Path::Coord(0.9411954757280063, 0.0198881825504173, 0.0418340270443526, 0.0, 2.0000, 9),
    CSV_get_Path::Coord(1.0457001275393674, 0.0245303330936634, 0.0406559768858829, 0.0, 2.0000, 10),
    CSV_get_Path::Coord(1.1502359339424602, 0.0283916122344307, 0.0279997985429540, 0.0, 2.0000, 11),
    CSV_get_Path::Coord(1.2548245109173526, 0.0303873043784745, 0.0126050309736959, 0.0, 2.0000, 12),
    CSV_get_Path::Coord(1.3594301917353748, 0.0310286519985174, 0.0032188867367615, 0.0, 2.0000, 13),
    CSV_get_Path::Coord(1.4640379039650255, 0.0310607409203875, 0.0001530677266062, 0.0, 2.0000, 14),
    CSV_get_Path::Coord(1.5686456212372684, 0.0310606761289323, 0.0010615308167525, 0.0, 2.0000, 15),
    CSV_get_Path::Coord(1.6732530902176321, 0.0312828293713472, 0.0037522317650276, 0.0, 2.0000, 16),
    CSV_get_Path::Coord(1.7778592804546935, 0.0318456979509001, 0.0109321981249455, 0.0, 2.0000, 17),
    CSV_get_Path::Coord(1.8824527865932823, 0.0335699330133691, 0.0269623327074424, 0.0, 2.0000, 18),
    CSV_get_Path::Coord(1.9869868269829651, 0.0374856311874322, 0.0502083046514198, 0.0, 2.0000, 19),
    CSV_get_Path::Coord(2.0913859970465647, 0.0440689390018469, 0.0758077351472735, 0.0, 2.0000, 20),
    CSV_get_Path::Coord(2.1955816647732660, 0.0533290948438329, 0.1015438203282261, 0.0, 2.0000, 21),
    CSV_get_Path::Coord(2.2995032003121336, 0.0652748909482845, 0.1267622816989857, 0.0, 2.0000, 22),
    CSV_get_Path::Coord(2.4031001463006767, 0.0797764208780680, 0.1519139383488468, 0.0, 2.0000, 23),
    CSV_get_Path::Coord(2.5062913907327187, 0.0969328070208453, 0.1755352459303521, 0.0, 2.0000, 24),
    CSV_get_Path::Coord(2.6090885562620771, 0.1163106587143703, 0.1971033807939297, 0.0, 2.0000, 25)
};











    // ============================================================
    // 从 CSV 读 A/B 岔路轨迹。
    // 文件格式：x,y,yaw,v  （和 path.csv 一样，可带表头）
    // 返回 true 表示读成功并覆盖了 out；false 表示读不到，调用方保留原有的硬编码数组。
    // ============================================================
    bool load_csv_file(const std::string& path, std::vector<CSV_get_Path::Coord>& out)
    {
        std::ifstream f(path);
        if (!f.is_open())
        {
            return false;
        }

        std::vector<CSV_get_Path::Coord> pts;
        std::string line;
        bool first_line = true;

        while (std::getline(f, line))
        {
            if (!line.empty() && line.back() == '\r')     // 兼容 Windows 换行
            {
                line.pop_back();
            }
            if (line.empty())
            {
                continue;
            }

            // 跳过表头
            if (first_line && line.find("x,y") != std::string::npos)
            {
                first_line = false;
                continue;
            }
            first_line = false;

            std::stringstream ss(line);
            std::string token;
            std::vector<std::string> tokens;
            while (std::getline(ss, token, ','))
            {
                tokens.push_back(token);
            }

            if (tokens.size() < 3)
            {
                continue;
            }

            try
            {
                double x = std::stod(tokens[0]);
                double y = std::stod(tokens[1]);
                double yaw = std::stod(tokens[2]);
                // 第 4 列是速度；没写就按硬编码数组里的 2.0
                double v = (tokens.size() >= 4) ? std::stod(tokens[3]) : 2.0;
                pts.push_back(CSV_get_Path::Coord(x, y, yaw, 0.0, v, (int)pts.size()));
            }
            catch (const std::exception&)
            {
                continue;                                  // 坏行直接跳过，不影响其它点
            }
        }
        f.close();

        if (pts.size() < 2)                                // 太少了，视为无效
        {
            return false;
        }

        out = pts;
        return true;
    }

    //进行轨迹拼接的函数
    void concat_path( std::vector<CSV_get_Path::Coord>& old_path,int ab_flag,float dy)
    {
        //ab标识位
        if(ab_flag == 1)
        {
            //提取原轨迹最后一个点的x y yaw
            float x = old_path[old_path.size()-1].x;
            float y = old_path[old_path.size()-1].y;
            float yaw = old_path[old_path.size()-1].yaw;

            for(const auto& point:ab_a_path)
            {
                float new_yaw = point.yaw + yaw;
                if(new_yaw >M_PI)
                {
                    new_yaw -=2*M_PI;
                }
                if(new_yaw<-M_PI)
                {
                    new_yaw +=2*M_PI;
                }


                // 先按基准朝向旋转，再平移
                // 原版只平移、yaw 却加了基准朝向，两者对不上；
                // 岔路口朝向不为 0 时，轨迹会整体放错位置。
                float cs = cos(yaw), sn = sin(yaw);
                CSV_get_Path::Coord temp_point(
                    point.x*cs - point.y*sn + x,
                    point.x*sn + point.y*cs + y + dy,
                    new_yaw, point.k, point.v, (int)old_path.size());
                // 去重：和上一个点重合就不插入（A/B 段的第 0 个点就是接缝点本身，
                // 平移后会和主路径最后一个点完全重合，产生一条零长度线段）
                if (fabs(temp_point.x - old_path.back().x) < 1e-6 &&
                    fabs(temp_point.y - old_path.back().y) < 1e-6) continue;
                old_path.push_back(temp_point);
            }
        }
        else if(ab_flag == 2)
        {
            //提取原轨迹最后一个点的x y yaw
            float x = old_path[old_path.size()-1].x;
            float y = old_path[old_path.size()-1].y;
            float yaw = old_path[old_path.size()-1].yaw;

            for(const auto& point:ab_b_path)
            {
                float new_yaw = point.yaw + yaw;
                if(new_yaw >M_PI)
                {
                    new_yaw -=2*M_PI;
                }
                if(new_yaw<-M_PI)
                {
                    new_yaw +=2*M_PI;
                }
                // 先按基准朝向旋转，再平移
                // 原版只平移、yaw 却加了基准朝向，两者对不上；
                // 岔路口朝向不为 0 时，轨迹会整体放错位置。
                float cs = cos(yaw), sn = sin(yaw);
                CSV_get_Path::Coord temp_point(
                    point.x*cs - point.y*sn + x,
                    point.x*sn + point.y*cs + y + dy,
                    new_yaw, point.k, point.v, (int)old_path.size());
                // 去重：和上一个点重合就不插入（A/B 段的第 0 个点就是接缝点本身，
                // 平移后会和主路径最后一个点完全重合，产生一条零长度线段）
                if (fabs(temp_point.x - old_path.back().x) < 1e-6 &&
                    fabs(temp_point.y - old_path.back().y) < 1e-6) continue;
                old_path.push_back(temp_point);
            }
        }

    }


};




//现在，应该发布一个话题，不断的把预期全局路径和当前全局路径发布出去


struct Point2D
{
    float x;
    float y;

    Point2D() : x(0.0f), y(0.0f) {}

    Point2D(float x_, float y_) : x(x_), y(y_) {}
};


/**
 * ICP 结果
 */
struct ICPResult
{
    float dx = 0.0f;
    float dy = 0.0f;
    float dyaw = 0.0f;
    float error = std::numeric_limits<float>::max();
    int iterations = 0;
    bool converged = false;
};


/**
 * 2D ICP
 *
 * source_points: 当前雷达点云
 * target_points: init_scan.csv 参考点云
 *
 * 输出：
 * 当前雷达点云到参考雷达点云的 dx dy yaw
 */
class ICP2D
{
public:
    struct Config
    {
        int max_iterations = 30;
        float convergence_threshold = 1e-6f;
        float max_correspondence_dist = 0.4f;
        int min_correspondences = 20;
    };

    Config config_;

public:
    ICPResult match(
        const std::vector<Point2D>& source_points,
        const std::vector<Point2D>& target_points,
        float init_dx = 0.0f,
        float init_dy = 0.0f,
        float init_yaw = 0.0f)
    {
        ICPResult result;

        if (source_points.size() < 3 || target_points.size() < 3)
        {
            return result;
        }

        float total_dx = init_dx;
        float total_dy = init_dy;
        float total_yaw = init_yaw;

        float last_error = std::numeric_limits<float>::max();

        for (int iter = 0; iter < config_.max_iterations; ++iter)
        {
            std::vector<Point2D> transformed_source;
            transformed_source.reserve(source_points.size());

            float cos_yaw = std::cos(total_yaw);
            float sin_yaw = std::sin(total_yaw);

            for (const auto& p : source_points)
            {
                Point2D tp;
                tp.x = cos_yaw * p.x - sin_yaw * p.y + total_dx;
                tp.y = sin_yaw * p.x + cos_yaw * p.y + total_dy;
                transformed_source.push_back(tp);
            }

            std::vector<Point2D> src_corr;
            std::vector<Point2D> tgt_corr;

            src_corr.reserve(transformed_source.size());
            tgt_corr.reserve(transformed_source.size());

            float error_sum = 0.0f;
            int correspondence_count = 0;

            float max_dist_sq =
                config_.max_correspondence_dist *
                config_.max_correspondence_dist;

            for (const auto& src_p : transformed_source)
            {
                float best_dist_sq = std::numeric_limits<float>::max();
                Point2D best_target;

                for (const auto& tgt_p : target_points)
                {
                    float dx = src_p.x - tgt_p.x;
                    float dy = src_p.y - tgt_p.y;
                    float dist_sq = dx * dx + dy * dy;

                    if (dist_sq < best_dist_sq)
                    {
                        best_dist_sq = dist_sq;
                        best_target = tgt_p;
                    }
                }

                if (best_dist_sq < max_dist_sq)
                {
                    src_corr.push_back(src_p);
                    tgt_corr.push_back(best_target);
                    error_sum += std::sqrt(best_dist_sq);
                    correspondence_count++;
                }
            }

            if (correspondence_count < config_.min_correspondences)
            {
                result.iterations = iter + 1;
                result.error = last_error;
                result.converged = false;
                return result;
            }

            float mean_error =
                error_sum / static_cast<float>(correspondence_count);

            Point2D src_centroid;
            Point2D tgt_centroid;

            for (int i = 0; i < correspondence_count; ++i)
            {
                src_centroid.x += src_corr[i].x;
                src_centroid.y += src_corr[i].y;

                tgt_centroid.x += tgt_corr[i].x;
                tgt_centroid.y += tgt_corr[i].y;
            }

            src_centroid.x /= correspondence_count;
            src_centroid.y /= correspondence_count;

            tgt_centroid.x /= correspondence_count;
            tgt_centroid.y /= correspondence_count;

            Eigen::Matrix2f H = Eigen::Matrix2f::Zero();

            for (int i = 0; i < correspondence_count; ++i)
            {
                Eigen::Vector2f src_vec(
                    src_corr[i].x - src_centroid.x,
                    src_corr[i].y - src_centroid.y);

                Eigen::Vector2f tgt_vec(
                    tgt_corr[i].x - tgt_centroid.x,
                    tgt_corr[i].y - tgt_centroid.y);

                H += src_vec * tgt_vec.transpose();
            }

            Eigen::JacobiSVD<Eigen::Matrix2f> svd(
                H,
                Eigen::ComputeFullU | Eigen::ComputeFullV);

            Eigen::Matrix2f U = svd.matrixU();
            Eigen::Matrix2f V = svd.matrixV();

            Eigen::Matrix2f R = V * U.transpose();

            if (R.determinant() < 0)
            {
                V.col(1) *= -1;
                R = V * U.transpose();
            }

            Eigen::Vector2f src_c(src_centroid.x, src_centroid.y);
            Eigen::Vector2f tgt_c(tgt_centroid.x, tgt_centroid.y);

            Eigen::Vector2f t = tgt_c - R * src_c;

            float delta_yaw = std::atan2(R(1, 0), R(0, 0));
            float delta_dx = t.x();
            float delta_dy = t.y();

            /*
             * T_new = T_delta * T_total
             */
            float cos_delta = std::cos(delta_yaw);
            float sin_delta = std::sin(delta_yaw);

            float new_dx =
                cos_delta * total_dx -
                sin_delta * total_dy +
                delta_dx;

            float new_dy =
                sin_delta * total_dx +
                cos_delta * total_dy +
                delta_dy;

            float new_yaw = total_yaw + delta_yaw;

            total_dx = new_dx;
            total_dy = new_dy;
            total_yaw = normalize_angle(new_yaw);

            result.dx = total_dx;
            result.dy = total_dy;
            result.dyaw = total_yaw;
            result.error = mean_error;
            result.iterations = iter + 1;

            float error_change = std::fabs(last_error - mean_error);

            if (error_change < config_.convergence_threshold)
            {
                result.converged = true;
                return result;
            }

            last_error = mean_error;
        }

        result.dx = total_dx;
        result.dy = total_dy;
        result.dyaw = total_yaw;
        result.error = last_error;
        result.iterations = config_.max_iterations;
        result.converged = true;

        return result;
    }

private:
    static float normalize_angle(float angle)
    {
        while (angle > ICP_PI)
        {
            angle -= 2.0f * ICP_PI;
        }

        while (angle < -ICP_PI)
        {
            angle += 2.0f * ICP_PI;
        }

        return angle;
    }
};
class GetLocalPath : public rclcpp::Node
{
private:
    /***********话题相关*************/
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr current_path_pub_;


    /*********** ICP 初始校准相关 *************/
    std::string init_scan_path_ =
        "/home/racecar/src/my_racecar/include/init_scan.csv";

    int skip_frames_ = 5;
    int scan_count_ = 0;

    double error_threshold_ = 0.1;

    std::vector<Point2D> ref_points_;
    std::vector<Point2D> current_points_;

    ICP2D icp_;

    float result_dx_ = 0.0f;
    float result_dy_ = 0.0f;
    float result_dyaw_ = 0.0f;
    float match_error_ = 0.0f;

    bool calibration_done_ = false;


    /*********路径相关参数*******/
    std::vector<CSV_get_Path::Coord> global_path_;
    std::string csv_path =
        "/home/racecar/src/my_racecar/include/path.csv";

    std::vector<CSV_get_Path::Coord> current_path_;

    std::shared_ptr<PathCSVLogger> csv_logger_;
    rclcpp::Time start_time_;

    std::vector<float> curvature_;
    Curvature curvature_smooth_;

    Path_transform path_transformer;

    float front_dist = 1.5;

    float last_x = 0.0f;
    float last_y = 0.0f;

    int pub_global_path = 0;

    float concat_dy = 0.0f;

    int stop_point = 2;
    int slow_point = 20;

    int ab_flag = 1;

    // A/B 岔路轨迹的 CSV 路径（改轨迹只要换文件 + 重启节点，不用重新编译）
    std::string ab_a_path_file_ = "/home/racecar/src/my_racecar/include/ab_a.csv";
    std::string ab_b_path_file_ = "/home/racecar/src/my_racecar/include/ab_b.csv";

    int start_icp = 0;


public:
    GetLocalPath() : Node("get_local_path")
    {
        /*********** 参数声明 *************/
        this->declare_parameter("front_dist", front_dist);
        this->declare_parameter("stop_point", stop_point);
        this->declare_parameter("slow_point", slow_point);
        this->declare_parameter("pub_global_path", pub_global_path);

        this->declare_parameter("init_scan_path", init_scan_path_);
        this->declare_parameter("skip_frames", skip_frames_);
        this->declare_parameter("error_threshold", error_threshold_);
        this->declare_parameter("start_icp", start_icp);
        this->declare_parameter("ab_flag", ab_flag);
        this->declare_parameter("concat_dy", concat_dy);
        this->declare_parameter("ab_a_path_file", ab_a_path_file_);
        this->declare_parameter("ab_b_path_file", ab_b_path_file_);


        /*********** 获取参数 *************/
        this->get_parameter("front_dist", front_dist);
        this->get_parameter("stop_point", stop_point);
        this->get_parameter("slow_point", slow_point);
        this->get_parameter("pub_global_path", pub_global_path);

        this->get_parameter("init_scan_path", init_scan_path_);
        this->get_parameter("skip_frames", skip_frames_);
        this->get_parameter("error_threshold", error_threshold_);
        this->get_parameter("start_icp", start_icp);
        this->get_parameter("ab_flag", ab_flag);
        this->get_parameter("concat_dy", concat_dy);
        this->get_parameter("ab_a_path_file", ab_a_path_file_);
        this->get_parameter("ab_b_path_file", ab_b_path_file_);


        /*********** 1. 读取全局路径 *************/
        CSV_get_Path path_loader(csv_path);
        global_path_ = path_loader.get_path();

        /*********** A/B 停车轨迹拼接（必须放在打速度标记、算曲率之前）***********/
        if(ab_flag != 0)
        {
            AB_stop ab;

            // A/B 岔路轨迹优先从 CSV 读（改轨迹不用重新编译）；
            // 读不到就用源码里硬编码的，行为和以前完全一样。
            if(ab.load_csv_file(ab_a_path_file_, ab.ab_a_path))
            {
                RCLCPP_INFO(this->get_logger(), "A 轨迹：从 %s 读入 %zu 个点",
                            ab_a_path_file_.c_str(), ab.ab_a_path.size());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "A 轨迹：%s 读不到，用源码里硬编码的 %zu 个点",
                            ab_a_path_file_.c_str(), ab.ab_a_path.size());
            }

            if(ab.load_csv_file(ab_b_path_file_, ab.ab_b_path))
            {
                RCLCPP_INFO(this->get_logger(), "B 轨迹：从 %s 读入 %zu 个点",
                            ab_b_path_file_.c_str(), ab.ab_b_path.size());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "B 轨迹：%s 读不到，用源码里硬编码的 %zu 个点",
                            ab_b_path_file_.c_str(), ab.ab_b_path.size());
            }

            ab.concat_path(global_path_, ab_flag, concat_dy);
        }

        if (global_path_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "全局路径为空，请检查 path.csv");
        }


        /*********** 2. 创建 CSV 保存器 *************/
        csv_logger_ = std::make_shared<PathCSVLogger>(
            "/home/car_log/path_track",
            "path_tracking");

        if (csv_logger_)
        {
            csv_logger_->save_target_path(global_path_);
        }


        /*********** 3. 对速度进行赋值 *************/
        if (!global_path_.empty())
        {
            int path_size = static_cast<int>(global_path_.size());

            int slow_start =
                std::max<int>(0, path_size - slow_point);

            int stop_start =
                std::max<int>(0, path_size - stop_point);

            for (int i = slow_start; i < path_size; i++)
            {
                global_path_[i].v = -1;
            }

            for (int i = stop_start; i < path_size; i++)
            {
                global_path_[i].v = -2;
            }
        }


        /*********** 4. 计算曲率 *************/
        if (!global_path_.empty())
        {
            curvature_ =
                curvature_smooth_.get_smoothed_curvature(global_path_);

            for (int i = 0; i < static_cast<int>(global_path_.size()); i++)
            {
                global_path_[i].k = curvature_[i];
                global_path_[i].index = i;
            }
        }


        /*********** 5. 加载 ICP 参考雷达点云 *************/
        
        if(start_icp)
        {
        
          if (!load_reference_scan())
          {
              RCLCPP_ERROR(this->get_logger(), "加载 ICP 参考点云失败");
          }
          else
          {
              RCLCPP_INFO(
                  this->get_logger(),
                  "ICP 参考点云加载成功，点数: %zu",
                  ref_points_.size());
          }

        }
        /*********** 6. 订阅雷达，用于 ICP 初始校准 *************/
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(
                &GetLocalPath::scan_callback,
                this,
                std::placeholders::_1));


        /*********** 7. 订阅里程计话题 *************/
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/encoder_imu_odom",
            10,
            std::bind(
                &GetLocalPath::odom_callback,
                this,
                std::placeholders::_1));


        /*********** 8. 发布局部路径话题 *************/
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/my_local_path",
            10);


        /*********** 9. 发布全局路径话题 *************/
        global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/my_global_path",
            10);


        /*********** 10. 发布当前路径话题 *************/
        current_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/my_current_path",
            10);


        start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "GetLocalPath 节点启动");
        RCLCPP_INFO(this->get_logger(), "等待 /scan 进行 ICP 初始校准...");
    }


    /*********** 角度归一化 *************/
    double normalize_angle(double angle)
    {
        while (angle > ICP_PI)
        {
            angle -= 2.0 * ICP_PI;
        }

        while (angle < -ICP_PI)
        {
            angle += 2.0 * ICP_PI;
        }

        return angle;
    }


    /*********** 四元数转 yaw *************/
    double quaternionToYaw(double x, double y, double z, double w)
    {
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);

        return std::atan2(siny_cosp, cosy_cosp);
    }


    /*********** 加载 ICP 参考雷达点云 *************/
    bool load_reference_scan()
    {
        std::ifstream file(init_scan_path_);

        if (!file.is_open())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "无法打开参考点云文件: %s",
                init_scan_path_.c_str());

            return false;
        }

        ref_points_.clear();

        std::string line;

        while (std::getline(file, line))
        {
            if (line.empty())
            {
                continue;
            }

            if (line[0] == '#')
            {
                continue;
            }

            if (line.find("index") != std::string::npos ||
                line.find("angle") != std::string::npos ||
                line.find("range") != std::string::npos ||
                line.find("x") != std::string::npos ||
                line.find("y") != std::string::npos)
            {
                continue;
            }

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

            /*
             * 支持两种格式：
             *
             * 1. index, angle, range, x, y
             *    使用 values[3], values[4]
             *
             * 2. x, y
             *    使用 values[0], values[1]
             */
            if (values.size() >= 5)
            {
                ref_points_.emplace_back(values[3], values[4]);
            }
            else if (values.size() >= 2)
            {
                ref_points_.emplace_back(values[0], values[1]);
            }
        }

        file.close();

        if (ref_points_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "参考点云为空");
            return false;
        }

        return true;
    }


    /*********** LaserScan 转 2D 点云 *************/
    std::vector<Point2D> scan_to_points(
        const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::vector<Point2D> points;
        points.reserve(msg->ranges.size());

        for (size_t i = 0; i < msg->ranges.size(); i++)
        {
            float range = msg->ranges[i];

            if (std::isnan(range) || std::isinf(range))
            {
                continue;
            }

            if (range < msg->range_min || range > msg->range_max)
            {
                continue;
            }

            if (range < 0.05f || range > 8.0f)
            {
                continue;
            }

            float angle =
                msg->angle_min +
                static_cast<float>(i) * msg->angle_increment;

            float x = range * std::cos(angle);
            float y = range * std::sin(angle);

            points.emplace_back(x, y);
        }

        return points;
    }


    /*********** ICP 匹配 *************/
    bool icp_match()
    {
        if (ref_points_.size() < 3 || current_points_.size() < 3)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "点云数量不足，无法 ICP。ref=%zu, current=%zu",
                ref_points_.size(),
                current_points_.size());

            return false;
        }

        icp_.config_.max_iterations = 30;
        icp_.config_.convergence_threshold = 1e-6f;
        icp_.config_.max_correspondence_dist = 0.4f;
        icp_.config_.min_correspondences = 20;

        auto result = icp_.match(
            current_points_,
            ref_points_,
            0.0f,
            0.0f,
            0.0f);

        /*
         * 核心：
         * ICP 校准结果存入成员变量
         */
        result_dx_ = result.dx;
        result_dy_ = result.dy;
        result_dyaw_ = result.dyaw;
        match_error_ = result.error;

        RCLCPP_INFO(this->get_logger(), "---------- ICP 匹配结果 ----------");
        RCLCPP_INFO(this->get_logger(), "dx    = %.6f m", result_dx_);
        RCLCPP_INFO(this->get_logger(), "dy    = %.6f m", result_dy_);
        RCLCPP_INFO(this->get_logger(), "yaw   = %.6f rad", result_dyaw_);
        RCLCPP_INFO(
            this->get_logger(),
            "yaw   = %.3f deg",
            result_dyaw_ * 180.0 / ICP_PI);
        RCLCPP_INFO(this->get_logger(), "error = %.6f", match_error_);
        RCLCPP_INFO(this->get_logger(), "iterations = %d", result.iterations);
        RCLCPP_INFO(
            this->get_logger(),
            "converged = %s",
            result.converged ? "true" : "false");

        if (!result.converged)
        {
            return false;
        }

        if (result.error > error_threshold_)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "ICP 误差过大：%.6f > %.6f",
                result.error,
                error_threshold_);

            return false;
        }

        return true;
    }


    /*********** 雷达回调：只用于 ICP 初始校准 *************/
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if(start_icp==0)
        {
          return;
        }
    
        if (calibration_done_)
        {
            return;
        }

        scan_count_++;

        if (scan_count_ <= skip_frames_)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "跳过第 %d 帧雷达数据",
                scan_count_);

            return;
        }

        current_points_ = scan_to_points(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "开始 ICP 初始校准，当前雷达点数: %zu",
            current_points_.size());

        if (icp_match())
        {
            calibration_done_ = true;

            RCLCPP_INFO(this->get_logger(), "");
            RCLCPP_INFO(this->get_logger(), "========== ICP 初始校准成功 ==========");
            RCLCPP_INFO(
                this->get_logger(),
                "校准结果已存入成员变量：");
            RCLCPP_INFO(this->get_logger(), "result_dx_   = %.6f m", result_dx_);
            RCLCPP_INFO(this->get_logger(), "result_dy_   = %.6f m", result_dy_);
            RCLCPP_INFO(this->get_logger(), "result_dyaw_ = %.6f rad", result_dyaw_);
            RCLCPP_INFO(
                this->get_logger(),
                "result_dyaw_ = %.3f deg",
                result_dyaw_ * 180.0 / ICP_PI);
            RCLCPP_INFO(this->get_logger(), "=====================================");
            RCLCPP_INFO(this->get_logger(), "");

            /*
             * 初始校准只需要一次，
             * 成功后取消 /scan 订阅。
             *
             * 注意：
             * 这里不会影响 odom_callback，
             * 你的路径发布功能会继续运行。
             */
            scan_sub_.reset();
        }
        else
        {
            RCLCPP_WARN(
                this->get_logger(),
                "ICP 初始校准失败，error=%.6f，等待下一帧重新匹配",
                match_error_);
        }
    }


    /*********** 里程计回调：原路径功能保留 *************/
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {

            
        double raw_x = msg->pose.pose.position.x;
        double raw_y = msg->pose.pose.position.y;
        
        auto& orientation = msg->pose.pose.orientation;
        
        double raw_yaw = quaternionToYaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w);
        
        // ICP 角度
        double c = std::cos(result_dyaw_);
        double s = std::sin(result_dyaw_);
        
        // 
        double current_x = c * raw_x - s * raw_y + result_dx_;
        double current_y = s * raw_x + c * raw_y + result_dy_;
        double current_yaw = normalize_angle(raw_yaw + result_dyaw_);
      




        CSV_get_Path::Coord current_pos(
            current_x,
            current_y,
            current_yaw,
            0);

        int nearest_index = 0;

        std::vector<CSV_get_Path::Coord> local_path =
            path_transformer.get_local_path(
                global_path_,
                current_pos,
                front_dist,
                &nearest_index);

        /*
         * 每 10cm 记录一次当前位置
         */
        if ((current_x - last_x) * (current_x - last_x) +
                (current_y - last_y) * (current_y - last_y) >
            0.01)
        {
            current_path_.push_back(
                CSV_get_Path::Coord(
                    current_x,
                    current_y,
                    current_yaw,
                    0));

            last_x = current_x;
            last_y = current_y;

            double t = (this->now() - start_time_).seconds();

            if (csv_logger_)
            {
                csv_logger_->save_current_point(
                    current_x,
                    current_y,
                    current_yaw,
                    t,
                    nearest_index);
            }

            if (pub_global_path == 1)
            {
                publish_current_path(
                    current_path_,
                    msg->header.stamp);

                publish_global_path(
                    global_path_,
                    msg->header.stamp);
            }
        }

        publish_local_path(
            local_path,
            msg->header.stamp);
    }


    /*********** 发布局部路径 *************/
    void publish_local_path(
        const std::vector<CSV_get_Path::Coord>& local_path,
        const builtin_interfaces::msg::Time& stamp)
    {
        if (local_path.empty())
        {
            return;
        }

        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.stamp = stamp;
        path_msg.header.frame_id = "laser_link";

        for (const auto& point : local_path)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;

            pose_stamped.header.stamp = stamp;
            pose_stamped.header.frame_id = "laser_link";

            pose_stamped.pose.position.x = point.x;
            pose_stamped.pose.position.y = point.y;
            pose_stamped.pose.position.z = point.k;

            yaw_to_quaternion(
                point.yaw,
                pose_stamped.pose.orientation,
                point.v,
                point.index);

            path_msg.poses.push_back(pose_stamped);
        }

        path_pub_->publish(path_msg);
    }


    /*********** 发布全局路径 *************/
    void publish_global_path(
        const std::vector<CSV_get_Path::Coord>& global_path,
        const builtin_interfaces::msg::Time& stamp)
    {
        if (global_path.empty())
        {
            return;
        }

        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.stamp = stamp;
        path_msg.header.frame_id = "map";

        for (const auto& point : global_path)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;

            pose_stamped.header.stamp = stamp;
            pose_stamped.header.frame_id = "map";

            pose_stamped.pose.position.x = point.x;
            pose_stamped.pose.position.y = point.y;
            pose_stamped.pose.position.z = point.k;

            yaw_to_quaternion(
                point.yaw,
                pose_stamped.pose.orientation,
                0,
                0);

            path_msg.poses.push_back(pose_stamped);
        }

        global_path_pub_->publish(path_msg);
    }


    /*********** 发布当前实际路径 *************/
    void publish_current_path(
        const std::vector<CSV_get_Path::Coord>& current_path,
        const builtin_interfaces::msg::Time& stamp)
    {
        if (current_path.empty())
        {
            return;
        }

        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.stamp = stamp;
        path_msg.header.frame_id = "map";

        for (const auto& point : current_path)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;

            pose_stamped.header.stamp = stamp;
            pose_stamped.header.frame_id = "map";

            pose_stamped.pose.position.x = point.x;
            pose_stamped.pose.position.y = point.y;
            pose_stamped.pose.position.z = point.k;

            yaw_to_quaternion(
                point.yaw,
                pose_stamped.pose.orientation,
                0,
                0);

            path_msg.poses.push_back(pose_stamped);
        }

        current_path_pub_->publish(path_msg);
    }


    /*********** yaw 转四元数 *************/
    void yaw_to_quaternion(
        double yaw,
        geometry_msgs::msg::Quaternion& quat,
        double v,
        double index)
    {
        double half_yaw = yaw * 0.5;

        double sin_half_yaw = std::sin(half_yaw);
        double cos_half_yaw = std::cos(half_yaw);

        quat.w = cos_half_yaw;

        /*
         * 保留你原来的设计：
         * quat.x 存速度标记 v
         * quat.y 存 index
         */
        quat.x = v;
        quat.y = index;
        quat.z = sin_half_yaw;
    }


    /*********** 可选 getter *************/
    bool is_calibration_done() const
    {
        return calibration_done_;
    }

    float get_result_dx() const
    {
        return result_dx_;
    }

    float get_result_dy() const
    {
        return result_dy_;
    }

    float get_result_dyaw() const
    {
        return result_dyaw_;
    }
};






int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GetLocalPath>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


//这个文件是一个实验测试思路可行性的代码
//这里需要做好可视化调试的工作,重要的是检验代码的可行性
//使用圆形ROI好处在于便于极坐标系下进行筛选，但是考虑到离得越远的点，距离会越远，因此需要调节ROI区域可能很大
//所以，在进行ROI区域筛选的时候，使用以原点为中点的矩形区域可能效果反而更号
//对ROI提取完毕后，使用聚类算法，将连续的点集使用一个点来代表
//之后在极坐标系下对点进行筛选处理，把不符合变小规则的点剔除出去，保证路障不会提取错误
//之后找到左右各自一个边界点，进而计算中点
//使用类对这个进行封装。之后直接调用
//之前已经做过聚类算法和ROI区域提取了，只是接口可能不是很适配
//我认为不需要引入opencv，而是使用自己结构体规定的点就可以了
#include <vector>
#include <limits>
#include <cmath>
#include <math.h>
#include <rclcpp/rclcpp.hpp> // 引入ROS2 C++客户端库
#include <visualization_msgs/msg/marker.hpp> // 引入可视化消息类型
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <algorithm>
#include <visualization_msgs/msg/marker_array.hpp>
#include <stdio.h>
#include <queue>

class Coord;
class PolarCoord;
class FindMiddlePointProcess;
/******数据类型的定义******/
//汽车坐标系下的点，单位m
class Coord {
public:
    double x;
    double y;

    Coord() : x(0), y(0) {
    }

    Coord(double x_val, double y_val) : x(x_val), y(y_val) {
    }

    PolarCoord toPolarCoord() const;
};

//极坐标系下的点,使用弧度值表示角度，半径单位m
class PolarCoord {
public:
    double theta;
    double r;

    PolarCoord() : theta(0), r(0) {
    }

    PolarCoord(double theta_val, double r_val) : theta(theta_val), r(r_val) {
    }

    Coord toCoord() const {
        Coord new_point;
        new_point.x = r * cos(theta);
        new_point.y = r * sin(theta);
        return new_point;
    };
};

PolarCoord Coord::toPolarCoord() const {
    PolarCoord new_point;
    new_point.theta = atan2(y, x);
    new_point.r = sqrt(x * x + y * y);

    if (new_point.theta < 0) {
        new_point.theta += 2 * M_PI;
    }

    return new_point;
}

class FindLineConstant {
public:
    /********ROI相关变量******/
    static constexpr double ROI_LENGTH = 3.0;//为了尽可能保证y轴上下都有路障，可以把ROI长度设大一点
    static constexpr double ROI_WIDTH = 2.5;

    /*******激光雷达相关变量******/
    static double ANGLE_INCREMENT;
    static constexpr double ANGLE_MIN = 0;

    /********连续点相关变量******/
    static constexpr double CONTINUAL_POINTS_MAX_DIST = 0.2; //小于0.2m的才认为是连续点

    /********单点合并为同一点相关变量******/
    static constexpr double MERGE_THRESHOLD = 0.25; //当两个点的曼哈顿距离小于这个值，就把两个点合并为同一个点

    /********判断上下多少度范围直接用于左边界点******/
    static constexpr double ANGLE_RANGE = 0.0873; //上下多少度范围直接用于左边界点,现在是5度
};
double FindLineConstant::ANGLE_INCREMENT = 0.006424524821341038;


class FindLineUtil {
public:
    //ROI点的提取
    static std::vector<PolarCoord> getROIRanges(const std::vector<float>& old_ranges, bool* find) //进行引入为了减少不必要的复制
    {
        //新的数组
        std::vector<PolarCoord> new_ranges;
        double length;
        double theta;


        //ROI区域筛选
        double y_min = -FindLineConstant::ROI_WIDTH / 2;
        double y_max = FindLineConstant::ROI_WIDTH / 2;
        double x_min = -FindLineConstant::ROI_LENGTH / 2;
        double x_max = FindLineConstant::ROI_LENGTH / 2;


        //记录存在的点的数量
        int num = 0;


        //逻辑关系
        for (size_t i = 0; i < old_ranges.size(); i++) {
            theta = FindLineConstant::ANGLE_MIN + FindLineConstant::ANGLE_INCREMENT * i;

            //先判断点是否都有效
            if (old_ranges[i] <= 0 || std::isinf(old_ranges[i]) || std::isnan(old_ranges[i])) {
                new_ranges.push_back(PolarCoord(theta, std::numeric_limits<double>::infinity()));
                continue;
            }

            length = old_ranges[i];

            Coord new_point = PolarCoord(theta, length).toCoord();

            if (new_point.x < x_min || new_point.x > x_max) {
                new_ranges.push_back(PolarCoord(theta, std::numeric_limits<double>::infinity()));
                continue;
            }
            if (new_point.y < y_min || new_point.y > y_max) {
                new_ranges.push_back(PolarCoord(theta, std::numeric_limits<double>::infinity()));
                continue;
            }

            //检测
            //std::cout<<"次数：" << i << "  角度:"<<theta<<"   半径"<<length<<std::endl;
            num++;
            new_ranges.push_back(PolarCoord(theta, length));
        }

        //std::cout << new_ranges.size() << std::endl;

        if (num > 0) {
            *find = true;
        }
        else {
            *find = false;
        }
        return new_ranges;
    }

     //聚类算法
    static std::vector<std::vector<Coord>> combineClosePoint(const std::vector<PolarCoord>& old_ranges) {
        std::vector<std::vector<Coord>> many_points;
        std::vector<Coord> one_points;

        int find_new_start = 1;


        for (size_t i = 0; i < old_ranges.size(); i++) {
            //寻找第一个新的非inf点
            if (find_new_start == 1) {
                if (std::isinf(old_ranges[i].r) || std::isnan(old_ranges[i].r)) {
                    continue;
                }
                else {
                    find_new_start = 0; //找到了新的非inf点

                    one_points.push_back(old_ranges[i].toCoord());
                    continue;
                }
            }
            else {
                //判断是否存在断点
                if (std::isinf(old_ranges[i].r) || std::isnan(old_ranges[i].r)) {
                    //std::cout << "becaue inf" << std::endl;
                    //发现断点，寻找新的起点
                    find_new_start = 1;
                    //把之前的数据推入点集
                    if (!one_points.empty()) {
                        many_points.push_back(one_points);
                        one_points.clear();
                    }
                    continue;
                }

                if (fabs(old_ranges[i].r - old_ranges[i - 1].r) > FindLineConstant::CONTINUAL_POINTS_MAX_DIST) {
                    //std::cout << "becaue too big" << std::endl;
                    //距离差距过大，也认为是断点，但是把当前点作为新的起点，跳过寻新起点
                    if (!one_points.empty()) {
                        many_points.push_back(one_points);
                        one_points.clear();
                    }
                    one_points.push_back(old_ranges[i].toCoord());
                    continue;
                }

                //把该点推入之前的集合之中
                one_points.push_back(old_ranges[i].toCoord());
            }
        }
        if (!one_points.empty()) {
            many_points.push_back(one_points);
            one_points.clear();
        }


        return many_points;
    }


    /*
    * 求最大曼哈顿距离
    */
    static double maxDist(const std::vector<Coord>& points) {
        if (points.size() < 2) {
            return 0.0;
        }

        // 曼哈顿距离：|x1-x2| + |y1-y2| = max(
        //     (x1 + y1) - (x2 + y2),
        //     (x1 - y1) - (x2 - y2),
        //     (-x1 + y1) - (-x2 + y2),
        //     (-x1 - y1) - (-x2 - y2)
        // )

        double max_sum = -std::numeric_limits<double>::infinity();
        double min_sum = std::numeric_limits<double>::infinity();
        double max_diff = -std::numeric_limits<double>::infinity();
        double min_diff = std::numeric_limits<double>::infinity();

        for (const auto& p : points) {
            double sum = p.x + p.y;
            double diff = p.x - p.y;

            max_sum = std::max(max_sum, sum);
            min_sum = std::min(min_sum, sum);
            max_diff = std::max(max_diff, diff);
            min_diff = std::min(min_diff, diff);
        }

        double dist1 = max_sum - min_sum; // 对应 (x1+y1) - (x2+y2)
        double dist2 = max_diff - min_diff; // 对应 (x1-y1) - (x2-y2)

        return std::max(dist1, dist2);
    }


    /*
    * 寻找几何中心点
    */
    static Coord findCenterPoint(const std::vector<Coord>& points) {
        if (points.empty()) {
            return {0, 0};
        }

        double sum_x = 0.0, sum_y = 0.0;


        for (const auto& p : points) {
            sum_x += p.x;
            sum_y += p.y;
        }

        return {sum_x / points.size(), sum_y / points.size()};
    }


    //提取得到所有点集的中心点
    static std::vector<Coord> getCenterPoints(const std::vector<std::vector<Coord>>& many_points) {
        int order = 0;
        Coord middle_point; //点集的中心点

        std::vector<Coord> road_block;


        for (const auto& points : many_points) {
            middle_point = findCenterPoint(points);
            road_block.push_back(middle_point);
        }


        return road_block;
    }


    /*
    * 把距离特别近的点进行合并
    */
    static std::vector<Coord> combine(const std::vector<Coord>& points) {
        //无点时直接返回
        if (points.empty()) return points;

        // 因为点的数量比较少，直接暴力算力要求也可以忽略不计
        double manha_dis;
        std::vector<Coord> new_points;
        std::vector<bool> merged(points.size(), false); // 标记点是否已被合并（哈希）

        for (size_t i = 0; i < points.size() - 1; i++) {
            if (merged[i]) continue; // 如果该点已被合并，跳过

            Coord current_point = points[i];
            int count = 1; // 统计合并到当前点的数量

            for (size_t j = i + 1; j < points.size(); j++) {
                if (merged[j]) continue; // 如果该点已被合并，跳过

                manha_dis = fabs(points[i].x - points[j].x) + fabs(points[i].y - points[j].y);
                //std::cout << "manhadis:" << manha_dis << std::endl;

                if (manha_dis < FindLineConstant::MERGE_THRESHOLD) {
                    // 合并点：累加坐标
                    current_point.x += points[j].x;
                    current_point.y += points[j].y;
                    count++;
                    merged[j] = true; // 标记为已合并
                }
            }

            // 计算平均坐标作为新点
            current_point.x /= count;
            current_point.y /= count;
            new_points.push_back(current_point);
            merged[i] = true; // 标记当前点已处理
        }

        // 处理最后一个点（如果未被合并）
        if (!merged[points.size() - 1]) {
            new_points.push_back(points[points.size() - 1]);
        }

        return new_points;
    }



    //把点转化为极坐标系
    static std::vector<PolarCoord> getPolarPoints(const std::vector<Coord>& points) {
        //首先，先把所有点都转化为极坐标系
        std::vector<PolarCoord> polar_points;
        for (const auto& p : points) {
            polar_points.push_back(p.toPolarCoord());
        }

        //之后对vector的顺序进行排序，按角度大小
        std::sort(polar_points.begin(), polar_points.end(), [](const PolarCoord& a, const PolarCoord& b) {
            return a.theta < b.theta;
        });


        return polar_points;
    }


    //把一些突然半径变长不符合规范的点删了
    static std::vector<PolarCoord> filter_polar_points(const std::vector<PolarCoord>& polar_points) {
        std::vector<PolarCoord> filtered_points;

        if (polar_points.empty()) {
            return filtered_points;
        }
        bool first1 = false;
        bool first2 = false;
        bool first3 = false;
        bool first4 = false;

        int num = 1;

        for (size_t i = 0; i < polar_points.size(); i++) {
            num = 1;
            if (polar_points[i].theta > M_PI / 2) {
                num = 2;
            }
            if (polar_points[i].theta > M_PI) {
                num = 3;
            }
            if (polar_points[i].theta > M_PI / 2 * 3) {
                num = 4;
            }

            if (num == 1) {
                if (!first1) {
                    filtered_points.push_back(polar_points[i]);
                    first1 = true;
                }
                else {
                    if (polar_points[i].r < polar_points[i - 1].r) {
                        filtered_points.push_back(polar_points[i]);
                    }
                }
            }
            if (num == 2) {
                if (!first2) {
                    filtered_points.push_back(polar_points[i]);
                    first2 = true;
                }
                else {
                    if (polar_points[i].r > polar_points[i - 1].r) {
                        filtered_points.push_back(polar_points[i]);
                    }
                }
            }
            if (num == 3) {
                if (!first3) {
                    filtered_points.push_back(polar_points[i]);
                    first3 = true;
                }
                else {
                    if (polar_points[i].r < polar_points[i - 1].r) {
                        filtered_points.push_back(polar_points[i]);
                    }
                }
            }
            if (num == 4) {
                if (!first4) {
                    filtered_points.push_back(polar_points[i]);
                    first4 = true;
                }
                else {
                    if (polar_points[i].r > polar_points[i - 1].r) {
                        filtered_points.push_back(polar_points[i]);
                    }
                }
            }
        }

        return filtered_points;
    }

    // 通过两点求直线方程，计算x=0时的y值
    static double calculate_y_at_x_zero(const Coord& p1, const Coord& p2) {
        // 计算斜率
        double slope = (p2.y - p1.y) / (p2.x - p1.x);

        // 计算截距：y = slope * x + intercept
        double intercept = p1.y - slope * p1.x;

        // 当x=0时，y = intercept
        return intercept;
    }


    //找到左右边界中点各自一个点，然后返回中点坐标
    static Coord getRoadMidPoint(const std::vector<PolarCoord>& polar_points, bool* find) {
        Coord left_point;
        Coord right_point;

        bool find_left = false;
        bool find_right = false;

        bool error = false;

        // 寻找左边界点
        for (const auto& p : polar_points) {
            if (fabs(p.theta - M_PI / 2) < FindLineConstant::ANGLE_RANGE) {
                left_point = p.toCoord();
                find_left = true;
                break;
            }
        }


        PolarCoord left_point_front;
        PolarCoord left_point_behind;
        PolarCoord right_point_front;
        PolarCoord right_point_behind;


        //如果没有发现一个在范围内的左边界点的话
        if (!find_left) {
            //开始寻找左侧前方一点，首先看第一个角度
            if (polar_points[0].theta > M_PI / 2) {
                //最小的角度障碍物就是左后方，这个时候无法正常找到左边界点，放弃这一帧的数据，等待新的机会
                error = true;
            }
            else {
                for (size_t i = 1; i < polar_points.size(); i++) {
                    //如果大于了90度
                    if (polar_points[i].theta > M_PI / 2) {
                        left_point_front = polar_points[i - 1];
                        if (polar_points[i].theta > M_PI) {
                            error = true; //说明左后方没有点
                        }
                        else {
                            left_point_behind = polar_points[i];
                            break;
                        }
                    }
                    if (i == polar_points.size() - 1) {
                        //没有出现一个大于90度的
                        error = true;
                    }
                }
            }

            // //计算左侧点
            if (!error) {
                //通过左侧前方和后方的点，可以列出这条直线的方程
                left_point = Coord(
                    0, calculate_y_at_x_zero(left_point_front.toCoord(), left_point_behind.toCoord()));
            }
        }

        // 寻找右边界点（类似逻辑）
        for (const auto& p : polar_points) {
            if (fabs(p.theta - M_PI * 3 / 2) < FindLineConstant::ANGLE_RANGE) {
                right_point = p.toCoord();
                find_right = true;
                break;
            }
        }
        if (!find_right) {
            //开始寻找右侧前方一点，首先看最后一个角度
            if (polar_points[polar_points.size() - 1].theta < M_PI * 3 / 2) {
                //最大的角度障碍物就是右后方，这个时候无法正常找到右边界点，放弃这一帧的数据，等待新的机会
                error = true;
            }
            else {
                for (size_t i = polar_points.size() - 2; i > 0; i--) {
                    //如果小于了270度
                    if (polar_points[i].theta < M_PI * 3 / 2) {
                        right_point_front = polar_points[i + 1];
                        if (polar_points[i].theta < M_PI) {
                            error = true; //说明右后方没有点
                        }
                        else {
                            right_point_behind = polar_points[i];
                            break;
                        }
                    }
                    if (i == 0) {
                        //没有出现一个小于270度的
                        error = true;
                    }
                }
                if (!error) {
                    //通过右侧前方和后方的点，可以列出这条直线的方程
                    right_point = Coord(
                        0, calculate_y_at_x_zero(right_point_front.toCoord(),
                                                 right_point_behind.toCoord()));
                }
            }
        }


        if (!error) {
            //计算中间点
            Coord middle_point = Coord((left_point.x + right_point.x) / 2, (left_point.y + right_point.y) / 2);
            *find = true;
            return middle_point;
        }
        else {
            *find = false;
            return Coord(0, 0);
        }
    }

        //函数二，数据转化，输入极坐标系下的所有点和新的中点，进行数据转化
    static std::vector<float> transform_to_new_points(std::vector<float>& old_ranges,
                                                      const Coord& new_origin, double rotateAngle) {

        // 1. 先批量计算所有新极坐标
        std::vector<PolarCoord> new_polar_points;
        new_polar_points.reserve(old_ranges.size());

        for (size_t i = 0; i < old_ranges.size(); i++) {
            if (old_ranges[i] <= 0 || std::isinf(old_ranges[i]) || std::isnan(old_ranges[i])) {
                new_polar_points.push_back(PolarCoord(0, std::numeric_limits<double>::infinity()));
                continue;
            }

            double theta_old = i * FindLineConstant::ANGLE_INCREMENT + FindLineConstant::ANGLE_MIN;
            double r_old = old_ranges[i];

            double x_old = r_old * cos(theta_old);
            double y_old = r_old * sin(theta_old);
            double x_new = x_old - new_origin.x;
            double y_new = y_old - new_origin.y;

            double theta_new = atan2(y_new, x_new);
            double r_new = sqrt(x_new * x_new + y_new * y_new);

            if (theta_new < 0) theta_new += 2 * M_PI;

            new_polar_points.push_back(PolarCoord(theta_new, r_new));
        }

        // 2. 初始化输出vector
        std::vector<float> new_ranges(old_ranges.size(), std::numeric_limits<float>::infinity());

        // 3. 将极坐标点映射回固定角度的vector<float>
        for (size_t i = 0; i < new_polar_points.size(); i++) {
            if (std::isinf(new_polar_points[i].r)) continue;

            double theta_new = new_polar_points[i].theta;
            double r_new = new_polar_points[i].r;

            // 找到最接近的标准角度索引
            int closest_index = (theta_new - FindLineConstant::ANGLE_MIN) / FindLineConstant::ANGLE_INCREMENT + 0.5; // 四舍五入

            if (closest_index >= 0 && closest_index < (int)new_ranges.size()) {
                // 取最小距离（处理多个点映射到同一角度的情况）
                if (std::isinf(new_ranges[closest_index]) || r_new < new_ranges[closest_index]) {
                    new_ranges[closest_index] = r_new;
                }
            }
        }
        return new_ranges;
        return rotateRadarRange(new_ranges, rotateAngle);
    }

    static std::vector<float> rotateRadarRange(std::vector<float> oldRange, double rotateAngle) {
        if (oldRange.empty()) return oldRange;
        const double EPS = 1e-6;
        if (std::abs(rotateAngle) < EPS) return oldRange;

        std::vector<float> newRange;
        int startIdx = (int)(rotateAngle / FindLineConstant::ANGLE_INCREMENT);
        if (startIdx < 0) startIdx += oldRange.size();
        for (int i = startIdx; i < oldRange.size(); i++) newRange.push_back(oldRange[i]);
        for (int i = 0; i < startIdx;i++) newRange.push_back(oldRange[i]);
        return newRange;
    }
};

class FindMiddlePointProcess {
public:

    /*****中点值**/
    Coord my_middle_point;
    bool find_middle = false;

    std::vector<Coord> findMidPointMain(const std::vector<float>& old_ranges) {
        find_middle = false;
        //第一步，进行roi点的提取
        bool find_point;
        std::vector<PolarCoord> ROI_points = FindLineUtil::getROIRanges(old_ranges, &find_point);

        //如果ROI区域内有点
        if (find_point) {
            //第二步，通过连续性检验分为一个个点集
            std::vector<std::vector<Coord>> many_points = FindLineUtil::combineClosePoint(ROI_points);

            //把路障的点集的中心点提取出来
            std::vector<Coord> road_block_old = FindLineUtil::getCenterPoints(many_points);

            if (!road_block_old.empty()) //确保不是把所有点滤除
            {
                //第四步，把相隔很近的点合并
                std::vector<Coord> road_block_new = FindLineUtil::combine(road_block_old);

                //第五步，把点转化为极坐标系下
                std::vector<PolarCoord> polar_points = FindLineUtil::getPolarPoints(road_block_new);

                //第六步，提取中点
                my_middle_point = FindLineUtil::getRoadMidPoint(FindLineUtil::filter_polar_points(polar_points), &find_middle);

                return road_block_new;
            }
            else {
                //所有点都被滤除了，说明ROI内的障碍物都不是路障
                std::cout << "all points is removed" << std::endl;
                return {};
            }
        }
        else {
            //ROI内无障碍物
            std::cout << "can not find any point in ROI" << std::endl;
            return {};
        }
    }
};

class FindContinueLineProcess {
public:
    /********类的初始化*****/
    FindMiddlePointProcess findMiddlePointProcess; //进行中点提取


    /*******中间数据*****/
    //当前中点
    Coord currentMidPoint{0, 0}; // 使用大括号初始化
    Coord current_origin{0, 0};


    /********参数调节*****/
    float dx = 0.2; //对于向前x的增量
    int point_num = 5; //向前找的点的数量

    /*******激光雷达相关变量******/
    double angle_increment = 0.006424524821341038;
    double angle_min = 0;

    std::vector<Coord> roadBlockPoints;

    //主函数
    std::vector<Coord> findContinueLineMain(std::vector<float>& ranges) {
        std::vector<Coord> middle_points; //用于存储所有的中点


        std::vector<PolarCoord> rangesToPolar;
        for (int i = 0; i < ranges.size(); i++) {
            rangesToPolar.push_back(PolarCoord(i * FindLineConstant::ANGLE_INCREMENT + FindLineConstant::ANGLE_MIN,
                ranges[i]));
        }
        std::vector<std::vector<Coord>> pointSet = FindLineUtil::combineClosePoint(rangesToPolar);
        roadBlockPoints = FindLineUtil::getCenterPoints(pointSet);

        //开始寻找中点
        for (int i = 0; i < point_num; i++) {
            if (i == 0) {
                //第一次提取，不用事先数据转化
                //直接提取第一个点
                findMiddlePointProcess.findMidPointMain(ranges);
                currentMidPoint = findMiddlePointProcess.my_middle_point;

                printf("middle_point : %d  : %f , %f\n", i, currentMidPoint.x, currentMidPoint.y);

                middle_points.push_back(currentMidPoint);

                //计算下一个点的坐标原点
                current_origin = Coord(currentMidPoint.x + dx, currentMidPoint.y);
                printf("next_origin : %d  : %f , %f\n", i, current_origin.x, current_origin.y);
            }
            else {
                //先进行数据转化
                //把当前点的坐标转化为以当前点为原点的坐标
                double rotateAngle = 0;
                if (i >= 2) {
                    Coord point1 = middle_points[i - 1];
                    Coord point2 = middle_points[i - 2];

                    rotateAngle = atan2(point1.y - point2.y, point1.x - point2.x);
                }
                findMiddlePointProcess.findMidPointMain(FindLineUtil::transform_to_new_points(ranges, current_origin, rotateAngle));
                currentMidPoint = findMiddlePointProcess.my_middle_point;

                printf("middle_point : %d  : %f , %f\n", i, currentMidPoint.x, currentMidPoint.y);

                //把当前点的坐标转化为以原始原点为原点的坐标
                Coord current_middle_point_origin = Coord(
                    currentMidPoint.x + current_origin.x, currentMidPoint.y + current_origin.y);
                middle_points.push_back(current_middle_point_origin);

                printf("middle_point_origin : %d  : %f , %f\n", i, current_middle_point_origin.x,
                       current_middle_point_origin.y);
                //得到新的坐标原点
                current_origin = Coord(current_middle_point_origin.x + dx, current_middle_point_origin.y);
                printf("next_origin : %d  : %f , %f\n", i, current_origin.x, current_origin.y);
            }
        }

        return middle_points;
    }
};


class get_middle_test : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    FindContinueLineProcess findContinueLineProc;
    int marker_id_;


    int point_num = 5;
    float dx = 0.1;

public:
    get_middle_test() : Node("get_middle_test_node"), marker_id_(0) {
        this->declare_parameter("point_num", point_num);
        this->declare_parameter("dx", dx); //

        this->get_parameter("point_num", point_num);
        this->get_parameter("dx", dx);

        findContinueLineProc.dx = dx;
        findContinueLineProc.point_num = point_num;

        // 订阅激光雷达
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&get_middle_test::laser_callback, this, std::placeholders::_1));

        // 发布可视化标记
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/roadblock_markers", 10);
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 转换激光数据
        std::vector<float> ranges(msg->ranges.begin(), msg->ranges.end());
        findContinueLineProc.angle_increment = msg->angle_increment;
        FindLineConstant::ANGLE_INCREMENT = msg->angle_increment;

        // 检测路障
        auto roadblocks = findContinueLineProc.findContinueLineMain(ranges);

        // 可视化路障
        // visualize_roadblocks(findContinueLineProc.roadBlockPoints, msg->header.frame_id, 2);

        visualize_roadblocks(roadblocks, msg->header.frame_id, 1);
    }

    void visualize_roadblocks(const std::vector<Coord>& roadblocks,
                              const std::string& frame_id, int color) {
        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < roadblocks.size(); ++i) {
            visualization_msgs::msg::Marker marker;

            // 基础设置
            marker.header.frame_id = frame_id;
            marker.header.stamp = this->now();
            marker.ns = "roadblocks";
            marker.id = marker_id_++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // 位置
            marker.pose.position.x = roadblocks[i].x;
            marker.pose.position.y = roadblocks[i].y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;

            // 尺寸和颜色
            marker.scale.x = 0.15;
            marker.scale.y = 0.15;
            marker.scale.z = 0.15;

            marker.color.r = 0.0; // 蓝色
            if (color == 1) marker.color.r = 1.0; // 蓝色
            marker.color.g = 0.0;
            if (color == 2) marker.color.g = 1.0;
            marker.color.b = 0.0;
            if (color == 3) marker.color.b = 1.0;
            marker.color.a = 0.8;

            marker.lifetime = rclcpp::Duration::from_seconds(0.2);

            marker_array.markers.push_back(marker);
        }


        marker_pub_->publish(marker_array);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<get_middle_test>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

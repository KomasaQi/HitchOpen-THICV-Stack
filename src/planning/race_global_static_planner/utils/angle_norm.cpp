#include "angle_norm.h"
#include <cmath>
#include <stdexcept>
#include <string>

namespace AngleUtils {

// 辅助函数：角度转弧度
inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// 辅助函数：弧度转角度
inline double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

// 辅助函数：将角度归一化到指定范围[limit_min, limit_max)
inline double normalize_angle(double angle, double limit_min, double limit_max) {
    double range_span = limit_max - limit_min;
    // 处理负数情况的取模运算
    angle = std::fmod(angle - limit_min, range_span);
    if (angle < 0) {
        angle += range_span;
    }
    return angle + limit_min;
}

// 标量-标量版本实现
double angle_norm(double angle1, double angle2, const Eigen::Vector2d& limit, const std::string& unit) {
    // 验证单位合法性
    if (unit != "rad" && unit != "deg") {
        throw std::invalid_argument("无效的单位，必须是\"rad\"或\"deg\"");
    }

    // 验证范围合法性
    if (limit[1] <= limit[0]) {
        throw std::invalid_argument("无效的周期范围，上限必须大于下限");
    }

    // 保存原始单位用于结果转换
    bool is_degree = (unit == "deg");
    
    // 转换为弧度计算
    double a1 = angle1;
    double a2 = angle2;
    Eigen::Vector2d lim = limit;
    
    if (is_degree) {
        a1 = deg2rad(a1);
        a2 = deg2rad(a2);
        lim[0] = deg2rad(lim[0]);
        lim[1] = deg2rad(lim[1]);
    }

    // 计算周期跨度
    double range_span = lim[1] - lim[0];
    
    // 归一化角度到指定范围
    double a1_norm = normalize_angle(a1, lim[0], lim[1]);
    double a2_norm = normalize_angle(a2, lim[0], lim[1]);
    
    // 计算周期性最短距离
    double direct_diff = a1_norm - a2_norm;
    double wrapped_diff = std::fmod(direct_diff + range_span / 2.0, range_span) - range_span / 2.0;
    double dist = std::abs(wrapped_diff);
    
    // 如果原始单位是角度，转换回角度
    if (is_degree) {
        dist = rad2deg(dist);
    }
    
    return dist;
}

// 标量-向量版本实现
Eigen::VectorXd angle_norm(double angle1, const Eigen::VectorXd& angle2, const Eigen::Vector2d& limit, const std::string& unit) {
    Eigen::VectorXd dist(angle2.size());
    for (int i = 0; i < angle2.size(); ++i) {
        dist[i] = angle_norm(angle1, angle2[i], limit, unit);
    }
    return dist;
}

// 向量-标量版本实现
Eigen::VectorXd angle_norm(const Eigen::VectorXd& angle1, double angle2, const Eigen::Vector2d& limit, const std::string& unit) {
    return angle_norm(angle2, angle1, limit, unit); // 利用对称性复用代码
}

// 向量-向量版本实现
Eigen::VectorXd angle_norm(const Eigen::VectorXd& angle1, const Eigen::VectorXd& angle2, const Eigen::Vector2d& limit, const std::string& unit) {
    if (angle1.size() != angle2.size()) {
        throw std::invalid_argument("输入向量尺寸不匹配");
    }
    
    Eigen::VectorXd dist(angle1.size());
    for (int i = 0; i < angle1.size(); ++i) {
        dist[i] = angle_norm(angle1[i], angle2[i], limit, unit);
    }
    return dist;
}

} // namespace AngleUtils

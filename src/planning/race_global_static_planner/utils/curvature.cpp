#include "curvature.h"
#include <cmath>
#include <stdexcept>

std::vector<double> Curvature::curvature(const std::vector<Eigen::Vector2d>& path) {
    if (path.size() < 3) {
        throw std::invalid_argument("Path must contain at least 3 points to calculate curvature");
    }
    
    size_t len = path.size();
    std::vector<double> k(len, 0.0);
    
    Eigen::Vector2d A = path[0];
    Eigen::Vector2d B = path[1];
    
    for (size_t i = 1; i < len - 1; ++i) {  // C++使用0基索引，对应MATLAB的2到len-1
        Eigen::Vector2d C = path[i + 1];
        
        Eigen::Vector2d BC = C - B;
        Eigen::Vector2d BA = A - B;
        
        // 计算BA的极坐标角度
        double theta = atan2(BA.y(), BA.x());
        Eigen::Vector2d bc = rotate(BC, -theta);
        
        double a = distance(B, C);
        double b = distance(A, C);
        double c = distance(A, B);
        
        // 计算cosA，使用数值稳定的方式避免精度问题
        double cosA = (b*b + c*c - a*a) / (2 * b * c);
        // 确保cosA在[-1, 1]范围内，避免acos返回NaN
        cosA = std::max(-1.0, std::min(1.0, cosA));
        
        double alpha = acos(cosA);
        
        // 计算曲率
        double angle = atan2(bc.y(), bc.x());
        k[i] = -std::signbit(angle) * std::sin(alpha) / (0.5 * a);
        
        // 更新点
        A = B;
        B = C;
    }
    
    // 处理边界点
    k[0] = k[1];
    k[len - 1] = k[len - 2];
    
    return k;
}

Eigen::Vector2d Curvature::rotate(const Eigen::Vector2d& vec0, double th) {
    double cosTh = cos(th);
    double sinTh = sin(th);
    
    Eigen::Vector2d vec1;
    vec1.x() = vec0.x() * cosTh - vec0.y() * sinTh;
    vec1.y() = vec0.x() * sinTh + vec0.y() * cosTh;
    
    return vec1;
}

double Curvature::distance(const Eigen::Vector2d& x1, const Eigen::Vector2d& x2) {
    return (x1 - x2).norm();
}

#include "angle_norm.h"
#include <iostream>

int main() {
    try {
        // 示例1：标量-标量对比（弧度）
        double dist1 = AngleUtils::angle_norm(3.14, -M_PI);
        std::cout << "示例1实际输出: " << dist1 << "     输入为: 3.14, -M_PI, 应输出接近0的值 " << std::endl;  // 应输出接近0的值

        // 示例2：标量-向量对比（弧度）
        Eigen::VectorXd angles(4);
        angles << 0, M_PI, 2*M_PI, 3*M_PI;
        Eigen::VectorXd dist2 = AngleUtils::angle_norm(0, angles);
        std::cout << "示例2实际输出: " << dist2.transpose() << "     输入为: 0, [0, π, 2π, 3π], 应输出[0, π, 0, π] " << std::endl;  // 应输出[0, π, 0, π]

        // 示例3：角度单位对比
        double dist3 = AngleUtils::angle_norm(180, -180, Eigen::Vector2d(-180, 180), "deg");
        std::cout << "示例3实际输出: " << dist3 << "     输入为: 180, -180, 应输出接近0的值 " << std::endl;  // 应输出接近0的值

        // 示例4：自定义范围[0, 2π]
        Eigen::Vector2d custom_limit(0, 2*M_PI);
        double dist4 = AngleUtils::angle_norm(0.5, 3.5, custom_limit);
        std::cout << "示例4实际输出: " << dist4 << "     输入为: 0.5, 3.5, 应输出3.0 " << std::endl;  // 应输出约3.0
    }
    catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
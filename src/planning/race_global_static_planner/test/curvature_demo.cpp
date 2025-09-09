#include <iostream>
#include <vector>
#include "curvature.h"
#include <Eigen/Dense>

int main() {
    try {
        // 创建Curvature对象
        Curvature curvatureCalculator;
        
        // 1. 创建示例轨迹点 - 圆形轨迹
        std::vector<Eigen::Vector2d> path;
        const double radius = 10.0;  // 圆半径
        const int numPoints = 100;    // 点数量
        
        for (int i = 0; i < numPoints; ++i) {
            double angle = 2 * M_PI * i / (numPoints - 1);  // 0到2π的角度
            double x = radius * cos(angle);
            double y = radius * sin(angle);
            path.emplace_back(x, y);
        }
        
        // 2. 计算曲率
        std::vector<double> curvatures = curvatureCalculator.curvature(path);
        
        // 3. 输出结果
        std::cout << "轨迹点数量: " << path.size() << std::endl;
        std::cout << "曲率计算结果:" << std::endl;
        std::cout << "索引\tX坐标\tY坐标\t曲率值\t半径估计" << std::endl;
        
        for (size_t i = 0; i < path.size(); ++i) {
            double radiusEstimate = (curvatures[i] != 0) ? 1.0 / fabs(curvatures[i]) : INFINITY;
            printf("%zu\t%.2f\t%.2f\t%.6f\t%.2f\n", 
                   i, 
                   path[i].x(), 
                   path[i].y(), 
                   curvatures[i],
                   radiusEstimate);
        }
    }
    catch (const std::exception& e) {
        std::cerr << "发生错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
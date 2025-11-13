#ifndef CURVATURE_H
#define CURVATURE_H

#include <Eigen/Dense>
#include <vector>

class Curvature {
public:
    /**
     * 计算轨迹上各点的曲率
     * @param path 轨迹点向量，每个点为Eigen::Vector2d
     * @return 每个点的曲率向量
     */
    std::vector<double> curvature(const std::vector<Eigen::Vector2d>& path);
    
private:
    /**
     * 旋转向量
     * @param vec0 原始向量
     * @param th 旋转角度(弧度)
     * @return 旋转后的向量
     */
    Eigen::Vector2d rotate(const Eigen::Vector2d& vec0, double th);
    
    /**
     * 计算两点之间的距离
     * @param x1 第一个点
     * @param x2 第二个点
     * @return 两点间的距离
     */
    double distance(const Eigen::Vector2d& x1, const Eigen::Vector2d& x2);
};

#endif // CURVATURE_H

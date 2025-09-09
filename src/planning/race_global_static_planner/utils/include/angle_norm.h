#ifndef ANGLE_NORM_H
#define ANGLE_NORM_H

#include <vector>
#include <string>
#include <Eigen/Dense>

namespace AngleUtils {

/**
 * @brief 计算两个角度之间的周期性最短距离
 * @param angle1 第一个角度（标量）
 * @param angle2 第二个角度（标量）
 * @param limit 角度周期范围，默认[-M_PI, M_PI]
 * @param unit 角度单位，"rad"（弧度）或"deg"（角度），默认"rad"
 * @return 两个角度之间的最短距离
 */
double angle_norm(double angle1, double angle2, 
                 const Eigen::Vector2d& limit = Eigen::Vector2d(-M_PI, M_PI), 
                 const std::string& unit = "rad");

/**
 * @brief 计算标量角度与向量角度之间的周期性最短距离
 * @param angle1 标量角度
 * @param angle2 向量角度
 * @param limit 角度周期范围，默认[-M_PI, M_PI]
 * @param unit 角度单位，"rad"（弧度）或"deg"（角度），默认"rad"
 * @return 距离向量，与angle2尺寸相同
 */
Eigen::VectorXd angle_norm(double angle1, const Eigen::VectorXd& angle2, 
                          const Eigen::Vector2d& limit = Eigen::Vector2d(-M_PI, M_PI), 
                          const std::string& unit = "rad");

/**
 * @brief 计算向量角度与标量角度之间的周期性最短距离
 * @param angle1 向量角度
 * @param angle2 标量角度
 * @param limit 角度周期范围，默认[-M_PI, M_PI]
 * @param unit 角度单位，"rad"（弧度）或"deg"（角度），默认"rad"
 * @return 距离向量，与angle1尺寸相同
 */
Eigen::VectorXd angle_norm(const Eigen::VectorXd& angle1, double angle2, 
                          const Eigen::Vector2d& limit = Eigen::Vector2d(-M_PI, M_PI), 
                          const std::string& unit = "rad");

/**
 * @brief 计算两个向量角度之间的周期性最短距离（对应元素）
 * @param angle1 第一个向量角度
 * @param angle2 第二个向量角度（需与angle1尺寸相同）
 * @param limit 角度周期范围，默认[-M_PI, M_PI]
 * @param unit 角度单位，"rad"（弧度）或"deg"（角度），默认"rad"
 * @return 距离向量，与输入向量尺寸相同
 */
Eigen::VectorXd angle_norm(const Eigen::VectorXd& angle1, const Eigen::VectorXd& angle2, 
                          const Eigen::Vector2d& limit = Eigen::Vector2d(-M_PI, M_PI), 
                          const std::string& unit = "rad");

} // namespace AngleUtils

#endif // ANGLE_NORM_H

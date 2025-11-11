#ifndef RACE_TRACKER_PID_TRACKER_H
#define RACE_TRACKER_PID_TRACKER_H

#include "race_tracker/controller_plugin_base.h"
#include <geometry_msgs/Point.h>
#include <cmath>
#include <limits>
#include <algorithm>

namespace race_tracker {

/**
 * @brief 基于PID的横向控制器（通过预瞄点误差计算转向角）
 */
class PIDTracker : public ControllerPluginBase {
public:
    PIDTracker();
    ~PIDTracker() override = default;

    // 实现基类接口
    bool initialize(ros::NodeHandle& nh) override;
    void computeControl(
        const race_msgs::VehicleStatusConstPtr& vehicle_status,
        const race_msgs::PathConstPtr& path,
        race_msgs::Control* control_msg,
        const double dt,
        const race_msgs::Flag::ConstPtr& flag) override;
    std::string getName() const override { return "PIDTracker"; }

private:
    /**
     * @brief 查找预瞄点（参考纯追踪算法）
     * @param vehicle_status 车辆状态
     * @param path 局部路径
     * @return 有效预瞄点
     */
    geometry_msgs::Point findLookaheadPoint(
        const race_msgs::VehicleStatusConstPtr& vehicle_status,
        const race_msgs::PathConstPtr& path,
        size_t closest_idx); // 新增：传入已计算的最近点索引


    /**
     * @brief 转换点到车辆坐标系
     * @param world_point 世界坐标系点
     * @param vehicle_pose 车辆位姿
     * @return 车辆坐标系点
     */
    geometry_msgs::Point transformToVehicleFrame(
        const geometry_msgs::Point& world_point,
        const geometry_msgs::Pose& vehicle_pose);

    /**
     * @brief 计算动态预瞄距离
     * @param current_speed 当前速度
     * @return 预瞄距离
     */
    double calculateDynamicLookahead(double current_speed);

    /**
     * @brief 计算航向误差（车辆朝向与路径方向的夹角）
     * @param vehicle_pose 车辆位姿
     * @param path 局部路径
     * @param closest_idx 最近路径点索引
     * @param lookahead_point 预瞄点位置
     * @return 航向误差（rad）
     */
    double calculateHeadingError(
        const geometry_msgs::Pose& vehicle_pose,
        const geometry_msgs::Point& closest_point, // 最近点位置
        const geometry_msgs::Point& lookahead_point); // 预瞄点位置

    /**
     * @brief 重置PID状态
     */
    void resetPID();

private:
    // 预瞄相关参数
    double min_lookahead_distance_;  // 最小预瞄距离（m）
    double lookahead_speed_coeff_;   // 预瞄距离速度系数（s）
    double current_lookahead_;       // 当前预瞄距离（动态更新）
    double min_path_points_;         // 最小有效路径点数

    // PID控制参数
    double kp_;                      // 比例增益
    double ki_;                      // 积分增益
    double kd_;                      // 微分增益
    double integral_limit_;          // 积分限幅
    double last_steering_angle_;     // 上一个时刻的转角指令
    double max_steering_angle_;      // 最大转向角（rad）

    // 误差权重参数
    double lateral_error_weight_;    // 横向误差权重
    double heading_error_weight_;    // 航向误差权重

    // PID状态变量
    double last_error_;              // 上一次综合误差
    double integral_error_;          // 积分误差累积

    // 动力学相关参数
    double max_steering_rate_;     // 最大转向角变化率（rad/s）
};

} // namespace race_tracker

#endif // RACE_TRACKER_PID_TRACKER_H
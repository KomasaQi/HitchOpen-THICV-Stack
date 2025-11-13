#ifndef RACE_TRACKER_PURE_PURSUIT_CONTROLLER_H
#define RACE_TRACKER_PURE_PURSUIT_CONTROLLER_H

#include "race_tracker/controller_plugin_base.h"
#include <geometry_msgs/Point.h>
#include <cmath>
#include <limits>
#include <algorithm>


namespace race_tracker {

/**
 * @brief 纯追踪横向控制器（基于预瞄点的转向角计算）
 */
class PurePursuitController : public ControllerPluginBase {
public:
    PurePursuitController();
    ~PurePursuitController() override = default;

    // 实现基类接口
    bool initialize(ros::NodeHandle& nh) override;
    void computeControl(
        const race_msgs::VehicleStatusConstPtr& vehicle_status,
        const race_msgs::PathConstPtr& path,
        race_msgs::Control* control_msg,
        const double dt,
        const race_msgs::Flag::ConstPtr& flag) override;
    std::string getName() const override { return "PurePursuitController"; }

private:
    /**
     * @brief 查找预瞄点（车辆前方最近的、距离满足预瞄距离的路径点）
     * @param vehicle_status 车辆状态（位置+姿态）
     * @param path 局部路径
     * @return 有效预瞄点（若未找到则返回路径最后一个点）
     */
    geometry_msgs::Point findLookaheadPoint(
        const race_msgs::VehicleStatusConstPtr& vehicle_status,
        const race_msgs::PathConstPtr& path);

    /**
     * @brief 计算车辆坐标系下的预瞄点（用于纯追踪公式）
     * @param world_point 世界坐标系下的预瞄点
     * @param vehicle_pose 车辆在世界坐标系下的位姿
     * @return 车辆坐标系下的预瞄点（x向前，y向左）
     */
    geometry_msgs::Point transformToVehicleFrame(
        const geometry_msgs::Point& world_point,
        const geometry_msgs::Pose& vehicle_pose);
    /**
     * @brief 根据当前速度计算动态预瞄距离
     * @param current_speed 车辆当前速度（m/s）
     * @return 动态预瞄距离（不小于最小预瞄距离）
     */
    double calculateDynamicLookahead(double current_speed) {
        // 确保速度非负（取绝对值，兼容倒车场景）
        double speed = std::fabs(current_speed);
        // 动态预瞄距离 = 最小距离 + 速度×系数
        double dynamic_distance = min_lookahead_distance_ + speed * lookahead_speed_coeff_;
        // 确保不小于最小预瞄距离
        return std::max(dynamic_distance, min_lookahead_distance_);
    }



private:
    // 纯追踪核心参数（从参数服务器加载）
    double lookahead_distance_;  // 预瞄距离（m），默认5.0
    double wheelbase_;           // 车辆轴距（m），默认2.8（需根据实车调整）
    double max_steering_angle_;  // 最大转向角（rad），默认0.87（约50度）
    double min_path_points_;     // 最小有效路径点数，默认3（避免路径过短）

    // 新增：动态预瞄距离参数
    double min_lookahead_distance_;  // 最小预瞄距离（m）
    double lookahead_speed_coeff_;   // 速度系数（s），预瞄距离 = 最小距离 + 速度×系数    
    int curvature_point_num_;        // 计算曲率所用点数，默认5
    int curvature_space_num_;        // 间隔点数，默认5
};

} // namespace race_tracker

#endif // RACE_TRACKER_PURE_PURSUIT_CONTROLLER_H
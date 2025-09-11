#ifndef RACE_TRACKER_PID_CONTROLLER_H
#define RACE_TRACKER_PID_CONTROLLER_H

#include "race_tracker/controller_plugin_base.h"
#include <cmath>
#include <limits>
#include <algorithm>

namespace race_tracker {

/**
 * @brief PID纵向控制器（控制油门/刹车，跟踪目标速度）
 */
class PIDController : public ControllerPluginBase {
public:
    PIDController();
    ~PIDController() override = default;

    // 实现基类接口
    bool initialize(ros::NodeHandle& nh) override;
    void computeControl(
        const race_msgs::VehicleStatusConstPtr& vehicle_status,
        const race_msgs::PathConstPtr& path,
        race_msgs::Control* control_msg,
        const double dt) override;
    std::string getName() const override { return "PIDController"; }

private:
    /**
     * @brief 查找目标速度（车辆前方最近路径点的速度）
     * @param vehicle_status 车辆状态（位置）
     * @param path 局部路径
     * @return 目标速度（m/s），若路径无效则返回0
     */
    double findTargetVelocity(
        const race_msgs::VehicleStatusConstPtr& vehicle_status,
        const race_msgs::PathConstPtr& path);

private:
    // PID核心参数（从参数服务器加载）
    double kp_;                // 比例增益，默认0.6
    double ki_;                // 积分增益，默认0.05
    double kd_;                // 微分增益，默认0.1
    double max_throttle_;      // 最大油门输出（0~1），默认0.8
    double max_brake_;         // 最大刹车输出（0~1），默认0.8
    double integral_limit_;    // 积分限幅（防止积分饱和），默认2.0
    double speed_tolerance_;   // 速度误差容忍度（m/s），默认0.2（误差小于此值时不输出）
    double min_target_speed_;  // 最小目标速度（m/s），默认0.5（避免低速抖动）

    // PID状态变量（需持续更新）
    double last_error_;        // 上一次速度误差
    double integral_error_;    // 积分误差累积


    // 新增：resetPID函数声明
    void resetPID();  // 关键：补充函数声明，与源文件定义对应

};

} // namespace race_tracker

#endif // RACE_TRACKER_PID_CONTROLLER_H
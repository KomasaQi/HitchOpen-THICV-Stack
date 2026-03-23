#ifndef RACE_TRACKER_CONST_THROTTLE_CONTROLLER_H
#define RACE_TRACKER_CONST_THROTTLE_CONTROLLER_H

#include "race_tracker/controller_plugin_base.h"
#include <cmath>
#include <limits>
#include <algorithm>

namespace race_tracker {

/**
 * @brief 前馈 + 微量PID补偿 的纵向控制器
 *
 * 核心思想：
 * 1. 用标定好的稳态 throttle-speed 查表模型给出基础油门
 * 2. 用小范围 PID 仅做动态补偿
 * 3. 当减速需求较大时，优先减小油门，不够再输出刹车
 */
class ConstThrottleController : public ControllerPluginBase {
public:
    ConstThrottleController();
    ~ConstThrottleController() override = default;

    bool initialize(ros::NodeHandle& nh) override;

    void computeControl(
        const race_msgs::VehicleStatusConstPtr& vehicle_status,
        const race_msgs::PathConstPtr& path,
        race_msgs::Control* control_msg,
        const double dt,
        const race_msgs::Flag::ConstPtr& flag) override;

    std::string getName() const override { return "ConstThrottleController"; }

private:
    double findTargetVelocity(
        const race_msgs::VehicleStatusConstPtr& vehicle_status,
        const race_msgs::PathConstPtr& path);

    double speedToFeedforwardThrottle(double speed) const;
    double clamp(double val, double low, double high) const;
    void resetPID();

private:
    // PID补偿参数
    double kp_;
    double ki_;
    double kd_;

    // 输出限幅
    double max_throttle_;
    double max_brake_;
    double pid_comp_limit_;       // PID补偿油门限幅，默认±0.05
    double integral_limit_;
    double speed_tolerance_;
    double min_target_speed_;
    double predict_time_horizon_;

    // 前馈模型参数
    double ff_deadzone_throttle_; // 0.097
    double ff_speed_slope_;       // 26.3152
    double ff_speed_intercept_;   // -2.5058
    double ff_min_enable_speed_;  // 很低速时是否启用最小前馈油门，默认0.10 m/s

    // 刹车启用相关
    double brake_speed_error_threshold_; // 速度误差小于该值时只收油不刹车

    // PID状态
    double last_error_;
    double integral_error_;
};

} // namespace race_tracker

#endif // RACE_TRACKER_CONST_THROTTLE_CONTROLLER_H
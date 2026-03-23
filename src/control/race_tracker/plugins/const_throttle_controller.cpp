#include "race_tracker/const_throttle_controller.h"
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>

namespace race_tracker {

ConstThrottleController::ConstThrottleController()
    : kp_(0.15),
      ki_(0.02),
      kd_(0.0),
      max_throttle_(0.8),
      max_brake_(0.3),
      pid_comp_limit_(0.05),
      integral_limit_(0.3),
      speed_tolerance_(0.15),
      min_target_speed_(0.0),
      predict_time_horizon_(0.5),
      ff_deadzone_throttle_(0.097),
      ff_speed_slope_(26.3152),
      ff_speed_intercept_(-2.5058),
      ff_min_enable_speed_(0.10),
      brake_speed_error_threshold_(0.3),
      last_error_(0.0),
      integral_error_(0.0) {}

bool ConstThrottleController::initialize(ros::NodeHandle& nh) {
    ROS_INFO("[ConstThrottleController] 父NodeHandle命名空间: %s", nh.getNamespace().c_str());

    ros::NodeHandle nh_const_throttle(nh, "const_throttle_controller");
    ROS_INFO("[ConstThrottleController] 插件专属NodeHandle命名空间: %s",
             nh_const_throttle.getNamespace().c_str());

    // PID参数
    nh_const_throttle.param("kp", kp_, 0.15);
    nh_const_throttle.param("ki", ki_, 0.02);
    nh_const_throttle.param("kd", kd_, 0.0);

    // 输出限幅
    nh_const_throttle.param("max_throttle", max_throttle_, 0.8);
    nh_const_throttle.param("max_brake", max_brake_, 0.3);
    nh_const_throttle.param("pid_comp_limit", pid_comp_limit_, 0.05);
    nh_const_throttle.param("integral_limit", integral_limit_, 0.3);

    // 误差与目标速度参数
    nh_const_throttle.param("speed_tolerance", speed_tolerance_, 0.15);
    nh_const_throttle.param("min_target_speed", min_target_speed_, 0.0);
    nh_const_throttle.param("predict_time_horizon", predict_time_horizon_, 0.5);
    nh_const_throttle.param("brake_speed_error_threshold", brake_speed_error_threshold_, 0.3);

    // 前馈模型参数
    nh_const_throttle.param("ff_deadzone_throttle", ff_deadzone_throttle_, 0.097);
    nh_const_throttle.param("ff_speed_slope", ff_speed_slope_, 26.3152);
    nh_const_throttle.param("ff_speed_intercept", ff_speed_intercept_, -2.5058);
    nh_const_throttle.param("ff_min_enable_speed", ff_min_enable_speed_, 0.10);

    // 参数日志
    logParamLoad("kp", kp_, 0.15);
    logParamLoad("ki", ki_, 0.02);
    logParamLoad("kd", kd_, 0.0);
    logParamLoad("max_throttle", max_throttle_, 0.8);
    logParamLoad("max_brake", max_brake_, 0.3);
    logParamLoad("pid_comp_limit", pid_comp_limit_, 0.05);
    logParamLoad("integral_limit", integral_limit_, 0.3);
    logParamLoad("speed_tolerance", speed_tolerance_, 0.15);
    logParamLoad("min_target_speed", min_target_speed_, 0.0);
    logParamLoad("predict_time_horizon", predict_time_horizon_, 0.5);
    logParamLoad("brake_speed_error_threshold", brake_speed_error_threshold_, 0.3);
    logParamLoad("ff_deadzone_throttle", ff_deadzone_throttle_, 0.097);
    logParamLoad("ff_speed_slope", ff_speed_slope_, 26.3152);
    logParamLoad("ff_speed_intercept", ff_speed_intercept_, -2.5058);
    logParamLoad("ff_min_enable_speed", ff_min_enable_speed_, 0.10);

    // 合法性检查
    max_throttle_ = clamp(max_throttle_, 0.0, 1.0);
    max_brake_ = clamp(max_brake_, 0.0, 1.0);
    pid_comp_limit_ = clamp(pid_comp_limit_, 0.0, 0.5);

    if (ff_speed_slope_ <= 1e-6) {
        ROS_WARN("[%s] ff_speed_slope 无效，重置为 26.3152", getName().c_str());
        ff_speed_slope_ = 26.3152;
    }

    resetPID();
    return true;
}

void ConstThrottleController::computeControl(
    const race_msgs::VehicleStatusConstPtr& vehicle_status,
    const race_msgs::PathConstPtr& path,
    race_msgs::Control* control_msg,
    const double dt,
    const race_msgs::Flag::ConstPtr& flag) {

    if (!vehicle_status || !path || !control_msg) {
        ROS_ERROR("[%s] 输入数据为空", getName().c_str());
        if (control_msg) {
            control_msg->throttle = 0.0;
            control_msg->brake = 0.0;
        }
        return;
    }

    if (path->points.empty()) {
        ROS_WARN("[%s] 路径为空，输出0油门+适度刹车", getName().c_str());
        control_msg->throttle = 0.0;
        control_msg->brake = max_brake_;
        resetPID();
        return;
    }

    if (dt < 1e-4) {
        ROS_WARN_THROTTLE(1.0, "[%s] dt 过小: %.6f", getName().c_str(), dt);
        return;
    }

    const double current_speed = std::fabs(vehicle_status->vel.linear.x);
    double target_speed = findTargetVelocity(vehicle_status, path);
    target_speed = std::max(target_speed, min_target_speed_);

    const double error = target_speed - current_speed;

    // 1) 前馈油门
    const double ff_throttle = clamp(speedToFeedforwardThrottle(target_speed), 0.0, max_throttle_);

    // 2) PID小补偿
    // 小误差区仍然保留前馈，只是减弱/冻结积分
    if (std::fabs(error) < speed_tolerance_) {
        integral_error_ *= 0.9;  // 轻微泄放，避免长期残留
    } else {
        integral_error_ += error * dt;
        integral_error_ = clamp(integral_error_, -integral_limit_, integral_limit_);
    }

    const double p_term = kp_ * error;
    const double i_term = ki_ * integral_error_;
    const double d_term = kd_ * (error - last_error_) / dt;

    double pid_comp = p_term + i_term + d_term;
    pid_comp = clamp(pid_comp, -pid_comp_limit_, pid_comp_limit_);

    // 3) 油门优先 = 前馈 + 小PID
    double raw_throttle = ff_throttle + pid_comp;
    double throttle_cmd = clamp(raw_throttle, 0.0, max_throttle_);

    // 4) 刹车逻辑：
    //    - 优先通过减小油门来减速
    //    - 若误差明显为负，并且油门已经降到很低/为零，则再补刹车
    double brake_cmd = 0.0;
    if (error < -brake_speed_error_threshold_) {
        // 只有在明显超速时才刹车
        double brake_demand = -error;
        // 简单映射，可后续再单独调
        brake_cmd = clamp(brake_demand * 0.2, 0.0, max_brake_);

        // 若还有较大油门，就先尽量不刹车
        if (throttle_cmd > 0.02) {
            brake_cmd = 0.0;
        }
    }

    // 防止同时大油门大刹车
    if (brake_cmd > 1e-3) {
        throttle_cmd = 0.0;
    }

    control_msg->throttle = throttle_cmd;
    control_msg->brake = brake_cmd;
    control_msg->longitudinal.velocity = target_speed;
    control_msg->longitudinal.acceleration = error / dt;
    control_msg->control_mode = race_msgs::Control::THROTTLE_BRAKE_ONLY;

    last_error_ = error;

    // 调试建议打开
    /*
    ROS_INFO_THROTTLE(0.2,
        "[%s] v_cur=%.3f, v_tar=%.3f, err=%.3f, ff=%.3f, pid=%.3f, thr=%.3f, brk=%.3f",
        getName().c_str(), current_speed, target_speed, error,
        ff_throttle, pid_comp, throttle_cmd, brake_cmd);
    */
}

double ConstThrottleController::findTargetVelocity(
    const race_msgs::VehicleStatusConstPtr& vehicle_status,
    const race_msgs::PathConstPtr& path) {

    if (!vehicle_status || !path || path->points.empty()) {
        return 0.0;
    }

    const auto& vehicle_pos = vehicle_status->pose.position;

    double min_dist_sq = std::numeric_limits<double>::max();
    size_t closest_idx = 0;

    for (size_t i = 0; i < path->points.size(); ++i) {
        const auto& p = path->points[i].pose.position;
        const double dx = p.x - vehicle_pos.x;
        const double dy = p.y - vehicle_pos.y;
        const double dist_sq = dx * dx + dy * dy;
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            closest_idx = i;
        }
    }

    const double predicted_distance =
        std::max(0.0, std::fabs(vehicle_status->vel.linear.x) * predict_time_horizon_);

    size_t target_idx = closest_idx;
    double accumulated_distance = 0.0;

    for (size_t i = closest_idx; i + 1 < path->points.size(); ++i) {
        const auto& p0 = path->points[i].pose.position;
        const auto& p1 = path->points[i + 1].pose.position;

        const double dx = p1.x - p0.x;
        const double dy = p1.y - p0.y;
        accumulated_distance += std::sqrt(dx * dx + dy * dy);

        target_idx = i + 1;
        if (accumulated_distance >= predicted_distance) {
            break;
        }
    }

    return path->points[target_idx].velocity;
}

double ConstThrottleController::speedToFeedforwardThrottle(double speed) const {
    if (speed <= 0.0) {
        return 0.0;
    }

    // 对应你的 speed2throttle 模型
    if (speed < ff_min_enable_speed_) {
        return ff_deadzone_throttle_;
    }

    return (speed - ff_speed_intercept_) / ff_speed_slope_;
    // 注意：因为 intercept = -2.5058
    // 所以等价于 (speed + 2.5058) / 26.3152
}

double ConstThrottleController::clamp(double val, double low, double high) const {
    return std::max(low, std::min(val, high));
}

void ConstThrottleController::resetPID() {
    last_error_ = 0.0;
    integral_error_ = 0.0;
}

} // namespace race_tracker

PLUGINLIB_EXPORT_CLASS(race_tracker::ConstThrottleController,
                       race_tracker::ControllerPluginBase)
#include "race_tracker/pid_controller.h"
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>

namespace race_tracker {

PIDController::PIDController()
    : kp_(0.6),
      ki_(0.05),
      kd_(0.1),
      max_throttle_(0.8),
      max_brake_(0.8),
      integral_limit_(2.0),
      speed_tolerance_(0.2),
      min_target_speed_(0.5),
      last_error_(0.0),
      integral_error_(0.0) {}

bool PIDController::initialize(ros::NodeHandle& nh) {
    // 从参数服务器加载参数（未设置则用默认值）
    nh.param("pid/kp", kp_, 0.6);
    nh.param("pid/ki", ki_, 0.05);
    nh.param("pid/kd", kd_, 0.1);
    nh.param("pid/max_throttle", max_throttle_, 0.8);
    nh.param("pid/max_brake", max_brake_, 0.8);
    nh.param("pid/integral_limit", integral_limit_, 2.0);
    nh.param("pid/speed_tolerance", speed_tolerance_, 0.2);
    nh.param("pid/min_target_speed", min_target_speed_, 0.5);

    // 打印参数加载日志
    logParamLoad("pid/kp", kp_, 0.6);
    logParamLoad("pid/ki", ki_, 0.05);
    logParamLoad("pid/kd", kd_, 0.1);
    logParamLoad("pid/max_throttle", max_throttle_, 0.8);
    logParamLoad("pid/max_brake", max_brake_, 0.8);
    logParamLoad("pid/integral_limit", integral_limit_, 2.0);

    // 检查参数有效性
    if (max_throttle_ < 0.1 || max_throttle_ > 1.0) {
        ROS_WARN("[%s] 最大油门无效（%.2f），重置为0.8", getName().c_str(), max_throttle_);
        max_throttle_ = 0.8;
    }
    if (max_brake_ < 0.1 || max_brake_ > 1.0) {
        ROS_WARN("[%s] 最大刹车无效（%.2f），重置为0.8", getName().c_str(), max_brake_);
        max_brake_ = 0.8;
    }

    return true;
}

void PIDController::computeControl(
    const race_msgs::VehicleStatusConstPtr& vehicle_status,
    const race_msgs::PathConstPtr& path,
    race_msgs::Control* control_msg,
    const double dt) {

    // 1. 检查输入数据有效性
    if (!vehicle_status || !path || !control_msg) {
        ROS_ERROR("[%s] 输入数据为空（车辆状态/路径/控制指令）", getName().c_str());
        control_msg->throttle = 0.0;
        control_msg->brake = 0.0;
        return;
    }
    if (path->points.empty()) {
        ROS_WARN("[%s] 路径为空，输出0油门+最大刹车", getName().c_str());
        control_msg->throttle = 0.0;
        control_msg->brake = max_brake_;
        resetPID(); // 重置PID状态
        return;
    }
    if (vehicle_status->emergency || vehicle_status->hand_brake) {
        ROS_WARN("[%s] 紧急状态/手刹开启，输出0油门+最大刹车", getName().c_str());
        control_msg->throttle = 0.0;
        control_msg->brake = max_brake_;
        resetPID(); // 重置PID状态
        return;
    }
    if (dt < 1e-6) { // 避免dt过小导致微分项爆炸
        ROS_WARN("[%s] 控制周期过短（dt=%.6f），跳过本次计算", getName().c_str(), dt);
        return;
    }

    // 2. 获取当前速度和目标速度
    double current_speed = std::fabs(vehicle_status->vel.linear.x); // 取绝对值（前进/倒车通用）
    double target_speed = findTargetVelocity(vehicle_status, path);
    target_speed = std::max(target_speed, min_target_speed_); // 限制最小目标速度

    // 3. 计算速度误差
    double error = target_speed - current_speed;

    // 4. 若误差小于容忍度，不输出控制（避免抖动）
    if (std::fabs(error) < speed_tolerance_) {
        control_msg->throttle = 0.0;
        control_msg->brake = 0.0;
        // ROS_DEBUG("[%s] 速度误差过小（%.2f < %.2f），不输出控制",
        //          getName().c_str(), std::fabs(error), speed_tolerance_);
        return;
    }

    // 5. PID计算（比例+积分+微分）
    // 比例项
    double p_term = kp_ * error;
    // 积分项（带限幅，防止饱和）
    integral_error_ += error * dt;
    // 替换为
    if (integral_error_ > integral_limit_) {
        integral_error_ = integral_limit_;
    } else if (integral_error_ < -integral_limit_) {
        integral_error_ = -integral_limit_;
    }
    double i_term = ki_ * integral_error_;
    // 微分项（用误差变化率，避免测量噪声）
    double d_term = kd_ * (error - last_error_) / dt;
    // 总输出
    double pid_output = p_term + i_term + d_term;

    // 6. 输出映射到油门/刹车
    if (pid_output > 0) { // 需加速：输出油门
        control_msg->throttle = std::min(pid_output, max_throttle_);
        control_msg->brake = 0.0;
    } else { // 需减速：输出刹车
        control_msg->throttle = 0.0;
        control_msg->brake = std::min(-pid_output, max_brake_);
    }

    // 7. 更新PID状态（保存当前误差）
    last_error_ = error;

    // 8. 赋值纵向控制其他字段
    control_msg->longitudinal.velocity = target_speed;
    control_msg->longitudinal.acceleration = error / dt; // 简化加速度计算
    control_msg->control_mode = race_msgs::Control::THROTTLE_BRAKE_ONLY; // 油门刹车模式

    // 调试日志（可选）
    // ROS_DEBUG("[%s] 目标速度: %.2f | 当前速度: %.2f | 油门: %.2f | 刹车: %.2f",
    //          getName().c_str(), target_speed, current_speed, control_msg->throttle, control_msg->brake);
}

double PIDController::findTargetVelocity(
    const race_msgs::VehicleStatusConstPtr& vehicle_status,
    const race_msgs::PathConstPtr& path) {

    const auto& vehicle_pos = vehicle_status->pose.position;
    double min_dist_to_veh = std::numeric_limits<double>::max();
    size_t closest_idx = 0;

    // 找到车辆当前位置最近的路径点（取其速度为目标速度）
    for (size_t i = 0; i < path->points.size(); ++i) {
        const auto& path_pos = path->points[i].pose.position;
        double dist_sq = (path_pos.x - vehicle_pos.x) * (path_pos.x - vehicle_pos.x) +
                        (path_pos.y - vehicle_pos.y) * (path_pos.y - vehicle_pos.y);
        if (dist_sq < min_dist_to_veh) {
            min_dist_to_veh = dist_sq;
            closest_idx = i;
        }
    }

    // 若最近点是最后一个，返回其速度；否则返回下一个点（避免滞后）
    size_t target_idx = (closest_idx + 1) < path->points.size() ? closest_idx + 1 : closest_idx;
    return path->points[target_idx].velocity;
}

void PIDController::resetPID() {
    last_error_ = 0.0;
    integral_error_ = 0.0;
}

} // namespace race_tracker

// 插件注册：让pluginlib能找到该类
PLUGINLIB_EXPORT_CLASS(race_tracker::PIDController, race_tracker::ControllerPluginBase)
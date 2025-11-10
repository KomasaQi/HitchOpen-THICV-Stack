#include "race_tracker/pid_tracker.h"
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>

namespace race_tracker {

PIDTracker::PIDTracker()
    : min_lookahead_distance_(3.0),
      lookahead_speed_coeff_(0.5),
      current_lookahead_(3.0),
      min_path_points_(3),
      kp_(1.0),
      ki_(0.1),
      kd_(0.2),
      integral_limit_(1.0),
      max_steering_angle_(0.87),
      lateral_error_weight_(0.7),
      heading_error_weight_(0.3),
      last_error_(0.0),
      last_steering_angle_(0.0),
      integral_error_(0.0) {}

bool PIDTracker::initialize(ros::NodeHandle& nh) {
    ROS_INFO("[PIDTracker] 父NodeHandle命名空间: %s", nh.getNamespace().c_str());
    ros::NodeHandle nh_pid(nh, "pid_tracker");
    ROS_INFO("[PIDTracker] 插件专属NodeHandle命名空间: %s", nh_pid.getNamespace().c_str());

    // 加载预瞄参数
    nh_pid.param("min_lookahead_distance", min_lookahead_distance_, 3.0);
    nh_pid.param("lookahead_speed_coeff", lookahead_speed_coeff_, 0.5);
    nh_pid.param("min_path_points", min_path_points_, 3.0);

    // 加载PID参数
    nh_pid.param("kp", kp_, 1.0);
    nh_pid.param("ki", ki_, 0.1);
    nh_pid.param("kd", kd_, 0.2);
    nh_pid.param("integral_limit", integral_limit_, 1.0);
    nh_pid.param("max_steering_angle", max_steering_angle_, 0.87);

    // 加载误差权重参数
    nh_pid.param("lateral_error_weight", lateral_error_weight_, 0.7);
    nh_pid.param("heading_error_weight", heading_error_weight_, 0.3);

    // 打印参数日志
    logParamLoad("min_lookahead_distance", min_lookahead_distance_, 3.0);
    logParamLoad("lookahead_speed_coeff", lookahead_speed_coeff_, 0.5);
    logParamLoad("min_path_points", min_path_points_, 3.0);
    logParamLoad("kp", kp_, 1.0);
    logParamLoad("ki", ki_, 0.1);
    logParamLoad("kd", kd_, 0.2);
    logParamLoad("integral_limit", integral_limit_, 1.0);
    logParamLoad("max_steering_angle", max_steering_angle_, 0.87);
    logParamLoad("lateral_error_weight", lateral_error_weight_, 0.7);
    logParamLoad("heading_error_weight", heading_error_weight_, 0.3);

    // 参数有效性检查
    if (min_lookahead_distance_ < 1.0) {
        ROS_WARN("[%s] 最小预瞄距离过小，重置为1.0m", getName().c_str());
        min_lookahead_distance_ = 1.0;
    }
    if (lookahead_speed_coeff_ < 0.1) {
        ROS_WARN("[%s] 预瞄速度系数过小，重置为0.1", getName().c_str());
        lookahead_speed_coeff_ = 0.1;
    }
    if (integral_limit_ < 0.1) {
        ROS_WARN("[%s] 积分限幅过小，重置为0.1", getName().c_str());
        integral_limit_ = 0.1;
    }
    if (lateral_error_weight_ < 0 || lateral_error_weight_ > 100) {
        ROS_WARN("[%s] 横向误差权重无效，重置为0.7", getName().c_str());
        lateral_error_weight_ = 0.7;
    }
    if (heading_error_weight_ < 0 || heading_error_weight_ > 100) {
        ROS_WARN("[%s] 航向误差权重无效，重置为0.3", getName().c_str());
        heading_error_weight_ = 0.3;
    }

    current_lookahead_ = min_lookahead_distance_;
    return true;
}

void PIDTracker::computeControl(
    const race_msgs::VehicleStatusConstPtr& vehicle_status,
    const race_msgs::PathConstPtr& path,
    race_msgs::Control* control_msg,
    const double dt,
    const race_msgs::Flag::ConstPtr& flag) {

    // 输入有效性检查
    if (!vehicle_status || !path || !control_msg) {
        ROS_ERROR("[%s] 输入数据为空", getName().c_str());
        control_msg->lateral.steering_angle = 0.0;
        return;
    }
    if (path->points.size() < min_path_points_) {
        ROS_WARN("[%s] 路径点数不足，输出0转向角", getName().c_str());
        control_msg->lateral.steering_angle = 0.0;
        resetPID();
        return;
    }
    if (vehicle_status->emergency) {
        ROS_WARN("[%s] 紧急状态，输出0转向角", getName().c_str());
        control_msg->lateral.steering_angle = 0.0;
        resetPID();
        return;
    }
    if (dt < 1e-6) {
        ROS_WARN("[%s] 控制周期过短，跳过计算", getName().c_str());
        return;
    }

    // 1. 查找预瞄点
    geometry_msgs::Point lookahead_point = findLookaheadPoint(vehicle_status, path);

    // 2. 转换到车辆坐标系
    geometry_msgs::Point veh_lookahead = transformToVehicleFrame(lookahead_point, vehicle_status->pose);
    double lateral_error = veh_lookahead.y;  // 横向误差（车辆坐标系y轴）

    // 3. 计算最近路径点索引
    const auto& vehicle_pos = vehicle_status->pose.position;
    double min_dist_sq = std::numeric_limits<double>::max();
    size_t closest_idx = 0;
    for (size_t i = 0; i < path->points.size(); ++i) {
        const auto& pos = path->points[i].pose.position;
        double dist_sq = (pos.x - vehicle_pos.x) * (pos.x - vehicle_pos.x) +
                        (pos.y - vehicle_pos.y) * (pos.y - vehicle_pos.y);
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            closest_idx = i;
        }
    }

    // 4. 计算航向误差
    double heading_error = calculateHeadingError(vehicle_status->pose, path, closest_idx);

    // 5. 计算综合误差（带权重）
    double combined_error = lateral_error * lateral_error_weight_ + 
                           heading_error * heading_error_weight_;

    // 6. PID控制计算
    double p_term = kp_ * combined_error;
    integral_error_ += combined_error * dt;
    if (integral_error_ > integral_limit_) {
        integral_error_ = integral_limit_;
    } else if (integral_error_ < -integral_limit_) {
        integral_error_ = -integral_limit_;
    }

    double i_term = ki_ * integral_error_;
    double d_term = kd_ * (combined_error - last_error_) / dt;
    double steering_angle_d = p_term + i_term + d_term;
    if (steering_angle_d > 0.015){
        steering_angle_d = 0.015;
    } else if (steering_angle_d < -0.015){
        steering_angle_d = -0.015;
    }
    double steering_angle = last_steering_angle_ + steering_angle_d;
    last_steering_angle_ = steering_angle;

    // 7. 限制转向角
    if (steering_angle > max_steering_angle_) {
        steering_angle = max_steering_angle_;
    } else if (steering_angle < -max_steering_angle_) {
        steering_angle = -max_steering_angle_;
    }

    // 8. 更新控制指令
    control_msg->lateral.steering_angle = steering_angle;
    control_msg->lateral.steering_angle_velocity = steering_angle / dt;
    control_msg->steering_mode = race_msgs::Control::FRONT_STEERING_MODE;

    // 9. 保存当前误差
    last_error_ = combined_error;

    ROS_INFO("[%s] 横向误差: %.2f, 航向误差: %.2f, 综合误差: %.2f, 转向角: %.2f rad",
             getName().c_str(), lateral_error, heading_error, combined_error, steering_angle);
}

geometry_msgs::Point PIDTracker::findLookaheadPoint(
    const race_msgs::VehicleStatusConstPtr& vehicle_status,
    const race_msgs::PathConstPtr& path) {

    // 计算动态预瞄距离
    double current_speed = std::fabs(vehicle_status->vel.linear.x);
    current_lookahead_ = calculateDynamicLookahead(current_speed);
    ROS_INFO("[%s] 动态预瞄距离: %.2f m（速度: %.2f m/s）",
             getName().c_str(), current_lookahead_, current_speed);

    const auto& vehicle_pos = vehicle_status->pose.position;
    double lookahead_sq = current_lookahead_ * current_lookahead_;

    // 找到最近点
    double min_dist_sq = std::numeric_limits<double>::max();
    size_t closest_idx = 0;
    for (size_t i = 0; i < path->points.size(); ++i) {
        const auto& pos = path->points[i].pose.position;
        double dist_sq = (pos.x - vehicle_pos.x) * (pos.x - vehicle_pos.x) +
                        (pos.y - vehicle_pos.y) * (pos.y - vehicle_pos.y);
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            closest_idx = i;
        }
    }

    // 从最近点向后找满足预瞄距离的点
    for (size_t i = closest_idx; i < path->points.size(); ++i) {
        const auto& pos = path->points[i].pose.position;
        double dist_sq = (pos.x - vehicle_pos.x) * (pos.x - vehicle_pos.x) +
                        (pos.y - vehicle_pos.y) * (pos.y - vehicle_pos.y);
        if (dist_sq >= lookahead_sq) {
            return pos;
        }
    }

    // 未找到则返回最后一个点
    ROS_WARN("[%s] 未找到满足预瞄距离的点，使用路径终点", getName().c_str());
    return path->points.back().pose.position;
}

geometry_msgs::Point PIDTracker::transformToVehicleFrame(
    const geometry_msgs::Point& world_point,
    const geometry_msgs::Pose& vehicle_pose) {

    double dx = world_point.x - vehicle_pose.position.x;
    double dy = world_point.y - vehicle_pose.position.y;
    double yaw = tf::getYaw(vehicle_pose.orientation);

    geometry_msgs::Point veh_point;
    veh_point.x = dx * std::cos(yaw) + dy * std::sin(yaw);
    veh_point.y = -dx * std::sin(yaw) + dy * std::cos(yaw);
    veh_point.z = 0.0;
    return veh_point;
}

double PIDTracker::calculateDynamicLookahead(double current_speed) {
    double speed = std::fabs(current_speed);
    double dynamic_distance = min_lookahead_distance_ + speed * lookahead_speed_coeff_;
    return std::max(dynamic_distance, min_lookahead_distance_);
}

double PIDTracker::calculateHeadingError(
    const geometry_msgs::Pose& vehicle_pose,
    const race_msgs::PathConstPtr& path,
    size_t closest_idx) {

    // 车辆当前航向角
    double vehicle_yaw = tf::getYaw(vehicle_pose.orientation);

    // 路径方向（取最近点和下一个点的方向）
    size_t next_idx = std::min(closest_idx + 1, path->points.size() - 1);
    const auto& curr_path_pose = path->points[closest_idx].pose;
    const auto& next_path_pose = path->points[next_idx].pose;

    double path_dx = next_path_pose.position.x - curr_path_pose.position.x;
    double path_dy = next_path_pose.position.y - curr_path_pose.position.y;
    double path_yaw = std::atan2(path_dy, path_dx);

    // 计算航向误差（归一化到[-π, π]）
    double error = path_yaw - vehicle_yaw;
    error = std::atan2(std::sin(error), std::cos(error));  // 角度归一化
    return error;
}

void PIDTracker::resetPID() {
    last_error_ = 0.0;
    integral_error_ = 0.0;
}

} // namespace race_tracker

PLUGINLIB_EXPORT_CLASS(race_tracker::PIDTracker, race_tracker::ControllerPluginBase)
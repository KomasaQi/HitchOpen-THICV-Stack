#include "race_tracker/pure_pursuit_controller.h"
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>

namespace race_tracker {

PurePursuitController::PurePursuitController() 
    : wheelbase_(2.8),
      max_steering_angle_(0.87),
      min_path_points_(3),
      min_lookahead_distance_(3.0),
      amplify_coeff_(1.0),
      lookahead_speed_coeff_(0.5) {}

bool PurePursuitController::initialize(ros::NodeHandle& nh) {
    // 1. 打印原命名空间（确认基础路径）
    ROS_INFO("[PurePursuitController] 父NodeHandle命名空间: %s", nh.getNamespace().c_str());
    // 2. 创建插件专属的子NodeHandle（匹配Launch中的ns="pure_pursuit"）
    ros::NodeHandle nh_pursuit(nh, "pure_pursuit");  // 子命名空间：父ns + "/pure_pursuit"
    ROS_INFO("[PurePursuitController] 插件专属NodeHandle命名空间: %s", nh_pursuit.getNamespace().c_str());

    // 从参数服务器加载参数（未设置则用默认值）
    nh_pursuit.param("wheelbase", wheelbase_, 2.8);
    nh_pursuit.param("max_steering_angle", max_steering_angle_, 0.87);
    nh_pursuit.param("min_path_points", min_path_points_, 3.0);
    // 新增：加载动态预瞄参数（默认最小3m，系数0.5s）
    nh_pursuit.param("min_lookahead_distance", min_lookahead_distance_, 3.0);
    nh_pursuit.param("lookahead_speed_coeff", lookahead_speed_coeff_, 0.5);

    nh_pursuit.param("amplify_coeff", amplify_coeff_, 1.0);

    // 打印参数加载日志
    logParamLoad("wheelbase", wheelbase_, 2.8);
    logParamLoad("max_steering_angle", max_steering_angle_, 0.87);
    logParamLoad("min_path_points", min_path_points_, 3);
    // 打印新增参数日志
    logParamLoad("min_lookahead_distance", min_lookahead_distance_, 3.0);
    logParamLoad("lookahead_speed_coeff", lookahead_speed_coeff_, 0.5);

    // 检查参数有效性
    if (wheelbase_ < 1.0) {
        ROS_WARN("[%s] 轴距过小（<1m），重置为2.8m", getName().c_str());
        wheelbase_ = 2.8;
    }
    // 新增参数有效性检查
    if (min_lookahead_distance_ < 1.0) {
        ROS_WARN("[%s] 最小预瞄距离过小（<1m），重置为1.0m", getName().c_str());
        min_lookahead_distance_ = 1.0;
    }
    if (lookahead_speed_coeff_ < 0.1) {
        ROS_WARN("[%s] 速度系数过小（<0.1），重置为0.1", getName().c_str());
        lookahead_speed_coeff_ = 0.1;
    }
    lookahead_distance_ = min_lookahead_distance_;
    return true;
}

void PurePursuitController::computeControl(
    const race_msgs::VehicleStatusConstPtr& vehicle_status,
    const race_msgs::PathConstPtr& path,
    race_msgs::Control* control_msg,
    const double dt,
    const race_msgs::Flag::ConstPtr& flag) {

    // 1. 检查输入数据有效性
    if (!vehicle_status || !path || !control_msg) {
        ROS_ERROR("[%s] 输入数据为空（车辆状态/路径/控制指令）", getName().c_str());
        control_msg->lateral.steering_angle = 0.0;
        return;
    }
    if (path->points.size() < min_path_points_) {
        ROS_WARN("[%s] 路径点数不足（%lu < %f），输出0转向角", 
                 getName().c_str(), path->points.size(), static_cast<double>(min_path_points_));
        control_msg->lateral.steering_angle = 0.0;
        return;
    }
    if (vehicle_status->emergency) {
        ROS_WARN("[%s] 车辆处于紧急状态，输出0转向角", getName().c_str());
        control_msg->lateral.steering_angle = 0.0;
        return;
    }


    // 2. 查找预瞄点
    geometry_msgs::Point lookahead_point = findLookaheadPoint(vehicle_status, path);

    // 3. 转换到车辆坐标系
    geometry_msgs::Point veh_lookahead = transformToVehicleFrame(lookahead_point, vehicle_status->pose);

    // 4. 纯追踪核心公式：计算转向角（参考《自动驾驶汽车技术》纯追踪模型）
    double L = lookahead_distance_;          // 预瞄距离
    double y = veh_lookahead.y;              // 车辆坐标系下预瞄点y坐标（横向偏差）
    double steering_angle = 0.0;

    if (std::fabs(L) > 1e-6) { // 避免除以0
        steering_angle = std::atan2(2 * wheelbase_ * y, L * L);
    }

    // 5. 限制转向角在安全范围（防止过冲）
    // 替换为
    if (steering_angle > max_steering_angle_) {
        steering_angle = max_steering_angle_;
    } else if (steering_angle < -max_steering_angle_) {
        steering_angle = -max_steering_angle_;
    }

    // 6. 赋值到控制指令（横向控制相关字段）
    control_msg->lateral.steering_angle = steering_angle*amplify_coeff_;
    control_msg->lateral.steering_angle_velocity = steering_angle / dt; // 转向角速度（简化计算）
    control_msg->steering_mode = race_msgs::Control::FRONT_STEERING_MODE; // 默认前轮转向

    // 调试日志（可选，发布时可注释）
    // ROS_DEBUG("[%s] 预瞄点y: %.2f | 转向角: %.2f rad (%.2f deg)",
    //          getName().c_str(), y, steering_angle, steering_angle * 180 / M_PI);
}

geometry_msgs::Point PurePursuitController::findLookaheadPoint(
    const race_msgs::VehicleStatusConstPtr& vehicle_status,
    const race_msgs::PathConstPtr& path) {

    // 新增：获取当前速度并计算动态预瞄距离
    double current_speed = vehicle_status->vel.linear.x;
    lookahead_distance_ = calculateDynamicLookahead(current_speed);
    ROS_INFO("[%s] 动态预瞄距离: %.2f m（速度: %.2f m/s）",
             getName().c_str(), lookahead_distance_, current_speed);

    const auto& vehicle_pos = vehicle_status->pose.position;
    double min_dist_to_veh = std::numeric_limits<double>::max();
    size_t closest_idx = 0;

    // 第一步：找到车辆当前位置最近的路径点（作为起点）
    for (size_t i = 0; i < path->points.size(); ++i) {
        const auto& path_pos = path->points[i].pose.position;
        double dist_sq = (path_pos.x - vehicle_pos.x) * (path_pos.x - vehicle_pos.x) +
                        (path_pos.y - vehicle_pos.y) * (path_pos.y - vehicle_pos.y);
        if (dist_sq < min_dist_to_veh) {
            min_dist_to_veh = dist_sq;
            closest_idx = i;
        }
    }

    // 第二步：从最近点向后找第一个满足预瞄距离的点（确保是前方点）
    double lookahead_sq = lookahead_distance_ * lookahead_distance_;
    for (size_t i = closest_idx; i < path->points.size(); ++i) {
        const auto& path_pos = path->points[i].pose.position;
        double dist_sq = (path_pos.x - vehicle_pos.x) * (path_pos.x - vehicle_pos.x) +
                        (path_pos.y - vehicle_pos.y) * (path_pos.y - vehicle_pos.y);
        // 若当前点距离超过预瞄距离，且在车辆前方（x方向为正），则选为预瞄点
        if (dist_sq >= lookahead_sq) {
            return path_pos;
        }
    }

    // 第三步：若未找到（如路径末端），返回路径最后一个点
    ROS_WARN("[%s] 未找到满足预瞄距离的点，使用路径最后一个点", getName().c_str());
    return path->points.back().pose.position;
}

geometry_msgs::Point PurePursuitController::transformToVehicleFrame(
    const geometry_msgs::Point& world_point,
    const geometry_msgs::Pose& vehicle_pose) {

    // 1. 计算世界坐标系下车辆到预瞄点的向量
    double dx = world_point.x - vehicle_pose.position.x;
    double dy = world_point.y - vehicle_pose.position.y;

    // 2. 从四元数获取车辆偏航角（yaw）
    double yaw = tf::getYaw(vehicle_pose.orientation);

    // 3. 旋转矩阵：世界坐标系 -> 车辆坐标系（x向前，y向左）
    geometry_msgs::Point veh_point;
    veh_point.x = dx * std::cos(yaw) + dy * std::sin(yaw); // 纵向距离
    veh_point.y = -dx * std::sin(yaw) + dy * std::cos(yaw); // 横向距离
    veh_point.z = 0.0; // 忽略z轴（平面运动）

    return veh_point;
}

} // namespace race_tracker

// 插件注册：让pluginlib能找到该类
PLUGINLIB_EXPORT_CLASS(race_tracker::PurePursuitController, race_tracker::ControllerPluginBase)
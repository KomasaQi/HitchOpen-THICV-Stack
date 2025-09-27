#ifndef RACE_TRACKER_ANTI_ROLLOVER_CONTROLLER_H
#define RACE_TRACKER_ANTI_ROLLOVER_CONTROLLER_H

#include <casadi/casadi.hpp>
#include <ros/ros.h>
#include <vector>
#include <tf/transform_datatypes.h>
#include "race_tracker/controller_plugin_base.h"
#include <race_msgs/Control.h>
#include <race_msgs/VehicleStatus.h>
#include <race_msgs/Path.h>
#include <memory>


namespace race_tracker {

class AntiRolloverController : public ControllerPluginBase {
public:
    AntiRolloverController() = default;
    ~AntiRolloverController() override = default;

    bool initialize(ros::NodeHandle& nh) override;
    void computeControl(
        const race_msgs::VehicleStatusConstPtr& vehicle_status,
        const race_msgs::PathConstPtr& path,
        race_msgs::Control* control_msg,
        const double dt,
        const race_msgs::Flag::ConstPtr& flag) override;
    std::string getName() const override { return "AntiRolloverController"; }

private:
    // 辅助函数
    double quaternion_to_yaw(const geometry_msgs::Quaternion& q);
    int find_nearest_path_point(const double x0, const double y0, const race_msgs::Path& path);
    std::vector<double> calculate_cumulative_distance(const race_msgs::Path& path, int start_idx);
    std::vector<double> linear_interpolate(const std::vector<double>& s_original, 
                                         const std::vector<double>& val_original, 
                                         const std::vector<double>& s_target);
    casadi::DM interpolate_path_segment(const race_msgs::Path& path, const std::vector<double>& cum_dist, 
                                      int start_idx, int end_idx, int n_waypoints, double yaw0);
    casadi::DM process_race_path(const race_msgs::Path& input_path, const std::vector<double>& current_state);

    // NMPC求解函数
    bool solveNMPC(const std::vector<double>& current_state, const casadi::DM& waypoints,
                  std::vector<double>& control_output);

    // 车辆状态转换
    std::vector<double> vehicleStatusToStateVector(const race_msgs::VehicleStatus& status);

    // 控制器参数
    int nx_;                // 状态维度 [x, y, vx, theta, delta1, delta2]
    int nu_;                // 控制量维度 [delta1_des, delta2_des]
    int N_;                 // NMPC预测步长
    int Nc_;                // 稀疏控制量步数
    double T_d1_;           // 前轴转向动态时间常数
    double T_d2_;           // 后轴转向动态时间常数
    double dt_;             // 采样时间 (s)
    double L_;              // 车辆轴距 (m)
    double g_;              // 重力加速度 (m/s²)
    double a_max_;          // 预设最大加速度 (m/s²)
    int n_waypoints_;       // 目标参考路点数量

    // 控制量边界
    double delta1_min_;     // 前轴最小转向角
    double delta1_max_;     // 前轴最大转向角
    double delta2_min_;     // 后轴最小转向角
    double delta2_max_;     // 后轴最大转向角

    // 代价函数权重
    double w_pos_;          // 位置跟踪权重
    double w_theta_;        // 航向跟踪权重
    double w_v_;            // 速度跟踪权重
    double w_ax_;           // 加速度平滑权重
    double w_delta1_;       // 前轴转向角平滑权重
    double w_delta2_;       // 后轴转向角平滑权重
    double w_term_pos_;     // 终端位置权重
    double w_term_theta_;   // 终端航向权重
    double w_term_v_;       // 终端速度权重
    double w_delta_cmd1_;   // 前轴转向角指令权重
    double w_delta_cmd2_;   // 后轴转向角指令权重


    // NMPC求解器相关
    std::unique_ptr<casadi::OptiSol> sol_prev_;  // 原：casadi::OptiSol sol_prev_;
    casadi::MX X_;          // 状态序列
    casadi::MX U_sparse_;   // 稀疏控制量
    casadi::MX x0_;         // 初始状态参数
    casadi::MX waypoints_;  // 参考路点参数
    casadi::Function f_func_; // 动力学模型函数
    casadi::Opti opti_;     // NMPC优化器
    std::vector<double> last_control_output_; // 上一次的控制输出
    bool has_prev_sol_;     // 是否有前一次求解结果

    // 稀疏控制量分布
    std::vector<int> steps_per_control_;
};

} // namespace race_tracker

#endif // RACE_TRACKER_ANTI_ROLLOVER_CONTROLLER_H
    
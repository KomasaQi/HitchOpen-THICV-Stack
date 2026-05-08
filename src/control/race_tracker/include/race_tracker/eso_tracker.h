#ifndef RACE_TRACKER_ESO_TRACKER_H
#define RACE_TRACKER_ESO_TRACKER_H

#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <memory>
#include <tf/transform_datatypes.h>

// ROS 插件和消息相关头文件
#include "race_tracker/controller_plugin_base.h"
#include <race_msgs/Control.h>
#include <race_msgs/VehicleStatus.h>
#include <race_msgs/Path.h>
#include <race_msgs/Flag.h>

namespace race_tracker {

// 监督层与纯跟踪参数
struct SupervisorParams {
    double startup_time;
    double blend_speed_low;
    double blend_speed_high;
    double lookahead_distance;
};

// NMPC参数 
struct NMPCParams {
    // --- 基础配置 ---
    double m;
    double Iz;
    double lf;
    double lr;
    double T_lag;
    double dt;
    int N;
    int Nc;
    int nx;
    int nu;
    
    // --- 控制约束 ---
    double delta_max;
    double delta_min;
    double delta_c_max;

    // --- 轮胎参数 (含辨识上下限) ---
    double Cf;
    double Cr;
    double Cf_min;
    double Cf_max;
    double Cr_min;
    double Cr_max;

    // --- 积分器 ---
    double integration_grade;

    // --- 代价函数权重 ---
    double Q_x, Q_y, Q_theta;
    double Q_vy, Q_r, Q_delta;
    double R;
    double dR;

    Eigen::Matrix<double, 6, 6> Q;

    NMPCParams(); // 声明构造函数，在cpp中实现矩阵初始化
    void updateQMatrix();
};

// NMPC求解器结构 
struct NMPSolver {
    casadi::Opti opti;
    casadi::MX X;
    casadi::MX U_sparse;
    casadi::MX P_x0;
    casadi::MX P_waypoints;
    casadi::MX P_vx;
    casadi::MX P_u_prev;
    casadi::MX P_h_hat;
    casadi::MX P_dyn_params;
    std::unique_ptr<casadi::OptiSol> sol_prev; 
    bool has_prev_sol;
};

// 核心控制器类继承自 ControllerPluginBase
class ESOTracker : public ControllerPluginBase {
public:
    ESOTracker();
    ~ESOTracker() override = default;

    // --- 核心 ROS 插件重载函数 ---
    bool initialize(ros::NodeHandle& nh) override;
    
    void computeControl(
        const race_msgs::VehicleStatusConstPtr& vehicle_status,
        const race_msgs::PathConstPtr& path,
        race_msgs::Control* control_msg,
        const double dt,
        const race_msgs::Flag::ConstPtr& flag) override;

    std::string getName() const override { return "ESOTracker"; }

private:
    // --- 算法核心函数  ---
    void buildNMPSolver();

    casadi::MX vehicleDynamicsModel(const casadi::MX& state, const casadi::MX& cmd_delta,
                                    const casadi::MX& vx, const casadi::MX& h_dist,
                                    const casadi::MX& dyn_params);

    bool solveNMPC(const std::vector<double>& current_state, const casadi::DM& waypoints,
                   std::vector<double>& control_output);

    
    void ukfEstimateVy(double curr_vx, double curr_delta, double curr_ay, double curr_r, double dt);
    
    void rlsIdentifyStiffness(double curr_vx, double vy_est, double curr_delta, 
                              double curr_r, double curr_ay, double dt);
    
    void esoCompute(double curr_r, double curr_delta, double dt);

    double normalizeAngle(double angle);

    // --- ROS 与路径处理辅助函数  ---
    double quaternion_to_yaw(const geometry_msgs::Quaternion& q);
    int find_nearest_path_point(const double x0, const double y0, const race_msgs::Path& path);
    std::vector<double> calculate_cumulative_distance(const race_msgs::Path& path, int start_idx);
    std::vector<double> linear_interpolate(const std::vector<double>& s_original, 
                                           const std::vector<double>& val_original, 
                                           const std::vector<double>& s_target);
    casadi::DM interpolate_path_segment(const race_msgs::Path& path, const std::vector<double>& cum_dist, 
                                        int start_idx, int end_idx, const std::vector<double>& s_target, double yaw0);
    casadi::DM process_race_path(const race_msgs::Path& input_path, const std::vector<double>& current_state);
    
    std::vector<double> vehicleStatusToStateVector(const race_msgs::VehicleStatus& status);

private:

    bool is_high_speed_last_;
    double blend_alpha_;
    double nmpc_safe_cmd_;
    ros::Time start_time_;
    double last_final_cmd_;
    
     // 动态预瞄参数
    double min_lookahead_distance_;
    double lookahead_speed_coeff_;

    // --- 核心参数结构体 ---
    NMPCParams nmpc_params_;
    SupervisorParams supervisor_params_;
    NMPSolver solver_;
    
    double current_cmd_;

    // ESO观测器相关
    double eso_x1_;
    double eso_x2_;

    // UKF相关
    Eigen::Vector2d ukf_x_est_;
    Eigen::Matrix2d ukf_P_est_;

    // RLS相关
    double rls_P_f_;
    double rls_theta_f_;
    double rls_P_r_;
    double rls_theta_r_;
    double rls_r_prev_;
    double rls_r_dot_pre_;
    double rls_Cf_est_;
    double rls_Cr_est_;
};

} // namespace race_tracker

#endif // RACE_TRACKER_ESO_TRACKER_H
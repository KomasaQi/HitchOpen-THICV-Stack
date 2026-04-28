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

// NMPC参数 
struct NMPCParams {
    double m = 10000.0;
    double Iz = 50000.0;
    double lf = 2.0;
    double lr = 2.135;
    double Cf = 250000.0;
    double Cr = 1000000.0;
    double T_lag = 0.2;
    double dt = 0.05;
    int N = 35;
    int Nc = 5;
    int nx = 6;
    int nu = 1;
    double delta_max = 0.5;
    double delta_min = -0.5;
    double delta_c_max = 1.5;

    // 代价函数权重独立变量 
    double Q_x = 1000.0, Q_y = 5000.0, Q_theta = 4000.0;
    double Q_vy = 100.0, Q_r = 800.0, Q_delta = 1000.0;
    double R = 10.0;
    double dR = 500.0;

    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();

    NMPCParams() {
        updateQMatrix();
    }

    // 加载完YAML后调用此函数更新Eigen矩阵
    void updateQMatrix() {
        Q.setZero();
        Q(0,0) = Q_x; Q(1,1) = Q_y; Q(2,2) = Q_theta;
        Q(3,3) = Q_vy; Q(4,4) = Q_r; Q(5,5) = Q_delta;
    }
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
    bool has_prev_sol = false;
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

    bool is_high_speed_last_ = false;       // 上一帧是否为高速模式
    double blend_alpha_ = 0.0;              // 切换权重：0=纯跟踪 1=完全NMPC
    static constexpr double BLEND_LOW = 15.0 / 3.6;  // 13km/h 开始过渡
    static constexpr double BLEND_HIGH = 18.0 / 3.6; // 17km/h 完全切NMPC
    double nmpc_safe_cmd_ = 0.0;            // NMPC预计算的安全输出（纯跟踪模式下预热用
    // --- 状态与持久化变量 ---
    NMPCParams nmpc_params_;
    NMPSolver solver_;
    
    double current_cmd_ = 0.0;
    int nmpc_counter_ = 4;

    // ESO观测器相关
    double eso_x1_ = 0.0;
    double eso_x2_ = 0.0;

    // UKF相关
    Eigen::Vector2d ukf_x_est_;  // [vy, r]
    Eigen::Matrix2d ukf_P_est_;

    // RLS相关
    double rls_P_f_ = 1e5;
    double rls_theta_f_ = 250000.0;
    double rls_P_r_ = 1e5;
    double rls_theta_r_ = 1000000.0;
    double rls_r_prev_ = 0.0;
    double rls_r_dot_pre_ = 0.0;
    double rls_Cf_est_ = 250000.0;
    double rls_Cr_est_ = 1000000.0;
};

} // namespace race_tracker

#endif // RACE_TRACKER_ESO_TRACKER_H
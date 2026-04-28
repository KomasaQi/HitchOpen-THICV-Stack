#ifndef RACE_TRACKER_ESO_TRACKER2_H
#define RACE_TRACKER_ESO_TRACKER2_H

#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <memory>
#include <tf/transform_datatypes.h>
// #include <stdexcept>
// #include <numeric>
// #include <limits>

// ROS 插件和消息相关头文件
#include "race_tracker/controller_plugin_base.h"
#include <race_msgs/Control.h>
#include <race_msgs/VehicleStatus.h>
#include <race_msgs/Path.h>
#include <race_msgs/Flag.h>

namespace race_tracker {

// 扩展NMPC参数（兼容原结构，新增挂车参数）
struct NMPCParams {
    // 牵引车参数（Matlab默认值覆盖原单车值）
    double m = 6595.0;
    double Iz = 34806.2;
    double lf = 2.0;
    double lr = 2.135;
    double Cf = 250000.0;
    double Cr = 1000000.0;
    // 挂车参数（新增）
    double m_t = 7570.0;
    double Iz_t = 150000.0;
    double lh = 0.0;
    double lt = 3.4;
    double L2 = 7.9;
    double Ct = 400000.0;
    double M = m+m_t;     // 总质量
    // 约束参数（新增）
    double delta_rate_max = 100.0/180.0*M_PI;
    double delta_rate_min = -100.0/180.0*M_PI;
    // double gamma_max = 50.0/180.0*M_PI;
    // double gamma_min = -50.0/180.0*M_PI;
    // 原有NMPC核心参数（Matlab值覆盖）
    double T_lag = 0.2;
    double dt = 0.05;
    int N = 25;
    int Nc = 5;
    int nx = 8; // 扩展为8维状态：[x,y,theta,vy,r,delta,r_t,gamma]
    int nu = 1;
    double delta_max = 0.5;
    double delta_min = -0.5;       
    // 代价函数权重（兼容原结构，新增挂车权重）
    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    double R = 800.0;
    double dR = 1500.0;
    double Q_gamma = 10.0;
    double Q_r_t = 50.0;
    double Q_gamma_rate = 300.0;

    NMPCParams() {
        // 原Q矩阵初始化（Matlab值覆盖）
        Q(0,0) = 4000; Q(1,1) = 4000; Q(2,2) = 2000;
        Q(3,3) = 100;  Q(4,4) = 800;  Q(5,5) = 100;
    }
};

// 扩展NMPC求解器（兼容原结构，新增热启动缓存）
struct NMPSolver {
    casadi::Opti opti;
    casadi::MX X;
    casadi::MX U_sparse;
    casadi::MX P_x0;
    casadi::MX P_waypoints;
    casadi::MX P_vx;
    casadi::MX P_u_prev;
    casadi::MX P_h_hat;
    casadi::MX P_dyn_params; // 扩展为10维动力学参数
    std::unique_ptr<casadi::OptiSol> sol_prev;
    bool has_prev_sol = false;
    // // 热启动缓存（新增，对应Matlab last_sol_X/U）
    // casadi::DM last_sol_X;
    // casadi::DM last_sol_U;
};

// 核心控制器类（完全保留原类名）
class ESOTracker2 : public ControllerPluginBase {
public:
    ESOTracker2();
    ~ESOTracker2() override = default;

    // --- 完全保留原ROS插件接口 ---
    bool initialize(ros::NodeHandle& nh) override;
    
    void computeControl(
        const race_msgs::VehicleStatusConstPtr& vehicle_status,
        const race_msgs::PathConstPtr& path,
        race_msgs::Control* control_msg,
        const double dt,
        const race_msgs::Flag::ConstPtr& flag) override;

    std::string getName() const override { return "ESOTracker2"; }

private:
    // --- 完全保留原核心函数名，内部实现替换为挂车逻辑 ---
    void buildNMPSolver();

    casadi::MX vehicleDynamicsModel(const casadi::MX& state, const casadi::MX& cmd_delta,
                                    const casadi::MX& vx, const casadi::MX& h_dist,
                                    const casadi::MX& dyn_params);
    bool solveNMPC(const std::vector<double>& current_state, const casadi::DM& waypoints,
                   std::vector<double>& control_output);
    
    // 原UKF替换为EKF，函数名微调（逻辑完全对应Matlab EKF_LateralVelocity）
    void ekfEstimateVy(double curr_vx, double curr_delta, double curr_ay, double curr_r, double M, double dt);
    // 原RLS替换为FF-RLS，保留原函数名
    void rlsIdentifyStiffness(double curr_vx, double vy_est, double curr_delta,double curr_r,
                              double curr_ay, double curr_gamma, double curr_r_t, double M, double dt);
    // 保留原ESO函数名，实现替换为Matlab逻辑
    void esoCompute(double curr_r, double curr_delta, double dt);

    double normalizeAngle(double angle);

    // ---  ROS 与路径处理辅助函数 ---
    double quaternion_to_yaw(const geometry_msgs::Quaternion& q);
    int find_nearest_path_point(const double x0, const double y0, const race_msgs::Path& path);
    std::vector<double> calculate_cumulative_distance(const race_msgs::Path& path, int start_idx);
    std::vector<double> linear_interpolate(const std::vector<double>& s_original, 
                                           const std::vector<double>& val_original, 
                                           const std::vector<double>& s_target);
    casadi::DM interpolate_path_segment(const race_msgs::Path& path, const std::vector<double>& cum_dist, 
                                        int start_idx, int end_idx, const std::vector<double>& s_target, double yaw0);
    casadi::DM process_race_path(const race_msgs::Path& input_path, const std::vector<double>& current_state);

    // --- 预瞄点读取及挂车状态计算函数 ---
    double calculate_curvature_and_steering(const Eigen::Vector2d& p1, 
                                            const Eigen::Vector2d& p2, 
                                            const Eigen::Vector2d& p3, 
                                            double L1);
    void calculate_trailer_kinematics(double delta_f, double curr_vx, double curr_r, double dt);

    std::vector<double> vehicleStatusToStateVector(const race_msgs::VehicleStatus& status);

private:
  // --- 状态与持久化变量 ---
    NMPCParams nmpc_params_;
    NMPSolver solver_;
    
    double current_cmd_ = 0.0;
    int nmpc_counter_ = 4;

    // ESO观测器（保留原变量名）
    double eso_x1_ = 0.0;
    double eso_x2_ = 0.0;

    // 原UKF变量替换为EKF变量
    Eigen::Vector4d ekf_x_hat_;  // [vy, r, r_t, gamma]
    Eigen::Matrix4d ekf_P_;

    // 原RLS变量替换为FF-RLS变量（保留rls_前缀）
    double rls_w1_prev_ = 0.0;
    double rls_w2_prev_ = 0.0;
    Eigen::Matrix3d rls_P_;
    double rls_w1_dot_prev_ = 0.0;
    double rls_w2_dot_prev_ = 0.0;
    Eigen::Vector3d rls_C_out_prev_;
    double rls_Cf_est_ = 250000.0;
    double rls_Cr_est_ = 1000000.0;
    double rls_Ct_est_ = 400000.0;

    // 挂车状态变量（新增）
    double gamma_ = 0.0;
    double r_t_ = 0.0;

    // 预测轨迹（新增）
    // casadi::DM predicted_trajectory_;
};

} // namespace race_tracker

#endif // RACE_TRACKER_ESO_TRACKER2_H

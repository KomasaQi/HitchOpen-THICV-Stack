#include "race_tracker/eso_tracker2.h"
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <numeric>
#include <limits>
#include <stdexcept>
#include <chrono>
#include <algorithm>  // for std::clamp


using namespace casadi;
using namespace Eigen;
using namespace std;

namespace race_tracker {

// 构造函数
ESOTracker2::ESOTracker2() {
    // EKF初始化
    ekf_x_hat_ = Vector4d::Zero();
    ekf_P_ = Matrix4d::Identity() * 1.0;
    // RLS初始化（FF-RLS）
    rls_P_ = Matrix3d::Identity() * 1e6;
    // rls_C_out_prev_ = Vector3d(rls_Cf_est_, rls_Cr_est_, rls_Ct_est_);
    // 原ESO变量初始化
    eso_x1_ = 0.0;
    eso_x2_ = 0.0;

    // 挂车状态初始化
    gamma_ = 0.0;
    r_t_ = 0.0;
    // 原NMPC变量初始化
    current_cmd_ = 0.0;
    solver_.has_prev_sol = false;
    solver_.sol_prev = nullptr;

    // --- 模式切换与软过渡变量初始化（参考ESOTracker） ---
    is_high_speed_last_ = false;
    blend_alpha_ = 0.0;
    nmpc_safe_cmd_ = 0.0;
    last_final_cmd_ = 0.0;

    supervisor_params_.startup_time = 5.0;
    supervisor_params_.blend_speed_low = 4.1667;
    supervisor_params_.blend_speed_high = 5.0;
    supervisor_params_.standstill_speed = 0.05;
    supervisor_params_.nmpc_speed_floor = 1.0;

}

// 插件初始化
bool ESOTracker2::initialize(ros::NodeHandle& nh) {
    ros::NodeHandle nh_nmpc(nh, "eso_tracker2");
    ROS_INFO("[%s] NMPC 控制器命名空间: %s", getName().c_str(), nh_nmpc.getNamespace().c_str());

    // 加载NMPC核心参数
    nh_nmpc.param("nx", nmpc_params_.nx, 8);
    nh_nmpc.param("nu", nmpc_params_.nu, 1);
    nh_nmpc.param("prediction_step", nmpc_params_.N, 35);
    nh_nmpc.param("sparse_control_step", nmpc_params_.Nc, 5);
    nh_nmpc.param("sampling_time", nmpc_params_.dt, 0.05);
    // 加载—— 车辆核心动力学参数（含挂车） ——
    nh_nmpc.param("m", nmpc_params_.m, 10000.0);
    nh_nmpc.param("Iz", nmpc_params_.Iz, 50000.0);
    nh_nmpc.param("lf", nmpc_params_.lf, 2.0);
    nh_nmpc.param("lr", nmpc_params_.lr, 2.135);
    nh_nmpc.param("m_t_empty", nmpc_params_.m_t, 7570.0);
    nh_nmpc.param("Iz_t_empty", nmpc_params_.Iz_t, 150000.0);
    nh_nmpc.param("m_t_total", nmpc_params_.m_t_total, 39500.0);
    nh_nmpc.param("lt", nmpc_params_.lt, 3.4);
    nh_nmpc.param("L2", nmpc_params_.L2, 7.9);
    nh_nmpc.param("lh", nmpc_params_.lh, 0.0);
    nh_nmpc.param("T_lag", nmpc_params_.T_lag, 0.1);
    m_t_total_default_ = nmpc_params_.m_t_total;
    m_t_empty_default_ = nmpc_params_.m_t;

    // 加载—— 控制量边界约束 ——
    nh_nmpc.param("min_steer", nmpc_params_.delta_min, -0.6);
    nh_nmpc.param("max_steer", nmpc_params_.delta_max, 0.6);
    nh_nmpc.param("delta_rate_max", nmpc_params_.delta_rate_max, 1.5);
    nh_nmpc.param("delta_rate_min", nmpc_params_.delta_rate_min, -1.5);

    // 加载—— 代价函数权重 ——
    nh_nmpc.param("Q_x", nmpc_params_.Q(0,0), 10000.0);          // 横向位置误差权重
    nh_nmpc.param("Q_y", nmpc_params_.Q(1,1), 10000.0);          // 纵向位置误差权重
    nh_nmpc.param("Q_theta", nmpc_params_.Q(2,2), 5000.0);       // 航向角误差权重
    nh_nmpc.param("Q_vy", nmpc_params_.Q(3,3), 100.0);          // 侧向速度权重
    nh_nmpc.param("Q_r", nmpc_params_.Q(4,4), 800.0);          // 横摆角速度权重
    nh_nmpc.param("Q_delta", nmpc_params_.Q(5,5), 1000.0);      // 未使用该权重项
    nh_nmpc.param("Q_rt", nmpc_params_.Q(6,6), 1.0);      // 挂车状态权重
    nh_nmpc.param("Q_gamma", nmpc_params_.Q(7,7), 1.0);      // 挂车状态权重


    nh_nmpc.param("Q_R", nmpc_params_.R, 10.0);      // 控制量权重 
    nh_nmpc.param("Q_dR", nmpc_params_.dR, 100000.0);      // 控制增量权重 (平滑性)

    //  —— 参数估计中参数 ——
    nh_nmpc.param("Kiz", nmpc_params_.Kiz, 5.1);      // 横摆转动惯量比例系数

    // 加载—— 参数估计中参数 ——
    nh_nmpc.param("rls_Cf_est", rls_Cf_est_default_, 250000.0);          // 挂车前轴刚度初始值（N/m）
    nh_nmpc.param("rls_Cr_est", rls_Cr_est_default_, 1000000.0);         // 挂车后轴刚度初始值（N/m）
    nh_nmpc.param("rls_Ct_est", rls_Ct_est_default_, 400000.0);          // 挂车横摆转动惯量初始值（kg·m²）
    nh_nmpc.param("rls_w1_prev", rls_w1_prev_, 0.0);              // 挂车横摆角初始值（rad）
    nh_nmpc.param("rls_w1_dot_prev", rls_w1_dot_prev_, 0.0);        // 挂车横摆角速度初始值（rad/s）
    nh_nmpc.param("rls_Cf_est_max", rls_Cf_est_max_, 500000.0);          // 挂车前轴刚度最大值（N/m）
    nh_nmpc.param("rls_Cr_est_max", rls_Cr_est_max_, 2000000.0);          // 挂车后轴刚度最大值（N/m）
    nh_nmpc.param("rls_Ct_est_max", rls_Ct_est_max_, 200000.0);          // 挂车横摆转动惯量最大值（kg·m²）
    nh_nmpc.param("rls_Cf_est_min", rls_Cf_est_min_, 50000.0);          // 挂车前轴刚度最小值（N/m）
    nh_nmpc.param("rls_Cr_est_min", rls_Cr_est_min_, 100000.0);          // 挂车后轴刚度最小值（N/m）
    nh_nmpc.param("rls_Ct_est_min", rls_Ct_est_min_, 50000.0);          // 挂车横摆转动惯量最小值（kg·m²）

    //  —— IPOPT求解器参数 ——
    nh_nmpc.param("ipopt_max_iter", ipopt_max_iter_, 100.0);                      // IPPT最大迭代次数，默认100次
    nh_nmpc.param("ipopt_acceptable_tol", ipopt_acceptable_tol_, 1e-2);               // IPPT可接受解的容差，默认1e-2
    nh_nmpc.param("ipopt_acceptable_iter", ipopt_acceptable_iter_, 5.0);                 // IPPT可接受迭代次数，默认5次 连续 5 次迭代满足 acceptable_tol 即可停止
    nh_nmpc.param("ipopt_warm_start_bound_push", ipopt_warm_start_bound_push_, 1e-3);        // IPPT预热边界推送系数，默认1e-3，用于初始化控制量边界，热启动时，将初始点向变量边界 “推” 的最小距离
    nh_nmpc.param("ipopt_warm_start_slack_bound_push", ipopt_warm_start_slack_bound_push_, 1e-3);  // IPPT预热边界松弛系数，默认1e-3，用于初始化控制量边界,针对松弛变量
    nh_nmpc.param("ipopt_warm_start_mult_bound_push", ipopt_warm_start_mult_bound_push_, 1e-3);   // IPPT预热边界松弛系数，默认1e-3，用于初始化控制量边界,针对拉格朗日乘子

    // 加载—— 求解器设定 ——
    nh_nmpc.param("integration_grade", integration_grade_, 2.0);          // 求解器积分阶数，默认2阶

    rls_Cf_est_ = rls_Cf_est_default_; // 将默认参数赋予变量
    rls_Cr_est_ = rls_Cr_est_default_;
    rls_Ct_est_ = rls_Ct_est_default_;
    rls_C_out_prev_ = Vector3d(rls_Cf_est_, rls_Cr_est_, rls_Ct_est_);

    // 加载—— 动态预瞄参数 ——
    nh_nmpc.param("min_lookahead_distance", min_lookahead_distance_, 3.0);
    nh_nmpc.param("lookahead_speed_coeff", lookahead_speed_coeff_, 0.5);
    

    // 打印加载的参数
    // NMPC核心参数
    logParamLoad("nx",nmpc_params_.nx, 8);
    logParamLoad("nu",nmpc_params_.nu, 1);
    logParamLoad("prediction_step",nmpc_params_.N, 35);
    logParamLoad("sparse_control_step",nmpc_params_.Nc, 5);
    logParamLoad("sampling_time",nmpc_params_.dt, 0.05);
    // 车辆核心动力学参数（含挂车） ——
    logParamLoad("m", nmpc_params_.m, 10000.0);
    logParamLoad("Iz", nmpc_params_.Iz, 50000.0);
    logParamLoad("lf", nmpc_params_.lf, 2.0);
    logParamLoad("lr", nmpc_params_.lr, 2.135);
    logParamLoad("m_t", nmpc_params_.m_t, 7570.0);
    logParamLoad("Iz_t", nmpc_params_.Iz_t, 150000.0);
    logParamLoad("lt", nmpc_params_.lt, 3.4);
    logParamLoad("L2", nmpc_params_.L2, 7.9);
    logParamLoad("lh", nmpc_params_.lh, 0.0);
    logParamLoad("T_lag", nmpc_params_.T_lag, 0.1);
    // 控制量边界约束 ——
    logParamLoad("min_steer", nmpc_params_.delta_min, -0.6);
    logParamLoad("max_steer", nmpc_params_.delta_max, 0.6);
    logParamLoad("delta_rate_max", nmpc_params_.delta_rate_max, 1.5);
    logParamLoad("delta_rate_min", nmpc_params_.delta_rate_min, -1.5);
    // 代价函数权重 ——
    logParamLoad("Q_x",nmpc_params_.Q(0,0), 10001.0); 
    logParamLoad("Q_y",nmpc_params_.Q(1,1), 10000.0);          // 纵向位置误差权重
    logParamLoad("Q_theta",nmpc_params_.Q(2,2), 5000.0);       // 航向角误差权重
    logParamLoad("Q_vy",nmpc_params_.Q(3,3), 100.0);          // 侧向速度权重
    logParamLoad("Q_r",nmpc_params_.Q(4,4), 800.0);          // 横摆角速度权重
    logParamLoad("Q_delta",nmpc_params_.Q(5,5), 1000.0);      // 未使用该权重项
    logParamLoad("Q_rt",nmpc_params_.Q(6,6), 1.0);      // 挂车状态权重
    logParamLoad("Q_gamma",nmpc_params_.Q(7,7), 1.0);      // 挂车状态权重
    logParamLoad("Q_R", nmpc_params_.R, 10.0);      // 控制量权重 
    logParamLoad("Q_dR", nmpc_params_.dR, 100000.0);      // 控制增量权重 (平滑性)
    // —— 参数估计中参数 ——
    logParamLoad("Kiz", nmpc_params_.Kiz, 5.1);      // 横摆转动惯量比例系数
    logParamLoad("rls_Cf_est", rls_Cf_est_, 250000.0);          // 挂车前轴刚度初始值（N/m）
    logParamLoad("rls_Cr_est", rls_Cr_est_, 1000000.0);         // 挂车后轴刚度初始值（N/m）
    logParamLoad("rls_Ct_est", rls_Ct_est_, 400000.0);          // 挂车横摆转动惯量初始值（kg·m²）
    logParamLoad("rls_w1_prev", rls_w1_prev_, 0.0);              // 挂车横摆角初始值（rad）
    logParamLoad("rls_w1_dot_prev", rls_w1_dot_prev_, 0.0);        // 挂车横摆角速度初始值（rad/s）
    logParamLoad("rls_Cf_est_max", rls_Cf_est_max_, 500000.0);          // 挂车前轴刚度最大值（N/m）
    logParamLoad("rls_Cr_est_max", rls_Cr_est_max_, 2000000.0);          // 挂车后轴刚度最大值（N/m）
    logParamLoad("rls_Ct_est_max", rls_Ct_est_max_, 200000.0);          // 挂车横摆转动惯量最大值（kg·m²）
    logParamLoad("rls_Cf_est_min", rls_Cf_est_min_, 50000.0);          // 挂车前轴刚度最小值（N/m）
    logParamLoad("rls_Cr_est_min", rls_Cr_est_min_, 100000.0);          // 挂车后轴刚度最小值（N/m）
    logParamLoad("rls_Ct_est_min", rls_Ct_est_min_, 50000.0);          // 挂车横摆转动惯量最小值（kg·m²）
    logParamLoad("integration_grade", integration_grade_, 2.0);          // 求解器积分阶数，默认2阶

    // -------------------------------------------------------------------------
    // 6. 加载 Supervisor 配置 (模式切换与纯跟踪)
    // -------------------------------------------------------------------------
    ros::NodeHandle nh_super(nh, "supervisor_config");
    nh_super.param("startup_time", supervisor_params_.startup_time, 5.0);
    nh_super.param("blend_speed_low", supervisor_params_.blend_speed_low, 4.1667);
    nh_super.param("blend_speed_high", supervisor_params_.blend_speed_high, 5.0);
    nh_super.param("standstill_speed", supervisor_params_.standstill_speed, 0.05);
    nh_super.param("nmpc_speed_floor", supervisor_params_.nmpc_speed_floor, 1.0);
    nh_super.param("min_lookahead_distance", min_lookahead_distance_, 6.0);
    nh_super.param("lookahead_speed_coeff", lookahead_speed_coeff_, 0.7);



    // IPOPT
    logParamLoad("ipopt_max_iter", ipopt_max_iter_, 100.0);                      // IPPT最大迭代次数，默认100次
    logParamLoad("ipopt_acceptable_tol", ipopt_acceptable_tol_, 1e-2);               // IPPT可接受解的容差，默认1e-2
    logParamLoad("ipopt_acceptable_iter", ipopt_acceptable_iter_, 5.0);                 // IPPT可接受迭代次数，默认5次 连续 5 次迭代满足 acceptable_tol 即可停止
    logParamLoad("ipopt_warm_start_bound_push", ipopt_warm_start_bound_push_, 1e-3);        // IPPT预热边界推送系数，默认1e-3，用于初始化控制量边界，热启动时，将初始点向变量边界 “推” 的最小距离
    logParamLoad("ipopt_warm_start_slack_bound_push", ipopt_warm_start_slack_bound_push_, 1e-3);  // IPPT预热边界松弛系数，默认1e-3，用于初始化控制量边界,针对松弛变量
    logParamLoad("ipopt_warm_start_mult_bound_push", ipopt_warm_start_mult_bound_push_, 1e-3);   // IPPT预热边界松弛系数，默认1e-3，用于初始化控制量边界,针对拉格朗日乘子


    start_time_ = ros::Time::now(); // 记录控制器启动时间
    
    // 构建CasADi求解器（保留原函数名）
    buildNMPSolver();

    // 初始化铰接角发布器
    est_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/race/estimation_states", 1);


    ROS_INFO("[%s] 控制器初始化完成（挂车版）", getName().c_str());
    return true;
}

// 核心控制循环（完全保留原接口，内部替换为挂车逻辑）
void ESOTracker2::computeControl(
    const race_msgs::VehicleStatusConstPtr& vehicle_status,
    const race_msgs::PathConstPtr& path,
    race_msgs::Control* control_msg,
    const double dt,
    const race_msgs::Flag::ConstPtr& flag) {

    if (!vehicle_status || !path || !control_msg) {
        ROS_ERROR("[%s] 收到空指针消息", getName().c_str());
        return;
    }
    if (path->points.empty()) {
        ROS_WARN_THROTTLE(1.0, "[%s] 收到空路径，保持上一帧转角 %.3f rad", getName().c_str(), last_final_cmd_);
        control_msg->lateral.steering_angle = last_final_cmd_;
        control_msg->steering_mode = race_msgs::Control::FRONT_STEERING_MODE;
        control_msg->control_mode = race_msgs::Control::DES_ACCEL_ONLY;
        return;
    }

    const ros::Time current_time = ros::Time::now();
    const double obs_dt = std::max(dt, 0.01);


    // 提取当前状态
    const double curr_x = vehicle_status->pose.position.x;
    const double curr_y = vehicle_status->pose.position.y;
    const double curr_theta = vehicle_status->euler.yaw;
    const double curr_vx_raw = vehicle_status->vel.linear.x;
    const double v_abs = std::abs(curr_vx_raw);
    const double vx_floor = std::max(0.1, supervisor_params_.nmpc_speed_floor);
    const double curr_vx = std::max(v_abs, vx_floor);   // NMPC/观测器内部速度，避免低速奇异
    const double curr_ay = vehicle_status->acc.linear.y;
    const double curr_r = vehicle_status->vel.angular.z;
    double curr_delta = vehicle_status->lateral.steering_angle;
     // 根据收到的车辆状态更新挂车载货质量
    if (vehicle_status->trailer.mass > m_t_empty_default_) {
        nmpc_params_.m_t_total = vehicle_status->trailer.mass;
        ROS_INFO("[%s] 更新挂车总质量: %.2f kg", getName().c_str(), nmpc_params_.m_t_total);

    } else {
        nmpc_params_.m_t_total = m_t_total_default_;
        ROS_WARN("[%s] 收到异常挂车质量为%.2f kg , 使用默认挂车总质量: %.2f kg", getName().c_str(), vehicle_status->trailer.mass, nmpc_params_.m_t_total);
    }

    curr_delta = std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, curr_delta));

    static ros::Time last_control_time = ros::Time(0);
    if (last_control_time.toSec() != 0.0 && (current_time - last_control_time).toSec() > 0.2) {
        ROS_WARN("[%s] 检测到控制重连，清空历史记忆与热启动！", getName().c_str());
        solver_.has_prev_sol = false;
        solver_.sol_prev = nullptr;
        current_cmd_ = curr_delta;
        last_final_cmd_ = curr_delta;
        nmpc_safe_cmd_ = curr_delta;
        blend_alpha_ = 0.0;
        is_high_speed_last_ = false;
        eso_x1_ = curr_r;
        eso_x2_ = 0.0;
        ekf_P_ = Matrix4d::Identity() * 1.0;
        start_time_ = current_time;
    }
    last_control_time = current_time;

    if (rls_w1_prev_ == 0.0 && curr_r != 0.0) {
        rls_w1_prev_ = curr_r;
    }

    // 1. 路径处理始终执行：低速也生成参考，保证 PP 与 NMPC 热启动一致
    std::vector<double> current_pose = {curr_x, curr_y, curr_theta, curr_vx};
    casadi::DM waypoints_dm = process_race_path(*path, current_pose);
    Vector2d p1(waypoints_dm(0,0), waypoints_dm(1,0));
    Vector2d p2(waypoints_dm(0,1), waypoints_dm(1,1));
    Vector2d p3(waypoints_dm(0,2), waypoints_dm(1,2));
    const double tractor_L = nmpc_params_.lf + nmpc_params_.lr;
    const double delta_f = calculate_curvature_and_steering(p1, p2, p3, tractor_L);

    // 2. 观测器/估计器始终更新，但内部速度使用 vx_floor 防止低速分母异常
    calculate_trailer_kinematics(delta_f, curr_vx, curr_r, obs_dt);
    const double curr_gamma = -gamma_;
    const double curr_r_t = r_t_;
    ROS_INFO_THROTTLE(0.5, "\033[36m[%s] 估计挂车转角: %.3f rad (%.1f deg)\033[0m",
                      getName().c_str(), curr_gamma, curr_gamma * 180.0 / M_PI);

    ekfEstimateVy(curr_vx, curr_delta, curr_ay, curr_r, nmpc_params_.m_t_total, obs_dt);
    const double vy_est = ekf_x_hat_(0);

    rlsIdentifyStiffness(curr_vx, vy_est, curr_delta, curr_r, curr_ay, curr_gamma, curr_r_t, nmpc_params_.m_t_total, obs_dt);

    esoCompute(curr_r, curr_delta, obs_dt);
    const double h_hat_total = eso_x2_;
    const double d_pure_trailer = h_hat_total;
    ROS_INFO_THROTTLE(0.5, "[%s] \033[35mESO观测结果: x1=%.4f rad/s, x2=%.4f, h_hat_total=%.4f\033[0m",
                      getName().c_str(), eso_x1_, eso_x2_, h_hat_total);

    // 3. 启动/低速/高速软切换权重：0=纯跟踪，1=NMPC
    const double time_elapsed = (current_time - start_time_).toSec();
    if (time_elapsed < supervisor_params_.startup_time) {
        blend_alpha_ = 0.0;
        ROS_INFO_THROTTLE(1.0, "[%s][STARTUP] 预热 %.1f / %.1f s，纯跟踪锁定",
                          getName().c_str(), time_elapsed, supervisor_params_.startup_time);
    } else if (v_abs <= supervisor_params_.blend_speed_low) {
        blend_alpha_ = 0.0;
    } else if (v_abs >= supervisor_params_.blend_speed_high) {
        blend_alpha_ = 1.0;
    } else {
        const double denom = std::max(1e-3, supervisor_params_.blend_speed_high - supervisor_params_.blend_speed_low);
        blend_alpha_ = (v_abs - supervisor_params_.blend_speed_low) / denom;
        blend_alpha_ = std::max(0.0, std::min(1.0, blend_alpha_));
    }
    const bool is_current_high_speed = (blend_alpha_ >= 0.99);

    // 4. NMPC 后台始终尝试求解，低速/启动阶段也刷新热启动；失败时保持上一帧 NMPC 有效值
    std::vector<double> nmpc_state = {curr_x, curr_y, curr_theta, vy_est, curr_r,
                                      curr_delta, curr_r_t, curr_gamma};
    std::vector<double> control_output(1, nmpc_safe_cmd_);

    std::vector<double> dyn_params = {
        nmpc_params_.m, nmpc_params_.Iz, nmpc_params_.lf, nmpc_params_.lr, rls_Cf_est_,
        rls_Cr_est_, nmpc_params_.m_t_total, nmpc_params_.Iz_t, nmpc_params_.lt, rls_Ct_est_, nmpc_params_.L2
    };
    solver_.opti.set_value(solver_.P_vx, curr_vx);
    solver_.opti.set_value(solver_.P_h_hat, d_pure_trailer);
    solver_.opti.set_value(solver_.P_dyn_params, dyn_params);

    const bool nmpc_solve_success = solveNMPC(nmpc_state, waypoints_dm, control_output);
    if (nmpc_solve_success) {
        nmpc_safe_cmd_ = control_output[0];
    } else {
        nmpc_safe_cmd_ = current_cmd_;
        ROS_WARN_THROTTLE(0.5, "[%s] NMPC 求解失败，NMPC支路保持上一帧有效输出 %.3f rad",
                          getName().c_str(), nmpc_safe_cmd_);
    }
    nmpc_safe_cmd_ = std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, nmpc_safe_cmd_));

    // 5. 纯跟踪保护支路：启动、静止、低速、NMPC失败时都可兜底；最终通过 blend_alpha_ 连续融合
    const double lookahead_dist = min_lookahead_distance_ + lookahead_speed_coeff_ * curr_vx;
    double pp_safe_cmd = computePurePursuitSteering(*path, curr_x, curr_y, curr_theta, lookahead_dist);
    pp_safe_cmd = std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, pp_safe_cmd));

    if (v_abs <= supervisor_params_.standstill_speed) {
        blend_alpha_ = 0.0;
        ROS_INFO_THROTTLE(1.0, "[%s][STANDSTILL] v=%.2f m/s，保持纯跟踪保护，不再输出固定零转角",
                          getName().c_str(), curr_vx_raw);
    }

    if (blend_alpha_ < 0.01) {
        ROS_INFO_THROTTLE(0.5, "[%s][PP] v=%.1f km/h | Ld=%.2f m | PP=%.3f | NMPC后台=%s",
                          getName().c_str(), curr_vx_raw * 3.6, lookahead_dist, pp_safe_cmd,
                          nmpc_solve_success ? "OK" : "FAIL");
    } else if (blend_alpha_ > 0.99) {
        ROS_INFO_THROTTLE(1.0, "[%s][NMPC] v=%.1f km/h | cmd=%.3f rad", getName().c_str(), curr_vx_raw * 3.6, nmpc_safe_cmd_);
    } else {
        ROS_INFO_THROTTLE(0.5, "[%s][BLEND] v=%.1f km/h | alpha=%.2f | PP=%.3f | NMPC=%.3f",
                          getName().c_str(), curr_vx_raw * 3.6, blend_alpha_, pp_safe_cmd, nmpc_safe_cmd_);
    }

    // 6. 融合输出 + 全局转角速率限制。用 last_final_cmd_ 作为最终输出历史，避免支路切换跳变。
    double final_cmd = blend_alpha_ * nmpc_safe_cmd_ + (1.0 - blend_alpha_) * pp_safe_cmd;
    final_cmd = std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, final_cmd));

    const double rate_limit = std::max(std::abs(nmpc_params_.delta_rate_max), std::abs(nmpc_params_.delta_rate_min));
    const double d_delta_max = std::max(0.01, rate_limit * obs_dt);
    final_cmd = std::max(last_final_cmd_ - d_delta_max, std::min(last_final_cmd_ + d_delta_max, final_cmd));

    last_final_cmd_ = final_cmd;
    current_cmd_ = final_cmd;
    is_high_speed_last_ = is_current_high_speed;

    control_msg->lateral.steering_angle = final_cmd;
    control_msg->steering_mode = race_msgs::Control::FRONT_STEERING_MODE;
    control_msg->control_mode = race_msgs::Control::DES_ACCEL_ONLY;

    // 发布估计状态
    std_msgs::Float64MultiArray est_msg;
    est_msg.data.resize(7);
    est_msg.data[0] = curr_r_t;        // 挂车横摆率
    est_msg.data[1] = -gamma_;        // 铰接角
    est_msg.data[2] = vy_est;          // 牵引车侧向速度
    est_msg.data[3] = h_hat_total;     // ESO扰动估计
    est_msg.data[4] = rls_Cf_est_;     // RLS Cf
    est_msg.data[5] = rls_Cr_est_;     // RLS Cr
    est_msg.data[6] = rls_Ct_est_;     // RLS Ct
    est_pub_.publish(est_msg);

}

// ---------------------- 路径处理与辅助函数----------------------
double ESOTracker2::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

double ESOTracker2::quaternion_to_yaw(const geometry_msgs::Quaternion& q) {
    tf::Quaternion tf_quat(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 rot_matrix(tf_quat);
    double roll, pitch, yaw;
    rot_matrix.getRPY(roll, pitch, yaw);
    return yaw;
}

int ESOTracker2::find_nearest_path_point(const double x0, const double y0, const race_msgs::Path& path) {
    double min_dist_sq = std::numeric_limits<double>::max();
    int nearest_idx = 0;
    for (size_t i = 0; i < path.points.size(); ++i) {
        const auto& pt = path.points[i].pose.position;
        double dist_sq = (pt.x - x0) * (pt.x - x0) + (pt.y - y0) * (pt.y - y0);
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            nearest_idx = i;
        }
    }
    return nearest_idx;
}

std::vector<double> ESOTracker2::calculate_cumulative_distance(const race_msgs::Path& path, int start_idx) {
    std::vector<double> cum_dist;
    cum_dist.push_back(0.0);
    double current_total = 0.0;
    for (int i = start_idx + 1; i < static_cast<int>(path.points.size()); ++i) {
        const auto& prev_pt = path.points[i-1].pose.position;
        const auto& curr_pt = path.points[i].pose.position;
        double dist = std::sqrt(std::pow(curr_pt.x - prev_pt.x, 2) + std::pow(curr_pt.y - prev_pt.y, 2));
        current_total += dist;
        cum_dist.push_back(current_total);
    }
    return cum_dist;
}

std::vector<double> ESOTracker2::linear_interpolate(const std::vector<double>& s_original, 
                                                            const std::vector<double>& val_original, 
                                                            const std::vector<double>& s_target) {
    std::vector<double> val_target(s_target.size(), val_original.empty() ? 0.0 : val_original[0]);
    if (s_original.size() < 2) return val_target;
   
    double s_min = s_original[0], s_max = s_original.back();
    for (size_t k = 0; k < s_target.size(); ++k) {
        double s_t = s_target[k];
        if (s_t <= s_min) { val_target[k] = val_original[0]; continue; }
        if (s_t >= s_max) { val_target[k] = val_original.back(); continue; }

        size_t i = 0;
        while (i < s_original.size() - 1 && s_original[i+1] < s_t) ++i;
        double ratio = (s_t - s_original[i]) / (s_original[i+1] - s_original[i]);
        val_target[k] = val_original[i] + ratio * (val_original[i+1] - val_original[i]);
    }
    return val_target;
}

casadi::DM ESOTracker2::interpolate_path_segment(const race_msgs::Path& path, const std::vector<double>& cum_dist, 
                                                         int start_idx, int end_idx,  const std::vector<double>& s_target, double yaw0) {
    std::vector<double> s_orig, x_orig, y_orig, theta_orig, kappa_orig;
    for (int i = start_idx; i <= end_idx; ++i) {
        const auto& pt = path.points[i];
        s_orig.push_back(cum_dist[i - start_idx]);
        x_orig.push_back(pt.pose.position.x);   
        y_orig.push_back(pt.pose.position.y);
        
        double yaw = quaternion_to_yaw(pt.pose.orientation);
        if (i == start_idx) yaw0 = yaw; 
        double diff = normalizeAngle(yaw - (theta_orig.empty() ? yaw0 : theta_orig.back()));
        theta_orig.push_back((theta_orig.empty() ? yaw0 : theta_orig.back()) + diff);
    }

    // 简单差分计算曲率 kappa
    kappa_orig.resize(theta_orig.size(), 0.0);
    for (size_t i = 1; i < theta_orig.size() - 1; ++i) {
        double ds = s_orig[i+1] - s_orig[i-1];
        kappa_orig[i] = (ds > 1e-4) ? (theta_orig[i+1] - theta_orig[i-1]) / ds : 0.0;
    }

    // 根据动态 s_target 进行插值
    auto x_interp = linear_interpolate(s_orig, x_orig, s_target);
    auto y_interp = linear_interpolate(s_orig, y_orig, s_target);
    auto theta_interp = linear_interpolate(s_orig, theta_orig, s_target);
    auto kappa_interp = linear_interpolate(s_orig, kappa_orig, s_target);

    int n_waypoints = s_target.size();
    casadi::DM waypoints = casadi::DM::zeros(4, n_waypoints);
    for (int i = 0; i < n_waypoints; ++i) {
        waypoints(0, i) = x_interp[i];
        waypoints(1, i) = y_interp[i];
        waypoints(2, i) = theta_interp[i];
        waypoints(3, i) = kappa_interp[i];
    }
    return waypoints;
}

casadi::DM ESOTracker2::process_race_path(const race_msgs::Path& input_path, const std::vector<double>& current_state) {
    int nearest_idx = find_nearest_path_point(current_state[0], current_state[1], input_path);
    if (nearest_idx == -1) return casadi::DM::zeros(4, nmpc_params_.N + 1);
    
    // 极低速保护
    double calc_vx = std::max(current_state[3], 0.1); 
    
    // 1. 生成基于实时车速的动态距离向量 s_target
    std::vector<double> s_target(nmpc_params_.N + 1);
    for (int i = 0; i <= nmpc_params_.N; ++i) {
        s_target[i] = calc_vx * nmpc_params_.dt * i;
    }

    // 2. 截取原始路径的最大长度 
    double max_dist = s_target.back() + 10.0; 
    
    std::vector<double> cum_dist = calculate_cumulative_distance(input_path, nearest_idx);
   
    int end_idx = nearest_idx;
    for (size_t i = 0; i < cum_dist.size(); ++i) {
        if (cum_dist[i] > max_dist) { end_idx = nearest_idx + i; break; }
        if (i == cum_dist.size() - 1) end_idx = nearest_idx + i;
    }
    end_idx = std::min(end_idx, static_cast<int>(input_path.points.size()) - 1);

    return interpolate_path_segment(input_path, cum_dist, nearest_idx, end_idx, s_target, current_state[2]);
}

// ---------------------- 核心算法（替换为挂车逻辑） ----------------------
// EKF横向速度估计（解决命名空间冲突 + 完整变量声明）
void ESOTracker2::ekfEstimateVy(double curr_vx, double curr_delta, double curr_ay, double curr_r, double M, double dt) {
    double a = nmpc_params_.lf;
    double b = nmpc_params_.lr;
    double c = b;
    double d = nmpc_params_.lt;
    double L2 = nmpc_params_.L2;
    double m1 = nmpc_params_.m;
    double m2 = nmpc_params_.m_t_total;
    double Iz1 = nmpc_params_.Iz;
    double Iz2 = nmpc_params_.Iz_t +  nmpc_params_.Kiz*(m2-nmpc_params_.m_t);
    double Cf = rls_Cf_est_;
    double Cr = rls_Cr_est_;
    double Ct = rls_Ct_est_;

    // --- 显式使用 Eigen:: 前缀，避免与 casadi::Matrix 冲突 ---
    Eigen::Matrix3d M_mat;
    M_mat << m1 + m2,  -m2*c,        -m2*d,
            -m2*c,     Iz1 + m2*c*c,  m2*c*d,
            -m2*d,     m2*c*d,        Iz2 + m2*d*d;

    Eigen::Matrix<double, 3, 4> K;
    K(0,0) = -(Cf+Cr+Ct)/curr_vx;
    K(0,1) = -(a*Cf - b*Cr - c*Ct)/curr_vx - (m1+m2)*curr_vx;
    K(0,2) = (L2*Ct)/curr_vx;
    K(0,3) = -Ct;
    K(1,0) = -(a*Cf - b*Cr - c*Ct)/curr_vx;
    K(1,1) = -(a*a*Cf + b*b*Cr + c*c*Ct)/curr_vx + m2*c*curr_vx;
    K(1,2) = -(c*L2*Ct)/curr_vx;
    K(1,3) = c*Ct;
    K(2,0) = (L2*Ct)/curr_vx;
    K(2,1) = -(c*L2*Ct)/curr_vx + m2*d*curr_vx;
    K(2,2) = -(L2*L2*Ct)/curr_vx;
    K(2,3) = L2*Ct;

    Eigen::Matrix<double, 3, 1> D;
    D << Cf, a*Cf, 0.0;

    Eigen::Matrix4d A_sys;
    A_sys.block(0,0,3,4) = M_mat.inverse() * K;
    A_sys.row(3) << 0, 1, -1, 0;
    Eigen::Vector4d B_sys;
    B_sys.block(0,0,3,1) = M_mat.inverse() * D;
    B_sys(3) = 0.0;

    // EKF预测步骤
    Eigen::Vector4d x_dot = A_sys * ekf_x_hat_ + B_sys * curr_delta;
    Eigen::Vector4d x_pred = ekf_x_hat_ + x_dot * dt;
    Eigen::Matrix4d Phi = Eigen::Matrix4d::Identity() + A_sys * dt;
    Eigen::Matrix4d Q;
    Q << 0.005,0,0,0, 0,0.01,0,0, 0,0,0.05,0, 0,0,0,0.01;
    Eigen::Matrix4d P_pred = Phi * ekf_P_ * Phi.transpose() + Q;

    // --- 完整定义 H_sys 和 K_gain，确保在作用域内 ---
    Eigen::Matrix<double, 2, 4> H_sys;
    // 关键修正：使用 RowVector4d（行向量）
    H_sys.row(0) = A_sys.row(0) + Eigen::RowVector4d(0, curr_vx, 0, 0); 
    H_sys.row(1) << 0, 1, 0, 0; // 观测r
    
    Eigen::Vector2d D_obs;
    D_obs << B_sys(0) * curr_delta, 0.0;

    Eigen::Vector2d z_pred = H_sys * x_pred + D_obs;
    Eigen::Vector2d z_meas;
    z_meas << curr_ay, curr_r;
    Eigen::Matrix2d R;
    R << 1.0,0, 0,0.05;
    
    // 完整定义 K_gain
    Eigen::Matrix<double, 4, 2> K_gain = P_pred * H_sys.transpose() * (H_sys * P_pred * H_sys.transpose() + R).inverse();

    // 状态与协方差更新
    ekf_x_hat_ = x_pred + K_gain * (z_meas - z_pred);
    ekf_P_ = (Eigen::Matrix4d::Identity() - K_gain * H_sys) * P_pred;
}

// FF-RLS侧偏刚度辨识（保留原函数名，替换为3刚度辨识）
void ESOTracker2::rlsIdentifyStiffness(double curr_vx, double vy_est, double curr_delta,double curr_r, 
                                      double curr_ay, double curr_gamma, double curr_r_t, double M, double dt) {
    if (curr_vx < 1.0) {
        rls_Cf_est_ = rls_Cf_est_default_;
        rls_Cr_est_ = rls_Cr_est_default_;
        rls_Ct_est_ = rls_Ct_est_default_;
        rls_P_ = Matrix3d::Identity() * 1e6;
        return;
    }

    double lamda = 0.99999; // 遗忘因子，接近1表示慢速遗忘
    double lf1 = nmpc_params_.lf;
    double lr1 = nmpc_params_.lr;
    double lf2 = nmpc_params_.lt;
    double lr2 = nmpc_params_.L2 - lf2;
    double lh = nmpc_params_.lh;
    double m1 = nmpc_params_.m;
    double m2 = nmpc_params_.m_t_total;
    double Iz1 = nmpc_params_.Iz;
    double Iz2 = nmpc_params_.Iz_t + nmpc_params_.Kiz*(m2-nmpc_params_.m_t);

    // 横摆角加速度滤波
    double alpha_filter = 0.3;
    double w1_dot_raw = (curr_r - rls_w1_prev_) / dt;
    double w2_dot_raw = (curr_r_t - rls_w2_prev_) / dt;
    double w1_dot = alpha_filter * w1_dot_raw + (1 - alpha_filter) * rls_w1_dot_prev_;
    double w2_dot = alpha_filter * w2_dot_raw + (1 - alpha_filter) * rls_w2_dot_prev_;

    rls_w1_prev_ = curr_r;
    rls_w2_prev_ = curr_r_t;
    rls_w1_dot_prev_ = w1_dot;
    rls_w2_dot_prev_ = w2_dot;

    // 求解侧向力
    Matrix3d A;
    A << cos(curr_delta), 1, cos(curr_gamma),
         lf1*cos(curr_delta), -lr1, lh*lf2*cos(curr_gamma)/lr2,
         cos(curr_delta), 1, -lr2*cos(curr_gamma)/lf2;
    
    Vector3d B;
    B << m1*curr_ay + cos(curr_gamma)*m2*(curr_ay - lh*w1_dot - lf2*w2_dot),
         Iz1*w1_dot - lh*cos(curr_gamma)*Iz2*w2_dot/lf1,
         m1*curr_ay + Iz2*w2_dot/lf2;

    Vector3d Fy = A.inverse() * B;

    // 计算侧偏角
    Matrix2d rot;
    rot << cos(curr_gamma), sin(curr_gamma),
          -sin(curr_gamma), cos(curr_gamma);
    Vector2d b_vec;
    b_vec << lf2*curr_r_t*sin(curr_gamma) - curr_vx,
             vy_est - lh*curr_r - lf2*curr_r_t*cos(curr_gamma);
    double vx1_safe = max(curr_vx, 1.0);
    Vector2d V2 = rot.inverse() * b_vec;
    double V2_x_safe = max(V2(0), 1.0);

    double alpha1 = curr_delta - atan((vy_est + lf1*curr_r)/vx1_safe);
    double alpha2 = -atan((vy_est - lr1*curr_r)/vx1_safe);
    double alpha3 = -atan((V2(1) - lr2*curr_r_t)/V2_x_safe);
    Vector3d alpha(alpha1, alpha2, alpha3);

    // RLS递推
    // 建议保留条件判断中的绝对值（cwiseAbs 和 abs），以确保左转和右转都能正常触发递推
    double max_alpha = alpha.cwiseAbs().maxCoeff();
    if (max_alpha > 0.01 && max_alpha < 0.15 && std::abs(curr_ay) > 0.1) {
        
        // 【修改点】：去掉了侧向力和侧偏角的绝对值，使用带符号的原始数据进行计算
        Vector3d Y = Fy; 
        Matrix3d Phi = alpha.asDiagonal();
        // 增益矩阵计算
        Matrix3d K = rls_P_ * Phi.transpose() * (lamda*Matrix3d::Identity() + Phi*rls_P_*Phi.transpose()).inverse();
        // 误差计算（带符号）
        Vector3d error = Y - Phi * Vector3d(rls_Cf_est_, rls_Cr_est_, rls_Ct_est_);
        // 状态更新
        rls_Cf_est_ += K(0,0)*error(0) + K(0,1)*error(1) + K(0,2)*error(2);
        rls_Cr_est_ += K(1,0)*error(0) + K(1,1)*error(1) + K(1,2)*error(2);
        rls_Ct_est_ += K(2,0)*error(0) + K(2,1)*error(1) + K(2,2)*error(2);
        // 物理限幅
        rls_Cf_est_ = std::max(std::min(rls_Cf_est_, rls_Cf_est_max_), rls_Cf_est_min_);
        rls_Cr_est_ = std::max(std::min(rls_Cr_est_, rls_Cr_est_max_), rls_Cr_est_min_);
        rls_Ct_est_ = std::max(std::min(rls_Ct_est_, rls_Ct_est_max_), rls_Ct_est_min_);
        // 协方差矩阵更新
        rls_P_ = (Matrix3d::Identity() - K*Phi) * rls_P_ / lamda;
    }

    // 输出平滑
    double smooth_factor = 0.1;
    rls_C_out_prev_ = smooth_factor * Vector3d(rls_Cf_est_, rls_Cr_est_, rls_Ct_est_) + (1 - smooth_factor) * rls_C_out_prev_;
    rls_Cf_est_ = rls_C_out_prev_(0);
    rls_Cr_est_ = rls_C_out_prev_(1);
    rls_Ct_est_ = rls_C_out_prev_(2);
    
    // 输出侧偏刚度估计值（橙色）
    ROS_INFO("[%s] \033[33m侧偏刚度估计: Cf=%.1f N/rad, Cr=%.1f N/rad, Ct=%.1f N/rad\033[0m", 
                      getName().c_str(), rls_Cf_est_, rls_Cr_est_, rls_Ct_est_);
}

// ESO观测器（保留原函数名，更新为Matlab实现）
void ESOTracker2::esoCompute(double curr_r, double curr_delta, double dt) {
    double Iz1 = nmpc_params_.Iz;
    double Cf = rls_Cf_est_;
    double lf = nmpc_params_.lf;
    double B1 = 20.0;
    double B2 = 100.0;

    double b_eso = (Cf * lf) / Iz1;
    double error_eso = curr_r - eso_x1_;

    eso_x1_ += (b_eso * curr_delta + B1 * error_eso + eso_x2_) * dt;
    eso_x2_ += (B2 * error_eso) * dt;
}


// 预瞄曲率与运动学转角（新增内部函数）
double ESOTracker2::calculate_curvature_and_steering(const Vector2d& p1, const Vector2d& p2, 
                                                  const Vector2d& p3, double L1) {
    double x1 = p1(0), y1 = p1(1);
    double x2 = p2(0), y2 = p2(1);
    double x3 = p3(0), y3 = p3(1);

    double a = sqrt(pow(x2-x1,2) + pow(y2-y1,2));
    double b = sqrt(pow(x3-x2,2) + pow(y3-y2,2));
    double c = sqrt(pow(x3-x1,2) + pow(y3-y1,2));

    if (a < 1e-6 || b < 1e-6 || c < 1e-6) return 0.0;

    double cross_prod = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    double kappa = (2 * cross_prod) / (a * b * c);
    return atan(L1 * kappa);
}

// 挂车运动学状态预估（新增内部函数）
void ESOTracker2::calculate_trailer_kinematics(double delta_f, double curr_vx, double curr_r, double dt) {
    double L1 = nmpc_params_.lf + nmpc_params_.lr;
    double L2 = nmpc_params_.L2;
    double Lh = 0.0;
    double vx = max(curr_vx, 0.5);
    double Tau = 0.15;
    double K = 1.0;

    double r_raw = (vx * tan(delta_f)) / L1;
    double alpha = dt / (Tau + dt);
    double r_filtered = (1 - alpha) * curr_r + alpha * K * r_raw;

    r_t_ = (vx * sin(gamma_) + Lh * r_filtered * cos(gamma_)) / L2;
    double gamma_dot = r_filtered - r_t_;
    gamma_ += gamma_dot * dt;
}

MX ESOTracker2::vehicleDynamicsModel(const MX& state, const MX& cmd_delta,
                                     const MX& vx, const MX& h_dist,
                                     const MX& dyn_params) {
    MX theta = state(2), vy = state(3), r = state(4), delta = state(5);
    MX r_t = state(6), gamma = state(7);

    MX m1 = dyn_params(0), Iz1 = dyn_params(1), lf = dyn_params(2), lr = dyn_params(3);
    MX Cf = dyn_params(4), Cr = dyn_params(5), M = dyn_params(6), Iz2 = dyn_params(7);
    MX lt = dyn_params(8), Ct = dyn_params(9), L2 = dyn_params(10);
    MX m2 = M - m1;

    MX vx_safe = fmax(vx, 0.5); 
    MX cos_gamma = fmax(cos(gamma), 0.005); 
    MX sin_gamma = sin(gamma);
    // 1. 轮胎侧偏角与力计算
    MX alpha_f = delta - atan2((vy + lf * r), vx_safe);
    MX alpha_r = -atan2((vy - lr * r), vx_safe);
    MX Fyf = Cf * alpha_f;
    MX Fyr = Cr * alpha_r;
    // 挂车车轴处侧偏力
    MX vy_h1 = vy - lr * r; 
    MX vy_axle = -vx_safe * sin_gamma + vy_h1 * cos_gamma - L2 * r_t; 
    MX alpha_t = -atan2(vy_axle, vx_safe);
    MX Fyt = Ct * alpha_t;
    // 牵引车自身的合外力与力矩
    MX Y1 = Fyf * cos(delta) + Fyr;
    MX N1 = lf * Fyf * cos(delta) - lr * Fyr;
    // 2. 铰接力 Hy 分离推导 (Hy = Hy_const + Hy_dyn * d_r_t)
    MX Hy_const = -((L2 - lt) * Fyt) / (lt * cos_gamma);
    MX Hy_dyn   = -Iz2 / (lt * cos_gamma);
    // 3. 牵引车加速度分离推导 
    // 横移加速度 d_vy = d_vy_const + d_vy_dyn * d_r_t
    MX d_vy_const = (Y1 - m1 * vx * r + Hy_const) / m1;
    MX d_vy_dyn   = Hy_dyn / m1;
    // 横摆加速度 d_r = d_r_const + d_r_dyn * d_r_t
    MX d_r_const = (N1 - lr * Hy_const) / Iz1;
    MX d_r_dyn   = (-lr * Hy_dyn) / Iz1;
    // 挂车因速度产生的向心加速度外项
    MX a_y2_ext = vx_safe * r * cos_gamma + vy_h1 * r * sin_gamma;
    // 平衡方程右侧常数项基础值 (由 Fyt 和向心力组成)
    MX RHS_trailer = (L2 / lt) * Fyt - m2 * a_y2_ext;
    // 将 d_vy 和 d_r 代入平移方程，提取出 d_r_t 的系数
    MX coeff_drt = m2 * cos_gamma * d_vy_dyn 
                 - m2 * lr * cos_gamma * d_r_dyn 
                 - (m2 * lt + Iz2 / lt);

    // 将 d_vy 和 d_r 的常数部分移到方程右侧，计算最终的方程右边(RHS)
    MX RHS_total = RHS_trailer 
                 - m2 * cos_gamma * d_vy_const 
                 + m2 * lr * cos_gamma * d_r_const;

    // 解得挂车横摆加速度
    MX d_r_t = RHS_total / coeff_drt;

    // 5. 回代求取牵引车加速度
    MX d_vy = d_vy_const + d_vy_dyn * d_r_t;
    MX d_r  = d_r_const + d_r_dyn * d_r_t + h_dist*0; // 加入ESO或外部扰动

    // 6. 运动学状态更新
    // MX d_gamma = r - r_t;
    MX d_gamma = r_t - r; // 挂车相对转角变化率
    MX d_x = vx * cos(theta) - vy * sin(theta);
    MX d_y = vx * sin(theta) + vy * cos(theta);
    MX d_theta = r;
    MX d_delta = (cmd_delta - delta) / nmpc_params_.T_lag;

    std::vector<MX> state_derivatives = {d_x, d_y, d_theta, d_vy, d_r, d_delta, d_r_t, d_gamma};

    return vertcat(state_derivatives);
}

// 构建NMPC求解器（替换为挂车约束，保留函数名）
void ESOTracker2::buildNMPSolver() {
    solver_.opti = Opti();
    int nx = nmpc_params_.nx, nu = nmpc_params_.nu, N = nmpc_params_.N, Nc = nmpc_params_.Nc;

    solver_.X = solver_.opti.variable(nx, N+1);
    solver_.U_sparse = solver_.opti.variable(nu, Nc);
    solver_.P_x0 = solver_.opti.parameter(nx);
    solver_.P_waypoints = solver_.opti.parameter(4, N+1);  
    solver_.P_vx = solver_.opti.parameter(1);
    solver_.P_u_prev = solver_.opti.parameter(1);
    solver_.P_h_hat = solver_.opti.parameter(1);
    solver_.P_dyn_params = solver_.opti.parameter(11); // 扩展为11维动力学参数

    // 控制量稀疏化
    MX U_full = MX::zeros(nu, N);
    int base_steps = N / Nc, remainder = N % Nc, current_idx = 0;
    for (int i=0; i<Nc; i++) {
        int steps = base_steps + (i == Nc-1 ? remainder : 0);
        int end_idx = min(current_idx + steps, N);
        U_full(Slice(), Slice(current_idx, end_idx)) = repmat(solver_.U_sparse(Slice(), i), 1, end_idx - current_idx);
        current_idx = end_idx;
    }

    // //转角变化率约束（新增）
    // double max_dU = nmpc_params_.delta_rate_max * nmpc_params_.dt;
    // double min_dU = nmpc_params_.delta_rate_min * nmpc_params_.dt;
    // solver_.opti.subject_to(solver_.opti.bounded(min_dU, solver_.U_sparse(0) - solver_.P_u_prev, max_dU));
    // for (int i=1; i<Nc; i++) {
    //     solver_.opti.subject_to(solver_.opti.bounded(min_dU, solver_.U_sparse(i) - solver_.U_sparse(i-1), max_dU));
    // }

    MX J = 0.0;
    solver_.opti.subject_to(solver_.X(Slice(), 0) == solver_.P_x0);

    for (int k=0; k<N; k++) {
        MX st = solver_.X(Slice(), k), con = U_full(Slice(), k);
        MX h = solver_.P_h_hat * pow(0.85, k);

        if (integration_grade_ >= 0.5 && integration_grade_ < 1.5) { // Euler前向积分
            solver_.opti.subject_to(solver_.X(Slice(), k+1) == st + nmpc_params_.dt * vehicleDynamicsModel(st, con, solver_.P_vx, h, solver_.P_dyn_params));
        }

        else if (integration_grade_ >= 1.5 && integration_grade_ < 2.5) { // RK2积分
            // RK2积分（新增）
            MX k1 = vehicleDynamicsModel(st, con, solver_.P_vx, h, solver_.P_dyn_params);
            MX k2 = vehicleDynamicsModel(st + nmpc_params_.dt / 2.0 * k1, con, solver_.P_vx, h, solver_.P_dyn_params);
            solver_.opti.subject_to(solver_.X(Slice(), k+1) == st + nmpc_params_.dt * k2);
        }
        else if (integration_grade_ >= 3.5 && integration_grade_ < 4.5) { // RK4积分
            // RK4积分（保留原逻辑）
            MX k1 = vehicleDynamicsModel(st, con, solver_.P_vx, h, solver_.P_dyn_params);
            MX k2 = vehicleDynamicsModel(st + nmpc_params_.dt/2 * k1, con, solver_.P_vx, h, solver_.P_dyn_params);
            MX k3 = vehicleDynamicsModel(st + nmpc_params_.dt/2 * k2, con, solver_.P_vx, h, solver_.P_dyn_params);
            MX k4 = vehicleDynamicsModel(st + nmpc_params_.dt * k3, con, solver_.P_vx, h, solver_.P_dyn_params);
            solver_.opti.subject_to(solver_.X(Slice(), k+1) == st + nmpc_params_.dt/6 * (k1 + 2*k2 + 2*k3 + k4));
        }
        else {
            // 报错：integration_grade_不在有效范围内，无法选择积分方法
            throw std::runtime_error("integration_grade_不在有效范围内，无法选择积分方法");
        }

        // 代价函数（扩展挂车项）
        MX ref_x = solver_.P_waypoints(0, k+1), ref_y = solver_.P_waypoints(1, k+1);
        MX ref_theta = solver_.P_waypoints(2, k+1), ref_kappa = solver_.P_waypoints(3, k+1);
        MX e_theta = atan2(sin(solver_.X(2, k+1) - ref_theta), cos(solver_.X(2, k+1) - ref_theta));
        MX r_ref = solver_.P_vx * ref_kappa;
        MX gamma_dot_actual = solver_.X(4, k+1) - solver_.X(6, k+1);

        J += nmpc_params_.Q(0,0) * (pow(solver_.X(0, k+1) - ref_x, 2) + pow(solver_.X(1, k+1) - ref_y, 2));
        J += nmpc_params_.Q(2,2) * pow(e_theta, 2);
        J += nmpc_params_.Q(4,4) * pow(solver_.X(4, k+1) - r_ref, 2);
        J += nmpc_params_.Q(3,3) * pow(solver_.X(3, k+1), 2);
        // J += nmpc_params_.Q_gamma * pow(solver_.X(7, k+1), 2);
        // J += nmpc_params_.Q_r_t * pow(solver_.X(6, k+1) - r_ref, 2);
        // J += nmpc_params_.Q_gamma_rate * pow(gamma_dot_actual, 2);
        J += nmpc_params_.R * pow(con, 2);
    }
    // 控制量平滑项（保留原逻辑）
    J += nmpc_params_.dR * pow(solver_.U_sparse(0) - solver_.P_u_prev, 2);
    for (int i=1; i<Nc; i++) J += nmpc_params_.dR * pow(solver_.U_sparse(i) - solver_.U_sparse(i-1), 2);

    // 硬约束（保留原逻辑）
    solver_.opti.subject_to(solver_.opti.bounded(nmpc_params_.delta_min, solver_.U_sparse, nmpc_params_.delta_max));

    solver_.opti.minimize(J);

    // IPOPT参数（Matlab实时性优化）
    Dict opts = {
        {"ipopt.print_level", 0}, 
        {"ipopt.sb", "yes"}, 
        {"ipopt.max_iter", ipopt_max_iter_},
        {"ipopt.tol", 1e-3},
        {"ipopt.acceptable_tol", ipopt_acceptable_tol_},
        {"ipopt.acceptable_iter", ipopt_acceptable_iter_},
        {"ipopt.mu_strategy", "adaptive"},
        {"print_time", 0},
        {"expand", true},

        {"ipopt.warm_start_init_point", "yes"},        // 明确告诉求解器接受热启动
        {"ipopt.warm_start_bound_push", ipopt_warm_start_bound_push_},        
        {"ipopt.warm_start_slack_bound_push", ipopt_warm_start_slack_bound_push_},
        {"ipopt.warm_start_mult_bound_push", ipopt_warm_start_mult_bound_push_}, 
        
        {"print_time", 0}
    };
    solver_.opti.solver("ipopt", opts);
}

// 求解NMPC（修正 isEmpty 用法）
bool ESOTracker2::solveNMPC(const std::vector<double>& current_state, const casadi::DM& waypoints,
                          std::vector<double>& control_output) {

    auto start_time = std::chrono::high_resolution_clock::now();//记录开始的时间

    try {
        solver_.opti.set_value(solver_.P_x0, current_state);
        solver_.opti.set_value(solver_.P_waypoints, waypoints);
        solver_.opti.set_value(solver_.P_u_prev, current_cmd_);

         if (solver_.has_prev_sol && solver_.sol_prev) {
            solver_.opti.set_initial(solver_.X, solver_.sol_prev->value(solver_.X));
            solver_.opti.set_initial(solver_.U_sparse, solver_.sol_prev->value(solver_.U_sparse));

            solver_.opti.set_initial(solver_.opti.lam_g(), solver_.sol_prev->value(solver_.opti.lam_g()));
        }

        casadi::OptiSol sol = solver_.opti.solve();

        auto end_time = std::chrono::high_resolution_clock::now();//打印求解耗时
        std::chrono::duration<double, std::milli> elapsed = end_time - start_time;
        ROS_INFO("\033[32m[%s] NMPC 求解成功! 耗时: %.2f ms\033[0m", getName().c_str(), elapsed.count());
        
        solver_.sol_prev = std::make_unique<casadi::OptiSol>(sol);
        solver_.has_prev_sol = true;


        control_output[0] = static_cast<double>(sol.value(solver_.U_sparse(0)));
        return true;
    } catch (std::exception& e) {

         auto end_time = std::chrono::high_resolution_clock::now();//求解失败的耗时
       std::chrono::duration<double, std::milli> elapsed = end_time - start_time;
       ROS_WARN("\033[31m[%s] NMPC 求解失败! 耗时: %.2f ms, 原因: %s\033[0m", getName().c_str(), elapsed.count(), e.what());
        solver_.has_prev_sol = false;
        solver_.sol_prev = nullptr;
        return false;
    }
}

// ---------------------- [新增] 纯跟踪兜底保护实现 ----------------------
double ESOTracker2::computePurePursuitSteering(const race_msgs::Path& path,
                                               double curr_x, double curr_y,
                                               double curr_theta, double lookahead_dist) {
    if (path.points.empty()) return 0.0;

    // 1. 找最近点
    int nearest_idx = find_nearest_path_point(curr_x, curr_y, path);
    int target_idx = nearest_idx;
    
    // 2. 向前搜索预瞄距离内的目标点
    for (int i = nearest_idx; i < static_cast<int>(path.points.size()); ++i) {
        double dx = path.points[i].pose.position.x - curr_x;
        double dy = path.points[i].pose.position.y - curr_y;
        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist >= lookahead_dist) {
            target_idx = i;
            break;
        }
    }
    // 如果列表走完还没够到预瞄距离，就用最后一个点
    if (target_idx == nearest_idx) {
        target_idx = path.points.size() - 1;
    }

    // 3. 坐标转换到车体坐标系
    const auto& target_pt = path.points[target_idx].pose.position;
    double dx = target_pt.x - curr_x;
    double dy = target_pt.y - curr_y;
    
    // 旋转矩阵：将世界坐标误差转换为车体坐标
    double local_x = std::cos(curr_theta) * dx + std::sin(curr_theta) * dy;
    double local_y = -std::sin(curr_theta) * dx + std::cos(curr_theta) * dy;

    // 4. 纯跟踪公式计算
    double L = nmpc_params_.lf + nmpc_params_.lr; // 牵引车轴距
    double ld = std::max(lookahead_dist, std::sqrt(local_x*local_x + local_y*local_y));
    
    // 反正切计算前轮转角
    double delta_pp = std::atan2(2.0 * L * local_y, ld * ld);

    // 5. 限幅保护
    return std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, delta_pp));
}


} // namespace race_tracker

// 保留原插件注册（完全不变）
PLUGINLIB_EXPORT_CLASS(race_tracker::ESOTracker2, race_tracker::ControllerPluginBase)
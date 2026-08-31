#include "race_tracker/eso_tracker2.h"
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <numeric>
#include <limits>
#include <stdexcept>
#include <chrono>
#include <algorithm>  // for std::clamp
#include <array>
#include <cmath>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>


using namespace casadi;
using namespace Eigen;
using namespace std;

namespace race_tracker {

// 构造函数
ESOTracker2::ESOTracker2() {
    // 三自由度 CKF：x=[vy, r1, r2, gamma]^T。
    ckf_x_hat_ = Vector4d::Zero();
    ckf_P_ = Matrix4d::Identity() * 1.0;
    rls_P_ = Matrix3d::Identity() * 1e4;
    eso_x1_ = 0.0;
    eso_x2_ = 0.0;
    eso_initialized_ = false;
    gamma_ = 0.0;
    r_t_ = 0.0;
    rls_w2_prev_ = 0.0;
    rls_w2_dot_prev_ = 0.0;
    r_tractor_filt_ = 0.0;
    r_filter_initialized_ = false;
    current_cmd_ = 0.0;
    solver_.has_prev_sol = false;
    solver_.sol_prev = nullptr;
    blend_alpha_ = 0.0;
    nmpc_safe_cmd_ = 0.0;
    supervisor_params_.startup_time = 0.1;
    supervisor_params_.blend_speed_low = 4.1667;
    supervisor_params_.blend_speed_high = 5.0;
    supervisor_params_.standstill_speed = 0.05;
    supervisor_params_.nmpc_speed_floor = 1.0;
    start_time_ = ros::Time(0);
    last_control_time_ = ros::Time(0);
    fallback_enter_time_ = ros::Time(0);
}

void ESOTracker2::captureNmpcSolverStats() {
    last_nmpc_iter_count_ = -1;
    try {
        const casadi::Dict stats = solver_.opti.stats();
        last_nmpc_return_status_ = casadi::get_from_dict<std::string>(
            stats, "return_status", std::string("unknown"));
        last_nmpc_iter_count_ = casadi::get_from_dict<int>(stats, "iter_count", -1);
    } catch (const std::exception&) {
        last_nmpc_return_status_ = "stats_unavailable";
    }
    for (char& ch : last_nmpc_return_status_) {
        if (ch == ',' || ch == '\n' || ch == '\r') ch = ';';
    }
}

void ESOTracker2::enterFallback(int reason_code, const ros::Time& now) {
    fallback_latched_ = true;
    fallback_reentry_active_ = false;
    fallback_reentry_alpha_ = 0.0;
    fallback_reason_code_ = reason_code;
    fallback_enter_time_ = now;
    nmpc_success_streak_ = 0;
}

void ESOTracker2::drivingModeCallback(const std_msgs::Int32::ConstPtr& msg) {
    if (!msg) return;
    latest_driving_mode_ = msg->data;
    driving_mode_received_ = true;
    driving_mode_stamp_ = ros::Time::now();
}

// 插件初始化
bool ESOTracker2::initialize(ros::NodeHandle& nh) {
    ros::NodeHandle nh_nmpc(nh, "eso_tracker2");
    ROS_INFO("[%s] NMPC 控制器命名空间: %s", getName().c_str(), nh_nmpc.getNamespace().c_str());

    // 加载NMPC核心参数
    nh_nmpc.param("nx", nmpc_params_.nx, 8);
    nh_nmpc.param("nu", nmpc_params_.nu, 1);
    nh_nmpc.param("prediction_step", nmpc_params_.N, 25);
    nh_nmpc.param("sparse_control_step", nmpc_params_.Nc, 8);
    nh_nmpc.param("sampling_time", nmpc_params_.dt, 0.05);
    nh_nmpc.param("integration_grade", nmpc_params_.integration_grade, 2.0);
    nh_nmpc.param("eso_disturbance_decay", nmpc_params_.eso_disturbance_decay, 0.95);
    nh_nmpc.param("eso_disturbance_time_constant", nmpc_params_.eso_disturbance_tau_s, 0.974786);
    nh_nmpc.param("near_dense_control_steps", nmpc_params_.near_dense_control_steps, 4);
    nmpc_params_.eso_disturbance_decay = std::max(
        0.0, std::min(1.0, nmpc_params_.eso_disturbance_decay));
    if (nmpc_params_.eso_disturbance_tau_s > 0.0) {
        nmpc_params_.eso_disturbance_decay =
            std::exp(-nmpc_params_.dt / nmpc_params_.eso_disturbance_tau_s);
    }
    if (nmpc_params_.nx != 8 || nmpc_params_.nu != 1 || nmpc_params_.N <= 0 ||
        nmpc_params_.Nc <= 0 || nmpc_params_.Nc > nmpc_params_.N ||
        !std::isfinite(nmpc_params_.dt) || nmpc_params_.dt <= 0.0) {
        ROS_ERROR("[%s] NMPC基础配置无效: nx=%d nu=%d N=%d Nc=%d dt=%.6f",
                  getName().c_str(), nmpc_params_.nx, nmpc_params_.nu,
                  nmpc_params_.N, nmpc_params_.Nc, nmpc_params_.dt);
        return false;
    }
    // 加载—— 车辆核心动力学参数（含挂车） ——
    nh_nmpc.param("m", nmpc_params_.m, 10000.0);
    nh_nmpc.param("Iz", nmpc_params_.Iz, 50000.0);
    nh_nmpc.param("lf", nmpc_params_.lf, 2.0);
    nh_nmpc.param("lr", nmpc_params_.lr, 2.135);
    nh_nmpc.param("m_t_empty", nmpc_params_.m_t, 7570.0);
    nh_nmpc.param("Iz_t_empty", nmpc_params_.Iz_t, 150000.0);
    nh_nmpc.param("m_t_total", nmpc_params_.m_t_total, 29500.0);
    nh_nmpc.param("lt", nmpc_params_.lt, 3.4);
    nh_nmpc.param("L2", nmpc_params_.L2, 7.9);
    nh_nmpc.param("lh", nmpc_params_.lh, 0.0);
    nh_nmpc.param("T_lag", nmpc_params_.T_lag, 0.25);
    m_t_empty_default_ = nmpc_params_.m_t;

    // 加载—— 控制量边界约束 ——
    nh_nmpc.param("min_steer", nmpc_params_.delta_min, -0.5);
    nh_nmpc.param("max_steer", nmpc_params_.delta_max, 0.5);
    nh_nmpc.param("delta_rate_max", nmpc_params_.delta_rate_max, 0.35);
    nh_nmpc.param("delta_rate_min", nmpc_params_.delta_rate_min, -0.35);
    nmpc_params_.delta_rate_max = std::max(1e-3, std::abs(nmpc_params_.delta_rate_max));
    nmpc_params_.delta_rate_min = -std::max(1e-3, std::abs(nmpc_params_.delta_rate_min));
    nh_nmpc.param("nmpc_solve_deadline_ms", nmpc_solve_deadline_ms_, 50.0);
    nh_nmpc.param("nmpc_ipopt_cpu_time_limit_ms", nmpc_ipopt_cpu_time_limit_ms_, 45.0);
    nh_nmpc.param("enforce_final_output_rate_limit", enforce_final_output_rate_limit_, true);
    nh_nmpc.param("publish_steering_angle_velocity", publish_steering_angle_velocity_, true);
    nh_nmpc.param("steering_angle_velocity_cmd_radps", steering_angle_velocity_cmd_radps_,
                  nmpc_params_.delta_rate_max);
    nmpc_solve_deadline_ms_ = std::max(1.0, nmpc_solve_deadline_ms_);
    nmpc_ipopt_cpu_time_limit_ms_ = std::max(
        1.0, std::min(nmpc_ipopt_cpu_time_limit_ms_, nmpc_solve_deadline_ms_));
    steering_angle_velocity_cmd_radps_ = std::max(
        0.0, std::min(steering_angle_velocity_cmd_radps_, nmpc_params_.delta_rate_max));

    // 加载—— 代价函数权重 ——
    nh_nmpc.param("Q_x", nmpc_params_.Q(0,0), 1.0);              // 纵向位置误差权重
    nh_nmpc.param("Q_y", nmpc_params_.Q(1,1), 10000.0);          // 纵向位置误差权重
    nh_nmpc.param("Q_theta", nmpc_params_.Q(2,2), 104000.0);     // 航向角误差权重
    nh_nmpc.param("Q_vy", nmpc_params_.Q(3,3), 1.0);             // 侧向速度权重
    nh_nmpc.param("Q_r", nmpc_params_.Q(4,4), 200.0);            // 横摆角速度权重
    nh_nmpc.param("Q_delta", nmpc_params_.Q(5,5), 1.0);          // 实际转角状态权重

    nh_nmpc.param("Q_r_t", nmpc_params_.Q(6,6), 300.0);          // 挂车横摆率权重
    nh_nmpc.param("Q_gamma", nmpc_params_.Q(7,7), 500.0);        // 铰接角权重
    nh_nmpc.param("Q_dgamma", nmpc_params_.dgamma, 1000.0);      // 铰接角速度权重

    nh_nmpc.param("Q_R", nmpc_params_.R, 20.0);                  // 前馈外反馈修正权重
    nh_nmpc.param("Q_dR", nmpc_params_.dR, 4000000.0);           // 实际控制增量权重

    //  —— 参数估计中参数 ——
    nh_nmpc.param("Kiz", nmpc_params_.Kiz, 10.0);      // 横摆转动惯量比例系数

    // 加载—— 参数估计中参数 ——
    nh_nmpc.param("rls_Cf_est", rls_Cf_est_default_, 270000.0);          // 牵引车前轴侧偏刚度初始值（N/rad）
    nh_nmpc.param("rls_Cr_est", rls_Cr_est_default_, 1500000.0);         // 牵引车后轴侧偏刚度初始值（N/rad）
    nh_nmpc.param("rls_Ct_est", rls_Ct_est_default_, 500000.0);          // 挂车轮胎侧偏刚度初始值（N/rad）
    nh_nmpc.param("rls_w1_prev", rls_w1_prev_, 0.0);              // 挂车横摆角初始值（rad）
    nh_nmpc.param("rls_w1_dot_prev", rls_w1_dot_prev_, 0.0);        // 挂车横摆角速度初始值（rad/s）
    nh_nmpc.param("rls_Cf_est_max", rls_Cf_est_max_, 270000.0);
    nh_nmpc.param("rls_Cr_est_max", rls_Cr_est_max_, 2000000.0);          // 挂车后轴刚度最大值（N/m）
    nh_nmpc.param("rls_Ct_est_max", rls_Ct_est_max_, 1000000.0);
    nh_nmpc.param("rls_Cf_est_min", rls_Cf_est_min_, 270000.0);
    nh_nmpc.param("rls_Cr_est_min", rls_Cr_est_min_, 1200000.0);
    nh_nmpc.param("rls_Ct_est_min", rls_Ct_est_min_, 400000.0);

    //  —— IPOPT求解器参数 ——
    nh_nmpc.param("ipopt_max_iter", ipopt_max_iter_, 50);                         // IPOPT最大迭代次数
    nh_nmpc.param("ipopt_acceptable_tol", ipopt_acceptable_tol_, 5e-2);
    nh_nmpc.param("ipopt_acceptable_iter", ipopt_acceptable_iter_, 3);
    nh_nmpc.param("ipopt_warm_start_bound_push", ipopt_warm_start_bound_push_, 1e-3);        // IPPT预热边界推送系数，默认1e-3，用于初始化控制量边界，热启动时，将初始点向变量边界 “推” 的最小距离
    nh_nmpc.param("ipopt_warm_start_slack_bound_push", ipopt_warm_start_slack_bound_push_, 1e-3);  // IPPT预热边界松弛系数，默认1e-3，用于初始化控制量边界,针对松弛变量
    nh_nmpc.param("ipopt_warm_start_mult_bound_push", ipopt_warm_start_mult_bound_push_, 1e-3);   // IPPT预热边界松弛系数，默认1e-3，用于初始化控制量边界,针对拉格朗日乘子

    // 加载—— 求解器设定 ——
    // integration_grade 已在核心参数区加载。

    rls_Cf_est_ = rls_Cf_est_default_; // 将默认参数赋予变量
    rls_Cr_est_ = rls_Cr_est_default_;
    rls_Ct_est_ = rls_Ct_est_default_;
    if (rls_Cf_est_min_ > rls_Cf_est_max_) std::swap(rls_Cf_est_min_, rls_Cf_est_max_);
    if (rls_Cr_est_min_ > rls_Cr_est_max_) std::swap(rls_Cr_est_min_, rls_Cr_est_max_);
    if (rls_Ct_est_min_ > rls_Ct_est_max_) std::swap(rls_Ct_est_min_, rls_Ct_est_max_);
    rls_Cf_est_ = std::max(rls_Cf_est_min_, std::min(rls_Cf_est_max_, rls_Cf_est_));
    rls_Cr_est_ = std::max(rls_Cr_est_min_, std::min(rls_Cr_est_max_, rls_Cr_est_));
    rls_Ct_est_ = std::max(rls_Ct_est_min_, std::min(rls_Ct_est_max_, rls_Ct_est_));
    nmpc_params_.Cf = rls_Cf_est_;
    nmpc_params_.Cr = rls_Cr_est_;
    nmpc_params_.Ct = rls_Ct_est_;
    rls_C_out_prev_ = Vector3d(rls_Cf_est_, rls_Cr_est_, rls_Ct_est_);

    // 路径几何参数；PP预瞄参数统一从 supervisor_config 加载。
    nh_nmpc.param("curvature_smoothing_distance_m", curvature_smoothing_distance_m_, 6.0);
    nh_nmpc.param("use_geometric_path_heading", use_geometric_path_heading_, true);
    nh_nmpc.param("geometric_heading_window_m", geometric_heading_window_m_, 2.0);
    nh_nmpc.param("path_projection_heading_weight_m2", path_projection_heading_weight_m2_, 4.0);
    nh_nmpc.param("path_projection_heading_gate_rad", path_projection_heading_gate_rad_, 1.2);
    nh_nmpc.param("path_projection_rear_gate_m", path_projection_rear_gate_m_, 5.0);
    nh_nmpc.param("use_equilibrium_feedforward", use_equilibrium_feedforward_, true);
    nh_nmpc.param("equilibrium_feedforward_gain", equilibrium_feedforward_gain_, 1.0);
    nh_nmpc.param("equilibrium_feedforward_limit", equilibrium_feedforward_limit_, 0.45);
    nh_nmpc.param("observer_dynamic_min_speed_mps", observer_dynamic_min_speed_mps_, 4.0);

    curvature_smoothing_distance_m_ = std::max(0.5, curvature_smoothing_distance_m_);
    geometric_heading_window_m_ = std::max(0.2, geometric_heading_window_m_);
    path_projection_heading_weight_m2_ = std::max(0.0, path_projection_heading_weight_m2_);
    path_projection_heading_gate_rad_ = std::max(0.1, path_projection_heading_gate_rad_);
    path_projection_rear_gate_m_ = std::max(0.0, path_projection_rear_gate_m_);
    equilibrium_feedforward_gain_ = std::max(0.0, equilibrium_feedforward_gain_);
    equilibrium_feedforward_limit_ = std::max(0.0, equilibrium_feedforward_limit_);
    observer_dynamic_min_speed_mps_ = std::max(1.0, observer_dynamic_min_speed_mps_);
    

    // 打印加载的参数
    // NMPC核心参数
    logParamLoad("nx",nmpc_params_.nx, 8);
    logParamLoad("nu",nmpc_params_.nu, 1);
    logParamLoad("prediction_step",nmpc_params_.N, 25);
    logParamLoad("sparse_control_step",nmpc_params_.Nc, 8);
    logParamLoad("sampling_time",nmpc_params_.dt, 0.05);
    // 车辆核心动力学参数（含挂车） ——
    logParamLoad("m", nmpc_params_.m, 10000.0);
    logParamLoad("Iz", nmpc_params_.Iz, 50000.0);
    logParamLoad("lf", nmpc_params_.lf, 2.0);
    logParamLoad("lr", nmpc_params_.lr, 2.135);
    logParamLoad("m_t_empty", nmpc_params_.m_t, 7570.0);
    logParamLoad("Iz_t_empty", nmpc_params_.Iz_t, 150000.0);
    logParamLoad("lt", nmpc_params_.lt, 3.4);
    logParamLoad("L2", nmpc_params_.L2, 7.9);
    logParamLoad("lh", nmpc_params_.lh, 0.0);
    logParamLoad("T_lag", nmpc_params_.T_lag, 0.25);
    // 控制量边界约束 ——
    logParamLoad("min_steer", nmpc_params_.delta_min, -0.5);
    logParamLoad("max_steer", nmpc_params_.delta_max, 0.5);
    logParamLoad("delta_rate_max", nmpc_params_.delta_rate_max, 0.35);
    logParamLoad("delta_rate_min", nmpc_params_.delta_rate_min, -0.35);
    // 代价函数权重 ——
    logParamLoad("Q_x",nmpc_params_.Q(0,0), 1.0);
    logParamLoad("Q_y",nmpc_params_.Q(1,1), 10000.0);          // 纵向位置误差权重
    logParamLoad("Q_theta",nmpc_params_.Q(2,2), 104000.0);
    logParamLoad("Q_vy",nmpc_params_.Q(3,3), 1.0);
    logParamLoad("Q_r",nmpc_params_.Q(4,4), 200.0);
    logParamLoad("Q_delta",nmpc_params_.Q(5,5), 1.0);
    logParamLoad("Q_r_t",nmpc_params_.Q(6,6), 300.0);
    logParamLoad("Q_gamma",nmpc_params_.Q(7,7), 500.0);
    logParamLoad("Q_dgamma", nmpc_params_.dgamma, 1000.0);
    logParamLoad("Q_R", nmpc_params_.R, 20.0);
    logParamLoad("Q_dR", nmpc_params_.dR, 4000000.0);
    // —— 参数估计中参数 ——
    logParamLoad("Kiz", nmpc_params_.Kiz, 10.0);
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
    logParamLoad("integration_grade", nmpc_params_.integration_grade, 2.0);

    // -------------------------------------------------------------------------
    // 6. 加载 Supervisor 配置 (模式切换与纯跟踪)
    // -------------------------------------------------------------------------
    ros::NodeHandle nh_super(nh, "supervisor_config");
    nh_super.param("startup_time", supervisor_params_.startup_time, 0.1);
    nh_super.param("blend_speed_low", supervisor_params_.blend_speed_low, 4.1667);
    nh_super.param("blend_speed_high", supervisor_params_.blend_speed_high, 5.0);
    nh_super.param("standstill_speed", supervisor_params_.standstill_speed, 0.05);
    nh_super.param("nmpc_speed_floor", supervisor_params_.nmpc_speed_floor, 1.0);
    nh_super.param("min_lookahead_distance", min_lookahead_distance_, 5.0);
    nh_super.param("lookahead_speed_coeff", lookahead_speed_coeff_, 0.5);
    nh_super.param("lookahead_curvature_coeff", lookahead_curvature_coeff_, 0.0);
    nh_super.param("control_time", control_time_, 0.05);
    nh_super.param("control_delay_sec", control_delay_sec_, 0.0);
    nh_super.param("output_lpf_tau", output_lpf_tau_, 0.0);
    nh_super.param("degrade_failure_times", degrade_failure_times_, 3);
    nh_super.param("require_overtake_times", require_overtake_times_, 10);
    nh_super.param("fallback_min_hold_s", fallback_min_hold_s_, 0.5);
    nh_super.param("fallback_required_successes", fallback_required_successes_, 8);
    nh_super.param("fallback_reentry_blend_time_s", fallback_reentry_blend_time_s_, 0.5);
    nh_super.param("fallback_reentry_max_lateral_error_m", fallback_reentry_max_lateral_error_m_, 0.60);
    nh_super.param("fallback_reentry_max_heading_error_rad", fallback_reentry_max_heading_error_rad_, 0.15);
    nh_super.param("fallback_reentry_max_yaw_rate_radps", fallback_reentry_max_yaw_rate_radps_, 0.25);
    nh_super.param("fallback_reentry_max_kappa_step_1pm", fallback_reentry_max_kappa_step_1pm_, 0.003);
    nh_super.param("fallback_reentry_max_nearest_index_jump", fallback_reentry_max_nearest_index_jump_, 5);
    nh_super.param("nmpc_attempt_min_speed_mps", nmpc_attempt_min_speed_mps_, 3.0);
    nh_super.param("startup_recovery_enabled", startup_recovery_enabled_, true);
    nh_super.param("startup_recovery_entry_lateral_error_m", startup_recovery_entry_lateral_error_m_, 1.0);
    nh_super.param("startup_recovery_entry_heading_error_rad", startup_recovery_entry_heading_error_rad_, 0.25);
    nh_super.param("startup_recovery_exit_lateral_error_m", startup_recovery_exit_lateral_error_m_, 0.35);
    nh_super.param("startup_recovery_exit_heading_error_rad", startup_recovery_exit_heading_error_rad_, 0.10);
    nh_super.param("startup_recovery_exit_yaw_rate_radps", startup_recovery_exit_yaw_rate_radps_, 0.15);
    nh_super.param("startup_recovery_exit_cycles", startup_recovery_exit_cycles_, 10);
    nh_super.param("startup_recovery_min_lookahead_m", startup_recovery_min_lookahead_m_, 12.0);
    nh_super.param("startup_recovery_lookahead_error_gain", startup_recovery_lookahead_error_gain_, 2.0);
    nh_super.param("startup_recovery_max_steer_rad", startup_recovery_max_steer_rad_, 0.25);
    nh_super.param("startup_recovery_stationary_hold_speed_mps", startup_recovery_stationary_hold_speed_mps_, 0.50);
    nh_super.param("infer_manual_mode_from_zero_tracking_error", infer_manual_mode_from_zero_tracking_error_, true);
    nh_super.param("use_driving_mode_topic", use_driving_mode_topic_, true);
    nh_super.param("driving_mode_topic", driving_mode_topic_, std::string("/dfcv_bridge/driving_mode"));
    nh_super.param("autonomous_driving_mode_value", autonomous_driving_mode_value_, 2);
    nh_super.param("driving_mode_timeout_s", driving_mode_timeout_s_, 0.5);
    nh_super.param("manual_mode_zero_error_epsilon_m", manual_mode_zero_error_epsilon_m_, 1e-9);
    nh_super.param("manual_mode_confirm_cycles", manual_mode_confirm_cycles_, 3);
    nh_super.param("autonomous_mode_confirm_cycles", autonomous_mode_confirm_cycles_, 2);

    min_lookahead_distance_ = std::max(0.1, min_lookahead_distance_);
    lookahead_speed_coeff_ = std::max(0.0, lookahead_speed_coeff_);
    lookahead_curvature_coeff_ = std::max(0.0, lookahead_curvature_coeff_);
    control_time_ = std::max(1e-3, control_time_);
    control_delay_sec_ = std::max(0.0, control_delay_sec_);
    output_lpf_tau_ = std::max(0.0, output_lpf_tau_);
    degrade_failure_times_ = std::max(1, degrade_failure_times_);
    require_overtake_times_ = std::max(degrade_failure_times_, require_overtake_times_);
    fallback_min_hold_s_ = std::max(0.0, fallback_min_hold_s_);
    fallback_required_successes_ = std::max(1, fallback_required_successes_);
    fallback_reentry_blend_time_s_ = std::max(0.05, fallback_reentry_blend_time_s_);
    fallback_reentry_max_lateral_error_m_ = std::max(0.0, fallback_reentry_max_lateral_error_m_);
    fallback_reentry_max_heading_error_rad_ = std::max(0.0, fallback_reentry_max_heading_error_rad_);
    fallback_reentry_max_yaw_rate_radps_ = std::max(0.0, fallback_reentry_max_yaw_rate_radps_);
    fallback_reentry_max_kappa_step_1pm_ = std::max(0.0, fallback_reentry_max_kappa_step_1pm_);
    fallback_reentry_max_nearest_index_jump_ = std::max(0, fallback_reentry_max_nearest_index_jump_);
    nmpc_attempt_min_speed_mps_ = std::max(0.0, nmpc_attempt_min_speed_mps_);
    startup_recovery_exit_cycles_ = std::max(1, startup_recovery_exit_cycles_);
    startup_recovery_min_lookahead_m_ = std::max(min_lookahead_distance_, startup_recovery_min_lookahead_m_);
    startup_recovery_max_steer_rad_ = std::max(
        0.0, std::min(startup_recovery_max_steer_rad_, nmpc_params_.delta_max));
    startup_recovery_stationary_hold_speed_mps_ = std::max(
        0.0, startup_recovery_stationary_hold_speed_mps_);
    driving_mode_timeout_s_ = std::max(0.05, driving_mode_timeout_s_);
    manual_mode_zero_error_epsilon_m_ = std::max(0.0, manual_mode_zero_error_epsilon_m_);
    manual_mode_confirm_cycles_ = std::max(1, manual_mode_confirm_cycles_);
    autonomous_mode_confirm_cycles_ = std::max(1, autonomous_mode_confirm_cycles_);
    nmpc_params_.near_dense_control_steps = std::max(
        0, std::min(nmpc_params_.near_dense_control_steps, nmpc_params_.Nc - 1));

    const bool physical_params_valid =
        std::isfinite(nmpc_params_.m) && nmpc_params_.m > 0.0 &&
        std::isfinite(nmpc_params_.Iz) && nmpc_params_.Iz > 0.0 &&
        std::isfinite(nmpc_params_.lf) && nmpc_params_.lf > 0.0 &&
        std::isfinite(nmpc_params_.lr) && nmpc_params_.lr > 0.0 &&
        std::isfinite(nmpc_params_.m_t) && nmpc_params_.m_t > 0.0 &&
        std::isfinite(nmpc_params_.m_t_total) && nmpc_params_.m_t_total >= nmpc_params_.m_t &&
        std::isfinite(nmpc_params_.Iz_t) && nmpc_params_.Iz_t > 0.0 &&
        std::isfinite(nmpc_params_.lt) && nmpc_params_.lt > 0.0 &&
        std::isfinite(nmpc_params_.L2) && nmpc_params_.L2 > 0.0 &&
        std::isfinite(nmpc_params_.T_lag) && nmpc_params_.T_lag > 0.0 &&
        nmpc_params_.delta_min < nmpc_params_.delta_max;
    bool weights_valid = nmpc_params_.R >= 0.0 && nmpc_params_.dR >= 0.0 &&
                         nmpc_params_.dgamma >= 0.0;
    for (int i = 0; i < 8; ++i) {
        weights_valid = weights_valid && std::isfinite(nmpc_params_.Q(i, i)) &&
                        nmpc_params_.Q(i, i) >= 0.0;
    }
    if (!physical_params_valid || !weights_valid) {
        ROS_ERROR("[%s] 车辆物理参数、转角边界或NMPC权重无效，初始化终止",
                  getName().c_str());
        return false;
    }
    ipopt_max_iter_ = std::max(1, ipopt_max_iter_);
    ipopt_acceptable_iter_ = std::max(1, ipopt_acceptable_iter_);
    ipopt_acceptable_tol_ = std::max(1e-9, ipopt_acceptable_tol_);

    // IPOPT
    logParamLoad("ipopt_max_iter", ipopt_max_iter_, 100);
    logParamLoad("ipopt_acceptable_tol", ipopt_acceptable_tol_, 1e-2);               // IPPT可接受解的容差，默认1e-2
    logParamLoad("ipopt_acceptable_iter", ipopt_acceptable_iter_, 5);
    logParamLoad("ipopt_warm_start_bound_push", ipopt_warm_start_bound_push_, 1e-3);        // IPPT预热边界推送系数，默认1e-3，用于初始化控制量边界，热启动时，将初始点向变量边界 “推” 的最小距离
    logParamLoad("ipopt_warm_start_slack_bound_push", ipopt_warm_start_slack_bound_push_, 1e-3);  // IPPT预热边界松弛系数，默认1e-3，用于初始化控制量边界,针对松弛变量
    logParamLoad("ipopt_warm_start_mult_bound_push", ipopt_warm_start_mult_bound_push_, 1e-3);   // IPPT预热边界松弛系数，默认1e-3，用于初始化控制量边界,针对拉格朗日乘子

    start_time_ = ros::Time::now(); // 记录控制器启动时间
    
    // 构建 CasADi 求解器；配置/积分器异常时让插件初始化明确失败。
    try {
        buildNMPSolver();
    } catch (const std::exception& e) {
        ROS_ERROR("[%s] 构建NMPC求解器失败: %s", getName().c_str(), e.what());
        return false;
    }
    // 初始化铰接角发布器
    est_pub_ = nh.advertise<race_msgs::ESOEstimation>("/race/eso_estimation_states", 1);
    if (use_driving_mode_topic_) {
        driving_mode_sub_ = nh.subscribe<std_msgs::Int32>(
            driving_mode_topic_, 1, &ESOTracker2::drivingModeCallback, this);
    }
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

    const auto control_start_time = std::chrono::steady_clock::now();
    (void)flag;
    if (!vehicle_status || !path || !control_msg) {
        ROS_ERROR("[%s] 收到空指针消息", getName().c_str());
        return;
    }
    if (path->points.empty()) {
        const double measured_delta = vehicle_status->lateral.steering_angle;
        const double safe_hold = std::isfinite(measured_delta)
            ? std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, measured_delta))
            : current_cmd_;
        current_cmd_ = safe_hold;
        nmpc_safe_cmd_ = safe_hold;
        final_cmd_filt_ = safe_hold;
        ROS_WARN_THROTTLE(1.0, "[%s] 收到空路径，保持实测/上一有效转角 %.3f rad",
                          getName().c_str(), safe_hold);
        control_msg->lateral.steering_angle = safe_hold;
        control_msg->lateral.steering_angle_velocity =
            publish_steering_angle_velocity_ ? steering_angle_velocity_cmd_radps_ : 0.0;
        control_msg->steering_mode = race_msgs::Control::FRONT_STEERING_MODE;
        control_msg->control_mode = race_msgs::Control::DES_ACCEL_ONLY;
        return;
    }

    // 提取当前状态
    const double curr_x = vehicle_status->pose.position.x;
    const double curr_y = vehicle_status->pose.position.y;
    const double curr_theta = vehicle_status->euler.yaw;
    const double curr_vx_raw = vehicle_status->vel.linear.x;
    const double v_abs = std::abs(curr_vx_raw);
    const double vx_floor = std::max(1.0, supervisor_params_.nmpc_speed_floor);
    const double curr_vx = std::max(v_abs, vx_floor);   // NMPC/观测器内部速度，避免低速奇异
    const double curr_ay = vehicle_status->acc.linear.y;
    const double curr_r = vehicle_status->vel.angular.z;
    const double curr_vy_status = vehicle_status->vel.linear.y;
    const double curr_lateral_tracking_error = vehicle_status->tracking.lateral_tracking_error;
    const double curr_heading_tracking_error = vehicle_status->tracking.heading_angle_error;
    double curr_delta = vehicle_status->lateral.steering_angle;
    if (!std::isfinite(curr_x) || !std::isfinite(curr_y) || !std::isfinite(curr_theta) ||
        !std::isfinite(curr_vx_raw) || !std::isfinite(curr_vy_status) ||
        !std::isfinite(curr_ay) || !std::isfinite(curr_r) || !std::isfinite(curr_delta)) {
        ROS_ERROR_THROTTLE(0.5, "[%s] 车辆状态含NaN/Inf，保持上一有效转角 %.4f rad",
                           getName().c_str(), current_cmd_);
        control_msg->lateral.steering_angle = current_cmd_;
        control_msg->lateral.steering_angle_velocity =
            publish_steering_angle_velocity_ ? steering_angle_velocity_cmd_radps_ : 0.0;
        control_msg->steering_mode = race_msgs::Control::FRONT_STEERING_MODE;
        control_msg->control_mode = race_msgs::Control::DES_ACCEL_ONLY;
        return;
    }
    const bool measurement_is_new = isNewVehicleMeasurement(
        curr_x, curr_y, curr_theta, curr_vx_raw, curr_vy_status,
        curr_r, curr_delta, curr_ay);
     // 根据收到的车辆状态更新挂车载货质量
    if (std::isfinite(vehicle_status->trailer.mass) &&
        vehicle_status->trailer.mass >= m_t_empty_default_ &&
        vehicle_status->trailer.mass < 100000.0) {
        nmpc_params_.m_t_total = vehicle_status->trailer.mass;
        ROS_INFO_THROTTLE(1.0, "[%s] 更新挂车总质量: %.2f kg",
                          getName().c_str(), nmpc_params_.m_t_total);

    } else {
        ROS_WARN_THROTTLE(1.0,
                          "[%s] 挂车质量 %.2f kg 无效，沿用上一有效值 %.2f kg",
                          getName().c_str(), vehicle_status->trailer.mass,
                          nmpc_params_.m_t_total);
    }

    curr_delta = std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, curr_delta));

    ros::Time current_time = ros::Time::now();

    if (!control_output_initialized_) {
        current_cmd_ = curr_delta;
        nmpc_safe_cmd_ = curr_delta;
        final_cmd_filt_ = curr_delta;
        start_time_ = current_time;
        fallback_enter_time_ = current_time;
        control_output_initialized_ = true;
    }
    
    if (last_control_time_.toSec() != 0.0 && (current_time - last_control_time_).toSec() > 0.2) {
        ROS_WARN("[%s] 检测到控制重连，清空历史记忆与热启动！", getName().c_str());
        solver_.has_prev_sol = false;
        solver_.sol_prev = nullptr;
        current_cmd_ = curr_delta;
        nmpc_safe_cmd_ = curr_delta;
        blend_alpha_ = 0.0;
        eso_x1_ = curr_r;
        eso_x2_ = 0.0;
        eso_initialized_ = false;
        ckf_x_hat_ << 0.0, curr_r, r_t_, gamma_;
        ckf_P_ = Matrix4d::Identity() * 1.0;
        r_filter_initialized_ = false;
        r_tractor_filt_ = curr_r;
        final_cmd_filt_ = curr_delta;
        final_cmd_filt_init_ = false;
        measurement_fingerprint_valid_ = false;
        fallback_latched_ = false;
        fallback_reentry_active_ = false;
        fallback_reason_code_ = 0;
        nmpc_success_streak_ = 0;
        reference_stable_streak_ = 0;
        reference_prev_nearest_idx_ = -1;
        last_reference_kappa_valid_ = false;
        startup_recovery_checked_ = false;
        startup_recovery_active_ = false;
        startup_recovery_alignment_streak_ = 0;
        pp_cmd_queue_.clear();
        start_time_ = current_time;
    }
    last_control_time_ = current_time;
    const double obs_dt = std::max(0.01, std::min(dt, 0.05));

    // 优先采用显式 driving_mode；未接入桥时兼容 ESOTracker 的零误差推断。
    bool manual_state_available = false;
    bool manual_mode_observed = false;
    const double driving_mode_age_s = driving_mode_received_
        ? std::max(0.0, (current_time - driving_mode_stamp_).toSec())
        : std::numeric_limits<double>::infinity();
    if (use_driving_mode_topic_ && driving_mode_received_ &&
        driving_mode_age_s <= driving_mode_timeout_s_) {
        manual_state_available = true;
        manual_mode_observed = latest_driving_mode_ != autonomous_driving_mode_value_;
    } else if (infer_manual_mode_from_zero_tracking_error_) {
        manual_state_available = true;
        manual_mode_observed = std::isfinite(curr_lateral_tracking_error) &&
            std::isfinite(curr_heading_tracking_error) &&
            std::abs(curr_lateral_tracking_error) <= manual_mode_zero_error_epsilon_m_ &&
            std::abs(curr_heading_tracking_error) <= manual_mode_zero_error_epsilon_m_;
    }

    if (manual_state_available) {
        if (manual_mode_observed) {
            ++zero_tracking_error_streak_;
            nonzero_tracking_error_streak_ = 0;
        } else {
            ++nonzero_tracking_error_streak_;
            zero_tracking_error_streak_ = 0;
        }
        if (!inferred_manual_mode_ && zero_tracking_error_streak_ >= manual_mode_confirm_cycles_) {
            inferred_manual_mode_ = true;
            solver_.has_prev_sol = false;
            solver_.sol_prev = nullptr;
            pp_cmd_queue_.clear();
        }
        if (inferred_manual_mode_) {
            current_cmd_ = curr_delta;
            final_cmd_filt_ = curr_delta;
            final_cmd_filt_init_ = false;
            if (nonzero_tracking_error_streak_ >= autonomous_mode_confirm_cycles_) {
                inferred_manual_mode_ = false;
                solver_.has_prev_sol = false;
                solver_.sol_prev = nullptr;
                pp_cmd_queue_.clear();
                start_time_ = current_time;
                ckf_x_hat_ << 0.0, curr_r, r_t_, gamma_;
                ckf_P_ = Matrix4d::Identity();
                eso_x1_ = curr_r;
                eso_x2_ = 0.0;
                eso_initialized_ = true;
                startup_recovery_checked_ = false;
                reference_prev_nearest_idx_ = -1;
                last_reference_kappa_valid_ = false;
                reference_stable_streak_ = 0;
                enterFallback(4, current_time);
                ROS_WARN("[%s] 检测到自动驾驶重新接管，输出锚定实测转角 %.4f rad",
                         getName().c_str(), curr_delta);
            }
        }
    }

    if (rls_w1_prev_ == 0.0 && curr_r != 0.0) {
        rls_w1_prev_ = curr_r;}
    // 1. 路径处理始终执行：低速也生成参考，保证 PP 与 NMPC 热启动一致
    std::vector<double> current_pose = {curr_x, curr_y, curr_theta, curr_vx};
    casadi::DM waypoints_dm = process_race_path(*path, current_pose);
    // 2. 观测器始终预测；重复的保持值不重复校正，低速区锚定可测状态。
    calculate_trailer_kinematics(v_abs, curr_r, obs_dt);
    double gamma_pre = gamma_;
    double r_t_pre = r_t_;
    if (v_abs < observer_dynamic_min_speed_mps_) {
        const double vy_seed = (std::isfinite(curr_vy_status) && std::abs(curr_vy_status) <= 3.0)
            ? curr_vy_status : 0.0;
        ckf_x_hat_ << vy_seed, curr_r, r_t_pre, normalizeAngle(gamma_pre);
        ckf_P_ = Matrix4d::Identity() * 0.25;
        eso_x1_ = curr_r;
        eso_x2_ = 0.0;
        eso_initialized_ = true;
    } else {
        // z=[ay,r1,r2,gamma]，r2/gamma 使用挂车运动学伪测量。
        ckfEstimate(curr_vx, curr_delta, curr_ay, curr_r,
                    r_t_pre, gamma_pre, obs_dt, measurement_is_new);
    }
    const double vy_est = ckf_x_hat_(0);
    double curr_r_t = ckf_x_hat_(2);
    double curr_gamma = normalizeAngle(ckf_x_hat_(3));
    r_t_ = curr_r_t;
    gamma_ = curr_gamma;
    ROS_INFO_THROTTLE(0.5,
                      "[%s] CKF: vy=%.4f m/s, r1=%.4f, r_t=%.4f rad/s, gamma=%.4f rad",
                      getName().c_str(), vy_est, ckf_x_hat_(1), curr_r_t, curr_gamma);

    if (measurement_is_new) {
        rlsIdentifyStiffness(curr_vx, vy_est, curr_delta, curr_r, curr_ay,
                             curr_gamma, curr_r_t, nmpc_params_.m_t_total, obs_dt);
    }
    nmpc_params_.Cf = rls_Cf_est_;
    nmpc_params_.Cr = rls_Cr_est_;
    nmpc_params_.Ct = rls_Ct_est_;

    // esoCompute(curr_r, curr_delta, obs_dt);
    if (v_abs >= observer_dynamic_min_speed_mps_) {
        esoCompute(vy_est, curr_r, curr_delta, curr_r_t, curr_gamma,
                   curr_vx, obs_dt, measurement_is_new);
    }
    const double h_hat_total = eso_x2_;
    const double d_pure_trailer = h_hat_total;
    ROS_INFO_THROTTLE(0.5, "[%s] ESO: x1=%.4f rad/s, disturbance=%.4f rad/s^2",
                      getName().c_str(), eso_x1_, h_hat_total);

    // 3. 启动/低速/高速软切换权重：0=纯跟踪，1=NMPC
    const double time_elapsed = (current_time - start_time_).toSec();
    double base_blend_alpha = 0.0;
    if (time_elapsed < supervisor_params_.startup_time) {
        base_blend_alpha = 0.0;
        ROS_INFO_THROTTLE(0.5, "[%s][STARTUP] 预热 %.1f / %.1f s，纯跟踪锁定",
                          getName().c_str(), time_elapsed, supervisor_params_.startup_time);
    } else if (v_abs <= supervisor_params_.blend_speed_low) {
        base_blend_alpha = 0.0;
    } else if (v_abs >= supervisor_params_.blend_speed_high) {
        base_blend_alpha = 1.0;
    } else {
        const double denom = std::max(1e-3, supervisor_params_.blend_speed_high - supervisor_params_.blend_speed_low);
        base_blend_alpha = (v_abs - supervisor_params_.blend_speed_low) / denom;
        base_blend_alpha = std::max(0.0, std::min(1.0, base_blend_alpha));
    }
    blend_alpha_ = base_blend_alpha;

    const int nearest_idx = find_nearest_path_point(curr_x, curr_y, curr_theta, *path);
    const auto& nearest_path_point = path->points[nearest_idx];
    double nearest_path_yaw = quaternion_to_yaw(nearest_path_point.pose.orientation);
    if (use_geometric_path_heading_ && path->points.size() >= 2) {
        const int left = std::max(0, nearest_idx - 2);
        const int right = std::min(static_cast<int>(path->points.size()) - 1, nearest_idx + 2);
        const double tdx = path->points[right].pose.position.x - path->points[left].pose.position.x;
        const double tdy = path->points[right].pose.position.y - path->points[left].pose.position.y;
        if (std::hypot(tdx, tdy) > 1e-4) nearest_path_yaw = std::atan2(tdy, tdx);
    }
    const double nearest_dx = curr_x - nearest_path_point.pose.position.x;
    const double nearest_dy = curr_y - nearest_path_point.pose.position.y;
    const double geometric_lateral_error =
        -std::sin(nearest_path_yaw) * nearest_dx + std::cos(nearest_path_yaw) * nearest_dy;
    const double heading_error = normalizeAngle(curr_theta - nearest_path_yaw);
    const double kappa = static_cast<double>(waypoints_dm(3, std::min(1, nmpc_params_.N)));
    last_delta_ff_ = static_cast<double>(waypoints_dm(4, std::min(1, nmpc_params_.N)));

    const int nearest_jump = reference_prev_nearest_idx_ >= 0
        ? nearest_idx - reference_prev_nearest_idx_ : 0;
    reference_kappa_step_ = last_reference_kappa_valid_ && std::isfinite(kappa)
        ? std::abs(kappa - last_reference_kappa_) : std::numeric_limits<double>::infinity();
    reference_stable_this_cycle_ = last_reference_kappa_valid_ && std::isfinite(kappa) &&
        reference_kappa_step_ <= fallback_reentry_max_kappa_step_1pm_ &&
        std::abs(nearest_jump) <= fallback_reentry_max_nearest_index_jump_;
    reference_stable_streak_ = reference_stable_this_cycle_ ? reference_stable_streak_ + 1 : 0;
    last_reference_kappa_valid_ = std::isfinite(kappa);
    if (last_reference_kappa_valid_) last_reference_kappa_ = kappa;
    reference_prev_nearest_idx_ = nearest_idx;

    if (!startup_recovery_checked_ && !inferred_manual_mode_) {
        startup_recovery_checked_ = true;
        startup_recovery_active_ = startup_recovery_enabled_ &&
            (std::abs(geometric_lateral_error) >= startup_recovery_entry_lateral_error_m_ ||
             std::abs(heading_error) >= startup_recovery_entry_heading_error_rad_);
        if (startup_recovery_active_) enterFallback(1, current_time);
    }
    if (startup_recovery_active_) {
        const bool aligned =
            std::abs(geometric_lateral_error) <= startup_recovery_exit_lateral_error_m_ &&
            std::abs(heading_error) <= startup_recovery_exit_heading_error_rad_ &&
            std::abs(curr_r) <= startup_recovery_exit_yaw_rate_radps_;
        startup_recovery_alignment_streak_ = aligned ? startup_recovery_alignment_streak_ + 1 : 0;
        if (startup_recovery_alignment_streak_ >= startup_recovery_exit_cycles_) {
            startup_recovery_active_ = false;
            fallback_latched_ = true;
            fallback_enter_time_ = current_time;
            nmpc_success_streak_ = 0;
            solver_.has_prev_sol = false;
            solver_.sol_prev = nullptr;
        }
    }

    // 4. 满足速度与监督条件时求解 NMPC；低速/人工/恢复阶段主动跳过。
    // NMPC 现在工作在“自车体坐标系”：原点为自车当前位置，x 轴沿自车当前航向。
    // 因此初始 x,y,theta 均为 0；vy/r/delta/r_t/gamma 仍为实际物理量
    // （挂车横摆率 r_t、铰接角 gamma 与全局朝向无关，保持真实值不变）。
    std::vector<double> nmpc_state = {0.0, 0.0, 0.0, vy_est, curr_r,
                                      curr_delta, curr_r_t, curr_gamma};
    std::vector<double> control_output(1, nmpc_safe_cmd_);

    std::vector<double> dyn_params = {
        nmpc_params_.m, nmpc_params_.Iz, nmpc_params_.lf, nmpc_params_.lr, rls_Cf_est_,
        rls_Cr_est_, nmpc_params_.m_t_total,
        nmpc_params_.Iz_t + nmpc_params_.Kiz * (nmpc_params_.m_t_total - nmpc_params_.m_t),
        nmpc_params_.lt, rls_Ct_est_, nmpc_params_.L2
    };
    solver_.opti.set_value(solver_.P_vx, curr_vx);
    solver_.opti.set_value(solver_.P_h_hat, d_pure_trailer);
    solver_.opti.set_value(solver_.P_dyn_params, dyn_params);

    bool nmpc_solve_success = false;
    last_nmpc_attempted_ = false;
    last_nmpc_solver_returned_success_ = false;
    last_nmpc_warm_start_used_ = false;
    last_nmpc_deadline_missed_ = false;
    last_nmpc_status_code_ = 0;
    last_nmpc_return_status_ = "not_attempted";
    iter_time_ = 0.0;
    if (inferred_manual_mode_) {
        last_nmpc_status_code_ = 6;
        last_nmpc_return_status_ = "skipped_manual_mode";
    } else if (startup_recovery_active_) {
        last_nmpc_status_code_ = 5;
        last_nmpc_return_status_ = "skipped_startup_recovery";
    } else if (v_abs < nmpc_attempt_min_speed_mps_) {
        last_nmpc_status_code_ = 4;
        last_nmpc_return_status_ = "skipped_low_speed";
    } else {
        last_nmpc_attempted_ = true;
        const auto solve_start = std::chrono::steady_clock::now();
        nmpc_solve_success = solveNMPC(nmpc_state, waypoints_dm, control_output);
        iter_time_ = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - solve_start).count();
    }
    if (nmpc_solve_success) {
        nmpc_safe_cmd_ = control_output[0];
        mpc_failure_flag_ = false;
        mpc_failure_count_ = 0;
    } else if (last_nmpc_attempted_) {
        nmpc_safe_cmd_ = current_cmd_;
        mpc_failure_flag_ = true;
        ++mpc_failure_count_;
        enterFallback(last_nmpc_deadline_missed_ ? 2 : 3, current_time);
        ROS_WARN_THROTTLE(0.5, "[%s] NMPC失败/过期，锁存PP: status=%s time=%.2fms",
                          getName().c_str(), last_nmpc_return_status_.c_str(), iter_time_);
    } else {
        nmpc_safe_cmd_ = current_cmd_;
        mpc_failure_flag_ = false;
    }
    nmpc_safe_cmd_ = std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, nmpc_safe_cmd_));

    // 5. 纯跟踪保护支路：启动、静止、低速、NMPC失败时都可兜底；最终通过 blend_alpha_ 连续融合
    const double speed_lookahead =
        min_lookahead_distance_ + lookahead_speed_coeff_ * v_abs;
    double recovery_lookahead = speed_lookahead;
    if (startup_recovery_active_) {
        recovery_lookahead = std::max(
            startup_recovery_min_lookahead_m_,
            speed_lookahead + startup_recovery_lookahead_error_gain_ *
                std::abs(geometric_lateral_error));
    }

    double preview_abs_curvature = 0.0;
    double preview_distance = 0.0;
    for (int i = nearest_idx + 2; i < static_cast<int>(path->points.size()); ++i) {
        const auto& pm1 = path->points[i - 2].pose.position;
        const auto& p0 = path->points[i - 1].pose.position;
        const auto& p1 = path->points[i].pose.position;
        const double ds0 = std::hypot(p0.x - pm1.x, p0.y - pm1.y);
        const double ds1 = std::hypot(p1.x - p0.x, p1.y - p0.y);
        preview_distance += ds1;
        if (ds0 > 1e-4 && ds1 > 1e-4) {
            const double yaw0 = std::atan2(p0.y - pm1.y, p0.x - pm1.x);
            const double yaw1 = std::atan2(p1.y - p0.y, p1.x - p0.x);
            preview_abs_curvature = std::max(
                preview_abs_curvature,
                std::abs(normalizeAngle(yaw1 - yaw0) / std::max(0.5 * (ds0 + ds1), 1e-4)));
        }
        if (preview_distance >= recovery_lookahead) break;
    }
    double lookahead_dist = std::max(
        min_lookahead_distance_,
        recovery_lookahead - lookahead_curvature_coeff_ * preview_abs_curvature);
    if (startup_recovery_active_) {
        lookahead_dist = std::max(startup_recovery_min_lookahead_m_, lookahead_dist);
    }

    double pp_candidate = computePurePursuitSteering(
        *path, curr_x, curr_y, curr_theta, lookahead_dist);
    if (startup_recovery_active_) {
        if (v_abs < startup_recovery_stationary_hold_speed_mps_) {
            pp_candidate = curr_delta;
        } else {
            pp_candidate = std::max(-startup_recovery_max_steer_rad_,
                                    std::min(startup_recovery_max_steer_rad_, pp_candidate));
        }
    }

    double pp_safe_cmd = pp_candidate;
    if (control_delay_sec_ <= 1e-9) {
        pp_cmd_queue_.clear();
    } else {
        const size_t delay_steps = static_cast<size_t>(std::max(
            1.0, std::ceil(control_delay_sec_ / control_time_)));
        pp_cmd_queue_.push_back(pp_candidate);
        if (pp_cmd_queue_.size() > delay_steps) {
            pp_safe_cmd = pp_cmd_queue_.front();
            pp_cmd_queue_.pop_front();
        } else {
            pp_safe_cmd = curr_delta;
        }
    }
    pp_safe_cmd = std::max(nmpc_params_.delta_min,
                           std::min(nmpc_params_.delta_max, pp_safe_cmd));

    if (v_abs <= supervisor_params_.standstill_speed) {
        blend_alpha_ = 0.0;
        ROS_INFO_THROTTLE(0.5, "[%s][STANDSTILL] v=%.2f m/s，保持纯跟踪保护",
                          getName().c_str(), curr_vx_raw);
    }

    // 一次求解失败立即锁存 PP；只有连续成功、参考稳定且车辆回稳后才平滑重入。
    if (nmpc_solve_success) {
        nmpc_success_streak_ = std::min(nmpc_success_streak_ + 1, 1000000);
    } else if (last_nmpc_attempted_) {
        nmpc_success_streak_ = 0;
    }
    if (fallback_latched_) {
        const double fallback_age = std::max(0.0, (current_time - fallback_enter_time_).toSec());
        const bool ready = !startup_recovery_active_ && !inferred_manual_mode_ &&
            fallback_age >= fallback_min_hold_s_ &&
            nmpc_success_streak_ >= fallback_required_successes_ &&
            reference_stable_streak_ >= fallback_required_successes_ &&
            std::abs(geometric_lateral_error) <= fallback_reentry_max_lateral_error_m_ &&
            std::abs(heading_error) <= fallback_reentry_max_heading_error_rad_ &&
            std::abs(curr_r) <= fallback_reentry_max_yaw_rate_radps_;
        if (!fallback_reentry_active_ && ready) {
            fallback_reentry_active_ = true;
            fallback_reentry_alpha_ = 0.0;
        }
        const bool guard_ok = nmpc_solve_success && reference_stable_this_cycle_ &&
            std::abs(geometric_lateral_error) <= fallback_reentry_max_lateral_error_m_ &&
            std::abs(heading_error) <= fallback_reentry_max_heading_error_rad_ &&
            std::abs(curr_r) <= fallback_reentry_max_yaw_rate_radps_;
        if (fallback_reentry_active_ && !guard_ok) {
            fallback_reentry_active_ = false;
            fallback_reentry_alpha_ = 0.0;
        }
        if (fallback_reentry_active_) {
            fallback_reentry_alpha_ = std::min(
                1.0, fallback_reentry_alpha_ + obs_dt / fallback_reentry_blend_time_s_);
            blend_alpha_ = base_blend_alpha * fallback_reentry_alpha_;
            if (fallback_reentry_alpha_ >= 1.0) {
                fallback_latched_ = false;
                fallback_reentry_active_ = false;
                fallback_reason_code_ = 0;
                blend_alpha_ = base_blend_alpha;
            }
        } else {
            blend_alpha_ = 0.0;
        }
    }
    if (inferred_manual_mode_) blend_alpha_ = 0.0;
    require_over_take_flag_ = mpc_failure_count_ >= require_overtake_times_;

    if (blend_alpha_ < 0.01) {
        using_pure_pursuit_flag_ = true;
        using_mixed_mode_flag_ = false;
        ROS_INFO_THROTTLE(0.5, "[%s][PP] v=%.1f km/h | Ld=%.2f m | PP=%.3f | NMPC=%s",
                          getName().c_str(), curr_vx_raw * 3.6, lookahead_dist, pp_safe_cmd,
                          nmpc_solve_success ? "OK" : "FAIL");
    } else if (blend_alpha_ > 0.99) {
        using_pure_pursuit_flag_ = false;
        using_mixed_mode_flag_ = false;
        ROS_INFO_THROTTLE(0.5, "[%s][NMPC] v=%.1f km/h | cmd=%.3f rad",
                          getName().c_str(), curr_vx_raw * 3.6, nmpc_safe_cmd_);
    } else {
        using_pure_pursuit_flag_ = false;
        using_mixed_mode_flag_ = true;
        ROS_INFO_THROTTLE(0.5, "[%s][BLEND] v=%.1f km/h | alpha=%.2f | PP=%.3f | NMPC=%.3f",
                          getName().c_str(), curr_vx_raw * 3.6, blend_alpha_, pp_safe_cmd, nmpc_safe_cmd_);
    }

    // 6. 融合输出、可选低通和统一物理速率保护。
    double final_cmd = blend_alpha_ * nmpc_safe_cmd_ + (1.0 - blend_alpha_) * pp_safe_cmd;
    if (inferred_manual_mode_) {
        final_cmd = curr_delta;
        final_cmd_filt_ = curr_delta;
        final_cmd_filt_init_ = false;
    }

    if (output_lpf_tau_ > 1e-6) {
        if (!final_cmd_filt_init_) {
            final_cmd_filt_ = final_cmd;
            final_cmd_filt_init_ = true;
        }
        const double alpha = obs_dt / (output_lpf_tau_ + obs_dt);
        final_cmd_filt_ = (1.0 - alpha) * final_cmd_filt_ + alpha * final_cmd;
        final_cmd = final_cmd_filt_;
    }

    last_final_output_rate_limited_ = false;
    if (enforce_final_output_rate_limit_ && !inferred_manual_mode_) {
        const double lower = current_cmd_ + nmpc_params_.delta_rate_min * obs_dt;
        const double upper = current_cmd_ + nmpc_params_.delta_rate_max * obs_dt;
        const double limited = std::max(lower, std::min(upper, final_cmd));
        last_final_output_rate_limited_ = std::abs(limited - final_cmd) > 1e-12;
        final_cmd = limited;
    }
    final_cmd = std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, final_cmd));

    current_cmd_ = final_cmd;

    control_msg->lateral.steering_angle = final_cmd;
    control_msg->lateral.steering_angle_velocity =
        publish_steering_angle_velocity_ ? steering_angle_velocity_cmd_radps_ : 0.0;
    control_msg->steering_mode = race_msgs::Control::FRONT_STEERING_MODE;
    control_msg->control_mode = race_msgs::Control::DES_ACCEL_ONLY;

    // 状态参数对照输出
    double dr_nominal = calcNominalYawAccel(vy_est,curr_r,curr_delta,curr_r_t,curr_gamma,curr_vx);
    double dr_h_dist = dr_nominal + h_hat_total;
    model_r2_ = curr_r + dr_h_dist*obs_dt;
    total_control_time_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - control_start_time).count();

    // 发布估计状态
    race_msgs::ESOEstimation est_msg;
    est_msg.r_t = curr_r_t;        // 挂车横摆率
    est_msg.gamma_angle = curr_gamma;    // CKF联合估计铰接角
    est_msg.vy_est2 = vy_est;          // 牵引车侧向速度
    est_msg.eso2_total = h_hat_total;     // ESO扰动估计
    est_msg.Cf_est2 = rls_Cf_est_;     // RLS Cf
    est_msg.Cr_est2 = rls_Cr_est_;     // RLS Cr
    est_msg.Ct_est2 = rls_Ct_est_;     // RLS Ct
    est_msg.model_r2 = model_r2_;     // 模型预测横摆率
    est_msg.iter_time = iter_time_;
    est_msg.total_control_time = total_control_time_;
    est_msg.mpc_failure_flag = mpc_failure_flag_;
    est_msg.using_pure_pursuit_flag = using_pure_pursuit_flag_;
    est_msg.require_over_take_flag = require_over_take_flag_;
    est_msg.using_mixed_mode_flag = using_mixed_mode_flag_;
    est_pub_.publish(est_msg);

}

// ---------------------- 路径处理与辅助函数----------------------
double ESOTracker2::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

namespace {
// 计算 a 相对 b 的最短角度差，结果恒落在 [-pi, pi]。
// 等价于 atan2(sin(a-b), cos(a-b))，消除 ±pi 缠绕造成的跳变。
inline double angleDiff(double a, double b) {
    double d = a - b;
    while (d > M_PI)  d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    return d;
}

// 体坐标系变换的原点（自车当前位置）。由 process_race_path 在每帧调用前设置，
// 供 interpolate_path_segment 把全局位置参考转换为体坐标系。控制器单实例串行调用，安全。
double g_ref_x0 = 0.0;
double g_ref_y0 = 0.0;
} // anonymous namespace

double ESOTracker2::quaternion_to_yaw(const geometry_msgs::Quaternion& q) {
    tf::Quaternion tf_quat(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 rot_matrix(tf_quat);
    double roll, pitch, yaw;
    rot_matrix.getRPY(roll, pitch, yaw);
    return yaw;
}

int ESOTracker2::find_nearest_path_point(const double x0, const double y0,
                                         const double yaw0, const race_msgs::Path& path) {
    double min_cost = std::numeric_limits<double>::max();
    int nearest_idx = -1;
    const double cy = std::cos(yaw0);
    const double sy = std::sin(yaw0);
    for (size_t i = 0; i < path.points.size(); ++i) {
        const auto& pt = path.points[i].pose.position;
        const double dx = pt.x - x0;
        const double dy = pt.y - y0;
        if (cy * dx + sy * dy < -path_projection_rear_gate_m_) continue;

        double path_yaw = quaternion_to_yaw(path.points[i].pose.orientation);
        if (use_geometric_path_heading_ && path.points.size() >= 2) {
            const int left = std::max(0, static_cast<int>(i) - 2);
            const int right = std::min(
                static_cast<int>(path.points.size()) - 1, static_cast<int>(i) + 2);
            const double tdx = path.points[right].pose.position.x -
                               path.points[left].pose.position.x;
            const double tdy = path.points[right].pose.position.y -
                               path.points[left].pose.position.y;
            if (std::hypot(tdx, tdy) > 1e-4) path_yaw = std::atan2(tdy, tdx);
        }
        const double yaw_error = std::abs(angleDiff(path_yaw, yaw0));
        if (yaw_error > path_projection_heading_gate_rad_) continue;
        const double cost = dx * dx + dy * dy +
            path_projection_heading_weight_m2_ * yaw_error * yaw_error;
        if (cost < min_cost) {
            min_cost = cost;
            nearest_idx = static_cast<int>(i);
        }
    }
    if (nearest_idx >= 0) return nearest_idx;

    // 航向信息缺失或定位偏差很大时退回欧氏最近点。
    double min_dist_sq = std::numeric_limits<double>::max();
    nearest_idx = 0;
    for (size_t i = 0; i < path.points.size(); ++i) {
        const auto& pt = path.points[i].pose.position;
        const double dist_sq = std::pow(pt.x - x0, 2) + std::pow(pt.y - y0, 2);
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            nearest_idx = static_cast<int>(i);
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
        const double ds = s_original[i+1] - s_original[i];
        if (ds <= 1e-9) {
            val_target[k] = val_original[i];
            continue;
        }
        double ratio = (s_t - s_original[i]) / ds;
        val_target[k] = val_original[i] + ratio * (val_original[i+1] - val_original[i]);
    }
    return val_target;
}

casadi::DM ESOTracker2::interpolate_path_segment(const race_msgs::Path& path, const std::vector<double>& cum_dist, 
                                                         int start_idx, int end_idx,  const std::vector<double>& s_target, double yaw0) {
    // yaw0 = 自车当前航向 curr_theta（由 process_race_path 透传 current_state[2]）。
    const double veh_yaw = yaw0;

    std::vector<double> s_orig, x_orig, y_orig, theta_orig;

    // 关键修改：把每个参考点的航向都用 angleDiff 表示为“相对自车当前航向”的量，
    // 再沿弧长连续解缠绕(unwrap)，得到一条连续、且锚定在 0 附近的参考航向曲线。
    // 这样无论自车朝向是 0、±pi/2 还是 ±pi，参考航向都不会跨越 ±pi 折叠边界，
    // 从源头消除 ±pi/2 附近的航向突变与稳态误差放大问题。
    double theta_prev = 0.0;  // 上一个参考点的(相对、已解缠绕)航向
    for (int i = start_idx; i <= end_idx; ++i) {
        const auto& pt = path.points[i];
        s_orig.push_back(cum_dist[i - start_idx]);
        x_orig.push_back(pt.pose.position.x);   
        y_orig.push_back(pt.pose.position.y);

        double yaw_abs = quaternion_to_yaw(pt.pose.orientation);
        // 相对自车航向的航向角（首点会落在 [-pi, pi]）
        double yaw_rel = angleDiff(yaw_abs, veh_yaw);

        if (theta_orig.empty()) {
            theta_prev = yaw_rel;                    // 首点直接采用相对航向
        } else {
            // 沿弧长连续解缠绕：在上一点基础上加最短增量，保持曲线连续
            theta_prev += angleDiff(yaw_rel, theta_prev);
        }
        theta_orig.push_back(theta_prev);
    }

    // 根据动态 s_target 进行插值
    auto x_interp = linear_interpolate(s_orig, x_orig, s_target);
    auto y_interp = linear_interpolate(s_orig, y_orig, s_target);
    auto theta_interp = linear_interpolate(s_orig, theta_orig, s_target);
    std::vector<double> kappa_interp(s_target.size(), 0.0);

    // 与 ESOTracker 对齐：位置、航向和曲率来自同一条几何曲线。
    if (use_geometric_path_heading_ && s_target.size() >= 2) {
        const int n = static_cast<int>(s_target.size());
        const double ds_grid = std::max(1e-3, s_target[1] - s_target[0]);
        const int span = std::max(1, std::min(
            static_cast<int>(std::lround(geometric_heading_window_m_ / ds_grid)), n - 1));
        std::vector<double> theta_geometric(n, 0.0);
        double previous = 0.0;
        for (int i = 0; i < n; ++i) {
            int left = std::max(0, std::min(i - span / 2, n - 1 - span));
            const int right = left + span;
            double relative = theta_interp[i];
            const double dx = x_interp[right] - x_interp[left];
            const double dy = y_interp[right] - y_interp[left];
            if (std::hypot(dx, dy) > 1e-4) {
                relative = angleDiff(std::atan2(dy, dx), veh_yaw);
            }
            previous = (i == 0) ? relative : previous + angleDiff(relative, previous);
            theta_geometric[i] = previous;
        }
        theta_interp.swap(theta_geometric);
    }

    // 固定空间窗口计算曲率，避免同一位置随车速改变平滑尺度。
    if (s_target.size() >= 2) {
        const int n = static_cast<int>(s_target.size());
        const double ds_grid = std::max(1e-3, s_target[1] - s_target[0]);
        const int span = std::max(1, std::min(
            static_cast<int>(std::lround(curvature_smoothing_distance_m_ / ds_grid)), n - 1));
        for (int i = 0; i < n; ++i) {
            const int left = std::max(0, std::min(i - span / 2, n - 1 - span));
            const int right = left + span;
            const double ds = s_target[right] - s_target[left];
            kappa_interp[i] = ds > 1e-4
                ? (theta_interp[right] - theta_interp[left]) / ds : 0.0;
        }
    }

    // 位置参考也转换到“自车体坐标系”（原点为自车当前位置，x 轴沿自车当前航向），
    // 与相对航向、相对动力学初值 (x=y=theta=0) 保持一致，彻底摆脱全局朝向的影响。
    const double cos_y = std::cos(veh_yaw);
    const double sin_y = std::sin(veh_yaw);
    int n_waypoints = s_target.size();
    casadi::DM waypoints = casadi::DM::zeros(8, n_waypoints);
    const double vx_reference = s_target.size() >= 2
        ? std::max(1.0, (s_target[1] - s_target[0]) / nmpc_params_.dt) : 1.0;
    double previous_delta_ff = current_cmd_;
    for (int i = 0; i < n_waypoints; ++i) {
        double dx = x_interp[i] - g_ref_x0;
        double dy = y_interp[i] - g_ref_y0;
        double bx =  cos_y * dx + sin_y * dy;   // 体坐标系纵向
        double by = -sin_y * dx + cos_y * dy;   // 体坐标系横向
        waypoints(0, i) = bx;
        waypoints(1, i) = by;
        waypoints(2, i) = theta_interp[i];   // 相对自车当前航向、连续解缠绕后的参考航向
        waypoints(3, i) = kappa_interp[i];
        const auto reference = computeSteadyStateReference(vx_reference, kappa_interp[i]);
        double delta_ff = previous_delta_ff;
        if (i > 0) {
            const double lower = previous_delta_ff + nmpc_params_.delta_rate_min * nmpc_params_.dt;
            const double upper = previous_delta_ff + nmpc_params_.delta_rate_max * nmpc_params_.dt;
            delta_ff = std::max(lower, std::min(upper, reference[0]));
            delta_ff = std::max(nmpc_params_.delta_min,
                                std::min(nmpc_params_.delta_max, delta_ff));
        }
        previous_delta_ff = delta_ff;
        waypoints(4, i) = delta_ff;
        waypoints(5, i) = reference[1];
        waypoints(6, i) = reference[2];
        waypoints(7, i) = reference[3];
    }
    return waypoints;
}

casadi::DM ESOTracker2::process_race_path(const race_msgs::Path& input_path, const std::vector<double>& current_state) {
    int nearest_idx = find_nearest_path_point(
        current_state[0], current_state[1], current_state[2], input_path);
    if (nearest_idx == -1) return casadi::DM::zeros(8, nmpc_params_.N + 1);

    // 设置体坐标系变换原点为自车当前位置，供 interpolate_path_segment 使用
    g_ref_x0 = current_state[0];
    g_ref_y0 = current_state[1];

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

std::array<double, 4> ESOTracker2::computeSteadyStateReference(
    double vx, double kappa) const {
    if (!use_equilibrium_feedforward_ || !std::isfinite(vx) || !std::isfinite(kappa)) {
        return {0.0, 0.0, 0.0, 0.0};
    }
    const double r_ref = std::abs(vx) * kappa;
    const double wheelbase = std::max(0.1, nmpc_params_.lf + nmpc_params_.lr);
    double delta_ff = equilibrium_feedforward_gain_ * std::atan(wheelbase * kappa);
    if (equilibrium_feedforward_limit_ > 0.0) {
        delta_ff = std::max(-equilibrium_feedforward_limit_,
                            std::min(equilibrium_feedforward_limit_, delta_ff));
    }

    // 采用与本控制器挂车运动学伪测量相同的符号：稳态 r_t=r，
    // sin(gamma)=-(L2*kappa + lh*kappa*cos(gamma))；lh=0 时为精确闭式近似。
    const double gamma_argument = std::max(
        -0.95, std::min(0.95, -(nmpc_params_.L2 + nmpc_params_.lh) * kappa));
    const double gamma_ref = std::asin(gamma_argument);
    return {delta_ff, 0.0, r_ref, gamma_ref};
}

//  --- 核心算法：CKF 状态估计 (替换原 EKF) ---
namespace {
inline bool finiteVec4(const Eigen::Vector4d& v) {
    return v.allFinite();
}

inline bool finiteMat4(const Eigen::Matrix4d& m) {
    return m.allFinite();
}

inline Eigen::Matrix4d symmetrize4(const Eigen::Matrix4d& p) {
    return 0.5 * (p + p.transpose());
}

inline double clampFinite(double value, double lo, double hi, double fallback) {
    if (!std::isfinite(value)) return fallback;
    return std::max(lo, std::min(hi, value));
}

// 返回协方差矩阵平方根 S，使 S*S^T 近似等于 P。
// 优先 LLT；失败时通过特征值裁剪恢复半正定，避免 CKF 因数值噪声中断。
inline Eigen::Matrix4d sqrtCovariance4(const Eigen::Matrix4d& p_in) {
    Eigen::Matrix4d P = symmetrize4(p_in);
    if (!finiteMat4(P)) {
        P = Eigen::Matrix4d::Identity();
    }

    double jitter = 1e-9;
    for (int i = 0; i < 8; ++i) {
        Eigen::LLT<Eigen::Matrix4d> llt(P + jitter * Eigen::Matrix4d::Identity());
        if (llt.info() == Eigen::Success) {
            return llt.matrixL();
        }
        jitter *= 10.0;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(P);
    if (es.info() != Eigen::Success) {
        return Eigen::Matrix4d::Identity();
    }
    Eigen::Vector4d vals = es.eigenvalues();
    for (int i = 0; i < vals.size(); ++i) {
        vals(i) = std::sqrt(std::max(vals(i), 1e-8));
    }
    return es.eigenvectors() * vals.asDiagonal();
}

inline Eigen::Matrix4d regularizeCovariance4(const Eigen::Matrix4d& p_in, double min_diag = 1e-8, double max_diag = 1e4) {
    Eigen::Matrix4d P = symmetrize4(p_in);
    if (!finiteMat4(P)) {
        return Eigen::Matrix4d::Identity();
    }
    for (int i = 0; i < 4; ++i) {
        P(i, i) = clampFinite(P(i, i), min_diag, max_diag, 1.0);
    }
    P += min_diag * Eigen::Matrix4d::Identity();
    return symmetrize4(P);
}
} // anonymous namespace

Eigen::Vector4d ESOTracker2::compute_dynamics_3dof(const Eigen::Vector4d& x,
                                                   const Eigen::Vector2d& u) {
    const double vy = x(0);
    const double r1 = x(1);
    const double r2 = x(2);
    const double gamma = x(3);
    const double delta = u(0);
    const double vx_safe = std::max(u(1), 1.0);

    // 获取动力学参数
    const double m1 = nmpc_params_.m;
    const double m2 = nmpc_params_.m_t_total;
    const double Iz1 = nmpc_params_.Iz;
    const double Iz2 = nmpc_params_.Iz_t + nmpc_params_.Kiz * (m2 - nmpc_params_.m_t);
    const double a = nmpc_params_.lf;
    const double b = nmpc_params_.lr;
    const double c = b;
    const double d = nmpc_params_.lt;
    const double L2 = nmpc_params_.L2;
    const double e = std::max(0.1, L2 - d);
    const double Cf = std::max(1.0, rls_Cf_est_);
    const double Cr = std::max(1.0, rls_Cr_est_);
    const double Ct = std::max(1.0, rls_Ct_est_);

    // 轮胎侧偏角，使用 atan2 提高低速/符号切换鲁棒性。
    const double alpha_f = delta - std::atan2(vy + a * r1, vx_safe);
    const double alpha_r = -std::atan2(vy - b * r1, vx_safe);
    const double alpha_t = -std::atan2(vy - c * r1 - L2 * r2 - vx_safe * gamma, vx_safe);

    // 线性侧向力 + 平滑饱和，避免容积点跑到大侧偏角时力无限增大。
    const double Fyf_lin = Cf * alpha_f;
    const double Fyr_lin = Cr * alpha_r;
    const double Fyt_lin = Ct * alpha_t;
    const double mu = 0.85;
    const double g = 9.81;
    const double Fz_f = std::max(1.0, m1 * g * (b / std::max(0.1, a + b)));
    const double Fz_r = std::max(1.0, m1 * g * (a / std::max(0.1, a + b)) + m2 * g * (d / std::max(0.1, L2)));
    const double Fz_t = std::max(1.0, m2 * g * (e / std::max(0.1, L2)));
    const double Fyf = mu * Fz_f * std::tanh(Fyf_lin / std::max(1.0, mu * Fz_f));
    const double Fyr = mu * Fz_r * std::tanh(Fyr_lin / std::max(1.0, mu * Fz_r));
    const double Fyt = mu * Fz_t * std::tanh(Fyt_lin / std::max(1.0, mu * Fz_t));

    // 质量矩阵
    Eigen::Matrix3d M;
    M << m1 + m2,       -m2 * c,             -m2 * d,
         -m2 * c,        Iz1 + m2 * c * c,    m2 * c * d,
         -m2 * d,        m2 * c * d,          Iz2 + m2 * d * d;
    M += 1e-6 * Eigen::Matrix3d::Identity();

    // 广义力
    Eigen::Vector3d F;
    F(0) = Fyf * std::cos(delta) + Fyr + Fyt - (m1 + m2) * vx_safe * r1;
    F(1) = a * Fyf * std::cos(delta) - b * Fyr - c * Fyt + m2 * c * vx_safe * r1;
    F(2) = -L2 * Fyt + m2 * d * vx_safe * r1;

    Eigen::Vector3d accels = M.ldlt().solve(F);
    if (!accels.allFinite()) {
        accels.setZero();
    }

    Eigen::Vector4d x_dot;
    x_dot(0) = accels(0); // vy_dot
    x_dot(1) = accels(1); // r1_dot
    x_dot(2) = accels(2); // r2_dot
    x_dot(3) = r2 - r1;   // gamma_dot
    return x_dot;
}

Eigen::Vector4d ESOTracker2::vehicle_state_3dof(const Eigen::Vector4d& x,
                                                const Eigen::Vector2d& u,
                                                double dt) {
    const double dt_safe = std::max(1e-3, std::min(0.2, dt));

    // RK2 比欧拉积分更稳定，计算量对 4 维 CKF 仍很小。
    Eigen::Vector4d k1 = compute_dynamics_3dof(x, u);
    Eigen::Vector4d x_mid = x + 0.5 * dt_safe * k1;
    x_mid(3) = normalizeAngle(x_mid(3));
    Eigen::Vector4d k2 = compute_dynamics_3dof(x_mid, u);
    Eigen::Vector4d x_next = x + dt_safe * k2;

    const double vx_safe = std::max(u(1), 1.0);
    const double vy_limit = std::max(3.0, 0.7 * vx_safe);
    x_next(0) = clampFinite(x_next(0), -vy_limit, vy_limit, x(0));
    x_next(1) = clampFinite(x_next(1), -3.0, 3.0, x(1));
    x_next(2) = clampFinite(x_next(2), -3.0, 3.0, x(2));
    x_next(3) = clampFinite(normalizeAngle(x_next(3)), -1.3, 1.3, x(3));
    return x_next;
}

Eigen::Vector4d ESOTracker2::vehicle_meas_3dof(const Eigen::Vector4d& x,
                                               const Eigen::Vector2d& u) {
    const Eigen::Vector4d x_dot = compute_dynamics_3dof(x, u);
    const double vx_safe = std::max(u(1), 1.0);
    const double ay_pred = x_dot(0) + vx_safe * x(1);

    Eigen::Vector4d z_pred;
    z_pred << ay_pred, x(1), x(2), normalizeAngle(x(3));
    return z_pred;
}

void ESOTracker2::ckfEstimate(double curr_vx, double curr_delta, double curr_ay,
                              double curr_r, double pseudo_r2, double pseudo_gamma,
                              double dt, bool measurement_is_new) {
    constexpr int nx = 4, m_pts = 2 * nx;
    const double dt_safe = std::max(1e-3, std::min(0.2, dt));
    const double vx_safe = std::max(curr_vx, 1.0);
    Eigen::Vector2d u(curr_delta, vx_safe);
    Eigen::Vector4d z_meas(curr_ay, curr_r, pseudo_r2, normalizeAngle(pseudo_gamma));
    if (!finiteVec4(ckf_x_hat_) || !finiteMat4(ckf_P_)) {
        ckf_x_hat_ << 0.0, curr_r, pseudo_r2, normalizeAngle(pseudo_gamma);
        ckf_P_ = Eigen::Matrix4d::Identity();
    }
    // 首帧/重置后，用可测状态对 r1/r2/gamma 对齐，避免 CKF 初始瞬态过大。
    if (std::abs(ckf_x_hat_(1)) < 1e-6 && std::abs(curr_r) > 1e-6) {
        ckf_x_hat_(1) = curr_r;}
    if (std::abs(ckf_x_hat_(2)) < 1e-6 && std::abs(pseudo_r2) > 1e-6) {
        ckf_x_hat_(2) = pseudo_r2;}
    if (std::abs(ckf_x_hat_(3)) < 1e-6 && std::abs(pseudo_gamma) > 1e-6) {
        ckf_x_hat_(3) = normalizeAngle(pseudo_gamma);}
    // 过程噪声与测量噪声。ay 噪声不要设得过小，否则 vy 会追随加速度噪声抖动。
    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    Q.diagonal() << 5e-3, 5e-5, 1e-4, 2e-5;
    Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
    R.diagonal() << 5e-2, 2.5e-4, 2e-3, 2e-3;
    // ==================== 1. 时间更新：预测步 ====================
    ckf_P_ = regularizeCovariance4(ckf_P_);
    const Eigen::Matrix4d S = sqrtCovariance4(ckf_P_);
    Eigen::Matrix<double, nx, m_pts> xi;
    xi.leftCols<nx>() = Eigen::Matrix4d::Identity();
    xi.rightCols<nx>() = -Eigen::Matrix4d::Identity();
    xi *= std::sqrt(static_cast<double>(nx));
    Eigen::Matrix<double, nx, m_pts> X_pts;
    for (int i = 0; i < m_pts; ++i) {
        const Eigen::Vector4d dx = (S * xi.col(i)).eval();
        X_pts.col(i) = ckf_x_hat_ + dx;
        X_pts(3, i) = normalizeAngle(X_pts(3, i));
    }
    Eigen::Matrix<double, nx, m_pts> X_pred_pts;
    for (int i = 0; i < m_pts; ++i) {
        X_pred_pts.col(i) = vehicle_state_3dof(X_pts.col(i), u, dt_safe);
    }
    Eigen::Vector4d x_pred = X_pred_pts.rowwise().mean();
    x_pred(3) = normalizeAngle(x_pred(3));
    Eigen::Matrix4d P_pred = Q;
    for (int i = 0; i < m_pts; ++i) {
        Eigen::Vector4d err = X_pred_pts.col(i) - x_pred;
        err(3) = normalizeAngle(err(3));
        P_pred += (err * err.transpose()) / static_cast<double>(m_pts);
    }
    P_pred = regularizeCovariance4(P_pred);
    if (!measurement_is_new) {
        ckf_x_hat_ = x_pred;
        ckf_P_ = P_pred;
        return;
    }
    // ==================== 2. 测量更新：校正步 ====================
    const Eigen::Matrix4d S_pred = sqrtCovariance4(P_pred);
    Eigen::Matrix<double, nx, m_pts> X_pred_pts_new;
    for (int i = 0; i < m_pts; ++i) {
        const Eigen::Vector4d dx = (S_pred * xi.col(i)).eval();
        X_pred_pts_new.col(i) = x_pred + dx;
        X_pred_pts_new(3, i) = normalizeAngle(X_pred_pts_new(3, i));
    }
    Eigen::Matrix<double, nx, m_pts> Z_pred_pts;
    for (int i = 0; i < m_pts; ++i) {
        Z_pred_pts.col(i) = vehicle_meas_3dof(X_pred_pts_new.col(i), u);
    }
    Eigen::Vector4d z_pred = Z_pred_pts.rowwise().mean();
    z_pred(3) = normalizeAngle(z_pred(3));
    Eigen::Matrix4d P_zz = R;
    Eigen::Matrix4d P_xz = Eigen::Matrix4d::Zero();
    for (int i = 0; i < m_pts; ++i) {
        Eigen::Vector4d err_x = X_pred_pts_new.col(i) - x_pred;
        Eigen::Vector4d err_z = Z_pred_pts.col(i) - z_pred;
        err_x(3) = normalizeAngle(err_x(3));
        err_z(3) = normalizeAngle(err_z(3));
        P_zz += (err_z * err_z.transpose()) / static_cast<double>(m_pts);
        P_xz += (err_x * err_z.transpose()) / static_cast<double>(m_pts);
    }
    P_zz = regularizeCovariance4(P_zz, 1e-7, 1e6);
    Eigen::LDLT<Eigen::Matrix4d> ldlt(P_zz);
    if (ldlt.info() != Eigen::Success) {
        ROS_WARN_THROTTLE(1.0, "[%s] CKF P_zz 分解失败，本帧仅使用预测值", getName().c_str());
        ckf_x_hat_ = x_pred;
        ckf_P_ = P_pred;
    } else {
        Eigen::Matrix4d K_gain = ldlt.solve(P_xz.transpose()).transpose();
        Eigen::Vector4d innovation = z_meas - z_pred;
        innovation(3) = normalizeAngle(innovation(3));
        ckf_x_hat_ = x_pred + K_gain * innovation;
        ckf_x_hat_(3) = normalizeAngle(ckf_x_hat_(3));
        ckf_P_ = P_pred - K_gain * P_zz * K_gain.transpose();
        ckf_P_ = regularizeCovariance4(ckf_P_);
    }
    // 物理限幅，防止异常加速度/伪测量导致 NMPC 初值跳变。
    const double vy_limit = std::max(3.0, 0.7 * vx_safe);
    ckf_x_hat_(0) = clampFinite(ckf_x_hat_(0), -vy_limit, vy_limit, 0.0);
    ckf_x_hat_(1) = clampFinite(ckf_x_hat_(1), -3.0, 3.0, curr_r);
    ckf_x_hat_(2) = clampFinite(ckf_x_hat_(2), -3.0, 3.0, pseudo_r2);
    ckf_x_hat_(3) = clampFinite(normalizeAngle(ckf_x_hat_(3)), -1.3, 1.3, normalizeAngle(pseudo_gamma));
}

// FF-RLS侧偏刚度辨识（保留原函数名，替换为3刚度辨识）
void ESOTracker2::rlsIdentifyStiffness(double curr_vx, double vy_est, double curr_delta,double curr_r, 
                                      double curr_ay, double curr_gamma, double curr_r_t, double M, double dt) {
    if (curr_vx < 3.0) {
        rls_Cf_est_ = std::max(rls_Cf_est_min_, std::min(rls_Cf_est_max_, rls_Cf_est_default_));
        rls_Cr_est_ = std::max(rls_Cr_est_min_, std::min(rls_Cr_est_max_, rls_Cr_est_default_));
        rls_Ct_est_ = std::max(rls_Ct_est_min_, std::min(rls_Ct_est_max_, rls_Ct_est_default_));
        rls_C_out_prev_ << rls_Cf_est_, rls_Cr_est_, rls_Ct_est_;
        rls_P_ = Matrix3d::Identity() * 1e4;
        return;
    }
    const double dt_safe = std::max(1e-3, std::min(0.2, dt));
    double lamda = 0.99999; // 遗忘因子，接近1表示慢速遗忘
    double lf1 = nmpc_params_.lf, lr1 = nmpc_params_.lr;
    double lf2 = nmpc_params_.lt, lh = nmpc_params_.lh;
    double lr2 = nmpc_params_.L2 - lf2;
    double m1 = nmpc_params_.m, m2 = nmpc_params_.m_t_total;
    double Iz1 = nmpc_params_.Iz, Iz2 = nmpc_params_.Iz_t + nmpc_params_.Kiz*(m2-nmpc_params_.m_t);
    // 横摆角加速度滤波
    double alpha_filter = 0.3;
    double w1_dot_raw = (curr_r - rls_w1_prev_) / dt_safe;
    double w2_dot_raw = (curr_r_t - rls_w2_prev_) / dt_safe;
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
    Eigen::FullPivLU<Matrix3d> force_solver(A);
    if (!force_solver.isInvertible()) {
        ROS_WARN_THROTTLE(1.0, "[%s] RLS侧向力方程病态，本帧跳过参数更新", getName().c_str());
        return;
    }
    Vector3d Fy = force_solver.solve(B);
    if (!Fy.allFinite()) return;
    // 计算侧偏角
    Matrix2d rot;
    rot << cos(curr_gamma), sin(curr_gamma),
          -sin(curr_gamma), cos(curr_gamma);
    Vector2d b_vec;
    b_vec << lf2*curr_r_t*sin(curr_gamma) - curr_vx,
             vy_est - lh*curr_r - lf2*curr_r_t*cos(curr_gamma);
    double vx1_safe = max(curr_vx, 1.0);
    // rot 为正交旋转矩阵，转置即逆；避免通用矩阵求逆带来的额外数值误差。
    Vector2d V2 = rot.transpose() * b_vec;
    double V2_x_safe = max(V2(0), 1.0);
    // 计算侧偏角
    double alpha1 = curr_delta - atan((vy_est + lf1*curr_r)/vx1_safe);
    double alpha2 = -atan((vy_est - lr1*curr_r)/vx1_safe);
    double alpha3 = -atan((V2(1) - lr2*curr_r_t)/V2_x_safe);
    Vector3d alpha(alpha1, alpha2, alpha3);
    // RLS递推
    double max_alpha = alpha.cwiseAbs().maxCoeff();
    if (max_alpha > 0.01 && max_alpha < 0.15 && std::abs(curr_ay) > 0.1) {
        Vector3d Y = Fy; 
        Matrix3d Phi = alpha.asDiagonal();
        // 增益矩阵计算
        const Matrix3d innovation_cov =
            lamda * Matrix3d::Identity() + Phi * rls_P_ * Phi.transpose();
        Eigen::LDLT<Matrix3d> innovation_solver(innovation_cov);
        if (innovation_solver.info() != Eigen::Success) return;
        Matrix3d K = innovation_solver.solve((rls_P_ * Phi.transpose()).transpose()).transpose();
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
    double smooth_factor = 0.01;
    rls_C_out_prev_ = smooth_factor * Vector3d(rls_Cf_est_, rls_Cr_est_, rls_Ct_est_) + (1 - smooth_factor) * rls_C_out_prev_;
    rls_Cf_est_ = rls_C_out_prev_(0);   
    rls_Cr_est_ = rls_C_out_prev_(1);
    rls_Ct_est_ = rls_C_out_prev_(2);
    // 输出侧偏刚度估计值（橙色）
    ROS_INFO_THROTTLE(1.0,
                      "[%s] 侧偏刚度估计: Cf=%.1f, Cr=%.1f, Ct=%.1f N/rad",
                      getName().c_str(), rls_Cf_est_, rls_Cr_est_, rls_Ct_est_);
}


void ESOTracker2::calculate_trailer_kinematics(double curr_vx, double curr_r, double dt) {
    const double L2 = nmpc_params_.L2;
    const double Lh = nmpc_params_.lh;          // 建议用参数，不要写死0
    const double vx = std::max(std::abs(curr_vx), 0.0);
    // 牵引车横摆率直接采用状态量，并做轻微低通抑制噪声
    const double tau_r = 0.08;                  // 可调: 0.05~0.15
    if (!r_filter_initialized_) {
        r_tractor_filt_ = curr_r;
        r_filter_initialized_ = true;}
    const double dt_safe = std::max(1e-3, std::min(0.2, dt));
    const double alpha_r = dt_safe / (tau_r + dt_safe);
    r_tractor_filt_ = (1.0 - alpha_r) * r_tractor_filt_ + alpha_r * curr_r;
    const double r_tractor = r_tractor_filt_;
    // 挂车横摆率运动学估计
    r_t_ = -(vx * std::sin(gamma_) + Lh * r_tractor * std::cos(gamma_)) / L2;
    const double gamma_dot = r_t_ - r_tractor;
    gamma_ = normalizeAngle(gamma_ + gamma_dot * dt_safe);
    gamma_ = std::max(-1.3, std::min(1.3, gamma_));
}


MX ESOTracker2::vehicleDynamicsModel(const MX& state, const MX& cmd_delta,const MX& vx, const MX& h_dist,const MX& dyn_params) {
    // ================== 状态量读取 ==================
    MX theta = state(2);
    MX vy    = state(3);
    MX r     = state(4);
    MX delta = state(5);
    MX r_t   = state(6);
    MX gamma = state(7);
    // ================== 动力学参数读取 ==================
    MX m1  = dyn_params(0);
    MX Iz1 = dyn_params(1);
    MX lf  = dyn_params(2);
    MX lr  = dyn_params(3);
    MX Cf  = dyn_params(4);
    MX Cr  = dyn_params(5);
    MX M   = dyn_params(6);
    MX Iz2 = dyn_params(7);
    MX lt  = dyn_params(8);
    MX Ct  = dyn_params(9);
    MX L2  = dyn_params(10);
    // dyn_params(6) 与 YAML 的 m_t_total 均表示挂车质量，不含牵引车。
    MX m2 = fmax(M, 1.0);
    // ================== 数值保护 ==================
    MX vx_safe = fmax(vx, 0.5);
    MX cg = cos(gamma);
    MX sg = sin(gamma);
    // ================== 牵引车轮胎侧偏角与侧向力 ==================
    MX alpha_f = delta - atan2((vy + lf * r), vx_safe);
    MX alpha_r = -atan2((vy - lr * r), vx_safe);
    MX Fyf = Cf * alpha_f;
    MX Fyr = Cr * alpha_r;
    // ================== 铰接点速度 ==================
    MX vx_h1 = vx_safe;
    MX vy_h1 = vy - lr * r;
    MX vx_h2 = vx_h1 * cg + vy_h1 * sg;
    MX vy_h2 = -vx_h1 * sg + vy_h1 * cg;
    // ================== 挂车轴速度与挂车侧向力 ==================
    MX vx2 = vx_h2;
    MX vy2 = vy_h2 - L2 * r_t;
    MX vx2_safe = fmax(vx2, 0.5);
    MX alpha_t = -atan2(vy2, vx2_safe);
    MX Fyt = Ct * alpha_t;
    // ================== 广义力 F1, F2, F3 ==================
    MX Fyt_y1 = Fyt * cg;
    MX F1=Fyf*cos(delta)+Fyr+Fyt_y1-(m1+m2)*vx_safe*r-m2*lt*r_t*r_t*sg;
    MX F2=lf*Fyf*cos(delta)-lr*Fyr-lr*Fyt_y1+m2*lr*vx_safe*r+m2*lr*lt*r_t*r_t*sg;
    MX F3=-L2*Fyt+m2*lt*r*(vx_safe*cg+(vy-lr*r)*sg);
    MX A=m1+m2;
    MX S11=Iz1+m2*lr*lr-(m2*m2*lr*lr)/A;
    MX S12=m2*lr*lt*cg-(m2*m2*lr*lt*cg)/A;
    MX S22=Iz2+m2*lt*lt-(m2*m2*lt*lt*cg*cg)/A;
    MX b1=F2+(m2*lr/A)*F1;
    MX b2=F3+(m2*lt*cg/A)*F1;
    MX detS=S11*S22-S12*S12;
    // 这里加一个很小的正则项，避免符号优化中出现除零风险。
    MX detS_safe = detS + 1e-9;
    MX d_r_nominal=(b1*S22-b2*S12)/detS_safe;
    MX d_r_t=(S11*b2-S12*b1)/detS_safe;
    MX d_vy=(F1+m2*lr*d_r_nominal+m2*lt*cg*d_r_t)/A;
    // ================== ESO 扰动引入 ==================
    // MX d_r=d_r_nominal;
    MX d_r=d_r_nominal+h_dist;
    // ================== 运动学状态更新 ==================
    MX d_gamma=r_t-r;   
    MX d_x=vx_safe*cos(theta)-vy*sin(theta);
    MX d_y=vx_safe*sin(theta)+vy*cos(theta);
    MX d_theta=r;
    MX d_delta=(cmd_delta-delta)/std::max(nmpc_params_.T_lag, 1e-3);
    std::vector<MX> state_derivatives = {d_x,d_y,d_theta,d_vy,d_r,d_delta,d_r_t,d_gamma};
    return vertcat(state_derivatives);
}

// 构建NMPC求解器（替换为挂车约束，保留函数名）
void ESOTracker2::buildNMPSolver() {
    solver_.opti = Opti();
    int nx = nmpc_params_.nx, nu = nmpc_params_.nu, N = nmpc_params_.N, Nc = nmpc_params_.Nc;

    solver_.X = solver_.opti.variable(nx, N+1);
    solver_.U_sparse = solver_.opti.variable(nu, Nc);
    solver_.P_x0 = solver_.opti.parameter(nx);
    solver_.P_waypoints = solver_.opti.parameter(8, N+1);
    solver_.P_vx = solver_.opti.parameter(1);
    solver_.P_u_prev = solver_.opti.parameter(1);
    solver_.P_h_hat = solver_.opti.parameter(1);
    solver_.P_dyn_params = solver_.opti.parameter(11); // 扩展为11维动力学参数

    // 与 ESOTracker 对齐的前密后疏 move blocking。
    solver_.control_block_start.clear();
    solver_.control_block_length.clear();
    const int dense_blocks = std::max(0, std::min(
        nmpc_params_.near_dense_control_steps, std::min(N, Nc - 1)));
    int assigned_steps = 0;
    for (int i = 0; i < dense_blocks; ++i) {
        solver_.control_block_start.push_back(assigned_steps++);
        solver_.control_block_length.push_back(1);
    }
    const int remaining_blocks = Nc - dense_blocks;
    for (int i = 0; i < remaining_blocks; ++i) {
        const int blocks_left = remaining_blocks - i;
        const int steps_left = N - assigned_steps;
        const int steps = std::max(1, steps_left / blocks_left);
        solver_.control_block_start.push_back(assigned_steps);
        solver_.control_block_length.push_back(steps);
        assigned_steps += steps;
    }
    if (!solver_.control_block_length.empty() && assigned_steps < N) {
        solver_.control_block_length.back() += N - assigned_steps;
        assigned_steps = N;
    }
    if (assigned_steps != N || static_cast<int>(solver_.control_block_length.size()) != Nc) {
        throw std::runtime_error("ESOTracker2 control block layout invalid");
    }

    // U_sparse 为相对稳态前馈的反馈修正量，物理命令为 delta_ff + feedback。
    solver_.U_full_feedback = MX::zeros(nu, N);
    for (int i = 0; i < Nc; ++i) {
        const int begin = solver_.control_block_start[i];
        const int length = solver_.control_block_length[i];
        solver_.U_full_feedback(Slice(), Slice(begin, begin + length)) =
            repmat(solver_.U_sparse(Slice(), i), 1, length);
    }
    solver_.U_full_command = MX::zeros(nu, N);
    for (int k = 0; k < N; ++k) {
        solver_.U_full_command(0, k) =
            solver_.P_waypoints(4, k + 1) + solver_.U_full_feedback(0, k);
    }


    MX J = 0.0;
    solver_.opti.subject_to(solver_.X(Slice(), 0) == solver_.P_x0);

    for (int k=0; k<N; k++) {
        MX st = solver_.X(Slice(), k);
        MX con = solver_.U_full_command(Slice(), k);
        MX feedback = solver_.U_full_feedback(Slice(), k);
        MX h = solver_.P_h_hat * pow(nmpc_params_.eso_disturbance_decay, k);

        if (nmpc_params_.integration_grade >= 0.5 && nmpc_params_.integration_grade < 1.5) {
            solver_.opti.subject_to(solver_.X(Slice(), k+1) == st + nmpc_params_.dt * vehicleDynamicsModel(st, con, solver_.P_vx, h, solver_.P_dyn_params));
        }

        else if (nmpc_params_.integration_grade >= 1.5 && nmpc_params_.integration_grade < 2.5) {
            // RK2积分（新增）
            MX k1 = vehicleDynamicsModel(st, con, solver_.P_vx, h, solver_.P_dyn_params);
            MX k2 = vehicleDynamicsModel(st + nmpc_params_.dt / 2.0 * k1, con, solver_.P_vx, h, solver_.P_dyn_params);
            solver_.opti.subject_to(solver_.X(Slice(), k+1) == st + nmpc_params_.dt * k2);
        }
        else if (nmpc_params_.integration_grade >= 3.5 && nmpc_params_.integration_grade < 4.5) {
            // RK4积分（保留原逻辑）
            MX k1 = vehicleDynamicsModel(st, con, solver_.P_vx, h, solver_.P_dyn_params);
            MX k2 = vehicleDynamicsModel(st + nmpc_params_.dt/2 * k1, con, solver_.P_vx, h, solver_.P_dyn_params);
            MX k3 = vehicleDynamicsModel(st + nmpc_params_.dt/2 * k2, con, solver_.P_vx, h, solver_.P_dyn_params);
            MX k4 = vehicleDynamicsModel(st + nmpc_params_.dt * k3, con, solver_.P_vx, h, solver_.P_dyn_params);
            solver_.opti.subject_to(solver_.X(Slice(), k+1) == st + nmpc_params_.dt/6 * (k1 + 2*k2 + 2*k3 + k4));
        }
        else {
            throw std::runtime_error("integration_grade不在有效范围内");
        }

        // 代价函数（扩展挂车项）
        MX ref_x = solver_.P_waypoints(0, k+1), ref_y = solver_.P_waypoints(1, k+1);
        MX ref_theta = solver_.P_waypoints(2, k+1), ref_kappa = solver_.P_waypoints(3, k+1);
        MX ref_delta = solver_.P_waypoints(4, k+1), ref_vy = solver_.P_waypoints(5, k+1);
        MX ref_r_t = solver_.P_waypoints(6, k+1), ref_gamma = solver_.P_waypoints(7, k+1);
        // 体坐标系下，X(2) 与 ref_theta 均为连续、锚定在 0 附近的相对航向，
        // 不会跨越 ±pi 折叠边界，因此直接作差即可，无需 atan2(sin,cos) 折叠。
        // 移除折叠后，代价函数对航向误差全程光滑可导，消除 ±pi/2 附近的梯度突变。
        MX e_theta = solver_.X(2, k+1) - ref_theta;
        MX r_ref = solver_.P_vx * ref_kappa;
        MX gamma_dot_actual = solver_.X(6, k+1) - solver_.X(4, k+1);

        J += nmpc_params_.Q(0,0) * pow(solver_.X(0, k+1) - ref_x, 2);
        J += nmpc_params_.Q(1,1) * pow(solver_.X(1, k+1) - ref_y, 2);
        J += nmpc_params_.Q(2,2) * pow(e_theta, 2);
        J += nmpc_params_.Q(3,3) * pow(solver_.X(3, k+1) - ref_vy, 2);
        J += nmpc_params_.Q(4,4) * pow(solver_.X(4, k+1) - r_ref, 2);
        J += nmpc_params_.Q(5,5) * pow(solver_.X(5, k+1) - ref_delta, 2);
        J += nmpc_params_.Q(6,6) * pow(solver_.X(6, k+1) - ref_r_t, 2);
        J += nmpc_params_.Q(7,7) * pow(solver_.X(7, k+1) - ref_gamma, 2);
        J += nmpc_params_.dgamma * pow(gamma_dot_actual, 2);
        J += nmpc_params_.R * pow(feedback, 2);

        const MX previous_command = (k == 0)
            ? solver_.P_u_prev : solver_.U_full_command(0, k - 1);
        const MX command_increment = solver_.U_full_command(0, k) - previous_command;
        J += nmpc_params_.dR * pow(command_increment, 2);
        solver_.opti.subject_to(solver_.opti.bounded(
            nmpc_params_.delta_rate_min * nmpc_params_.dt,
            command_increment,
            nmpc_params_.delta_rate_max * nmpc_params_.dt));
    }
    solver_.opti.subject_to(solver_.opti.bounded(
        nmpc_params_.delta_min, solver_.U_full_command, nmpc_params_.delta_max));

    solver_.opti.minimize(J);

    // IPOPT参数（Matlab实时性优化）
    Dict opts = {
        {"ipopt.print_level", 0}, 
        {"ipopt.sb", "yes"}, 
        {"ipopt.max_iter", ipopt_max_iter_},
        {"ipopt.tol", 1e-2},
        {"ipopt.acceptable_tol", ipopt_acceptable_tol_},
        {"ipopt.acceptable_iter", ipopt_acceptable_iter_},
        {"ipopt.mu_strategy", "adaptive"},
        {"ipopt.max_cpu_time", nmpc_ipopt_cpu_time_limit_ms_ / 1000.0},
        {"print_time", 0},
        {"expand", true},

        {"ipopt.warm_start_init_point", "yes"},        // 明确告诉求解器接受热启动
        {"ipopt.warm_start_bound_push", ipopt_warm_start_bound_push_},        
        {"ipopt.warm_start_slack_bound_push", ipopt_warm_start_slack_bound_push_},
        {"ipopt.warm_start_mult_bound_push", ipopt_warm_start_mult_bound_push_}
    };
    solver_.opti.solver("ipopt", opts);
}

// 求解NMPC
bool ESOTracker2::solveNMPC(const std::vector<double>& current_state, const casadi::DM& waypoints,
                          std::vector<double>& control_output) {

    const auto start_time = std::chrono::steady_clock::now();
    last_nmpc_deadline_missed_ = false;
    last_nmpc_solver_returned_success_ = false;
    last_nmpc_warm_start_used_ = solver_.has_prev_sol && static_cast<bool>(solver_.sol_prev);

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
        last_nmpc_solver_returned_success_ = true;
        captureNmpcSolverStats();

        const double elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start_time).count();
        if (elapsed_ms > nmpc_solve_deadline_ms_) {
            last_nmpc_deadline_missed_ = true;
            ++nmpc_timeout_count_;
            last_nmpc_status_code_ = 2;
            last_nmpc_return_status_ += ";rejected_wall_deadline";
            solver_.has_prev_sol = false;
            solver_.sol_prev = nullptr;
            return false;
        }
        ROS_INFO_THROTTLE(1.0, "[%s] NMPC求解成功，耗时 %.2f ms",
                          getName().c_str(), elapsed_ms);
        
        solver_.sol_prev = std::make_unique<casadi::OptiSol>(sol);
        solver_.has_prev_sol = true;


        last_nmpc_status_code_ = 1;
        const double command = static_cast<double>(
            sol.value(solver_.U_full_command(0, 0)));
        if (!std::isfinite(command)) {
            last_nmpc_status_code_ = 3;
            last_nmpc_return_status_ += ";nonfinite_command";
            solver_.has_prev_sol = false;
            solver_.sol_prev = nullptr;
            return false;
        }
        control_output[0] = command;
        return true;
    } catch (std::exception& e) {
        const double elapsed_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start_time).count();
        const std::string reason(e.what());
        const bool timeout = elapsed_ms >= nmpc_solve_deadline_ms_ ||
            reason.find("Maximum_CpuTime_Exceeded") != std::string::npos ||
            reason.find("max_cpu_time") != std::string::npos;
        if (timeout) {
            last_nmpc_deadline_missed_ = true;
            ++nmpc_timeout_count_;
        }
        captureNmpcSolverStats();
        last_nmpc_status_code_ = timeout ? 2 : 3;
        ROS_WARN_THROTTLE(0.5, "[%s] NMPC求解失败，耗时 %.2f ms，timeout=%d: %s",
                          getName().c_str(), elapsed_ms, timeout ? 1 : 0, e.what());
        solver_.has_prev_sol = false;
        solver_.sol_prev = nullptr;
        return false;
    }
}

// ---------------------- [新增] 纯跟踪兜底保护实现 ----------------------
double ESOTracker2::computePurePursuitSteering(const race_msgs::Path& path,double curr_x, double curr_y,double curr_theta, double lookahead_dist) {
    if (path.points.empty()) return 0.0;
    int nearest_idx = find_nearest_path_point(curr_x, curr_y, curr_theta, path);
    int target_idx = nearest_idx;
    // 目标点按路径累计弧长选择；横向大偏差不会被误算成前视距离。
    double arc_length = 0.0;
    for (int i = nearest_idx + 1; i < static_cast<int>(path.points.size()); ++i) {
        const auto& p0 = path.points[i - 1].pose.position;
        const auto& p1 = path.points[i].pose.position;
        arc_length += std::hypot(p1.x - p0.x, p1.y - p0.y);
        target_idx = i;
        if (arc_length >= lookahead_dist) break;
    }
    const auto& target_pt = path.points[target_idx].pose.position;
    double dx = target_pt.x - curr_x;
    double dy = target_pt.y - curr_y;
    // 旋转矩阵：将世界坐标误差转换为车体坐标
    double local_x = std::cos(curr_theta) * dx + std::sin(curr_theta) * dy;
    double local_y = -std::sin(curr_theta) * dx + std::cos(curr_theta) * dy;
    double L = nmpc_params_.lf + nmpc_params_.lr; // 牵引车轴距
    double ld = std::max(lookahead_dist, std::sqrt(local_x*local_x + local_y*local_y));
    double delta_pp = std::atan2(2.0 * L * local_y, ld * ld);
    return std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, delta_pp));
}

bool ESOTracker2::isNewVehicleMeasurement(double x, double y, double yaw,
                                           double vx, double vy, double r,
                                           double delta, double ay) {
    if (!measurement_fingerprint_valid_) {
        measurement_fingerprint_valid_ = true;
        measurement_prev_x_ = x;
        measurement_prev_y_ = y;
        measurement_prev_yaw_ = yaw;
        measurement_prev_vx_ = vx;
        measurement_prev_vy_ = vy;
        measurement_prev_r_ = r;
        measurement_prev_delta_ = delta;
        measurement_prev_ay_ = ay;
        return true;
    }
    const bool changed =
        std::hypot(x - measurement_prev_x_, y - measurement_prev_y_) > 1e-4 ||
        std::abs(angleDiff(yaw, measurement_prev_yaw_)) > 1e-5 ||
        std::abs(vx - measurement_prev_vx_) > 1e-4 ||
        std::abs(vy - measurement_prev_vy_) > 1e-4 ||
        std::abs(r - measurement_prev_r_) > 1e-5 ||
        std::abs(delta - measurement_prev_delta_) > 1e-5 ||
        std::abs(ay - measurement_prev_ay_) > 1e-4;
    if (changed) {
        measurement_prev_x_ = x;
        measurement_prev_y_ = y;
        measurement_prev_yaw_ = yaw;
        measurement_prev_vx_ = vx;
        measurement_prev_vy_ = vy;
        measurement_prev_r_ = r;
        measurement_prev_delta_ = delta;
        measurement_prev_ay_ = ay;
    }
    return changed;
}

void ESOTracker2::esoCompute(double curr_vy,double curr_r,double curr_delta,double curr_r_t,
                             double curr_gamma,double curr_vx,double dt,
                             bool measurement_is_new) {
    if (dt <= 1e-6) {return;}
    const double omega_o = 10.0;
    const double B1 = 2.0 * omega_o;
    const double B2 = omega_o * omega_o;
    if (!eso_initialized_) {
        eso_x1_ = curr_r;
        eso_x2_ = 0.0;
        eso_initialized_ = true;
    }
    const double d_r_nominal = calcNominalYawAccel(curr_vy,curr_r,curr_delta,curr_r_t,curr_gamma,curr_vx);
    const double error_eso = measurement_is_new ? (curr_r - eso_x1_) : 0.0;
    const double eso_x1_dot = d_r_nominal + eso_x2_ + B1 * error_eso;
    const double eso_x2_dot = B2 * error_eso;
    eso_x1_ += eso_x1_dot * dt;
    eso_x2_ += eso_x2_dot * dt;
    const double max_yaw_acc_dist = 5.0;  // rad/s^2
    eso_x2_ = std::max(-max_yaw_acc_dist,std::min(max_yaw_acc_dist, eso_x2_));
}

double ESOTracker2::calcNominalYawAccel(double curr_vy,double curr_r,double curr_delta,double curr_r_t,double curr_gamma,double curr_vx) {
    // ================== 参数读取 ==================
    const double m1  = nmpc_params_.m;
    const double Iz1 = nmpc_params_.Iz;
    const double lf  = nmpc_params_.lf;
    const double lr  = nmpc_params_.lr;
    const double Cf  = rls_Cf_est_;
    const double Cr  = rls_Cr_est_;
    const double Ct  = rls_Ct_est_;
    const double M   = nmpc_params_.m_t_total;
    const double Iz2 = nmpc_params_.Iz_t+ nmpc_params_.Kiz*(nmpc_params_.m_t_total- nmpc_params_.m_t);
    const double lt  = nmpc_params_.lt;
    const double L2  = nmpc_params_.L2;
    const double m2  = std::max(M, 1.0);
    // ================== 数值保护 ==================
    const double vx_safe = std::max(curr_vx, 0.5);
    const double cg = std::cos(curr_gamma);
    const double sg = std::sin(curr_gamma);
    // ================== 牵引车轮胎侧偏角与侧向力 ==================
    const double alpha_f = curr_delta - std::atan2(curr_vy + lf * curr_r, vx_safe);
    const double alpha_r = -std::atan2(curr_vy - lr * curr_r, vx_safe);
    const double Fyf = Cf * alpha_f;
    const double Fyr = Cr * alpha_r;
    // ================== 铰接点速度 ==================
    const double vx_h1 = vx_safe;
    const double vy_h1 = curr_vy-lr*curr_r;
    const double vx_h2 = vx_h1*cg+vy_h1*sg;
    const double vy_h2 = -vx_h1*sg+vy_h1*cg;
    // ================== 挂车轴速度与挂车侧向力 ==================
    const double vx2 = vx_h2;
    const double vy2 = vy_h2-L2*curr_r_t;
    const double vx2_safe = std::max(vx2, 0.5);
    const double alpha_t = -std::atan2(vy2, vx2_safe);
    const double Fyt = Ct * alpha_t;
    // ================== 广义力 F1, F2, F3 ==================
    const double Fyt_y1 = Fyt * cg;
    const double F1 = Fyf*std::cos(curr_delta)+Fyr+Fyt_y1-(m1+m2)*vx_safe*curr_r-m2*lt*curr_r_t*curr_r_t*sg;
    const double F2 = lf*Fyf*std::cos(curr_delta)-lr*Fyr-lr*Fyt_y1+m2*lr*vx_safe*curr_r+m2*lr*lt*curr_r_t*curr_r_t*sg;
    const double F3 = -L2*Fyt+m2*lt*curr_r*(vx_safe*cg+(curr_vy-lr*curr_r)*sg);
    const double A = m1 + m2;
    const double S11 = Iz1+m2*lr*lr-(m2*m2*lr*lr)/A;
    const double S12 = m2*lr*lt*cg-(m2*m2*lr*lt*cg)/A;
    const double S22 = Iz2+m2*lt*lt-(m2*m2*lt*lt*cg*cg)/A;
    const double b1 = F2+(m2*lr/A) * F1;
    const double b2 = F3+(m2*lt*cg/A) * F1;
    double detS = S11*S22-S12*S12;
    // ================== 防止除零 ==================
    if (std::abs(detS) < 1e-9) {detS = (detS >= 0.0) ? 1e-9 : -1e-9;}
    // ================== 求解牵引车横摆角加速度 ==================
    const double d_r_nominal=(b1*S22-b2*S12)/detS;
    return d_r_nominal;
}

} // namespace race_tracker

// 保留原插件注册（完全不变）
PLUGINLIB_EXPORT_CLASS(race_tracker::ESOTracker2, race_tracker::ControllerPluginBase)

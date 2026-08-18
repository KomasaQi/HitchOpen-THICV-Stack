#include "race_tracker/eso_tracker.h"
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <numeric>
#include <limits>
#include <stdexcept>
#include <chrono>
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

using namespace casadi;
using namespace Eigen;
using namespace std;

namespace race_tracker {

// -----------------------------------------------------------------------------
// NMPCParams 构造函数实现
// -----------------------------------------------------------------------------
NMPCParams::NMPCParams() {
    // 参数尚未从 ROS/YAML 加载，构造阶段只做确定性清零；加载完成后再调用 updateQMatrix()。
    Q.setZero();
}

void NMPCParams::updateQMatrix() {
    Q.setZero();

    if (Q.rows() >= 6 && Q.cols() >= 6) {
        Q(0,0) = Q_x; Q(1,1) = Q_y; Q(2,2) = Q_theta;
        Q(3,3) = Q_vy; Q(4,4) = Q_r; Q(5,5) = Q_delta;
    }
}

// -----------------------------------------------------------------------------
// ESOTracker 构造函数 (初始化所有变量)
// -----------------------------------------------------------------------------
ESOTracker::ESOTracker() {
    // --- 1. 基础状态初始化 ---
    blend_alpha_ = 0.0;
    nmpc_safe_cmd_ = 0.0;
    start_time_ = ros::Time(0);
    current_cmd_ = 0.0;
    model_r_comp_ = 0.0;
    model_comp_initialized_ = false;
    // --- 2. 求解器状态初始化 ---
    solver_.has_prev_sol = false;
    solver_.sol_prev = nullptr;

    // --- 3. 观测器初始化  ---
    // ESO
    eso_x1_ = 0.0;
    eso_x2_ = 0.0;

    // UKF
    ukf_x_est_ = Vector2d::Zero();
    ukf_P_est_ = (Matrix2d() << 1.0, 0.0, 0.0, 0.1).finished();

    min_lookahead_distance_ = 6.0;  // 默认最小预瞄距 6m
    lookahead_speed_coeff_ = 0.7;   // 默认速度系数 0.7
    curvature_smoothing_steps_ = 5;
    curvature_smoothing_distance_m_ = 6.0;
    lookahead_curvature_coeff_ = 0.0; // 默认关闭曲率预瞄修正

    use_equilibrium_feedforward_ = true;
    equilibrium_feedforward_gain_ = 1.0;
    equilibrium_feedforward_limit_ = 0.45;
    last_delta_ff_ = 0.0;
    path_projection_heading_weight_m2_ = 4.0;
    path_projection_heading_gate_rad_ = 1.2;
    path_projection_rear_gate_m_ = 5.0;

    // 标定与横坡/ay 补偿默认值
    const_steer_bias_ = 0.0;
    use_slope_compensation_ = false;
    ay_slope_compensation_ = 0.0;
    slope_compensation_coeff_ = 1.0;
    slope_compensation_filter_tau_ = 1.0;
    ay_slope_compensation_initialized_ = false;
    slope_estimator_mode_ = "legacy_quasistatic";
    slope_dynamic_gate_enabled_ = true;
    slope_gate_max_yaw_accel_ = 0.15;
    slope_gate_max_steer_rate_ = 0.08;
    slope_compensation_limit_ = 0.8;
    slope_gate_active_ = false;
    slope_prev_valid_ = false;
    slope_prev_r_ = 0.0;
    slope_prev_delta_ = 0.0;

    ukf_use_slope_disturbance_ = true;
    ukf_q_vy_ = 0.01;
    ukf_q_r_ = 0.001;
    ukf_r_ay_ = 0.5;
    ukf_r_r_ = 0.1;
    ukf_ay_innovation_limit_ = 0.8;
    ukf_vy_abs_max_ = 3.0;
    ukf_ay_innovation_raw_ = 0.0;
    ukf_ay_innovation_used_ = 0.0;
    observer_dynamic_min_speed_mps_ = 4.0;

    use_ay_bias_compensation_ = false;
    const_ay_bias_ = 0.0;
    use_dynamic_ay_compensation_ = false;
    ay_bias_estimate_ = 0.0;
    effective_ay_bias_ = 0.0;
    dynamic_ay_error_window_size_ = 200;
    dynamic_ay_error_threshold_ = 0.02;
    dynamic_ay_bias_learning_rate_ = 2.0e-5;
    dynamic_ay_bias_max_step_ = 5.0e-5;
    dynamic_ay_bias_min_ = -1.0;
    dynamic_ay_bias_max_ = 1.0;
    dynamic_ay_bias_error_sign_ = -1;
    dynamic_ay_require_full_window_ = true;

    // NMPC求解以及算法切换相关
    mpc_failure_flag_ = false;
    using_pure_pursuit_flag_ = false;
    require_over_take_flag_ = false;
    using_mixed_mode_flag_ = false;

    resetNmpcPredictionDiagnostics();
}

ESOTracker::~ESOTracker() {
    if (local_log_stream_.is_open()) {
        local_log_stream_.flush();
        local_log_stream_.close();
    }
}

void ESOTracker::resetNmpcPredictionDiagnostics() {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    diagnostic_pred_k1_.fill(nan);
    diagnostic_pred_k5_.fill(nan);
    diagnostic_pred_kN_.fill(nan);
    diagnostic_u_sparse_.fill(nan);
}

void ESOTracker::initializeLocalLog() {
    if (!enable_local_log_) {
        return;
    }
    if (local_log_directory_.empty()) {
        ROS_ERROR("[%s] 本地日志目录为空，关闭本地日志", getName().c_str());
        enable_local_log_ = false;
        return;
    }

    if (::mkdir(local_log_directory_.c_str(), 0755) != 0 && errno != EEXIST) {
        ROS_ERROR("[%s] 无法创建本地日志目录 %s: %s",
                  getName().c_str(), local_log_directory_.c_str(), std::strerror(errno));
        enable_local_log_ = false;
        return;
    }

    const std::time_t now = std::time(nullptr);
    std::tm local_tm;
    localtime_r(&now, &local_tm);
    std::ostringstream filename;
    filename << "eso_tracker_" << std::put_time(&local_tm, "%Y%m%d_%H%M%S")
             << "_pid" << static_cast<long>(::getpid())
             << "_ns" << ros::WallTime::now().toNSec() << ".csv";
    const std::string separator =
        (!local_log_directory_.empty() && local_log_directory_.back() == '/') ? "" : "/";
    local_log_path_ = local_log_directory_ + separator + filename.str();

    local_log_stream_.open(local_log_path_, std::ios::out | std::ios::trunc);
    if (!local_log_stream_.is_open()) {
        ROS_ERROR("[%s] 无法打开本地日志文件 %s",
                  getName().c_str(), local_log_path_.c_str());
        enable_local_log_ = false;
        return;
    }

    local_log_stream_
        << "ros_time_s,log_schema_version,dt_input_s,obs_dt_s,vx_mps,vx_kmh,vy_status_mps,ax_raw_mps2,x_m,y_m,yaw_rad,"
        << "mass_input_kg,model_m_kg,Iz_kgm2,lf_m,lr_m,Cf_Nprad,Cr_Nprad,"
        << "N,Nc,integration_grade,eso_disturbance_decay,T_lag_s,Q_y,Q_theta,Q_r,dR,"
        << "slope_filter_tau_s,yaw_input_gain_1ps2prad,"
        << "Q_x,Q_vy,Q_delta,R,steer_min_rad,steer_max_rad,output_lpf_tau_s,"
        << "use_slope_compensation,slope_compensation_coeff,use_ay_bias_compensation,"
        << "use_dynamic_ay_compensation,auto_update_total_weight,"
        << "lateral_error_m,steer_meas_rad,steer_nmpc_raw_rad,steer_nmpc_cmd_rad,"
        << "steer_pp_raw_rad,steer_pp_cmd_rad,steer_final_rad,blend_alpha,"
        << "kappa_1pm,r_ref_radps,ref_theta_rad,yaw_rate_radps,vy_est_mps,ay_raw_mps2,"
        << "ay_bias_mps2,ay_slope_raw_mps2,ay_slope_filt_mps2,ay_slope_model_input_mps2,"
        << "eso_x1_radps,eso_x2_radps2,eso_disturbance_radps2,model_r_radps,"
        << "lookahead_m,preview_abs_kappa_1pm,nmpc_success,nmpc_iter_ms,"
        << "control_core_ms,mpc_failure_count,using_pp,using_mixed,"
        // 转向执行器归因：上一周期指令驱动的一阶预测与实际转角残差。
        << "steer_meas_raw_rad,steer_meas_was_clamped,steer_prev_final_cmd_rad,"
        << "steer_meas_rate_radps,steer_cmd_rate_radps,"
        << "steer_model_1step_rad,steer_model_residual_rad,steer_nmpc_clamped,steer_final_saturated,"
        // 横摆归因：实测差分、名义模型、ESO 注入前后残差。
        << "yaw_rate_error_radps,yaw_rate_dot_raw_radps2,yaw_rate_dot_nominal_radps2,"
        << "yaw_rate_dot_with_eso_radps2,yaw_residual_nominal_radps2,yaw_residual_with_eso_radps2,"
        << "yaw_model_1step_nominal_radps,yaw_model_1step_eso_radps,"
        << "eso_observation_error_radps,eso_injected_k0_radps2,eso_injected_k1_radps2,"
        // 横向动力学归因：轮胎状态、侧向合力、ay/vy 残差和侧偏角。
        << "alpha_f_rad,alpha_r_rad,Fyf_N,Fyr_N,ay_corrected_mps2,ay_model_no_slope_mps2,"
        << "ay_model_with_slope_mps2,ay_residual_no_slope_mps2,ay_residual_with_slope_mps2,"
        << "vx_dot_raw_mps2,vy_status_dot_raw_mps2,vy_dot_raw_mps2,vy_dot_model_mps2,"
        << "vy_dot_residual_mps2,vy_status_minus_est_mps,beta_status_rad,beta_est_rad,"
        // 路径/定位与运动学归因。
        << "path_size,nearest_idx,nearest_idx_jump,nearest_distance_m,nearest_path_x_m,nearest_path_y_m,"
        << "nearest_path_yaw_rad,geometric_lateral_error_m,tracking_error_minus_geometric_m,"
        << "heading_error_rad,heading_error_rate_kin_radps,r_ref_frenet_radps,"
        << "tracking_error_dot_raw_mps,geometric_error_dot_raw_mps,geometric_error_dot_kin_mps,"
        << "ref_x_k0_m,ref_y_k0_m,ref_theta_k0_rad,ref_kappa_k0_1pm,"
        << "ref_x_k1_m,ref_y_k1_m,ref_theta_k1_rad,ref_kappa_k1_1pm,"
        << "ref_y_k5_m,ref_theta_k5_rad,ref_kappa_k5_1pm,"
        << "ref_y_kN_m,ref_theta_kN_rad,ref_kappa_kN_1pm,"
        // NMPC 求解后预测状态抽样，用于逐层替换回放；不回灌控制器。
        << "pred_k1_x_m,pred_k1_y_m,pred_k1_theta_rad,pred_k1_vy_mps,pred_k1_r_radps,pred_k1_delta_rad,"
        << "pred_k1_y_error_m,pred_k1_heading_error_rad,pred_k1_yaw_rate_error_radps,"
        << "pred_k5_index,pred_k5_y_m,pred_k5_theta_rad,pred_k5_vy_mps,pred_k5_r_radps,pred_k5_delta_rad,"
        << "pred_k5_y_error_m,pred_k5_heading_error_rad,pred_k5_yaw_rate_error_radps,"
        << "pred_kN_y_m,pred_kN_theta_rad,pred_kN_vy_mps,pred_kN_r_radps,pred_kN_delta_rad,"
        << "pred_kN_y_error_m,pred_kN_heading_error_rad,pred_kN_yaw_rate_error_radps,"
        << "u_sparse_0_rad,u_sparse_1_rad,u_sparse_2_rad,target_idx,diagnostic_prev_valid,"
        << "eso_disturbance_tau_s,curvature_smoothing_steps,slope_estimator_mode_code,"
        << "slope_dynamic_gate_enabled,slope_gate_active,slope_gate_yaw_accel_radps2,"
        << "slope_gate_steer_rate_radps,slope_compensation_limit_mps2,"
        << "ukf_use_slope_disturbance,ukf_ay_innovation_raw_mps2,ukf_ay_innovation_used_mps2,"
        << "ukf_q_vy,ukf_q_r,ukf_r_ay,ukf_r_r,ukf_vy_abs_max_mps,"
        << "curvature_smoothing_distance_m,use_equilibrium_feedforward,delta_ff_k0_rad,"
        << "near_dense_control_steps,max_steer_rate_radps,measurement_is_new,"
        << "observer_low_speed_reset,observer_dynamic_min_speed_mps\n";
    local_log_stream_.flush();
    local_log_stream_ << std::fixed << std::setprecision(8);
    local_log_pending_rows_ = 0;
    ROS_INFO("[%s] 本地诊断日志已启用: %s",
             getName().c_str(), local_log_path_.c_str());
}

// -----------------------------------------------------------------------------
// 插件初始化
// -----------------------------------------------------------------------------
bool ESOTracker::initialize(ros::NodeHandle& nh) {
    ros::NodeHandle nh_nmpc(nh, "eso_tracker");
    ROS_INFO("[%s] NMPC 控制器命名空间: %s", getName().c_str(), nh_nmpc.getNamespace().c_str());

    // -------------------------------------------------------------------------
    // 1. 加载 NMPC 核心参数
    // -------------------------------------------------------------------------
    nh_nmpc.param("nx", nmpc_params_.nx, 6);
    nh_nmpc.param("nu", nmpc_params_.nu, 1);
    nh_nmpc.param("prediction_step", nmpc_params_.N, 35);
    nh_nmpc.param("sparse_control_step", nmpc_params_.Nc, 3);
    nh_nmpc.param("sampling_time", nmpc_params_.dt, 0.05);
    nh_nmpc.param("integration_grade", nmpc_params_.integration_grade, 1.0);
    nh_nmpc.param("eso_disturbance_decay", nmpc_params_.eso_disturbance_decay, 0.85);
    nmpc_params_.eso_disturbance_decay =
        std::max(0.0, std::min(1.0, nmpc_params_.eso_disturbance_decay));
    nh_nmpc.param("eso_disturbance_time_constant", nmpc_params_.eso_disturbance_tau_s, 0.974786);
    if (nmpc_params_.eso_disturbance_tau_s > 0.0) {
        nmpc_params_.eso_disturbance_decay =
            std::exp(-nmpc_params_.dt / nmpc_params_.eso_disturbance_tau_s);
    }

    nh_nmpc.param<bool>("enable_local_log", enable_local_log_, false);
    nh_nmpc.param<std::string>("local_log_directory", local_log_directory_,
                               std::string("/tmp/eso_tracker_logs"));
    nh_nmpc.param<int>("local_log_flush_interval", local_log_flush_interval_, 100);
    local_log_flush_interval_ = std::max(1, local_log_flush_interval_);

    if (nmpc_params_.nx != 6 || nmpc_params_.nu != 1 || nmpc_params_.N <= 0 ||
        nmpc_params_.Nc <= 0 || nmpc_params_.Nc > nmpc_params_.N ||
        !std::isfinite(nmpc_params_.dt) || nmpc_params_.dt <= 0.0) {
        ROS_ERROR("[%s] NMPC基础配置无效: nx=%d, nu=%d, N=%d, Nc=%d, dt=%.6f",
                  getName().c_str(), nmpc_params_.nx, nmpc_params_.nu,
                  nmpc_params_.N, nmpc_params_.Nc, nmpc_params_.dt);
        return false;
    }

    // -------------------------------------------------------------------------
    // 2. 加载车辆物理参数及质量插值表
    // -------------------------------------------------------------------------
    nh_nmpc.param("m", nmpc_params_.m, 10000.0);  // 牵引车模型质量，保持为固定结构参数
    nh_nmpc.param("L", nmpc_params_.L, 4.135);    // 兼容旧配置；加载插值表后由 lf+lr 更新
    nh_nmpc.param("T_lag", nmpc_params_.T_lag, 0.25);

    // 默认值用于兼容 YAML 缺项；正式标定值应直接在 YAML 的六个数组中维护。
    nmpc_params_.mass_interp_points = {10000.0, 16000.0, 33000.0, 49000.0};
    nmpc_params_.Iz_interp_points = {50000.0, 62200.0, 62200.0, 62200.0};
    nmpc_params_.lf_interp_points = {2.0, 1.67, 1.67, 2.0};
    nmpc_params_.lr_interp_points = {2.135, 2.33, 2.33, 2.0};
    nmpc_params_.Cf_interp_points = {270000.0, 270000.0, 270000.0, 270000.0};
    nmpc_params_.Cr_interp_points = {1500000.0, 1500000.0, 1500000.0, 1500000.0};

    nh_nmpc.getParam("mass_interp_points", nmpc_params_.mass_interp_points);
    nh_nmpc.getParam("Iz_interp_points", nmpc_params_.Iz_interp_points);
    nh_nmpc.getParam("lf_interp_points", nmpc_params_.lf_interp_points);
    nh_nmpc.getParam("lr_interp_points", nmpc_params_.lr_interp_points);
    nh_nmpc.getParam("Cf_interp_points", nmpc_params_.Cf_interp_points);
    nh_nmpc.getParam("Cr_interp_points", nmpc_params_.Cr_interp_points);

    if (!validateMassInterpolationTables()) {
        ROS_ERROR("[%s] 质量插值参数表无效，控制器初始化终止", getName().c_str());
        return false;
    }

    // -------------------------------------------------------------------------
    // 3. 加载控制量约束
    // -------------------------------------------------------------------------
    nh_nmpc.param("min_steer", nmpc_params_.delta_min, -0.5);
    nh_nmpc.param("max_steer", nmpc_params_.delta_max, 0.5);
    nh_nmpc.param("max_steer_rate", nmpc_params_.delta_rate_max, 0.35);
    nmpc_params_.delta_rate_max = std::max(1e-3, nmpc_params_.delta_rate_max);

    // -------------------------------------------------------------------------
    // 4. 加载代价函数权重
    // -------------------------------------------------------------------------
    nh_nmpc.param("Q_x", nmpc_params_.Q_x, 1000.0);
    nh_nmpc.param("Q_y", nmpc_params_.Q_y, 5000.0);
    nh_nmpc.param("Q_theta", nmpc_params_.Q_theta, 4000.0);
    nh_nmpc.param("Q_vy", nmpc_params_.Q_vy, 100.0);
    nh_nmpc.param("Q_r", nmpc_params_.Q_r, 800.0);
    nh_nmpc.param("Q_delta", nmpc_params_.Q_delta, 1000.0);
    nh_nmpc.param("R", nmpc_params_.R, 10.0);
    nh_nmpc.param("dR", nmpc_params_.dR, 500.0); //
    nh_nmpc.param<int>("near_dense_control_steps", nmpc_params_.near_dense_control_steps, 5);
    nh_nmpc.param<int>("curvature_smoothing_steps", curvature_smoothing_steps_, 5);
    curvature_smoothing_steps_ = std::max(1, curvature_smoothing_steps_);
    nh_nmpc.param<double>("curvature_smoothing_distance_m", curvature_smoothing_distance_m_, 6.0);
    curvature_smoothing_distance_m_ = std::max(0.5, curvature_smoothing_distance_m_);
    nmpc_params_.near_dense_control_steps = std::max(
        0, std::min(nmpc_params_.near_dense_control_steps, nmpc_params_.Nc - 1));

    nh_nmpc.param<bool>("use_equilibrium_feedforward", use_equilibrium_feedforward_, true);
    nh_nmpc.param<double>("equilibrium_feedforward_gain", equilibrium_feedforward_gain_, 1.0);
    nh_nmpc.param<double>("equilibrium_feedforward_limit", equilibrium_feedforward_limit_, 0.45);
    equilibrium_feedforward_gain_ = std::max(0.0, equilibrium_feedforward_gain_);
    equilibrium_feedforward_limit_ = std::max(0.0, equilibrium_feedforward_limit_);

    nh_nmpc.param<double>("path_projection_heading_weight_m2", path_projection_heading_weight_m2_, 4.0);
    nh_nmpc.param<double>("path_projection_heading_gate_rad", path_projection_heading_gate_rad_, 1.2);
    nh_nmpc.param<double>("path_projection_rear_gate_m", path_projection_rear_gate_m_, 5.0);
    path_projection_heading_weight_m2_ = std::max(0.0, path_projection_heading_weight_m2_);
    path_projection_heading_gate_rad_ = std::max(0.1, path_projection_heading_gate_rad_);
    path_projection_rear_gate_m_ = std::max(0.0, path_projection_rear_gate_m_);

    // ay 零偏补偿：仅用于横坡补偿项的 ay_slope = (ay_raw - ay_bias) - vx*r
    nh_nmpc.param<bool>("use_ay_bias_compensation", use_ay_bias_compensation_, true);   // false=完全关闭 ay 零偏补偿
    nh_nmpc.param("const_ay_bias", const_ay_bias_, 0.0);                               // 静态 ay 零偏/动态初值，单位 m/s^2
    nh_nmpc.param<bool>("use_dynamic_ay_compensation", use_dynamic_ay_compensation_, false);
    nh_nmpc.param<int>("dynamic_ay_error_window_size", dynamic_ay_error_window_size_, 200);
    nh_nmpc.param<double>("dynamic_ay_error_threshold", dynamic_ay_error_threshold_, 0.02);
    nh_nmpc.param<double>("dynamic_ay_bias_learning_rate", dynamic_ay_bias_learning_rate_, 2.0e-5);
    nh_nmpc.param<double>("dynamic_ay_bias_max_step", dynamic_ay_bias_max_step_, 5.0e-5);
    nh_nmpc.param<double>("dynamic_ay_bias_min", dynamic_ay_bias_min_, -1.0);
    nh_nmpc.param<double>("dynamic_ay_bias_max", dynamic_ay_bias_max_, 1.0);
    nh_nmpc.param<double>("dynamic_ay_bias_error_sign", dynamic_ay_bias_error_sign_, -1.0);
    nh_nmpc.param<bool>("dynamic_ay_require_full_window", dynamic_ay_require_full_window_, true);

    nh_nmpc.param("const_steer_bias", const_steer_bias_, 0.0); // 转向偏置补偿，默认0=不补偿
    nh_nmpc.param<bool>("use_slope_compensation", use_slope_compensation_, false);
    nh_nmpc.param<double>("slope_compensation_coeff", slope_compensation_coeff_, 1.0);
    nh_nmpc.param<double>("slope_compensation_filter_tau", slope_compensation_filter_tau_, 1.0);
    nh_nmpc.param<std::string>("slope_estimator_mode", slope_estimator_mode_,
                               std::string("legacy_quasistatic"));
    nh_nmpc.param<bool>("slope_dynamic_gate_enabled", slope_dynamic_gate_enabled_, true);
    nh_nmpc.param<double>("slope_gate_max_yaw_accel", slope_gate_max_yaw_accel_, 0.15);
    nh_nmpc.param<double>("slope_gate_max_steer_rate", slope_gate_max_steer_rate_, 0.08);
    nh_nmpc.param<double>("slope_compensation_limit", slope_compensation_limit_, 0.8);

    nh_nmpc.param<bool>("ukf_use_slope_disturbance", ukf_use_slope_disturbance_, true);
    nh_nmpc.param<double>("ukf_q_vy", ukf_q_vy_, 0.01);
    nh_nmpc.param<double>("ukf_q_r", ukf_q_r_, 0.001);
    nh_nmpc.param<double>("ukf_r_ay", ukf_r_ay_, 0.5);
    nh_nmpc.param<double>("ukf_r_r", ukf_r_r_, 0.1);
    nh_nmpc.param<double>("ukf_ay_innovation_limit", ukf_ay_innovation_limit_, 0.8);
    nh_nmpc.param<double>("ukf_vy_abs_max", ukf_vy_abs_max_, 3.0);
    nh_nmpc.param<double>("observer_dynamic_min_speed_mps", observer_dynamic_min_speed_mps_, 4.0);

    slope_compensation_filter_tau_ = std::max(0.0, slope_compensation_filter_tau_);
    if (slope_estimator_mode_ != "legacy_quasistatic" &&
        slope_estimator_mode_ != "tire_force_residual") {
        ROS_WARN("[%s] slope_estimator_mode=%s 无效，回退为 legacy_quasistatic",
                 getName().c_str(), slope_estimator_mode_.c_str());
        slope_estimator_mode_ = "legacy_quasistatic";
    }
    if (slope_estimator_mode_ == "tire_force_residual" && slope_compensation_coeff_ < 0.0) {
        ROS_WARN("[%s] tire_force_residual 理论上应以正号进入 vy_dot，当前 coeff=%.3f；不自动改参，请仅在单变量验证时使用",
                 getName().c_str(), slope_compensation_coeff_);
    }
    slope_gate_max_yaw_accel_ = std::max(0.0, slope_gate_max_yaw_accel_);
    slope_gate_max_steer_rate_ = std::max(0.0, slope_gate_max_steer_rate_);
    slope_compensation_limit_ = std::max(0.0, slope_compensation_limit_);
    ukf_q_vy_ = std::max(1e-9, ukf_q_vy_);
    ukf_q_r_ = std::max(1e-9, ukf_q_r_);
    ukf_r_ay_ = std::max(1e-9, ukf_r_ay_);
    ukf_r_r_ = std::max(1e-9, ukf_r_r_);
    ukf_ay_innovation_limit_ = std::max(0.0, ukf_ay_innovation_limit_);
    ukf_vy_abs_max_ = std::max(0.1, ukf_vy_abs_max_);
    observer_dynamic_min_speed_mps_ = std::max(1.0, observer_dynamic_min_speed_mps_);
    dynamic_ay_error_window_size_ = std::max(1, dynamic_ay_error_window_size_);
    dynamic_ay_error_threshold_ = std::max(0.0, dynamic_ay_error_threshold_);
    dynamic_ay_bias_learning_rate_ = std::max(0.0, dynamic_ay_bias_learning_rate_);
    dynamic_ay_bias_max_step_ = std::max(0.0, dynamic_ay_bias_max_step_);
    if (dynamic_ay_bias_min_ > dynamic_ay_bias_max_) {
        std::swap(dynamic_ay_bias_min_, dynamic_ay_bias_max_);
    }
    ay_bias_estimate_ = std::max(dynamic_ay_bias_min_, std::min(const_ay_bias_, dynamic_ay_bias_max_));
    effective_ay_bias_ = use_ay_bias_compensation_ ? ay_bias_estimate_ : 0.0;
    lateral_error_history_.clear();

    // 整车质量只作为参数插值的调度输入
    nh_nmpc.param("m_total", nmpc_params_.m_total, 10000.0); // 无有效 EBS 质量时的整车质量回退值，单位 kg
    nh_nmpc.param<bool>("auto_update_total_weight", auto_update_total_weight_, false); // 是否根据话题信息自动更新整车重量

    received_mass_ = nmpc_params_.m_total;
    updateMassDependentParameters(received_mass_);

    // -------------------------------------------------------------------------
    // 加载 Supervisor 配置 (模式切换与纯跟踪)
    // -------------------------------------------------------------------------
    ros::NodeHandle nh_super(nh, "supervisor_config");
    nh_super.param("startup_time", supervisor_params_.startup_time, 5.0);
    nh_super.param("blend_speed_low", supervisor_params_.blend_speed_low, 4.1667);
    nh_super.param("blend_speed_high", supervisor_params_.blend_speed_high, 5.0);
    nh_super.param("min_lookahead_distance", min_lookahead_distance_, 6.0);
    nh_super.param("lookahead_speed_coeff", lookahead_speed_coeff_, 0.7);
    nh_super.param("lookahead_curvature_coeff", lookahead_curvature_coeff_, 0.0);
    nh_super.param("control_time", control_time_, 0.05);
    nh_super.param("control_delay_sec", control_delay_sec_, 0.0);
    nh_super.param("output_lpf_tau", output_lpf_tau_, 0.0);   // 输出低通时间常数(s)，默认0=关闭
    nh_super.param("degrade_failure_times", degrade_failure_times_, 3); // NMPC连续失败次数阈值，超过该值则降级为纯跟踪模式
    nh_super.param("require_overtake_times", require_overtake_times_, 10); // 连续要求超车次数阈值，超过该值则提示要求人工接管

    min_lookahead_distance_ = std::max(0.1, min_lookahead_distance_);
    lookahead_speed_coeff_ = std::max(0.0, lookahead_speed_coeff_);
    lookahead_curvature_coeff_ = std::max(0.0, lookahead_curvature_coeff_);

    // -------------------------------------------------------------------------
    // 后处理与打印
    // -------------------------------------------------------------------------
    // 更新 Eigen Q 矩阵
    nmpc_params_.updateQMatrix();

    // 简单的参数确认打印 (替代不存在的 logParamLoad)
    ROS_INFO("[%s] 参数加载完毕: tractor_m=%.0f, received_mass=%.0f, Iz=%.1f, lf=%.3f, lr=%.3f, Cf=%.1f, Cr=%.1f, N=%d, dR=%.1f, Lookahead=%.1f",
             getName().c_str(), nmpc_params_.m, received_mass_, nmpc_params_.Iz,
             nmpc_params_.lf, nmpc_params_.lr, nmpc_params_.Cf, nmpc_params_.Cr,
             nmpc_params_.N, nmpc_params_.dR, min_lookahead_distance_);
    ROS_INFO("[%s] ay零偏补偿: enable=%d, dynamic=%d, const=%.4f, init=%.4f, window=%d, threshold=%.4f, lr=%.8f, max_step=%.8f, sign=%.1f",
             getName().c_str(), use_ay_bias_compensation_, use_dynamic_ay_compensation_, const_ay_bias_,
             ay_bias_estimate_, dynamic_ay_error_window_size_, dynamic_ay_error_threshold_,
             dynamic_ay_bias_learning_rate_, dynamic_ay_bias_max_step_, dynamic_ay_bias_error_sign_);
    ROS_INFO("[%s] 共用轴距 L=%.3f m | PP预瞄: min=%.2f m, speed_coeff=%.3f s, curvature_coeff=%.3f m^2",
             getName().c_str(), nmpc_params_.L, min_lookahead_distance_,
             lookahead_speed_coeff_, lookahead_curvature_coeff_);
    ROS_INFO("[%s] V6: ESO tau=%.4fs, effective_decay=%.6f | curvature_distance=%.2fm | feedforward=%d gain=%.2f | Nc=%d near_dense=%d | steer_rate<=%.3frad/s | slope_mode=%s, gate=%d, limit=%.3f | UKF slope_consistency=%d",
             getName().c_str(), nmpc_params_.eso_disturbance_tau_s,
             nmpc_params_.eso_disturbance_decay, curvature_smoothing_distance_m_,
             use_equilibrium_feedforward_, equilibrium_feedforward_gain_,
             nmpc_params_.Nc, nmpc_params_.near_dense_control_steps, nmpc_params_.delta_rate_max,
             slope_estimator_mode_.c_str(), slope_dynamic_gate_enabled_,
             slope_compensation_limit_, ukf_use_slope_disturbance_);

    start_time_ = ros::Time::now();

    // 构建 CasADi 求解器
    buildNMPSolver();
    // 初始化发布器
    est_pub_ = nh.advertise<race_msgs::ESOEstimation>("/race/eso_estimation_states", 1);
    initializeLocalLog();
    ROS_INFO("[%s] 控制器初始化完成", getName().c_str());
    return true;
}

// -----------------------------------------------------------------------------
// 核心控制循环
// -----------------------------------------------------------------------------
void ESOTracker::computeControl(
    const race_msgs::VehicleStatusConstPtr& vehicle_status,
    const race_msgs::PathConstPtr& path,
    race_msgs::Control* control_msg,
    const double dt,
    const race_msgs::Flag::ConstPtr& flag) {

    auto control_start_time = std::chrono::high_resolution_clock::now();

    // 1. 提取基础信息
    if (!vehicle_status || !path || !control_msg) {
        ROS_ERROR("[%s] 收到空指针消息", getName().c_str());
        return;
    }
    if (path->points.empty()) return;

    double curr_vx_raw = vehicle_status->vel.linear.x;
    double curr_vx = std::max(vehicle_status->vel.linear.x, 1.0); // 防零除
    double curr_vy_status = vehicle_status->vel.linear.y;
    double curr_ax = vehicle_status->acc.linear.x;
    double curr_x = vehicle_status->pose.position.x;
    double curr_y = vehicle_status->pose.position.y;
    double curr_theta = vehicle_status->euler.yaw;
    double curr_ay = vehicle_status->acc.linear.y;
    double curr_r = vehicle_status->vel.angular.z;
    const double curr_delta_raw = vehicle_status->lateral.steering_angle;
    double curr_delta = curr_delta_raw;
    double curr_lateral_tracking_error = vehicle_status->tracking.lateral_tracking_error;

    const bool measurement_is_new = isNewVehicleMeasurement(
        curr_x, curr_y, curr_theta, curr_vx_raw, curr_vy_status,
        curr_r, curr_delta_raw, curr_ay);
    last_measurement_is_new_ = measurement_is_new;

    curr_delta = std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, curr_delta));

    if (auto_update_total_weight_) {
        const double mass_from_ebs = vehicle_status->mass;
        if (std::isfinite(mass_from_ebs) && mass_from_ebs > 0.0) {
            received_mass_ = mass_from_ebs;
        } else {
            ROS_WARN("[%s] 收到无效整车质量 %.3f kg，沿用上一有效值 %.1f kg",
                              getName().c_str(), mass_from_ebs, received_mass_);
        }
    } else {
        received_mass_ = nmpc_params_.m_total;
    }

    // 质量是 Iz、lf、Cf、Cr 的唯一调度变量；插值区间外保持首/末端值。
    updateMassDependentParameters(received_mass_);

    // ==========================================================
    // 无论低速还是高速，UKF/ESO 都更新
    // ==========================================================

    static ros::Time last_control_time = ros::Time(0);
    ros::Time current_time = ros::Time::now();

    if (last_control_time.toSec() != 0.0 && (current_time - last_control_time).toSec() > 1) {     //5-28原本是0.02，现在改成1
        ROS_WARN("[%s] 检测到控制重连，清空观测器记忆！", getName().c_str());
        solver_.has_prev_sol = false;
        solver_.sol_prev = nullptr;
        current_cmd_ = curr_delta;
        ukf_x_est_ = Vector2d::Zero();
        ukf_x_est_(1) = curr_r;
        ukf_P_est_ = (Matrix2d() << 1.0, 0.0, 0.0, 0.1).finished();
        eso_x1_ = curr_r;
        eso_x2_ = 0.0;
        blend_alpha_ = 0.0;
        start_time_ = current_time;
        final_cmd_filt_ = curr_delta;
        final_cmd_filt_init_ = false;
        model_r_comp_ = 0.0;
        model_comp_initialized_ = false;
        ay_slope_compensation_ = 0.0;
        ay_slope_compensation_initialized_ = false;
        slope_prev_valid_ = false;
        slope_gate_active_ = false;
        lateral_error_history_.clear();
        ay_bias_estimate_ = std::max(dynamic_ay_bias_min_, std::min(const_ay_bias_, dynamic_ay_bias_max_));
        effective_ay_bias_ = use_ay_bias_compensation_ ? ay_bias_estimate_ : 0.0;
        diagnostic_prev_valid_ = false;
        diagnostic_prev_nearest_idx_ = -1;
        measurement_fingerprint_valid_ = false;
        resetNmpcPredictionDiagnostics();
    }
    last_control_time = current_time;

    const double obs_dt = std::max(0.01, std::min(dt, 0.05));//0.01只设置了下限

    // 1) 先更新准静态横坡/外部侧向加速度。旧公式仅在 vy_dot 较小时成立，
    //    因此默认保留实车已验证的符号，但在转向/横摆快变时冻结估计。
    double ay_slope_compensation_raw = 0.0;
    double slope_yaw_accel = 0.0;
    double slope_steer_rate = 0.0;

    if (use_slope_compensation_) {
        if (use_ay_bias_compensation_) {
            if (use_dynamic_ay_compensation_) {
                updateDynamicAyBias(curr_lateral_tracking_error);
            } else {
                lateral_error_history_.clear();
                ay_bias_estimate_ = std::max(dynamic_ay_bias_min_,
                    std::min(const_ay_bias_, dynamic_ay_bias_max_));
                effective_ay_bias_ = ay_bias_estimate_;
            }
        } else {
            lateral_error_history_.clear();
            ay_bias_estimate_ = 0.0;
            effective_ay_bias_ = 0.0;
        }

        const double ay_corrected_for_slope = curr_ay - effective_ay_bias_;
        if (slope_estimator_mode_ == "tire_force_residual") {
            const double vx_for_slope = std::max(std::abs(curr_vx_raw), 2.0);
            const double vy_prior = ukf_x_est_(0);
            const double alpha_f_prior = curr_delta -
                std::atan2(vy_prior + nmpc_params_.lf * curr_r, vx_for_slope);
            const double alpha_r_prior = -
                std::atan2(vy_prior - nmpc_params_.lr * curr_r, vx_for_slope);
            const double ay_tire_prior =
                (nmpc_params_.Cf * alpha_f_prior * std::cos(curr_delta) +
                 nmpc_params_.Cr * alpha_r_prior) / nmpc_params_.m;
            ay_slope_compensation_raw = ay_corrected_for_slope - ay_tire_prior;
        } else {
            // 兼容原实车符号和校准：准稳态时 ay-vx*r 为低频横坡/侧向扰动代理量。
            ay_slope_compensation_raw = ay_corrected_for_slope - curr_r * curr_vx;
        }

        if (slope_compensation_limit_ > 0.0) {
            ay_slope_compensation_raw = std::max(-slope_compensation_limit_,
                std::min(slope_compensation_limit_, ay_slope_compensation_raw));
        }

        if (slope_prev_valid_ && obs_dt > 1e-6) {
            slope_yaw_accel = (curr_r - slope_prev_r_) / obs_dt;
            slope_steer_rate = (curr_delta - slope_prev_delta_) / obs_dt;
        }
        slope_gate_active_ = slope_dynamic_gate_enabled_ && slope_prev_valid_ &&
            (std::abs(slope_yaw_accel) > slope_gate_max_yaw_accel_ ||
             std::abs(slope_steer_rate) > slope_gate_max_steer_rate_);

        if (!ay_slope_compensation_initialized_) {
            if (!slope_gate_active_) {
                ay_slope_compensation_ = ay_slope_compensation_raw;
                ay_slope_compensation_initialized_ = true;
            }
        } else if (!slope_gate_active_) {
            const double alpha = (slope_compensation_filter_tau_ <= 0.0)
                ? 1.0 : obs_dt / (slope_compensation_filter_tau_ + obs_dt);
            ay_slope_compensation_ +=
                alpha * (ay_slope_compensation_raw - ay_slope_compensation_);
        }
    } else {
        ay_slope_compensation_ = 0.0;
        ay_slope_compensation_initialized_ = false;
        effective_ay_bias_ = 0.0;
        lateral_error_history_.clear();
        slope_gate_active_ = false;
    }
    slope_prev_valid_ = true;
    slope_prev_r_ = curr_r;
    slope_prev_delta_ = curr_delta;

    const double slope_model_input_control =
        ay_slope_compensation_ * slope_compensation_coeff_;

    // 2) UKF 使用去零偏 ay，并在过程/测量方程中使用同一横向扰动。
    //    这样已注入 NMPC 的横坡不再被 UKF 重复吸收为 vy。
    const double obs_vx = std::max(std::abs(curr_vx_raw), 1.0);
    const double ay_for_ukf = curr_ay - effective_ay_bias_;
    const double disturbance_for_ukf =
        ukf_use_slope_disturbance_ ? slope_model_input_control : 0.0;
    last_observer_low_speed_reset_ = std::abs(curr_vx_raw) < observer_dynamic_min_speed_mps_;
    if (last_observer_low_speed_reset_) {
        // 动态单轨模型在低速区不可观且含 1/vx 项。低速期间用实测/零侧速锚定，
        // 避免长期更新把 vy 推到限幅，随后在 18~20 km/h 交给 NMPC 时产生冷启动冲击。
        const double vy_seed = (std::isfinite(curr_vy_status) &&
                                std::abs(curr_vy_status) <= ukf_vy_abs_max_)
            ? curr_vy_status : 0.0;
        ukf_x_est_ << vy_seed, curr_r;
        ukf_P_est_ = (Matrix2d() << 0.25, 0.0, 0.0, 0.02).finished();
        ukf_ay_innovation_raw_ = 0.0;
        ukf_ay_innovation_used_ = 0.0;
        eso_x1_ = curr_r;
        eso_x2_ = 0.0;
    } else {
        ukfEstimateVy(obs_vx, curr_delta, ay_for_ukf, curr_r,
                      disturbance_for_ukf, obs_dt, measurement_is_new);
        esoCompute(curr_r, curr_delta, obs_dt, measurement_is_new);
    }
    const double vy_est = ukf_x_est_(0);

    // ==========================================================
    // 启动阶段：使用 supervisor_params_
    // ==========================================================

    double time_elapsed = (current_time - start_time_).toSec();

    // ==========================================================
    // 模式平滑过渡权重计算
    // ==========================================================
    if (time_elapsed < supervisor_params_.startup_time) {
        // 启动前N秒：强制纯跟踪
        blend_alpha_ = 0.0;
        ROS_INFO("[STARTUP] 预热中: %.1f / %.1f s | 纯跟踪锁定", time_elapsed, supervisor_params_.startup_time);
    } else {
        // 恢复原有车速切换逻辑
        if (curr_vx_raw <= supervisor_params_.blend_speed_low) {
            blend_alpha_ = 0.0;
        } else if (curr_vx_raw >= supervisor_params_.blend_speed_high) {
            blend_alpha_ = 1.0;
        } else {
            blend_alpha_ = (curr_vx_raw - supervisor_params_.blend_speed_low) / (supervisor_params_.blend_speed_high - supervisor_params_.blend_speed_low);
        }
    }

    // ==========================================================
    // NMPC都后台预计算，保持热启动
    // ==========================================================
    // 扰动纯化（始终计算)
    if (!model_comp_initialized_) {
    model_r_comp_ = curr_r;
    model_comp_initialized_ = true;
    }
    double vx_safe_external = std::max(curr_vx, 1.0);
    double alpha_f_curr = curr_delta - atan2((vy_est + nmpc_params_.lf * curr_r), vx_safe_external);
    double alpha_r_curr = -atan2((vy_est - nmpc_params_.lr * curr_r), vx_safe_external);
    double Fyf_curr = nmpc_params_.Cf * alpha_f_curr;
    double Fyr_curr = nmpc_params_.Cr * alpha_r_curr;
    double r_dot_nominal = (nmpc_params_.lf * Fyf_curr * cos(curr_delta) - nmpc_params_.lr * Fyr_curr) / nmpc_params_.Iz;
    double b_eso = (nmpc_params_.Cf * nmpc_params_.lf) / nmpc_params_.Iz;
    double r_dot_actual = b_eso * curr_delta + eso_x2_;
    double d_pure_trailer = r_dot_actual - r_dot_nominal;
    model_r_comp_ += r_dot_nominal * obs_dt;
    Model_r1_ = model_r_comp_;

    // 路径处理（始终计算）
    std::vector<double> current_pose = {curr_x, curr_y, curr_theta, curr_vx};
    casadi::DM waypoints_dm = process_race_path(*path, current_pose);
    double kappa = static_cast<double>(waypoints_dm(3,1));
    last_delta_ff_ = static_cast<double>(waypoints_dm(4, 1));
    double theta = static_cast<double>(waypoints_dm(2,0));
    double r_ref = curr_vx * kappa;
    double vy_model = curr_vx * sin(theta) + vy_est * cos(theta);

    // NMPC 现在工作在“自车体坐标系”：原点为自车当前位置，x 轴沿自车当前航向。
    // 因此初始 x,y,theta 均为 0；vy/r/delta 仍为实际物理量。
    std::vector<double> nmpc_state = {0.0, 0.0, 0.0, vy_est, curr_r, curr_delta};
    std::vector<double> control_output(1);
    ROS_WARN("横坡补偿: mode=%s, raw_ay=%.4f, ay_bias=%.4f, slope_raw=%.4f, slope_filt=%.4f, model_input=%.4f, gate=%d, yaw_acc=%.4f, steer_rate=%.4f",
             slope_estimator_mode_.c_str(), curr_ay, effective_ay_bias_,
             ay_slope_compensation_raw, ay_slope_compensation_,
             slope_model_input_control, slope_gate_active_,
             slope_yaw_accel, slope_steer_rate);

    // NMPC参数绑定（始终更新）
    std::vector<double> dyn_params = {nmpc_params_.m, nmpc_params_.Iz, nmpc_params_.lf,
                                      nmpc_params_.lr, nmpc_params_.Cf, nmpc_params_.Cr};
    solver_.opti.set_value(solver_.P_vx, curr_vx);
    solver_.opti.set_value(solver_.P_ay_slope_comp, slope_model_input_control);
    solver_.opti.set_value(solver_.P_h_hat, d_pure_trailer);//d_pure_trailer
    solver_.opti.set_value(solver_.P_dyn_params, dyn_params);

    // 无论什么模式，都调用NMPC求解，保持热启动状态
    auto nmpc_start_time = std::chrono::high_resolution_clock::now();
    bool nmpc_solve_success = solveNMPC(nmpc_state, waypoints_dm, control_output);
    auto nmpc_end_time = std::chrono::high_resolution_clock::now();
    iter_time_ = std::chrono::duration<double, std::milli>(nmpc_end_time - nmpc_start_time).count();
    double nmpc_raw_cmd = std::numeric_limits<double>::quiet_NaN();

    if (nmpc_solve_success) {
        nmpc_raw_cmd = control_output[0];
        nmpc_safe_cmd_ = control_output[0];
        nmpc_safe_cmd_ += const_steer_bias_;
        mpc_failure_flag_ = false;
        mpc_failure_count_ = 0;
    } else {
        // 求解失败时，暂时给上一帧的有效输出，但是实际用的是纯跟踪输出
        nmpc_safe_cmd_ = current_cmd_;
        mpc_failure_flag_ = true;
        mpc_failure_count_++;
        ROS_ERROR("[%s] NMPC求解失败，使用上一帧输出 | 迭代时间: %.2f ms | 失败次数: %d", getName().c_str(), iter_time_, mpc_failure_count_);
    }
    // 所有 NMPC 后处理完成后统一限幅保护。
    nmpc_safe_cmd_ = std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, nmpc_safe_cmd_));

    // ==========================================================
    // 纯跟踪逻辑
    // ==========================================================
    double pp_safe_cmd_ = 0.0;
    // PP 与 NMPC 直接共用同一个固定轴距参数，避免 lf/lr 调度后出现口径歧义。
    const double L = nmpc_params_.L;

    // 先按原速度公式确定预瞄范围，再直接从该范围内的 PP 路径计算最大绝对曲率。
    // 这样曲率预瞄范围与 PP 自己的目标点范围一致，不受 NMPC 预测时域长度限制。
    const double speed_lookahead =
        min_lookahead_distance_ + lookahead_speed_coeff_ * curr_vx;
    const int nearest_idx = find_nearest_path_point(curr_x, curr_y, curr_theta, *path);
    double preview_abs_curvature = 0.0;
    double preview_distance = 0.0;
    for (int i = nearest_idx + 1; i < static_cast<int>(path->points.size()); ++i) {
        const auto& prev_point = path->points[i - 1];
        const auto& curr_point = path->points[i];
        const double dx_segment = curr_point.pose.position.x - prev_point.pose.position.x;
        const double dy_segment = curr_point.pose.position.y - prev_point.pose.position.y;
        const double ds = std::hypot(dx_segment, dy_segment);

        if (ds > 1e-4) {
            const double yaw_prev = quaternion_to_yaw(prev_point.pose.orientation);
            const double yaw_curr = quaternion_to_yaw(curr_point.pose.orientation);
            const double kappa_segment = normalizeAngle(yaw_curr - yaw_prev) / ds;
            if (std::isfinite(kappa_segment)) {
                preview_abs_curvature =
                    std::max(preview_abs_curvature, std::abs(kappa_segment));
            }
            preview_distance += ds;
        }

        if (preview_distance >= speed_lookahead) {
            break;
        }
    }

    // lookahead_curvature_coeff_=0 时严格退化为原始速度预瞄公式。
    const double lookahead_dist = std::max(
        min_lookahead_distance_,
        speed_lookahead - lookahead_curvature_coeff_ * preview_abs_curvature);

    // 找目标点
    int target_idx = 0;
    bool found = false;

    for (int i = nearest_idx; i < static_cast<int>(path->points.size()); ++i) {
        double dx = path->points[i].pose.position.x - curr_x;
        double dy = path->points[i].pose.position.y - curr_y;
        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist >= lookahead_dist) {
            target_idx = i;
            found = true;
            break;
        }
    }
    if (!found) target_idx = path->points.size() - 1;

    // 提取目标点并转换坐标
    double tx = path->points[target_idx].pose.position.x;
    double ty = path->points[target_idx].pose.position.y;
    double dx = tx - curr_x;
    double dy = ty - curr_y;
    double local_x = cos(curr_theta) * dx + sin(curr_theta) * dy;
    double local_y = -sin(curr_theta) * dx + cos(curr_theta) * dy;

    // 纯跟踪公式
    double ld = std::max(lookahead_dist, sqrt(local_x*local_x + local_y*local_y));
    double delta_pp_raw = atan2(2.0 * L * local_y, ld * ld);
    delta_pp_raw = std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, delta_pp_raw));

     // ---- 时延队列处理 ----
    pp_cmd_queue_.push_back(delta_pp_raw);
    // 计算队列的最大容量 (时延秒数 / 控制周期)
    size_t max_queue_size = static_cast<size_t>(std::max(1.0, control_delay_sec_ / control_time_));

    if (pp_cmd_queue_.size() > max_queue_size) {
        // 取出队列最前端（即历史时刻）的控制量作为当前输出
        pp_safe_cmd_ = pp_cmd_queue_.front();
        pp_cmd_queue_.pop_front();
    } else {
        // 初始阶段队列未填满时，可以直接输出当前值，或者输出 0
        pp_safe_cmd_ = delta_pp_raw;
    }
    // 检测NMPC是否求解失败
    if (mpc_failure_flag_) {

        if (mpc_failure_count_ >= degrade_failure_times_) {
            blend_alpha_ = 0.0; // 强制切换到纯跟踪模式
            ROS_ERROR("[%s] NMPC连续失败 %d 次，强制切换到纯跟踪模式 | PP: %.3f rad", getName().c_str(), mpc_failure_count_, pp_safe_cmd_);
        }else{
            ROS_ERROR("[%s] NMPC连续失败 %d 次，仍尝试使用上一次输出 | NMPC: %.3f rad", getName().c_str(), mpc_failure_count_, nmpc_safe_cmd_);
        }

        if (mpc_failure_count_ >= require_overtake_times_) {
            require_over_take_flag_ = true;
            ROS_WARN("[%s] 连续要求超车 %d 次，提示人工接管", getName().c_str(), mpc_failure_count_);
        }

    }else {
            require_over_take_flag_ = false;
    }

    // 打印调试信息
    if (blend_alpha_ < 0.01) {
        using_pure_pursuit_flag_ = true;
        using_mixed_mode_flag_ = false;
        ROS_INFO("[PP] 纯跟踪模式 | Local: (%.2f, %.2f) | Lookahead: %.2f m | Preview |kappa|max: %.5f 1/m | Delta: %.3f rad",
                 local_x, local_y, lookahead_dist, preview_abs_curvature, pp_safe_cmd_);
    } else if (blend_alpha_ > 0.99) {
        using_pure_pursuit_flag_ = false;
        using_mixed_mode_flag_ = false;
        ROS_INFO("[%s] 高速模式 (%.1f km/h)", getName().c_str(), curr_vx_raw * 3.6);
    } else {
        using_pure_pursuit_flag_ = false;
        using_mixed_mode_flag_ = true;
        ROS_INFO("[BLEND] 过渡模式 | 车速: %.1f km/h | 权重: %.2f | PP: %.3f | NMPC: %.3f",
                            curr_vx_raw * 3.6, blend_alpha_, pp_safe_cmd_, nmpc_safe_cmd_);
    }

    // ==========================================================
    // 加权融合输出，最终平滑控制
    // ==========================================================
    // 加权融合两个算法的输出
    double final_cmd = blend_alpha_ * nmpc_safe_cmd_ + (1.0 - blend_alpha_) * pp_safe_cmd_;

    // 输出端一阶低通滤波：滤掉驾驶员能感知的高频抖动，进一步提升方向盘转动质量。
    // tau<=0 时直接透传，不改变原行为。注意 LPF 引入的相位滞后由 tau 控制，
    // 取较小值(如 0.08~0.15s)可在几乎不损失跟踪的前提下显著“顺滑”手感。
    if (output_lpf_tau_ > 1e-6) {
        if (!final_cmd_filt_init_) {
            final_cmd_filt_ = final_cmd;
            final_cmd_filt_init_ = true;
        }
        double a = obs_dt / (output_lpf_tau_ + obs_dt);
        final_cmd_filt_ = (1.0 - a) * final_cmd_filt_ + a * final_cmd;
        final_cmd = final_cmd_filt_;
    }

    // 最后再做绝对转角限幅
    final_cmd = std::max(nmpc_params_.delta_min,
             std::min(nmpc_params_.delta_max, final_cmd));

    // 更新current_cmd_，保持状态连续
    current_cmd_ = final_cmd;

    // 填装消息输出
    control_msg->lateral.steering_angle = final_cmd;
    control_msg->steering_mode = race_msgs::Control::FRONT_STEERING_MODE;
    control_msg->control_mode = race_msgs::Control::DES_ACCEL_ONLY;

    auto control_end_time = std::chrono::high_resolution_clock::now();
    total_control_time_ = std::chrono::duration<double, std::milli>(
    control_end_time - control_start_time
    ).count();

    if (enable_local_log_ && local_log_stream_.is_open()) {
        const double diagnostic_nan = std::numeric_limits<double>::quiet_NaN();
        const bool prev_valid = diagnostic_prev_valid_;

        // 1) 转向执行器层：用上一周期实际转角和最终下发指令做一阶模型一步预测。
        double steer_meas_rate = diagnostic_nan;
        double steer_cmd_rate = diagnostic_nan;
        double steer_model_1step = diagnostic_nan;
        double steer_model_residual = diagnostic_nan;
        if (prev_valid && obs_dt > 1e-6) {
            steer_meas_rate = (curr_delta - diagnostic_prev_delta_) / obs_dt;
            steer_cmd_rate = (final_cmd - diagnostic_prev_final_cmd_) / obs_dt;
            steer_model_1step = diagnostic_prev_delta_ + obs_dt *
                (diagnostic_prev_final_cmd_ - diagnostic_prev_delta_) /
                std::max(nmpc_params_.T_lag, 1e-6);
            steer_model_residual = curr_delta - steer_model_1step;
        }
        const bool nmpc_clamped = nmpc_solve_success && std::isfinite(nmpc_raw_cmd) &&
            std::abs(nmpc_safe_cmd_ - (nmpc_raw_cmd + const_steer_bias_)) > 1e-9;
        const bool final_saturated =
            final_cmd <= nmpc_params_.delta_min + 1e-9 ||
            final_cmd >= nmpc_params_.delta_max - 1e-9;

        // 2) 横摆层：明确区分 ESO 总扰动、纯化后注入量及注入后的模型残差。
        double yaw_rate_dot_raw = diagnostic_nan;
        if (prev_valid && obs_dt > 1e-6) {
            yaw_rate_dot_raw = (curr_r - diagnostic_prev_r_) / obs_dt;
        }
        const double eso_injected_k0 = d_pure_trailer;
        const double eso_injected_k1 =
            d_pure_trailer * nmpc_params_.eso_disturbance_decay;
        const double yaw_rate_dot_with_eso = r_dot_nominal + eso_injected_k0;
        const double yaw_residual_nominal = std::isfinite(yaw_rate_dot_raw)
            ? yaw_rate_dot_raw - r_dot_nominal : diagnostic_nan;
        const double yaw_residual_with_eso = std::isfinite(yaw_rate_dot_raw)
            ? yaw_rate_dot_raw - yaw_rate_dot_with_eso : diagnostic_nan;
        const double yaw_model_1step_nominal = curr_r + obs_dt * r_dot_nominal;
        const double yaw_model_1step_eso = curr_r + obs_dt * yaw_rate_dot_with_eso;

        // 3) 横向动力学层：侧偏角/轮胎力、ay 合力残差和 vy 状态残差。
        const double slope_model_input =
            ay_slope_compensation_ * slope_compensation_coeff_;
        const double ay_corrected = curr_ay - effective_ay_bias_;
        const double ay_model_no_slope =
            (Fyf_curr * std::cos(curr_delta) + Fyr_curr) / nmpc_params_.m;
        const double ay_model_with_slope = ay_model_no_slope + slope_model_input;
        const double ay_residual_no_slope = ay_corrected - ay_model_no_slope;
        const double ay_residual_with_slope = ay_corrected - ay_model_with_slope;
        double vx_dot_raw = diagnostic_nan;
        double vy_status_dot_raw = diagnostic_nan;
        double vy_dot_raw = diagnostic_nan;
        if (prev_valid && obs_dt > 1e-6) {
            vx_dot_raw = (curr_vx_raw - diagnostic_prev_vx_) / obs_dt;
            vy_status_dot_raw = (curr_vy_status - diagnostic_prev_vy_status_) / obs_dt;
            vy_dot_raw = (vy_est - diagnostic_prev_vy_) / obs_dt;
        }
        const double vy_dot_model =
            ay_model_no_slope - curr_vx * curr_r + slope_model_input;
        const double vy_dot_residual = std::isfinite(vy_dot_raw)
            ? vy_dot_raw - vy_dot_model : diagnostic_nan;
        const double beta_status =
            std::atan2(curr_vy_status, std::max(std::abs(curr_vx_raw), 1.0));
        const double beta_est =
            std::atan2(vy_est, std::max(std::abs(curr_vx_raw), 1.0));

        // 4) 路径/定位与运动学层：独立几何误差用于核对消息中的 tracking error。
        const auto& nearest_path_point = path->points[nearest_idx];
        const double nearest_path_x = nearest_path_point.pose.position.x;
        const double nearest_path_y = nearest_path_point.pose.position.y;
        const double nearest_path_yaw = quaternion_to_yaw(nearest_path_point.pose.orientation);
        const double nearest_dx = curr_x - nearest_path_x;
        const double nearest_dy = curr_y - nearest_path_y;
        const double nearest_distance = std::hypot(nearest_dx, nearest_dy);
        const double geometric_lateral_error =
            -std::sin(nearest_path_yaw) * nearest_dx +
             std::cos(nearest_path_yaw) * nearest_dy;
        const double heading_error = normalizeAngle(curr_theta - nearest_path_yaw);
        const int nearest_idx_jump = diagnostic_prev_nearest_idx_ >= 0
            ? nearest_idx - diagnostic_prev_nearest_idx_ : 0;

        const int k1_index = std::min(1, nmpc_params_.N);
        const int k5_index = std::min(5, nmpc_params_.N);
        const int kN_index = nmpc_params_.N;
        const double ref_x_k0 = static_cast<double>(waypoints_dm(0, 0));
        const double ref_y_k0 = static_cast<double>(waypoints_dm(1, 0));
        const double ref_theta_k0 = static_cast<double>(waypoints_dm(2, 0));
        const double ref_kappa_k0 = static_cast<double>(waypoints_dm(3, 0));
        const double ref_x_k1 = static_cast<double>(waypoints_dm(0, k1_index));
        const double ref_y_k1 = static_cast<double>(waypoints_dm(1, k1_index));
        const double ref_theta_k1 = static_cast<double>(waypoints_dm(2, k1_index));
        const double ref_kappa_k1 = static_cast<double>(waypoints_dm(3, k1_index));
        const double ref_y_k5 = static_cast<double>(waypoints_dm(1, k5_index));
        const double ref_theta_k5 = static_cast<double>(waypoints_dm(2, k5_index));
        const double ref_kappa_k5 = static_cast<double>(waypoints_dm(3, k5_index));
        const double ref_y_kN = static_cast<double>(waypoints_dm(1, kN_index));
        const double ref_theta_kN = static_cast<double>(waypoints_dm(2, kN_index));
        const double ref_kappa_kN = static_cast<double>(waypoints_dm(3, kN_index));

        double frenet_denominator = 1.0 - ref_kappa_k0 * geometric_lateral_error;
        if (std::abs(frenet_denominator) < 0.1) {
            frenet_denominator = std::copysign(0.1, frenet_denominator == 0.0 ? 1.0 : frenet_denominator);
        }
        const double r_ref_frenet = curr_vx * ref_kappa_k0 / frenet_denominator;
        const double heading_error_rate_kin = curr_r - r_ref_frenet;
        const double geometric_error_dot_kin =
            curr_vx * std::sin(heading_error) + vy_est * std::cos(heading_error);
        double tracking_error_dot_raw = diagnostic_nan;
        double geometric_error_dot_raw = diagnostic_nan;
        if (prev_valid && obs_dt > 1e-6) {
            tracking_error_dot_raw =
                (curr_lateral_tracking_error - diagnostic_prev_tracking_error_) / obs_dt;
            geometric_error_dot_raw =
                (geometric_lateral_error - diagnostic_prev_geometric_error_) / obs_dt;
        }

        const double pred_k1_y_error = diagnostic_pred_k1_[1] - ref_y_k1;
        const double pred_k1_heading_error = diagnostic_pred_k1_[2] - ref_theta_k1;
        const double pred_k1_yaw_rate_error = diagnostic_pred_k1_[4] - curr_vx * ref_kappa_k1;
        const double pred_k5_y_error = diagnostic_pred_k5_[1] - ref_y_k5;
        const double pred_k5_heading_error = diagnostic_pred_k5_[2] - ref_theta_k5;
        const double pred_k5_yaw_rate_error = diagnostic_pred_k5_[4] - curr_vx * ref_kappa_k5;
        const double pred_kN_y_error = diagnostic_pred_kN_[1] - ref_y_kN;
        const double pred_kN_heading_error = diagnostic_pred_kN_[2] - ref_theta_kN;
        const double pred_kN_yaw_rate_error = diagnostic_pred_kN_[4] - curr_vx * ref_kappa_kN;

        local_log_stream_
            << current_time.toSec() << ',' << 6 << ',' << dt << ',' << obs_dt << ','
            << curr_vx_raw << ',' << curr_vx_raw * 3.6 << ','
            << curr_vy_status << ',' << curr_ax << ','
            << curr_x << ',' << curr_y << ',' << curr_theta << ','
            << received_mass_ << ',' << nmpc_params_.m << ',' << nmpc_params_.Iz << ','
            << nmpc_params_.lf << ',' << nmpc_params_.lr << ','
            << nmpc_params_.Cf << ',' << nmpc_params_.Cr << ','
            << nmpc_params_.N << ',' << nmpc_params_.Nc << ','
            << nmpc_params_.integration_grade << ',' << nmpc_params_.eso_disturbance_decay << ','
            << nmpc_params_.T_lag << ',' << nmpc_params_.Q_y << ',' << nmpc_params_.Q_theta << ','
            << nmpc_params_.Q_r << ',' << nmpc_params_.dR << ',' << slope_compensation_filter_tau_ << ','
            << nmpc_params_.Cf * nmpc_params_.lf / nmpc_params_.Iz << ','
            << nmpc_params_.Q_x << ',' << nmpc_params_.Q_vy << ',' << nmpc_params_.Q_delta << ','
            << nmpc_params_.R << ',' << nmpc_params_.delta_min << ',' << nmpc_params_.delta_max << ','
            << output_lpf_tau_ << ',' << (use_slope_compensation_ ? 1 : 0) << ','
            << slope_compensation_coeff_ << ',' << (use_ay_bias_compensation_ ? 1 : 0) << ','
            << (use_dynamic_ay_compensation_ ? 1 : 0) << ',' << (auto_update_total_weight_ ? 1 : 0) << ','
            << curr_lateral_tracking_error << ',' << curr_delta << ','
            << nmpc_raw_cmd << ',' << nmpc_safe_cmd_ << ','
            << delta_pp_raw << ',' << pp_safe_cmd_ << ',' << final_cmd << ','
            << blend_alpha_ << ',' << kappa << ',' << r_ref << ',' << theta << ','
            << curr_r << ',' << vy_est << ',' << curr_ay << ',' << effective_ay_bias_ << ','
            << ay_slope_compensation_raw << ',' << ay_slope_compensation_ << ','
            << ay_slope_compensation_ * slope_compensation_coeff_ << ','
            << eso_x1_ << ',' << eso_x2_ << ',' << d_pure_trailer << ',' << Model_r1_ << ','
            << lookahead_dist << ',' << preview_abs_curvature << ','
            << (nmpc_solve_success ? 1 : 0) << ',' << iter_time_ << ','
            << total_control_time_ << ',' << mpc_failure_count_ << ','
            << (using_pure_pursuit_flag_ ? 1 : 0) << ','
            << (using_mixed_mode_flag_ ? 1 : 0) << ','
            << curr_delta_raw << ',' << (std::abs(curr_delta_raw - curr_delta) > 1e-12 ? 1 : 0) << ','
            << (prev_valid ? diagnostic_prev_final_cmd_ : diagnostic_nan) << ','
            << steer_meas_rate << ',' << steer_cmd_rate << ','
            << steer_model_1step << ',' << steer_model_residual << ','
            << (nmpc_clamped ? 1 : 0) << ',' << (final_saturated ? 1 : 0) << ','
            << curr_r - r_ref << ',' << yaw_rate_dot_raw << ',' << r_dot_nominal << ','
            << yaw_rate_dot_with_eso << ',' << yaw_residual_nominal << ',' << yaw_residual_with_eso << ','
            << yaw_model_1step_nominal << ',' << yaw_model_1step_eso << ','
            << curr_r - eso_x1_ << ',' << eso_injected_k0 << ',' << eso_injected_k1 << ','
            << alpha_f_curr << ',' << alpha_r_curr << ',' << Fyf_curr << ',' << Fyr_curr << ','
            << ay_corrected << ',' << ay_model_no_slope << ',' << ay_model_with_slope << ','
            << ay_residual_no_slope << ',' << ay_residual_with_slope << ','
            << vx_dot_raw << ',' << vy_status_dot_raw << ',' << vy_dot_raw << ','
            << vy_dot_model << ',' << vy_dot_residual << ',' << curr_vy_status - vy_est << ','
            << beta_status << ',' << beta_est << ','
            << path->points.size() << ',' << nearest_idx << ',' << nearest_idx_jump << ','
            << nearest_distance << ',' << nearest_path_x << ',' << nearest_path_y << ','
            << nearest_path_yaw << ',' << geometric_lateral_error << ','
            << curr_lateral_tracking_error - geometric_lateral_error << ','
            << heading_error << ',' << heading_error_rate_kin << ',' << r_ref_frenet << ','
            << tracking_error_dot_raw << ',' << geometric_error_dot_raw << ',' << geometric_error_dot_kin << ','
            << ref_x_k0 << ',' << ref_y_k0 << ',' << ref_theta_k0 << ',' << ref_kappa_k0 << ','
            << ref_x_k1 << ',' << ref_y_k1 << ',' << ref_theta_k1 << ',' << ref_kappa_k1 << ','
            << ref_y_k5 << ',' << ref_theta_k5 << ',' << ref_kappa_k5 << ','
            << ref_y_kN << ',' << ref_theta_kN << ',' << ref_kappa_kN << ','
            << diagnostic_pred_k1_[0] << ',' << diagnostic_pred_k1_[1] << ','
            << diagnostic_pred_k1_[2] << ',' << diagnostic_pred_k1_[3] << ','
            << diagnostic_pred_k1_[4] << ',' << diagnostic_pred_k1_[5] << ','
            << pred_k1_y_error << ',' << pred_k1_heading_error << ',' << pred_k1_yaw_rate_error << ','
            << k5_index << ',' << diagnostic_pred_k5_[1] << ',' << diagnostic_pred_k5_[2] << ','
            << diagnostic_pred_k5_[3] << ',' << diagnostic_pred_k5_[4] << ',' << diagnostic_pred_k5_[5] << ','
            << pred_k5_y_error << ',' << pred_k5_heading_error << ',' << pred_k5_yaw_rate_error << ','
            << diagnostic_pred_kN_[1] << ',' << diagnostic_pred_kN_[2] << ','
            << diagnostic_pred_kN_[3] << ',' << diagnostic_pred_kN_[4] << ',' << diagnostic_pred_kN_[5] << ','
            << pred_kN_y_error << ',' << pred_kN_heading_error << ',' << pred_kN_yaw_rate_error << ','
            << diagnostic_u_sparse_[0] << ',' << diagnostic_u_sparse_[1] << ','
            << diagnostic_u_sparse_[2] << ',' << target_idx << ',' << (prev_valid ? 1 : 0) << ','
            << nmpc_params_.eso_disturbance_tau_s << ',' << curvature_smoothing_steps_ << ','
            << (slope_estimator_mode_ == "tire_force_residual" ? 1 : 0) << ','
            << (slope_dynamic_gate_enabled_ ? 1 : 0) << ',' << (slope_gate_active_ ? 1 : 0) << ','
            << slope_yaw_accel << ',' << slope_steer_rate << ',' << slope_compensation_limit_ << ','
            << (ukf_use_slope_disturbance_ ? 1 : 0) << ','
            << ukf_ay_innovation_raw_ << ',' << ukf_ay_innovation_used_ << ','
            << ukf_q_vy_ << ',' << ukf_q_r_ << ',' << ukf_r_ay_ << ',' << ukf_r_r_ << ','
            << ukf_vy_abs_max_ << ',' << curvature_smoothing_distance_m_ << ','
            << (use_equilibrium_feedforward_ ? 1 : 0) << ',' << last_delta_ff_ << ','
            << nmpc_params_.near_dense_control_steps << ',' << nmpc_params_.delta_rate_max << ','
            << (last_measurement_is_new_ ? 1 : 0) << ','
            << (last_observer_low_speed_reset_ ? 1 : 0) << ','
            << observer_dynamic_min_speed_mps_ << '\n';

        diagnostic_prev_valid_ = true;
        diagnostic_prev_delta_ = curr_delta;
        diagnostic_prev_vx_ = curr_vx_raw;
        diagnostic_prev_vy_status_ = curr_vy_status;
        diagnostic_prev_r_ = curr_r;
        diagnostic_prev_vy_ = vy_est;
        diagnostic_prev_tracking_error_ = curr_lateral_tracking_error;
        diagnostic_prev_geometric_error_ = geometric_lateral_error;
        diagnostic_prev_final_cmd_ = final_cmd;
        diagnostic_prev_nearest_idx_ = nearest_idx;

        ++local_log_pending_rows_;
        if (local_log_pending_rows_ >= local_log_flush_interval_) {
            local_log_stream_.flush();
            local_log_pending_rows_ = 0;
        }
        if (!local_log_stream_.good()) {
            ROS_ERROR("[%s] 本地诊断日志写入失败，关闭日志: %s",
                      getName().c_str(), local_log_path_.c_str());
            local_log_stream_.close();
            enable_local_log_ = false;
        }
    }

    race_msgs::ESOEstimation est_msg;
    est_msg.model_r1 = Model_r1_;              // 模型输出横摆角速度
    est_msg.vy_est1 = vy_est;                  // 侧向速度
    est_msg.eso1_x = eso_x1_;                  // ESO_x
    est_msg.eso1_total = eso_x2_;              // ESO_x2
    est_msg.eso1_pure = d_pure_trailer;        // 纯扰动
    est_msg.Cf_est = nmpc_params_.Cf;              // 当前质量插值得到的前轴侧偏刚度
    est_msg.Cr_est = nmpc_params_.Cr;              // 当前质量插值得到的后轴侧偏刚度
    est_msg.kappa = kappa;                     // 参考曲率
    est_msg.r_ref = r_ref;                     // 参考横摆率
    est_msg.theta = theta;                     // 参考航向角
    est_msg.vy_model = vy_model;               // 侧向速度模型
    est_msg.ay_slope_compensation = ay_slope_compensation_; // 侧向速度模型
    est_msg.iter_time = iter_time_;           // 迭代时间
    est_msg.total_control_time = total_control_time_;   // 整个控制周期时间，单位ms
    est_msg.mpc_failure_flag = mpc_failure_flag_;
    est_msg.using_pure_pursuit_flag = using_pure_pursuit_flag_;
    est_msg.require_over_take_flag = require_over_take_flag_;
    est_msg.using_mixed_mode_flag = using_mixed_mode_flag_;
    est_pub_.publish(est_msg);
}

// ---------------------- 路径处理与辅助函数 ----------------------
double ESOTracker::normalizeAngle(double angle) {
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

double ESOTracker::quaternion_to_yaw(const geometry_msgs::Quaternion& q) {
    tf::Quaternion tf_quat(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 rot_matrix(tf_quat);
    double roll, pitch, yaw;
    rot_matrix.getRPY(roll, pitch, yaw);
    return yaw;
}

int ESOTracker::find_nearest_path_point(const double x0, const double y0, const double yaw0,
                                        const race_msgs::Path& path) {
    double min_cost = std::numeric_limits<double>::max();
    int nearest_idx = -1;
    const double cy = std::cos(yaw0);
    const double sy = std::sin(yaw0);
    for (size_t i = 0; i < path.points.size(); ++i) {
        const auto& pt = path.points[i].pose.position;
        const double dx = pt.x - x0;
        const double dy = pt.y - y0;
        const double longitudinal = cy * dx + sy * dy;
        if (longitudinal < -path_projection_rear_gate_m_) {
            continue;
        }
        const double yaw_path = quaternion_to_yaw(path.points[i].pose.orientation);
        const double yaw_error = std::abs(angleDiff(yaw_path, yaw0));
        if (yaw_error > path_projection_heading_gate_rad_) {
            continue;
        }
        const double dist_sq = dx * dx + dy * dy;
        const double cost = dist_sq + path_projection_heading_weight_m2_ * yaw_error * yaw_error;
        if (cost < min_cost) {
            min_cost = cost;
            nearest_idx = i;
        }
    }
    if (nearest_idx >= 0) {
        return nearest_idx;
    }

    // 路径航向缺失或极端定位偏差时回退到原始欧氏最近点，保持与第五版兼容。
    double min_dist_sq = std::numeric_limits<double>::max();
    nearest_idx = 0;
    for (size_t i = 0; i < path.points.size(); ++i) {
        const auto& pt = path.points[i].pose.position;
        const double dist_sq = (pt.x - x0) * (pt.x - x0) + (pt.y - y0) * (pt.y - y0);
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            nearest_idx = static_cast<int>(i);
        }
    }
    return nearest_idx;
}

std::vector<double> ESOTracker::calculate_cumulative_distance(const race_msgs::Path& path, int start_idx) {
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

std::vector<double> ESOTracker::linear_interpolate(const std::vector<double>& s_original,
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

casadi::DM ESOTracker::interpolate_path_segment(const race_msgs::Path& path, const std::vector<double>& cum_dist,
                                                int start_idx, int end_idx, const std::vector<double>& s_target, double yaw0) {
    // yaw0 = 自车当前航向 curr_theta（由 process_race_path 透传 current_state[2]）。
    const double veh_yaw = yaw0;

    std::vector<double> s_orig, x_orig, y_orig, theta_orig;

    // 关键修改：把每个参考点的航向都用 angleDiff 表示为“相对自车当前航向”的量，
    // 再在 s 方向上连续解缠绕(unwrap)，得到一条连续、且锚定在 0 附近的参考航向曲线。
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

    auto x_interp = linear_interpolate(s_orig, x_orig, s_target);
    auto y_interp = linear_interpolate(s_orig, y_orig, s_target);
    auto theta_interp = linear_interpolate(s_orig, theta_orig, s_target);
    std::vector<double> kappa_interp(s_target.size(), 0.0);

    // V6：曲率窗口使用固定空间长度，不再使用固定预测步数。
    // 因此同一绝对位置在不同车速下看到的曲率尺度一致。
    if (s_target.size() >= 2) {
        const int n = static_cast<int>(s_target.size());
        const double ds_grid = std::max(1e-3, s_target[1] - s_target[0]);
        const int span = std::max(1, std::min(
            static_cast<int>(std::lround(curvature_smoothing_distance_m_ / ds_grid)), n - 1));
        for (int i = 0; i < n; ++i) {
            int left = i - span / 2;
            left = std::max(0, std::min(left, n - 1 - span));
            const int right = left + span;
            const double ds = s_target[right] - s_target[left];
            kappa_interp[i] = (ds > 1e-4)
                ? (theta_interp[right] - theta_interp[left]) / ds : 0.0;
        }
    }

    // 位置参考也转换到“自车体坐标系”（原点为自车当前位置，x 轴沿自车当前航向），
    // 与相对航向、相对动力学初值 (0,0,0) 保持一致，彻底摆脱全局朝向的影响。
    const double cos_y = std::cos(veh_yaw);
    const double sin_y = std::sin(veh_yaw);
    int n_waypoints = s_target.size();
    casadi::DM waypoints = casadi::DM::zeros(6, n_waypoints);
    const double vx_reference = (s_target.size() >= 2)
        ? std::max(1.0, (s_target[1] - s_target[0]) / nmpc_params_.dt)
        : 1.0;
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
        const auto equilibrium = computeSteadyStateFeedforward(vx_reference, kappa_interp[i]);
        // 前馈本身也从上一周期实际输出开始按物理速率生成，否则“前馈+反馈”的硬速率
        // 约束可能因为参考前馈跳变而无解。k=0锚定当前输出，k>=1每步推进一次。
        double delta_ff = previous_delta_ff;
        if (i > 0) {
            const double max_step = nmpc_params_.delta_rate_max * nmpc_params_.dt;
            delta_ff = std::max(previous_delta_ff - max_step,
                                std::min(previous_delta_ff + max_step, equilibrium[0]));
            delta_ff = std::max(nmpc_params_.delta_min,
                                std::min(nmpc_params_.delta_max, delta_ff));
        }
        previous_delta_ff = delta_ff;
        waypoints(4, i) = delta_ff;        // 速率可行的名义稳态前轮转角
        waypoints(5, i) = equilibrium[1];  // 名义稳态侧向速度
    }
    return waypoints;
}

casadi::DM ESOTracker::process_race_path(const race_msgs::Path& input_path, const std::vector<double>& current_state) {
    int nearest_idx = find_nearest_path_point(current_state[0], current_state[1], current_state[2], input_path);
    if (nearest_idx == -1) return casadi::DM::zeros(6, nmpc_params_.N + 1);

    // 设置体坐标系变换原点为自车当前位置，供 interpolate_path_segment 使用
    g_ref_x0 = current_state[0];
    g_ref_y0 = current_state[1];

    double calc_vx = std::max(current_state[3], 1.0);

    std::vector<double> s_target(nmpc_params_.N + 1);
    for (int i = 0; i <= nmpc_params_.N; ++i) {
        s_target[i] = calc_vx * nmpc_params_.dt * i;
    }

    double max_dist = s_target.back() + 10.0;

    std::vector<double> cum_dist = calculate_cumulative_distance(input_path, nearest_idx);

    int end_idx = nearest_idx;
    for (size_t i = 0; i < cum_dist.size(); ++i) {
        if (cum_dist[i] > max_dist) {
            end_idx = nearest_idx + i;
            break;
        }
        if (i == cum_dist.size() - 1) end_idx = nearest_idx + i;
    }
    end_idx = std::min(end_idx, static_cast<int>(input_path.points.size()) - 1);

    return interpolate_path_segment(input_path, cum_dist, nearest_idx, end_idx, s_target, current_state[2]);
}

std::array<double, 2> ESOTracker::computeSteadyStateFeedforward(double vx, double kappa) const {
    if (!use_equilibrium_feedforward_ || !std::isfinite(vx) || !std::isfinite(kappa)) {
        return {0.0, 0.0};
    }

    const double vx_safe = std::max(std::abs(vx), 2.0);
    const double r_eq = vx_safe * kappa;
    const double Cf = nmpc_params_.Cf;
    const double Cr = nmpc_params_.Cr;
    const double lf = nmpc_params_.lf;
    const double lr = nmpc_params_.lr;
    const double m = nmpc_params_.m;

    // 未知量 z=[delta_ff, vy_eq]。采用与当前线性侧偏刚度模型一致的稳态力/力矩平衡：
    // Cf*alpha_f + Cr*alpha_r = m*vx*r，lf*Cf*alpha_f-lr*Cr*alpha_r=0。
    Eigen::Matrix2d A;
    A << Cf, (-Cf - Cr) / vx_safe,
         lf * Cf, (-lf * Cf + lr * Cr) / vx_safe;
    Eigen::Vector2d b;
    b << m * vx_safe * r_eq - ((-Cf * lf + Cr * lr) / vx_safe) * r_eq,
         -((-lf * lf * Cf - lr * lr * Cr) / vx_safe) * r_eq;

    Eigen::Vector2d z = Eigen::Vector2d::Zero();
    const double det = A.determinant();
    if (std::isfinite(det) && std::abs(det) > 1e-9) {
        z = A.fullPivLu().solve(b);
    } else {
        z(0) = std::atan(nmpc_params_.L * kappa);
        z(1) = 0.0;
    }

    double delta_ff = equilibrium_feedforward_gain_ * z(0);
    if (!std::isfinite(delta_ff)) delta_ff = 0.0;
    if (!std::isfinite(z(1))) z(1) = 0.0;
    if (equilibrium_feedforward_limit_ > 0.0) {
        delta_ff = std::max(-equilibrium_feedforward_limit_,
                            std::min(equilibrium_feedforward_limit_, delta_ff));
    }
    return {delta_ff, z(1)};
}

bool ESOTracker::isNewVehicleMeasurement(double x, double y, double yaw, double vx, double vy,
                                         double r, double delta, double ay) {
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

// ---------------------- 核心算法  ----------------------

double ESOTracker::interpolateWithClampedEnds(
    double mass,
    const std::vector<double>& mass_points,
    const std::vector<double>& value_points) const {

    if (mass <= mass_points.front()) {
        return value_points.front();
    }
    if (mass >= mass_points.back()) {
        return value_points.back();
    }

    const auto upper = std::upper_bound(mass_points.begin(), mass_points.end(), mass);
    const std::size_t i1 = static_cast<std::size_t>(std::distance(mass_points.begin(), upper));
    const std::size_t i0 = i1 - 1;
    const double ratio = (mass - mass_points[i0]) / (mass_points[i1] - mass_points[i0]);
    return value_points[i0] + ratio * (value_points[i1] - value_points[i0]);
}

bool ESOTracker::validateMassInterpolationTables() const {
    if (!std::isfinite(nmpc_params_.m) || nmpc_params_.m <= 0.0) {
        ROS_ERROR("[%s] 固定车辆模型质量无效: m=%.3f kg",
                  getName().c_str(), nmpc_params_.m);
        return false;
    }

    const std::size_t n = nmpc_params_.mass_interp_points.size();
    if (n < 2 ||
        nmpc_params_.Iz_interp_points.size() != n ||
        nmpc_params_.lf_interp_points.size() != n ||
        nmpc_params_.lr_interp_points.size() != n ||
        nmpc_params_.Cf_interp_points.size() != n ||
        nmpc_params_.Cr_interp_points.size() != n) {
        ROS_ERROR("[%s] 插值数组长度不一致，且至少需要 2 个质量节点", getName().c_str());
        return false;
    }

    for (std::size_t i = 0; i < n; ++i) {
        const double mass = nmpc_params_.mass_interp_points[i];
        const double Iz = nmpc_params_.Iz_interp_points[i];
        const double lf = nmpc_params_.lf_interp_points[i];
        const double lr = nmpc_params_.lr_interp_points[i];
        const double Cf = nmpc_params_.Cf_interp_points[i];
        const double Cr = nmpc_params_.Cr_interp_points[i];

        if (!std::isfinite(mass) || !std::isfinite(Iz) || !std::isfinite(lf) || !std::isfinite(lr) ||
            !std::isfinite(Cf) || !std::isfinite(Cr)) {
            ROS_ERROR("[%s] 插值表第 %zu 个节点含 NaN/Inf", getName().c_str(), i);
            return false;
        }
        if (mass <= 0.0 || Iz <= 0.0 || Cf <= 0.0 || Cr <= 0.0 || lf <= 0.0 || lr <= 0.0) {
            ROS_ERROR("[%s] 插值表第 %zu 个节点物理范围无效: mass=%.3f, Iz=%.3f, lf=%.3f, lr=%.3f, Cf=%.3f, Cr=%.3f",
                      getName().c_str(), i, mass, Iz, lf, lr, Cf, Cr);
            return false;
        }
        if (i > 0 && mass <= nmpc_params_.mass_interp_points[i - 1]) {
            ROS_ERROR("[%s] mass_interp_points 必须严格递增: index=%zu, prev=%.3f, curr=%.3f",
                      getName().c_str(), i,
                      nmpc_params_.mass_interp_points[i - 1], mass);
            return false;
        }
    }
    return true;
}

void ESOTracker::updateMassDependentParameters(double mass) {
    if (!std::isfinite(mass) || mass <= 0.0) {
        ROS_WARN("[%s] 质量插值输入无效: %.3f kg，本周期保留上一组车辆参数",
                          getName().c_str(), mass);
        return;
    }

    nmpc_params_.m_total = mass;
    nmpc_params_.Iz = interpolateWithClampedEnds(
        mass, nmpc_params_.mass_interp_points, nmpc_params_.Iz_interp_points);
    nmpc_params_.lf = interpolateWithClampedEnds(
        mass, nmpc_params_.mass_interp_points, nmpc_params_.lf_interp_points);
    nmpc_params_.lr = interpolateWithClampedEnds(
        mass, nmpc_params_.mass_interp_points, nmpc_params_.lr_interp_points);
    nmpc_params_.Cf = interpolateWithClampedEnds(
        mass, nmpc_params_.mass_interp_points, nmpc_params_.Cf_interp_points);
    nmpc_params_.Cr = interpolateWithClampedEnds(
        mass, nmpc_params_.mass_interp_points, nmpc_params_.Cr_interp_points);
    nmpc_params_.L = nmpc_params_.lf + nmpc_params_.lr;

    ROS_INFO(
        "[%s][mass_interp] mass_input=%.1f kg, model_m=%.1f kg | Iz=%.1f kg*m^2, lf=%.4f m, lr=%.4f m, L=%.4f m, Cf=%.1f N/rad, Cr=%.1f N/rad, yaw_gain=%.4f",
        getName().c_str(), mass, nmpc_params_.m, nmpc_params_.Iz, nmpc_params_.lf, nmpc_params_.lr,
        nmpc_params_.L, nmpc_params_.Cf, nmpc_params_.Cr,
        nmpc_params_.Cf * nmpc_params_.lf / nmpc_params_.Iz);
}

void ESOTracker::updateDynamicAyBias(double lateral_tracking_error) {
    // 动态估计只在总开关和动态开关均打开时生效；否则由调用处退回静态/关闭逻辑。
    if (!use_ay_bias_compensation_ || !use_dynamic_ay_compensation_) {
        effective_ay_bias_ = use_ay_bias_compensation_ ? ay_bias_estimate_ : 0.0;
        return;
    }

    if (!std::isfinite(lateral_tracking_error)) {
        ROS_WARN("[ay_bias_dyn] 横向跟踪误差不是有限值，跳过本次更新: err=%.6f", lateral_tracking_error);
        effective_ay_bias_ = ay_bias_estimate_;
        return;
    }

    lateral_error_history_.push_back(lateral_tracking_error);
    if (lateral_error_history_.size() > static_cast<size_t>(dynamic_ay_error_window_size_)) {
        lateral_error_history_.pop_front();
    }

    const bool window_ready = !dynamic_ay_require_full_window_ ||
                              lateral_error_history_.size() >= static_cast<size_t>(dynamic_ay_error_window_size_);
    const double err_sum = std::accumulate(lateral_error_history_.begin(), lateral_error_history_.end(), 0.0);
    const double err_mean = lateral_error_history_.empty() ? 0.0 : err_sum / lateral_error_history_.size();

    if (!window_ready) {
        effective_ay_bias_ = ay_bias_estimate_;
        return;
    }

    if (std::abs(err_mean) <= dynamic_ay_error_threshold_) {
        effective_ay_bias_ = ay_bias_estimate_;
        return;
    }

    // 增量式准静态估计。默认 sign=-1：对应“ay 正偏 -> 横向误差均值为负”的当前诊断，
    // 即 err_mean<0 时增大 ay_bias_estimate_。若实车符号相反，将 dynamic_ay_bias_error_sign 设为 +1。
    const double raw_step = dynamic_ay_bias_error_sign_ * dynamic_ay_bias_learning_rate_ * err_mean;
    const double limited_step = std::max(-dynamic_ay_bias_max_step_,
                                  std::min(dynamic_ay_bias_max_step_, raw_step));
    const double old_bias = ay_bias_estimate_;
    ay_bias_estimate_ = std::max(dynamic_ay_bias_min_,
                          std::min(dynamic_ay_bias_max_, ay_bias_estimate_ + limited_step));
    effective_ay_bias_ = ay_bias_estimate_;

    ROS_WARN("[ay_bias_dyn] lat_err_now=%.5f, lat_err_mean=%.5f, queue=%zu/%d, threshold=%.5f, raw_step=%.8f, step=%.8f, ay_bias: %.6f -> %.6f, sign=%.1f",
             lateral_tracking_error, err_mean, lateral_error_history_.size(), dynamic_ay_error_window_size_,
             dynamic_ay_error_threshold_, raw_step, ay_bias_estimate_ - old_bias, old_bias, ay_bias_estimate_,
             dynamic_ay_bias_error_sign_);
}

void ESOTracker::ukfEstimateVy(double curr_vx, double curr_delta,
                              double curr_ay_corrected, double curr_r,
                              double lateral_disturbance, double dt,
                              bool measurement_is_new) {

    int L = 2;
    int n_sig = 2*L + 1;
    double lambda = 1.0 * (L + 1.0) - L;

    VectorXd Wm(n_sig), Wc(n_sig);
    Wm(0) = lambda / (L + lambda);
    Wc(0) = lambda / (L + lambda) + 2.0;
    for (int i=1; i<n_sig; i++) {
        Wm(i) = 1.0 / (2 * (L + lambda));
        Wc(i) = 1.0 / (2 * (L + lambda));
    }

    Matrix2d Q_ukf = (Matrix2d() << ukf_q_vy_, 0.0, 0.0, ukf_q_r_).finished();
    Matrix2d R_ukf = (Matrix2d() << ukf_r_ay_, 0.0, 0.0, ukf_r_r_).finished();

    Matrix2d P_scaled = (L + lambda) * ukf_P_est_;
    P_scaled = 0.5 * (P_scaled + P_scaled.transpose()) + 1e-6 * Matrix2d::Identity();

    Matrix2d sqrtP;
    LLT<Matrix2d> llt(P_scaled);
    if (llt.info() == Success) sqrtP = llt.matrixL();
    else sqrtP = Matrix2d::Identity() * 0.1;

    MatrixXd X_sig(L, n_sig);
    X_sig.col(0) = ukf_x_est_;
    for (int i=0; i<L; i++) {
        X_sig.col(i+1) = ukf_x_est_ + sqrtP.col(i);
        X_sig.col(i+1+L) = ukf_x_est_ - sqrtP.col(i);
    }

    MatrixXd X_sig_pred(L, n_sig);
    for (int i=0; i<n_sig; i++) {
        double vy_i = X_sig(0, i), r_i = X_sig(1, i);
        double alpha_f = curr_delta - std::atan2(vy_i + nmpc_params_.lf * r_i, curr_vx);
        double alpha_r = -std::atan2(vy_i - nmpc_params_.lr * r_i, curr_vx);
        double Fyf = nmpc_params_.Cf * alpha_f;
        double Fyr = nmpc_params_.Cr * alpha_r;
        double vy_dot = (Fyf * cos(curr_delta) + Fyr) / nmpc_params_.m -
                        curr_vx * r_i + lateral_disturbance;
        double r_dot = (nmpc_params_.lf * Fyf * cos(curr_delta) - nmpc_params_.lr * Fyr) / nmpc_params_.Iz;
        X_sig_pred(0, i) = vy_i + vy_dot * dt;
        X_sig_pred(1, i) = r_i + r_dot * dt;
    }

    Vector2d x_pred = Vector2d::Zero();
    for (int i=0; i<n_sig; i++) x_pred += Wm(i) * X_sig_pred.col(i);

    Matrix2d P_pred = Q_ukf;
    for (int i=0; i<n_sig; i++) {
        Vector2d diff_x = X_sig_pred.col(i) - x_pred;
        P_pred += Wc(i) * diff_x * diff_x.transpose();
    }
    P_pred = 0.5 * (P_pred + P_pred.transpose());

    Matrix2d P_pred_scaled = (L + lambda) * P_pred + 1e-6 * Matrix2d::Identity();
    LLT<Matrix2d> llt_pred(0.5 * (P_pred_scaled + P_pred_scaled.transpose()));
    Matrix2d chol_pred = (llt_pred.info() == Success) ? Matrix2d(llt_pred.matrixL()) : Matrix2d(Matrix2d::Identity() * 0.1);

    MatrixXd X_sig_update(L, n_sig);
    X_sig_update.col(0) = x_pred;
    for (int i=0; i<L; i++) {
        X_sig_update.col(i+1) = x_pred + chol_pred.col(i);
        X_sig_update.col(i+1+L) = x_pred - chol_pred.col(i);
    }

    MatrixXd Z_sig(2, n_sig);
    for (int i=0; i<n_sig; i++) {
        double vy_i = X_sig_update(0, i), r_i = X_sig_update(1, i);
        double alpha_f = curr_delta - std::atan2(vy_i + nmpc_params_.lf * r_i, curr_vx);
        double alpha_r = -std::atan2(vy_i - nmpc_params_.lr * r_i, curr_vx);
        double ay_model = (nmpc_params_.Cf * alpha_f * cos(curr_delta) + nmpc_params_.Cr * alpha_r) / nmpc_params_.m;
        Z_sig(0, i) = ay_model + lateral_disturbance;
        Z_sig(1, i) = r_i;
    }

    Vector2d z_pred = Vector2d::Zero();
    for (int i=0; i<n_sig; i++) z_pred += Wm(i) * Z_sig.col(i);

    Matrix2d P_zz = R_ukf;
    MatrixXd P_xz = MatrixXd::Zero(L, 2);
    for (int i=0; i<n_sig; i++) {
        Vector2d z_diff = Z_sig.col(i) - z_pred;
        Vector2d x_diff = X_sig_update.col(i) - x_pred;
        P_zz += Wc(i) * z_diff * z_diff.transpose();
        P_xz += Wc(i) * x_diff * z_diff.transpose();
    }

    if (measurement_is_new) {
        MatrixXd K = P_xz * P_zz.inverse();
        Vector2d innovation = Vector2d(curr_ay_corrected, curr_r) - z_pred;
        ukf_ay_innovation_raw_ = innovation(0);
        if (ukf_ay_innovation_limit_ > 0.0) {
            innovation(0) = std::max(-ukf_ay_innovation_limit_,
                                std::min(ukf_ay_innovation_limit_, innovation(0)));
        }
        ukf_ay_innovation_used_ = innovation(0);
        ukf_x_est_ = x_pred + K * innovation;
        ukf_P_est_ = P_pred - K * P_zz * K.transpose();
    } else {
        // 车辆状态保持值被控制回调重复使用时，只传播一次模型，不把同一测量重复融合。
        ukf_ay_innovation_raw_ = 0.0;
        ukf_ay_innovation_used_ = 0.0;
        ukf_x_est_ = x_pred;
        ukf_P_est_ = P_pred;
    }
    ukf_x_est_(0) = std::max(-ukf_vy_abs_max_,
                        std::min(ukf_vy_abs_max_, ukf_x_est_(0)));
    ukf_P_est_ = 0.5 * (ukf_P_est_ + ukf_P_est_.transpose());
}

void ESOTracker::esoCompute(double curr_r, double curr_delta, double dt,
                            bool measurement_is_new){
    double b_eso = (nmpc_params_.Cf * nmpc_params_.lf) / nmpc_params_.Iz;
    // 对保持的旧测量只做模型预测，不重复注入同一个横摆观测误差。
    double error_eso = measurement_is_new ? (curr_r - eso_x1_) : 0.0;
    eso_x1_ += (b_eso * curr_delta + 20.0 * error_eso + eso_x2_) * dt;
    eso_x2_ += (100.0 * error_eso) * dt;
}

MX ESOTracker::vehicleDynamicsModel(const MX& state, const MX& cmd_delta,
                                              const MX& vx, const MX& h_dist, const MX& dyn_params, const MX& ay_slope_comp) {
    MX theta = state(2), vy = state(3), r = state(4), delta = state(5);
    MX m_sym = dyn_params(0), Iz_sym = dyn_params(1), lf_sym = dyn_params(2), lr_sym = dyn_params(3);
    MX Cf_sym = dyn_params(4), Cr_sym = dyn_params(5);

    MX vx_safe = fmax(vx, 2.0);

    MX alpha_f = delta - atan2((vy + lf_sym * r), vx_safe);
    MX alpha_r = -atan2((vy - lr_sym * r), vx_safe);
    MX Fyf = Cf_sym * alpha_f;
    MX Fyr = Cr_sym * alpha_r;

    MX d_vy = (Fyf * cos(delta) + Fyr) / m_sym - vx * r + ay_slope_comp;

    MX d_r = (lf_sym * Fyf * cos(delta) - lr_sym * Fyr) / Iz_sym + h_dist;
    MX d_x = vx * cos(theta) - vy * sin(theta);
    MX d_y = vx * sin(theta) + vy * cos(theta);
    MX d_theta = r;
    MX d_delta = (cmd_delta - delta) / nmpc_params_.T_lag;

    return vertcat(d_x, d_y, d_theta, d_vy, d_r, d_delta);
}

void ESOTracker::buildNMPSolver() {
    solver_.opti = Opti();
    int nx = nmpc_params_.nx, nu = nmpc_params_.nu, N = nmpc_params_.N, Nc = nmpc_params_.Nc;

    solver_.X = solver_.opti.variable(nx, N+1);
    solver_.U_sparse = solver_.opti.variable(nu, Nc);
    solver_.P_x0 = solver_.opti.parameter(nx);
    // [x_ref, y_ref, theta_ref, kappa_ref, delta_ff, vy_eq]
    solver_.P_waypoints = solver_.opti.parameter(6, N+1);
    solver_.P_vx = solver_.opti.parameter(1);
    solver_.P_ay_slope_comp = solver_.opti.parameter(1);
    solver_.P_u_prev = solver_.opti.parameter(1);
    solver_.P_h_hat = solver_.opti.parameter(1);
    solver_.P_dyn_params = solver_.opti.parameter(6);

    // V6 前密后疏 move blocking。前若干步逐拍优化，远端自动均分，避免 V5 每个控制块
    // 固定保持约 0.55~0.65s。U_sparse 表示相对稳态前馈的反馈修正量。
    solver_.control_block_start.clear();
    solver_.control_block_length.clear();
    const int dense_blocks = std::max(0, std::min(nmpc_params_.near_dense_control_steps,
                                                   std::min(N, Nc - 1)));
    int assigned_steps = 0;
    for (int i = 0; i < dense_blocks; ++i) {
        solver_.control_block_start.push_back(assigned_steps);
        solver_.control_block_length.push_back(1);
        ++assigned_steps;
    }
    const int remaining_blocks = Nc - dense_blocks;
    const int remaining_steps = N - assigned_steps;
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
        throw std::runtime_error("V6 control block layout invalid");
    }

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
        MX st = solver_.X(Slice(), k), con = solver_.U_full_command(Slice(), k);
        MX feedback = solver_.U_full_feedback(Slice(), k);
        MX h = solver_.P_h_hat * pow(nmpc_params_.eso_disturbance_decay, k);


        if (nmpc_params_.integration_grade >= 0.5 && nmpc_params_.integration_grade < 1.5) { // Euler前向积分
            solver_.opti.subject_to(solver_.X(Slice(), k+1) == st + nmpc_params_.dt * vehicleDynamicsModel(st, con, solver_.P_vx, h, solver_.P_dyn_params, solver_.P_ay_slope_comp));
        }
        else if (nmpc_params_.integration_grade >= 1.5 && nmpc_params_.integration_grade < 2.5) { // RK2积分
            MX k1 = vehicleDynamicsModel(st, con, solver_.P_vx, h, solver_.P_dyn_params, solver_.P_ay_slope_comp);
            MX k2 = vehicleDynamicsModel(st + nmpc_params_.dt / 2.0 * k1, con, solver_.P_vx, h, solver_.P_dyn_params, solver_.P_ay_slope_comp);
            solver_.opti.subject_to(solver_.X(Slice(), k+1) == st + nmpc_params_.dt * k2);
        }
        else if (nmpc_params_.integration_grade >= 3.5 && nmpc_params_.integration_grade < 4.5) { // RK4积分
            MX k1 = vehicleDynamicsModel(st, con, solver_.P_vx, h, solver_.P_dyn_params, solver_.P_ay_slope_comp);
            MX k2 = vehicleDynamicsModel(st + nmpc_params_.dt/2 * k1, con, solver_.P_vx, h, solver_.P_dyn_params, solver_.P_ay_slope_comp);
            MX k3 = vehicleDynamicsModel(st + nmpc_params_.dt/2 * k2, con, solver_.P_vx, h, solver_.P_dyn_params, solver_.P_ay_slope_comp);
            MX k4 = vehicleDynamicsModel(st + nmpc_params_.dt * k3, con, solver_.P_vx, h, solver_.P_dyn_params, solver_.P_ay_slope_comp);
            solver_.opti.subject_to(solver_.X(Slice(), k+1) == st + nmpc_params_.dt/6 * (k1 + 2*k2 + 2*k3 + k4));
        }
        else {
            throw std::runtime_error("integration_grade不在有效范围内，无法选择积分方法");
        }

        MX ref_x = solver_.P_waypoints(0, k+1), ref_y = solver_.P_waypoints(1, k+1);
        MX ref_theta = solver_.P_waypoints(2, k+1), ref_kappa = solver_.P_waypoints(3, k+1);
        MX ref_delta = solver_.P_waypoints(4, k+1), ref_vy = solver_.P_waypoints(5, k+1);
        // 体坐标系下，X(2) 与 ref_theta 均为连续、锚定在 0 附近的相对航向，
        // 不会跨越 ±pi 折叠边界，因此直接作差即可，无需 atan2(sin,cos) 折叠。
        // 移除折叠后，代价函数对航向误差全程光滑可导，消除 ±pi/2 附近的梯度突变。
        MX e_theta = solver_.X(2, k+1) - ref_theta;

        J += nmpc_params_.Q(0,0) * pow(solver_.X(0, k+1) - ref_x, 2);
        J += nmpc_params_.Q(1,1) * pow(solver_.X(1, k+1) - ref_y, 2);
        J += nmpc_params_.Q(2,2) * pow(e_theta, 2);
        J += nmpc_params_.Q(4,4) * pow(solver_.X(4, k+1) - solver_.P_vx * ref_kappa, 2);
        J += nmpc_params_.Q(3,3) * pow(solver_.X(3, k+1) - ref_vy, 2);
        J += nmpc_params_.Q(5,5) * pow(solver_.X(5, k+1) - ref_delta, 2);
        // 只惩罚相对平衡态前馈的反馈量，避免优化器为了 R 项主动抵消正确前馈。
        J += nmpc_params_.R * pow(feedback, 2);

        const MX previous_command = (k == 0)
            ? solver_.P_u_prev : solver_.U_full_command(0, k - 1);
        const MX command_increment = solver_.U_full_command(0, k) - previous_command;
        J += nmpc_params_.dR * pow(command_increment, 2);
        solver_.opti.subject_to(solver_.opti.bounded(
            -nmpc_params_.delta_rate_max * nmpc_params_.dt,
            command_increment,
             nmpc_params_.delta_rate_max * nmpc_params_.dt));
    }

    solver_.opti.subject_to(solver_.opti.bounded(
        nmpc_params_.delta_min, solver_.U_full_command, nmpc_params_.delta_max));

    solver_.opti.minimize(J);

    Dict opts = {
        {"ipopt.print_level", 0},
        {"ipopt.sb", "yes"},
        {"ipopt.max_iter", 100},
        {"ipopt.tol", 1e-2},
        {"ipopt.acceptable_tol", 5e-2},
        {"ipopt.acceptable_iter", 5},
        {"print_time", 0},

        {"ipopt.warm_start_init_point", "yes"},
        {"ipopt.warm_start_bound_push", 1e-9},
        {"ipopt.warm_start_slack_bound_push", 1e-9},
        {"ipopt.warm_start_mult_bound_push", 1e-9},

    };
    solver_.opti.solver("ipopt", opts);
}

bool ESOTracker::solveNMPC(const std::vector<double>& current_state, const casadi::DM& waypoints,
                                      std::vector<double>& control_output) {

    auto start_time = std::chrono::high_resolution_clock::now();

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

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end_time - start_time;
      //  ROS_INFO("[%s] NMPC 求解成功! 耗时: %.2f ms", getName().c_str(), elapsed.count());
        ROS_INFO("[%s] NMPC 求解成功! \033[1;32m耗时: %.2f ms\033[0m, \033[38;5;208mCf_interp: %.2f, Cr_interp: %.2f\033[0m",
        getName().c_str(), elapsed.count(), nmpc_params_.Cf, nmpc_params_.Cr);

        solver_.sol_prev = std::make_unique<casadi::OptiSol>(sol);
        solver_.has_prev_sol = true;

        if (enable_local_log_) {
            const casadi::DM x_solution = sol.value(solver_.X);
            const casadi::DM u_solution = sol.value(solver_.U_sparse);
            const casadi::DM command_solution = sol.value(solver_.U_full_command);
            const int k1 = std::min(1, nmpc_params_.N);
            const int k5 = std::min(5, nmpc_params_.N);
            const int kN = nmpc_params_.N;

            for (int state_idx = 0; state_idx < 6; ++state_idx) {
                diagnostic_pred_k1_[state_idx] = static_cast<double>(x_solution(state_idx, k1));
                diagnostic_pred_k5_[state_idx] = static_cast<double>(x_solution(state_idx, k5));
                diagnostic_pred_kN_[state_idx] = static_cast<double>(x_solution(state_idx, kN));
            }
            diagnostic_u_sparse_.fill(std::numeric_limits<double>::quiet_NaN());
            for (int control_idx = 0; control_idx < std::min(3, nmpc_params_.Nc); ++control_idx) {
                const int prediction_index = solver_.control_block_start[control_idx];
                diagnostic_u_sparse_[control_idx] =
                    static_cast<double>(command_solution(0, prediction_index));
            }
        }

        control_output[0] = static_cast<double>(sol.value(solver_.U_full_command(0, 0)));
        return true;
    } catch (std::exception& e) {

       auto end_time = std::chrono::high_resolution_clock::now();
       std::chrono::duration<double, std::milli> elapsed = end_time - start_time;
       ROS_WARN("[%s] NMPC 求解失败! 耗时: %.2f ms, 原因: %s", getName().c_str(), elapsed.count(), e.what());

        solver_.has_prev_sol = false;
        solver_.sol_prev = nullptr;
        resetNmpcPredictionDiagnostics();
        return false;
    }
}

} // namespace race_tracker

PLUGINLIB_EXPORT_CLASS(race_tracker::ESOTracker, race_tracker::ControllerPluginBase)

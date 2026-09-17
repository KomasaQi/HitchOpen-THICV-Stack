#ifndef RACE_TRACKER_ESO_TRACKER_H
#define RACE_TRACKER_ESO_TRACKER_H

#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <memory>
#include <tf/transform_datatypes.h>
#include <deque>
#include <cstddef>
#include <fstream>
#include <array>
#include <string>
#include <future>
#include <chrono>
#include <limits>

// ROS 插件和消息相关头文件
#include "race_tracker/controller_plugin_base.h"
#include <race_msgs/Control.h>
#include <race_msgs/VehicleStatus.h>
#include <race_msgs/Path.h>
#include <race_msgs/Flag.h>
#include <race_msgs/ESOEstimation.h>
#include <std_msgs/Int32.h>

namespace race_tracker {

// NMPC参数
struct NMPCParams {
    // --- 基础配置 ---
    double m;
    double Iz;
    double lf;
    double lr;
    double L;  // 当前质量节点下的轴距，始终由 lf + lr 更新
    double T_lag;
    double actuator_rate_limit;  // 实际前轮执行器速率上限，rad/s；不同于指令速率约束
    double dt;
    int N;
    int Nc;
    int nx;
    int nu;

    // --- 转角幅值约束 ---
    double delta_max;
    double delta_min;
    double delta_rate_max;  // 前轮转角指令硬速率约束，rad/s

    // --- 随整车质量插值的车辆参数 ---
    // Iz、lf、lr、Cf、Cr 为当前质量下的实时插值结果；L 始终由 lf+lr 更新。
    double Cf;
    double Cr;
    std::vector<double> mass_interp_points;
    std::vector<double> Iz_interp_points;
    std::vector<double> lf_interp_points;
    std::vector<double> lr_interp_points;
    std::vector<double> Cf_interp_points;
    std::vector<double> Cr_interp_points;

    // --- 积分器 ---
    double integration_grade;
    double eso_disturbance_decay;  // ESO 扰动在预测域内的逐步保留系数 [0, 1]
    double eso_disturbance_tau_s;  // >0 时用 exp(-dt/tau) 生成上述系数；<=0 时使用旧系数

    // --- 代价函数权重 ---
    double Q_x, Q_y, Q_theta;
    double Q_vy, Q_r, Q_delta;
    double R;
    double dR;  // V11: 反馈修正增量软惩罚；总指令变化率仍由硬约束保证
    int near_dense_control_steps;  // 预测域前部逐拍控制的步数，其余控制块自动均分

    double m_total;  // 质量插值的回退输入，kg


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
    casadi::MX P_nominal;
    casadi::MX P_transient_yaw_weight;
    casadi::MX P_feedback_first_weight;
    casadi::MX P_lateral_weight;
    casadi::MX P_stiffness_weight;
    casadi::Function step;
    unsigned long long generation = 0;
    casadi::MX P_vx;
    casadi::MX P_u_prev;
    casadi::MX P_feedback_prev;
    casadi::MX P_h_hat;
    casadi::MX P_dyn_params;
    casadi::MX P_ay_slope_comp;
    casadi::MX U_full_feedback;
    casadi::MX U_full_command;
    std::vector<int> control_block_start;
    std::vector<int> control_block_length;
    std::unique_ptr<casadi::OptiSol> sol_prev;
    bool has_prev_sol;
};

// V11: worker owns Opti and its warm state. Main thread exchanges value snapshots only.
struct NmpcJob {
    std::vector<double> state, dyn;
    casadi::DM waypoints;
    double vx, slope, disturbance, previous_command, previous_feedback, previous_nominal;
    double transient_yaw_weight = 0.0;
    double feedback_first_weight = std::numeric_limits<double>::quiet_NaN();
    double lateral_weight = std::numeric_limits<double>::quiet_NaN();
    unsigned long long generation;
};
struct NmpcResult {
    bool success = false, solver_success = false, warm = false;
    int status_code = 3, iterations = -1;
    std::string status = "not_attempted";
    double command = 0.0, nominal0 = 0.0, nominal_anchor = 0.0, wall_ms = 0.0;
    double solve_ms = std::numeric_limits<double>::quiet_NaN(), stiffness_weight = 0.0;
    double road_ff0 = 0.0, preview_ff0 = 0.0, rate_anticipated_ff0 = 0.0, feedback0 = 0.0;
    std::vector<double> command_sequence;
    double inf_pr = std::numeric_limits<double>::quiet_NaN();
    double inf_du = std::numeric_limits<double>::quiet_NaN();
    double constraint_violation = std::numeric_limits<double>::infinity();
    std::array<double, 6> pred1, pred5, predN;
    std::array<double, 3> controls;
    NmpcResult() {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        pred1.fill(nan); pred5.fill(nan); predN.fill(nan); controls.fill(nan);
    }
};

// 核心控制器类继承自 ControllerPluginBase
class ESOTracker : public ControllerPluginBase {
public:
    ESOTracker();
    ~ESOTracker() override;

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
    void initializeLocalLog();

    casadi::MX vehicleDynamicsModel(const casadi::MX& state, const casadi::MX& cmd_delta,
                                    const casadi::MX& vx, const casadi::MX& h_dist,
                                    const casadi::MX& dyn_params, const casadi::MX& ay_slope_comp);

    bool solveNMPC(const std::vector<double>& current_state, const casadi::DM& waypoints,
                   std::vector<double>& control_output);

    // V8: 采集 IPOPT/CasADi 求解状态；统计读取失败不影响控制。
    void captureNmpcSolverStats(NmpcResult& result);
    NmpcResult runNmpcJob(const NmpcJob& job);
    double nmpcStiffnessWeight(double vx, const std::vector<double>& parameters) const;
    void recordFallbackReason(int reason_code, const ros::Time& now);
    void drivingModeCallback(const std_msgs::Int32::ConstPtr& msg);

    // 仅用于本地日志：缓存求解后的预测状态，不参与控制计算。
    void resetNmpcPredictionDiagnostics();


    void ukfEstimateVy(double curr_vx, double curr_delta, double curr_ay_corrected,
                       double curr_r, double lateral_disturbance, double dt,
                       bool measurement_is_new);

    // 根据 received_mass_ 对 Iz、lf、Cf、Cr 进行分段线性插值；区间外保持端点值。
    double interpolateWithClampedEnds(double mass,
                                      const std::vector<double>& mass_points,
                                      const std::vector<double>& value_points) const;
    bool validateMassInterpolationTables() const;
    void updateMassDependentParameters(double mass);

    void esoCompute(double curr_r, double curr_delta, double dt, bool measurement_is_new);

    double normalizeAngle(double angle);

    // --- ROS 与路径处理辅助函数  ---
    double quaternion_to_yaw(const geometry_msgs::Quaternion& q);
    int find_nearest_path_point(const double x0, const double y0, const double yaw0,
                                const race_msgs::Path& path);
    std::vector<double> calculate_cumulative_distance(const race_msgs::Path& path, int start_idx);
    std::vector<double> linear_interpolate(const std::vector<double>& s_original,
                                           const std::vector<double>& val_original,
                                           const std::vector<double>& s_target);
    casadi::DM interpolate_path_segment(const race_msgs::Path& path, const std::vector<double>& cum_dist,
                                        int start_idx, int end_idx, const std::vector<double>& s_target, double yaw0);
    casadi::DM process_race_path(const race_msgs::Path& input_path, const std::vector<double>& current_state);

    // 线性单轨稳态平衡前馈。返回 [delta_ff, vy_eq]；反馈和扰动观测继续负责模型失配。
    std::array<double, 2> computeSteadyStateFeedforward(double vx, double kappa) const;
    double previewPathValue(const casadi::DM& waypoints, int row, double base_index,
                            double preview_time_s) const;
    void updateActuatorLagEstimate(double measured_delta, double vx, double dt,
                                   bool measurement_is_new, bool adaptation_allowed);

    bool isNewVehicleMeasurement(double x, double y, double yaw, double vx, double vy,
                                 double r, double delta, double ay);


    // 侧向加速度零偏的准静态自校正：基于横向跟踪误差滑动窗口，非常缓慢地修正 ay 零偏估计
    void updateDynamicAyBias(double lateral_tracking_error);

private:
    ros::Publisher est_pub_;//发布话题
    ros::Subscriber driving_mode_sub_;
    double blend_alpha_;
    double nmpc_safe_cmd_;
    ros::Time start_time_;

    // 输出端一阶低通滤波，平滑方向盘高频抖动
    double output_lpf_tau_ = 0.0;   // 时间常数(s)，<=0 表示关闭
    double final_cmd_filt_ = 0.0;   // 滤波器状态
    bool   final_cmd_filt_init_ = false;


     // 动态预瞄参数
    double min_lookahead_distance_;
    double lookahead_speed_coeff_;
    int curvature_smoothing_steps_;          // 仅兼容旧配置/日志
    double curvature_smoothing_distance_m_;  // V6/V7固定空间曲率平滑长度，不再随车速变化
    double lookahead_curvature_coeff_;  // 根据预瞄路径最大绝对曲率缩短预瞄距离，单位 m^2

    // V6/V7名义稳态转角前馈
    bool use_equilibrium_feedforward_;
    double equilibrium_feedforward_gain_;
    double equilibrium_feedforward_limit_;
    double last_delta_ff_;
    double last_road_delta_ff_ = 0.0;
    double last_preview_delta_ff_ = 0.0;
    double last_preview_kappa_ = 0.0;
    double last_feedback_command_ = 0.0;
    double feedback_memory_command_ = 0.0;

    // V16转向执行链：固定低阶物理模型 = 一拍传输延迟 + 快一阶 + 实际速率饱和。
    // 只有两个连续物理量需要标定：T_lag 与 actuator_rate_limit；不按速度/误差分段。
    bool actuator_one_step_delay_enabled_ = true;
    bool actuator_lag_adaptation_enabled_ = false;
    double actuator_lag_estimate_s_ = 0.10;
    double actuator_lag_raw_s_ = std::numeric_limits<double>::quiet_NaN();
    double actuator_lag_min_s_ = 0.05;
    double actuator_lag_max_s_ = 0.80;
    double actuator_lag_filter_tau_s_ = 3.0;
    double actuator_lag_max_update_s_per_cycle_ = 0.01;
    double actuator_lag_min_speed_mps_ = 5.0;
    double actuator_lag_min_command_error_rad_ = 0.01;
    double actuator_lag_min_measured_rate_radps_ = 0.01;
    double actuator_lag_max_measured_rate_radps_ = 0.60;
    bool actuator_lag_sample_valid_ = false;
    bool actuator_lag_prev_valid_ = false;
    double actuator_lag_prev_delta_ = 0.0;
    double actuator_lag_prev_command_ = 0.0;
    bool feedforward_preview_enabled_ = true;
    double feedforward_preview_gain_ = 1.0;
    double feedforward_extra_preview_s_ = 0.0;
    double feedforward_preview_max_s_ = 0.40;
    double feedforward_preview_time_s_ = 0.0;
    double feedforward_preview_distance_m_ = 0.0;
    bool feedforward_rate_anticipation_enabled_ = true;
    double last_rate_anticipated_ff_ = 0.0;

    // V12: keep NMPC ownership when the supplied path misses only a very short tail.
    // The extension is bounded and follows the last reliable tangent/curvature.
    bool reference_tail_extrapolation_enabled_ = true;
    double reference_tail_extrapolation_max_m_ = 5.0;
    double reference_tail_min_coverage_ratio_ = 0.85;
    bool reference_tail_extrapolated_ = false;
    double reference_tail_extrapolation_used_m_ = 0.0;

    // V12--V15: continuous NMPC cost scheduling, not a recovery controller or mode switch.
    bool transient_yaw_damping_enabled_ = true;
    double transient_yaw_rate_extra_weight_ = 5000.0;
    double transient_lateral_error_start_m_ = 1.00;
    double transient_lateral_error_full_m_ = 2.00;
    double transient_yaw_error_start_radps_ = 0.06;
    double transient_yaw_error_full_radps_ = 0.20;
    double transient_yaw_recovery_start_radps_ = 0.05;
    double transient_yaw_recovery_full_radps_ = 0.16;
    double transient_course_tighten_start_rad_ = 0.06;
    double transient_course_tighten_full_rad_ = 0.12;
    double transient_yaw_damping_factor_ = 0.0;
    double transient_yaw_rate_weight_ = 0.0;
    bool eso_transient_gate_enabled_ = true;
    double eso_transient_gate_factor_ = 0.25;
    double eso_transient_gate_recovery_factor_ = 0.20;
    bool eso_transient_gated_ = false;
    double transient_course_tighten_factor_ = 0.0;
    double effective_transient_yaw_start_radps_ = 0.06;
    double effective_transient_yaw_full_radps_ = 0.20;
    double effective_eso_transient_gate_factor_ = 0.25;

    // V14/V15: preserve the large global/in-horizon dR needed by the real vehicle, but
    // relax only the first cross-cycle feedback increment during a low-speed transient.
    // The release fades to zero before highway speed and at high load.
    bool feedback_first_step_release_enabled_ = true;
    double feedback_first_step_min_scale_ = 0.25;
    double feedback_release_speed_start_kmh_ = 80.0;
    double feedback_release_speed_full_kmh_ = 95.0;
    double feedback_release_lateral_start_m_ = 0.15;
    double feedback_release_lateral_full_m_ = 0.75;
    double feedback_release_yaw_start_radps_ = 0.04;
    double feedback_release_yaw_full_radps_ = 0.14;
    double feedback_release_factor_ = 0.0;
    double feedback_first_dR_weight_ = 0.0;

    // V15: course alignment is the primary proof that the vehicle is following a
    // parallel offset. Yaw mismatch attenuates but no longer completely suppresses Qy.
    bool aligned_lateral_weight_enabled_ = true;
    double aligned_lateral_extra_Q_y_ = 200.0;
    double aligned_lateral_error_start_m_ = 0.08;
    double aligned_lateral_error_full_m_ = 0.22;
    double aligned_course_error_start_rad_ = 0.04;
    double aligned_course_error_full_rad_ = 0.12;
    double aligned_yaw_error_start_radps_ = 0.04;
    double aligned_yaw_error_full_radps_ = 0.18;
    double aligned_yaw_gate_floor_ = 0.45;
    double aligned_lateral_speed_start_kmh_ = 85.0;
    double aligned_lateral_speed_full_kmh_ = 100.0;
    double aligned_lateral_weight_factor_ = 0.0;
    double effective_lateral_weight_ = 0.0;
    double aligned_yaw_alignment_gate_ = 1.0;
    double aligned_lateral_speed_gate_ = 1.0;

    // V13: loaded operation is an explicitly low-confidence model regime because the
    // received gross mass does not replace the fixed bicycle-model mass. Add damping
    // and de-rate ESO smoothly without inventing uncalibrated mass/inertia tables.
    bool load_aware_stability_enabled_ = true;
    double load_stability_ratio_start_ = 1.8;
    double load_stability_ratio_full_ = 4.0;
    double load_yaw_rate_extra_weight_ = 1000.0;
    double load_eso_confidence_min_ = 0.35;
    double load_stability_factor_ = 0.0;
    double load_yaw_rate_weight_ = 0.0;
    double effective_yaw_rate_extra_weight_ = 0.0;
    double eso_load_confidence_scale_ = 1.0;

    // 路径投影在空间交叉处加入航向和车后点惩罚，避免纯欧氏最近点跳支路。
    double path_projection_heading_weight_m2_;
    double path_projection_heading_gate_rad_;
    double path_projection_rear_gate_m_;
    bool use_geometric_path_heading_ = true;
    double geometric_heading_window_m_ = 2.0;

    // --- 核心参数结构体 ---
    NMPCParams nmpc_params_;
    NMPSolver solver_;

    double current_cmd_;
    // Worker solver parameters set only in worker; these snapshots are owned by control callback.
    NmpcJob nmpc_job_input_;
    std::future<NmpcResult> nmpc_future_;
    unsigned long long warm_generation_ = 1;
    unsigned long long late_result_count_ = 0;
    double last_worker_wall_ms_ = 0.0;
    double last_constraint_violation_ = 0.0;
    double last_nominal_command_ = 0.0;
    double last_nominal_anchor_ = 0.0;
    double last_feedback_memory_input_ = 0.0;
    double last_previous_command_input_ = 0.0;
    double last_late_worker_wall_ms_ = 0.0;
    int last_late_status_code_ = 0;
    bool solver_worker_busy_ = false;
    int nmpc_integration_substeps_ = 1;
    bool enable_low_speed_stabilization_ = true;
    double nmpc_euler_stability_margin_ = 1.8;
    double nmpc_timing_print_period_s_ = 0.0;
    double last_ipopt_wall_ms_ = 0.0, last_stiffness_weight_ = 0.0;
    std::chrono::steady_clock::time_point last_timing_print_;
    std::vector<double> cached_plan_;
    ros::Time cached_plan_stamp_;
    double fallback_plan_max_age_s_ = 0.15, cached_plan_age_s_ = -1.0;
    bool enable_pp_emergency_fallback_ = false;
    int output_source_code_ = 0;
    int ukf_integration_substeps_ = 4;
    double nmpc_constraint_tolerance_ = 0.02;
    bool reference_valid_ = false;
    double reference_origin_x_ = 0.0, reference_origin_y_ = 0.0;
    double reference_remaining_m_ = 0.0, reference_extension_m_ = 0.0;
    double reference_dkappa_ds_ = 0.0;
    double eso_raw_disturbance_ = 0.0, eso_filtered_disturbance_ = 0.0;
    double eso_disturbance_confidence_ = 0.0;
    bool eso_disturbance_limited_ = false;
    ros::Time last_control_time_;


    // V7/V8求解截止与输出安全层。IPOPT内部CPU上限用于主动终止，外层墙钟截止
    // 用于拒绝过期解；V11成功即采用，无PP锁存或恢复等待。
    double nmpc_solve_deadline_ms_ = 50.0;
    double nmpc_ipopt_cpu_time_limit_ms_ = 45.0;
    bool enforce_final_output_rate_limit_ = true;
    bool publish_steering_angle_velocity_ = true;
    double steering_angle_velocity_cmd_radps_ = 0.35;
    bool last_nmpc_deadline_missed_ = false;
    bool last_final_output_rate_limited_ = false;
    unsigned long long nmpc_timeout_count_ = 0;

    // V11 status: 0=idle,1=accepted,2=deadline,3=exception,4=low/reverse speed,
    // 5=reserved,6=manual,7=invalid reference,8=worker busy,9=solution validation rejected.
    bool last_nmpc_attempted_ = false;
    bool last_nmpc_solver_returned_success_ = false;
    bool last_nmpc_warm_start_used_ = false;
    int last_nmpc_status_code_ = 0;
    int last_nmpc_iter_count_ = -1;
    double last_nmpc_inf_pr_ = 0.0;
    double last_nmpc_inf_du_ = 0.0;
    std::string last_nmpc_return_status_ = "not_attempted";

    // Legacy CSV state columns. V11 has no fallback hold or recovery eligibility gates.
    bool fallback_latched_ = false;
    bool fallback_reentry_active_ = false;
    int fallback_reason_code_ = 0;  // 0=无,1=启动大偏差,2=超时,3=求解失败/worker忙,4=AD重新接管,5=路径无效
    ros::Time fallback_enter_time_;
    int nmpc_success_streak_ = 0;
    double fallback_reentry_alpha_ = 0.0;
    double fallback_reentry_max_kappa_step_1pm_ = 0.003;
    double nmpc_attempt_min_speed_mps_ = 3.0;
    int reference_stable_streak_ = 0;
    bool last_reference_kappa_valid_ = false;
    double last_reference_kappa_ = 0.0;
    double reference_kappa_step_ = 0.0;
    bool reference_stable_this_cycle_ = false;
    int reference_prev_nearest_idx_ = -1;

    // Legacy recovery diagnostics are always false/zero in V11.
    bool startup_recovery_checked_ = false;
    bool startup_recovery_active_ = false;
    int startup_recovery_alignment_streak_ = 0;
    bool startup_recovery_steer_limited_ = false;
    bool control_output_initialized_ = false;

    // 现有DFCV桥在非AD模式会把tracking error精确置零。V8可据此推断控制权
    // 重新建立；推荐后续由桥直接发布driving_mode替代该兼容检测。
    bool infer_manual_mode_from_zero_tracking_error_ = true;
    bool use_driving_mode_topic_ = true;
    std::string driving_mode_topic_ = "/dfcv_bridge/driving_mode";
    int autonomous_driving_mode_value_ = 2;
    bool driving_mode_received_ = false;
    int latest_driving_mode_ = 0;
    ros::Time driving_mode_stamp_;
    double driving_mode_timeout_s_ = 0.5;
    double manual_mode_zero_error_epsilon_m_ = 1e-9;
    int manual_mode_confirm_cycles_ = 3;
    int autonomous_mode_confirm_cycles_ = 2;
    int zero_tracking_error_streak_ = 0;
    int nonzero_tracking_error_streak_ = 0;
    bool inferred_manual_mode_ = false;
    bool autonomy_reentry_detected_ = false;

    // ESO观测器相关
    double eso_x1_;
    double eso_x2_;
    double model_r_comp_;//模型计算横摆角速度
    bool model_comp_initialized_;//是否初始化

    //输出模型计算量
    double Model_r1_;

    // UKF相关
    Eigen::Vector2d ukf_x_est_;
    Eigen::Matrix2d ukf_P_est_;

    // 标定特性
    double const_steer_bias_;

    // 横坡补偿
    bool use_slope_compensation_;
    double ay_slope_compensation_;
    double slope_compensation_coeff_;
    double slope_compensation_filter_tau_;
    bool ay_slope_compensation_initialized_;
    std::string slope_estimator_mode_;        // legacy_quasistatic 或 tire_force_residual
    bool slope_dynamic_gate_enabled_;
    double slope_gate_max_yaw_accel_;
    double slope_gate_max_steer_rate_;
    double slope_compensation_limit_;
    bool slope_gate_active_;
    bool slope_prev_valid_;
    double slope_prev_r_;
    double slope_prev_delta_;

    // UKF 与 NMPC 共用已估横向扰动，避免同一 ay 残差被重复解释为 vy。
    bool ukf_use_slope_disturbance_;
    double ukf_q_vy_;
    double ukf_q_r_;
    double ukf_r_ay_;
    double ukf_r_r_;
    double ukf_ay_innovation_limit_;
    double ukf_vy_abs_max_;
    double ukf_ay_innovation_raw_;
    double ukf_ay_innovation_used_;

    // 迭代时间
    double iter_time_ = 0.0;
    double total_control_time_ = 0.0;      // 整个控制周期耗时，单位ms

    // 侧向加速度零偏补偿
    bool use_ay_bias_compensation_;          // 总开关：false 时完全不做 ay 零偏补偿
    double const_ay_bias_;                  // 静态 ay 零偏初值/固定值，单位 m/s^2
    bool use_dynamic_ay_compensation_;      // true 时基于横向误差窗口进行准静态增量修正
    double ay_bias_estimate_;               // 当前 ay 零偏估计，动态模式下围绕 const_ay_bias_ 迭代
    double effective_ay_bias_;              // 本周期实际用于横坡补偿的 ay 零偏
    int dynamic_ay_error_window_size_;      // 横向误差滑动窗口长度，20Hz*10s 默认 200
    double dynamic_ay_error_threshold_;     // 窗口平均横向误差死区，单位 m
    double dynamic_ay_bias_learning_rate_;  // 每周期每米横向误差对应的 ay 偏置修正量
    double dynamic_ay_bias_max_step_;       // 单周期最大 ay 偏置修正量，防止过渡修正
    double dynamic_ay_bias_min_;            // ay 偏置估计下限
    double dynamic_ay_bias_max_;            // ay 偏置估计上限
    double dynamic_ay_bias_error_sign_;     // 横向误差均值到 ay 偏置修正方向的符号
    bool dynamic_ay_require_full_window_;   // true 时窗口填满后才允许更新
    std::deque<double> lateral_error_history_;

    bool auto_update_total_weight_; // 是否使用 EBS 整车质量更新插值输入
    double received_mass_ = 10000.0; // EBS接收到的整车质量，单位 kg；同时作为质量插值自变量

    // 在类中添加以下成员变量
    std::deque<double> pp_cmd_queue_;
    double control_time_;
    double control_delay_sec_;

    // 本地 CSV 诊断日志
    bool enable_local_log_ = false;
    std::string local_log_directory_ = "/tmp/eso_tracker_logs";
    int local_log_flush_interval_ = 100;
    int local_log_pending_rows_ = 0;
    std::string local_log_path_;
    std::ofstream local_log_stream_;

    // 分层归因日志的上一周期测量值，只用于计算原始差分/一步预测残差。
    bool diagnostic_prev_valid_ = false;
    double diagnostic_prev_delta_ = 0.0;
    double diagnostic_prev_vx_ = 0.0;
    double diagnostic_prev_vy_status_ = 0.0;
    double diagnostic_prev_r_ = 0.0;
    double diagnostic_prev_vy_ = 0.0;
    double diagnostic_prev_tracking_error_ = 0.0;
    double diagnostic_prev_geometric_error_ = 0.0;
    double diagnostic_prev_final_cmd_ = 0.0;
    double diagnostic_prev2_final_cmd_ = 0.0;
    bool diagnostic_prev2_cmd_valid_ = false;
    double diagnostic_prev_cmd_rate_ = 0.0;
    bool diagnostic_prev_cmd_rate_valid_ = false;
    int diagnostic_prev_nearest_idx_ = -1;

    // 车辆状态可能以保持值重复发布；重复测量只做观测器预测，不重复做测量校正。
    bool measurement_fingerprint_valid_ = false;
    double measurement_prev_x_ = 0.0;
    double measurement_prev_y_ = 0.0;
    double measurement_prev_yaw_ = 0.0;
    double measurement_prev_vx_ = 0.0;
    double measurement_prev_vy_ = 0.0;
    double measurement_prev_r_ = 0.0;
    double measurement_prev_delta_ = 0.0;
    double measurement_prev_ay_ = 0.0;
    bool last_measurement_is_new_ = true;
    bool last_observer_low_speed_reset_ = false;
    double observer_dynamic_min_speed_mps_ = 4.0;

    // NMPC 求解结果抽样：k=1、k=min(5,N)、k=N；状态顺序为 x,y,theta,vy,r,delta。
    std::array<double, 6> diagnostic_pred_k1_;
    std::array<double, 6> diagnostic_pred_k5_;
    std::array<double, 6> diagnostic_pred_kN_;
    std::array<double, 3> diagnostic_u_sparse_;

    // NMPC求解以及算法切换相关
    bool mpc_failure_flag_;
    bool using_pure_pursuit_flag_;
    bool require_over_take_flag_;
    bool using_mixed_mode_flag_;

    int mpc_failure_count_ = 0; // NMPC连续失败计数器
    int require_overtake_times_; // 连续要求超车次数阈值，超过该值则提示要求人工接管

};

} // namespace race_tracker

#endif // RACE_TRACKER_ESO_TRACKER_H
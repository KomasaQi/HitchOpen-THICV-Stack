#ifndef RACE_TRACKER_ESO_TRACKER2_H
#define RACE_TRACKER_ESO_TRACKER2_H

#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <array>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "race_tracker/controller_plugin_base.h"
#include <race_msgs/Control.h>
#include <race_msgs/ESOEstimation.h>
#include <race_msgs/Flag.h>
#include <race_msgs/Path.h>
#include <race_msgs/VehicleStatus.h>
#include <std_msgs/Int32.h>

namespace race_tracker {

// 与 ESOTracker 对齐的监督层参数。三自由度和 CKF 的差异保留在 ESOTracker2 内部。
struct SupervisorParams {
    double startup_time;
    double blend_speed_low;
    double blend_speed_high;
    double standstill_speed;
    double nmpc_speed_floor;
};

struct NMPCParams {
    // 牵引车参数
    double m;
    double Iz;
    double lf;
    double lr;
    double Cf;
    double Cr;

    // 挂车参数。m_t_total 始终表示“挂车及其载荷”的质量，不含牵引车。
    double m_t;
    double Iz_t;
    double m_t_total;
    double lh;
    double lt;
    double L2;
    double Ct;
    double M;

    double delta_rate_max;
    double delta_rate_min;
    double T_lag;
    double dt;
    int N;
    int Nc;
    int nx;
    int nu;
    double delta_max;
    double delta_min;

    // [x,y,theta,vy,r,delta,r_t,gamma]
    Eigen::Matrix<double, 8, 8> Q = Eigen::Matrix<double, 8, 8>::Zero();
    double dgamma;
    double R;
    double dR;
    double Kiz;

    double integration_grade;
    double eso_disturbance_decay;
    double eso_disturbance_tau_s;
    int near_dense_control_steps;
};

struct NMPSolver {
    casadi::Opti opti;
    casadi::MX X;
    casadi::MX U_sparse;
    casadi::MX P_x0;
    // [x_ref,y_ref,theta_ref,kappa_ref,delta_ff,vy_ref,r_t_ref,gamma_ref]
    casadi::MX P_waypoints;
    casadi::MX P_vx;
    casadi::MX P_u_prev;
    casadi::MX P_h_hat;
    casadi::MX P_dyn_params;
    casadi::MX U_full_feedback;
    casadi::MX U_full_command;
    std::vector<int> control_block_start;
    std::vector<int> control_block_length;
    std::unique_ptr<casadi::OptiSol> sol_prev;
    bool has_prev_sol = false;
};

class ESOTracker2 : public ControllerPluginBase {
public:
    ESOTracker2();
    ~ESOTracker2() override = default;

    bool initialize(ros::NodeHandle& nh) override;

    void computeControl(
        const race_msgs::VehicleStatusConstPtr& vehicle_status,
        const race_msgs::PathConstPtr& path,
        race_msgs::Control* control_msg,
        const double dt,
        const race_msgs::Flag::ConstPtr& flag) override;

    std::string getName() const override { return "ESOTracker2"; }

private:
    void buildNMPSolver();
    casadi::MX vehicleDynamicsModel(const casadi::MX& state,
                                    const casadi::MX& cmd_delta,
                                    const casadi::MX& vx,
                                    const casadi::MX& h_dist,
                                    const casadi::MX& dyn_params);
    bool solveNMPC(const std::vector<double>& current_state,
                   const casadi::DM& waypoints,
                   std::vector<double>& control_output);
    void captureNmpcSolverStats();
    void enterFallback(int reason_code, const ros::Time& now);
    void drivingModeCallback(const std_msgs::Int32::ConstPtr& msg);

    // CKF 联合估计状态保持为 [vy,r,r_t,gamma]。
    void ckfEstimate(double curr_vx, double curr_delta, double curr_ay,
                     double curr_r, double pseudo_r2, double pseudo_gamma,
                     double dt, bool measurement_is_new);
    Eigen::Vector4d compute_dynamics_3dof(const Eigen::Vector4d& x,
                                          const Eigen::Vector2d& u);
    Eigen::Vector4d vehicle_state_3dof(const Eigen::Vector4d& x,
                                       const Eigen::Vector2d& u,
                                       double dt);
    Eigen::Vector4d vehicle_meas_3dof(const Eigen::Vector4d& x,
                                      const Eigen::Vector2d& u);

    void rlsIdentifyStiffness(double curr_vx, double vy_est, double curr_delta,
                              double curr_r, double curr_ay, double curr_gamma,
                              double curr_r_t, double trailer_mass, double dt);
    void esoCompute(double curr_vy, double curr_r, double curr_delta,
                    double curr_r_t, double curr_gamma, double curr_vx,
                    double dt, bool measurement_is_new);
    double calcNominalYawAccel(double curr_vy, double curr_r, double curr_delta,
                               double curr_r_t, double curr_gamma, double curr_vx);

    double normalizeAngle(double angle);
    double quaternion_to_yaw(const geometry_msgs::Quaternion& q);
    int find_nearest_path_point(double x0, double y0, double yaw0,
                                const race_msgs::Path& path);
    std::vector<double> calculate_cumulative_distance(const race_msgs::Path& path,
                                                       int start_idx);
    std::vector<double> linear_interpolate(const std::vector<double>& s_original,
                                           const std::vector<double>& val_original,
                                           const std::vector<double>& s_target);
    casadi::DM interpolate_path_segment(const race_msgs::Path& path,
                                        const std::vector<double>& cum_dist,
                                        int start_idx, int end_idx,
                                        const std::vector<double>& s_target,
                                        double yaw0);
    casadi::DM process_race_path(const race_msgs::Path& input_path,
                                 const std::vector<double>& current_state);
    std::array<double, 4> computeSteadyStateReference(double vx,
                                                      double kappa) const;
    double computePurePursuitSteering(const race_msgs::Path& path,
                                      double curr_x, double curr_y,
                                      double curr_theta, double lookahead_dist);
    void calculate_trailer_kinematics(double curr_vx, double curr_r, double dt);
    bool isNewVehicleMeasurement(double x, double y, double yaw, double vx,
                                 double vy, double r, double delta, double ay);

private:
    ros::Publisher est_pub_;
    ros::Subscriber driving_mode_sub_;
    NMPCParams nmpc_params_;
    NMPSolver solver_;
    SupervisorParams supervisor_params_;

    ros::Time start_time_;
    ros::Time last_control_time_;
    double current_cmd_ = 0.0;
    double blend_alpha_ = 0.0;
    double nmpc_safe_cmd_ = 0.0;
    bool control_output_initialized_ = false;

    double r_tractor_filt_ = 0.0;
    bool r_filter_initialized_ = false;

    // ESO
    double eso_x1_ = 0.0;
    double eso_x2_ = 0.0;
    bool eso_initialized_ = false;

    // CKF
    Eigen::Vector4d ckf_x_hat_;
    Eigen::Matrix4d ckf_P_;

    // FF-RLS
    double rls_w1_prev_ = 0.0;
    double rls_w2_prev_ = 0.0;
    Eigen::Matrix3d rls_P_;
    double rls_w1_dot_prev_ = 0.0;
    double rls_w2_dot_prev_ = 0.0;
    Eigen::Vector3d rls_C_out_prev_;
    double rls_Cf_est_ = 0.0;
    double rls_Cr_est_ = 0.0;
    double rls_Ct_est_ = 0.0;
    double rls_Cf_est_min_ = 0.0;
    double rls_Cr_est_min_ = 0.0;
    double rls_Ct_est_min_ = 0.0;
    double rls_Cf_est_max_ = 0.0;
    double rls_Cr_est_max_ = 0.0;
    double rls_Ct_est_max_ = 0.0;
    double rls_Cf_est_default_ = 0.0;
    double rls_Cr_est_default_ = 0.0;
    double rls_Ct_est_default_ = 0.0;

    double gamma_ = 0.0;
    double r_t_ = 0.0;
    double m_t_empty_default_ = 0.0;

    // IPOPT 与实时截止
    int ipopt_max_iter_ = 50;
    double ipopt_acceptable_tol_ = 5e-2;
    int ipopt_acceptable_iter_ = 3;
    double ipopt_warm_start_bound_push_ = 1e-3;
    double ipopt_warm_start_slack_bound_push_ = 1e-3;
    double ipopt_warm_start_mult_bound_push_ = 1e-3;
    double nmpc_solve_deadline_ms_ = 50.0;
    double nmpc_ipopt_cpu_time_limit_ms_ = 45.0;
    bool last_nmpc_deadline_missed_ = false;
    unsigned long long nmpc_timeout_count_ = 0;
    bool last_nmpc_attempted_ = false;
    bool last_nmpc_solver_returned_success_ = false;
    bool last_nmpc_warm_start_used_ = false;
    int last_nmpc_status_code_ = 0;
    int last_nmpc_iter_count_ = -1;
    std::string last_nmpc_return_status_ = "not_attempted";

    // 路径与前馈
    double min_lookahead_distance_ = 6.0;
    double lookahead_speed_coeff_ = 0.7;
    double lookahead_curvature_coeff_ = 0.0;
    double curvature_smoothing_distance_m_ = 6.0;
    bool use_geometric_path_heading_ = true;
    double geometric_heading_window_m_ = 2.0;
    double path_projection_heading_weight_m2_ = 4.0;
    double path_projection_heading_gate_rad_ = 1.2;
    double path_projection_rear_gate_m_ = 5.0;
    bool use_equilibrium_feedforward_ = true;
    double equilibrium_feedforward_gain_ = 1.0;
    double equilibrium_feedforward_limit_ = 0.45;
    double last_delta_ff_ = 0.0;

    // PP 延迟、输出低通和最终速率保护
    std::deque<double> pp_cmd_queue_;
    double control_time_ = 0.05;
    double control_delay_sec_ = 0.0;
    double output_lpf_tau_ = 0.0;
    double final_cmd_filt_ = 0.0;
    bool final_cmd_filt_init_ = false;
    bool enforce_final_output_rate_limit_ = true;
    bool publish_steering_angle_velocity_ = true;
    double steering_angle_velocity_cmd_radps_ = 0.35;
    bool last_final_output_rate_limited_ = false;

    // 锁存式 fallback 与高偏差启动恢复
    bool fallback_latched_ = false;
    bool fallback_reentry_active_ = false;
    int fallback_reason_code_ = 0;
    ros::Time fallback_enter_time_;
    int nmpc_success_streak_ = 0;
    double fallback_reentry_alpha_ = 0.0;
    double fallback_min_hold_s_ = 0.5;
    int fallback_required_successes_ = 8;
    double fallback_reentry_blend_time_s_ = 0.5;
    double fallback_reentry_max_lateral_error_m_ = 0.60;
    double fallback_reentry_max_heading_error_rad_ = 0.15;
    double fallback_reentry_max_yaw_rate_radps_ = 0.25;
    double fallback_reentry_max_kappa_step_1pm_ = 0.003;
    int fallback_reentry_max_nearest_index_jump_ = 5;
    double nmpc_attempt_min_speed_mps_ = 3.0;
    int reference_stable_streak_ = 0;
    bool last_reference_kappa_valid_ = false;
    double last_reference_kappa_ = 0.0;
    double reference_kappa_step_ = 0.0;
    bool reference_stable_this_cycle_ = false;
    int reference_prev_nearest_idx_ = -1;

    bool startup_recovery_checked_ = false;
    bool startup_recovery_enabled_ = true;
    bool startup_recovery_active_ = false;
    int startup_recovery_alignment_streak_ = 0;
    double startup_recovery_entry_lateral_error_m_ = 1.0;
    double startup_recovery_entry_heading_error_rad_ = 0.25;
    double startup_recovery_exit_lateral_error_m_ = 0.35;
    double startup_recovery_exit_heading_error_rad_ = 0.10;
    double startup_recovery_exit_yaw_rate_radps_ = 0.15;
    int startup_recovery_exit_cycles_ = 10;
    double startup_recovery_min_lookahead_m_ = 12.0;
    double startup_recovery_lookahead_error_gain_ = 2.0;
    double startup_recovery_max_steer_rad_ = 0.25;
    double startup_recovery_stationary_hold_speed_mps_ = 0.50;

    // 人工/自动驾驶切换
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

    // 重复测量和低速估计保护
    bool measurement_fingerprint_valid_ = false;
    double measurement_prev_x_ = 0.0;
    double measurement_prev_y_ = 0.0;
    double measurement_prev_yaw_ = 0.0;
    double measurement_prev_vx_ = 0.0;
    double measurement_prev_vy_ = 0.0;
    double measurement_prev_r_ = 0.0;
    double measurement_prev_delta_ = 0.0;
    double measurement_prev_ay_ = 0.0;
    double observer_dynamic_min_speed_mps_ = 4.0;

    // 诊断/上层状态标志
    double iter_time_ = 0.0;
    double total_control_time_ = 0.0;
    double model_r2_ = 0.0;
    int mpc_failure_count_ = 0;
    int degrade_failure_times_ = 3;
    int require_overtake_times_ = 10;
    bool mpc_failure_flag_ = false;
    bool using_pure_pursuit_flag_ = false;
    bool require_over_take_flag_ = false;
    bool using_mixed_mode_flag_ = false;
};

} // namespace race_tracker

#endif // RACE_TRACKER_ESO_TRACKER2_H

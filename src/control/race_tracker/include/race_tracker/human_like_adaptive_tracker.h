#ifndef RACE_TRACKER_HUMAN_LIKE_ADAPTIVE_TRACKER_H
#define RACE_TRACKER_HUMAN_LIKE_ADAPTIVE_TRACKER_H

#include "race_tracker/controller_plugin_base.h"

#include <geometry_msgs/Point.h>

#include <array>
#include <cstddef>
#include <deque>
#include <fstream>
#include <string>
#include <vector>

namespace race_tracker {

/**
 * @brief V0.14 连续滑模与一阶动态补偿横向控制器。
 *
 * 主控制使用低维、非抖振结构：
 *
 * 1. 把横向误差、航向误差和参考曲率统一预测到执行器响应时刻；
 * 2. 以空间域滑模面约束误差收敛，天然适配车速变化；
 * 3. 用tanh边界层代替sign切换，避免滑模控制的高频抖振；
 * 4. 采用有界、泄漏、可回退的连续积分项消除弯道稳态偏差；
 * 5. 显式使用“纯时延 + 执行器一阶动态 + 随车速变快的横向动态”；
 * 6. 前轮转角输出再经过连续速率/加速度约束，不允许锯齿命令；
 * 7. 旧在线增益学习、候选试验、走廊恢复和CasADi主写入全部关闭；
 * 8. 自动CSV记录滑模面的每个组成量，便于闭环复核。
 */
class HumanLikeAdaptiveTracker : public ControllerPluginBase {
public:
    HumanLikeAdaptiveTracker();
    ~HumanLikeAdaptiveTracker() override = default;

    bool initialize(ros::NodeHandle& nh) override;

    void computeControl(
        const race_msgs::VehicleStatusConstPtr& vehicle_status,
        const race_msgs::PathConstPtr& path,
        race_msgs::Control* control_msg,
        const double dt,
        const race_msgs::Flag::ConstPtr& flag) override;

    std::string getName() const override {
        return "HumanLikeAdaptiveTracker";
    }

private:
    void computeControlLegacyV12(
        const race_msgs::VehicleStatusConstPtr& vehicle_status,
        const race_msgs::PathConstPtr& path,
        race_msgs::Control* control_msg,
        double dt,
        const race_msgs::Flag::ConstPtr& flag);

    struct PathGeometry {
        std::vector<geometry_msgs::Point> points;
        std::vector<double> arc_lengths;
        double closest_arc_length = 0.0;
        geometry_msgs::Point closest_point;
        double lateral_error = 0.0;
        double heading_error = 0.0;
        double total_length = 0.0;
    };

    struct CommandSample {
        double time = 0.0;
        double commanded_curvature = 0.0;
        double preview_reference_curvature = 0.0;
        double speed = 0.0;
        double lateral_error = 0.0;
        double heading_error = 0.0;
    };

    struct FixedControlModel {
        double response_gain = 1.0;
        double response_time = 0.25;
        double delay = 0.20;
    };

    struct PathDifferential {
        double heading = 0.0;
        double curvature = 0.0;
        bool valid = false;
    };

    struct BaselineControl {
        double target_steering = 0.0;
        double feedforward_steering = 0.0;
        double feedback_steering = 0.0;
        double feedback_curvature = 0.0;
        double predicted_heading_error = 0.0;
        double convergence_distance = 0.0;
    };

    struct SlidingModeControl {
        double target_steering = 0.0;
        double predicted_lateral_error = 0.0;
        double predicted_heading_error = 0.0;
        double preview_reference_curvature = 0.0;
        double lateral_response_time = 0.0;
        double prediction_time = 0.0;
        double convergence_length = 0.0;
        double surface = 0.0;
        double smooth_switch = 0.0;
        double linear_feedback_curvature = 0.0;
        double switching_curvature = 0.0;
        double integral_curvature = 0.0;
        double desired_vehicle_curvature = 0.0;
        double dynamically_compensated_curvature = 0.0;
    };

    struct ShadowValidationSample {
        double speed_coordinate = 0.0;
        double log_feedforward_ratio = 0.0;
    };

    struct PerformanceCell {
        double certified_score = 0.0;
        double observation_time = 0.0;
    };

    bool buildPathGeometry(
        const race_msgs::PathConstPtr& path,
        const geometry_msgs::Pose& vehicle_pose,
        PathGeometry* geometry) const;

    geometry_msgs::Point transformToVehicleFrame(
        const geometry_msgs::Point& world_point,
        const geometry_msgs::Pose& vehicle_pose) const;

    geometry_msgs::Point samplePointAtArcLength(
        const PathGeometry& geometry,
        double arc_length) const;

    double estimateHeading(
        const PathGeometry& geometry,
        double arc_length) const;

    double estimateCurvature(
        const PathGeometry& geometry,
        double arc_length) const;

    PathDifferential fitPathDifferential(
        const PathGeometry& geometry,
        double arc_length,
        double half_window) const;

    double estimateSmoothedCurvature(
        const PathGeometry& geometry,
        double arc_length,
        double speed) const;

    std::vector<double> buildReferenceCurvatureSequence(
        const PathGeometry& geometry,
        double speed) const;

    std::vector<double> buildDesiredActualCurvatureSequence(
        double speed,
        const std::vector<double>& reference_curvature,
        double feedback_curvature,
        double convergence_distance) const;

    double robustPreviewCurvature(
        const std::vector<double>& reference_curvature,
        std::size_t preview_index) const;

    double filterPreviewCurvature(
        double preview_curvature,
        double dt);

    BaselineControl computeDecoupledBaseline(
        double lateral_error,
        double heading_error,
        double measured_curvature,
        double speed,
        double current_reference_curvature,
        double preview_reference_curvature,
        const FixedControlModel& model) const;

    SlidingModeControl computeSmoothSlidingModeControl(
        double lateral_error,
        double heading_error,
        double measured_curvature,
        double speed,
        double current_reference_curvature,
        double preview_reference_curvature,
        const FixedControlModel& model,
        double dt);

    double lateralResponseTime(double speed) const;

    double normalizedSpeed(double speed) const;

    double priorGainAtSpeed(double speed) const;

    FixedControlModel fixedControlModel(double speed) const;

    std::size_t previewIndex(
        double speed,
        const FixedControlModel& model,
        std::size_t sequence_size) const;

    void initializeShadowLearner();

    void updateTrackingSafety(
        double lateral_error,
        double heading_error,
        double speed,
        double dt);

    void updateShadowLearner(
        double speed,
        double longitudinal_acceleration,
        double lateral_error,
        double heading_error,
        double dt);

    void updateShadowCandidate();

    double validationRmse(
        double beta_scale,
        double beta_speed) const;

    void updateTrackingPerformance(
        double lateral_error,
        double heading_error,
        double speed,
        double reference_curvature,
        double dt);

    std::size_t performanceCellIndex(
        double speed,
        double absolute_curvature) const;

    void tryStartTrial(
        double speed,
        double reference_curvature);

    void acceptTrial();

    void rejectTrial(const char* reason);

    double computeCertifiedLearningResidual(
        double preview_reference_curvature,
        double speed,
        double baseline_target);

    bool lookupCommandSampleAtTime(
        double target_time,
        CommandSample* sample) const;

    bool lookupCommandAtTime(
        double target_time,
        double* commanded_curvature) const;

    std::vector<double> buildPredictionCommandHistory() const;

    void appendCommandHistory(
        double commanded_curvature,
        double preview_reference_curvature,
        double speed,
        double lateral_error,
        double heading_error);

    double filterYawRate(double yaw_rate, double dt);

    double filterMeasuredCurvature(double measured_curvature, double dt);

    void updateResponseGainObserver(
        double speed,
        double measured_curvature,
        double lateral_error,
        double heading_error,
        double dt);

    double applyCorridorRecoverySupervisor(
        double lateral_error,
        double heading_error,
        double measured_curvature,
        double speed,
        double current_reference_curvature,
        const FixedControlModel& model,
        const BaselineControl& baseline,
        double dt);

    void writeDiagnosticCsv(
        double speed,
        const PathGeometry& geometry,
        double current_reference_curvature,
        double raw_preview_curvature,
        double preview_reference_curvature,
        double measured_curvature,
        const BaselineControl& baseline,
        const FixedControlModel& model,
        double baseline_target,
        double supervised_target,
        double steering_command,
        double dt);

    void writeSlidingModeCsv(
        double speed,
        const PathGeometry& geometry,
        double current_reference_curvature,
        double measured_curvature,
        const FixedControlModel& model,
        const SlidingModeControl& sliding,
        double steering_command,
        double dt);

    bool buildPredictiveSolver();

    bool solvePredictiveControl(
        double measured_curvature,
        const std::vector<double>& desired_actual_curvature,
        const FixedControlModel& model,
        double* target_steering);

    double shapeSteeringCommand(double target_steering, double dt);

    double sanitizeControlDt(double dt) const;

    void resetTransientStates(bool reset_learning);

    void writeZeroSteering(
        race_msgs::Control* control_msg,
        bool reset_transient_state);

    static double normalizeAngle(double angle);

    static double clampValue(
        double value,
        double lower,
        double upper);

private:
    // 车辆已知量和执行器硬约束
    double wheelbase_;
    double max_steering_angle_;
    double max_steering_rate_;
    double max_steering_acceleration_;

    // 性能规格
    double allowed_lateral_error_;
    double minimum_control_speed_;

    // 参考轨迹几何
    int min_path_points_;
    double minimum_path_point_spacing_;
    double curvature_smoothing_distance_;

    // 固定模型预测控制；默认关闭，固定解析基线不依赖CasADi求解结果
    bool enable_predictive_control_;
    int prediction_horizon_steps_;
    double prediction_dt_;
    int solver_max_iterations_;
    double solver_max_cpu_time_;

    // 固定、不可被学习器修改的主控制先验
    double prior_response_gain_;
    double prior_response_time_;
    double prior_delay_;

    // V0.14一阶动态与连续滑模的少量物理/归一化先验。
    double actuator_time_constant_;
    double lateral_response_time_at_reference_speed_;
    double lateral_response_reference_speed_;
    double sliding_minimum_convergence_length_;
    double sliding_boundary_layer_;
    double sliding_reaching_curvature_;
    double sliding_integral_rate_;
    double sliding_integral_base_limit_;
    double sliding_integral_curve_ratio_;
    double sliding_integral_leak_time_;

    // 旧版安全影子学习（V0.7默认关闭，仅保留接口兼容）
    bool enable_shadow_learning_;
    bool enable_certified_learning_residual_;
    double learning_soft_error_;
    double learning_hard_error_;

    // 信号处理与离散安全
    double yaw_rate_filter_time_constant_;
    double curvature_filter_time_constant_;
    double default_control_dt_;
    double max_control_dt_;

    // 调试
    bool enable_debug_log_;
    double debug_log_period_;
    bool enable_csv_log_;
    std::string csv_log_path_;
    double csv_log_period_;

    // 旧预测控制接口保留为关闭状态，V0.14主控制不依赖CasADi。
    bool predictive_solver_initialized_;
    int maximum_prediction_delay_steps_;
    bool has_previous_solution_;

    // 示范学习充分统计量：
    // log(required_ff/prior_ff)=beta_scale+beta_speed*z(speed)
    double shadow_normal_00_;
    double shadow_normal_01_;
    double shadow_normal_11_;
    double shadow_rhs_0_;
    double shadow_rhs_1_;
    double candidate_beta_scale_;
    double candidate_beta_speed_;
    double certified_beta_scale_;
    double certified_beta_speed_;
    double trial_beta_scale_;
    double trial_beta_speed_;
    double candidate_validation_rmse_;
    double prior_validation_rmse_;
    double shadow_confidence_;
    double shadow_sample_timer_;
    double promotion_hold_timer_;
    double trial_score_sum_;
    double trial_reference_score_sum_;
    double trial_reference_level_;
    double trial_active_time_;
    double trial_block_timer_;
    std::size_t trial_performance_cell_index_;
    std::size_t shadow_total_samples_;
    std::size_t shadow_training_samples_;
    std::size_t shadow_validation_samples_count_;
    std::size_t promotion_count_;
    std::size_t rollback_count_;
    std::deque<ShadowValidationSample> shadow_validation_buffer_;
    std::array<PerformanceCell, 6> performance_cells_;
    bool trial_active_;
    bool learning_rollback_latched_;
    bool learning_soft_suppressed_;
    double applied_learning_residual_;
    double certified_learning_residual_;
    double trial_learning_residual_;

    // 信号滤波和安全监督状态
    double controller_time_;
    bool yaw_rate_filter_initialized_;
    double filtered_yaw_rate_;
    bool curvature_filter_initialized_;
    double filtered_curvature_;
    bool preview_curvature_filter_initialized_;
    double filtered_preview_curvature_;
    // 单一车型响应尺度：左右弯共用，并乘在连续速度先验上。
    // 观测器只使用延迟后的稳态转向—横摆响应，不进行探索动作。
    double response_gain_scale_;
    double response_gain_confidence_;
    double response_gain_observation_hold_;
    double response_gain_last_observation_;
    bool response_gain_curvature_initialized_;
    double response_gain_previous_curvature_;
    bool speed_state_initialized_;
    double previous_speed_;
    bool lateral_error_state_initialized_;
    double previous_lateral_error_;
    double filtered_lateral_error_rate_;

    // V0.14连续滑模状态。该积分是有界控制状态，不跨启动学习，
    // 也不随圈数积累车辆参数。
    double sliding_integral_curvature_;
    bool sliding_reference_sign_initialized_;
    double sliding_previous_reference_sign_;
    bool corridor_recovery_active_;
    double corridor_recovery_blend_;
    double corridor_recovery_release_hold_;
    double corridor_recovery_target_;
    double corridor_predicted_error_;

    // 指令和求解状态
    std::deque<CommandSample> command_history_;
    bool steering_state_initialized_;
    double previous_steering_angle_;
    double previous_steering_rate_;
    double solver_time_since_update_;
    bool held_target_initialized_;
    double held_target_steering_;
    bool last_solver_succeeded_;
    int consecutive_solver_failures_;

    // 文件日志只做诊断，不参与控制。
    std::ofstream csv_log_stream_;
    double csv_log_timer_;
    double csv_flush_timer_;
};

}  // namespace race_tracker

#endif  // RACE_TRACKER_HUMAN_LIKE_ADAPTIVE_TRACKER_H

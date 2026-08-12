#include "race_tracker/human_like_adaptive_tracker.h"

#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iterator>
#include <limits>
#include <string>

namespace race_tracker {

namespace {

constexpr double kMinimumResponseGain = 0.20;
constexpr double kMaximumResponseGain = 2.00;

// 以下常数只服务于默认关闭的旧版影子学习兼容路径。V0.7主控制
// 自适应使用后文的单尺度响应观测器，不使用候选试探或认证残差。
constexpr double kShadowSamplePeriod = 0.10;
constexpr double kShadowMinimumReferenceCurvature = 0.0015;
constexpr double kShadowMaximumCommandRate = 0.0060;
constexpr double kShadowMaximumReferenceRate = 0.0040;
constexpr double kShadowMaximumLongitudinalAcceleration = 0.75;
constexpr double kShadowMaximumHeadingError = 0.025;
constexpr double kShadowMaximumLateralErrorRate = 0.10;
constexpr double kShadowHuberLogResidual = 0.20;
constexpr double kShadowScaleRegularization = 20.0;
constexpr double kShadowSpeedRegularization = 120.0;
constexpr std::size_t kShadowMinimumTrainingSamples = 40;
constexpr std::size_t kShadowMinimumValidationSamples = 10;
constexpr std::size_t kShadowMaximumValidationSamples = 200;
constexpr double kShadowRequiredRmseRatio = 0.85;
constexpr double kShadowMaximumCandidateRmse = 0.25;
constexpr double kShadowMaximumScaleMagnitude = 0.70;
constexpr double kShadowMaximumSpeedMagnitude = 0.25;
constexpr double kTrialParameterStep = 0.04;
constexpr double kTrialQualificationHoldTime = 2.0;
constexpr double kPostPromotionObservationTime = 6.0;
constexpr double kTrialWarmupTime = 4.0;
constexpr double kTrialEvaluationTime = 4.0;
constexpr double kPerformanceMinimumObservationTime = 6.0;
constexpr double kPerformanceFilterTimeConstant = 8.0;
constexpr double kTrialMaximumScoreRatio = 1.05;
constexpr double kTrialMaximumScoreAllowance = 0.01;
constexpr double kInitialLearningResidualLimit = 0.008;
constexpr double kLearningResidualGrowthPerPromotion = 0.006;
constexpr double kMaximumLearningResidual = 0.050;
constexpr double kInitialRelativeLearningLimit = 0.06;
constexpr double kRelativeLearningGrowthPerPromotion = 0.04;
constexpr double kMaximumRelativeLearningResidual = 0.30;
constexpr double kMaximumTrialResidualIncrement = 0.008;

bool solveFourBySix(double matrix[4][6]) {
    for (int column = 0;
         column < 4;
         ++column) {
        int pivot = column;
        double pivot_magnitude =
            std::fabs(matrix[pivot][column]);
        for (int row = column + 1;
             row < 4;
             ++row) {
            const double magnitude =
                std::fabs(matrix[row][column]);
            if (magnitude > pivot_magnitude) {
                pivot = row;
                pivot_magnitude = magnitude;
            }
        }
        if (!std::isfinite(pivot_magnitude) ||
            pivot_magnitude < 1.0e-10) {
            return false;
        }
        if (pivot != column) {
            for (int entry = column;
                 entry < 6;
                 ++entry) {
                std::swap(
                    matrix[pivot][entry],
                    matrix[column][entry]);
            }
        }

        const double divisor =
            matrix[column][column];
        for (int entry = column;
             entry < 6;
             ++entry) {
            matrix[column][entry] /= divisor;
        }
        for (int row = 0;
             row < 4;
             ++row) {
            if (row == column) {
                continue;
            }
            const double factor =
                matrix[row][column];
            for (int entry = column;
                 entry < 6;
                 ++entry) {
                matrix[row][entry] -=
                    factor *
                    matrix[column][entry];
            }
        }
    }
    return true;
}

}  // namespace

HumanLikeAdaptiveTracker::HumanLikeAdaptiveTracker()
    : wheelbase_(4.135),
      max_steering_angle_(0.40),
      max_steering_rate_(0.60),
      max_steering_acceleration_(2.50),
      allowed_lateral_error_(0.10),
      minimum_control_speed_(1.0),
      min_path_points_(5),
      minimum_path_point_spacing_(0.02),
      curvature_smoothing_distance_(2.5),
      enable_predictive_control_(false),
      prediction_horizon_steps_(15),
      prediction_dt_(0.10),
      solver_max_iterations_(40),
      solver_max_cpu_time_(0.04),
      prior_response_gain_(0.50),
      prior_response_time_(0.25),
      prior_delay_(0.20),
      actuator_time_constant_(0.25),
      lateral_response_time_at_reference_speed_(0.22),
      lateral_response_reference_speed_(10.0),
      sliding_minimum_convergence_length_(7.50),
      sliding_boundary_layer_(0.022),
      sliding_reaching_curvature_(0.0090),
      sliding_integral_rate_(0.0022),
      sliding_integral_base_limit_(0.0045),
      sliding_integral_curve_ratio_(0.28),
      sliding_integral_leak_time_(18.0),
      enable_shadow_learning_(false),
      enable_certified_learning_residual_(false),
      learning_soft_error_(0.15),
      learning_hard_error_(0.35),
      yaw_rate_filter_time_constant_(0.06),
      curvature_filter_time_constant_(0.08),
      default_control_dt_(0.02),
      max_control_dt_(0.10),
      enable_debug_log_(true),
      debug_log_period_(0.50),
      enable_csv_log_(true),
      csv_log_path_(
          "/tmp/human_like_adaptive_tracker_v14.csv"),
      csv_log_period_(0.10),
      predictive_solver_initialized_(false),
      maximum_prediction_delay_steps_(0),
      has_previous_solution_(false),
      shadow_normal_00_(kShadowScaleRegularization),
      shadow_normal_01_(0.0),
      shadow_normal_11_(kShadowSpeedRegularization),
      shadow_rhs_0_(0.0),
      shadow_rhs_1_(0.0),
      candidate_beta_scale_(0.0),
      candidate_beta_speed_(0.0),
      certified_beta_scale_(0.0),
      certified_beta_speed_(0.0),
      trial_beta_scale_(0.0),
      trial_beta_speed_(0.0),
      candidate_validation_rmse_(
          std::numeric_limits<double>::infinity()),
      prior_validation_rmse_(
          std::numeric_limits<double>::infinity()),
      shadow_confidence_(0.0),
      shadow_sample_timer_(0.0),
      promotion_hold_timer_(0.0),
      trial_score_sum_(0.0),
      trial_reference_score_sum_(0.0),
      trial_reference_level_(0.0),
      trial_active_time_(0.0),
      trial_block_timer_(0.0),
      trial_performance_cell_index_(0),
      shadow_total_samples_(0),
      shadow_training_samples_(0),
      shadow_validation_samples_count_(0),
      promotion_count_(0),
      rollback_count_(0),
      trial_active_(false),
      learning_rollback_latched_(false),
      learning_soft_suppressed_(false),
      applied_learning_residual_(0.0),
      certified_learning_residual_(0.0),
      trial_learning_residual_(0.0),
      controller_time_(0.0),
      yaw_rate_filter_initialized_(false),
      filtered_yaw_rate_(0.0),
      curvature_filter_initialized_(false),
      filtered_curvature_(0.0),
      preview_curvature_filter_initialized_(false),
      filtered_preview_curvature_(0.0),
      response_gain_scale_(1.0),
      response_gain_confidence_(0.0),
      response_gain_observation_hold_(0.0),
      response_gain_last_observation_(1.0),
      response_gain_curvature_initialized_(false),
      response_gain_previous_curvature_(0.0),
      speed_state_initialized_(false),
      previous_speed_(0.0),
      lateral_error_state_initialized_(false),
      previous_lateral_error_(0.0),
      filtered_lateral_error_rate_(0.0),
      sliding_integral_curvature_(0.0),
      sliding_reference_sign_initialized_(false),
      sliding_previous_reference_sign_(0.0),
      corridor_recovery_active_(false),
      corridor_recovery_blend_(0.0),
      corridor_recovery_release_hold_(0.0),
      corridor_recovery_target_(0.0),
      corridor_predicted_error_(0.0),
      steering_state_initialized_(false),
      previous_steering_angle_(0.0),
      previous_steering_rate_(0.0),
      solver_time_since_update_(0.10),
      held_target_initialized_(false),
      held_target_steering_(0.0),
      last_solver_succeeded_(false),
      consecutive_solver_failures_(0),
      csv_log_timer_(0.0),
      csv_flush_timer_(0.0) {}

bool HumanLikeAdaptiveTracker::initialize(ros::NodeHandle& nh) {
    ROS_INFO("[%s] 父NodeHandle命名空间: %s",
             getName().c_str(), nh.getNamespace().c_str());

    ros::NodeHandle nh_inner(nh, "human_like_adaptive_tracker");
    ROS_INFO("[%s] 插件专属NodeHandle命名空间: %s",
             getName().c_str(), nh_inner.getNamespace().c_str());

    // 只有物理约束、误差目标和三个通用先验需要出现在常用参数中。
    nh_inner.param("wheelbase", wheelbase_, 4.135);
    nh_inner.param("max_steering_angle", max_steering_angle_, 0.40);
    nh_inner.param("max_steering_rate", max_steering_rate_, 0.60);
    nh_inner.param("max_steering_acceleration",
                   max_steering_acceleration_, 2.50);
    nh_inner.param("allowed_lateral_error",
                   allowed_lateral_error_, 0.10);

    nh_inner.param("enable_predictive_control",
                   enable_predictive_control_, false);
    nh_inner.param("prediction_horizon_steps",
                   prediction_horizon_steps_, 15);
    nh_inner.param("prediction_dt", prediction_dt_, 0.10);
    nh_inner.param("solver_max_iterations",
                   solver_max_iterations_, 40);
    nh_inner.param("solver_max_cpu_time",
                   solver_max_cpu_time_, 0.04);

    nh_inner.param("prior_response_gain",
                   prior_response_gain_, 0.50);
    nh_inner.param("prior_response_time",
                   prior_response_time_, 0.25);
    nh_inner.param("prior_delay", prior_delay_, 0.20);
    nh_inner.param("actuator_time_constant",
                   actuator_time_constant_, prior_response_time_);
    nh_inner.param("lateral_response_time_at_10ms",
                   lateral_response_time_at_reference_speed_, 0.22);
    nh_inner.param("enable_shadow_learning",
                   enable_shadow_learning_, false);
    nh_inner.param("enable_certified_learning_residual",
                   enable_certified_learning_residual_, false);

    // 下列参数通常保持默认，只为不同消息频率和路径质量保留接口。
    nh_inner.param("minimum_control_speed",
                   minimum_control_speed_, 1.0);
    nh_inner.param("min_path_points", min_path_points_, 5);
    nh_inner.param("minimum_path_point_spacing",
                   minimum_path_point_spacing_, 0.02);
    nh_inner.param("curvature_smoothing_distance",
                   curvature_smoothing_distance_, 2.5);
    nh_inner.param("yaw_rate_filter_time_constant",
                   yaw_rate_filter_time_constant_, 0.06);
    nh_inner.param("curvature_filter_time_constant",
                   curvature_filter_time_constant_, 0.08);
    nh_inner.param("default_control_dt", default_control_dt_, 0.02);
    nh_inner.param("max_control_dt", max_control_dt_, 0.10);
    nh_inner.param("enable_debug_log", enable_debug_log_, true);
    nh_inner.param("debug_log_period", debug_log_period_, 0.50);
    nh_inner.param("enable_csv_log", enable_csv_log_, true);
    nh_inner.param(
        "csv_log_path",
        csv_log_path_,
        std::string(
            "/tmp/human_like_adaptive_tracker_v14.csv"));
    nh_inner.param("csv_log_period", csv_log_period_, 0.10);

    // V0.14只有连续滑模主控制写入车辆。防止旧launch中的true
    // 重新打开此前已经出现过越学越差/求解抖动的支路。
    enable_predictive_control_ = false;
    enable_shadow_learning_ = false;
    enable_certified_learning_residual_ = false;

    if (!std::isfinite(wheelbase_) || wheelbase_ < 1.0) {
        ROS_WARN("[%s] wheelbase无效，恢复为4.135m", getName().c_str());
        wheelbase_ = 4.135;
    }
    max_steering_angle_ =
        clampValue(max_steering_angle_, 0.05, 1.20);
    max_steering_rate_ =
        clampValue(max_steering_rate_, 0.05, 5.0);
    max_steering_acceleration_ =
        clampValue(max_steering_acceleration_, 0.10, 50.0);
    allowed_lateral_error_ =
        clampValue(allowed_lateral_error_, 0.03, 1.0);
    minimum_control_speed_ =
        clampValue(minimum_control_speed_, 0.1, 10.0);
    min_path_points_ = std::max(3, min_path_points_);
    minimum_path_point_spacing_ =
        clampValue(minimum_path_point_spacing_, 1.0e-4, 1.0);
    curvature_smoothing_distance_ =
        clampValue(curvature_smoothing_distance_, 0.5, 10.0);

    prediction_horizon_steps_ =
        std::max(8, std::min(prediction_horizon_steps_, 40));
    prediction_dt_ =
        clampValue(prediction_dt_, 0.04, 0.25);
    solver_max_iterations_ =
        std::max(10, std::min(solver_max_iterations_, 200));
    solver_max_cpu_time_ =
        clampValue(solver_max_cpu_time_, 0.005, 0.50);

    prior_response_gain_ =
        clampValue(prior_response_gain_,
                   kMinimumResponseGain,
                   kMaximumResponseGain);
    prior_response_time_ =
        clampValue(prior_response_time_, 0.05, 2.0);
    prior_delay_ = clampValue(prior_delay_, 0.0, 1.0);
    actuator_time_constant_ =
        clampValue(actuator_time_constant_, 0.05, 1.0);
    lateral_response_time_at_reference_speed_ =
        clampValue(
            lateral_response_time_at_reference_speed_,
            0.05,
            0.80);
    lateral_response_reference_speed_ = 10.0;

    // 滑模参数由轴距、误差目标和物理转角范围自动缩放，不加入
    // 常用YAML调参表。跨车型时只需核对轴距和两个时间常数。
    sliding_minimum_convergence_length_ =
        std::max(6.0, 1.80 * wheelbase_);
    sliding_boundary_layer_ =
        clampValue(
            0.010 +
                allowed_lateral_error_ /
                    sliding_minimum_convergence_length_,
            0.018,
            0.040);
    sliding_reaching_curvature_ =
        clampValue(
            std::tan(
                0.040 * max_steering_angle_) /
                wheelbase_,
            0.0020,
            0.0050);
    sliding_integral_rate_ =
        0.24 * sliding_reaching_curvature_;
    sliding_integral_base_limit_ =
        clampValue(
            std::tan(
                0.035 * max_steering_angle_) /
                wheelbase_,
            0.0015,
            0.0040);
    sliding_integral_curve_ratio_ = 0.28;
    sliding_integral_leak_time_ = 18.0;
    learning_soft_error_ =
        std::max(
            1.5 * allowed_lateral_error_,
            0.15);
    learning_hard_error_ =
        std::max(
            3.5 * allowed_lateral_error_,
            0.35);

    yaw_rate_filter_time_constant_ =
        clampValue(yaw_rate_filter_time_constant_, 0.0, 1.0);
    curvature_filter_time_constant_ =
        clampValue(curvature_filter_time_constant_, 0.0, 1.0);
    default_control_dt_ =
        clampValue(default_control_dt_, 1.0e-4, 0.20);
    max_control_dt_ =
        clampValue(max_control_dt_, default_control_dt_, 0.50);
    debug_log_period_ =
        clampValue(debug_log_period_, 0.10, 60.0);
    csv_log_period_ =
        clampValue(csv_log_period_, 0.02, 10.0);

    logParamLoad("wheelbase", wheelbase_, 4.135);
    logParamLoad("max_steering_angle", max_steering_angle_, 0.40);
    logParamLoad("max_steering_rate", max_steering_rate_, 0.60);
    logParamLoad("max_steering_acceleration",
                 max_steering_acceleration_, 2.50);
    logParamLoad("allowed_lateral_error",
                 allowed_lateral_error_, 0.10);
    logParamLoad("enable_predictive_control",
                 enable_predictive_control_, false);
    logParamLoad("prediction_horizon_steps",
                 prediction_horizon_steps_, 15);
    logParamLoad("prediction_dt", prediction_dt_, 0.10);
    logParamLoad("prior_response_gain",
                 prior_response_gain_, 0.50);
    logParamLoad("prior_response_time",
                 prior_response_time_, 0.25);
    logParamLoad("prior_delay", prior_delay_, 0.20);
    logParamLoad("actuator_time_constant",
                 actuator_time_constant_, 0.25);
    logParamLoad("lateral_response_time_at_10ms",
                 lateral_response_time_at_reference_speed_, 0.22);
    logParamLoad("enable_shadow_learning",
                 enable_shadow_learning_, false);
    logParamLoad("enable_certified_learning_residual",
                 enable_certified_learning_residual_, false);

    initializeShadowLearner();
    resetTransientStates(false);

    if (csv_log_stream_.is_open()) {
        csv_log_stream_.close();
    }
    if (enable_csv_log_) {
        csv_log_stream_.open(
            csv_log_path_.c_str(),
            std::ios::out |
                std::ios::trunc);
        if (!csv_log_stream_.is_open()) {
            ROS_ERROR(
                "[%s] V0.14无法创建诊断CSV: %s",
                getName().c_str(),
                csv_log_path_.c_str());
            enable_csv_log_ = false;
        } else {
            csv_log_stream_
                << "time,v,ey,ey_pred,eydot,epsi,epsi_pred,"
                << "kref0,kprev,kyaw,gmodel,tau_act,tau_lat,tpred,"
                << "lc,s,smooth,klinear,kswitch,kint,kdesired,kdynamic,"
                << "delta_target,delta_cmd,delta_rate\n";
            csv_log_stream_.flush();
            ROS_INFO(
                "[%s] V0.14诊断CSV: %s",
                getName().c_str(),
                csv_log_path_.c_str());
        }
    }

    predictive_solver_initialized_ =
        enable_predictive_control_ && buildPredictiveSolver();
    if (enable_predictive_control_ &&
        !predictive_solver_initialized_) {
        ROS_WARN("[%s] CasADi预测控制器初始化失败，将使用固定解析基线",
                 getName().c_str());
    }

    ROS_INFO(
        "[%s] V0.14初始化完成: smooth_smc=1, sign_switch=0, "
        "first_order=1, speed_lateral_tau=1, online_gain_write=0, csv=%d, "
        "prior(g=%.3f,delay=%.3f,tau_act=%.3f,tau_lat10=%.3f), "
        "smc(lc_min=%.2f,phi=%.4f,kreach=%.5f,kirate=%.5f)",
        getName().c_str(),
        enable_csv_log_ ? 1 : 0,
        prior_response_gain_,
        prior_delay_,
        actuator_time_constant_,
        lateral_response_time_at_reference_speed_,
        sliding_minimum_convergence_length_,
        sliding_boundary_layer_,
        sliding_reaching_curvature_,
        sliding_integral_rate_);
    return true;
}

void HumanLikeAdaptiveTracker::computeControl(
    const race_msgs::VehicleStatusConstPtr& vehicle_status,
    const race_msgs::PathConstPtr& path,
    race_msgs::Control* control_msg,
    const double dt,
    const race_msgs::Flag::ConstPtr& flag) {

    (void)flag;

    if (!control_msg) {
        ROS_ERROR("[%s] 控制指针为空", getName().c_str());
        return;
    }
    if (!vehicle_status || !path) {
        ROS_ERROR_THROTTLE(
            1.0,
            "[%s] 车辆状态或路径为空，输出0转角",
            getName().c_str());
        writeZeroSteering(control_msg, true);
        return;
    }
    if (path->points.size() <
        static_cast<std::size_t>(min_path_points_)) {
        ROS_WARN_THROTTLE(
            1.0,
            "[%s] 路径点不足（%zu < %d），输出0转角",
            getName().c_str(),
            path->points.size(),
            min_path_points_);
        writeZeroSteering(control_msg, true);
        return;
    }
    if (vehicle_status->emergency) {
        ROS_WARN_THROTTLE(
            1.0,
            "[%s] 车辆处于紧急状态，输出0转角",
            getName().c_str());
        writeZeroSteering(control_msg, true);
        return;
    }

    const double effective_dt = sanitizeControlDt(dt);
    controller_time_ += effective_dt;
    const double speed =
        std::fabs(vehicle_status->vel.linear.x);
    const double yaw_rate =
        filterYawRate(
            vehicle_status->vel.angular.z,
            effective_dt);
    const double measured_curvature =
        filterMeasuredCurvature(
            speed > minimum_control_speed_
                ? yaw_rate /
                      std::max(speed, 0.5)
                : 0.0,
            effective_dt);

    previous_speed_ = speed;
    speed_state_initialized_ = true;

    PathGeometry geometry;
    if (!buildPathGeometry(
            path,
            vehicle_status->pose,
            &geometry)) {
        ROS_WARN_THROTTLE(
            1.0,
            "[%s] 无法构造有效路径几何，输出0转角",
            getName().c_str());
        writeZeroSteering(control_msg, true);
        return;
    }

    updateTrackingSafety(
        geometry.lateral_error,
        geometry.heading_error,
        speed,
        effective_dt);

    const std::vector<double> reference_curvature =
        buildReferenceCurvatureSequence(
            geometry,
            speed);
    if (reference_curvature.empty()) {
        writeZeroSteering(control_msg, true);
        return;
    }

    // V0.14冻结此前不稳定的在线响应增益写入。连续滑模的有界
    // 积分只消除本次弯道残差，不跨启动、不跨左右弯保存参数。
    response_gain_scale_ = 1.0;
    response_gain_confidence_ = 0.0;
    response_gain_last_observation_ = 1.0;
    const FixedControlModel model =
        fixedControlModel(speed);
    const std::size_t preview_index =
        previewIndex(
            speed,
            model,
            reference_curvature.size());
    const double raw_preview_curvature =
        robustPreviewCurvature(
            reference_curvature,
            preview_index);
    const double preview_curvature =
        filterPreviewCurvature(
            raw_preview_curvature,
            effective_dt);

    SlidingModeControl sliding =
        computeSmoothSlidingModeControl(
            geometry.lateral_error,
            geometry.heading_error,
            measured_curvature,
            speed,
            reference_curvature.front(),
            preview_curvature,
            model,
            effective_dt);

    if (speed < minimum_control_speed_) {
        sliding.target_steering = 0.0;
        sliding_integral_curvature_ = 0.0;
    }
    const double target_steering =
        clampValue(
            sliding.target_steering,
            -max_steering_angle_,
            max_steering_angle_);
    const double steering_command =
        shapeSteeringCommand(
            target_steering,
            effective_dt);

    appendCommandHistory(
        std::tan(steering_command) /
            wheelbase_,
        preview_curvature,
        speed,
        geometry.lateral_error,
        geometry.heading_error);

    control_msg->lateral.steering_angle =
        steering_command;
    control_msg->lateral.steering_angle_velocity =
        previous_steering_rate_;
    control_msg->steering_mode =
        race_msgs::Control::FRONT_STEERING_MODE;

    writeSlidingModeCsv(
        speed,
        geometry,
        reference_curvature.front(),
        measured_curvature,
        model,
        sliding,
        steering_command,
        effective_dt);

    if (enable_debug_log_) {
        ROS_INFO_THROTTLE(
            debug_log_period_,
            "[%s] V0.14 v=%.2f | ey=%.3f | eyp=%.3f | "
            "epsi=%.4f | epsip=%.4f | kref0=%.5f | kprev=%.5f | "
            "kyaw=%.5f | tau=%.3f+%.3f | tp=%.3f | lc=%.2f | "
            "s=%.4f | smooth=%.3f | klin=%.5f | ksw=%.5f | "
            "kint=%.5f | kdes=%.5f | kdyn=%.5f | g=%.3f | "
            "delta_target=%.4f | delta_cmd=%.4f | ddelta=%.4f",
            getName().c_str(),
            speed,
            geometry.lateral_error,
            sliding.predicted_lateral_error,
            geometry.heading_error,
            sliding.predicted_heading_error,
            reference_curvature.front(),
            sliding.preview_reference_curvature,
            measured_curvature,
            actuator_time_constant_,
            sliding.lateral_response_time,
            sliding.prediction_time,
            sliding.convergence_length,
            sliding.surface,
            sliding.smooth_switch,
            sliding.linear_feedback_curvature,
            sliding.switching_curvature,
            sliding.integral_curvature,
            sliding.desired_vehicle_curvature,
            sliding.dynamically_compensated_curvature,
            model.response_gain,
            target_steering,
            steering_command,
            previous_steering_rate_);
    }
}

void HumanLikeAdaptiveTracker::computeControlLegacyV12(
    const race_msgs::VehicleStatusConstPtr& vehicle_status,
    const race_msgs::PathConstPtr& path,
    race_msgs::Control* control_msg,
    const double dt,
    const race_msgs::Flag::ConstPtr& flag) {

    (void)flag;

    if (!control_msg) {
        ROS_ERROR("[%s] 控制指针为空", getName().c_str());
        return;
    }
    if (!vehicle_status || !path) {
        ROS_ERROR_THROTTLE(
            1.0, "[%s] 车辆状态或路径为空，输出0转角",
            getName().c_str());
        writeZeroSteering(control_msg, true);
        return;
    }
    if (path->points.size() <
        static_cast<std::size_t>(min_path_points_)) {
        ROS_WARN_THROTTLE(
            1.0, "[%s] 路径点不足（%zu < %d），输出0转角",
            getName().c_str(),
            path->points.size(),
            min_path_points_);
        writeZeroSteering(control_msg, true);
        return;
    }
    if (vehicle_status->emergency) {
        ROS_WARN_THROTTLE(
            1.0, "[%s] 车辆处于紧急状态，输出0转角",
            getName().c_str());
        writeZeroSteering(control_msg, true);
        return;
    }

    const double effective_dt = sanitizeControlDt(dt);
    controller_time_ += effective_dt;

    const double longitudinal_speed =
        vehicle_status->vel.linear.x;
    const double speed = std::fabs(longitudinal_speed);
    const double filtered_yaw_rate =
        filterYawRate(vehicle_status->vel.angular.z, effective_dt);
    const double raw_curvature =
        speed > minimum_control_speed_
            ? filtered_yaw_rate / std::max(speed, 0.5)
            : 0.0;
    const double measured_curvature =
        filterMeasuredCurvature(raw_curvature, effective_dt);

    double longitudinal_acceleration = 0.0;
    if (speed_state_initialized_) {
        longitudinal_acceleration =
            (speed - previous_speed_) / effective_dt;
    }
    previous_speed_ = speed;
    speed_state_initialized_ = true;

    PathGeometry geometry;
    if (!buildPathGeometry(path, vehicle_status->pose, &geometry)) {
        ROS_WARN_THROTTLE(
            1.0, "[%s] 无法构造有效路径几何，输出0转角",
            getName().c_str());
        writeZeroSteering(control_msg, true);
        return;
    }

    updateTrackingSafety(
        geometry.lateral_error,
        geometry.heading_error,
        speed,
        effective_dt);

    const std::vector<double> reference_curvature =
        buildReferenceCurvatureSequence(geometry, speed);
    if (reference_curvature.empty()) {
        writeZeroSteering(control_msg, true);
        return;
    }
    updateResponseGainObserver(
        speed,
        measured_curvature,
        geometry.lateral_error,
        geometry.heading_error,
        effective_dt);
    // 固定速度先验只由一个左右/速度共用的安全观测尺度校准；
    // 影子学习和候选试验仍没有写入固定基线的权限。
    const FixedControlModel model =
        fixedControlModel(speed);
    const std::size_t feedforward_preview_index =
        previewIndex(
            speed,
            model,
            reference_curvature.size());
    const double raw_preview_curvature =
        robustPreviewCurvature(
            reference_curvature,
            feedforward_preview_index);
    const double preview_curvature =
        filterPreviewCurvature(
            raw_preview_curvature,
            effective_dt);
    updateShadowLearner(
        speed,
        longitudinal_acceleration,
        geometry.lateral_error,
        geometry.heading_error,
        effective_dt);
    updateTrackingPerformance(
        geometry.lateral_error,
        geometry.heading_error,
        speed,
        reference_curvature.front(),
        effective_dt);

    const BaselineControl baseline =
        computeDecoupledBaseline(
            geometry.lateral_error,
            geometry.heading_error,
            measured_curvature,
            speed,
            reference_curvature.front(),
            preview_curvature,
            model);
    const std::vector<double>
        desired_actual_curvature =
            buildDesiredActualCurvatureSequence(
                speed,
                reference_curvature,
                baseline.feedback_curvature,
                baseline.convergence_distance);
    const double baseline_target =
        baseline.target_steering;
    // 恢复监督在V0.12只做影子计算，用于记录反事实候选。
    // 经历V0.11撞墙后，未经同一闭环数据认证的新修正不得写入主控制。
    (void)applyCorridorRecoverySupervisor(
        geometry.lateral_error,
        geometry.heading_error,
        measured_curvature,
        speed,
        reference_curvature.front(),
        model,
        baseline,
        effective_dt);
    const double fallback_target =
        baseline_target;

    double target_steering =
        fallback_target;
    bool solver_ran = false;
    bool solver_succeeded =
        last_solver_succeeded_;
    solver_time_since_update_ +=
        effective_dt;

    // 预测模型的离散步长是0.1s，因此只按同一节拍更新优化目标。
    // 控制周期内保持目标，由输出整形连续跟随，避免每20ms重求解
    // 导致的有限时域“追赶—反向追赶”。
    const bool solver_update_due =
        !held_target_initialized_ ||
        solver_time_since_update_ + 1.0e-9 >=
            prediction_dt_;
    if (speed >= minimum_control_speed_ &&
        predictive_solver_initialized_ &&
        solver_update_due) {
        solver_ran = true;
        solver_time_since_update_ =
            std::fmod(
                solver_time_since_update_,
                prediction_dt_);

        double predictive_target =
            fallback_target;
        solver_succeeded =
            solvePredictiveControl(
                measured_curvature,
                desired_actual_curvature,
                model,
                &predictive_target);
        last_solver_succeeded_ =
            solver_succeeded;

        if (!held_target_initialized_) {
            held_target_steering_ =
                previous_steering_angle_;
            held_target_initialized_ =
                true;
        }

        const double maximum_target_step =
            0.50 *
            max_steering_rate_ *
            prediction_dt_;
        if (solver_succeeded) {
            // MPC只负责执行器动态补偿，不能远离稳定外环给出的目标。
            const double residual_limit =
                std::max(
                    0.005,
                    0.025 *
                        max_steering_angle_);
            const double anchored_target =
                clampValue(
                    predictive_target,
                    fallback_target -
                        residual_limit,
                    fallback_target +
                        residual_limit);
            held_target_steering_ +=
                clampValue(
                    anchored_target -
                        held_target_steering_,
                    -maximum_target_step,
                    maximum_target_step);
            consecutive_solver_failures_ = 0;
        } else {
            // 单次求解失败不再突然切换控制律，只让保持目标向稳定
            // 外环缓慢靠拢，从而保证有扰切换不会形成锯齿。
            held_target_steering_ +=
                clampValue(
                    fallback_target -
                        held_target_steering_,
                    -0.25 *
                        maximum_target_step,
                    0.25 *
                        maximum_target_step);
            ++consecutive_solver_failures_;
        }
    }

    if (speed >= minimum_control_speed_ &&
        predictive_solver_initialized_ &&
        held_target_initialized_) {
        target_steering =
            held_target_steering_;
    }
    if (speed < minimum_control_speed_) {
        target_steering = 0.0;
        held_target_initialized_ = false;
        held_target_steering_ = 0.0;
        last_solver_succeeded_ = false;
        solver_time_since_update_ =
            prediction_dt_;
    }

    // V0.12不允许历史候选残差进入主输出。
    applied_learning_residual_ = 0.0;
    target_steering =
        clampValue(
            target_steering,
            -max_steering_angle_,
            max_steering_angle_);

    const double steering_command =
        shapeSteeringCommand(target_steering, effective_dt);
    const double commanded_curvature =
        std::tan(steering_command) / wheelbase_;
    appendCommandHistory(
        commanded_curvature,
        preview_curvature,
        speed,
        geometry.lateral_error,
        geometry.heading_error);

    control_msg->lateral.steering_angle = steering_command;
    control_msg->lateral.steering_angle_velocity =
        previous_steering_rate_;
    control_msg->steering_mode =
        race_msgs::Control::FRONT_STEERING_MODE;

    writeDiagnosticCsv(
        speed,
        geometry,
        reference_curvature.front(),
        raw_preview_curvature,
        preview_curvature,
        measured_curvature,
        baseline,
        model,
        baseline_target,
        target_steering,
        steering_command,
        effective_dt);

    if (enable_debug_log_) {
        const double speed_coordinate =
            2.0 * normalizedSpeed(speed) - 1.0;
        const double prior_multiplier =
            1.0 / priorGainAtSpeed(speed);
        const double candidate_multiplier =
            prior_multiplier *
            std::exp(
                candidate_beta_scale_ +
                candidate_beta_speed_ *
                    speed_coordinate);
        const double certified_multiplier =
            prior_multiplier *
            std::exp(
                certified_beta_scale_ +
                certified_beta_speed_ *
                    speed_coordinate);
        // 在ROS测试桩把日志宏展开为空时仍保持-Werror可编译。
        (void)solver_ran;
        (void)candidate_multiplier;
        (void)certified_multiplier;
        ROS_INFO_THROTTLE(
            debug_log_period_,
            "[%s] V0.12 v=%.2f | ey=%.3f | eydot=%.3f | "
            "epsi=%.4f | epsip=%.4f | "
            "sc=%.2f | kref0=%.5f | kprev_raw=%.5f | kprev=%.5f | "
            "kyaw=%.5f | kfb=%.5f | dff=%.4f | dfb=%.4f | "
            "gmodel=%.3f | gscale=%.3f | gobs=%.3f | gconf=%.2f | "
            "recovery=%d/%.2f | eyr=%.3f | dbase=%.4f | drec=%.4f | "
            "mprior=%.3f | mcand=%.3f | mcert=%.3f | "
            "conf=%.2f | val0=%.3f | valc=%.3f | "
            "samples=%zu/%zu | promotes=%zu | rollbacks=%zu | "
            "trial=%d | soft=%d | locked=%d | "
            "dcert=%.5f | dtrial=%.5f | dlearn=%.5f | "
            "delta_target=%.4f | delta_cmd=%.4f | "
            "solver_ran=%d | solver=%d | failures=%d",
            getName().c_str(),
            speed,
            geometry.lateral_error,
            filtered_lateral_error_rate_,
            geometry.heading_error,
            baseline.predicted_heading_error,
            baseline.convergence_distance,
            reference_curvature.front(),
            raw_preview_curvature,
            preview_curvature,
            measured_curvature,
            baseline.feedback_curvature,
            baseline.feedforward_steering,
            baseline.feedback_steering,
            model.response_gain,
            response_gain_scale_,
            response_gain_last_observation_,
            response_gain_confidence_,
            corridor_recovery_active_ ? 1 : 0,
            corridor_recovery_blend_,
            corridor_predicted_error_,
            baseline_target,
            corridor_recovery_target_,
            prior_multiplier,
            candidate_multiplier,
            certified_multiplier,
            shadow_confidence_,
            prior_validation_rmse_,
            candidate_validation_rmse_,
            shadow_training_samples_,
            shadow_validation_samples_count_,
            promotion_count_,
            rollback_count_,
            trial_active_ ? 1 : 0,
            learning_soft_suppressed_ ? 1 : 0,
            learning_rollback_latched_ ? 1 : 0,
            certified_learning_residual_,
            trial_learning_residual_,
            applied_learning_residual_,
            target_steering,
            steering_command,
            solver_ran ? 1 : 0,
            solver_succeeded ? 1 : 0,
            consecutive_solver_failures_);
    }
}

bool HumanLikeAdaptiveTracker::buildPathGeometry(
    const race_msgs::PathConstPtr& path,
    const geometry_msgs::Pose& vehicle_pose,
    PathGeometry* geometry) const {

    if (!path || !geometry) {
        return false;
    }

    geometry->points.clear();
    geometry->arc_lengths.clear();
    geometry->points.reserve(path->points.size());
    geometry->arc_lengths.reserve(path->points.size());

    for (const auto& path_point : path->points) {
        geometry_msgs::Point point_vehicle =
            transformToVehicleFrame(
                path_point.pose.position,
                vehicle_pose);
        if (!std::isfinite(point_vehicle.x) ||
            !std::isfinite(point_vehicle.y)) {
            continue;
        }
        if (!geometry->points.empty()) {
            const geometry_msgs::Point& previous =
                geometry->points.back();
            if (std::hypot(point_vehicle.x - previous.x,
                           point_vehicle.y - previous.y) <
                minimum_path_point_spacing_) {
                continue;
            }
        }
        geometry->points.push_back(point_vehicle);
    }

    if (geometry->points.size() <
        static_cast<std::size_t>(min_path_points_)) {
        return false;
    }

    geometry->arc_lengths.resize(
        geometry->points.size(), 0.0);
    for (std::size_t i = 1;
         i < geometry->points.size();
         ++i) {
        geometry->arc_lengths[i] =
            geometry->arc_lengths[i - 1] +
            std::hypot(
                geometry->points[i].x -
                    geometry->points[i - 1].x,
                geometry->points[i].y -
                    geometry->points[i - 1].y);
    }
    geometry->total_length =
        geometry->arc_lengths.back();
    if (!std::isfinite(geometry->total_length) ||
        geometry->total_length <
            minimum_path_point_spacing_) {
        return false;
    }

    double minimum_distance_squared =
        std::numeric_limits<double>::max();
    double closest_arc_length = 0.0;
    geometry_msgs::Point closest_point;

    for (std::size_t i = 0;
         i + 1 < geometry->points.size();
         ++i) {
        const geometry_msgs::Point& first =
            geometry->points[i];
        const geometry_msgs::Point& second =
            geometry->points[i + 1];
        const double segment_x = second.x - first.x;
        const double segment_y = second.y - first.y;
        const double segment_length_squared =
            segment_x * segment_x +
            segment_y * segment_y;
        if (segment_length_squared < 1.0e-12) {
            continue;
        }

        const double projection_ratio =
            clampValue(
                -(first.x * segment_x +
                  first.y * segment_y) /
                    segment_length_squared,
                0.0,
                1.0);
        geometry_msgs::Point projection;
        projection.x =
            first.x + projection_ratio * segment_x;
        projection.y =
            first.y + projection_ratio * segment_y;
        projection.z = 0.0;
        const double distance_squared =
            projection.x * projection.x +
            projection.y * projection.y;

        if (distance_squared <
            minimum_distance_squared) {
            minimum_distance_squared =
                distance_squared;
            closest_point = projection;
            closest_arc_length =
                geometry->arc_lengths[i] +
                projection_ratio *
                    std::sqrt(segment_length_squared);
        }
    }

    if (!std::isfinite(minimum_distance_squared)) {
        return false;
    }

    geometry->closest_arc_length =
        closest_arc_length;
    geometry->closest_point = closest_point;
    geometry->heading_error =
        normalizeAngle(
            estimateHeading(
                *geometry,
                closest_arc_length));

    const double tangent_x =
        std::cos(geometry->heading_error);
    const double tangent_y =
        std::sin(geometry->heading_error);
    // 正值表示参考路径在车辆左侧，需要正曲率。
    geometry->lateral_error =
        tangent_x * closest_point.y -
        tangent_y * closest_point.x;

    return std::isfinite(
               geometry->lateral_error) &&
           std::isfinite(
               geometry->heading_error);
}

geometry_msgs::Point
HumanLikeAdaptiveTracker::transformToVehicleFrame(
    const geometry_msgs::Point& world_point,
    const geometry_msgs::Pose& vehicle_pose) const {

    const double dx =
        world_point.x - vehicle_pose.position.x;
    const double dy =
        world_point.y - vehicle_pose.position.y;
    const double yaw =
        tf::getYaw(vehicle_pose.orientation);

    geometry_msgs::Point point_vehicle;
    point_vehicle.x =
        dx * std::cos(yaw) +
        dy * std::sin(yaw);
    point_vehicle.y =
        -dx * std::sin(yaw) +
        dy * std::cos(yaw);
    point_vehicle.z = 0.0;
    return point_vehicle;
}

geometry_msgs::Point
HumanLikeAdaptiveTracker::samplePointAtArcLength(
    const PathGeometry& geometry,
    double arc_length) const {

    if (geometry.points.empty()) {
        return geometry_msgs::Point();
    }
    if (geometry.points.size() == 1 ||
        geometry.total_length <= 0.0) {
        return geometry.points.front();
    }

    arc_length =
        clampValue(
            arc_length,
            0.0,
            geometry.total_length);
    const auto upper =
        std::lower_bound(
            geometry.arc_lengths.begin(),
            geometry.arc_lengths.end(),
            arc_length);
    if (upper == geometry.arc_lengths.begin()) {
        return geometry.points.front();
    }
    if (upper == geometry.arc_lengths.end()) {
        return geometry.points.back();
    }

    const std::size_t upper_index =
        static_cast<std::size_t>(
            std::distance(
                geometry.arc_lengths.begin(),
                upper));
    const std::size_t lower_index =
        upper_index - 1;
    const double lower_s =
        geometry.arc_lengths[lower_index];
    const double upper_s =
        geometry.arc_lengths[upper_index];
    const double ratio =
        upper_s - lower_s > 1.0e-9
            ? (arc_length - lower_s) /
                  (upper_s - lower_s)
            : 0.0;

    geometry_msgs::Point point;
    point.x =
        geometry.points[lower_index].x +
        ratio *
            (geometry.points[upper_index].x -
             geometry.points[lower_index].x);
    point.y =
        geometry.points[lower_index].y +
        ratio *
            (geometry.points[upper_index].y -
             geometry.points[lower_index].y);
    point.z = 0.0;
    return point;
}

double HumanLikeAdaptiveTracker::estimateHeading(
    const PathGeometry& geometry,
    double arc_length) const {

    const PathDifferential fitted =
        fitPathDifferential(
            geometry,
            arc_length,
            std::max(
                4.0,
                1.5 *
                    curvature_smoothing_distance_));
    if (fitted.valid) {
        return fitted.heading;
    }

    const double half_window =
        std::max(1.0,
                 curvature_smoothing_distance_);
    const geometry_msgs::Point first =
        samplePointAtArcLength(
            geometry,
            arc_length - half_window);
    const geometry_msgs::Point second =
        samplePointAtArcLength(
            geometry,
            arc_length + half_window);
    const double dx = second.x - first.x;
    const double dy = second.y - first.y;
    if (std::hypot(dx, dy) <
        minimum_path_point_spacing_) {
        return 0.0;
    }
    return std::atan2(dy, dx);
}

double HumanLikeAdaptiveTracker::estimateCurvature(
    const PathGeometry& geometry,
    double arc_length) const {

    const double half_window =
        std::max(0.6,
                 0.50 *
                     curvature_smoothing_distance_);
    const double first_s =
        std::max(
            0.0,
            arc_length - half_window);
    const double third_s =
        std::min(
            geometry.total_length,
            arc_length + half_window);
    if (third_s - first_s <
        std::max(
            0.2,
            2.0 *
                minimum_path_point_spacing_)) {
        return 0.0;
    }

    const double second_s =
        0.5 * (first_s + third_s);
    const geometry_msgs::Point first =
        samplePointAtArcLength(
            geometry, first_s);
    const geometry_msgs::Point second =
        samplePointAtArcLength(
            geometry, second_s);
    const geometry_msgs::Point third =
        samplePointAtArcLength(
            geometry, third_s);

    const double ab_x =
        second.x - first.x;
    const double ab_y =
        second.y - first.y;
    const double ac_x =
        third.x - first.x;
    const double ac_y =
        third.y - first.y;
    const double bc_x =
        third.x - second.x;
    const double bc_y =
        third.y - second.y;
    const double denominator =
        std::hypot(ab_x, ab_y) *
        std::hypot(ac_x, ac_y) *
        std::hypot(bc_x, bc_y);
    if (denominator < 1.0e-9) {
        return 0.0;
    }

    return 2.0 *
           (ab_x * ac_y -
            ab_y * ac_x) /
           denominator;
}

HumanLikeAdaptiveTracker::PathDifferential
HumanLikeAdaptiveTracker::fitPathDifferential(
    const PathGeometry& geometry,
    double arc_length,
    double half_window) const {

    PathDifferential result;
    if (geometry.points.size() < 4 ||
        geometry.total_length <= 0.0) {
        return result;
    }

    arc_length =
        clampValue(
            arc_length,
            0.0,
            geometry.total_length);
    half_window =
        clampValue(
            half_window,
            1.0,
            std::max(
                1.0,
                geometry.total_length));
    const double first_s =
        std::max(
            0.0,
            arc_length -
                half_window);
    const double last_s =
        std::min(
            geometry.total_length,
            arc_length +
                half_window);
    const double span =
        last_s - first_s;
    if (span <
        std::max(
            2.0,
            8.0 *
                minimum_path_point_spacing_)) {
        return result;
    }

    constexpr int sample_count = 13;
    double augmented[4][6] = {};
    const double scale =
        std::max(
            0.5,
            0.5 * span);
    for (int sample_index = 0;
         sample_index < sample_count;
         ++sample_index) {
        const double ratio =
            static_cast<double>(
                sample_index) /
            static_cast<double>(
                sample_count - 1);
        const double sample_s =
            first_s +
            ratio * span;
        const double normalized_s =
            (sample_s -
             arc_length) /
            scale;
        const double normalized_squared =
            normalized_s *
            normalized_s;
        const double basis[4] = {
            1.0,
            normalized_s,
            normalized_squared,
            normalized_squared *
                normalized_s};
        const double weight =
            std::exp(
                -1.5 *
                normalized_squared);
        const geometry_msgs::Point point =
            samplePointAtArcLength(
                geometry,
                sample_s);

        for (int row = 0;
             row < 4;
             ++row) {
            for (int column = 0;
                 column < 4;
                 ++column) {
                augmented[row][column] +=
                    weight *
                    basis[row] *
                    basis[column];
            }
            augmented[row][4] +=
                weight *
                basis[row] *
                point.x;
            augmented[row][5] +=
                weight *
                basis[row] *
                point.y;
        }
    }

    // 很小的曲率项正则只用于避免路径窗口近似直线时病态。
    augmented[2][2] += 1.0e-7;
    augmented[3][3] += 1.0e-6;
    if (!solveFourBySix(augmented)) {
        return result;
    }

    const double dx_ds =
        augmented[1][4] /
        scale;
    const double dy_ds =
        augmented[1][5] /
        scale;
    const double d2x_ds2 =
        2.0 *
        augmented[2][4] /
        (scale * scale);
    const double d2y_ds2 =
        2.0 *
        augmented[2][5] /
        (scale * scale);
    const double derivative_norm =
        std::hypot(
            dx_ds,
            dy_ds);
    if (!std::isfinite(derivative_norm) ||
        derivative_norm < 1.0e-4) {
        return result;
    }

    result.heading =
        std::atan2(
            dy_ds,
            dx_ds);
    result.curvature =
        (dx_ds * d2y_ds2 -
         dy_ds * d2x_ds2) /
        (derivative_norm *
         derivative_norm *
         derivative_norm);
    result.valid =
        std::isfinite(result.heading) &&
        std::isfinite(result.curvature);
    return result;
}

double
HumanLikeAdaptiveTracker::estimateSmoothedCurvature(
    const PathGeometry& geometry,
    double arc_length,
    double speed) const {

    const double half_window =
        clampValue(
            5.0 +
                0.15 *
                    std::fabs(speed),
            5.0,
            12.0);
    const PathDifferential fitted =
        fitPathDifferential(
            geometry,
            arc_length,
            half_window);
    const double raw_curvature =
        fitted.valid
            ? fitted.curvature
            : estimateCurvature(
                  geometry,
                  arc_length);
    const double maximum_curvature =
        1.25 *
        std::tan(
            max_steering_angle_) /
        wheelbase_;
    return clampValue(
        raw_curvature,
        -maximum_curvature,
        maximum_curvature);
}

std::vector<double>
HumanLikeAdaptiveTracker::buildReferenceCurvatureSequence(
    const PathGeometry& geometry,
    double speed) const {

    std::vector<double> reference(
        static_cast<std::size_t>(
            prediction_horizon_steps_),
        0.0);

    for (int i = 0;
         i < prediction_horizon_steps_;
         ++i) {
        const double future_distance =
            speed *
            prediction_dt_ *
            static_cast<double>(i);
        const double sample_s =
            clampValue(
                geometry.closest_arc_length +
                    future_distance,
                geometry.closest_arc_length,
                geometry.total_length);
        reference[static_cast<std::size_t>(i)] =
            estimateSmoothedCurvature(
                geometry,
                sample_s,
                speed);
    }
    return reference;
}

std::vector<double>
HumanLikeAdaptiveTracker::buildDesiredActualCurvatureSequence(
    double speed,
    const std::vector<double>& reference_curvature,
    double feedback_curvature,
    double convergence_distance) const {

    std::vector<double> desired(
        reference_curvature.size(), 0.0);
    if (reference_curvature.empty()) {
        return desired;
    }

    const double safe_speed =
        std::max(std::fabs(speed), 0.5);
    const double maximum_curvature =
        std::tan(max_steering_angle_) /
        wheelbase_;
    convergence_distance =
        std::max(
            wheelbase_,
            convergence_distance);

    for (std::size_t i = 0;
         i < reference_curvature.size();
         ++i) {
        // 只预瞄参考曲率，不用理想车辆模型外推误差。反馈修正按
        // 未来行驶距离衰减，使相同空间误差在高低速具有相近强度。
        const double future_distance =
            safe_speed *
            prediction_dt_ *
            static_cast<double>(i);
        const double correction =
            feedback_curvature /
            (1.0 +
             future_distance /
                 convergence_distance);
        desired[i] =
            clampValue(
                reference_curvature[i] +
                    correction,
                -maximum_curvature,
                maximum_curvature);
    }
    return desired;
}

double HumanLikeAdaptiveTracker::robustPreviewCurvature(
    const std::vector<double>& reference_curvature,
    std::size_t preview_index) const {

    if (reference_curvature.empty()) {
        return 0.0;
    }
    preview_index =
        std::min(
            preview_index,
            reference_curvature.size() - 1);
    const std::size_t first_index =
        preview_index > 1
            ? preview_index - 2
            : 0;
    const std::size_t last_index =
        std::min(
            reference_curvature.size() - 1,
            preview_index + 2);
    std::vector<double> local_values;
    local_values.reserve(
        last_index -
        first_index +
        1);
    for (std::size_t i = first_index;
         i <= last_index;
         ++i) {
        if (std::isfinite(
                reference_curvature[i])) {
            local_values.push_back(
                reference_curvature[i]);
        }
    }
    if (local_values.empty()) {
        return 0.0;
    }
    std::sort(
        local_values.begin(),
        local_values.end());
    const std::size_t middle =
        local_values.size() / 2;
    if (local_values.size() % 2 == 1) {
        return local_values[middle];
    }
    return 0.5 *
           (local_values[middle - 1] +
            local_values[middle]);
}

double HumanLikeAdaptiveTracker::filterPreviewCurvature(
    double preview_curvature,
    double dt) {

    if (!std::isfinite(preview_curvature)) {
        preview_curvature = 0.0;
    }
    if (!preview_curvature_filter_initialized_) {
        filtered_preview_curvature_ =
            preview_curvature;
        preview_curvature_filter_initialized_ =
            true;
        return filtered_preview_curvature_;
    }
    const double time_constant =
        std::max(
            0.10,
            0.50 *
                prior_delay_);
    const double alpha =
        clampValue(
            dt /
                (time_constant +
                 dt),
            0.0,
            1.0);
    filtered_preview_curvature_ +=
        alpha *
        (preview_curvature -
         filtered_preview_curvature_);
    return filtered_preview_curvature_;
}

HumanLikeAdaptiveTracker::BaselineControl
HumanLikeAdaptiveTracker::computeDecoupledBaseline(
    double lateral_error,
    double heading_error,
    double measured_curvature,
    double speed,
    double current_reference_curvature,
    double preview_reference_curvature,
    const FixedControlModel& model) const {

    BaselineControl result;
    const double safe_speed =
        std::max(
            std::fabs(speed),
            0.5);

    // 反馈收敛距离必须明显大于轴距，并覆盖时延和两倍执行器时间常数。
    // 这给半挂牵引车未建模动态保留相位裕度。
    result.convergence_distance =
        std::max(
            2.5 * wheelbase_,
            safe_speed *
                (model.delay +
                 2.0 *
                     model.response_time));

    // 只用运动学关系预测执行器真正响应时的航向误差：
    // d(e_psi)/dt = v * (kappa_ref - kappa_vehicle)。
    const double yaw_prediction_time =
        model.delay +
        0.5 *
            model.response_time;
    const double yaw_prediction =
        clampValue(
            safe_speed *
                yaw_prediction_time *
                (current_reference_curvature -
                 measured_curvature),
            -0.06,
            0.06);
    result.predicted_heading_error =
        clampValue(
            heading_error +
                yaw_prediction,
            -0.35,
            0.35);

    const double bounded_lateral_error =
        clampValue(
            lateral_error,
            -10.0,
            10.0);
    const double recovery_blend =
        clampValue(
            (std::fabs(lateral_error) -
             allowed_lateral_error_) /
                std::max(
                    0.40,
                    4.0 *
                        allowed_lateral_error_),
            0.0,
            1.0);
    const double raw_feedback_curvature =
        2.0 *
            result.predicted_heading_error /
            result.convergence_distance +
        bounded_lateral_error /
            (result.convergence_distance *
             result.convergence_distance);

    // 小误差时最多使用15%总转角作反馈；大误差平滑增至25%。
    // 该限制直接位于前轮转角域，不随响应增益倒数放大。
    const double maximum_feedback_steering =
        (0.15 +
         0.10 *
             recovery_blend) *
        max_steering_angle_;
    const double maximum_feedback_curvature =
        std::tan(
            maximum_feedback_steering) /
        wheelbase_;
    result.feedback_curvature =
        maximum_feedback_curvature *
        std::tanh(
            raw_feedback_curvature /
            std::max(
                maximum_feedback_curvature,
                1.0e-6));
    result.feedback_steering =
        std::atan(
            wheelbase_ *
            result.feedback_curvature);

    // 响应增益只补偿参考曲率前馈，绝不再放大误差反馈。
    result.feedforward_steering =
        std::atan(
            wheelbase_ *
            preview_reference_curvature /
            std::max(
                kMinimumResponseGain,
                model.response_gain));
    result.target_steering =
        clampValue(
            result.feedforward_steering +
                result.feedback_steering,
            -max_steering_angle_,
            max_steering_angle_);
    return result;
}

HumanLikeAdaptiveTracker::SlidingModeControl
HumanLikeAdaptiveTracker::computeSmoothSlidingModeControl(
    double lateral_error,
    double heading_error,
    double measured_curvature,
    double speed,
    double current_reference_curvature,
    double preview_reference_curvature,
    const FixedControlModel& model,
    double dt) {

    SlidingModeControl result;
    const double effective_dt = sanitizeControlDt(dt);
    const double safe_speed =
        std::max(std::fabs(speed), 0.5);
    result.preview_reference_curvature =
        preview_reference_curvature;
    result.lateral_response_time =
        lateralResponseTime(safe_speed);

    // 将所有误差统一外推到“纯时延结束 + 一阶动态基本建立”时刻。
    // 这避免旧版用未来曲率配当前误差产生系统性切弯。
    const double combined_response_time =
        actuator_time_constant_ +
        result.lateral_response_time;
    result.prediction_time =
        clampValue(
            model.delay +
                combined_response_time,
            0.10,
            1.20);
    const double reference_curvature_average =
        0.5 *
        (current_reference_curvature +
         preview_reference_curvature);
    // 已规划的预瞄前馈会把车辆曲率推向参考曲率，因此不能把
    // “当前曲率在整个一阶建立时间内保持不变”再次计入反馈，
    // 否则入弯瞬间会重复补偿并直接打满方向。这里只预测不可由
    // 新指令消除的纯时延段，并保守使用35%的曲率失配。
    const double predicted_heading_increment =
        clampValue(
            safe_speed *
                model.delay *
                0.35 *
                (reference_curvature_average -
                 measured_curvature),
            -0.045,
            0.045);
    result.predicted_heading_error =
        clampValue(
            heading_error +
                predicted_heading_increment,
            -0.35,
            0.35);

    const double maximum_error_rate =
        std::min(
            3.0,
            0.18 * safe_speed +
                0.25);
    const double bounded_error_rate =
        clampValue(
            filtered_lateral_error_rate_,
            -maximum_error_rate,
            maximum_error_rate);
    result.predicted_lateral_error =
        clampValue(
            lateral_error +
                clampValue(
                    result.prediction_time *
                        bounded_error_rate,
                    -std::max(
                        0.20,
                        2.0 *
                            allowed_lateral_error_),
                    std::max(
                        0.20,
                        2.0 *
                            allowed_lateral_error_)),
            -5.0,
            5.0);

    // Lc随车速与动态距离连续增长，限制高速闭环带宽；低速又不会
    // 因为过长预瞄而失去大曲率弯道的纠偏能力。
    const double dynamic_distance =
        safe_speed *
        (model.delay +
         0.35 * combined_response_time);
    result.convergence_length =
        std::max(
            sliding_minimum_convergence_length_,
            0.55 * safe_speed +
                0.80 * dynamic_distance);

    result.surface =
        result.predicted_heading_error +
        std::atan(
            result.predicted_lateral_error /
            result.convergence_length);
    const double speed_enlarged_boundary =
        sliding_boundary_layer_ +
        0.00100 * safe_speed;
    result.smooth_switch =
        std::tanh(
            result.surface /
            speed_enlarged_boundary);

    result.linear_feedback_curvature =
        2.0 *
            result.predicted_heading_error /
            result.convergence_length +
        result.predicted_lateral_error /
            (result.convergence_length *
             result.convergence_length);
    const double reaching_speed_scale =
        1.0 /
        std::sqrt(
            1.0 +
            safe_speed / 22.0);
    result.switching_curvature =
        sliding_reaching_curvature_ *
        reaching_speed_scale *
        result.smooth_switch;

    // 左右弯共享完全相同的积分状态规则；换向立即清零，防止上一弯
    // 的补偿带到下一弯。积分有泄漏、有幅值限制，不构成跨圈学习。
    const double reference_sign =
        std::fabs(preview_reference_curvature) >= 0.0010
            ? (preview_reference_curvature > 0.0
                   ? 1.0
                   : -1.0)
            : 0.0;
    if (sliding_reference_sign_initialized_ &&
        reference_sign != 0.0 &&
        sliding_previous_reference_sign_ != 0.0 &&
        reference_sign !=
            sliding_previous_reference_sign_) {
        sliding_integral_curvature_ = 0.0;
    }
    if (reference_sign != 0.0) {
        sliding_previous_reference_sign_ =
            reference_sign;
        sliding_reference_sign_initialized_ =
            true;
    }

    const bool integration_trustworthy =
        safe_speed >= minimum_control_speed_ &&
        std::fabs(result.predicted_lateral_error) <= 2.5 &&
        std::fabs(result.predicted_heading_error) <= 0.20 &&
        std::fabs(result.surface) <= 0.25;
    const double integral_limit =
        clampValue(
            sliding_integral_base_limit_ +
                sliding_integral_curve_ratio_ *
                    std::fabs(
                        preview_reference_curvature),
            sliding_integral_base_limit_,
            0.020);
    if (integration_trustworthy) {
        const double integral_derivative =
            sliding_integral_rate_ *
                result.smooth_switch -
            sliding_integral_curvature_ /
                sliding_integral_leak_time_;
        sliding_integral_curvature_ +=
            effective_dt *
            integral_derivative;
    } else {
        const double safe_decay_time =
            std::max(
                0.50,
                0.20 *
                    sliding_integral_leak_time_);
        sliding_integral_curvature_ *=
            std::exp(
                -effective_dt /
                safe_decay_time);
    }
    sliding_integral_curvature_ =
        clampValue(
            sliding_integral_curvature_,
            -integral_limit,
            integral_limit);
    result.integral_curvature =
        sliding_integral_curvature_;

    const double maximum_vehicle_curvature =
        std::tan(max_steering_angle_) /
        wheelbase_;
    result.desired_vehicle_curvature =
        clampValue(
            preview_reference_curvature +
                result.linear_feedback_curvature +
                result.switching_curvature +
                result.integral_curvature,
            -maximum_vehicle_curvature,
            maximum_vehicle_curvature);

    // 一阶逆响应：在一个组合时间常数后，希望实测曲率到达kdesired。
    // 对逆补偿单独限幅，模型失配时仍退化为普通有界滑模控制。
    const double first_order_memory = 0.45;
    const double raw_dynamic_curvature =
        preview_reference_curvature +
        first_order_memory *
            (preview_reference_curvature -
             measured_curvature);
    const double dynamic_correction_limit =
        clampValue(
            0.25 *
                    sliding_reaching_curvature_ +
                0.08 *
                    std::fabs(
                        preview_reference_curvature),
            0.0040,
            0.0120);
    result.dynamically_compensated_curvature =
        clampValue(
            raw_dynamic_curvature,
            preview_reference_curvature -
                dynamic_correction_limit,
            preview_reference_curvature +
                dynamic_correction_limit);

    const double safe_response_gain =
        std::max(0.30, model.response_gain);
    const double feedback_command_curvature =
        clampValue(
            result.linear_feedback_curvature +
                result.switching_curvature +
                result.integral_curvature,
            -0.015,
            0.015);
    const double command_curvature =
        result.dynamically_compensated_curvature /
            safe_response_gain +
        feedback_command_curvature;
    result.target_steering =
        clampValue(
            std::atan(
                wheelbase_ *
                command_curvature),
            -max_steering_angle_,
            max_steering_angle_);
    return result;
}

double HumanLikeAdaptiveTracker::lateralResponseTime(
    double speed) const {

    const double safe_speed =
        std::max(
            std::fabs(speed),
            0.5 *
                lateral_response_reference_speed_);
    return clampValue(
        lateral_response_time_at_reference_speed_ *
            lateral_response_reference_speed_ /
            safe_speed,
        0.08,
        0.45);
}

double HumanLikeAdaptiveTracker::normalizedSpeed(
    double speed) const {

    return clampValue(
        (std::fabs(speed) - 5.0) / 25.0,
        0.0,
        1.0);
}

double HumanLikeAdaptiveTracker::priorGainAtSpeed(
    double speed) const {

    // 固定先验沿用已观察到的“高速有效曲率响应较低”关系。学习器
    // 只能在该连续关系上估计一个全局尺度和很小的速度斜率。
    const double normalized =
        normalizedSpeed(speed);
    const double gain =
        (1.0 - normalized) *
            1.20 * prior_response_gain_ +
        normalized *
            0.50 * prior_response_gain_;
    return clampValue(
        gain,
        kMinimumResponseGain,
        kMaximumResponseGain);
}

HumanLikeAdaptiveTracker::FixedControlModel
HumanLikeAdaptiveTracker::fixedControlModel(
    double speed) const {

    FixedControlModel model;
    model.response_gain =
        clampValue(
            priorGainAtSpeed(speed) *
                response_gain_scale_,
            kMinimumResponseGain,
            kMaximumResponseGain);
    model.response_time =
        actuator_time_constant_ +
        lateralResponseTime(speed);
    model.delay =
        prior_delay_;
    return model;
}

std::size_t HumanLikeAdaptiveTracker::previewIndex(
    double speed,
    const FixedControlModel& model,
    std::size_t sequence_size) const {

    if (sequence_size == 0) {
        return 0;
    }
    (void)speed;
    const double preview_time =
        model.delay +
        model.response_time;
    return std::min(
        sequence_size - 1,
        static_cast<std::size_t>(
            std::max(
                0.0,
                std::round(
                    preview_time /
                    prediction_dt_))));
}


void HumanLikeAdaptiveTracker::initializeShadowLearner() {
    shadow_normal_00_ =
        kShadowScaleRegularization;
    shadow_normal_01_ = 0.0;
    shadow_normal_11_ =
        kShadowSpeedRegularization;
    shadow_rhs_0_ = 0.0;
    shadow_rhs_1_ = 0.0;
    candidate_beta_scale_ = 0.0;
    candidate_beta_speed_ = 0.0;
    certified_beta_scale_ = 0.0;
    certified_beta_speed_ = 0.0;
    trial_beta_scale_ = 0.0;
    trial_beta_speed_ = 0.0;
    candidate_validation_rmse_ =
        std::numeric_limits<double>::infinity();
    prior_validation_rmse_ =
        std::numeric_limits<double>::infinity();
    shadow_confidence_ = 0.0;
    shadow_sample_timer_ = 0.0;
    promotion_hold_timer_ = 0.0;
    trial_score_sum_ = 0.0;
    trial_reference_score_sum_ = 0.0;
    trial_reference_level_ = 0.0;
    trial_active_time_ = 0.0;
    trial_block_timer_ = 0.0;
    trial_performance_cell_index_ = 0;
    shadow_total_samples_ = 0;
    shadow_training_samples_ = 0;
    shadow_validation_samples_count_ = 0;
    promotion_count_ = 0;
    rollback_count_ = 0;
    shadow_validation_buffer_.clear();
    for (PerformanceCell& cell :
         performance_cells_) {
        cell = PerformanceCell();
    }
    trial_active_ = false;
    learning_rollback_latched_ = false;
    learning_soft_suppressed_ = false;
    applied_learning_residual_ = 0.0;
    certified_learning_residual_ = 0.0;
    trial_learning_residual_ = 0.0;
}

void HumanLikeAdaptiveTracker::updateTrackingSafety(
    double lateral_error,
    double heading_error,
    double speed,
    double dt) {

    if (!lateral_error_state_initialized_) {
        previous_lateral_error_ =
            lateral_error;
        filtered_lateral_error_rate_ = 0.0;
        lateral_error_state_initialized_ = true;
    }

    const double raw_error_rate =
        (lateral_error -
         previous_lateral_error_) /
        std::max(dt, 1.0e-4);
    previous_lateral_error_ =
        lateral_error;
    const double rate_alpha =
        clampValue(
            dt / (0.35 + dt),
            0.0,
            1.0);
    filtered_lateral_error_rate_ +=
        rate_alpha *
        (raw_error_rate -
         filtered_lateral_error_rate_);

    const double predicted_error_bound =
        std::fabs(lateral_error) +
        0.35 *
            std::fabs(speed) *
            std::fabs(
                std::sin(
                    clampValue(
                        heading_error,
                        -0.35,
                        0.35)));
    learning_soft_suppressed_ =
        std::fabs(lateral_error) >=
            learning_soft_error_ ||
        predicted_error_bound >=
            learning_hard_error_;

    const bool moving_outward =
        lateral_error *
            filtered_lateral_error_rate_ >
        0.01;
    const bool trial_was_active =
        trial_active_ &&
        std::fabs(trial_learning_residual_) >
            1.0e-5;
    const bool certified_baseline_was_good =
        trial_reference_level_ <=
            learning_soft_error_;

    if (trial_was_active &&
        certified_baseline_was_good &&
        learning_soft_suppressed_ &&
        moving_outward) {
        rejectTrial("tracking safety envelope");
    }
}

void HumanLikeAdaptiveTracker::updateShadowLearner(
    double speed,
    double longitudinal_acceleration,
    double lateral_error,
    double heading_error,
    double dt) {

    shadow_sample_timer_ += dt;
    if (!enable_shadow_learning_ ||
        shadow_sample_timer_ + 1.0e-9 <
            kShadowSamplePeriod) {
        return;
    }
    shadow_sample_timer_ =
        std::fmod(
            shadow_sample_timer_,
            kShadowSamplePeriod);

    const double demonstration_error_limit =
        std::max(
            1.0,
            10.0 * allowed_lateral_error_);
    if (speed < std::max(3.0, minimum_control_speed_) ||
        std::fabs(longitudinal_acceleration) >
            kShadowMaximumLongitudinalAcceleration ||
        std::fabs(lateral_error) >
            demonstration_error_limit ||
        std::fabs(heading_error) >
            kShadowMaximumHeadingError ||
        std::fabs(filtered_lateral_error_rate_) >
            kShadowMaximumLateralErrorRate) {
        promotion_hold_timer_ = 0.0;
        return;
    }

    CommandSample delayed_sample;
    const double demonstration_age =
        prior_delay_ +
        prior_response_time_;
    if (!lookupCommandSampleAtTime(
            controller_time_ -
                demonstration_age,
            &delayed_sample)) {
        return;
    }
    CommandSample previous_sample =
        delayed_sample;
    lookupCommandSampleAtTime(
        controller_time_ -
            demonstration_age -
            kShadowSamplePeriod,
        &previous_sample);

    const double command_rate =
        (delayed_sample.commanded_curvature -
         previous_sample.commanded_curvature) /
        kShadowSamplePeriod;
    const double reference_rate =
        (delayed_sample.
             preview_reference_curvature -
         previous_sample.
             preview_reference_curvature) /
        kShadowSamplePeriod;

    if (std::fabs(
            delayed_sample.
                preview_reference_curvature) <
            kShadowMinimumReferenceCurvature ||
        delayed_sample.commanded_curvature *
                delayed_sample.
                    preview_reference_curvature <=
            0.0 ||
        std::fabs(command_rate) >
            kShadowMaximumCommandRate ||
        std::fabs(reference_rate) >
            kShadowMaximumReferenceRate ||
        std::fabs(
            delayed_sample.lateral_error) >
            demonstration_error_limit ||
        std::fabs(
            delayed_sample.heading_error) >
            kShadowMaximumHeadingError) {
        promotion_hold_timer_ = 0.0;
        return;
    }

    const double required_multiplier =
        std::fabs(
            delayed_sample.commanded_curvature /
            delayed_sample.
                preview_reference_curvature);
    if (!std::isfinite(required_multiplier) ||
        required_multiplier < 0.25 ||
        required_multiplier > 10.0) {
        return;
    }

    const double sample_speed =
        std::max(
            0.0,
            delayed_sample.speed);
    const double speed_coordinate =
        2.0 * normalizedSpeed(sample_speed) -
        1.0;
    const double prior_multiplier =
        1.0 /
        priorGainAtSpeed(sample_speed);
    const double log_feedforward_ratio =
        std::log(
            required_multiplier /
            prior_multiplier);
    if (!std::isfinite(
            log_feedforward_ratio) ||
        std::fabs(log_feedforward_ratio) >
            1.20) {
        return;
    }

    const double current_prediction =
        candidate_beta_scale_ +
        candidate_beta_speed_ *
            speed_coordinate;
    const double raw_residual =
        log_feedforward_ratio -
        current_prediction;
    const double robust_weight =
        std::fabs(raw_residual) <=
                kShadowHuberLogResidual
            ? 1.0
            : kShadowHuberLogResidual /
                  std::fabs(raw_residual);

    const bool validation_sample =
        shadow_total_samples_ % 5 == 4;
    ++shadow_total_samples_;
    if (validation_sample) {
        ShadowValidationSample sample;
        sample.speed_coordinate =
            speed_coordinate;
        sample.log_feedforward_ratio =
            log_feedforward_ratio;
        shadow_validation_buffer_.push_back(
            sample);
        while (shadow_validation_buffer_.size() >
               kShadowMaximumValidationSamples) {
            shadow_validation_buffer_.pop_front();
        }
        shadow_validation_samples_count_ =
            shadow_validation_buffer_.size();
    } else {
        shadow_normal_00_ +=
            robust_weight;
        shadow_normal_01_ +=
            robust_weight *
            speed_coordinate;
        shadow_normal_11_ +=
            robust_weight *
            speed_coordinate *
            speed_coordinate;
        shadow_rhs_0_ +=
            robust_weight *
            log_feedforward_ratio;
        shadow_rhs_1_ +=
            robust_weight *
            speed_coordinate *
            log_feedforward_ratio;
        ++shadow_training_samples_;
    }

    updateShadowCandidate();

    const bool qualified =
        shadow_training_samples_ >=
            kShadowMinimumTrainingSamples &&
        shadow_validation_samples_count_ >=
            kShadowMinimumValidationSamples &&
        std::isfinite(
            candidate_validation_rmse_) &&
        std::isfinite(
            prior_validation_rmse_) &&
        candidate_validation_rmse_ <=
            kShadowMaximumCandidateRmse &&
        candidate_validation_rmse_ <=
            kShadowRequiredRmseRatio *
                prior_validation_rmse_;

    if (qualified &&
        !trial_active_ &&
        !learning_rollback_latched_) {
        promotion_hold_timer_ +=
            kShadowSamplePeriod;
    } else {
        promotion_hold_timer_ = 0.0;
    }
}

void HumanLikeAdaptiveTracker::updateShadowCandidate() {
    const double determinant =
        shadow_normal_00_ *
            shadow_normal_11_ -
        shadow_normal_01_ *
            shadow_normal_01_;
    if (!std::isfinite(determinant) ||
        determinant < 1.0e-9) {
        return;
    }

    candidate_beta_scale_ =
        clampValue(
            (shadow_rhs_0_ *
                 shadow_normal_11_ -
             shadow_rhs_1_ *
                 shadow_normal_01_) /
                determinant,
            -kShadowMaximumScaleMagnitude,
            kShadowMaximumScaleMagnitude);
    candidate_beta_speed_ =
        clampValue(
            (shadow_normal_00_ *
                 shadow_rhs_1_ -
             shadow_normal_01_ *
                 shadow_rhs_0_) /
                determinant,
            -kShadowMaximumSpeedMagnitude,
            kShadowMaximumSpeedMagnitude);

    prior_validation_rmse_ =
        validationRmse(0.0, 0.0);
    candidate_validation_rmse_ =
        validationRmse(
            candidate_beta_scale_,
            candidate_beta_speed_);

    const double sample_factor =
        std::min(
            1.0,
            std::min(
                static_cast<double>(
                    shadow_training_samples_) /
                    80.0,
                static_cast<double>(
                    shadow_validation_samples_count_) /
                    20.0));
    double improvement_factor = 0.0;
    if (std::isfinite(prior_validation_rmse_) &&
        prior_validation_rmse_ > 1.0e-6 &&
        std::isfinite(
            candidate_validation_rmse_)) {
        improvement_factor =
            clampValue(
                (prior_validation_rmse_ -
                 candidate_validation_rmse_) /
                    (0.25 *
                     prior_validation_rmse_),
                0.0,
                1.0);
    }
    shadow_confidence_ =
        sample_factor *
        improvement_factor;
}

double HumanLikeAdaptiveTracker::validationRmse(
    double beta_scale,
    double beta_speed) const {

    if (shadow_validation_buffer_.empty()) {
        return std::numeric_limits<double>::infinity();
    }

    double squared_error_sum = 0.0;
    for (const ShadowValidationSample& sample :
         shadow_validation_buffer_) {
        const double prediction =
            beta_scale +
            beta_speed *
                sample.speed_coordinate;
        const double residual =
            clampValue(
                sample.log_feedforward_ratio -
                    prediction,
                -0.70,
                0.70);
        squared_error_sum +=
            residual * residual;
    }
    return std::sqrt(
        squared_error_sum /
        static_cast<double>(
            shadow_validation_buffer_.size()));
}

void HumanLikeAdaptiveTracker::updateTrackingPerformance(
    double lateral_error,
    double heading_error,
    double speed,
    double reference_curvature,
    double dt) {

    trial_block_timer_ =
        std::max(
            0.0,
            trial_block_timer_ - dt);
    if (std::fabs(reference_curvature) <
            kShadowMinimumReferenceCurvature ||
        speed < minimum_control_speed_) {
        return;
    }

    const std::size_t current_cell_index =
        performanceCellIndex(
            speed,
            std::fabs(reference_curvature));
    const std::size_t cell_index =
        trial_active_
            ? trial_performance_cell_index_
            : current_cell_index;
    PerformanceCell& cell =
        performance_cells_[cell_index];
    const double convergence_distance =
        std::max(
            1.5 * wheelbase_,
            std::fabs(speed) *
                (prior_delay_ +
                 prior_response_time_));
    const double tracking_score =
        std::fabs(lateral_error) +
        0.5 *
            convergence_distance *
            std::fabs(heading_error);

    if (trial_active_) {
        if (std::fabs(
                trial_learning_residual_) >
                1.0e-5) {
            trial_active_time_ += dt;
            if (trial_active_time_ >
                kTrialWarmupTime) {
                trial_score_sum_ +=
                    tracking_score * dt;
                trial_reference_score_sum_ +=
                    cell.certified_score * dt;
            }

            const bool moving_outward =
                lateral_error *
                    filtered_lateral_error_rate_ >
                0.01;
            const bool bad_baseline =
                cell.certified_score >
                learning_soft_error_;
            const double instantaneous_limit =
                bad_baseline
                    ? std::max(
                          learning_hard_error_,
                          1.50 *
                                  cell.certified_score +
                              0.08)
                    : std::max(
                          learning_soft_error_,
                          1.25 *
                                  cell.certified_score +
                              0.05);
            if (moving_outward &&
                tracking_score >
                    instantaneous_limit) {
                rejectTrial(
                    "trial tracking score increased");
                return;
            }

            if (trial_active_time_ >=
                kTrialWarmupTime +
                    kTrialEvaluationTime) {
                const double evaluated_time =
                    std::max(
                        trial_active_time_ -
                            kTrialWarmupTime,
                        1.0e-6);
                const double trial_mean =
                    trial_score_sum_ /
                    evaluated_time;
                const double reference_mean =
                    trial_reference_score_sum_ /
                    evaluated_time;
                const bool bad_baseline =
                    reference_mean >
                    learning_soft_error_;
                const double acceptance_limit =
                    bad_baseline
                        ? 0.98 *
                              reference_mean
                        : kTrialMaximumScoreRatio *
                                  reference_mean +
                              kTrialMaximumScoreAllowance;
                if (trial_mean <=
                    acceptance_limit) {
                    acceptTrial();
                } else {
                    rejectTrial(
                        "trial mean score was worse");
                }
            }
        } else {
            // 小曲率或总残差已经到达硬上限时，参数步进可能对本周期
            // 指令没有可测作用。中性结束该trial，既不认证也不锁死。
            trial_active_time_ += dt;
            if (trial_active_time_ >= 0.25) {
                trial_beta_scale_ =
                    certified_beta_scale_;
                trial_beta_speed_ =
                    certified_beta_speed_;
                trial_active_ = false;
                trial_reference_level_ = 0.0;
                trial_active_time_ = 0.0;
                trial_performance_cell_index_ = 0;
                promotion_hold_timer_ = 0.0;
                trial_block_timer_ = 3.0;
            }
        }
        return;
    }

    // 只用误差变化已经稳定的片段建立对照。固定基线即使有稳定偏差
    // 也可作为基准，但首次入弯的大瞬态不会累计足够观察时间。
    if (std::fabs(
            filtered_lateral_error_rate_) >
            0.5 *
                kShadowMaximumLateralErrorRate ||
        std::fabs(heading_error) >
            2.0 *
                kShadowMaximumHeadingError) {
        return;
    }

    // 基线评分恶化时快速跟随、改善时慢速确认，近似保存稳定工况
    // 的近期上包络。试用由峰值/上包络而非偶然处于低谷的瞬时误差
    // 认证，可避免低频起伏恰好过零时错误拒绝有效前馈。
    const double performance_time_constant =
        tracking_score >
                cell.certified_score
            ? 0.50
            : kPerformanceFilterTimeConstant;
    const double alpha =
        clampValue(
            dt /
                (performance_time_constant +
                 dt),
            0.0,
            1.0);
    if (cell.observation_time <= 0.0) {
        cell.certified_score =
            tracking_score;
    } else {
        cell.certified_score +=
            alpha *
            (tracking_score -
             cell.certified_score);
    }
    cell.observation_time += dt;
    const double pairing_tolerance =
        0.02 +
        0.15 *
            std::max(
                cell.certified_score,
                learning_soft_error_);
    if (std::fabs(
            tracking_score -
            cell.certified_score) >
        pairing_tolerance) {
        return;
    }
    tryStartTrial(
        speed,
        reference_curvature);
}

std::size_t
HumanLikeAdaptiveTracker::performanceCellIndex(
    double speed,
    double absolute_curvature) const {

    const double normalized =
        normalizedSpeed(speed);
    std::size_t speed_bin = 0;
    if (normalized >= 2.0 / 3.0) {
        speed_bin = 2;
    } else if (normalized >= 1.0 / 3.0) {
        speed_bin = 1;
    }
    const std::size_t curvature_bin =
        absolute_curvature * wheelbase_ >=
                0.04
            ? 1
            : 0;
    return 2 * speed_bin +
           curvature_bin;
}

void HumanLikeAdaptiveTracker::tryStartTrial(
    double speed,
    double reference_curvature) {

    if (!enable_certified_learning_residual_ ||
        learning_rollback_latched_ ||
        trial_active_ ||
        trial_block_timer_ > 0.0 ||
        promotion_hold_timer_ <
            kTrialQualificationHoldTime) {
        return;
    }

    const std::size_t cell_index =
        performanceCellIndex(
            speed,
            std::fabs(reference_curvature));
    if (performance_cells_[cell_index].
            observation_time <
        kPerformanceMinimumObservationTime) {
        return;
    }
    // 已达到软误差目标时不再拿车辆做在线试验。候选继续在旁路更新，
    // 但零收益工况不会因为相位巧合被认证成长期偏置。
    if (performance_cells_[cell_index].
            certified_score <=
        learning_soft_error_ +
            0.20 * allowed_lateral_error_) {
        promotion_hold_timer_ = 0.0;
        return;
    }

    const double scale_difference =
        candidate_beta_scale_ -
        certified_beta_scale_;
    const double speed_difference =
        candidate_beta_speed_ -
        certified_beta_speed_;
    if (std::hypot(
            scale_difference,
            speed_difference) <
        0.25 * kTrialParameterStep) {
        promotion_hold_timer_ = 0.0;
        return;
    }
    const double local_parameter_difference =
        scale_difference +
        speed_difference *
            (2.0 * normalizedSpeed(speed) -
             1.0);
    if (std::fabs(
            certified_learning_residual_) >=
            kMaximumLearningResidual -
                1.0e-4 &&
        local_parameter_difference > 0.0) {
        promotion_hold_timer_ = 0.0;
        return;
    }

    trial_beta_scale_ =
        certified_beta_scale_ +
        clampValue(
            scale_difference,
            -kTrialParameterStep,
            kTrialParameterStep);
    trial_beta_speed_ =
        certified_beta_speed_ +
        clampValue(
            speed_difference,
            -0.5 * kTrialParameterStep,
            0.5 * kTrialParameterStep);
    trial_beta_scale_ =
        clampValue(
            trial_beta_scale_,
            -kShadowMaximumScaleMagnitude,
            kShadowMaximumScaleMagnitude);
    trial_beta_speed_ =
        clampValue(
            trial_beta_speed_,
            -kShadowMaximumSpeedMagnitude,
            kShadowMaximumSpeedMagnitude);

    trial_score_sum_ = 0.0;
    trial_reference_score_sum_ = 0.0;
    trial_reference_level_ =
        performance_cells_[cell_index].
            certified_score;
    trial_performance_cell_index_ =
        cell_index;
    trial_active_time_ = 0.0;
    promotion_hold_timer_ = 0.0;
    trial_active_ = true;
    ROS_WARN(
        "[%s] V0.7开始小步trial: beta(%.4f,%.4f) -> "
        "(%.4f,%.4f), candidate=(%.4f,%.4f)",
        getName().c_str(),
        certified_beta_scale_,
        certified_beta_speed_,
        trial_beta_scale_,
        trial_beta_speed_,
        candidate_beta_scale_,
        candidate_beta_speed_);
}

void HumanLikeAdaptiveTracker::acceptTrial() {
    if (!trial_active_) {
        return;
    }
    certified_beta_scale_ =
        trial_beta_scale_;
    certified_beta_speed_ =
        trial_beta_speed_;
    trial_active_ = false;
    trial_score_sum_ = 0.0;
    trial_reference_score_sum_ = 0.0;
    trial_reference_level_ = 0.0;
    trial_active_time_ = 0.0;
    trial_learning_residual_ = 0.0;
    trial_performance_cell_index_ = 0;
    trial_block_timer_ =
        kPostPromotionObservationTime;
    ++promotion_count_;
    ROS_WARN(
        "[%s] V0.7 trial通过轨迹性能认证: promotes=%zu, "
        "certified_beta=(%.4f,%.4f)",
        getName().c_str(),
        promotion_count_,
        certified_beta_scale_,
        certified_beta_speed_);
}

void HumanLikeAdaptiveTracker::rejectTrial(
    const char* reason) {

    (void)reason;
    if (!trial_active_) {
        return;
    }
    trial_beta_scale_ =
        certified_beta_scale_;
    trial_beta_speed_ =
        certified_beta_speed_;
    trial_active_ = false;
    trial_score_sum_ = 0.0;
    trial_reference_score_sum_ = 0.0;
    trial_reference_level_ = 0.0;
    trial_active_time_ = 0.0;
    trial_learning_residual_ = 0.0;
    trial_performance_cell_index_ = 0;
    promotion_hold_timer_ = 0.0;
    trial_block_timer_ = 5.0;
    learning_rollback_latched_ = true;
    ++rollback_count_;
    applied_learning_residual_ =
        certified_learning_residual_;
    ROS_ERROR_THROTTLE(
        1.0,
        "[%s] V0.7撤销trial并保留certified经验: %s, "
        "rollbacks=%zu",
        getName().c_str(),
        reason ? reason : "unknown",
        rollback_count_);
}

double
HumanLikeAdaptiveTracker::computeCertifiedLearningResidual(
    double preview_reference_curvature,
    double speed,
    double baseline_target) {

    certified_learning_residual_ = 0.0;
    trial_learning_residual_ = 0.0;
    if (!enable_certified_learning_residual_ ||
        !std::isfinite(
            preview_reference_curvature)) {
        applied_learning_residual_ = 0.0;
        return 0.0;
    }

    const double speed_coordinate =
        2.0 * normalizedSpeed(speed) - 1.0;
    const double prior_multiplier =
        1.0 / priorGainAtSpeed(speed);

    const auto feedforward_for =
        [&](double beta_scale,
            double beta_speed) {
            const double multiplier =
                prior_multiplier *
                std::exp(
                    beta_scale +
                    beta_speed *
                        speed_coordinate);
            return std::atan(
                wheelbase_ *
                preview_reference_curvature *
                multiplier);
        };

    const double prior_feedforward =
        feedforward_for(0.0, 0.0);
    const double certified_feedforward =
        feedforward_for(
            certified_beta_scale_,
            certified_beta_speed_);

    const double certified_absolute_limit =
        std::min(
            kMaximumLearningResidual,
            kInitialLearningResidualLimit +
                kLearningResidualGrowthPerPromotion *
                    static_cast<double>(
                        promotion_count_));
    const double certified_relative_limit =
        std::min(
            kMaximumRelativeLearningResidual,
            kInitialRelativeLearningLimit +
                kRelativeLearningGrowthPerPromotion *
                    static_cast<double>(
                        promotion_count_));
    const double certified_limit =
        std::min(
            certified_absolute_limit,
            certified_relative_limit *
                std::max(
                    0.03,
                    std::fabs(
                        baseline_target)));
    certified_learning_residual_ =
        clampValue(
            certified_feedforward -
                prior_feedforward,
            -certified_limit,
            certified_limit);

    if (trial_active_ &&
        !learning_rollback_latched_ &&
        std::fabs(
            preview_reference_curvature) >=
            kShadowMinimumReferenceCurvature) {
        const double trial_feedforward =
            feedforward_for(
                trial_beta_scale_,
                trial_beta_speed_);
        const double next_absolute_limit =
            std::min(
                kMaximumLearningResidual,
                certified_absolute_limit +
                    kLearningResidualGrowthPerPromotion);
        const double next_relative_limit =
            std::min(
                kMaximumRelativeLearningResidual,
                certified_relative_limit +
                    kRelativeLearningGrowthPerPromotion);
        const double next_total_limit =
            std::min(
                next_absolute_limit,
                next_relative_limit *
                    std::max(
                        0.03,
                        std::fabs(
                            baseline_target)));
        const double trial_total =
            clampValue(
                trial_feedforward -
                    prior_feedforward,
                -next_total_limit,
                next_total_limit);
        trial_learning_residual_ =
            clampValue(
                trial_total -
                    certified_learning_residual_,
                -kMaximumTrialResidualIncrement,
                kMaximumTrialResidualIncrement);
    }

    applied_learning_residual_ =
        certified_learning_residual_ +
        trial_learning_residual_;
    return applied_learning_residual_;
}

bool
HumanLikeAdaptiveTracker::lookupCommandSampleAtTime(
    double target_time,
    CommandSample* sample) const {

    if (!sample ||
        command_history_.empty()) {
        return false;
    }
    if (target_time <
        command_history_.front().time -
            1.0e-9) {
        return false;
    }
    if (target_time >=
        command_history_.back().time) {
        *sample =
            command_history_.back();
        return true;
    }

    for (std::size_t i = 1;
         i < command_history_.size();
         ++i) {
        const CommandSample& first =
            command_history_[i - 1];
        const CommandSample& second =
            command_history_[i];
        if (second.time < target_time) {
            continue;
        }
        const double interval =
            second.time - first.time;
        const double ratio =
            interval > 1.0e-9
                ? clampValue(
                      (target_time -
                       first.time) /
                          interval,
                      0.0,
                      1.0)
                : 0.0;
        sample->time =
            target_time;
        sample->commanded_curvature =
            first.commanded_curvature +
            ratio *
                (second.
                     commanded_curvature -
                 first.
                     commanded_curvature);
        sample->preview_reference_curvature =
            first.preview_reference_curvature +
            ratio *
                (second.
                     preview_reference_curvature -
                 first.
                     preview_reference_curvature);
        sample->speed =
            first.speed +
            ratio *
                (second.speed -
                 first.speed);
        sample->lateral_error =
            first.lateral_error +
            ratio *
                (second.lateral_error -
                 first.lateral_error);
        sample->heading_error =
            first.heading_error +
            ratio *
                (second.heading_error -
                 first.heading_error);
        return true;
    }
    return false;
}

bool HumanLikeAdaptiveTracker::lookupCommandAtTime(
    double target_time,
    double* commanded_curvature) const {

    if (!commanded_curvature) {
        return false;
    }
    CommandSample sample;
    if (!lookupCommandSampleAtTime(
            target_time,
            &sample)) {
        return false;
    }
    *commanded_curvature =
        sample.commanded_curvature;
    return true;
}

std::vector<double>
HumanLikeAdaptiveTracker::buildPredictionCommandHistory()
    const {

    std::vector<double> history(
        static_cast<std::size_t>(
            maximum_prediction_delay_steps_),
        0.0);
    for (int i = 0;
         i <
             maximum_prediction_delay_steps_;
         ++i) {
        const double age =
            static_cast<double>(
                maximum_prediction_delay_steps_ -
                i) *
            prediction_dt_;
        double command = 0.0;
        if (!lookupCommandAtTime(
                controller_time_ - age,
                &command)) {
            command =
                std::tan(
                    previous_steering_angle_) /
                wheelbase_;
        }
        history[
            static_cast<std::size_t>(i)] =
            command;
    }
    return history;
}

void HumanLikeAdaptiveTracker::appendCommandHistory(
    double commanded_curvature,
    double preview_reference_curvature,
    double speed,
    double lateral_error,
    double heading_error) {

    CommandSample sample;
    sample.time = controller_time_;
    sample.commanded_curvature =
        std::isfinite(commanded_curvature)
            ? commanded_curvature
            : 0.0;
    sample.preview_reference_curvature =
        std::isfinite(
            preview_reference_curvature)
            ? preview_reference_curvature
            : 0.0;
    sample.speed =
        std::isfinite(speed)
            ? std::fabs(speed)
            : 0.0;
    sample.lateral_error =
        std::isfinite(lateral_error)
            ? lateral_error
            : 0.0;
    sample.heading_error =
        std::isfinite(heading_error)
            ? heading_error
            : 0.0;
    command_history_.push_back(sample);

    const double history_duration =
        std::max(
            3.0,
            prior_delay_ +
                prediction_horizon_steps_ *
                    prediction_dt_ +
                1.0);
    while (command_history_.size() >
               2 &&
           command_history_[1].time <
               controller_time_ -
                   history_duration) {
        command_history_.pop_front();
    }
}

double HumanLikeAdaptiveTracker::filterYawRate(
    double yaw_rate,
    double dt) {

    if (!std::isfinite(yaw_rate)) {
        yaw_rate = 0.0;
    }
    if (!yaw_rate_filter_initialized_) {
        filtered_yaw_rate_ = yaw_rate;
        yaw_rate_filter_initialized_ = true;
        return filtered_yaw_rate_;
    }
    const double alpha =
        yaw_rate_filter_time_constant_ <=
                1.0e-6
            ? 1.0
            : dt /
                  (yaw_rate_filter_time_constant_ +
                   dt);
    filtered_yaw_rate_ +=
        clampValue(alpha, 0.0, 1.0) *
        (yaw_rate -
         filtered_yaw_rate_);
    return filtered_yaw_rate_;
}

double
HumanLikeAdaptiveTracker::filterMeasuredCurvature(
    double measured_curvature,
    double dt) {

    if (!std::isfinite(measured_curvature)) {
        measured_curvature = 0.0;
    }
    if (!curvature_filter_initialized_) {
        filtered_curvature_ =
            measured_curvature;
        curvature_filter_initialized_ =
            true;
        return filtered_curvature_;
    }
    const double alpha =
        curvature_filter_time_constant_ <=
                1.0e-6
            ? 1.0
            : dt /
                  (curvature_filter_time_constant_ +
                   dt);
    filtered_curvature_ +=
        clampValue(alpha, 0.0, 1.0) *
        (measured_curvature -
         filtered_curvature_);
    return filtered_curvature_;
}

void HumanLikeAdaptiveTracker::updateResponseGainObserver(
    double speed,
    double measured_curvature,
    double lateral_error,
    double heading_error,
    double dt) {

    const double effective_dt =
        sanitizeControlDt(dt);
    double curvature_rate = 0.0;
    if (response_gain_curvature_initialized_) {
        curvature_rate =
            (measured_curvature -
             response_gain_previous_curvature_) /
            effective_dt;
    }
    response_gain_previous_curvature_ =
        measured_curvature;
    response_gain_curvature_initialized_ =
        true;

    // 对齐纯时延后的指令，再用一阶逆响应
    // kappa_ss≈kappa+tau*kappa_dot 消除“执行器尚未到位看起来像
    // 低增益”的偏差。估计器不向车辆注入任何试探信号。
    CommandSample delayed_sample;
    CommandSample older_sample;
    const double observation_time =
        controller_time_ -
        prior_delay_;
    const double rate_interval =
        std::max(
            0.20,
            prior_response_time_);
    const bool have_delayed_sample =
        lookupCommandSampleAtTime(
            observation_time,
            &delayed_sample);
    const bool have_older_sample =
        lookupCommandSampleAtTime(
            observation_time -
                rate_interval,
            &older_sample);

    double command_rate =
        std::numeric_limits<double>::infinity();
    if (have_delayed_sample &&
        have_older_sample) {
        command_rate =
            (delayed_sample.commanded_curvature -
             older_sample.commanded_curvature) /
            rate_interval;
    }

    const double command_curvature =
        have_delayed_sample
            ? delayed_sample.commanded_curvature
            : 0.0;
    const bool trustworthy =
        have_delayed_sample &&
        have_older_sample &&
        std::fabs(speed) >=
            std::max(
                3.0,
                minimum_control_speed_) &&
        std::fabs(command_curvature) >=
            0.0040 &&
        command_curvature *
                measured_curvature >
            0.0 &&
        std::fabs(command_rate) <=
            0.0500 &&
        std::fabs(curvature_rate) <=
            0.1000 &&
        std::fabs(lateral_error) <=
            std::max(
                3.5,
                10.0 *
                    allowed_lateral_error_) &&
        std::fabs(heading_error) <=
            0.15;

    if (!trustworthy) {
        response_gain_observation_hold_ =
            std::max(
                0.0,
                response_gain_observation_hold_ -
                    2.0 *
                        effective_dt);
        return;
    }

    response_gain_observation_hold_ +=
        effective_dt;
    const double required_hold =
        std::max(
            0.10,
            0.40 *
                prior_response_time_);
    if (response_gain_observation_hold_ <
        required_hold) {
        return;
    }

    const double reconstruction_time =
        prior_response_time_ +
        1.00 *
            (yaw_rate_filter_time_constant_ +
             curvature_filter_time_constant_);
    const double reconstructed_steady_curvature =
        measured_curvature +
        reconstruction_time *
            curvature_rate;
    const double observed_gain =
        clampValue(
            reconstructed_steady_curvature /
                command_curvature,
            kMinimumResponseGain,
            kMaximumResponseGain);
    const double base_gain =
        priorGainAtSpeed(
            delayed_sample.speed);
    const double observed_scale =
        clampValue(
            observed_gain /
                std::max(
                    kMinimumResponseGain,
                    base_gain),
            0.60,
            1.50);

    // 保留V0.7已验证的快速单尺度跟随；V0.12不在已知最好
    // 基线上再改动这一主控制通道。
    const double low_pass_step =
        effective_dt /
            (0.20 +
             effective_dt) *
        (observed_scale -
         response_gain_scale_);
    const double bounded_step =
        clampValue(
            low_pass_step,
            -1.00 *
                effective_dt,
            1.00 *
                effective_dt);
    response_gain_scale_ =
        clampValue(
            response_gain_scale_ +
                bounded_step,
            0.60,
            1.50);
    response_gain_last_observation_ =
        observed_scale;
    response_gain_confidence_ =
        clampValue(
            response_gain_confidence_ +
                effective_dt /
                    3.0,
            0.0,
            1.0);
}

double
HumanLikeAdaptiveTracker::applyCorridorRecoverySupervisor(
    double lateral_error,
    double heading_error,
    double measured_curvature,
    double speed,
    double current_reference_curvature,
    const FixedControlModel& model,
    const BaselineControl& baseline,
    double dt) {

    const double effective_dt =
        sanitizeControlDt(dt);
    const double safe_speed =
        std::max(std::fabs(speed), 0.5);
    const double prediction_time =
        std::min(
            0.25,
            model.delay +
                0.20 * model.response_time);
    corridor_predicted_error_ =
        clampValue(
            lateral_error +
                clampValue(
                    prediction_time *
                        clampValue(
                            filtered_lateral_error_rate_,
                            -2.0,
                            2.0),
                    -0.25,
                    0.25),
            -3.0,
            3.0);

    const double entry_error =
        std::max(
            0.20,
            2.0 * allowed_lateral_error_);
    const double hard_entry_error =
        std::max(
            0.45,
            4.0 * allowed_lateral_error_);
    const double divergence =
        lateral_error *
        filtered_lateral_error_rate_;
    const bool corridor_is_diverging =
        std::fabs(corridor_predicted_error_) >=
            entry_error &&
        divergence >= 0.010;
    const bool corridor_is_already_large =
        std::fabs(lateral_error) >=
            hard_entry_error &&
        divergence >= 0.005;

    // 恢复权限不锁定：误差一旦开始回落，当前周期就撤销
    // active，然后只用0.20s连续退出混合。这避免恢复反馈在
    // 已经返回轨迹时仍长时间追赶，形成新的蛇行源。
    corridor_recovery_active_ =
        safe_speed >= minimum_control_speed_ &&
        (corridor_is_diverging ||
         corridor_is_already_large);
    corridor_recovery_release_hold_ = 0.0;

    const double error_level =
        clampValue(
            (std::fabs(corridor_predicted_error_) -
             entry_error) /
                std::max(
                    0.10,
                    hard_entry_error -
                        entry_error),
            0.0,
            1.0);
    const double divergence_level =
        clampValue(
            (divergence - 0.005) /
                0.050,
            0.0,
            1.0);
    const double requested_blend =
        corridor_recovery_active_
            ? clampValue(
                  0.25 +
                      0.75 *
                          std::max(
                              error_level,
                              divergence_level),
                  0.25,
                  1.0)
            : 0.0;
    const double blend_time_constant =
        requested_blend >
                corridor_recovery_blend_
            ? 0.10
            : 0.20;
    corridor_recovery_blend_ +=
        effective_dt /
            (blend_time_constant +
             effective_dt) *
        (requested_blend -
         corridor_recovery_blend_);
    corridor_recovery_blend_ =
        clampValue(
            corridor_recovery_blend_,
            0.0,
            1.0);

    // 恢复目标不改变V0.7前馈，只把误差与航向误差收敛距离
    // 缩短到可控的安全范围。误差变化率只进入上方的有界预测，
    // 因此误差已在回落时会自动减少修正，而不会在过零后继续追赶。
    const double recovery_distance =
        std::max(
            1.50 * wheelbase_,
            0.36 * safe_speed);
    const double yaw_prediction_time =
        model.delay +
        0.50 * model.response_time;
    const double predicted_heading_error =
        clampValue(
            heading_error +
                clampValue(
                    safe_speed *
                        yaw_prediction_time *
                        (current_reference_curvature -
                         measured_curvature),
                    -0.06,
                    0.06),
            -0.35,
            0.35);
    const double raw_recovery_curvature =
        2.0 *
            predicted_heading_error /
            recovery_distance +
        corridor_predicted_error_ /
            (recovery_distance *
             recovery_distance);
    const double maximum_recovery_steering =
        (0.32 +
         0.08 * normalizedSpeed(speed)) *
        max_steering_angle_;
    const double maximum_recovery_curvature =
        std::tan(
            maximum_recovery_steering) /
        wheelbase_;
    const double recovery_curvature =
        maximum_recovery_curvature *
        std::tanh(
            raw_recovery_curvature /
            std::max(
                maximum_recovery_curvature,
                1.0e-6));
    double recovery_feedback_steering =
        std::atan(
            wheelbase_ *
            recovery_curvature);

    // 同方向时恢复监督只能增强、不能削弱V0.7反馈。
    if (recovery_feedback_steering *
                baseline.feedback_steering >
            0.0 &&
        std::fabs(recovery_feedback_steering) <
            std::fabs(
                baseline.feedback_steering)) {
        recovery_feedback_steering =
            baseline.feedback_steering;
    }

    corridor_recovery_target_ =
        clampValue(
            baseline.feedforward_steering +
                recovery_feedback_steering,
            -max_steering_angle_,
            max_steering_angle_);
    // 返回值是影子反事实，computeControl明确丢弃它。保留完整候选
    // 而不是在这里限成0，是为了让CSV可以评估候选与V0.7的差值。
    return clampValue(
        baseline.target_steering +
            corridor_recovery_blend_ *
                (corridor_recovery_target_ -
                 baseline.target_steering),
        -max_steering_angle_,
        max_steering_angle_);
}

void HumanLikeAdaptiveTracker::writeSlidingModeCsv(
    double speed,
    const PathGeometry& geometry,
    double current_reference_curvature,
    double measured_curvature,
    const FixedControlModel& model,
    const SlidingModeControl& sliding,
    double steering_command,
    double dt) {

    if (!enable_csv_log_ ||
        !csv_log_stream_.is_open()) {
        return;
    }
    csv_log_timer_ += dt;
    csv_flush_timer_ += dt;
    if (csv_log_timer_ + 1.0e-9 <
        csv_log_period_) {
        return;
    }
    csv_log_timer_ =
        std::fmod(
            csv_log_timer_,
            csv_log_period_);

    csv_log_stream_
        << std::fixed
        << std::setprecision(7)
        << controller_time_ << ","
        << speed << ","
        << geometry.lateral_error << ","
        << sliding.predicted_lateral_error << ","
        << filtered_lateral_error_rate_ << ","
        << geometry.heading_error << ","
        << sliding.predicted_heading_error << ","
        << current_reference_curvature << ","
        << sliding.preview_reference_curvature << ","
        << measured_curvature << ","
        << model.response_gain << ","
        << actuator_time_constant_ << ","
        << sliding.lateral_response_time << ","
        << sliding.prediction_time << ","
        << sliding.convergence_length << ","
        << sliding.surface << ","
        << sliding.smooth_switch << ","
        << sliding.linear_feedback_curvature << ","
        << sliding.switching_curvature << ","
        << sliding.integral_curvature << ","
        << sliding.desired_vehicle_curvature << ","
        << sliding.dynamically_compensated_curvature << ","
        << sliding.target_steering << ","
        << steering_command << ","
        << previous_steering_rate_ << "\n";

    if (!csv_log_stream_) {
        ROS_ERROR_THROTTLE(
            1.0,
            "[%s] V0.14诊断CSV写入失败: %s",
            getName().c_str(),
            csv_log_path_.c_str());
        enable_csv_log_ = false;
        return;
    }
    if (csv_flush_timer_ >= 1.0) {
        csv_log_stream_.flush();
        csv_flush_timer_ =
            std::fmod(
                csv_flush_timer_,
                1.0);
    }
}

void HumanLikeAdaptiveTracker::writeDiagnosticCsv(
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
    double dt) {

    if (!enable_csv_log_ ||
        !csv_log_stream_.is_open()) {
        return;
    }

    const double effective_dt =
        sanitizeControlDt(dt);
    csv_log_timer_ += effective_dt;
    csv_flush_timer_ += effective_dt;
    if (csv_log_timer_ + 1.0e-9 <
        csv_log_period_) {
        return;
    }
    csv_log_timer_ =
        std::fmod(
            csv_log_timer_,
            csv_log_period_);

    csv_log_stream_
        << std::fixed
        << std::setprecision(6)
        << controller_time_ << ","
        << speed << ","
        << geometry.lateral_error << ","
        << filtered_lateral_error_rate_ << ","
        << geometry.heading_error << ","
        << current_reference_curvature << ","
        << raw_preview_curvature << ","
        << preview_reference_curvature << ","
        << measured_curvature << ","
        << model.response_gain << ","
        << response_gain_scale_ << ","
        << response_gain_last_observation_ << ","
        << baseline.feedforward_steering << ","
        << baseline.feedback_steering << ","
        << baseline_target << ","
        << corridor_recovery_target_ << ","
        << corridor_recovery_blend_ << ","
        << (corridor_recovery_active_ ? 1 : 0) << ","
        << corridor_predicted_error_ << ","
        << supervised_target << ","
        << steering_command << "\n";

    if (!csv_log_stream_) {
        ROS_ERROR_THROTTLE(
            1.0,
            "[%s] V0.12诊断CSV写入失败: %s",
            getName().c_str(),
            csv_log_path_.c_str());
        enable_csv_log_ = false;
        return;
    }
    if (csv_flush_timer_ >= 1.0) {
        csv_log_stream_.flush();
        csv_flush_timer_ =
            std::fmod(
                csv_flush_timer_,
                1.0);
    }
}

#if 0
// V0.4--V0.12遗留的CasADi实现保留在源文件中仅供历史比对。
// V0.14使用闭式连续滑模律，编译和运行均不依赖该代码。
bool HumanLikeAdaptiveTracker::buildPredictiveSolver() {
    try {
        opti_ = casadi::Opti();
        maximum_prediction_delay_steps_ =
            std::max(
                1,
                static_cast<int>(
                    std::ceil(
                        prior_delay_ /
                        prediction_dt_)));

        decision_curvature_ =
            opti_.variable(
                1,
                prediction_horizon_steps_);
        parameter_state_ =
            opti_.parameter(1, 1);
        parameter_reference_curvature_ =
            opti_.parameter(
                1,
                prediction_horizon_steps_);
        // [prediction_a, prediction_b]
        parameter_model_ =
            opti_.parameter(2, 1);
        parameter_delay_weights_ =
            opti_.parameter(
                1,
                maximum_prediction_delay_steps_ +
                    1);
        parameter_command_history_ =
            opti_.parameter(
                1,
                maximum_prediction_delay_steps_);
        parameter_previous_steering_ =
            opti_.parameter();
        parameter_previous_steering_rate_ =
            opti_.parameter();
        casadi::MX actual_curvature =
            parameter_state_(0);
        const casadi::MX prediction_a =
            parameter_model_(0);
        const casadi::MX prediction_b =
            parameter_model_(1);

        casadi::MX previous_steering =
            parameter_previous_steering_;
        casadi::MX previous_rate =
            parameter_previous_steering_rate_;
        casadi::MX objective = 0.0;
        const double maximum_curvature =
            std::tan(max_steering_angle_) /
            wheelbase_;
        const double curvature_tracking_scale =
            std::max(
                1.0e-3,
                0.05 *
                    maximum_curvature);

        for (int i = 0;
             i < prediction_horizon_steps_;
             ++i) {
            const casadi::MX desired_curvature =
                parameter_reference_curvature_(
                    0, i);
            const casadi::MX steering =
                atan(
                    wheelbase_ *
                    decision_curvature_(0, i));
            const casadi::MX steering_rate =
                (steering -
                 previous_steering) /
                prediction_dt_;
            const casadi::MX steering_acceleration =
                (steering_rate -
                 previous_rate) /
                prediction_dt_;

            opti_.subject_to(
                opti_.bounded(
                    -max_steering_angle_,
                    steering,
                    max_steering_angle_));
            opti_.subject_to(
                opti_.bounded(
                    -max_steering_rate_,
                    steering_rate,
                    max_steering_rate_));
            opti_.subject_to(
                opti_.bounded(
                    -max_steering_acceleration_,
                    steering_acceleration,
                    max_steering_acceleration_));

            objective +=
                squareMx(
                    steering_rate /
                        max_steering_rate_);
            objective +=
                squareMx(
                    steering_acceleration /
                        max_steering_acceleration_);
            objective +=
                0.02 *
                squareMx(
                    steering /
                        max_steering_angle_);

            casadi::MX applied_command =
                0.0;
            for (int delay_steps = 0;
                 delay_steps <=
                     maximum_prediction_delay_steps_;
                 ++delay_steps) {
                const int command_index =
                    i - delay_steps;
                casadi::MX delayed_value =
                    0.0;
                if (command_index >= 0) {
                    delayed_value =
                        decision_curvature_(
                            0,
                            command_index);
                } else {
                    const int history_index =
                        maximum_prediction_delay_steps_ +
                        command_index;
                    if (history_index >= 0) {
                        delayed_value =
                            parameter_command_history_(
                                0,
                                history_index);
                    } else {
                        delayed_value =
                            parameter_command_history_(
                                0,
                                0);
                    }
                }
                applied_command +=
                    parameter_delay_weights_(
                        0,
                        delay_steps) *
                    delayed_value;
            }

            const casadi::MX next_actual_curvature =
                prediction_a *
                    actual_curvature +
                prediction_b *
                    applied_command;
            objective +=
                squareMx(
                    (next_actual_curvature -
                     desired_curvature) /
                    curvature_tracking_scale);
            actual_curvature =
                next_actual_curvature;
            previous_steering = steering;
            previous_rate = steering_rate;
        }

        // 终端只加强目标曲率跟踪，不再让大位置误差直接压倒平顺项。
        objective +=
            4.0 *
            squareMx(
                (actual_curvature -
                 parameter_reference_curvature_(
                     0,
                     prediction_horizon_steps_ -
                         1)) /
                curvature_tracking_scale);

        opti_.minimize(objective);

        casadi::Dict options;
        options["print_time"] = false;
        options["ipopt.print_level"] = 0;
        options["ipopt.sb"] =
            std::string("yes");
        options["ipopt.max_iter"] =
            solver_max_iterations_;
        options["ipopt.max_cpu_time"] =
            solver_max_cpu_time_;
        options["ipopt.tol"] = 1.0e-3;
        options["ipopt.acceptable_tol"] =
            1.0e-2;
        options["ipopt.acceptable_iter"] = 3;
        options["ipopt.warm_start_init_point"] =
            std::string("yes");
        options["ipopt.warm_start_bound_push"] =
            1.0e-9;
        options["ipopt.warm_start_mult_bound_push"] =
            1.0e-9;
        options["ipopt.warm_start_slack_bound_push"] =
            1.0e-9;
        opti_.solver("ipopt", options);

        previous_decision_solution_ =
            casadi::DM::zeros(
                1,
                prediction_horizon_steps_);
        has_previous_solution_ = false;
        return true;
    } catch (const std::exception& exception) {
        ROS_ERROR(
            "[%s] CasADi预测控制器构建失败: %s",
            getName().c_str(),
            exception.what());
        has_previous_solution_ = false;
        return false;
    }
}

bool HumanLikeAdaptiveTracker::solvePredictiveControl(
    double measured_curvature,
    const std::vector<double>& desired_actual_curvature,
    const FixedControlModel& model,
    double* target_steering) {

    if (!target_steering ||
        !predictive_solver_initialized_ ||
        desired_actual_curvature.size() <
            static_cast<std::size_t>(
                prediction_horizon_steps_)) {
        return false;
    }

    try {
        const double response_time =
            clampValue(
                model.response_time,
                0.05,
                2.0);
        const double prediction_a =
            std::exp(
                -prediction_dt_ /
                response_time);
        const double prediction_b =
            (1.0 - prediction_a) *
            clampValue(
                model.response_gain,
                kMinimumResponseGain,
                kMaximumResponseGain);

        casadi::DM state =
            casadi::DM::zeros(1, 1);
        state(0) = measured_curvature;
        opti_.set_value(
            parameter_state_,
            state);

        casadi::DM reference =
            casadi::DM::zeros(
                1,
                prediction_horizon_steps_);
        for (int i = 0;
             i < prediction_horizon_steps_;
             ++i) {
            reference(0, i) =
                desired_actual_curvature[
                    static_cast<std::size_t>(i)];
        }
        opti_.set_value(
            parameter_reference_curvature_,
            reference);

        casadi::DM model_parameters =
            casadi::DM::zeros(2, 1);
        model_parameters(0) =
            prediction_a;
        model_parameters(1) =
            prediction_b;
        opti_.set_value(
            parameter_model_,
            model_parameters);

        casadi::DM delay_weights =
            casadi::DM::zeros(
                1,
                maximum_prediction_delay_steps_ +
                    1);
        const double continuous_delay_steps =
            clampValue(
                model.delay /
                    prediction_dt_,
                0.0,
                static_cast<double>(
                    maximum_prediction_delay_steps_));
        const int lower_delay =
            static_cast<int>(
                std::floor(
                    continuous_delay_steps));
        const int upper_delay =
            std::min(
                maximum_prediction_delay_steps_,
                lower_delay + 1);
        const double upper_weight =
            continuous_delay_steps -
            static_cast<double>(
                lower_delay);
        delay_weights(0, lower_delay) =
            1.0 - upper_weight;
        delay_weights(0, upper_delay) +=
            upper_weight;
        opti_.set_value(
            parameter_delay_weights_,
            delay_weights);

        const std::vector<double> history_values =
            buildPredictionCommandHistory();
        casadi::DM history =
            casadi::DM::zeros(
                1,
                maximum_prediction_delay_steps_);
        for (int i = 0;
             i <
                 maximum_prediction_delay_steps_;
             ++i) {
            history(0, i) =
                history_values[
                    static_cast<std::size_t>(i)];
        }
        opti_.set_value(
            parameter_command_history_,
            history);
        opti_.set_value(
            parameter_previous_steering_,
            previous_steering_angle_);
        opti_.set_value(
            parameter_previous_steering_rate_,
            previous_steering_rate_);

        if (has_previous_solution_) {
            casadi::DM shifted_solution =
                casadi::DM::zeros(
                    1,
                    prediction_horizon_steps_);
            for (int i = 0;
                 i + 1 <
                     prediction_horizon_steps_;
                 ++i) {
                shifted_solution(0, i) =
                    previous_decision_solution_(
                        0, i + 1);
            }
            shifted_solution(
                0,
                prediction_horizon_steps_ -
                    1) =
                previous_decision_solution_(
                    0,
                    prediction_horizon_steps_ -
                        1);
            opti_.set_initial(
                decision_curvature_,
                shifted_solution);
        } else {
            casadi::DM initial =
                casadi::DM::zeros(
                    1,
                    prediction_horizon_steps_);
            const double maximum_command =
                std::tan(
                    max_steering_angle_) /
                wheelbase_;
            const double safe_gain =
                std::max(
                    kMinimumResponseGain,
                    model.response_gain);
            for (int i = 0;
                 i <
                     prediction_horizon_steps_;
                 ++i) {
                initial(0, i) =
                    clampValue(
                        desired_actual_curvature[
                            static_cast<std::size_t>(
                                i)] /
                            safe_gain,
                        -maximum_command,
                        maximum_command);
            }
            opti_.set_initial(
                decision_curvature_,
                initial);
        }

        casadi::OptiSol solution =
            opti_.solve();
        previous_decision_solution_ =
            solution.value(
                decision_curvature_);
        has_previous_solution_ = true;

        const double first_curvature =
            static_cast<double>(
                previous_decision_solution_(
                    0, 0));
        *target_steering =
            clampValue(
                std::atan(
                    wheelbase_ *
                    first_curvature),
                -max_steering_angle_,
                max_steering_angle_);
        return std::isfinite(
            *target_steering);
    } catch (const std::exception& exception) {
        ROS_WARN_THROTTLE(
            1.0,
            "[%s] 预测控制求解失败，保持目标并回归固定基线: %s",
            getName().c_str(),
            exception.what());
        has_previous_solution_ = false;
        return false;
    }
}

#endif

bool HumanLikeAdaptiveTracker::buildPredictiveSolver() {
    predictive_solver_initialized_ = false;
    has_previous_solution_ = false;
    maximum_prediction_delay_steps_ = 0;
    return false;
}

bool HumanLikeAdaptiveTracker::solvePredictiveControl(
    double measured_curvature,
    const std::vector<double>& desired_actual_curvature,
    const FixedControlModel& model,
    double* target_steering) {

    (void)measured_curvature;
    (void)desired_actual_curvature;
    (void)model;
    (void)target_steering;
    return false;
}

double HumanLikeAdaptiveTracker::shapeSteeringCommand(
    double target_steering,
    double dt) {

    const double effective_dt =
        sanitizeControlDt(dt);
    target_steering =
        clampValue(
            target_steering,
            -max_steering_angle_,
            max_steering_angle_);

    if (!steering_state_initialized_) {
        previous_steering_angle_ = 0.0;
        previous_steering_rate_ = 0.0;
        steering_state_initialized_ = true;
    }

    // 连续双曲正切速率整形。它只实现物理速率/加速度约束，不再
    // 额外叠加一个二阶低通；小误差时线性靠近，大误差时平滑饱和，
    // 因而不会像sign滑模或bang-bang限幅器那样生成锯齿。
    const double steering_error =
        target_steering -
        previous_steering_angle_;
    const double command_follow_time =
        clampValue(
            0.50 *
                actuator_time_constant_,
            0.08,
            0.25);
    const double desired_rate =
        max_steering_rate_ *
        std::tanh(
            steering_error /
            std::max(
                1.0e-5,
                max_steering_rate_ *
                    command_follow_time));
    const double desired_acceleration =
        (desired_rate -
         previous_steering_rate_) /
        command_follow_time;
    const double bounded_acceleration =
        clampValue(
            desired_acceleration,
            -max_steering_acceleration_,
            max_steering_acceleration_);
    double new_rate =
        clampValue(
            previous_steering_rate_ +
                bounded_acceleration *
                    effective_dt,
            -max_steering_rate_,
            max_steering_rate_);
    double new_angle =
        previous_steering_angle_ +
        new_rate *
            effective_dt;

    if ((target_steering -
         previous_steering_angle_) *
            (target_steering -
             new_angle) <= 0.0 ||
        (std::fabs(steering_error) < 1.0e-5 &&
         std::fabs(new_rate) < 1.0e-3)) {
        new_angle = target_steering;
        new_rate = 0.0;
    }

    new_angle =
        clampValue(
            new_angle,
            -max_steering_angle_,
            max_steering_angle_);
    if ((new_angle >=
             max_steering_angle_ &&
         new_rate > 0.0) ||
        (new_angle <=
             -max_steering_angle_ &&
         new_rate < 0.0)) {
        new_rate = 0.0;
    }

    previous_steering_angle_ = new_angle;
    previous_steering_rate_ = new_rate;
    return previous_steering_angle_;
}

double HumanLikeAdaptiveTracker::sanitizeControlDt(
    double dt) const {

    if (!std::isfinite(dt) || dt <= 0.0) {
        dt = default_control_dt_;
    }
    return clampValue(
        dt,
        1.0e-4,
        max_control_dt_);
}

void HumanLikeAdaptiveTracker::resetTransientStates(
    bool reset_learning) {

    controller_time_ = 0.0;
    yaw_rate_filter_initialized_ = false;
    filtered_yaw_rate_ = 0.0;
    curvature_filter_initialized_ = false;
    filtered_curvature_ = 0.0;
    preview_curvature_filter_initialized_ =
        false;
    filtered_preview_curvature_ = 0.0;
    response_gain_observation_hold_ = 0.0;
    response_gain_curvature_initialized_ =
        false;
    response_gain_previous_curvature_ = 0.0;
    speed_state_initialized_ = false;
    previous_speed_ = 0.0;
    lateral_error_state_initialized_ = false;
    previous_lateral_error_ = 0.0;
    filtered_lateral_error_rate_ = 0.0;
    sliding_integral_curvature_ = 0.0;
    sliding_reference_sign_initialized_ = false;
    sliding_previous_reference_sign_ = 0.0;
    corridor_recovery_active_ = false;
    corridor_recovery_blend_ = 0.0;
    corridor_recovery_release_hold_ = 0.0;
    corridor_recovery_target_ = 0.0;
    corridor_predicted_error_ = 0.0;
    command_history_.clear();
    steering_state_initialized_ = false;
    previous_steering_angle_ = 0.0;
    previous_steering_rate_ = 0.0;
    solver_time_since_update_ =
        prediction_dt_;
    held_target_initialized_ = false;
    held_target_steering_ = 0.0;
    last_solver_succeeded_ = false;
    consecutive_solver_failures_ = 0;
    has_previous_solution_ = false;
    shadow_sample_timer_ = 0.0;
    learning_soft_suppressed_ = false;
    applied_learning_residual_ = 0.0;
    certified_learning_residual_ = 0.0;
    trial_learning_residual_ = 0.0;
    trial_beta_scale_ =
        certified_beta_scale_;
    trial_beta_speed_ =
        certified_beta_speed_;
    trial_active_ = false;
    trial_score_sum_ = 0.0;
    trial_reference_score_sum_ = 0.0;
    trial_reference_level_ = 0.0;
    trial_active_time_ = 0.0;
    trial_performance_cell_index_ = 0;
    promotion_hold_timer_ = 0.0;
    csv_log_timer_ = 0.0;
    csv_flush_timer_ = 0.0;

    if (reset_learning) {
        initializeShadowLearner();
        response_gain_scale_ = 1.0;
        response_gain_confidence_ = 0.0;
        response_gain_last_observation_ = 1.0;
    }
}

void HumanLikeAdaptiveTracker::writeZeroSteering(
    race_msgs::Control* control_msg,
    bool reset_transient_state) {

    if (!control_msg) {
        return;
    }
    if (reset_transient_state) {
        resetTransientStates(false);
    }
    control_msg->lateral.steering_angle = 0.0;
    control_msg->lateral.steering_angle_velocity = 0.0;
    control_msg->steering_mode =
        race_msgs::Control::FRONT_STEERING_MODE;
}

double HumanLikeAdaptiveTracker::normalizeAngle(
    double angle) {

    const double pi = std::acos(-1.0);
    while (angle > pi) {
        angle -= 2.0 * pi;
    }
    while (angle < -pi) {
        angle += 2.0 * pi;
    }
    return angle;
}

double HumanLikeAdaptiveTracker::clampValue(
    double value,
    double lower,
    double upper) {

    return std::max(
        lower,
        std::min(value, upper));
}

}  // namespace race_tracker

PLUGINLIB_EXPORT_CLASS(
    race_tracker::HumanLikeAdaptiveTracker,
    race_tracker::ControllerPluginBase)

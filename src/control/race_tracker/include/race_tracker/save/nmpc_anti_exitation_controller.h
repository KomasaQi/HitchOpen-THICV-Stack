#ifndef RACE_TRACKER_NMPC_ANTI_EXITATION_CONTROLLER_H
#define RACE_TRACKER_NMPC_ANTI_EXITATION_CONTROLLER_H

#include <casadi/casadi.hpp>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <string>
#include <vector>

#include "race_tracker/controller_plugin_base.h"
#include <race_msgs/Control.h>
#include <race_msgs/Flag.h>
#include <race_msgs/Path.h>
#include <race_msgs/VehicleStatus.h>

namespace race_tracker {

class NMPCAntiExitationController : public ControllerPluginBase {
public:
    NMPCAntiExitationController() = default;
    ~NMPCAntiExitationController() override = default;

    bool initialize(ros::NodeHandle& nh) override;

    void computeControl(
        const race_msgs::VehicleStatusConstPtr& vehicle_status,
        const race_msgs::PathConstPtr& path,
        race_msgs::Control* control_msg,
        const double dt,
        const race_msgs::Flag::ConstPtr& flag) override;

    std::string getName() const override { return "NMPCAntiExitationController"; }

private:
    struct SolverParams {
        int nx = 4;
        int nu = 1;
        int N = 16;
        int Nc = 4;

        double T_d1 = 0.18;
        double dt = 0.08;
        double L = 0.30;
        double gravity = 9.806;
        double ay_max = 1.80;

        double delta_min = -0.25;
        double delta_max = 0.25;

        double w_pos = 18.0;
        double w_theta = 28.0;
        double w_delta = 3.0;
        double w_delta_cmd = 0.3;
        double w_delta_cmd_rate = 0.5;
        double w_term_pos = 45.0;
        double w_term_theta = 50.0;
        double w_ddelta_cmd = 350.0;
        double w_first_delta_change = 120.0;

        bool use_sparse_control = true;
        bool use_linear_tan = true;
        bool use_heading_wrap_cost = true;
        bool shift_warm_start = true;
        int integrate_degree = 1;

        int ipopt_max_iter = 18;
        double ipopt_tol = 2e-3;
        double ipopt_acceptable_tol = 2e-2;
        int ipopt_acceptable_iter = 2;
        double ipopt_max_cpu_time = 0.045;
        bool print_solver_time = false;
        bool expand_nlp = true;

        // 路径重采样：按预测可达距离取参考点，避免低速追远点。
        bool use_speed_based_reference = true;
        double ref_min_step = 0.03;
        double ref_max_step = 0.35;
        double ref_speed_scale = 1.0;
        double ref_nearest_search_radius = 2.0;
        bool unwrap_reference_yaw = true;

        // 低速与失败降级：低速不用 NMPC，直接用 Pure Pursuit；中间速度软切换。
        bool enable_low_speed_pp = true;
        double pp_only_speed = 0.55;
        double nmpc_full_speed = 1.15;
        double pp_lookahead_min = 0.32;
        double pp_lookahead_gain = 0.34;
        double pp_lookahead_max = 1.10;
        double pp_max_steer_low_speed = 0.18;
        double pp_rate_limit = 0.20;       // rad per control cycle
        double output_rate_limit = 0.18;   // rad per control cycle
        double low_speed_delta_limit = 0.20;

        // 求解失败或超时时，是否退回 PP，而不是继续沿用上一帧。
        bool fallback_to_pp_on_fail = true;
    } p_;

private:
    template <typename T>
    void readParam(const ros::NodeHandle& nh, const std::string& name, T& value) const {
        const T default_value = value;
        nh.param(name, value, default_value);
        ROS_INFO_STREAM("[" << getName() << "] param " << name << " = " << value);
    }

    bool loadParams(ros::NodeHandle& nh_nmpc);
    void validateParams();
    void buildSingleShootingProblem();

    static double clamp(double v, double lo, double hi);
    static double wrapAngle(double a);
    static double nearestEquivalentAngle(double angle, double anchor);
    static double lerp(double a, double b, double t);

    double quaternion_to_yaw(const geometry_msgs::Quaternion& q) const;
    int find_nearest_path_point(double x0, double y0, const race_msgs::Path& path) const;

    casadi::DM process_race_path(const race_msgs::Path& input_path,
                                 const std::vector<double>& current_state,
                                 double vx_now) const;

    bool solveNMPC(const std::vector<double>& current_state,
                   double vx_now,
                   const casadi::DM& waypoints,
                   std::vector<double>& control_output);

    std::vector<double> vehicleStatusToStateVector(const race_msgs::VehicleStatus& status) const;
    casadi::DM makeInitialU() const;
    casadi::DM shiftInitialU(const casadi::DM& prev_u) const;

    double computePurePursuitSteer(const race_msgs::Path& path,
                                   const std::vector<double>& current_state,
                                   double vx_now) const;
    double applyRateLimit(double target, double previous, double max_step) const;
    double speedBlendRatio(double vx_now) const;

private:
    int control_dim_ = 4;
    std::vector<int> steps_per_control_;

    casadi::Opti opti_;
    casadi::MX U_decision_;
    casadi::MX x0_;
    casadi::MX vx0_;
    casadi::MX waypoints_;
    casadi::MX delta_bound_min_;
    casadi::MX delta_bound_max_;
    casadi::MX last_delta_cmd_param_;

    casadi::DM prev_U_;
    bool has_prev_sol_ = false;
    std::vector<double> last_control_output_{0.0};
};

} // namespace race_tracker

#endif // RACE_TRACKER_NMPC_ANTI_EXITATION_CONTROLLER_H

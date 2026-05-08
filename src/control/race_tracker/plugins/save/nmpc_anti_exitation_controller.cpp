#include "race_tracker/nmpc_anti_exitation_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ros/console.h>

using namespace casadi;

namespace race_tracker {

bool NMPCAntiExitationController::initialize(ros::NodeHandle& nh) {
    ROS_INFO("[NMPCAntiExitationController] parent namespace: %s", nh.getNamespace().c_str());

    ros::NodeHandle nh_nmpc(nh, "nmpc_anti_exitation_controller");
    ROS_INFO("[NMPCAntiExitationController] private namespace: %s", nh_nmpc.getNamespace().c_str());

    if (!loadParams(nh_nmpc)) return false;
    validateParams();
    buildSingleShootingProblem();

    has_prev_sol_ = false;
    prev_U_ = casadi::DM::zeros(1, control_dim_);
    last_control_output_ = {0.0};

    ROS_INFO("[%s] hybrid-fast NMPC initialized: N=%d, control_dim=%d, sparse=%s, low_speed_pp=%s",
             getName().c_str(), p_.N, control_dim_,
             p_.use_sparse_control ? "true" : "false",
             p_.enable_low_speed_pp ? "true" : "false");
    return true;
}

bool NMPCAntiExitationController::loadParams(ros::NodeHandle& nh) {
    readParam(nh, "nx", p_.nx);
    readParam(nh, "nu", p_.nu);
    readParam(nh, "prediction_step", p_.N);
    readParam(nh, "sparse_control_step", p_.Nc);

    readParam(nh, "front_steer_time_constant", p_.T_d1);
    readParam(nh, "sampling_time", p_.dt);
    readParam(nh, "wheelbase", p_.L);
    readParam(nh, "gravity", p_.gravity);
    readParam(nh, "max_lateral_acceleration", p_.ay_max);
    readParam(nh, "min_front_steer", p_.delta_min);
    readParam(nh, "max_front_steer", p_.delta_max);

    readParam(nh, "weight_position", p_.w_pos);
    readParam(nh, "weight_heading", p_.w_theta);
    readParam(nh, "weight_front_steer", p_.w_delta);
    readParam(nh, "weight_front_steer_control", p_.w_delta_cmd);
    readParam(nh, "weight_front_steer_rate", p_.w_delta_cmd_rate);
    readParam(nh, "weight_terminal_position", p_.w_term_pos);
    readParam(nh, "weight_terminal_heading", p_.w_term_theta);
    readParam(nh, "weight_steering_increment", p_.w_ddelta_cmd);
    readParam(nh, "weight_first_delta_change", p_.w_first_delta_change);

    readParam(nh, "use_sparse_control", p_.use_sparse_control);
    readParam(nh, "use_linear_tan", p_.use_linear_tan);
    readParam(nh, "use_heading_wrap_cost", p_.use_heading_wrap_cost);
    readParam(nh, "shift_warm_start", p_.shift_warm_start);
    readParam(nh, "integrate_degree", p_.integrate_degree);

    readParam(nh, "ipopt_max_iter", p_.ipopt_max_iter);
    readParam(nh, "ipopt_tol", p_.ipopt_tol);
    readParam(nh, "ipopt_acceptable_tol", p_.ipopt_acceptable_tol);
    readParam(nh, "ipopt_acceptable_iter", p_.ipopt_acceptable_iter);
    readParam(nh, "ipopt_max_cpu_time", p_.ipopt_max_cpu_time);
    readParam(nh, "print_solver_time", p_.print_solver_time);
    readParam(nh, "expand_nlp", p_.expand_nlp);

    readParam(nh, "use_speed_based_reference", p_.use_speed_based_reference);
    readParam(nh, "ref_min_step", p_.ref_min_step);
    readParam(nh, "ref_max_step", p_.ref_max_step);
    readParam(nh, "ref_speed_scale", p_.ref_speed_scale);
    readParam(nh, "ref_nearest_search_radius", p_.ref_nearest_search_radius);
    readParam(nh, "unwrap_reference_yaw", p_.unwrap_reference_yaw);

    readParam(nh, "enable_low_speed_pp", p_.enable_low_speed_pp);
    readParam(nh, "pp_only_speed", p_.pp_only_speed);
    readParam(nh, "nmpc_full_speed", p_.nmpc_full_speed);
    readParam(nh, "pp_lookahead_min", p_.pp_lookahead_min);
    readParam(nh, "pp_lookahead_gain", p_.pp_lookahead_gain);
    readParam(nh, "pp_lookahead_max", p_.pp_lookahead_max);
    readParam(nh, "pp_max_steer_low_speed", p_.pp_max_steer_low_speed);
    readParam(nh, "pp_rate_limit", p_.pp_rate_limit);
    readParam(nh, "output_rate_limit", p_.output_rate_limit);
    readParam(nh, "low_speed_delta_limit", p_.low_speed_delta_limit);
    readParam(nh, "fallback_to_pp_on_fail", p_.fallback_to_pp_on_fail);
    return true;
}

void NMPCAntiExitationController::validateParams() {
    p_.nx = 4;
    p_.nu = 1;
    p_.N = std::max(4, p_.N);
    p_.Nc = std::max(1, std::min(p_.Nc, p_.N));
    p_.dt = std::max(1e-3, p_.dt);
    p_.T_d1 = std::max(1e-3, p_.T_d1);
    p_.L = std::max(1e-3, p_.L);
    p_.ay_max = std::max(0.05, p_.ay_max);
    if (p_.delta_min > p_.delta_max) std::swap(p_.delta_min, p_.delta_max);

    p_.ref_min_step = std::max(1e-3, p_.ref_min_step);
    p_.ref_max_step = std::max(p_.ref_min_step, p_.ref_max_step);
    p_.pp_lookahead_min = std::max(0.05, p_.pp_lookahead_min);
    p_.pp_lookahead_max = std::max(p_.pp_lookahead_min, p_.pp_lookahead_max);
    p_.pp_only_speed = std::max(0.0, p_.pp_only_speed);
    p_.nmpc_full_speed = std::max(p_.pp_only_speed + 1e-3, p_.nmpc_full_speed);

    control_dim_ = p_.use_sparse_control ? p_.Nc : p_.N;
    steps_per_control_.assign(control_dim_, 1);
    if (p_.use_sparse_control) {
        steps_per_control_.assign(p_.Nc, 0);
        const int base = p_.N / p_.Nc;
        const int rem = p_.N % p_.Nc;
        for (int i = 0; i < p_.Nc; ++i) steps_per_control_[i] = base + (i < rem ? 1 : 0);
    }
}

void NMPCAntiExitationController::buildSingleShootingProblem() {
    opti_ = casadi::Opti();

    U_decision_ = opti_.variable(1, control_dim_);
    x0_ = opti_.parameter(p_.nx);
    vx0_ = opti_.parameter();
    waypoints_ = opti_.parameter(4, p_.N + 1);
    delta_bound_min_ = opti_.parameter();
    delta_bound_max_ = opti_.parameter();
    last_delta_cmd_param_ = opti_.parameter();

    std::vector<casadi::MX> u_full;
    u_full.reserve(p_.N);
    if (p_.use_sparse_control) {
        int k = 0;
        for (int i = 0; i < control_dim_; ++i) {
            for (int j = 0; j < steps_per_control_[i] && k < p_.N; ++j, ++k) {
                u_full.push_back(U_decision_(0, i));
            }
        }
    } else {
        for (int k = 0; k < p_.N; ++k) u_full.push_back(U_decision_(0, k));
    }

    opti_.subject_to(opti_.bounded(p_.delta_min, U_decision_, p_.delta_max));

    casadi::MX x = x0_(0);
    casadi::MX y = x0_(1);
    casadi::MX th = x0_(2);
    casadi::MX delta = x0_(3);
    casadi::MX cost = 0;

    auto steer_fun = [&](const casadi::MX& d) -> casadi::MX {
        return p_.use_linear_tan ? d : casadi::MX::tan(d);
    };

    auto heading_error = [&](const casadi::MX& raw) -> casadi::MX {
        return p_.use_heading_wrap_cost ? casadi::MX::atan2(casadi::MX::sin(raw), casadi::MX::cos(raw)) : raw;
    };

    for (int k = 0; k < p_.N; ++k) {
        const casadi::MX uk = u_full[k];
        const casadi::MX ex = x - waypoints_(0, k);
        const casadi::MX ey = y - waypoints_(1, k);
        const casadi::MX eth = heading_error(th - waypoints_(2, k));

        cost += p_.w_pos * (ex * ex + ey * ey);
        cost += p_.w_theta * eth * eth;
        cost += p_.w_delta * delta * delta;
        cost += p_.w_delta_cmd * uk * uk;
        if (k == 0) {
            cost += p_.w_first_delta_change * casadi::MX::pow(uk - last_delta_cmd_param_, 2);
        } else {
            cost += p_.w_ddelta_cmd * casadi::MX::pow(u_full[k] - u_full[k - 1], 2);
        }

        opti_.subject_to(opti_.bounded(delta_bound_min_, delta, delta_bound_max_));

        if (p_.integrate_degree >= 2) {
            const casadi::MX k1_th = vx0_ * steer_fun(delta) / p_.L;
            const casadi::MX k1_d = (uk - delta) / p_.T_d1;
            const casadi::MX th_mid = th + 0.5 * p_.dt * k1_th;
            const casadi::MX d_mid = delta + 0.5 * p_.dt * k1_d;
            x += p_.dt * vx0_ * casadi::MX::cos(th_mid);
            y += p_.dt * vx0_ * casadi::MX::sin(th_mid);
            th += p_.dt * vx0_ * steer_fun(d_mid) / p_.L;
            delta += p_.dt * (uk - d_mid) / p_.T_d1;
        } else {
            x += p_.dt * vx0_ * casadi::MX::cos(th);
            y += p_.dt * vx0_ * casadi::MX::sin(th);
            th += p_.dt * vx0_ * steer_fun(delta) / p_.L;
            delta += p_.dt * (uk - delta) / p_.T_d1;
        }
    }

    opti_.subject_to(opti_.bounded(delta_bound_min_, delta, delta_bound_max_));

    const casadi::MX exf = x - waypoints_(0, p_.N);
    const casadi::MX eyf = y - waypoints_(1, p_.N);
    const casadi::MX ethf = heading_error(th - waypoints_(2, p_.N));
    cost += p_.w_term_pos * (exf * exf + eyf * eyf);
    cost += p_.w_term_theta * ethf * ethf;

    if (p_.use_sparse_control) {
        for (int i = 1; i < control_dim_; ++i) {
            cost += p_.w_delta_cmd_rate * casadi::MX::pow(U_decision_(0, i) - U_decision_(0, i - 1), 2);
        }
    }

    opti_.minimize(cost);

    casadi::Dict solver_opts;
    solver_opts["print_time"] = p_.print_solver_time;
    solver_opts["expand"] = p_.expand_nlp;
    solver_opts["ipopt.print_level"] = 0;
    solver_opts["ipopt.sb"] = "yes";
    solver_opts["ipopt.max_iter"] = p_.ipopt_max_iter;
    solver_opts["ipopt.tol"] = p_.ipopt_tol;
    solver_opts["ipopt.acceptable_tol"] = p_.ipopt_acceptable_tol;
    solver_opts["ipopt.acceptable_iter"] = p_.ipopt_acceptable_iter;
    solver_opts["ipopt.max_cpu_time"] = p_.ipopt_max_cpu_time;
    solver_opts["ipopt.hessian_approximation"] = "limited-memory";
    solver_opts["ipopt.warm_start_init_point"] = "yes";
    solver_opts["ipopt.mu_strategy"] = "adaptive";

    opti_.solver("ipopt", solver_opts);
}

void NMPCAntiExitationController::computeControl(
    const race_msgs::VehicleStatusConstPtr& vehicle_status,
    const race_msgs::PathConstPtr& path,
    race_msgs::Control* control_msg,
    const double /*dt*/,
    const race_msgs::Flag::ConstPtr& /*flag*/) {

    if (!vehicle_status || !path || !control_msg) {
        ROS_ERROR("[%s] null input pointer", getName().c_str());
        return;
    }
    if (path->points.empty()) {
        ROS_WARN_THROTTLE(1.0, "[%s] empty path, keep previous output", getName().c_str());
        return;
    }

    const std::vector<double> current_state = vehicleStatusToStateVector(*vehicle_status);
    const double vx_raw = std::fabs(vehicle_status->vel.linear.x);
    const double vx_for_nmpc = std::max(0.03, vx_raw);

    const double pp_delta_raw = computePurePursuitSteer(*path, current_state, vx_raw);
    const double pp_delta = applyRateLimit(pp_delta_raw,
                                           last_control_output_.empty() ? 0.0 : last_control_output_[0],
                                           p_.pp_rate_limit);

    double control = pp_delta;
    const double blend = speedBlendRatio(vx_raw);

    if (p_.enable_low_speed_pp && blend <= 1e-6) {
        control = pp_delta;
    } else {
        const casadi::DM waypoints_dm = process_race_path(*path, current_state, vx_for_nmpc);
        std::vector<double> nmpc_output(1, 0.0);
        const bool ok = solveNMPC(current_state, vx_for_nmpc, waypoints_dm, nmpc_output);
        if (ok) {
            control = lerp(pp_delta, nmpc_output[0], blend);
        } else if (p_.fallback_to_pp_on_fail) {
            control = pp_delta;
            ROS_WARN_THROTTLE(0.5, "[%s] NMPC failed, fallback to PP steer %.6f", getName().c_str(), control);
        } else {
            control = last_control_output_.empty() ? pp_delta : last_control_output_[0];
            ROS_WARN_THROTTLE(0.5, "[%s] NMPC failed, using previous steer %.6f", getName().c_str(), control);
        }
    }

    const double speed_limit = (vx_raw < p_.nmpc_full_speed) ? p_.low_speed_delta_limit : p_.delta_max;
    control = clamp(control, -std::fabs(speed_limit), std::fabs(speed_limit));
    control = applyRateLimit(control, last_control_output_.empty() ? 0.0 : last_control_output_[0], p_.output_rate_limit);
    control = clamp(control, p_.delta_min, p_.delta_max);

    last_control_output_[0] = control;

    control_msg->lateral.steering_angle = control;
    control_msg->lateral.rear_wheel_angle = 0.0;
    control_msg->steering_mode = race_msgs::Control::FRONT_STEERING_MODE;
    control_msg->control_mode = race_msgs::Control::DES_ACCEL_ONLY;
}

double NMPCAntiExitationController::clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

double NMPCAntiExitationController::wrapAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

double NMPCAntiExitationController::nearestEquivalentAngle(double angle, double anchor) {
    return anchor + wrapAngle(angle - anchor);
}

double NMPCAntiExitationController::lerp(double a, double b, double t) {
    t = clamp(t, 0.0, 1.0);
    return a + t * (b - a);
}

double NMPCAntiExitationController::speedBlendRatio(double vx_now) const {
    if (!p_.enable_low_speed_pp) return 1.0;
    if (vx_now <= p_.pp_only_speed) return 0.0;
    if (vx_now >= p_.nmpc_full_speed) return 1.0;
    const double t = (vx_now - p_.pp_only_speed) / (p_.nmpc_full_speed - p_.pp_only_speed);
    return clamp(t * t * (3.0 - 2.0 * t), 0.0, 1.0); // smoothstep
}

double NMPCAntiExitationController::applyRateLimit(double target, double previous, double max_step) const {
    max_step = std::fabs(max_step);
    return previous + clamp(target - previous, -max_step, max_step);
}

double NMPCAntiExitationController::quaternion_to_yaw(const geometry_msgs::Quaternion& q) const {
    tf::Quaternion tf_quat(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 rot_matrix(tf_quat);
    double roll, pitch, yaw;
    rot_matrix.getRPY(roll, pitch, yaw);
    return yaw;
}

int NMPCAntiExitationController::find_nearest_path_point(double x0, double y0, const race_msgs::Path& path) const {
    if (path.points.empty()) return -1;
    double min_dist_sq = std::numeric_limits<double>::max();
    int nearest_idx = 0;
    for (size_t i = 0; i < path.points.size(); ++i) {
        const auto& pt = path.points[i].pose.position;
        const double dx = pt.x - x0;
        const double dy = pt.y - y0;
        const double d2 = dx * dx + dy * dy;
        if (d2 < min_dist_sq) {
            min_dist_sq = d2;
            nearest_idx = static_cast<int>(i);
        }
    }
    return nearest_idx;
}

casadi::DM NMPCAntiExitationController::process_race_path(
    const race_msgs::Path& input_path,
    const std::vector<double>& current_state,
    double vx_now) const {

    casadi::DM waypoints = casadi::DM::zeros(4, p_.N + 1);
    if (input_path.points.empty() || current_state.size() < 3) return waypoints;

    const int n = static_cast<int>(input_path.points.size());
    const int nearest_idx = find_nearest_path_point(current_state[0], current_state[1], input_path);
    if (nearest_idx < 0) return waypoints;

    if (!p_.use_speed_based_reference || n == 1) {
        for (int k = 0; k <= p_.N; ++k) {
            const int idx = std::min(nearest_idx + k, n - 1);
            const auto& pt = input_path.points[idx];
            waypoints(0, k) = pt.pose.position.x;
            waypoints(1, k) = pt.pose.position.y;
            double yaw_ref = quaternion_to_yaw(pt.pose.orientation);
            if (p_.unwrap_reference_yaw) {
                const double anchor = (k == 0) ? current_state[2] : static_cast<double>(waypoints(2, k - 1));
                yaw_ref = nearestEquivalentAngle(yaw_ref, anchor);
            }
            waypoints(2, k) = yaw_ref;
            waypoints(3, k) = pt.velocity;
        }
        return waypoints;
    }

    std::vector<double> s(n, 0.0);
    for (int i = 1; i < n; ++i) {
        const auto& p0 = input_path.points[i - 1].pose.position;
        const auto& p1 = input_path.points[i].pose.position;
        s[i] = s[i - 1] + std::hypot(p1.x - p0.x, p1.y - p0.y);
    }

    const double step_s = clamp(vx_now * p_.dt * p_.ref_speed_scale, p_.ref_min_step, p_.ref_max_step);
    const double s0 = s[nearest_idx];

    int seg = nearest_idx;
    for (int k = 0; k <= p_.N; ++k) {
        const double target_s = std::min(s0 + step_s * k, s.back());
        while (seg + 1 < n && s[seg + 1] < target_s) ++seg;
        const int i0 = std::min(seg, n - 1);
        const int i1 = std::min(seg + 1, n - 1);

        const auto& p0 = input_path.points[i0].pose.position;
        const auto& p1 = input_path.points[i1].pose.position;
        const double ds = std::max(1e-9, s[i1] - s[i0]);
        const double r = (i0 == i1) ? 0.0 : clamp((target_s - s[i0]) / ds, 0.0, 1.0);
        const double x = p0.x + r * (p1.x - p0.x);
        const double y = p0.y + r * (p1.y - p0.y);

        double yaw = 0.0;
        if (i1 > i0 && std::hypot(p1.x - p0.x, p1.y - p0.y) > 1e-6) {
            yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
        } else {
            yaw = quaternion_to_yaw(input_path.points[i0].pose.orientation);
        }

        if (p_.unwrap_reference_yaw) {
            const double anchor = (k == 0) ? current_state[2] : static_cast<double>(waypoints(2, k - 1));
            yaw = nearestEquivalentAngle(yaw, anchor);
        }

        waypoints(0, k) = x;
        waypoints(1, k) = y;
        waypoints(2, k) = yaw;
        waypoints(3, k) = input_path.points[i0].velocity;
    }
    return waypoints;
}

double NMPCAntiExitationController::computePurePursuitSteer(
    const race_msgs::Path& path,
    const std::vector<double>& current_state,
    double vx_now) const {

    if (path.points.empty() || current_state.size() < 3) return 0.0;

    const double x = current_state[0];
    const double y = current_state[1];
    const double yaw = current_state[2];
    const int nearest_idx = find_nearest_path_point(x, y, path);
    if (nearest_idx < 0) return 0.0;

    const double Ld = clamp(p_.pp_lookahead_min + p_.pp_lookahead_gain * vx_now,
                            p_.pp_lookahead_min, p_.pp_lookahead_max);

    int target_idx = nearest_idx;
    double acc_s = 0.0;
    for (int i = nearest_idx; i + 1 < static_cast<int>(path.points.size()); ++i) {
        const auto& p0 = path.points[i].pose.position;
        const auto& p1 = path.points[i + 1].pose.position;
        acc_s += std::hypot(p1.x - p0.x, p1.y - p0.y);
        target_idx = i + 1;
        if (acc_s >= Ld) break;
    }

    const auto& tp = path.points[target_idx].pose.position;
    const double dx = tp.x - x;
    const double dy = tp.y - y;
    const double local_x =  std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;

    if (std::hypot(local_x, local_y) < 1e-6) return 0.0;
    const double alpha = std::atan2(local_y, local_x);
    double delta = std::atan2(2.0 * p_.L * std::sin(alpha), Ld);
    const double lim = (vx_now < p_.nmpc_full_speed) ? p_.pp_max_steer_low_speed : p_.delta_max;
    return clamp(delta, -std::fabs(lim), std::fabs(lim));
}

casadi::DM NMPCAntiExitationController::makeInitialU() const {
    casadi::DM u0 = casadi::DM::zeros(1, control_dim_);
    const double last = last_control_output_.empty() ? 0.0 : last_control_output_[0];
    for (int i = 0; i < control_dim_; ++i) u0(0, i) = last;
    return u0;
}

casadi::DM NMPCAntiExitationController::shiftInitialU(const casadi::DM& prev_u) const {
    if (!p_.shift_warm_start || prev_u.is_empty() || prev_u.size2() != control_dim_) return prev_u;
    casadi::DM shifted = casadi::DM::zeros(1, control_dim_);
    for (int i = 0; i < control_dim_ - 1; ++i) shifted(0, i) = prev_u(0, i + 1);
    shifted(0, control_dim_ - 1) = prev_u(0, control_dim_ - 1);
    return shifted;
}

bool NMPCAntiExitationController::solveNMPC(const std::vector<double>& current_state,
                                            double vx_now,
                                            const casadi::DM& waypoints,
                                            std::vector<double>& control_output) {
    if (static_cast<int>(current_state.size()) != p_.nx || control_output.empty()) {
        ROS_ERROR("[%s] invalid input dimension", getName().c_str());
        return false;
    }

    try {
        opti_.set_value(x0_, current_state);
        opti_.set_value(vx0_, vx_now);
        opti_.set_value(waypoints_, waypoints);
        opti_.set_value(last_delta_cmd_param_, last_control_output_.empty() ? 0.0 : last_control_output_[0]);

        const double ay_delta_lim = p_.use_linear_tan
            ? p_.ay_max * p_.L / std::max(vx_now * vx_now, 1e-6)
            : std::atan(p_.ay_max * p_.L / std::max(vx_now * vx_now, 1e-6));
        double dmin = std::max(p_.delta_min, -std::fabs(ay_delta_lim));
        double dmax = std::min(p_.delta_max,  std::fabs(ay_delta_lim));
        if (vx_now < p_.nmpc_full_speed) {
            dmin = std::max(dmin, -std::fabs(p_.low_speed_delta_limit));
            dmax = std::min(dmax,  std::fabs(p_.low_speed_delta_limit));
        }
        opti_.set_value(delta_bound_min_, dmin);
        opti_.set_value(delta_bound_max_, dmax);

        const casadi::DM init_u = has_prev_sol_ ? shiftInitialU(prev_U_) : makeInitialU();
        opti_.set_initial(U_decision_, init_u);

        casadi::OptiSol sol = opti_.solve();
        prev_U_ = sol.value(U_decision_);
        has_prev_sol_ = true;

        control_output[0] = static_cast<double>(prev_U_(0, 0));
        control_output[0] = clamp(control_output[0], p_.delta_min, p_.delta_max);
        return true;
    } catch (const std::exception& e) {
        ROS_WARN("[%s] NMPC solve failed: %s", getName().c_str(), e.what());
        return false;
    }
}

std::vector<double> NMPCAntiExitationController::vehicleStatusToStateVector(const race_msgs::VehicleStatus& status) const {
    std::vector<double> state(p_.nx, 0.0);
    state[0] = status.pose.position.x;
    state[1] = status.pose.position.y;
    state[2] = wrapAngle(status.euler.yaw);
    state[3] = status.lateral.steering_angle;
    return state;
}

} // namespace race_tracker

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(race_tracker::NMPCAntiExitationController, race_tracker::ControllerPluginBase)

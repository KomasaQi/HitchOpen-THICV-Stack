#include "race_tracker/nmpc_accel_const_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <ros/console.h>

using namespace casadi;
using namespace std;

namespace race_tracker {

bool NMPCAccelConstController::initialize(ros::NodeHandle& nh) {
    ROS_INFO("[NMPCAccelConstController] 父NodeHandle命名空间: %s", nh.getNamespace().c_str());

    ros::NodeHandle nh_nmpc(nh, "nmpc_accel_const_controller");
    ROS_INFO("[NMPCAccelConstController] 插件专属NodeHandle命名空间: %s", nh_nmpc.getNamespace().c_str());

    nh_nmpc.param("nx", nx_, 4);
    nh_nmpc.param("nu", nu_, 1);

    nh_nmpc.param("prediction_step", N_, 20);
    nh_nmpc.param("sparse_control_step", Nc_, 4);

    nh_nmpc.param("front_steer_time_constant", T_d1_, 0.2);
    nh_nmpc.param("sampling_time", dt_, 0.1);
    nh_nmpc.param("wheelbase", L_, 1.3);
    nh_nmpc.param("gravity", g_, 9.806);
    nh_nmpc.param("max_lateral_acceleration", ay_max_, 1.5);

    nh_nmpc.param("min_front_steer", delta1_min_, -0.2);
    nh_nmpc.param("max_front_steer", delta1_max_, 0.2);

    nh_nmpc.param("weight_position", w_pos_, 25.0);
    nh_nmpc.param("weight_heading", w_theta_, 13.0);
    nh_nmpc.param("weight_front_steer", w_delta1_, 200.0);
    nh_nmpc.param("weight_front_steer_control", w_delta_cmd1_, 1.0);
    nh_nmpc.param("weight_front_steer_rate", w_delta_cmd_rate_, 10.0);
    nh_nmpc.param("weight_terminal_position", w_term_pos_, 20.0);
    nh_nmpc.param("weight_terminal_heading", w_term_theta_, 10.0);

    if (nx_ != 4) {
        ROS_WARN("[%s] 当前实现固定为 4 维状态 [x,y,theta,delta]，自动修正 nx=4", getName().c_str());
        nx_ = 4;
    }
    if (nu_ != 1) {
        ROS_WARN("[%s] 当前实现固定为 1 维控制 [delta_des]，自动修正 nu=1", getName().c_str());
        nu_ = 1;
    }
    if (N_ <= 0) {
        ROS_WARN("[%s] prediction_step 非法，自动修正 N=20", getName().c_str());
        N_ = 20;
    }
    if (Nc_ <= 0) {
        ROS_WARN("[%s] sparse_control_step 非法，自动修正 Nc=4", getName().c_str());
        Nc_ = 4;
    }
    if (Nc_ > N_) {
        ROS_WARN("[%s] Nc > N，不合理，自动修正 Nc=N", getName().c_str());
        Nc_ = N_;
    }

    // 均匀分配稀疏控制块
    steps_per_control_.assign(Nc_, 0);
    int base = N_ / Nc_;
    int rem = N_ % Nc_;
    for (int i = 0; i < Nc_; ++i) {
        steps_per_control_[i] = base + (i < rem ? 1 : 0);
    }

    // 动力学模型
    casadi::MX X_sym = casadi::MX::sym("X", nx_);   // [x,y,theta,delta]
    casadi::MX U_sym = casadi::MX::sym("U", nu_);   // [delta_des]
    casadi::MX vx_sym = casadi::MX::sym("vx");      // 常值速度参数

    casadi::MX f_expr = casadi::MX::vertcat({
        vx_sym * casadi::MX::cos(X_sym(2)),
        vx_sym * casadi::MX::sin(X_sym(2)),
        vx_sym * casadi::MX::tan(X_sym(3)) / L_,
        (U_sym(0) - X_sym(3)) / T_d1_
    });

    f_func_ = casadi::Function("f_dynamics", {X_sym, U_sym, vx_sym}, {f_expr});

    auto rk4_integrate = [&](const casadi::MX& x, const casadi::MX& u, const casadi::MX& vx, double dt) {
        casadi::MX k1 = f_func_({x, u, vx})[0];
        casadi::MX k2 = f_func_({x + dt / 2.0 * k1, u, vx})[0];
        casadi::MX k3 = f_func_({x + dt / 2.0 * k2, u, vx})[0];
        casadi::MX k4 = f_func_({x + dt * k3, u, vx})[0];
        return x + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    };

    // 构建优化问题
    opti_ = casadi::Opti();

    X_ = opti_.variable(nx_, N_ + 1);
    U_sparse_ = opti_.variable(nu_, Nc_);

    x0_ = opti_.parameter(nx_);
    vx0_ = opti_.parameter();
    waypoints_ = opti_.parameter(4, N_ + 1);

    // 稀疏控制展开
    casadi::MX U_full = casadi::MX::zeros(nu_, N_);
    int time_step = 0;
    for (int i = 0; i < Nc_; ++i) {
        int end_step = std::min(time_step + steps_per_control_[i], N_);
        U_full(casadi::Slice(), casadi::Slice(time_step, end_step)) =
            casadi::MX::repmat(U_sparse_(casadi::Slice(), i), 1, end_step - time_step);
        time_step = end_step;
        if (time_step >= N_) {
            break;
        }
    }

    // 初值约束
    opti_.subject_to(X_(casadi::Slice(), 0) == x0_);

    // 控制边界
    opti_.subject_to(opti_.bounded(delta1_min_, U_sparse_(0, casadi::Slice()), delta1_max_));

    // 动力学与约束
    for (int k = 0; k < N_; ++k) {
        opti_.subject_to(
            X_(casadi::Slice(), k + 1) ==
            rk4_integrate(X_(casadi::Slice(), k), U_full(casadi::Slice(), k), vx0_, dt_)
        );

        opti_.subject_to(opti_.bounded(delta1_min_, X_(3, k), delta1_max_));

        casadi::MX ay_k = vx0_ * vx0_ * casadi::MX::tan(X_(3, k)) / L_;
        opti_.subject_to(opti_.bounded(-ay_max_, ay_k, ay_max_));
    }

    opti_.subject_to(opti_.bounded(delta1_min_, X_(3, N_), delta1_max_));
    {
        casadi::MX ay_N = vx0_ * vx0_ * casadi::MX::tan(X_(3, N_)) / L_;
        opti_.subject_to(opti_.bounded(-ay_max_, ay_N, ay_max_));
    }

    // 代价函数
    casadi::MX cost = 0;

    for (int k = 0; k < N_; ++k) {
        casadi::MX ex = X_(0, k) - waypoints_(0, k);
        casadi::MX ey = X_(1, k) - waypoints_(1, k);

        casadi::MX e_theta_raw = X_(2, k) - waypoints_(2, k);
        casadi::MX e_theta = casadi::MX::atan2(casadi::MX::sin(e_theta_raw),
                                               casadi::MX::cos(e_theta_raw));

        cost += w_pos_ * (ex * ex + ey * ey);
        cost += w_theta_ * (e_theta * e_theta);
        cost += w_delta1_ * casadi::MX::sumsqr(X_(3, k));
        cost += w_delta_cmd1_ * casadi::MX::sumsqr(U_full(0, k));
    }

    for (int i = 1; i < Nc_; ++i) {
        cost += w_delta_cmd_rate_ * casadi::MX::sumsqr(U_sparse_(0, i) - U_sparse_(0, i - 1));
    }

    {
        int kf = N_;
        casadi::MX exf = X_(0, kf) - waypoints_(0, kf);
        casadi::MX eyf = X_(1, kf) - waypoints_(1, kf);

        casadi::MX e_theta_raw_f = X_(2, kf) - waypoints_(2, kf);
        casadi::MX e_theta_f = casadi::MX::atan2(casadi::MX::sin(e_theta_raw_f),
                                                 casadi::MX::cos(e_theta_raw_f));

        cost += w_term_pos_ * (exf * exf + eyf * eyf);
        cost += w_term_theta_ * (e_theta_f * e_theta_f);
    }

    opti_.minimize(cost);

    casadi::Dict solver_opts;
    solver_opts["ipopt.print_level"] = 0;
    solver_opts["print_time"] = false;
    solver_opts["ipopt.sb"] = "yes";
    solver_opts["ipopt.max_iter"] = 200;
    solver_opts["ipopt.tol"] = 1e-3;
    solver_opts["ipopt.acceptable_tol"] = 1e-2;
    solver_opts["ipopt.acceptable_iter"] = 8;
    solver_opts["ipopt.hessian_approximation"] = "limited-memory";
    solver_opts["ipopt.warm_start_init_point"] = "yes";
    // solver_opts["ipopt.linear_solver"] = "ma27";

    opti_.solver("ipopt", solver_opts);

    sol_prev_ = nullptr;
    has_prev_sol_ = false;
    last_control_output_ = {0.0};

    ROS_INFO("[%s] NMPC 控制器初始化完成", getName().c_str());
    return true;
}

void NMPCAccelConstController::computeControl(
    const race_msgs::VehicleStatusConstPtr& vehicle_status,
    const race_msgs::PathConstPtr& path,
    race_msgs::Control* control_msg,
    const double dt,
    const race_msgs::Flag::ConstPtr& flag) {

    if (!vehicle_status || !path || !control_msg) {
        ROS_ERROR("[%s] 输入参数为空指针", getName().c_str());
        return;
    }

    if (path->points.empty()) {
        ROS_WARN("[%s] 路径为空，不进行控制计算", getName().c_str());
        return;
    }

    std::vector<double> current_state = vehicleStatusToStateVector(*vehicle_status);

    double vx_now = std::fabs(vehicle_status->vel.linear.x);
    vx_now = std::max(0.1, vx_now);

    casadi::DM waypoints_dm = process_race_path(*path, current_state);

    std::vector<double> control_output(1, 0.0);

    if (!solveNMPC(current_state, vx_now, waypoints_dm, control_output)) {
        ROS_WARN("[%s] NMPC求解失败，使用上一次控制量: %.6f",
                 getName().c_str(),
                 last_control_output_.empty() ? 0.0 : last_control_output_[0]);

        if (!last_control_output_.empty()) {
            control_output = last_control_output_;
        } else {
            control_output[0] = 0.0;
        }
    } else {
        last_control_output_ = control_output;
    }

    control_msg->lateral.steering_angle = control_output[0];
    control_msg->lateral.rear_wheel_angle = 0.0;

    control_msg->steering_mode = race_msgs::Control::DUAL_STEERING_MODE;
    control_msg->control_mode = race_msgs::Control::DES_ACCEL_ONLY;
}

double NMPCAccelConstController::quaternion_to_yaw(const geometry_msgs::Quaternion& q) {
    tf::Quaternion tf_quat(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 rot_matrix(tf_quat);
    double roll, pitch, yaw;
    rot_matrix.getRPY(roll, pitch, yaw);
    return yaw;
}

int NMPCAccelConstController::find_nearest_path_point(double x0, double y0, const race_msgs::Path& path) {
    if (path.points.empty()) {
        ROS_ERROR("[%s] 路径为空", getName().c_str());
        return -1;
    }

    double min_dist_sq = std::numeric_limits<double>::max();
    int nearest_idx = 0;

    for (size_t i = 0; i < path.points.size(); ++i) {
        const auto& pt = path.points[i].pose.position;
        double dx = pt.x - x0;
        double dy = pt.y - y0;
        double dist_sq = dx * dx + dy * dy;

        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            nearest_idx = static_cast<int>(i);
        }
    }

    return nearest_idx;
}

casadi::DM NMPCAccelConstController::process_race_path(
    const race_msgs::Path& input_path,
    const std::vector<double>& current_state) {

    casadi::DM waypoints = casadi::DM::zeros(4, N_ + 1);

    if (input_path.points.empty()) {
        ROS_ERROR("[%s] 输入路径为空，返回零参考", getName().c_str());
        return waypoints;
    }

    if (current_state.size() < 2) {
        ROS_ERROR("[%s] 当前状态维度不足，返回零参考", getName().c_str());
        return waypoints;
    }

    double x0 = current_state[0];
    double y0 = current_state[1];

    int nearest_idx = find_nearest_path_point(x0, y0, input_path);
    if (nearest_idx < 0) {
        ROS_ERROR("[%s] 找不到最近路径点，返回零参考", getName().c_str());
        return waypoints;
    }

    for (int k = 0; k <= N_; ++k) {
        int idx = std::min(nearest_idx + k, static_cast<int>(input_path.points.size()) - 1);
        const auto& pt = input_path.points[idx];

        waypoints(0, k) = pt.pose.position.x;
        waypoints(1, k) = pt.pose.position.y;
        waypoints(2, k) = quaternion_to_yaw(pt.pose.orientation);
        waypoints(3, k) = pt.velocity;
    }

    return waypoints;
}

bool NMPCAccelConstController::solveNMPC(const std::vector<double>& current_state,
                                         double vx_now,
                                         const casadi::DM& waypoints,
                                         std::vector<double>& control_output) {
    if (static_cast<int>(current_state.size()) != nx_) {
        ROS_ERROR("[%s] 状态向量维度不匹配: 期望 %d, 实际 %zu",
                  getName().c_str(), nx_, current_state.size());
        return false;
    }

    if (control_output.size() < 1) {
        ROS_ERROR("[%s] control_output 大小不足", getName().c_str());
        return false;
    }

    try {
        opti_.set_value(x0_, current_state);
        opti_.set_value(vx0_, vx_now);
        opti_.set_value(waypoints_, waypoints);

        if (has_prev_sol_) {
            opti_.set_initial(X_, sol_prev_->value(X_));
            opti_.set_initial(U_sparse_, sol_prev_->value(U_sparse_));
        } else {
            opti_.set_initial(X_, casadi::DM::repmat(casadi::DM(current_state), 1, N_ + 1));
            opti_.set_initial(U_sparse_, casadi::DM::zeros(nu_, Nc_));
        }

        casadi::OptiSol sol_curr = opti_.solve();
        sol_prev_ = std::make_unique<casadi::OptiSol>(sol_curr);
        has_prev_sol_ = true;

        casadi::DM u0 = sol_curr.value(U_sparse_(casadi::Slice(), 0));
        control_output[0] = static_cast<double>(u0(0));

        return true;
    } catch (std::exception& e) {
        ROS_WARN("[%s] NMPC求解失败: %s", getName().c_str(), e.what());
        return false;
    }
}

std::vector<double> NMPCAccelConstController::vehicleStatusToStateVector(const race_msgs::VehicleStatus& status) {
    std::vector<double> state(nx_, 0.0);

    state[0] = status.pose.position.x;
    state[1] = status.pose.position.y;
    state[2] = status.euler.yaw;
    state[3] = status.lateral.steering_angle;

    return state;
}

} // namespace race_tracker

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(race_tracker::NMPCAccelConstController, race_tracker::ControllerPluginBase)
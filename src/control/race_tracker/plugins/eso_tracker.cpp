#include "race_tracker/eso_tracker.h"
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <numeric>
#include <limits>
#include <stdexcept>

#include <chrono>

using namespace casadi;
using namespace Eigen;
using namespace std;

namespace race_tracker {

// 构造函数：基本变量初始化
ESOTracker::ESOTracker() {
    ukf_x_est_ = Vector2d::Zero();
    ukf_P_est_ = (Matrix2d() << 1.0, 0.0, 0.0, 0.1).finished();
    rls_P_f_ = 1e5;
    rls_theta_f_ = 250000.0;
    rls_P_r_ = 1e5;
    rls_theta_r_ = 1000000.0;
    eso_x1_ = 0.0;
    eso_x2_ = 0.0;
    nmpc_counter_ = 4;
    current_cmd_ = 0.0;
    solver_.has_prev_sol = false;
    solver_.sol_prev = nullptr;
}

// 插件初始化（替代原flag=0）
bool ESOTracker::initialize(ros::NodeHandle& nh) {
    ros::NodeHandle nh_nmpc(nh, "nmpc_controller");
    ROS_INFO("[%s] NMPC 控制器命名空间: %s", getName().c_str(), nh_nmpc.getNamespace().c_str());

    // 从参数服务器加载 NMPC 核心参数
    nh_nmpc.param("m", nmpc_params_.m, 10000.0);
    nh_nmpc.param("Iz", nmpc_params_.Iz, 50000.0);
    nh_nmpc.param("lf", nmpc_params_.lf, 2.0);
    nh_nmpc.param("lr", nmpc_params_.lr, 2.135);
    nh_nmpc.param("prediction_step", nmpc_params_.N, 35);
    nh_nmpc.param("sparse_control_step", nmpc_params_.Nc, 5);
    nh_nmpc.param("sampling_time", nmpc_params_.dt, 0.05);
    nh_nmpc.param("max_steer", nmpc_params_.delta_max, 0.5);
    nh_nmpc.param("min_steer", nmpc_params_.delta_min, -0.5);

    // 重置辨识参数
    rls_Cf_est_ = 250000.0;
    rls_Cr_est_ = 1000000.0;
    rls_r_prev_ = 0.0;
    rls_r_dot_pre_ = 0.0;

    // 构建 CasADi 求解器
    buildNMPSolver();

    ROS_INFO("[%s] 控制器初始化完成", getName().c_str());
    return true;
}

// 核心控制循环（替代原flag=2离散更新和flag=3输出）
void ESOTracker::computeControl(
    const race_msgs::VehicleStatusConstPtr& vehicle_status,
    const race_msgs::PathConstPtr& path,
    race_msgs::Control* control_msg,
    const double dt,
    const race_msgs::Flag::ConstPtr& flag) {

    double curr_vx_raw = vehicle_status->vel.linear.x;//待机代码增加
    if (std::abs(curr_vx_raw) < 0.05) {
        ROS_INFO_THROTTLE(1.0, "[%s] 车辆尚未起步,NMPC 待机中...", getName().c_str());
        
        // 输出零转角或保持当前转角
        control_msg->lateral.steering_angle = 0.0; 
        control_msg->steering_mode = race_msgs::Control::FRONT_STEERING_MODE; 
        control_msg->control_mode = race_msgs::Control::DES_ACCEL_ONLY; 
        return; 
    }
    
    if (!vehicle_status || !path || !control_msg) {
        ROS_ERROR("[%s] 收到空指针消息", getName().c_str());
        return;
    }
    if (path->points.empty()) return;

    // 提取当前状态
    double curr_x = vehicle_status->pose.position.x;
    double curr_y = vehicle_status->pose.position.y;
    double curr_theta = vehicle_status->euler.yaw;
    double curr_vx = std::max(vehicle_status->vel.linear.x, 0.1); // 防零除
    double curr_ay = vehicle_status->acc.linear.y;
    double curr_r = vehicle_status->vel.angular.z;
    double curr_delta = vehicle_status->lateral.steering_angle;

    curr_delta = std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, curr_delta));//增加
    
    static ros::Time last_control_time = ros::Time(0);
    ros::Time current_time = ros::Time::now();

    // 如果是第一次运行，或者两次控制计算间隔超过了 0.2 秒（正常是 20Hz = 0.05s）
    if (last_control_time.toSec() != 0.0 && (current_time - last_control_time).toSec() > 0.2) {
        ROS_WARN("[%s] 检测到控制重连，清空历史记忆与热启动！", getName().c_str());
        
        // 1. 清空 NMPC 热启动
        solver_.has_prev_sol = false;
        solver_.sol_prev = nullptr;
        
        // 2. 将控制指令历史对齐到当前真实的物理方向盘转角
        current_cmd_ = curr_delta; 
        
        // 3. 复位 UKF 状态（防止协方差和侧向速度起飞）
        ukf_x_est_ = Eigen::Vector2d(0.0, curr_r);
        
        // 4. 复位 ESO 观测器（清空积累的错误扰动）
        eso_x1_ = curr_r; 
        eso_x2_ = 0.0;
    }
    last_control_time = current_time;//增加



    // 强制使用固定的观测器步长，或者直接使用传入的实际 dt
    const double obs_dt = std::max(dt, 0.01); 

    if (rls_r_prev_ == 0.0 && curr_r != 0.0) {
        rls_r_prev_ = curr_r;
    }

    // 1. UKF 横向速度估计
    ukfEstimateVy(curr_vx, curr_delta, curr_ay, curr_r, obs_dt);
    double vy_est = ukf_x_est_(0);

    // 2. RLS 侧偏刚度辨识
    rlsIdentifyStiffness(curr_vx, vy_est, curr_delta, curr_r, curr_ay, obs_dt);

    // 3. ESO 观测器计算
    esoCompute(curr_r, curr_delta, obs_dt);
    double h_hat_total_internal = eso_x2_;

    
    // 扰动纯化提取
    double alpha_f_curr = curr_delta - atan2((vy_est + nmpc_params_.lf * curr_r), curr_vx);
    double alpha_r_curr = -atan2((vy_est - nmpc_params_.lr * curr_r), curr_vx);
    double Fyf_curr = rls_Cf_est_ * alpha_f_curr;
    double Fyr_curr = rls_Cr_est_ * alpha_r_curr;

    double r_dot_nominal = (nmpc_params_.lf * Fyf_curr * cos(curr_delta) - nmpc_params_.lr * Fyr_curr) / nmpc_params_.Iz;
    double b_eso = (rls_Cf_est_ * nmpc_params_.lf) / nmpc_params_.Iz;
    double r_dot_actual = b_eso * curr_delta + h_hat_total_internal;
    double d_pure_trailer = r_dot_actual - r_dot_nominal;

    // 处理实时路径，生成参考轨迹 [x, y, theta, kappa]
    std::vector<double> current_pose = {curr_x, curr_y, curr_theta, curr_vx};
    casadi::DM waypoints_dm = process_race_path(*path, current_pose);

    // 调用求解器
    std::vector<double> nmpc_state = {curr_x, curr_y, curr_theta, vy_est, curr_r, curr_delta};
    std::vector<double> control_output(1);

    // 尝试求解，如果不成功则保留上一次的 current_cmd_
    if (solveNMPC(nmpc_state, waypoints_dm, control_output)) {
        current_cmd_ = control_output[0];
    }

    // 绑定 NMPC 模型运行时参数供下一次使用
    std::vector<double> dyn_params = {nmpc_params_.m, nmpc_params_.Iz, nmpc_params_.lf, 
                                      nmpc_params_.lr, rls_Cf_est_, rls_Cr_est_};
    solver_.opti.set_value(solver_.P_vx, curr_vx);
    solver_.opti.set_value(solver_.P_h_hat, d_pure_trailer);
    solver_.opti.set_value(solver_.P_dyn_params, dyn_params);


    // 5. 填装 ROS 消息输出
    control_msg->lateral.steering_angle = current_cmd_;
    control_msg->steering_mode = race_msgs::Control::FRONT_STEERING_MODE; 
    control_msg->control_mode = race_msgs::Control::DES_ACCEL_ONLY; 
}

// ---------------------- 路径处理与辅助函数 ----------------------
double ESOTracker::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

double ESOTracker::quaternion_to_yaw(const geometry_msgs::Quaternion& q) {
    tf::Quaternion tf_quat(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 rot_matrix(tf_quat);
    double roll, pitch, yaw;
    rot_matrix.getRPY(roll, pitch, yaw);
    return yaw;
}

int ESOTracker::find_nearest_path_point(const double x0, const double y0, const race_msgs::Path& path) {
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
    std::vector<double> s_orig, x_orig, y_orig, theta_orig, kappa_orig;
    
    for (int i = start_idx; i <= end_idx; ++i) {
        const auto& pt = path.points[i];
        s_orig.push_back(cum_dist[i - start_idx]);
        x_orig.push_back(pt.pose.position.x);
        y_orig.push_back(pt.pose.position.y);
        
        // 角度解卷绕逻辑
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

casadi::DM ESOTracker::process_race_path(const race_msgs::Path& input_path, const std::vector<double>& current_state) {
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
        if (cum_dist[i] > max_dist) { 
            end_idx = nearest_idx + i; 
            break; 
        }
        if (i == cum_dist.size() - 1) end_idx = nearest_idx + i;
    }
    end_idx = std::min(end_idx, static_cast<int>(input_path.points.size()) - 1);

    return interpolate_path_segment(input_path, cum_dist, nearest_idx, end_idx, s_target, current_state[2]);
}

// ---------------------- 核心算法 ----------------------

void ESOTracker::ukfEstimateVy(double curr_vx, double curr_delta, double curr_ay, double curr_r, double dt) {
    if (curr_vx < 3.0) {
        ukf_x_est_ = Eigen::Vector2d(0.0, curr_r);
        return; 
    }
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

    Matrix2d Q_ukf = (Matrix2d() << 0.01, 0.0, 0.0, 0.001).finished();
    Matrix2d R_ukf = (Matrix2d() << 0.5, 0.0, 0.0, 0.1).finished();

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
        double alpha_f = curr_delta - (vy_i + nmpc_params_.lf * r_i) / curr_vx;
        double alpha_r = -(vy_i - nmpc_params_.lr * r_i) / curr_vx;
        double Fyf = rls_Cf_est_ * alpha_f;
        double Fyr = rls_Cr_est_ * alpha_r;
        double vy_dot = (Fyf * cos(curr_delta) + Fyr) / nmpc_params_.m - curr_vx * r_i;
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
        double alpha_f = curr_delta - (vy_i + nmpc_params_.lf * r_i) / curr_vx;
        double alpha_r = -(vy_i - nmpc_params_.lr * r_i) / curr_vx;
        double ay_model = (rls_Cf_est_ * alpha_f * cos(curr_delta) + rls_Cr_est_ * alpha_r) / nmpc_params_.m;
        Z_sig(0, i) = ay_model;
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

    MatrixXd K = P_xz * P_zz.inverse();
    Vector2d z_meas(curr_ay, curr_r);
    ukf_x_est_ = x_pred + K * (z_meas - z_pred);
    ukf_P_est_ = P_pred - K * P_zz * K.transpose();
    ukf_P_est_ = 0.5 * (ukf_P_est_ + ukf_P_est_.transpose());
}

void ESOTracker::rlsIdentifyStiffness(double curr_vx, double vy_est, double curr_delta,
                                                double curr_r, double curr_ay, double dt) {
    if (curr_vx < 5.0) {
        rls_Cf_est_ = std::max(250000.0, 200000.0);
        rls_Cr_est_ = std::max(1e6, 1e6);
        rls_P_f_ = 1e5; rls_P_r_ = 1e5;
        return;
    }

    double r_dot = 0.2 * ((curr_r - rls_r_prev_) / dt) + 0.8 * rls_r_dot_pre_;
    rls_r_prev_ = curr_r; rls_r_dot_pre_ = r_dot;

    double m = nmpc_params_.m, Iz = nmpc_params_.Iz, lf = nmpc_params_.lf, lr = nmpc_params_.lr;
    double Fy_f_obs = (m * curr_ay * lr + Iz * r_dot) / (lf + lr);
    double Fy_r_obs = (m * curr_ay * lf - Iz * r_dot) / (lf + lr);

    double alpha_f = curr_delta - atan2((lf * curr_r) + vy_est, curr_vx);
    double alpha_r = -atan2((vy_est - (lr * curr_r)), curr_vx);

    if (abs(alpha_f) > 0.12 || abs(alpha_r) > 0.12) return;

    bool excite_flag = (abs(curr_ay) > 0.1);
    
    if (excite_flag && (abs(alpha_f) > 0.004)) {
        double phi = abs(alpha_f), e = abs(Fy_f_obs) - phi * rls_theta_f_;
        double K = (rls_P_f_ * phi) / (0.99 + phi * rls_P_f_ * phi);
        rls_theta_f_ += K * e;
        rls_P_f_ = (1.0 / 0.99) * (rls_P_f_ - K * phi * rls_P_f_) + 0.001;
    } else rls_P_f_ = std::min(rls_P_f_, 1e7);

    if (excite_flag && (abs(alpha_r) > 0.003)) {
        double phi = abs(alpha_r), e = abs(Fy_r_obs) - phi * rls_theta_r_;
        double K = (rls_P_r_ * phi) / (0.99 + phi * rls_P_r_ * phi);
        rls_theta_r_ += K * e;
        rls_P_r_ = (1.0 / 0.99) * (rls_P_r_ - K * phi * rls_P_r_) + 0.001;
    } else rls_P_r_ = std::min(rls_P_r_, 1e7);

    rls_Cf_est_ = std::max(200000.0, std::min(rls_theta_f_, 1e8));
    rls_Cr_est_ = std::max(1e6, std::min(rls_theta_r_, 5e8));
}

void ESOTracker::esoCompute(double curr_r, double curr_delta, double dt) {
    double b_eso = (rls_Cf_est_ * nmpc_params_.lf) / nmpc_params_.Iz;
    double error_eso = curr_r - eso_x1_;
    eso_x1_ += (b_eso * curr_delta + 20.0 * error_eso + eso_x2_) * dt;
    eso_x2_ += (100.0 * error_eso) * dt;
}

MX ESOTracker::vehicleDynamicsModel(const MX& state, const MX& cmd_delta,
                                              const MX& vx, const MX& h_dist, const MX& dyn_params) {
    MX theta = state(2), vy = state(3), r = state(4), delta = state(5);
    MX m_sym = dyn_params(0), Iz_sym = dyn_params(1), lf_sym = dyn_params(2), lr_sym = dyn_params(3);
    MX Cf_sym = dyn_params(4), Cr_sym = dyn_params(5);

    MX vx_safe = fmax(vx, 3.0); 

    MX alpha_f = delta - atan2((vy + lf_sym * r), vx_safe);
    MX alpha_r = -atan2((vy - lr_sym * r), vx_safe);
    MX Fyf = Cf_sym * alpha_f;
    MX Fyr = Cr_sym * alpha_r;

    MX d_vy = (Fyf * cos(delta) + Fyr) / m_sym - vx * r;
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
    solver_.P_waypoints = solver_.opti.parameter(4, N+1);  
    solver_.P_vx = solver_.opti.parameter(1);
    solver_.P_u_prev = solver_.opti.parameter(1);
    solver_.P_h_hat = solver_.opti.parameter(1);
    solver_.P_dyn_params = solver_.opti.parameter(6);  

    MX U_full = MX::zeros(nu, N);
    int base_steps = N / Nc, remainder = N % Nc, current_idx = 0;
    for (int i=0; i<Nc; i++) {
        int steps = base_steps + (i == Nc-1 ? remainder : 0);
        int end_idx = min(current_idx + steps, N);
        U_full(Slice(), Slice(current_idx, end_idx)) = repmat(solver_.U_sparse(Slice(), i), 1, end_idx - current_idx);
        current_idx = end_idx;
    }

    MX J = 0.0;
    solver_.opti.subject_to(solver_.X(Slice(), 0) == solver_.P_x0);

    for (int k=0; k<N; k++) {
        MX st = solver_.X(Slice(), k), con = U_full(Slice(), k);
        MX h = solver_.P_h_hat * pow(0.85, k);

       // MX k1 = vehicleDynamicsModel(st, con, solver_.P_vx, h, solver_.P_dyn_params);
       // MX k2 = vehicleDynamicsModel(st + nmpc_params_.dt/2 * k1, con, solver_.P_vx, h, solver_.P_dyn_params);
       // MX k3 = vehicleDynamicsModel(st + nmpc_params_.dt/2 * k2, con, solver_.P_vx, h, solver_.P_dyn_params);
       // MX k4 = vehicleDynamicsModel(st + nmpc_params_.dt * k3, con, solver_.P_vx, h, solver_.P_dyn_params);
      //solver_.opti.subject_to(solver_.X(Slice(), k+1) == st + nmpc_params_.dt/6 * (k1 + 2*k2 + 2*k3 + k4));

        MX k1 = vehicleDynamicsModel(st, con, solver_.P_vx, h, solver_.P_dyn_params);
        solver_.opti.subject_to(solver_.X(Slice(), k+1) == st + nmpc_params_.dt * k1);
        //MX k1 = vehicleDynamicsModel(st, con, solver_.P_vx, h, solver_.P_dyn_params);
       // MX k2 = vehicleDynamicsModel(st + nmpc_params_.dt / 2.0 * k1, con, solver_.P_vx, h, solver_.P_dyn_params);
        //solver_.opti.subject_to(solver_.X(Slice(), k+1) == st + nmpc_params_.dt * k2);

        MX ref_x = solver_.P_waypoints(0, k+1), ref_y = solver_.P_waypoints(1, k+1);
        MX ref_theta = solver_.P_waypoints(2, k+1), ref_kappa = solver_.P_waypoints(3, k+1);
        MX e_theta = atan2(sin(solver_.X(2, k+1) - ref_theta), cos(solver_.X(2, k+1) - ref_theta));

        J += nmpc_params_.Q(0,0) * (pow(solver_.X(0, k+1) - ref_x, 2) + pow(solver_.X(1, k+1) - ref_y, 2));
        J += nmpc_params_.Q(2,2) * pow(e_theta, 2);
        J += nmpc_params_.Q(4,4) * pow(solver_.X(4, k+1) - solver_.P_vx * ref_kappa, 2);
        J += nmpc_params_.Q(3,3) * pow(solver_.X(3, k+1), 2);
        J += nmpc_params_.R * pow(con, 2);
    }

    J += nmpc_params_.dR * pow(solver_.U_sparse(0) - solver_.P_u_prev, 2);
    for (int i=1; i<Nc; i++) J += nmpc_params_.dR * pow(solver_.U_sparse(i) - solver_.U_sparse(i-1), 2);

    solver_.opti.subject_to(solver_.opti.bounded(nmpc_params_.delta_min, solver_.U_sparse, nmpc_params_.delta_max));
   // solver_.opti.subject_to(solver_.opti.bounded(nmpc_params_.delta_min, solver_.X(5, Slice()), nmpc_params_.delta_max));

    solver_.opti.minimize(J);

    Dict opts = {
        {"ipopt.print_level", 0}, 
        {"ipopt.sb", "yes"}, 
        {"ipopt.max_iter", 100},              // 增加最大迭代次数
        {"ipopt.tol", 1e-2},                 // 放宽严格收敛容差
        {"ipopt.acceptable_tol", 5e-2},      // 只要误差在 5% 以内，也认为求解成功
        {"ipopt.acceptable_iter", 5},        // 如果次优状态维持了5次迭代，直接输出结果
        {"print_time", 0},

        {"ipopt.warm_start_init_point", "yes"},        // 明确告诉求解器接受热启动
        {"ipopt.warm_start_bound_push", 1e-9},         // 将变量向边界推的松弛量压到极小
        {"ipopt.warm_start_slack_bound_push", 1e-9},
        {"ipopt.warm_start_mult_bound_push", 1e-9},    // 乘子松弛量压到极小
        
        {"print_time", 0}
    };
    solver_.opti.solver("ipopt", opts);
}

bool ESOTracker::solveNMPC(const std::vector<double>& current_state, const casadi::DM& waypoints,
                                      std::vector<double>& control_output) {

    auto start_time = std::chrono::high_resolution_clock::now();//记录开始的时间戳

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
        ROS_INFO("[%s] NMPC 求解成功! 耗时: %.2f ms", getName().c_str(), elapsed.count());

        solver_.sol_prev = std::make_unique<casadi::OptiSol>(sol);
        solver_.has_prev_sol = true;

        control_output[0] = static_cast<double>(sol.value(solver_.U_sparse(0)));
        return true;
    } catch (std::exception& e) {

       auto end_time = std::chrono::high_resolution_clock::now();//求解失败的耗时
       std::chrono::duration<double, std::milli> elapsed = end_time - start_time;
       ROS_WARN("[%s] NMPC 求解失败! 耗时: %.2f ms, 原因: %s", getName().c_str(), elapsed.count(), e.what());
        
        solver_.has_prev_sol = false;
        solver_.sol_prev = nullptr;
        return false;
    }
}

} // namespace race_tracker

// 注册插件使其可以在ROS/Launch中通过 `controller_plugin_base` 动态加载
PLUGINLIB_EXPORT_CLASS(race_tracker::ESOTracker, race_tracker::ControllerPluginBase)

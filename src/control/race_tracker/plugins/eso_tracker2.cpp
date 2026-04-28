#include "race_tracker/eso_tracker2.h"
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

// 构造函数
ESOTracker2::ESOTracker2() {
    // EKF初始化
    ekf_x_hat_ = Vector4d::Zero();
    ekf_P_ = Matrix4d::Identity() * 1.0;
    // RLS初始化（FF-RLS）
    rls_P_ = Matrix3d::Identity() * 1e6;
    rls_C_out_prev_ = Vector3d(2e5, 4e5, 6e5);
    // 原ESO变量初始化
    eso_x1_ = 0.0;
    eso_x2_ = 0.0;

    nmpc_counter_ = 4; //？？
    // 挂车状态初始化
    gamma_ = 0.0;
    r_t_ = 0.0;
    // 原NMPC变量初始化
    current_cmd_ = 0.0;
    solver_.has_prev_sol = false;
    solver_.sol_prev = nullptr;
    // // 热启动与预测轨迹初始化
    // solver_.last_sol_X = DM::zeros(nmpc_params_.nx, nmpc_params_.N+1);
    // solver_.last_sol_U = DM::zeros(nmpc_params_.nu, nmpc_params_.Nc);
    // predicted_trajectory_ = DM::zeros(2, nmpc_params_.N+1);
}

// 插件初始化
bool ESOTracker2::initialize(ros::NodeHandle& nh) {
    ros::NodeHandle nh_nmpc(nh, "nmpc_controller");
    ROS_INFO("[%s] NMPC 控制器命名空间: %s", getName().c_str(), nh_nmpc.getNamespace().c_str());

    // 加载牵引车参数   
    nh_nmpc.param("m", nmpc_params_.m, 6595.0);
    nh_nmpc.param("Iz", nmpc_params_.Iz, 34806.2);
    nh_nmpc.param("lf", nmpc_params_.lf, 2.0);
    nh_nmpc.param("lr", nmpc_params_.lr, 2.135);
    // 加载挂车参数   
    nh_nmpc.param("m_t", nmpc_params_.m_t, 7570.0);
    nh_nmpc.param("Iz_t", nmpc_params_.Iz_t, 150000.0);
    nh_nmpc.param("lt", nmpc_params_.lt, 3.4);
    nh_nmpc.param("L2", nmpc_params_.L2, 7.9);
    nh_nmpc.param("M", nmpc_params_.M, nmpc_params_.m + nmpc_params_.m_t);
    // 加载原NMPC核心参数   1
    nh_nmpc.param("prediction_step", nmpc_params_.N, 35);
    nh_nmpc.param("sparse_control_step", nmpc_params_.Nc, 5);
    nh_nmpc.param("sampling_time", nmpc_params_.dt, 0.05);
    nh_nmpc.param("max_steer", nmpc_params_.delta_max, 0.5);
    nh_nmpc.param("min_steer", nmpc_params_.delta_min, -0.5);

    // 重置辨识参数（更新为挂车3个刚度）
    rls_Cf_est_ = 250000.0;
    rls_Cr_est_ = 1000000.0;
    rls_Ct_est_ = 400000.0;
    rls_w1_prev_ = 0.0;
    rls_w1_dot_prev_ = 0.0;

    // 构建CasADi求解器（保留原函数名）
    buildNMPSolver();
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

    // 提取当前状态（保留原逻辑）
    double curr_x = vehicle_status->pose.position.x;
    double curr_y = vehicle_status->pose.position.y;
    double curr_theta = vehicle_status->euler.yaw;
    double curr_vx = std::max(vehicle_status->vel.linear.x, 1.0); // 防零除
    double curr_ay = vehicle_status->acc.linear.y;
    double curr_r = vehicle_status->vel.angular.z;
    double curr_delta = vehicle_status->lateral.steering_angle;
    double M = nmpc_params_.m + nmpc_params_.m_t; // 总质量（新增）

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
        // ukf_x_est_ = Eigen::Vector2d(0.0, curr_r);
        
        // 4. 复位 ESO 观测器（清空积累的错误扰动）
        eso_x1_ = curr_r; 
        eso_x2_ = 0.0;
    }
    last_control_time = current_time;//增加

    // 强制使用固定的观测器步长，或者直接使用传入的实际 dt
    const double obs_dt = std::max(dt, 0.01);
    if (rls_w1_prev_ == 0.0 && curr_r != 0.0) {
        rls_w1_prev_ = curr_r;
    }

    // 1.处理实时路径，生成参考轨迹，并进行期望前轮转角计算
    std::vector<double> current_pose = {curr_x, curr_y, curr_theta, curr_vx};
    casadi::DM waypoints_dm = process_race_path(*path, current_pose);
    Vector2d p1(waypoints_dm(0,0), waypoints_dm(1,0));
    Vector2d p2(waypoints_dm(0,1), waypoints_dm(1,1));
    Vector2d p3(waypoints_dm(0,2), waypoints_dm(1,2));
    double tractor_L = nmpc_params_.lf + nmpc_params_.lr;
    double delta_f = calculate_curvature_and_steering(p1, p2, p3, tractor_L);

    // 2. 挂车运动学状态预估
    calculate_trailer_kinematics(delta_f, curr_vx, curr_r, obs_dt);
    double curr_gamma = -gamma_;
    double curr_r_t = r_t_;

    // 3. EKF横向速度估计
    ekfEstimateVy(curr_vx, curr_delta, curr_ay, curr_r, M, obs_dt);
    double vy_est = ekf_x_hat_(0);

    // 4. FF-RLS侧偏刚度辨识（替换原单车RLS，保留原函数名）
    rlsIdentifyStiffness(curr_vx, vy_est, curr_delta, curr_r, curr_ay, curr_gamma, curr_r_t, M, obs_dt);

    // 5. ESO观测器计算
    esoCompute(curr_r, curr_delta, obs_dt);
    double h_hat_total = eso_x2_ + (rls_Cf_est_ * nmpc_params_.lf) / nmpc_params_.Iz * curr_delta;
    double d_pure_trailer = h_hat_total;

    // 6. 构建8维状态向量，调用求解器
    std::vector<double> nmpc_state = {curr_x, curr_y, curr_theta, vy_est, curr_r, 
                                      curr_delta, curr_r_t, curr_gamma};
    std::vector<double> control_output(1);

    // 7. 尝试求解，如果不成功则保留上一次的 current_cmd_
    if (solveNMPC(nmpc_state, waypoints_dm, control_output)) {
        current_cmd_ = control_output[0];
    }

    // 8. 绑定NMPC运行时参数（扩展为10维）
    std::vector<double> dyn_params = {
        nmpc_params_.m, nmpc_params_.Iz, nmpc_params_.lf, nmpc_params_.lr,
        rls_Cf_est_, rls_Cr_est_, M, nmpc_params_.Iz_t, nmpc_params_.lt, rls_Ct_est_
    };
    solver_.opti.set_value(solver_.P_vx, curr_vx);
    solver_.opti.set_value(solver_.P_h_hat, d_pure_trailer);
    solver_.opti.set_value(solver_.P_dyn_params, dyn_params);

    // 9. 填装ROS消息输出（完全保留原逻辑）
    control_msg->lateral.steering_angle = current_cmd_;
    control_msg->steering_mode = race_msgs::Control::FRONT_STEERING_MODE; 
    control_msg->control_mode = race_msgs::Control::DES_ACCEL_ONLY; 
}

// ---------------------- 路径处理与辅助函数----------------------
double ESOTracker2::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

double ESOTracker2::quaternion_to_yaw(const geometry_msgs::Quaternion& q) {
    tf::Quaternion tf_quat(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 rot_matrix(tf_quat);
    double roll, pitch, yaw;
    rot_matrix.getRPY(roll, pitch, yaw);
    return yaw;
}

int ESOTracker2::find_nearest_path_point(const double x0, const double y0, const race_msgs::Path& path) {
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
        double ratio = (s_t - s_original[i]) / (s_original[i+1] - s_original[i]);
        val_target[k] = val_original[i] + ratio * (val_original[i+1] - val_original[i]);
    }
    return val_target;
}

casadi::DM ESOTracker2::interpolate_path_segment(const race_msgs::Path& path, const std::vector<double>& cum_dist, 
                                                         int start_idx, int end_idx,  const std::vector<double>& s_target, double yaw0) {
    std::vector<double> s_orig, x_orig, y_orig, theta_orig, kappa_orig;
    for (int i = start_idx; i <= end_idx; ++i) {
        const auto& pt = path.points[i];
        s_orig.push_back(cum_dist[i - start_idx]);
        x_orig.push_back(pt.pose.position.x);   
        y_orig.push_back(pt.pose.position.y);
        
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

casadi::DM ESOTracker2::process_race_path(const race_msgs::Path& input_path, const std::vector<double>& current_state) {
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
        if (cum_dist[i] > max_dist) { end_idx = nearest_idx + i; break; }
        if (i == cum_dist.size() - 1) end_idx = nearest_idx + i;
    }
    end_idx = std::min(end_idx, static_cast<int>(input_path.points.size()) - 1);

    return interpolate_path_segment(input_path, cum_dist, nearest_idx, end_idx, s_target, current_state[2]);
}

// ---------------------- 核心算法（替换为挂车逻辑） ----------------------
// EKF横向速度估计（解决命名空间冲突 + 完整变量声明）
void ESOTracker2::ekfEstimateVy(double curr_vx, double curr_delta, double curr_ay, double curr_r, double M, double dt) {
    double a = nmpc_params_.lf;
    double b = nmpc_params_.lr;
    double c = b;
    double d = nmpc_params_.lt;
    double L2 = nmpc_params_.L2;
    double m1 = nmpc_params_.m;
    double m2 = M - m1;
    double Iz1 = nmpc_params_.Iz;
    double Iz2 = nmpc_params_.Iz_t + 5*(m2/nmpc_params_.m_t);
    double Cf = rls_Cf_est_;
    double Cr = rls_Cr_est_;
    double Ct = rls_Ct_est_;

    // --- 显式使用 Eigen:: 前缀，避免与 casadi::Matrix 冲突 ---
    Eigen::Matrix3d M_mat;
    M_mat << m1 + m2,  -m2*c,        -m2*d,
            -m2*c,     Iz1 + m2*c*c,  m2*c*d,
            -m2*d,     m2*c*d,        Iz2 + m2*d*d;

    Eigen::Matrix<double, 3, 4> K;
    K(0,0) = -(Cf+Cr+Ct)/curr_vx;
    K(0,1) = -(a*Cf - b*Cr - c*Ct)/curr_vx - (m1+m2)*curr_vx;
    K(0,2) = (L2*Ct)/curr_vx;
    K(0,3) = -Ct;
    K(1,0) = -(a*Cf - b*Cr - c*Ct)/curr_vx;
    K(1,1) = -(a*a*Cf + b*b*Cr + c*c*Ct)/curr_vx + m2*c*curr_vx;
    K(1,2) = -(c*L2*Ct)/curr_vx;
    K(1,3) = c*Ct;
    K(2,0) = (L2*Ct)/curr_vx;
    K(2,1) = -(c*L2*Ct)/curr_vx + m2*d*curr_vx;
    K(2,2) = -(L2*L2*Ct)/curr_vx;
    K(2,3) = L2*Ct;

    Eigen::Matrix<double, 3, 1> D;
    D << Cf, a*Cf, 0.0;

    Eigen::Matrix4d A_sys;
    A_sys.block(0,0,3,4) = M_mat.inverse() * K;
    A_sys.row(3) << 0, 1, -1, 0;
    Eigen::Vector4d B_sys;
    B_sys.block(0,0,3,1) = M_mat.inverse() * D;
    B_sys(3) = 0.0;

    // EKF预测步骤
    Eigen::Vector4d x_dot = A_sys * ekf_x_hat_ + B_sys * curr_delta;
    Eigen::Vector4d x_pred = ekf_x_hat_ + x_dot * dt;
    Eigen::Matrix4d Phi = Eigen::Matrix4d::Identity() + A_sys * dt;
    Eigen::Matrix4d Q;
    Q << 0.005,0,0,0, 0,0.01,0,0, 0,0,0.05,0, 0,0,0,0.01;
    Eigen::Matrix4d P_pred = Phi * ekf_P_ * Phi.transpose() + Q;

    // --- 完整定义 H_sys 和 K_gain，确保在作用域内 ---
    Eigen::Matrix<double, 2, 4> H_sys;
    // 关键修正：使用 RowVector4d（行向量）
    H_sys.row(0) = A_sys.row(0) + Eigen::RowVector4d(0, curr_vx, 0, 0); 
    H_sys.row(1) << 0, 1, 0, 0; // 观测r
    
    Eigen::Vector2d D_obs;
    D_obs << B_sys(0) * curr_delta, 0.0;

    Eigen::Vector2d z_pred = H_sys * x_pred + D_obs;
    Eigen::Vector2d z_meas;
    z_meas << curr_ay, curr_r;
    Eigen::Matrix2d R;
    R << 1.0,0, 0,0.05;
    
    // 完整定义 K_gain
    Eigen::Matrix<double, 4, 2> K_gain = P_pred * H_sys.transpose() * (H_sys * P_pred * H_sys.transpose() + R).inverse();

    // 状态与协方差更新
    ekf_x_hat_ = x_pred + K_gain * (z_meas - z_pred);
    ekf_P_ = (Eigen::Matrix4d::Identity() - K_gain * H_sys) * P_pred;
}

// FF-RLS侧偏刚度辨识（保留原函数名，替换为3刚度辨识）
void ESOTracker2::rlsIdentifyStiffness(double curr_vx, double vy_est, double curr_delta,double curr_r, 
                                      double curr_ay, double curr_gamma, double curr_r_t, double M, double dt) {
    if (curr_vx < 5.0) {
        rls_Cf_est_ = 250000.0;
        rls_Cr_est_ = 1000000.0;
        rls_Ct_est_ = 400000.0;
        rls_P_ = Matrix3d::Identity() * 1e6;
        return;
    }

    double lamda = 0.995;
    double lf1 = nmpc_params_.lf;
    double lr1 = nmpc_params_.lr;
    double lf2 = nmpc_params_.lt;
    double lr2 = nmpc_params_.L2 - lf2;
    double lh = nmpc_params_.lh;
    double m1 = nmpc_params_.m;
    double m2 = M - m1;
    double Iz1 = nmpc_params_.Iz;
    double Iz2 = nmpc_params_.Iz_t + 5*(m2/nmpc_params_.m_t);

    // 横摆角加速度滤波
    double alpha_filter = 0.3;
    double w1_dot_raw = (curr_r - rls_w1_prev_) / dt;
    double w2_dot_raw = (curr_r_t - rls_w2_prev_) / dt;
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

    Vector3d Fy = A.inverse() * B;

    // 计算侧偏角
    Matrix2d rot;
    rot << cos(curr_gamma), sin(curr_gamma),
          -sin(curr_gamma), cos(curr_gamma);
    Vector2d b_vec;
    b_vec << lf2*curr_r_t*sin(curr_gamma) - curr_vx,
             vy_est - lh*curr_r - lf2*curr_r_t*cos(curr_gamma);
    double vx1_safe = max(curr_vx, 1.0);
    Vector2d V2 = rot.inverse() * b_vec;
    double V2_x_safe = max(V2(0), 1.0);

    double alpha1 = atan((vy_est + lf1*curr_r)/vx1_safe) - curr_delta;
    double alpha2 = atan((vy_est - lr1*curr_r)/vx1_safe);
    double alpha3 = -atan((V2(1) - lr2*curr_r_t)/V2_x_safe);
    Vector3d alpha(alpha1, alpha2, alpha3);

    // RLS递推
    double max_alpha = alpha.cwiseAbs().maxCoeff();
    if (max_alpha > 1e-4 && max_alpha < 0.15 && abs(curr_ay) > 0.1) {
        Vector3d Y = Fy.cwiseAbs();
        Matrix3d Phi = alpha.cwiseAbs().asDiagonal();

        Matrix3d K = rls_P_ * Phi.transpose() * (lamda*Matrix3d::Identity() + Phi*rls_P_*Phi.transpose()).inverse();
        Vector3d error = Y - Phi * Vector3d(rls_Cf_est_, rls_Cr_est_, rls_Ct_est_);
        
        rls_Cf_est_ += K(0,0)*error(0) + K(0,1)*error(1) + K(0,2)*error(2);
        rls_Cr_est_ += K(1,0)*error(0) + K(1,1)*error(1) + K(1,2)*error(2);
        rls_Ct_est_ += K(2,0)*error(0) + K(2,1)*error(1) + K(2,2)*error(2);

        // 物理限幅
        rls_Cf_est_ = max(min(rls_Cf_est_, 5e5), 0.5e5);
        rls_Cr_est_ = max(min(rls_Cr_est_, 20e5), 1.0e5);
        rls_Ct_est_ = max(min(rls_Ct_est_, 18e5), 1.0e5);

        rls_P_ = (Matrix3d::Identity() - K*Phi) * rls_P_ / lamda;
    }

    // 输出平滑
    double smooth_factor = 0.1;
    rls_C_out_prev_ = smooth_factor * Vector3d(rls_Cf_est_, rls_Cr_est_, rls_Ct_est_) + (1 - smooth_factor) * rls_C_out_prev_;
    rls_Cf_est_ = rls_C_out_prev_(0);
    rls_Cr_est_ = rls_C_out_prev_(1);
    rls_Ct_est_ = rls_C_out_prev_(2);
}

// ESO观测器（保留原函数名，更新为Matlab实现）
void ESOTracker2::esoCompute(double curr_r, double curr_delta, double dt) {
    double Iz1 = nmpc_params_.Iz;
    double Cf = rls_Cf_est_;
    double lf = nmpc_params_.lf;
    double B1 = 20.0;
    double B2 = 100.0;

    double b_eso = (Cf * lf) / Iz1;
    double error_eso = curr_r - eso_x1_;

    eso_x1_ += (b_eso * curr_delta + B1 * error_eso + eso_x2_) * dt;
    eso_x2_ += (B2 * error_eso) * dt;
}

// 预瞄曲率与运动学转角（新增内部函数）
double ESOTracker2::calculate_curvature_and_steering(const Vector2d& p1, const Vector2d& p2, 
                                                  const Vector2d& p3, double L1) {
    double x1 = p1(0), y1 = p1(1);
    double x2 = p2(0), y2 = p2(1);
    double x3 = p3(0), y3 = p3(1);

    double a = sqrt(pow(x2-x1,2) + pow(y2-y1,2));
    double b = sqrt(pow(x3-x2,2) + pow(y3-y2,2));
    double c = sqrt(pow(x3-x1,2) + pow(y3-y1,2));

    if (a < 1e-6 || b < 1e-6 || c < 1e-6) return 0.0;

    double cross_prod = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    double kappa = (2 * cross_prod) / (a * b * c);
    return atan(L1 * kappa);
}

// 挂车运动学状态预估（新增内部函数）
void ESOTracker2::calculate_trailer_kinematics(double delta_f, double curr_vx, double curr_r, double dt) {
    double L1 = nmpc_params_.lf + nmpc_params_.lr;
    double L2 = nmpc_params_.L2;
    double Lh = 0.0;
    double vx = max(curr_vx, 0.5);
    double Tau = 0.15;
    double K = 1.0;

    double r_raw = (vx * tan(delta_f)) / L1;
    double alpha = dt / (Tau + dt);
    double r_filtered = (1 - alpha) * curr_r + alpha * K * r_raw;

    r_t_ = (vx * sin(gamma_) + Lh * r_filtered * cos(gamma_)) / L2;
    double gamma_dot = r_filtered - r_t_;
    gamma_ += gamma_dot * dt;
}

// 挂车动力学模型
MX ESOTracker2::vehicleDynamicsModel(const MX& state, const MX& cmd_delta,
                                    const MX& vx, const MX& h_dist,
                                    const MX& dyn_params) {
    MX theta = state(2), vy = state(3), r = state(4), delta = state(5);
    MX r_t = state(6), gamma = state(7);

    // 10维动力学参数
    MX m1 = dyn_params(0), Iz1 = dyn_params(1), lf = dyn_params(2), lr = dyn_params(3);
    MX Cf = dyn_params(4), Cr = dyn_params(5), M = dyn_params(6), Iz2 = dyn_params(7);
    MX lt = dyn_params(8), Ct = dyn_params(9);
    MX m2 = M - m1;

    MX vx_safe = fmax(vx, 3.0); 

    // 牵引车侧向力
    MX alpha_f = delta - atan2((vy + lf * r), vx_safe);
    MX alpha_r = -atan2((vy - lr * r), vx_safe);
    MX Fyf = Cf * alpha_f;
    MX Fyr = Cr * alpha_r;

    // 牵引车2DOF横摆加速度
    MX r_dot_2DOF = (lf*Fyf*cos(delta) - lr*Fyr) / Iz1;

    // 铰接点速度
    MX vx_h1 = vx_safe;
    MX vy_h1 = vy - lr * r;
    MX vx_h2 = vx_h1 * cos(gamma) + vy_h1 * sin(gamma);
    MX vy_h2 = -vx_h1 * sin(gamma) + vy_h1 * cos(gamma);

    // 挂车质心速度与侧向力
    MX vx2 = vx_h2;
    MX vy2 = vy_h2 - lt * r_t;
    MX alpha_t = -atan2(vy2, vx2);
    MX Fyt = Ct * alpha_t;

    // 铰接力分解
    MX Fyt_hy1 = Fyt * cos(gamma);
    MX Fyt_hx1 = -Fyt * sin(gamma);
    MX hx = -lr, hy = 0;

    // 牵引车动力学
    MX d_trailor = (Fyt_hy1 * lr) / Iz1;
    MX d_vy = (Fyf*cos(delta) + Fyr + Fyt_hy1) / m1 - vx * r;
    MX Mh = hx * Fyt_hy1 - hy * Fyt_hx1;
    MX d_r = (lf*Fyf*cos(delta) - lr*Fyr + Mh) / Iz1 ;//+ h_dist - r_dot_2DOF- d_trailor;

    // 挂车动力学
    MX Fyt_hy2 = -Fyt;
    MX Mr = -lt * Fyt_hy2;
    MX d_r_t = Mr / Iz2;

    // 运动学状态
    MX d_gamma = r - r_t;
    MX d_x = vx * cos(theta) - vy * sin(theta);
    MX d_y = vx * sin(theta) + vy * cos(theta);
    MX d_theta = r;
    MX d_delta = (cmd_delta - delta) / nmpc_params_.T_lag;
    std::vector<MX> state_derivatives = {d_x, d_y, d_theta, d_vy, d_r, d_delta, d_r_t, d_gamma};

    return vertcat(state_derivatives);
}

// 构建NMPC求解器（替换为挂车约束，保留函数名）
void ESOTracker2::buildNMPSolver() {
    solver_.opti = Opti();
    int nx = nmpc_params_.nx, nu = nmpc_params_.nu, N = nmpc_params_.N, Nc = nmpc_params_.Nc;

    solver_.X = solver_.opti.variable(nx, N+1);
    solver_.U_sparse = solver_.opti.variable(nu, Nc);
    solver_.P_x0 = solver_.opti.parameter(nx);
    solver_.P_waypoints = solver_.opti.parameter(4, N+1);  
    solver_.P_vx = solver_.opti.parameter(1);
    solver_.P_u_prev = solver_.opti.parameter(1);
    solver_.P_h_hat = solver_.opti.parameter(1);
    solver_.P_dyn_params = solver_.opti.parameter(10); // 扩展为10维

    // 控制量稀疏化
    MX U_full = MX::zeros(nu, N);
    int base_steps = N / Nc, remainder = N % Nc, current_idx = 0;
    for (int i=0; i<Nc; i++) {
        int steps = base_steps + (i == Nc-1 ? remainder : 0);
        int end_idx = min(current_idx + steps, N);
        U_full(Slice(), Slice(current_idx, end_idx)) = repmat(solver_.U_sparse(Slice(), i), 1, end_idx - current_idx);
        current_idx = end_idx;
    }

    //转角变化率约束（新增）
    double max_dU = nmpc_params_.delta_rate_max * nmpc_params_.dt;
    double min_dU = nmpc_params_.delta_rate_min * nmpc_params_.dt;
    solver_.opti.subject_to(solver_.opti.bounded(min_dU, solver_.U_sparse(0) - solver_.P_u_prev, max_dU));
    for (int i=1; i<Nc; i++) {
        solver_.opti.subject_to(solver_.opti.bounded(min_dU, solver_.U_sparse(i) - solver_.U_sparse(i-1), max_dU));
    }

    MX J = 0.0;
    solver_.opti.subject_to(solver_.X(Slice(), 0) == solver_.P_x0);

    for (int k=0; k<N; k++) {
        MX st = solver_.X(Slice(), k), con = U_full(Slice(), k);
        MX h = solver_.P_h_hat * pow(0.85, k);

        // RK4积分（保留原逻辑）
        MX k1 = vehicleDynamicsModel(st, con, solver_.P_vx, h, solver_.P_dyn_params);
        MX k2 = vehicleDynamicsModel(st + nmpc_params_.dt/2 * k1, con, solver_.P_vx, h, solver_.P_dyn_params);
        MX k3 = vehicleDynamicsModel(st + nmpc_params_.dt/2 * k2, con, solver_.P_vx, h, solver_.P_dyn_params);
        MX k4 = vehicleDynamicsModel(st + nmpc_params_.dt * k3, con, solver_.P_vx, h, solver_.P_dyn_params);
        solver_.opti.subject_to(solver_.X(Slice(), k+1) == st + nmpc_params_.dt/6 * (k1 + 2*k2 + 2*k3 + k4));

        // // 挂车专属约束（新增）
        // MX r = st(4), r_t = st(6), gamma = st(7);
        // MX delta_next = solver_.X(5, k+1);
        // MX delta_rate = (delta_next - st(5)) / nmpc_params_.dt;
        // solver_.opti.subject_to(solver_.opti.bounded(nmpc_params_.delta_rate_min, delta_rate, nmpc_params_.delta_rate_max));

        // MX gamma_con = gamma + (r - r_t) * 0.05;
        // solver_.opti.subject_to(solver_.opti.bounded(nmpc_params_.gamma_min, gamma_con, nmpc_params_.gamma_max));

        // MX gamma_dot = r - r_t;
        // solver_.opti.subject_to(solver_.opti.bounded(-0.5, gamma_dot, 0.5));
        // solver_.opti.subject_to(solver_.opti.bounded(-0.5, r_t, 0.5));

        // 代价函数（扩展挂车项）
        MX ref_x = solver_.P_waypoints(0, k+1), ref_y = solver_.P_waypoints(1, k+1);
        MX ref_theta = solver_.P_waypoints(2, k+1), ref_kappa = solver_.P_waypoints(3, k+1);
        MX e_theta = atan2(sin(solver_.X(2, k+1) - ref_theta), cos(solver_.X(2, k+1) - ref_theta));
        MX r_ref = solver_.P_vx * ref_kappa;
        MX gamma_dot_actual = solver_.X(4, k+1) - solver_.X(6, k+1);

        J += nmpc_params_.Q(0,0) * (pow(solver_.X(0, k+1) - ref_x, 2) + pow(solver_.X(1, k+1) - ref_y, 2));
        J += nmpc_params_.Q(2,2) * pow(e_theta, 2);
        J += nmpc_params_.Q(4,4) * pow(solver_.X(4, k+1) - r_ref, 2);
        J += nmpc_params_.Q(3,3) * pow(solver_.X(3, k+1), 2);
        J += nmpc_params_.Q_gamma * pow(solver_.X(7, k+1), 2);
        J += nmpc_params_.Q_r_t * pow(solver_.X(6, k+1) - r_ref, 2);
        J += nmpc_params_.Q_gamma_rate * pow(gamma_dot_actual, 2);
        J += nmpc_params_.R * pow(con, 2);
    }
    // 控制量平滑项（保留原逻辑）
    J += nmpc_params_.dR * pow(solver_.U_sparse(0) - solver_.P_u_prev, 2);
    for (int i=1; i<Nc; i++) J += nmpc_params_.dR * pow(solver_.U_sparse(i) - solver_.U_sparse(i-1), 2);

    // 硬约束（保留原逻辑）
    solver_.opti.subject_to(solver_.opti.bounded(nmpc_params_.delta_min, solver_.U_sparse, nmpc_params_.delta_max));
    // solver_.opti.subject_to(solver_.opti.bounded(nmpc_params_.delta_min, solver_.X(5, Slice()), nmpc_params_.delta_max));

    solver_.opti.minimize(J);

    // IPOPT参数（Matlab实时性优化）
    Dict opts = {
        {"ipopt.print_level", 0}, 
        {"ipopt.sb", "yes"}, 
        {"ipopt.max_iter", 100},
        {"ipopt.tol", 1e-3},
        {"ipopt.acceptable_tol", 1e-2},
        {"ipopt.mu_strategy", "adaptive"},
        {"print_time", 0},
        {"expand", true},

        {"ipopt.warm_start_init_point", "yes"},        // 明确告诉求解器接受热启动
        {"ipopt.warm_start_bound_push", 1e-9},         // 将变量向边界推的松弛量压到极小
        {"ipopt.warm_start_slack_bound_push", 1e-9},
        {"ipopt.warm_start_mult_bound_push", 1e-9},    // 乘子松弛量压到极小
        
        {"print_time", 0}
    };
    solver_.opti.solver("ipopt", opts);
}

// 求解NMPC（修正 isEmpty 用法）
bool ESOTracker2::solveNMPC(const std::vector<double>& current_state, const casadi::DM& waypoints,
                          std::vector<double>& control_output) {

    auto start_time = std::chrono::high_resolution_clock::now();//记录开始的时间

    try {
        solver_.opti.set_value(solver_.P_x0, current_state);
        solver_.opti.set_value(solver_.P_waypoints, waypoints);
        solver_.opti.set_value(solver_.P_u_prev, current_cmd_);

        // --- 修正：使用 is_empty() 判断 DM 是否为空 ---
        // if (!solver_.last_sol_X.is_empty()) {
        //     DM X_guess = DM::zeros(nmpc_params_.nx, nmpc_params_.N+1);
        //     X_guess(Slice(), Slice(0, nmpc_params_.N)) = solver_.last_sol_X(Slice(), Slice(1, nmpc_params_.N+1));
        //     X_guess(Slice(), nmpc_params_.N) = solver_.last_sol_X(Slice(), nmpc_params_.N);
        //     solver_.opti.set_initial(solver_.X, X_guess);

        //     DM U_guess = DM::zeros(nmpc_params_.nu, nmpc_params_.Nc);
        //     U_guess(Slice(), Slice(0, nmpc_params_.Nc-1)) = solver_.last_sol_U(Slice(), Slice(1, nmpc_params_.Nc));
        //     U_guess(Slice(), nmpc_params_.Nc-1) = solver_.last_sol_U(Slice(), nmpc_params_.Nc-1);
        //     solver_.opti.set_initial(solver_.U_sparse, U_guess);
        // }

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

        // // 保存热启动数据
        // solver_.last_sol_X = sol.value(solver_.X);
        // solver_.last_sol_U = sol.value(solver_.U_sparse);
        // predicted_trajectory_(0, Slice()) = sol.value(solver_.X(0, Slice()));
        // predicted_trajectory_(1, Slice()) = sol.value(solver_.X(1, Slice()));

        control_output[0] = static_cast<double>(sol.value(solver_.U_sparse(0)));
        return true;
    } catch (std::exception& e) {

         auto end_time = std::chrono::high_resolution_clock::now();//求解失败的耗时
       std::chrono::duration<double, std::milli> elapsed = end_time - start_time;
       ROS_WARN("[%s] NMPC 求解失败! 耗时: %.2f ms, 原因: %s", getName().c_str(), elapsed.count(), e.what());
        // // --- 修正：清空 DM 赋值为空的 DM() ---
        // solver_.last_sol_X = DM();
        // solver_.last_sol_U = DM();
        solver_.has_prev_sol = false;
        solver_.sol_prev = nullptr;
        return false;
    }
}

} // namespace race_tracker

// 保留原插件注册（完全不变）
PLUGINLIB_EXPORT_CLASS(race_tracker::ESOTracker2, race_tracker::ControllerPluginBase)

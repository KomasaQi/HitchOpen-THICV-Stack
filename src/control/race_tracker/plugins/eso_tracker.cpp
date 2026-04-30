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

// -----------------------------------------------------------------------------
// NMPCParams 构造函数实现 
// -----------------------------------------------------------------------------
NMPCParams::NMPCParams() {
    updateQMatrix();
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
    is_high_speed_last_ = false;
    blend_alpha_ = 0.0;
    nmpc_safe_cmd_ = 0.0;
    start_time_ = ros::Time(0);
    last_final_cmd_ = 0.0;
    current_cmd_ = 0.0;

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

    // RLS
    rls_P_f_ = 1e5;
    rls_theta_f_ = 250000.0;
    rls_P_r_ = 1e5;
    rls_theta_r_ = 1000000.0;
    rls_r_prev_ = 0.0;
    rls_r_dot_pre_ = 0.0;
    rls_Cf_est_ = 250000.0;
    rls_Cr_est_ = 1000000.0;

    min_lookahead_distance_ = 6.0;  // 默认最小预瞄距 6m
    lookahead_speed_coeff_ = 0.7;   // 默认速度系数 0.7
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
    nh_nmpc.param("sparse_control_step", nmpc_params_.Nc, 5);
    nh_nmpc.param("sampling_time", nmpc_params_.dt, 0.05);
    nh_nmpc.param("integration_grade", nmpc_params_.integration_grade, 2.0);

    // -------------------------------------------------------------------------
    // 2. 加载车辆物理参数
    // -------------------------------------------------------------------------
    nh_nmpc.param("m", nmpc_params_.m, 10000.0);
    nh_nmpc.param("Iz", nmpc_params_.Iz, 50000.0);
    nh_nmpc.param("lf", nmpc_params_.lf, 2.0);
    nh_nmpc.param("lr", nmpc_params_.lr, 2.135);
    nh_nmpc.param("T_lag", nmpc_params_.T_lag, 0.2);

    // -------------------------------------------------------------------------
    // 3. 加载轮胎参数
    // -------------------------------------------------------------------------
    nh_nmpc.param("Cf", nmpc_params_.Cf, 250000.0);
    nh_nmpc.param("Cr", nmpc_params_.Cr, 1000000.0);
    nh_nmpc.param("Cf_min", nmpc_params_.Cf_min, 200000.0);
    nh_nmpc.param("Cf_max", nmpc_params_.Cf_max, 100000000.0);
    nh_nmpc.param("Cr_min", nmpc_params_.Cr_min, 1000000.0);
    nh_nmpc.param("Cr_max", nmpc_params_.Cr_max, 500000000.0);

    // -------------------------------------------------------------------------
    // 4. 加载控制量约束
    // -------------------------------------------------------------------------
    nh_nmpc.param("min_steer", nmpc_params_.delta_min, -0.5);
    nh_nmpc.param("max_steer", nmpc_params_.delta_max, 0.5);
    nh_nmpc.param("max_delta_delta", nmpc_params_.delta_c_max, 1.5);

    // -------------------------------------------------------------------------
    // 5. 加载代价函数权重
    // -------------------------------------------------------------------------
    nh_nmpc.param("Q_x", nmpc_params_.Q_x, 1000.0);
    nh_nmpc.param("Q_y", nmpc_params_.Q_y, 5000.0);
    nh_nmpc.param("Q_theta", nmpc_params_.Q_theta, 4000.0);
    nh_nmpc.param("Q_vy", nmpc_params_.Q_vy, 100.0);
    nh_nmpc.param("Q_r", nmpc_params_.Q_r, 800.0);
    nh_nmpc.param("Q_delta", nmpc_params_.Q_delta, 1000.0);
    nh_nmpc.param("R", nmpc_params_.R, 10.0);
    nh_nmpc.param("dR", nmpc_params_.dR, 500.0); // 

    // -------------------------------------------------------------------------
    // 6. 加载 Supervisor 配置 (模式切换与纯跟踪)
    // -------------------------------------------------------------------------
    ros::NodeHandle nh_super(nh, "supervisor_config");
    nh_super.param("startup_time", supervisor_params_.startup_time, 5.0);
    nh_super.param("blend_speed_low", supervisor_params_.blend_speed_low, 4.1667);
    nh_super.param("blend_speed_high", supervisor_params_.blend_speed_high, 5.0);
    nh_super.param("min_lookahead_distance", min_lookahead_distance_, 6.0);
    nh_super.param("lookahead_speed_coeff", lookahead_speed_coeff_, 0.7);


    // -------------------------------------------------------------------------
    // 后处理与打印
    // -------------------------------------------------------------------------
    // 更新 Eigen Q 矩阵
    nmpc_params_.updateQMatrix();

    // 重置 RLS 辨识器
    rls_Cf_est_ = nmpc_params_.Cf;
    rls_Cr_est_ = nmpc_params_.Cr;
    rls_theta_f_ = nmpc_params_.Cf;
    rls_theta_r_ = nmpc_params_.Cr;
    rls_r_prev_ = 0.0;
    rls_r_dot_pre_ = 0.0;

    // 简单的参数确认打印 (替代不存在的 logParamLoad)
    ROS_INFO("[%s] 参数加载完毕: m=%.0f, N=%d, dR=%.1f, Lookahead=%.1f", 
             getName().c_str(), nmpc_params_.m, nmpc_params_.N, nmpc_params_.dR, supervisor_params_.lookahead_distance);

    start_time_ = ros::Time::now(); 

    // 构建 CasADi 求解器
    buildNMPSolver();

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

    // 1. 提取基础信息
    if (!vehicle_status || !path || !control_msg) {
        ROS_ERROR("[%s] 收到空指针消息", getName().c_str());
        return;
    }
    if (path->points.empty()) return;

    double curr_vx_raw = vehicle_status->vel.linear.x;
    double curr_vx = std::max(vehicle_status->vel.linear.x, 1.0); // 防零除
    double curr_x = vehicle_status->pose.position.x;
    double curr_y = vehicle_status->pose.position.y;
    double curr_theta = vehicle_status->euler.yaw;
    double curr_ay = vehicle_status->acc.linear.y;
    double curr_r = vehicle_status->vel.angular.z;
    double curr_delta = vehicle_status->lateral.steering_angle;

    curr_delta = std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, curr_delta));
    
    // ==========================================================
    // 无论低速还是高速，UKF/RLS/ESO 都更新
    // ==========================================================
    
    static ros::Time last_control_time = ros::Time(0);
    ros::Time current_time = ros::Time::now();

    if (last_control_time.toSec() != 0.0 && (current_time - last_control_time).toSec() > 0.2) {
        ROS_WARN("[%s] 检测到控制重连，清空观测器记忆！", getName().c_str());
        solver_.has_prev_sol = false;
        solver_.sol_prev = nullptr;
        current_cmd_ = curr_delta; 
        ukf_P_est_ = (Matrix2d() << 1.0, 0.0, 0.0, 0.1).finished(); 
        eso_x1_ = curr_r; 
        eso_x2_ = 0.0;
        rls_r_prev_ = 0.0;
        blend_alpha_ = 0.0; 
        is_high_speed_last_ = false;
        start_time_ = current_time;
        last_final_cmd_ = curr_delta; 
    }
    last_control_time = current_time;

    const double obs_dt = std::max(dt, 0.01); 

    if (rls_r_prev_ == 0.0 && curr_r != 0.0) {
        rls_r_prev_ = curr_r;
    }

    // 1. UKF (始终更新)
    double obs_vx = std::max(std::abs(curr_vx_raw), 1.0);
    ukfEstimateVy(obs_vx, curr_delta, curr_ay, curr_r, obs_dt);
    double vy_est = ukf_x_est_(0);

    // 2. RLS (始终更新)
    rlsIdentifyStiffness(obs_vx, vy_est, curr_delta, curr_r, curr_ay, obs_dt);

    // 3. ESO (始终更新)
    esoCompute(curr_r, curr_delta, obs_dt);

    // ==========================================================
    // 启动阶段：使用 supervisor_params_
    // ==========================================================

    double time_elapsed = (current_time - start_time_).toSec();
    bool is_current_high_speed = false;

    // ==========================================================
    // 模式平滑过渡权重计算 
    // ==========================================================
    if (time_elapsed < supervisor_params_.startup_time) {
        // 启动前N秒：强制纯跟踪
        blend_alpha_ = 0.0;
        ROS_INFO_THROTTLE(1.0, "[STARTUP] 预热中: %.1f / %.1f s | 纯跟踪锁定", time_elapsed, supervisor_params_.startup_time);
    } else {
        // 后：恢复原有车速切换逻辑
        if (curr_vx_raw <= supervisor_params_.blend_speed_low) {
            blend_alpha_ = 0.0;
        } else if (curr_vx_raw >= supervisor_params_.blend_speed_high) {
            blend_alpha_ = 1.0;
        } else {
            blend_alpha_ = (curr_vx_raw - supervisor_params_.blend_speed_low) / (supervisor_params_.blend_speed_high - supervisor_params_.blend_speed_low);
        }
    }
    is_current_high_speed = (blend_alpha_ >= 0.99);

    // ==========================================================
    // NMPC都后台预计算，保持热启动
    // ==========================================================
    // 扰动纯化（始终计算)
    double vx_safe_external = std::max(curr_vx, 1.0); 
    double alpha_f_curr = curr_delta - atan2((vy_est + nmpc_params_.lf * curr_r), vx_safe_external);
    double alpha_r_curr = -atan2((vy_est - nmpc_params_.lr * curr_r), vx_safe_external);
    double Fyf_curr = rls_Cf_est_ * alpha_f_curr;
    double Fyr_curr = rls_Cr_est_ * alpha_r_curr;
    double r_dot_nominal = (nmpc_params_.lf * Fyf_curr * cos(curr_delta) - nmpc_params_.lr * Fyr_curr) / nmpc_params_.Iz;
    double b_eso = (rls_Cf_est_ * nmpc_params_.lf) / nmpc_params_.Iz;
    double r_dot_actual = b_eso * curr_delta + eso_x2_;
    double d_pure_trailer = r_dot_actual - r_dot_nominal;

    // 路径处理（始终计算）
    std::vector<double> current_pose = {curr_x, curr_y, curr_theta, curr_vx};
    casadi::DM waypoints_dm = process_race_path(*path, current_pose);
    std::vector<double> nmpc_state = {curr_x, curr_y, curr_theta, vy_est, curr_r, curr_delta};
    std::vector<double> control_output(1);

    // NMPC参数绑定（始终更新）
    std::vector<double> dyn_params = {nmpc_params_.m, nmpc_params_.Iz, nmpc_params_.lf, 
                                      nmpc_params_.lr, rls_Cf_est_, rls_Cr_est_};
    solver_.opti.set_value(solver_.P_vx, curr_vx);
    solver_.opti.set_value(solver_.P_h_hat, d_pure_trailer);
    solver_.opti.set_value(solver_.P_dyn_params, dyn_params);

    // 无论什么模式，都调用NMPC求解，保持热启动状态
    bool nmpc_solve_success = solveNMPC(nmpc_state, waypoints_dm, control_output);
    if (nmpc_solve_success) {
        nmpc_safe_cmd_ = control_output[0];
    } else {
        // 求解失败时，用上一帧的有效输出兜底
        nmpc_safe_cmd_ = current_cmd_;
    }
    // 限幅保护
    nmpc_safe_cmd_ = std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, nmpc_safe_cmd_));

    // ==========================================================
    // 纯跟踪逻辑 
    // ==========================================================
    double pp_safe_cmd_ = 0.0;
    double L = nmpc_params_.lf + nmpc_params_.lr; //轴距
    double lookahead_dist = min_lookahead_distance_ + lookahead_speed_coeff_ * curr_vx;
    
    // 找目标点
    int nearest_idx = find_nearest_path_point(curr_x, curr_y, *path);
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
    double delta_pp = atan2(2.0 * L * local_y, ld * ld);

    delta_pp = std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, delta_pp));
    pp_safe_cmd_ = delta_pp;
    
    // 打印调试信息
    if (blend_alpha_ < 0.01) {
        ROS_INFO_THROTTLE(0.5, "[PP] 纯跟踪模式 | Local: (%.2f, %.2f) | Lookahead Dist: %.2f m | Delta: %.3f rad", local_x, local_y, lookahead_dist, pp_safe_cmd_);
    } else if (blend_alpha_ > 0.99) {
        ROS_INFO_THROTTLE(2.0, "[%s] 高速模式 (%.1f km/h)", getName().c_str(), curr_vx_raw * 3.6);
    } else {
        ROS_INFO_THROTTLE(0.5, "[BLEND] 过渡模式 | 车速: %.1f km/h | 权重: %.2f | PP: %.3f | NMPC: %.3f", 
                            curr_vx_raw * 3.6, blend_alpha_, pp_safe_cmd_, nmpc_safe_cmd_);
    }

    // ==========================================================
    // 加权融合输出，最终平滑控制
    // ==========================================================
    // 加权融合两个算法的输出
    double final_cmd = blend_alpha_ * nmpc_safe_cmd_ + (1.0 - blend_alpha_) * pp_safe_cmd_;

    // 全局转角增量限制，防止任何情况下的跳变
    double max_delta_per_step = nmpc_params_.delta_c_max * obs_dt;
    final_cmd = std::max(last_final_cmd_ - max_delta_per_step, 
                     std::min(last_final_cmd_ + max_delta_per_step, final_cmd));
    last_final_cmd_ = final_cmd;

    // 更新current_cmd_，保持状态连续
    current_cmd_ = final_cmd;

    // 填装消息输出
    control_msg->lateral.steering_angle = final_cmd;
    control_msg->steering_mode = race_msgs::Control::FRONT_STEERING_MODE;
    control_msg->control_mode = race_msgs::Control::DES_ACCEL_ONLY;

    // 更新上一帧模式状态
    is_high_speed_last_ = is_current_high_speed;
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
        
        double yaw = quaternion_to_yaw(pt.pose.orientation);
        if (i == start_idx) yaw0 = yaw; 
        double diff = normalizeAngle(yaw - (theta_orig.empty() ? yaw0 : theta_orig.back()));
        theta_orig.push_back((theta_orig.empty() ? yaw0 : theta_orig.back()) + diff);
    }

    kappa_orig.resize(theta_orig.size(), 0.0);
    for (size_t i = 1; i < theta_orig.size() - 1; ++i) {
        double ds = s_orig[i+1] - s_orig[i-1];
        kappa_orig[i] = (ds > 1e-4) ? (theta_orig[i+1] - theta_orig[i-1]) / ds : 0.0;
    }

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

// ---------------------- 核心算法  ----------------------

void ESOTracker::ukfEstimateVy(double curr_vx, double curr_delta, double curr_ay, double curr_r, double dt) {
    
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

    MX vx_safe = fmax(vx, 2.0); 

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

        
        if (nmpc_params_.integration_grade >= 0.5 && nmpc_params_.integration_grade < 1.5) { // Euler前向积分
            solver_.opti.subject_to(solver_.X(Slice(), k+1) == st + nmpc_params_.dt * vehicleDynamicsModel(st, con, solver_.P_vx, h, solver_.P_dyn_params));
        }
        else if (nmpc_params_.integration_grade >= 1.5 && nmpc_params_.integration_grade < 2.5) { // RK2积分
            MX k1 = vehicleDynamicsModel(st, con, solver_.P_vx, h, solver_.P_dyn_params);
            MX k2 = vehicleDynamicsModel(st + nmpc_params_.dt / 2.0 * k1, con, solver_.P_vx, h, solver_.P_dyn_params);
            solver_.opti.subject_to(solver_.X(Slice(), k+1) == st + nmpc_params_.dt * k2);
        }
        else if (nmpc_params_.integration_grade >= 3.5 && nmpc_params_.integration_grade < 4.5) { // RK4积分
            MX k1 = vehicleDynamicsModel(st, con, solver_.P_vx, h, solver_.P_dyn_params);
            MX k2 = vehicleDynamicsModel(st + nmpc_params_.dt/2 * k1, con, solver_.P_vx, h, solver_.P_dyn_params);
            MX k3 = vehicleDynamicsModel(st + nmpc_params_.dt/2 * k2, con, solver_.P_vx, h, solver_.P_dyn_params);
            MX k4 = vehicleDynamicsModel(st + nmpc_params_.dt * k3, con, solver_.P_vx, h, solver_.P_dyn_params);
            solver_.opti.subject_to(solver_.X(Slice(), k+1) == st + nmpc_params_.dt/6 * (k1 + 2*k2 + 2*k3 + k4));
        }
        else {
            throw std::runtime_error("integration_grade不在有效范围内，无法选择积分方法");
        }

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
    solver_.opti.subject_to(solver_.U_sparse(0) - solver_.P_u_prev <= nmpc_params_.delta_c_max);
    solver_.opti.subject_to(solver_.U_sparse(0) - solver_.P_u_prev >= -nmpc_params_.delta_c_max);

    for (int i = 1; i < nmpc_params_.Nc; ++i) {
        solver_.opti.subject_to(solver_.U_sparse(i) - solver_.U_sparse(i-1) <= nmpc_params_.delta_c_max);
        solver_.opti.subject_to(solver_.U_sparse(i) - solver_.U_sparse(i-1) >= -nmpc_params_.delta_c_max);
    }

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
        
        {"print_time", 0}
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
        ROS_INFO("[%s] NMPC 求解成功! 耗时: %.2f ms", getName().c_str(), elapsed.count());

        solver_.sol_prev = std::make_unique<casadi::OptiSol>(sol);
        solver_.has_prev_sol = true;

        control_output[0] = static_cast<double>(sol.value(solver_.U_sparse(0)));
        return true;
    } catch (std::exception& e) {

       auto end_time = std::chrono::high_resolution_clock::now();
       std::chrono::duration<double, std::milli> elapsed = end_time - start_time;
       ROS_WARN("[%s] NMPC 求解失败! 耗时: %.2f ms, 原因: %s", getName().c_str(), elapsed.count(), e.what());
        
        solver_.has_prev_sol = false;
        solver_.sol_prev = nullptr;
        return false;
    }
}

} // namespace race_tracker

PLUGINLIB_EXPORT_CLASS(race_tracker::ESOTracker, race_tracker::ControllerPluginBase)
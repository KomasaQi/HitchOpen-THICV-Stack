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
    // EKF对照用
    // EKF 初始化（对照用）
    ekf_x_hat_ = Eigen::Vector4d::Zero();
    ekf_P_ = Eigen::Matrix4d::Identity();
    vy_ekf_est_ = 0.0;


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
    nh_nmpc.param("dR_dense", nmpc_params_.dR_dense, 0.0);     // 稠密增量惩罚，默认0=不改变原行为
    nh_nmpc.param("R_ddelta", nmpc_params_.R_ddelta, 0.0);     // 二阶差分惩罚，默认0=不改变原行为


    // -------------------------------------------------------------------------
    // 6. 加载 Supervisor 配置 (模式切换与纯跟踪)
    // -------------------------------------------------------------------------
    ros::NodeHandle nh_super(nh, "supervisor_config");
    nh_super.param("startup_time", supervisor_params_.startup_time, 5.0);
    nh_super.param("blend_speed_low", supervisor_params_.blend_speed_low, 4.1667);
    nh_super.param("blend_speed_high", supervisor_params_.blend_speed_high, 5.0);
    nh_super.param("min_lookahead_distance", min_lookahead_distance_, 6.0);
    nh_super.param("lookahead_speed_coeff", lookahead_speed_coeff_, 0.7);
    nh_super.param("control_time", control_time_, 0.05);
    nh_super.param("control_delay_sec", control_delay_sec_, 0.2);
    nh_super.param("output_lpf_tau", output_lpf_tau_, 0.0);   // 输出低通时间常数(s)，默认0=关闭



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
    // 初始化发布器
    est_pub_ = nh.advertise<race_msgs::ESOEstimation>("/race/eso_estimation_states", 1);    
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

    if (last_control_time.toSec() != 0.0 && (current_time - last_control_time).toSec() > 1) {     //5-28原本是0.02，现在改成1
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
        final_cmd_filt_ = curr_delta;
        final_cmd_filt_init_ = false;
    }
    last_control_time = current_time;

    const double obs_dt = std::max(0.01, std::min(dt, 0.05));//0.01只设置了下限，0.05设置了上限   5-28修改

    if (rls_r_prev_ == 0.0 && curr_r != 0.0) {
        rls_r_prev_ = curr_r;
    }

    // 1. UKF (始终更新)
    double obs_vx = std::max(std::abs(curr_vx_raw), 1.0);
    ukfEstimateVy(obs_vx, curr_delta, curr_ay, curr_r, obs_dt);
    double vy_est = ukf_x_est_(0);
    double vy_est1 = ukf_x_est_(0);
    // double vy_est = 0.0;

    // 2. RLS (始终更新)
    rlsIdentifyStiffness(obs_vx, vy_est, curr_delta, curr_r, curr_ay, obs_dt);

    // 3. ESO (始终更新)
    esoCompute(curr_r, curr_delta, obs_dt);

    // 参照参数引入对照输出
    calculate_trailer_kinematics(curr_vx, curr_r, obs_dt);
    const double curr_gamma = gamma_;
    const double curr_r_t = r_t_;
    ekfEstimateVy(curr_vx, curr_delta, curr_ay, curr_r, 350000.0, obs_dt);
    const double vy_est2 = ekf_x_hat_(0);

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
        // 恢复原有车速切换逻辑
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
    double r_dot_model = r_dot_nominal + d_pure_trailer;
    Model_r1_ = curr_r + r_dot_model * obs_dt;

    // 路径处理（始终计算）
    std::vector<double> current_pose = {curr_x, curr_y, curr_theta, curr_vx};
    casadi::DM waypoints_dm = process_race_path(*path, current_pose);
    double kappa = static_cast<double>(waypoints_dm(3,1));
    double theta = static_cast<double>(waypoints_dm(2,0));
    double r_ref = curr_vx * kappa;
    double vy_model = curr_vx * sin(theta) + vy_est * cos(theta);

    // NMPC 现在工作在“自车体坐标系”：原点为自车当前位置，x 轴沿自车当前航向。
    // 因此初始 x,y,theta 均为 0；vy/r/delta 仍为实际物理量。
    std::vector<double> nmpc_state = {0.0, 0.0, 0.0, vy_est, curr_r, curr_delta};
    std::vector<double> control_output(1);

    // NMPC参数绑定（始终更新）
    std::vector<double> dyn_params = {nmpc_params_.m, nmpc_params_.Iz, nmpc_params_.lf, 
                                      nmpc_params_.lr, rls_Cf_est_, rls_Cr_est_};
    solver_.opti.set_value(solver_.P_vx, curr_vx);
    solver_.opti.set_value(solver_.P_h_hat, d_pure_trailer);//d_pure_trailer
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
    double delta_pp_raw = atan2(2.0 * L * local_y, ld * ld);
    delta_pp_raw = std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, delta_pp_raw));

     // ---- 时延队列处理 ----
    pp_cmd_queue_.push_back(delta_pp_raw);
    // 计算队列的最大容量 (时延秒数 / 控制周期)
    size_t max_queue_size = static_cast<size_t>(std::max(1.0, control_delay_sec_ / control_time_));

    if (pp_cmd_queue_.size() > max_queue_size) {
        // 取出队列最前端（即历史时刻）的控制量作为当前输出
        pp_safe_cmd_ = pp_cmd_queue_.front();
        pp_cmd_queue_.pop_front();
    } else {
        // 初始阶段队列未填满时，可以直接输出当前值，或者输出 0
        pp_safe_cmd_ = delta_pp_raw;
    }

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

    // 输出端一阶低通滤波：滤掉驾驶员能感知的高频抖动，进一步提升方向盘转动质量。
    // tau<=0 时直接透传，不改变原行为。注意 LPF 引入的相位滞后由 tau 控制，
    // 取较小值(如 0.08~0.15s)可在几乎不损失跟踪的前提下显著“顺滑”手感。
    if (output_lpf_tau_ > 1e-6) {
        if (!final_cmd_filt_init_) {
            final_cmd_filt_ = final_cmd;
            final_cmd_filt_init_ = true;
        }
        double a = obs_dt / (output_lpf_tau_ + obs_dt);
        final_cmd_filt_ = (1.0 - a) * final_cmd_filt_ + a * final_cmd;
        final_cmd = final_cmd_filt_;
    }

    // 更新current_cmd_，保持状态连续
    current_cmd_ = final_cmd;

    // 填装消息输出
    control_msg->lateral.steering_angle = final_cmd;
    control_msg->steering_mode = race_msgs::Control::FRONT_STEERING_MODE;
    control_msg->control_mode = race_msgs::Control::DES_ACCEL_ONLY;

    // 更新上一帧模式状态
    is_high_speed_last_ = is_current_high_speed;

    race_msgs::ESOEstimation est_msg;
    est_msg.model_r1 = Model_r1_;              // 模型输出横摆角速度
    est_msg.vy_est1 = vy_est;                  // 侧向速度
    est_msg.eso1_total = eso_x2_;              // ESO_x2
    est_msg.eso1_pure = d_pure_trailer;        // 纯扰动
    est_msg.r_t = r_t_;                        // 挂车横摆率
    est_msg.gamma_angle = gamma_;              // 铰接角
    est_msg.vy_est2  = vy_est2;                // 方案二侧向速度估计
    est_msg.Cf_est = rls_Cf_est_;              // 前轴侧偏刚度
    est_msg.Cr_est = rls_Cr_est_;              // 后轴侧偏刚度
    est_msg.kappa = kappa;                     // 参考曲率
    est_msg.r_ref = r_ref;                     // 参考横摆率
    est_msg.theta = theta;                     // 参考航向角
    est_msg.vy_model = vy_model;               // 侧向速度模型
    est_pub_.publish(est_msg);
}

// ---------------------- 路径处理与辅助函数 ----------------------
double ESOTracker::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

namespace {
// 计算 a 相对 b 的最短角度差，结果恒落在 [-pi, pi]。
// 等价于 atan2(sin(a-b), cos(a-b))，消除 ±pi 缠绕造成的跳变。
inline double angleDiff(double a, double b) {
    double d = a - b;
    while (d > M_PI)  d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    return d;
}

// 体坐标系变换的原点（自车当前位置）。由 process_race_path 在每帧调用前设置，
// 供 interpolate_path_segment 把全局位置参考转换为体坐标系。控制器单实例串行调用，安全。
double g_ref_x0 = 0.0;
double g_ref_y0 = 0.0;
} // anonymous namespace

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
    // yaw0 = 自车当前航向 curr_theta（由 process_race_path 透传 current_state[2]）。
    const double veh_yaw = yaw0;

    std::vector<double> s_orig, x_orig, y_orig, theta_orig, kappa_orig;

    // 关键修改：把每个参考点的航向都用 angleDiff 表示为“相对自车当前航向”的量，
    // 再在 s 方向上连续解缠绕(unwrap)，得到一条连续、且锚定在 0 附近的参考航向曲线。
    // 这样无论自车朝向是 0、±pi/2 还是 ±pi，参考航向都不会跨越 ±pi 折叠边界，
    // 从源头消除 ±pi/2 附近的航向突变与稳态误差放大问题。
    double theta_prev = 0.0;  // 上一个参考点的(相对、已解缠绕)航向
    for (int i = start_idx; i <= end_idx; ++i) {
        const auto& pt = path.points[i];
        s_orig.push_back(cum_dist[i - start_idx]);
        x_orig.push_back(pt.pose.position.x);
        y_orig.push_back(pt.pose.position.y);

        double yaw_abs = quaternion_to_yaw(pt.pose.orientation);
        // 相对自车航向的航向角（首点会落在 [-pi, pi]）
        double yaw_rel = angleDiff(yaw_abs, veh_yaw);

        if (theta_orig.empty()) {
            theta_prev = yaw_rel;                    // 首点直接采用相对航向
        } else {
            // 沿弧长连续解缠绕：在上一点基础上加最短增量，保持曲线连续
            theta_prev += angleDiff(yaw_rel, theta_prev);
        }
        theta_orig.push_back(theta_prev);
    }

    kappa_orig.resize(theta_orig.size(), 0.0);
    for (size_t i = 1; i + 1 < theta_orig.size(); ++i) {
        double ds = s_orig[i+1] - s_orig[i-1];
        kappa_orig[i] = (ds > 1e-4) ? (theta_orig[i+1] - theta_orig[i-1]) / ds : 0.0;
    }

    auto x_interp = linear_interpolate(s_orig, x_orig, s_target);
    auto y_interp = linear_interpolate(s_orig, y_orig, s_target);
    auto theta_interp = linear_interpolate(s_orig, theta_orig, s_target);
    auto kappa_interp = linear_interpolate(s_orig, kappa_orig, s_target);

    // 位置参考也转换到“自车体坐标系”（原点为自车当前位置，x 轴沿自车当前航向），
    // 与相对航向、相对动力学初值 (0,0,0) 保持一致，彻底摆脱全局朝向的影响。
    const double cos_y = std::cos(veh_yaw);
    const double sin_y = std::sin(veh_yaw);
    int n_waypoints = s_target.size();
    casadi::DM waypoints = casadi::DM::zeros(4, n_waypoints);
    for (int i = 0; i < n_waypoints; ++i) {
        double dx = x_interp[i] - g_ref_x0;
        double dy = y_interp[i] - g_ref_y0;
        double bx =  cos_y * dx + sin_y * dy;   // 体坐标系纵向
        double by = -sin_y * dx + cos_y * dy;   // 体坐标系横向
        waypoints(0, i) = bx;
        waypoints(1, i) = by;
        waypoints(2, i) = theta_interp[i];   // 相对自车当前航向、连续解缠绕后的参考航向
        waypoints(3, i) = kappa_interp[i];
    }
    return waypoints;
}

casadi::DM ESOTracker::process_race_path(const race_msgs::Path& input_path, const std::vector<double>& current_state) {
    int nearest_idx = find_nearest_path_point(current_state[0], current_state[1], input_path);
    if (nearest_idx == -1) return casadi::DM::zeros(4, nmpc_params_.N + 1);

    // 设置体坐标系变换原点为自车当前位置，供 interpolate_path_segment 使用
    g_ref_x0 = current_state[0];
    g_ref_y0 = current_state[1];

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
    
    if (curr_vx < 3.0) {
        rls_P_f_ = 2e5; rls_P_r_ = 1e6; 
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
    
    if (excite_flag && (abs(alpha_f) > 0.003)) {
        double phi = alpha_f, e = Fy_f_obs - phi * rls_theta_f_;
        double K = (rls_P_f_ * phi) / (0.99 + phi * rls_P_f_ * phi);
        rls_theta_f_ += K * e;
        rls_P_f_ = (1.0 / 0.99) * (rls_P_f_ - K * phi * rls_P_f_) + 0.001;
    } else rls_P_f_ = std::min(rls_P_f_, 1e7);

    if (excite_flag && (abs(alpha_r) > 0.003)) {
        double phi = alpha_r, e = Fy_r_obs - phi * rls_theta_r_;
        double K = (rls_P_r_ * phi) / (0.99 + phi * rls_P_r_ * phi);
        rls_theta_r_ += K * e;
        rls_P_r_ = (1.0 / 0.99) * (rls_P_r_ - K * phi * rls_P_r_) + 0.001;
    } else rls_P_r_ = std::min(rls_P_r_, 1e7);

    rls_Cf_est_ = std::max(nmpc_params_.Cf_min, std::min(rls_theta_f_, nmpc_params_.Cf_max));
    rls_Cr_est_ = std::max(nmpc_params_.Cr_min, std::min(rls_theta_r_, nmpc_params_.Cr_max));
    }
void ESOTracker::esoCompute(double curr_r, double curr_delta, double dt){
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
        // 体坐标系下，X(2) 与 ref_theta 均为连续、锚定在 0 附近的相对航向，
        // 不会跨越 ±pi 折叠边界，因此直接作差即可，无需 atan2(sin,cos) 折叠。
        // 移除折叠后，代价函数对航向误差全程光滑可导，消除 ±pi/2 附近的梯度突变。
        MX e_theta = solver_.X(2, k+1) - ref_theta;

        J += nmpc_params_.Q(0,0) * pow(solver_.X(0, k+1) - ref_x, 2);
        J += nmpc_params_.Q(1,1) * pow(solver_.X(1, k+1) - ref_y, 2);
        J += nmpc_params_.Q(2,2) * pow(e_theta, 2);
        J += nmpc_params_.Q(4,4) * pow(solver_.X(4, k+1) - solver_.P_vx * ref_kappa, 2);
        J += nmpc_params_.Q(3,3) * pow(solver_.X(3, k+1), 2);
        J += nmpc_params_.R * pow(con, 2);
    }

    J += nmpc_params_.dR * pow(solver_.U_sparse(0) - solver_.P_u_prev, 2);
    for (int i=1; i<Nc; i++) J += nmpc_params_.dR * pow(solver_.U_sparse(i) - solver_.U_sparse(i-1), 2);

    // --- 平顺性增强项（默认权重为0时完全不改变原行为）---
    // (a) 稠密增量惩罚：对逐步展开后的整条转角序列 U_full 的相邻差分加惩罚，
    //     抑制稀疏控制点 dR 漏掉的“段内阶梯/段间台阶”抖动，让方向盘速度更连续。
    if (nmpc_params_.dR_dense > 0.0) {
        // 第0步相对上一帧实际输出，保证帧间连续
        J += nmpc_params_.dR_dense * pow(U_full(Slice(), 0) - solver_.P_u_prev, 2);
        for (int k=1; k<N; k++) {
            J += nmpc_params_.dR_dense * pow(U_full(Slice(), k) - U_full(Slice(), k-1), 2);
        }
    }

    // (b) 二阶差分(加速度)惩罚：专门压制来回摆动(limit cycle)。
    //     振荡的特征就是转角增量正负交替，即二阶差分大；惩罚它能显著“拉直”方向盘
    //     而对单调修正几乎无影响，因此能在不牺牲跟踪的前提下抑制蛇行。
    if (nmpc_params_.R_ddelta > 0.0) {
        // 以上一帧输出为锚点，构造 [P_u_prev, U_sparse(0..Nc-1)] 的二阶差分
        for (int i=1; i<Nc; i++) {
            MX u_im2 = (i >= 2) ? solver_.U_sparse(i-2) : solver_.P_u_prev;
            MX ddi = solver_.U_sparse(i) - 2.0 * solver_.U_sparse(i-1) + u_im2;
            J += nmpc_params_.R_ddelta * pow(ddi, 2);
        }
    }

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
      //  ROS_INFO("[%s] NMPC 求解成功! 耗时: %.2f ms", getName().c_str(), elapsed.count());
        ROS_INFO("[%s] NMPC 求解成功! \033[1;32m耗时: %.2f ms\033[0m, \033[38;5;208mCf_est: %.2f, Cr_est: %.2f\033[0m",
        getName().c_str(), elapsed.count(), rls_Cf_est_, rls_Cr_est_);

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

// 新增函数，状态估计与方案二做对照
void ESOTracker::ekfEstimateVy(double curr_vx,double curr_delta,double curr_ay,double curr_r,double M,double dt) {
    (void)M;  // 当前未使用，保留接口兼容

    // ==============================
    // 0) 基础保护
    // ==============================
    const double vx_safe = std::max(std::abs(curr_vx), 1.0);
    const double dt_safe = std::max(dt, 0.05);

    // ==============================
    // 1) 车辆/挂车经验参数（固定值）
    // ==============================
    // 主车参数（已有）
    const double a   = nmpc_params_.lf;
    const double b   = nmpc_params_.lr;
    const double m1  = nmpc_params_.m;
    const double Iz1 = nmpc_params_.Iz;

    // 缺失参数 -> 固定经验值（可后续再调）
    const double c   = b;         // 铰接点到主车质心近似，经验取 lr
    const double d   = 3.4;       // 挂车质心到铰接点距离[m]
    const double L2  = 7.9;       // 挂车等效轴距[m]
    const double m2  = 35000.0;   // 挂车总质量[kg]
    const double Iz2 = 150000.0+10.0*m2;  // 挂车偏航转动惯量[kg*m^2]

    // 轮胎侧偏刚度
    const double Cf = 250000.0;
    const double Cr = 1000000.0;
    const double Ct = 400000.0;   // 采用的固定值同方案二便于对照

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
    P_pred = 0.5 * (P_pred + P_pred.transpose());

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

void ESOTracker::calculate_trailer_kinematics(double curr_vx, double curr_r, double dt) {
    const double L2 = 7.9;
    const double Lh = 0.0;          // 建议用参数，不要写死0
    const double vx = std::max(curr_vx, 1.0);

    // 牵引车横摆率直接采用状态量，并做轻微低通抑制噪声
    const double tau_r = 0.08;                  // 可调: 0.05~0.15
    if (!r_filter_initialized_) {
        r_tractor_filt_ = curr_r;
        r_filter_initialized_ = true;
    }
    const double alpha_r = dt / (tau_r + dt);
    r_tractor_filt_ = (1.0 - alpha_r) * r_tractor_filt_ + alpha_r * curr_r;
    const double r_tractor = r_tractor_filt_;

    // 挂车横摆率运动学估计
    r_t_ = -(vx * std::sin(gamma_) + Lh * r_tractor * std::cos(gamma_)) / L2;

    // 铰接角动态
    const double gamma_dot = r_t_ - r_tractor;
    gamma_ += gamma_dot * dt;
}


} // namespace race_tracker

PLUGINLIB_EXPORT_CLASS(race_tracker::ESOTracker, race_tracker::ControllerPluginBase)
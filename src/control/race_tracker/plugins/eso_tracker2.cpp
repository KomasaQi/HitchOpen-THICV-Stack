#include "race_tracker/eso_tracker2.h"
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <numeric>
#include <limits>
#include <stdexcept>
#include <chrono>
#include <algorithm>  // for std::clamp
#include <array>
#include <cmath>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>


using namespace casadi;
using namespace Eigen;
using namespace std;

namespace race_tracker {

// 构造函数
ESOTracker2::ESOTracker2() {
    // CKF初始化：状态 x=[vy, r1, r2, gamma]^T。
    ckf_x_hat_ = Vector4d::Zero();
    ckf_P_ = Matrix4d::Identity() * 1.0;
    // RLS初始化（FF-RLS）
    rls_P_ = Matrix3d::Identity() * 1e4;
    // 原ESO变量初始化
    eso_x1_ = 0.0;
    eso_x2_ = 0.0;
    eso_initialized_ = false;

    // 挂车状态初始化
    gamma_ = 0.0;
    r_t_ = 0.0;
    rls_w2_prev_ = 0.0;
    rls_w2_dot_prev_ = 0.0;
    r_tractor_filt_ = 0.0;
    r_filter_initialized_ = false;
    // 原NMPC变量初始化
    current_cmd_ = 0.0;
    solver_.has_prev_sol = false;
    solver_.sol_prev = nullptr;

    // --- 模式切换与软过渡变量初始化（参考ESOTracker） ---
    is_high_speed_last_ = false;
    blend_alpha_ = 0.0;
    nmpc_safe_cmd_ = 0.0;
    last_final_cmd_ = 0.0;
    supervisor_params_.startup_time = 5.0;
    supervisor_params_.blend_speed_low = 4.1667;
    supervisor_params_.blend_speed_high = 5.0;
    supervisor_params_.standstill_speed = 0.05;
    supervisor_params_.nmpc_speed_floor = 1.0;

}

// 插件初始化
bool ESOTracker2::initialize(ros::NodeHandle& nh) {
    ros::NodeHandle nh_nmpc(nh, "eso_tracker2");
    ROS_INFO("[%s] NMPC 控制器命名空间: %s", getName().c_str(), nh_nmpc.getNamespace().c_str());

    // 加载NMPC核心参数
    nh_nmpc.param("nx", nmpc_params_.nx, 8);
    nh_nmpc.param("nu", nmpc_params_.nu, 1);
    nh_nmpc.param("prediction_step", nmpc_params_.N, 35);
    nh_nmpc.param("sparse_control_step", nmpc_params_.Nc, 5);
    nh_nmpc.param("sampling_time", nmpc_params_.dt, 0.05);
    // 加载—— 车辆核心动力学参数（含挂车） ——
    nh_nmpc.param("m", nmpc_params_.m, 10000.0);
    nh_nmpc.param("Iz", nmpc_params_.Iz, 50000.0);
    nh_nmpc.param("lf", nmpc_params_.lf, 2.0);
    nh_nmpc.param("lr", nmpc_params_.lr, 2.135);
    nh_nmpc.param("m_t_empty", nmpc_params_.m_t, 7570.0);
    nh_nmpc.param("Iz_t_empty", nmpc_params_.Iz_t, 150000.0);
    nh_nmpc.param("m_t_total", nmpc_params_.m_t_total, 39500.0);
    nh_nmpc.param("lt", nmpc_params_.lt, 3.4);
    nh_nmpc.param("L2", nmpc_params_.L2, 7.9);
    nh_nmpc.param("lh", nmpc_params_.lh, 0.0);
    nh_nmpc.param("T_lag", nmpc_params_.T_lag, 0.1);
    m_t_total_default_ = nmpc_params_.m_t_total;
    m_t_empty_default_ = nmpc_params_.m_t;

    // 加载—— 控制量边界约束 ——
    nh_nmpc.param("min_steer", nmpc_params_.delta_min, -0.6);
    nh_nmpc.param("max_steer", nmpc_params_.delta_max, 0.6);
    nh_nmpc.param("delta_rate_max", nmpc_params_.delta_rate_max, 1.5);
    nh_nmpc.param("delta_rate_min", nmpc_params_.delta_rate_min, -1.5);

    // 加载—— 代价函数权重 ——
    nh_nmpc.param("Q_x", nmpc_params_.Q(0,0), 10000.0);          // 横向位置误差权重
    nh_nmpc.param("Q_y", nmpc_params_.Q(1,1), 10000.0);          // 纵向位置误差权重
    nh_nmpc.param("Q_theta", nmpc_params_.Q(2,2), 5000.0);       // 航向角误差权重
    nh_nmpc.param("Q_vy", nmpc_params_.Q(3,3), 100.0);           // 侧向速度权重
    nh_nmpc.param("Q_r", nmpc_params_.Q(4,4), 800.0);            // 横摆角速度权重
    nh_nmpc.param("Q_delta", nmpc_params_.Q(5,5), 1000.0);       // 未使用该权重项

    nh_nmpc.param("Q_rt", nmpc_params_.Q(6,6), 1.0);             // 挂车状态权重
    nh_nmpc.param("Q_gamma", nmpc_params_.Q(7,7), 1.0);          // 挂车状态权重
    nh_nmpc.param("Q_dgamma", nmpc_params_.dgamma, 4500.0);      // 挂车状态权重

    nh_nmpc.param("Q_R", nmpc_params_.R, 10.0);                  // 控制量权重 
    nh_nmpc.param("Q_dR", nmpc_params_.dR, 100000.0);            // 控制增量权重 (平滑性)

    //  —— 参数估计中参数 ——
    nh_nmpc.param("Kiz", nmpc_params_.Kiz, 5.1);      // 横摆转动惯量比例系数

    // 加载—— 参数估计中参数 ——
    nh_nmpc.param("rls_Cf_est", rls_Cf_est_default_, 250000.0);          // 挂车前轴刚度初始值（N/m）
    nh_nmpc.param("rls_Cr_est", rls_Cr_est_default_, 1000000.0);         // 挂车后轴刚度初始值（N/m）
    nh_nmpc.param("rls_Ct_est", rls_Ct_est_default_, 400000.0);          // 挂车横摆转动惯量初始值（kg·m²）
    nh_nmpc.param("rls_w1_prev", rls_w1_prev_, 0.0);              // 挂车横摆角初始值（rad）
    nh_nmpc.param("rls_w1_dot_prev", rls_w1_dot_prev_, 0.0);        // 挂车横摆角速度初始值（rad/s）
    nh_nmpc.param("rls_Cf_est_max", rls_Cf_est_max_, 500000.0);          // 挂车前轴刚度最大值（N/m）
    nh_nmpc.param("rls_Cr_est_max", rls_Cr_est_max_, 2000000.0);          // 挂车后轴刚度最大值（N/m）
    nh_nmpc.param("rls_Ct_est_max", rls_Ct_est_max_, 200000.0);          // 挂车横摆转动惯量最大值（kg·m²）
    nh_nmpc.param("rls_Cf_est_min", rls_Cf_est_min_, 50000.0);          // 挂车前轴刚度最小值（N/m）
    nh_nmpc.param("rls_Cr_est_min", rls_Cr_est_min_, 100000.0);          // 挂车后轴刚度最小值（N/m）
    nh_nmpc.param("rls_Ct_est_min", rls_Ct_est_min_, 50000.0);          // 挂车横摆转动惯量最小值（kg·m²）

    //  —— IPOPT求解器参数 ——
    nh_nmpc.param("ipopt_max_iter", ipopt_max_iter_, 100.0);                      // IPPT最大迭代次数，默认100次
    nh_nmpc.param("ipopt_acceptable_tol", ipopt_acceptable_tol_, 1e-2);               // IPPT可接受解的容差，默认1e-2
    nh_nmpc.param("ipopt_acceptable_iter", ipopt_acceptable_iter_, 5.0);                 // IPPT可接受迭代次数，默认5次 连续 5 次迭代满足 acceptable_tol 即可停止
    nh_nmpc.param("ipopt_warm_start_bound_push", ipopt_warm_start_bound_push_, 1e-3);        // IPPT预热边界推送系数，默认1e-3，用于初始化控制量边界，热启动时，将初始点向变量边界 “推” 的最小距离
    nh_nmpc.param("ipopt_warm_start_slack_bound_push", ipopt_warm_start_slack_bound_push_, 1e-3);  // IPPT预热边界松弛系数，默认1e-3，用于初始化控制量边界,针对松弛变量
    nh_nmpc.param("ipopt_warm_start_mult_bound_push", ipopt_warm_start_mult_bound_push_, 1e-3);   // IPPT预热边界松弛系数，默认1e-3，用于初始化控制量边界,针对拉格朗日乘子

    // 加载—— 求解器设定 ——
    nh_nmpc.param("integration_grade", integration_grade_, 2.0);          // 求解器积分阶数，默认2阶

    rls_Cf_est_ = rls_Cf_est_default_; // 将默认参数赋予变量
    rls_Cr_est_ = rls_Cr_est_default_;
    rls_Ct_est_ = rls_Ct_est_default_;
    rls_C_out_prev_ = Vector3d(rls_Cf_est_, rls_Cr_est_, rls_Ct_est_);

    // 加载—— 动态预瞄参数 ——
    nh_nmpc.param("min_lookahead_distance", min_lookahead_distance_, 3.0);
    nh_nmpc.param("lookahead_speed_coeff", lookahead_speed_coeff_, 0.5);
    

    // 打印加载的参数
    // NMPC核心参数
    logParamLoad("nx",nmpc_params_.nx, 8);
    logParamLoad("nu",nmpc_params_.nu, 1);
    logParamLoad("prediction_step",nmpc_params_.N, 35);
    logParamLoad("sparse_control_step",nmpc_params_.Nc, 5);
    logParamLoad("sampling_time",nmpc_params_.dt, 0.05);
    // 车辆核心动力学参数（含挂车） ——
    logParamLoad("m", nmpc_params_.m, 10000.0);
    logParamLoad("Iz", nmpc_params_.Iz, 50000.0);
    logParamLoad("lf", nmpc_params_.lf, 2.0);
    logParamLoad("lr", nmpc_params_.lr, 2.135);
    logParamLoad("m_t", nmpc_params_.m_t, 7570.0);
    logParamLoad("Iz_t", nmpc_params_.Iz_t, 150000.0);
    logParamLoad("lt", nmpc_params_.lt, 3.4);
    logParamLoad("L2", nmpc_params_.L2, 7.9);
    logParamLoad("lh", nmpc_params_.lh, 0.0);
    logParamLoad("T_lag", nmpc_params_.T_lag, 0.1);
    // 控制量边界约束 ——
    logParamLoad("min_steer", nmpc_params_.delta_min, -0.6);
    logParamLoad("max_steer", nmpc_params_.delta_max, 0.6);
    logParamLoad("delta_rate_max", nmpc_params_.delta_rate_max, 1.5);
    logParamLoad("delta_rate_min", nmpc_params_.delta_rate_min, -1.5);
    // 代价函数权重 ——
    logParamLoad("Q_x",nmpc_params_.Q(0,0), 10001.0); 
    logParamLoad("Q_y",nmpc_params_.Q(1,1), 10000.0);          // 纵向位置误差权重
    logParamLoad("Q_theta",nmpc_params_.Q(2,2), 5000.0);       // 航向角误差权重
    logParamLoad("Q_vy",nmpc_params_.Q(3,3), 100.0);          // 侧向速度权重
    logParamLoad("Q_r",nmpc_params_.Q(4,4), 800.0);          // 横摆角速度权重
    logParamLoad("Q_delta",nmpc_params_.Q(5,5), 1000.0);      // 未使用该权重项
    logParamLoad("Q_rt",nmpc_params_.Q(6,6), 1.0);      // 挂车状态权重
    logParamLoad("Q_gamma",nmpc_params_.Q(7,7), 1.0);      // 挂车状态权重
    logParamLoad("Q_dgamma", nmpc_params_.dgamma, 4500.0);      // 挂车状态权重
    logParamLoad("Q_R", nmpc_params_.R, 10.0);      // 控制量权重 
    logParamLoad("Q_dR", nmpc_params_.dR, 100000.0);      // 控制增量权重 (平滑性)
    // —— 参数估计中参数 ——
    logParamLoad("Kiz", nmpc_params_.Kiz, 5.1);      // 横摆转动惯量比例系数
    logParamLoad("rls_Cf_est", rls_Cf_est_, 250000.0);          // 挂车前轴刚度初始值（N/m）
    logParamLoad("rls_Cr_est", rls_Cr_est_, 1000000.0);         // 挂车后轴刚度初始值（N/m）
    logParamLoad("rls_Ct_est", rls_Ct_est_, 400000.0);          // 挂车横摆转动惯量初始值（kg·m²）
    logParamLoad("rls_w1_prev", rls_w1_prev_, 0.0);              // 挂车横摆角初始值（rad）
    logParamLoad("rls_w1_dot_prev", rls_w1_dot_prev_, 0.0);        // 挂车横摆角速度初始值（rad/s）
    logParamLoad("rls_Cf_est_max", rls_Cf_est_max_, 500000.0);          // 挂车前轴刚度最大值（N/m）
    logParamLoad("rls_Cr_est_max", rls_Cr_est_max_, 2000000.0);          // 挂车后轴刚度最大值（N/m）
    logParamLoad("rls_Ct_est_max", rls_Ct_est_max_, 200000.0);          // 挂车横摆转动惯量最大值（kg·m²）
    logParamLoad("rls_Cf_est_min", rls_Cf_est_min_, 50000.0);          // 挂车前轴刚度最小值（N/m）
    logParamLoad("rls_Cr_est_min", rls_Cr_est_min_, 100000.0);          // 挂车后轴刚度最小值（N/m）
    logParamLoad("rls_Ct_est_min", rls_Ct_est_min_, 50000.0);          // 挂车横摆转动惯量最小值（kg·m²）
    logParamLoad("integration_grade", integration_grade_, 2.0);          // 求解器积分阶数，默认2阶

    // -------------------------------------------------------------------------
    // 6. 加载 Supervisor 配置 (模式切换与纯跟踪)
    // -------------------------------------------------------------------------
    ros::NodeHandle nh_super(nh, "supervisor_config");
    nh_super.param("startup_time", supervisor_params_.startup_time, 5.0);
    nh_super.param("blend_speed_low", supervisor_params_.blend_speed_low, 4.1667);
    nh_super.param("blend_speed_high", supervisor_params_.blend_speed_high, 5.0);
    nh_super.param("standstill_speed", supervisor_params_.standstill_speed, 0.05);
    nh_super.param("nmpc_speed_floor", supervisor_params_.nmpc_speed_floor, 1.0);
    nh_super.param("min_lookahead_distance", min_lookahead_distance_, 6.0);
    nh_super.param("lookahead_speed_coeff", lookahead_speed_coeff_, 0.7);

    // IPOPT
    logParamLoad("ipopt_max_iter", ipopt_max_iter_, 100.0);                      // IPPT最大迭代次数，默认100次
    logParamLoad("ipopt_acceptable_tol", ipopt_acceptable_tol_, 1e-2);               // IPPT可接受解的容差，默认1e-2
    logParamLoad("ipopt_acceptable_iter", ipopt_acceptable_iter_, 5.0);                 // IPPT可接受迭代次数，默认5次 连续 5 次迭代满足 acceptable_tol 即可停止
    logParamLoad("ipopt_warm_start_bound_push", ipopt_warm_start_bound_push_, 1e-3);        // IPPT预热边界推送系数，默认1e-3，用于初始化控制量边界，热启动时，将初始点向变量边界 “推” 的最小距离
    logParamLoad("ipopt_warm_start_slack_bound_push", ipopt_warm_start_slack_bound_push_, 1e-3);  // IPPT预热边界松弛系数，默认1e-3，用于初始化控制量边界,针对松弛变量
    logParamLoad("ipopt_warm_start_mult_bound_push", ipopt_warm_start_mult_bound_push_, 1e-3);   // IPPT预热边界松弛系数，默认1e-3，用于初始化控制量边界,针对拉格朗日乘子

    start_time_ = ros::Time::now(); // 记录控制器启动时间
    
    // 构建CasADi求解器（保留原函数名）
    buildNMPSolver();
    // 初始化铰接角发布器
    est_pub_ = nh.advertise<race_msgs::ESOEstimation>("/race/eso_estimation_states", 1);
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

    if (!vehicle_status || !path || !control_msg) {
        ROS_ERROR("[%s] 收到空指针消息", getName().c_str());
        return;
    }
    if (path->points.empty()) {
        ROS_WARN("[%s] 收到空路径，保持上一帧转角 %.3f rad", getName().c_str(), last_final_cmd_);
        control_msg->lateral.steering_angle = last_final_cmd_;
        control_msg->steering_mode = race_msgs::Control::FRONT_STEERING_MODE;
        control_msg->control_mode = race_msgs::Control::DES_ACCEL_ONLY;
        return;
    }

    // 提取当前状态
    const double curr_x = vehicle_status->pose.position.x;
    const double curr_y = vehicle_status->pose.position.y;
    const double curr_theta = vehicle_status->euler.yaw;
    const double curr_vx_raw = vehicle_status->vel.linear.x;
    const double v_abs = std::abs(curr_vx_raw);
    const double vx_floor = std::max(1.0, supervisor_params_.nmpc_speed_floor);
    const double curr_vx = std::max(v_abs, vx_floor);   // NMPC/观测器内部速度，避免低速奇异
    const double curr_ay = vehicle_status->acc.linear.y;
    const double curr_r = vehicle_status->vel.angular.z;
    double curr_delta = vehicle_status->lateral.steering_angle;
     // 根据收到的车辆状态更新挂车载货质量
    if (vehicle_status->trailer.mass > m_t_empty_default_) {
        nmpc_params_.m_t_total = vehicle_status->trailer.mass;
        ROS_INFO("[%s] 更新挂车总质量: %.2f kg", getName().c_str(), nmpc_params_.m_t_total);

    } else {
        nmpc_params_.m_t_total = m_t_total_default_;
        ROS_WARN("[%s] 收到异常挂车质量为%.2f kg , 使用默认挂车总质量: %.2f kg", getName().c_str(), vehicle_status->trailer.mass, nmpc_params_.m_t_total);
    }

    curr_delta = std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, curr_delta));

    static ros::Time last_control_time = ros::Time(0);
    ros::Time current_time = ros::Time::now();
    
    if (last_control_time.toSec() != 0.0 && (current_time - last_control_time).toSec() > 0.2) {
        ROS_WARN("[%s] 检测到控制重连，清空历史记忆与热启动！", getName().c_str());
        solver_.has_prev_sol = false;
        solver_.sol_prev = nullptr;
        current_cmd_ = curr_delta;
        last_final_cmd_ = curr_delta;
        nmpc_safe_cmd_ = curr_delta;
        blend_alpha_ = 0.0;
        is_high_speed_last_ = false;
        eso_x1_ = curr_r;
        eso_x2_ = 0.0;
        eso_initialized_ = false;
        ckf_x_hat_ << 0.0, curr_r, r_t_, gamma_;
        ckf_P_ = Matrix4d::Identity() * 1.0;
        r_filter_initialized_ = false;
        r_tractor_filt_ = curr_r;
        start_time_ = current_time;
    }
    last_control_time = current_time;
    const double obs_dt = std::max(dt, 0.01); 
    if (rls_w1_prev_ == 0.0 && curr_r != 0.0) {
        rls_w1_prev_ = curr_r;}
    // 1. 路径处理始终执行：低速也生成参考，保证 PP 与 NMPC 热启动一致
    std::vector<double> current_pose = {curr_x, curr_y, curr_theta, curr_vx};
    casadi::DM waypoints_dm = process_race_path(*path, current_pose);
    // 2. 观测器/估计器始终更新，但内部速度使用 vx_floor 防止低速分母异常
    calculate_trailer_kinematics(curr_vx, curr_r, obs_dt);
    double gamma_pre = gamma_;
    double r_t_pre = r_t_;
    // CKF 侧向速度估计：测量 z=[ay, r1, r2, gamma]^T，其中 r2/gamma 使用运动学估计作为伪测量。
    ckfEstimate(curr_vx, curr_delta, curr_ay, curr_r, r_t_pre, gamma_pre, obs_dt);
    const double vy_est = ckf_x_hat_(0);
    double curr_r_t = ckf_x_hat_(2);
    double curr_gamma = normalizeAngle(ckf_x_hat_(3));
    ROS_INFO("[%s] \033[36mCKF估计: vy=%.4f m/s, r1=%.4f rad/s, r_t=%.4f rad/s, gamma=%.4f rad\033[0m",
                      getName().c_str(), vy_est, ckf_x_hat_(1), curr_r_t, curr_gamma);

    rlsIdentifyStiffness(curr_vx, vy_est, curr_delta, curr_r, curr_ay, curr_gamma, curr_r_t, nmpc_params_.m_t_total, obs_dt);

    // esoCompute(curr_r, curr_delta, obs_dt);
    esoCompute(vy_est,curr_r,curr_delta,curr_r_t,curr_gamma,curr_vx,obs_dt);
    const double h_hat_total = eso_x2_;
    const double d_pure_trailer = h_hat_total;
    ROS_INFO("[%s] \033[35mESO观测结果: x1=%.4f rad/s, x2=%.4f, h_hat_total=%.4f\033[0m",
                      getName().c_str(), eso_x1_, eso_x2_, h_hat_total);

    // 3. 启动/低速/高速软切换权重：0=纯跟踪，1=NMPC
    const double time_elapsed = (current_time - start_time_).toSec();
    if (time_elapsed < supervisor_params_.startup_time) {
        blend_alpha_ = 0.0;
        ROS_INFO("[%s][STARTUP] 预热 %.1f / %.1f s，纯跟踪锁定",
                          getName().c_str(), time_elapsed, supervisor_params_.startup_time);
    } else if (v_abs <= supervisor_params_.blend_speed_low) {
        blend_alpha_ = 0.0;
    } else if (v_abs >= supervisor_params_.blend_speed_high) {
        blend_alpha_ = 1.0;
    } else {
        const double denom = std::max(1e-3, supervisor_params_.blend_speed_high - supervisor_params_.blend_speed_low);
        blend_alpha_ = (v_abs - supervisor_params_.blend_speed_low) / denom;
        blend_alpha_ = std::max(0.0, std::min(1.0, blend_alpha_));
    }
    const bool is_current_high_speed = (blend_alpha_ >= 0.99);

    // 4. NMPC 后台始终尝试求解，低速/启动阶段也刷新热启动；失败时保持上一帧 NMPC 有效值
    // NMPC 现在工作在“自车体坐标系”：原点为自车当前位置，x 轴沿自车当前航向。
    // 因此初始 x,y,theta 均为 0；vy/r/delta/r_t/gamma 仍为实际物理量
    // （挂车横摆率 r_t、铰接角 gamma 与全局朝向无关，保持真实值不变）。
    std::vector<double> nmpc_state = {0.0, 0.0, 0.0, vy_est, curr_r,
                                      curr_delta, curr_r_t, curr_gamma};
    std::vector<double> control_output(1, nmpc_safe_cmd_);

    std::vector<double> dyn_params = {
        nmpc_params_.m, nmpc_params_.Iz, nmpc_params_.lf, nmpc_params_.lr, rls_Cf_est_,
        rls_Cr_est_, nmpc_params_.m_t_total, nmpc_params_.Iz_t, nmpc_params_.lt, rls_Ct_est_, nmpc_params_.L2
    };
    solver_.opti.set_value(solver_.P_vx, curr_vx);
    solver_.opti.set_value(solver_.P_h_hat, d_pure_trailer);
    solver_.opti.set_value(solver_.P_dyn_params, dyn_params);

    const bool nmpc_solve_success = solveNMPC(nmpc_state, waypoints_dm, control_output);
    if (nmpc_solve_success) {
        nmpc_safe_cmd_ = control_output[0];
    } else {
        nmpc_safe_cmd_ = current_cmd_;
        ROS_WARN("[%s] NMPC 求解失败，NMPC支路保持上一帧有效输出 %.3f rad",
                          getName().c_str(), nmpc_safe_cmd_);
    }
    nmpc_safe_cmd_ = std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, nmpc_safe_cmd_));

    // 5. 纯跟踪保护支路：启动、静止、低速、NMPC失败时都可兜底；最终通过 blend_alpha_ 连续融合
    const double lookahead_dist = min_lookahead_distance_ + lookahead_speed_coeff_ * curr_vx;
    double pp_safe_cmd = computePurePursuitSteering(*path, curr_x, curr_y, curr_theta, lookahead_dist);
    pp_safe_cmd = std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, pp_safe_cmd));

    if (v_abs <= supervisor_params_.standstill_speed) {
        blend_alpha_ = 0.0;
        ROS_INFO("[%s][STANDSTILL] v=%.2f m/s，保持纯跟踪保护，不再输出固定零转角",
                          getName().c_str(), curr_vx_raw);
    }

    if (blend_alpha_ < 0.01) {
        ROS_INFO("[%s][PP] v=%.1f km/h | Ld=%.2f m | PP=%.3f | NMPC后台=%s",
                          getName().c_str(), curr_vx_raw * 3.6, lookahead_dist, pp_safe_cmd,
                          nmpc_solve_success ? "OK" : "FAIL");
    } else if (blend_alpha_ > 0.99) {
        ROS_INFO("[%s][NMPC] v=%.1f km/h | cmd=%.3f rad", getName().c_str(), curr_vx_raw * 3.6, nmpc_safe_cmd_);
    } else {
        ROS_INFO("[%s][BLEND] v=%.1f km/h | alpha=%.2f | PP=%.3f | NMPC=%.3f",
                          getName().c_str(), curr_vx_raw * 3.6, blend_alpha_, pp_safe_cmd, nmpc_safe_cmd_);
    }

    // 6. 融合输出 + 全局转角速率限制。用 last_final_cmd_ 作为最终输出历史，避免支路切换跳变。
    double final_cmd = blend_alpha_ * nmpc_safe_cmd_ + (1.0 - blend_alpha_) * pp_safe_cmd;
    final_cmd = std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, final_cmd));

    const double rate_limit = std::max(std::abs(nmpc_params_.delta_rate_max), std::abs(nmpc_params_.delta_rate_min));
    const double d_delta_max = std::max(0.01, rate_limit * obs_dt);
    final_cmd = std::max(last_final_cmd_ - d_delta_max, std::min(last_final_cmd_ + d_delta_max, final_cmd));

    last_final_cmd_ = final_cmd;
    current_cmd_ = final_cmd;
    is_high_speed_last_ = is_current_high_speed;

    control_msg->lateral.steering_angle = final_cmd;
    control_msg->steering_mode = race_msgs::Control::FRONT_STEERING_MODE;
    control_msg->control_mode = race_msgs::Control::DES_ACCEL_ONLY;

    // 状态参数对照输出
    double dr_nominal = calcNominalYawAccel(vy_est,curr_r,curr_delta,curr_r_t,curr_gamma,curr_vx);
    double dr_h_dist = dr_nominal + h_hat_total;
    double model_r2_ = curr_r + dr_h_dist*obs_dt;

    // 发布估计状态
    race_msgs::ESOEstimation est_msg;
    est_msg.r_t = curr_r_t;        // 挂车横摆率
    est_msg.gamma_angle = gamma_;        // 铰接角
    est_msg.vy_est2 = vy_est;          // 牵引车侧向速度
    est_msg.eso2_total = h_hat_total;     // ESO扰动估计
    est_msg.Cf_est2 = rls_Cf_est_;     // RLS Cf
    est_msg.Cr_est2 = rls_Cr_est_;     // RLS Cr
    est_msg.Ct_est2 = rls_Ct_est_;     // RLS Ct
    est_msg.model_r2 = model_r2_;     // 模型预测横摆率
    est_pub_.publish(est_msg);

}

// ---------------------- 路径处理与辅助函数----------------------
double ESOTracker2::normalizeAngle(double angle) {
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
    // yaw0 = 自车当前航向 curr_theta（由 process_race_path 透传 current_state[2]）。
    const double veh_yaw = yaw0;

    std::vector<double> s_orig, x_orig, y_orig, theta_orig, kappa_orig;

    // 关键修改：把每个参考点的航向都用 angleDiff 表示为“相对自车当前航向”的量，
    // 再沿弧长连续解缠绕(unwrap)，得到一条连续、且锚定在 0 附近的参考航向曲线。
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

    // 简单差分计算曲率 kappa
    kappa_orig.resize(theta_orig.size(), 0.0);
    for (size_t i = 1; i + 1 < theta_orig.size(); ++i) {
        double ds = s_orig[i+1] - s_orig[i-1];
        kappa_orig[i] = (ds > 1e-4) ? (theta_orig[i+1] - theta_orig[i-1]) / ds : 0.0;
    }

    // 根据动态 s_target 进行插值
    auto x_interp = linear_interpolate(s_orig, x_orig, s_target);
    auto y_interp = linear_interpolate(s_orig, y_orig, s_target);
    auto theta_interp = linear_interpolate(s_orig, theta_orig, s_target);
    auto kappa_interp = linear_interpolate(s_orig, kappa_orig, s_target);

    // 位置参考也转换到“自车体坐标系”（原点为自车当前位置，x 轴沿自车当前航向），
    // 与相对航向、相对动力学初值 (x=y=theta=0) 保持一致，彻底摆脱全局朝向的影响。
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

casadi::DM ESOTracker2::process_race_path(const race_msgs::Path& input_path, const std::vector<double>& current_state) {
    int nearest_idx = find_nearest_path_point(current_state[0], current_state[1], input_path);
    if (nearest_idx == -1) return casadi::DM::zeros(4, nmpc_params_.N + 1);

    // 设置体坐标系变换原点为自车当前位置，供 interpolate_path_segment 使用
    g_ref_x0 = current_state[0];
    g_ref_y0 = current_state[1];

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

//  --- 核心算法：CKF 状态估计 (替换原 EKF) ---
namespace {
inline bool finiteVec4(const Eigen::Vector4d& v) {
    return v.allFinite();
}

inline bool finiteMat4(const Eigen::Matrix4d& m) {
    return m.allFinite();
}

inline Eigen::Matrix4d symmetrize4(const Eigen::Matrix4d& p) {
    return 0.5 * (p + p.transpose());
}

inline double clampFinite(double value, double lo, double hi, double fallback) {
    if (!std::isfinite(value)) return fallback;
    return std::max(lo, std::min(hi, value));
}

// 返回协方差矩阵平方根 S，使 S*S^T 近似等于 P。
// 优先 LLT；失败时通过特征值裁剪恢复半正定，避免 CKF 因数值噪声中断。
inline Eigen::Matrix4d sqrtCovariance4(const Eigen::Matrix4d& p_in) {
    Eigen::Matrix4d P = symmetrize4(p_in);
    if (!finiteMat4(P)) {
        P = Eigen::Matrix4d::Identity();
    }

    double jitter = 1e-9;
    for (int i = 0; i < 8; ++i) {
        Eigen::LLT<Eigen::Matrix4d> llt(P + jitter * Eigen::Matrix4d::Identity());
        if (llt.info() == Eigen::Success) {
            return llt.matrixL();
        }
        jitter *= 10.0;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es(P);
    if (es.info() != Eigen::Success) {
        return Eigen::Matrix4d::Identity();
    }
    Eigen::Vector4d vals = es.eigenvalues();
    for (int i = 0; i < vals.size(); ++i) {
        vals(i) = std::sqrt(std::max(vals(i), 1e-8));
    }
    return es.eigenvectors() * vals.asDiagonal();
}

inline Eigen::Matrix4d regularizeCovariance4(const Eigen::Matrix4d& p_in, double min_diag = 1e-8, double max_diag = 1e4) {
    Eigen::Matrix4d P = symmetrize4(p_in);
    if (!finiteMat4(P)) {
        return Eigen::Matrix4d::Identity();
    }
    for (int i = 0; i < 4; ++i) {
        P(i, i) = clampFinite(P(i, i), min_diag, max_diag, 1.0);
    }
    P += min_diag * Eigen::Matrix4d::Identity();
    return symmetrize4(P);
}
} // anonymous namespace

Eigen::Vector4d ESOTracker2::compute_dynamics_3dof(const Eigen::Vector4d& x,
                                                   const Eigen::Vector2d& u) {
    const double vy = x(0);
    const double r1 = x(1);
    const double r2 = x(2);
    const double gamma = x(3);
    const double delta = u(0);
    const double vx_safe = std::max(u(1), 1.0);

    // 获取动力学参数
    const double m1 = nmpc_params_.m;
    const double m2 = nmpc_params_.m_t_total;
    const double Iz1 = nmpc_params_.Iz;
    const double Iz2 = nmpc_params_.Iz_t + nmpc_params_.Kiz * (m2 - nmpc_params_.m_t);
    const double a = nmpc_params_.lf;
    const double b = nmpc_params_.lr;
    const double c = b;
    const double d = nmpc_params_.lt;
    const double L2 = nmpc_params_.L2;
    const double e = std::max(0.1, L2 - d);
    const double Cf = std::max(1.0, rls_Cf_est_);
    const double Cr = std::max(1.0, rls_Cr_est_);
    const double Ct = std::max(1.0, rls_Ct_est_);

    // 轮胎侧偏角，使用 atan2 提高低速/符号切换鲁棒性。
    const double alpha_f = delta - std::atan2(vy + a * r1, vx_safe);
    const double alpha_r = -std::atan2(vy - b * r1, vx_safe);
    const double alpha_t = -std::atan2(vy - c * r1 - L2 * r2 - vx_safe * gamma, vx_safe);

    // 线性侧向力 + 平滑饱和，避免容积点跑到大侧偏角时力无限增大。
    const double Fyf_lin = Cf * alpha_f;
    const double Fyr_lin = Cr * alpha_r;
    const double Fyt_lin = Ct * alpha_t;
    const double mu = 0.85;
    const double g = 9.81;
    const double Fz_f = std::max(1.0, m1 * g * (b / std::max(0.1, a + b)));
    const double Fz_r = std::max(1.0, m1 * g * (a / std::max(0.1, a + b)) + m2 * g * (d / std::max(0.1, L2)));
    const double Fz_t = std::max(1.0, m2 * g * (e / std::max(0.1, L2)));
    const double Fyf = mu * Fz_f * std::tanh(Fyf_lin / std::max(1.0, mu * Fz_f));
    const double Fyr = mu * Fz_r * std::tanh(Fyr_lin / std::max(1.0, mu * Fz_r));
    const double Fyt = mu * Fz_t * std::tanh(Fyt_lin / std::max(1.0, mu * Fz_t));

    // 质量矩阵
    Eigen::Matrix3d M;
    M << m1 + m2,       -m2 * c,             -m2 * d,
         -m2 * c,        Iz1 + m2 * c * c,    m2 * c * d,
         -m2 * d,        m2 * c * d,          Iz2 + m2 * d * d;
    M += 1e-6 * Eigen::Matrix3d::Identity();

    // 广义力
    Eigen::Vector3d F;
    F(0) = Fyf * std::cos(delta) + Fyr + Fyt - (m1 + m2) * vx_safe * r1;
    F(1) = a * Fyf * std::cos(delta) - b * Fyr - c * Fyt + m2 * c * vx_safe * r1;
    F(2) = -L2 * Fyt + m2 * d * vx_safe * r1;

    Eigen::Vector3d accels = M.ldlt().solve(F);
    if (!accels.allFinite()) {
        accels.setZero();
    }

    Eigen::Vector4d x_dot;
    x_dot(0) = accels(0); // vy_dot
    x_dot(1) = accels(1); // r1_dot
    x_dot(2) = accels(2); // r2_dot
    x_dot(3) = r2 - r1;   // gamma_dot
    return x_dot;
}

Eigen::Vector4d ESOTracker2::vehicle_state_3dof(const Eigen::Vector4d& x,
                                                const Eigen::Vector2d& u,
                                                double dt) {
    const double dt_safe = std::max(1e-3, std::min(0.2, dt));

    // RK2 比欧拉积分更稳定，计算量对 4 维 CKF 仍很小。
    Eigen::Vector4d k1 = compute_dynamics_3dof(x, u);
    Eigen::Vector4d x_mid = x + 0.5 * dt_safe * k1;
    x_mid(3) = normalizeAngle(x_mid(3));
    Eigen::Vector4d k2 = compute_dynamics_3dof(x_mid, u);
    Eigen::Vector4d x_next = x + dt_safe * k2;

    const double vx_safe = std::max(u(1), 1.0);
    const double vy_limit = std::max(3.0, 0.7 * vx_safe);
    x_next(0) = clampFinite(x_next(0), -vy_limit, vy_limit, x(0));
    x_next(1) = clampFinite(x_next(1), -3.0, 3.0, x(1));
    x_next(2) = clampFinite(x_next(2), -3.0, 3.0, x(2));
    x_next(3) = clampFinite(normalizeAngle(x_next(3)), -1.3, 1.3, x(3));
    return x_next;
}

Eigen::Vector4d ESOTracker2::vehicle_meas_3dof(const Eigen::Vector4d& x,
                                               const Eigen::Vector2d& u) {
    const Eigen::Vector4d x_dot = compute_dynamics_3dof(x, u);
    const double vx_safe = std::max(u(1), 1.0);
    const double ay_pred = x_dot(0) + vx_safe * x(1);

    Eigen::Vector4d z_pred;
    z_pred << ay_pred, x(1), x(2), normalizeAngle(x(3));
    return z_pred;
}

void ESOTracker2::ckfEstimate(double curr_vx,double curr_delta,double curr_ay,double curr_r,double pseudo_r2,double pseudo_gamma,double dt) {
    constexpr int nx = 4, m_pts = 2 * nx;
    const double dt_safe = std::max(1e-3, std::min(0.2, dt));
    const double vx_safe = std::max(curr_vx, 1.0);
    Eigen::Vector2d u(curr_delta, vx_safe);
    Eigen::Vector4d z_meas(curr_ay, curr_r, pseudo_r2, normalizeAngle(pseudo_gamma));
    if (!finiteVec4(ckf_x_hat_) || !finiteMat4(ckf_P_)) {
        ckf_x_hat_ << 0.0, curr_r, pseudo_r2, normalizeAngle(pseudo_gamma);
        ckf_P_ = Eigen::Matrix4d::Identity();
    }
    // 首帧/重置后，用可测状态对 r1/r2/gamma 对齐，避免 CKF 初始瞬态过大。
    if (std::abs(ckf_x_hat_(1)) < 1e-6 && std::abs(curr_r) > 1e-6) {
        ckf_x_hat_(1) = curr_r;}
    if (std::abs(ckf_x_hat_(2)) < 1e-6 && std::abs(pseudo_r2) > 1e-6) {
        ckf_x_hat_(2) = pseudo_r2;}
    if (std::abs(ckf_x_hat_(3)) < 1e-6 && std::abs(pseudo_gamma) > 1e-6) {
        ckf_x_hat_(3) = normalizeAngle(pseudo_gamma);}
    // 过程噪声与测量噪声。ay 噪声不要设得过小，否则 vy 会追随加速度噪声抖动。
    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    Q.diagonal() << 5e-3, 5e-5, 1e-4, 2e-5;
    Eigen::Matrix4d R = Eigen::Matrix4d::Zero();
    R.diagonal() << 5e-2, 2.5e-4, 2e-3, 2e-3;
    // ==================== 1. 时间更新：预测步 ====================
    ckf_P_ = regularizeCovariance4(ckf_P_);
    const Eigen::Matrix4d S = sqrtCovariance4(ckf_P_);
    Eigen::Matrix<double, nx, m_pts> xi;
    xi.leftCols<nx>() = Eigen::Matrix4d::Identity();
    xi.rightCols<nx>() = -Eigen::Matrix4d::Identity();
    xi *= std::sqrt(static_cast<double>(nx));
    Eigen::Matrix<double, nx, m_pts> X_pts;
    for (int i = 0; i < m_pts; ++i) {
        const Eigen::Vector4d dx = (S * xi.col(i)).eval();
        X_pts.col(i) = ckf_x_hat_ + dx;
        X_pts(3, i) = normalizeAngle(X_pts(3, i));
    }
    Eigen::Matrix<double, nx, m_pts> X_pred_pts;
    for (int i = 0; i < m_pts; ++i) {
        X_pred_pts.col(i) = vehicle_state_3dof(X_pts.col(i), u, dt_safe);
    }
    Eigen::Vector4d x_pred = X_pred_pts.rowwise().mean();
    x_pred(3) = normalizeAngle(x_pred(3));
    Eigen::Matrix4d P_pred = Q;
    for (int i = 0; i < m_pts; ++i) {
        Eigen::Vector4d err = X_pred_pts.col(i) - x_pred;
        err(3) = normalizeAngle(err(3));
        P_pred += (err * err.transpose()) / static_cast<double>(m_pts);
    }
    P_pred = regularizeCovariance4(P_pred);
    // ==================== 2. 测量更新：校正步 ====================
    const Eigen::Matrix4d S_pred = sqrtCovariance4(P_pred);
    Eigen::Matrix<double, nx, m_pts> X_pred_pts_new;
    for (int i = 0; i < m_pts; ++i) {
        const Eigen::Vector4d dx = (S_pred * xi.col(i)).eval();
        X_pred_pts_new.col(i) = x_pred + dx;
        X_pred_pts_new(3, i) = normalizeAngle(X_pred_pts_new(3, i));
    }
    Eigen::Matrix<double, nx, m_pts> Z_pred_pts;
    for (int i = 0; i < m_pts; ++i) {
        Z_pred_pts.col(i) = vehicle_meas_3dof(X_pred_pts_new.col(i), u);
    }
    Eigen::Vector4d z_pred = Z_pred_pts.rowwise().mean();
    z_pred(3) = normalizeAngle(z_pred(3));
    Eigen::Matrix4d P_zz = R;
    Eigen::Matrix4d P_xz = Eigen::Matrix4d::Zero();
    for (int i = 0; i < m_pts; ++i) {
        Eigen::Vector4d err_x = X_pred_pts_new.col(i) - x_pred;
        Eigen::Vector4d err_z = Z_pred_pts.col(i) - z_pred;
        err_x(3) = normalizeAngle(err_x(3));
        err_z(3) = normalizeAngle(err_z(3));
        P_zz += (err_z * err_z.transpose()) / static_cast<double>(m_pts);
        P_xz += (err_x * err_z.transpose()) / static_cast<double>(m_pts);
    }
    P_zz = regularizeCovariance4(P_zz, 1e-7, 1e6);
    Eigen::LDLT<Eigen::Matrix4d> ldlt(P_zz);
    if (ldlt.info() != Eigen::Success) {
        ROS_WARN_THROTTLE(1.0, "[%s] CKF P_zz 分解失败，本帧仅使用预测值", getName().c_str());
        ckf_x_hat_ = x_pred;
        ckf_P_ = P_pred;
    } else {
        Eigen::Matrix4d K_gain = ldlt.solve(P_xz.transpose()).transpose();
        Eigen::Vector4d innovation = z_meas - z_pred;
        innovation(3) = normalizeAngle(innovation(3));
        ckf_x_hat_ = x_pred + K_gain * innovation;
        ckf_x_hat_(3) = normalizeAngle(ckf_x_hat_(3));
        ckf_P_ = P_pred - K_gain * P_zz * K_gain.transpose();
        ckf_P_ = regularizeCovariance4(ckf_P_);
    }
    // 物理限幅，防止异常加速度/伪测量导致 NMPC 初值跳变。
    const double vy_limit = std::max(3.0, 0.7 * vx_safe);
    ckf_x_hat_(0) = clampFinite(ckf_x_hat_(0), -vy_limit, vy_limit, 0.0);
    ckf_x_hat_(1) = clampFinite(ckf_x_hat_(1), -3.0, 3.0, curr_r);
    ckf_x_hat_(2) = clampFinite(ckf_x_hat_(2), -3.0, 3.0, pseudo_r2);
    ckf_x_hat_(3) = clampFinite(normalizeAngle(ckf_x_hat_(3)), -1.3, 1.3, normalizeAngle(pseudo_gamma));
}

// FF-RLS侧偏刚度辨识（保留原函数名，替换为3刚度辨识）
void ESOTracker2::rlsIdentifyStiffness(double curr_vx, double vy_est, double curr_delta,double curr_r, 
                                      double curr_ay, double curr_gamma, double curr_r_t, double M, double dt) {
    if (curr_vx < 3.0) {
        rls_Cf_est_ = rls_Cf_est_default_;
        rls_Cr_est_ = rls_Cr_est_default_;
        rls_Ct_est_ = rls_Ct_est_default_;
        rls_P_ = Matrix3d::Identity() * 1e4;
        return;
    }
    double lamda = 0.99999; // 遗忘因子，接近1表示慢速遗忘
    double lf1 = nmpc_params_.lf, lr1 = nmpc_params_.lr;
    double lf2 = nmpc_params_.lt, lh = nmpc_params_.lh;
    double lr2 = nmpc_params_.L2 - lf2;
    double m1 = nmpc_params_.m, m2 = nmpc_params_.m_t_total;
    double Iz1 = nmpc_params_.Iz, Iz2 = nmpc_params_.Iz_t + nmpc_params_.Kiz*(m2-nmpc_params_.m_t);
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
    // 计算侧偏角
    double alpha1 = curr_delta - atan((vy_est + lf1*curr_r)/vx1_safe);
    double alpha2 = -atan((vy_est - lr1*curr_r)/vx1_safe);
    double alpha3 = -atan((V2(1) - lr2*curr_r_t)/V2_x_safe);
    Vector3d alpha(alpha1, alpha2, alpha3);
    // RLS递推
    double max_alpha = alpha.cwiseAbs().maxCoeff();
    if (max_alpha > 0.01 && max_alpha < 0.15 && std::abs(curr_ay) > 0.1) {
        Vector3d Y = Fy; 
        Matrix3d Phi = alpha.asDiagonal();
        // 增益矩阵计算
        Matrix3d K = rls_P_ * Phi.transpose() * (lamda*Matrix3d::Identity() + Phi*rls_P_*Phi.transpose()).inverse();
        // 误差计算（带符号）
        Vector3d error = Y - Phi * Vector3d(rls_Cf_est_, rls_Cr_est_, rls_Ct_est_);
        // 状态更新
        rls_Cf_est_ += K(0,0)*error(0) + K(0,1)*error(1) + K(0,2)*error(2);
        rls_Cr_est_ += K(1,0)*error(0) + K(1,1)*error(1) + K(1,2)*error(2);
        rls_Ct_est_ += K(2,0)*error(0) + K(2,1)*error(1) + K(2,2)*error(2);
        // 物理限幅
        rls_Cf_est_ = std::max(std::min(rls_Cf_est_, rls_Cf_est_max_), rls_Cf_est_min_);
        rls_Cr_est_ = std::max(std::min(rls_Cr_est_, rls_Cr_est_max_), rls_Cr_est_min_);
        rls_Ct_est_ = std::max(std::min(rls_Ct_est_, rls_Ct_est_max_), rls_Ct_est_min_);
        // 协方差矩阵更新
        rls_P_ = (Matrix3d::Identity() - K*Phi) * rls_P_ / lamda;
    }
    // 输出平滑
    double smooth_factor = 0.01;
    rls_C_out_prev_ = smooth_factor * Vector3d(rls_Cf_est_, rls_Cr_est_, rls_Ct_est_) + (1 - smooth_factor) * rls_C_out_prev_;
    rls_Cf_est_ = rls_C_out_prev_(0);   
    rls_Cr_est_ = rls_C_out_prev_(1);
    rls_Ct_est_ = rls_C_out_prev_(2);
    // 输出侧偏刚度估计值（橙色）
    ROS_INFO("[%s] \033[33m侧偏刚度估计: Cf=%.1f N/rad, Cr=%.1f N/rad, Ct=%.1f N/rad\033[0m", 
                      getName().c_str(), rls_Cf_est_, rls_Cr_est_, rls_Ct_est_);
}


void ESOTracker2::calculate_trailer_kinematics(double curr_vx, double curr_r, double dt) {
    const double L2 = nmpc_params_.L2;
    const double Lh = nmpc_params_.lh;          // 建议用参数，不要写死0
    const double vx = std::max(curr_vx, 0.5);
    // 牵引车横摆率直接采用状态量，并做轻微低通抑制噪声
    const double tau_r = 0.08;                  // 可调: 0.05~0.15
    if (!r_filter_initialized_) {
        r_tractor_filt_ = curr_r;
        r_filter_initialized_ = true;}
    const double alpha_r = dt / (tau_r + dt);
    r_tractor_filt_ = (1.0 - alpha_r) * r_tractor_filt_ + alpha_r * curr_r;
    const double r_tractor = r_tractor_filt_;
    // 挂车横摆率运动学估计
    r_t_ = -(vx * std::sin(gamma_) + Lh * r_tractor * std::cos(gamma_)) / L2;
    const double gamma_dot = r_t_ - r_tractor;
    gamma_ += gamma_dot * dt;
}


MX ESOTracker2::vehicleDynamicsModel(const MX& state, const MX& cmd_delta,const MX& vx, const MX& h_dist,const MX& dyn_params) {
    // ================== 状态量读取 ==================
    MX theta = state(2);
    MX vy    = state(3);
    MX r     = state(4);
    MX delta = state(5);
    MX r_t   = state(6);
    MX gamma = state(7);
    // ================== 动力学参数读取 ==================
    MX m1  = dyn_params(0);
    MX Iz1 = dyn_params(1);
    MX lf  = dyn_params(2);
    MX lr  = dyn_params(3);
    MX Cf  = dyn_params(4);
    MX Cr  = dyn_params(5);
    MX M   = dyn_params(6);
    MX Iz2 = dyn_params(7);
    MX lt  = dyn_params(8);
    MX Ct  = dyn_params(9);
    MX L2  = dyn_params(10);
    MX m2 = M - m1;
    // ================== 数值保护 ==================
    MX vx_safe = fmax(vx, 0.5);
    MX cg = cos(gamma);
    MX sg = sin(gamma);
    // ================== 牵引车轮胎侧偏角与侧向力 ==================
    MX alpha_f = delta - atan2((vy + lf * r), vx_safe);
    MX alpha_r = -atan2((vy - lr * r), vx_safe);
    MX Fyf = Cf * alpha_f;
    MX Fyr = Cr * alpha_r;
    // ================== 铰接点速度 ==================
    MX vx_h1 = vx_safe;
    MX vy_h1 = vy - lr * r;
    MX vx_h2 = vx_h1 * cg + vy_h1 * sg;
    MX vy_h2 = -vx_h1 * sg + vy_h1 * cg;
    // ================== 挂车轴速度与挂车侧向力 ==================
    MX vx2 = vx_h2;
    MX vy2 = vy_h2 - L2 * r_t;
    MX vx2_safe = fmax(vx2, 0.5);
    MX alpha_t = -atan2(vy2, vx2_safe);
    MX Fyt = Ct * alpha_t;
    // ================== 广义力 F1, F2, F3 ==================
    MX Fyt_y1 = Fyt * cg;
    MX F1=Fyf*cos(delta)+Fyr+Fyt_y1-(m1+m2)*vx_safe*r-m2*lt*r_t*r_t*sg;
    MX F2=lf*Fyf*cos(delta)-lr*Fyr-lr*Fyt_y1+m2*lr*vx_safe*r+m2*lr*lt*r_t*r_t*sg;
    MX F3=-L2*Fyt+m2*lt*r*(vx_safe*cg+(vy-lr*r)*sg);
    MX A=m1+m2;
    MX S11=Iz1+m2*lr*lr-(m2*m2*lr*lr)/A;
    MX S12=m2*lr*lt*cg-(m2*m2*lr*lt*cg)/A;
    MX S22=Iz2+m2*lt*lt-(m2*m2*lt*lt*cg*cg)/A;
    MX b1=F2+(m2*lr/A)*F1;
    MX b2=F3+(m2*lt*cg/A)*F1;
    MX detS=S11*S22-S12*S12;
    // 这里加一个很小的正则项，避免符号优化中出现除零风险。
    MX detS_safe = detS + 1e-9;
    MX d_r_nominal=(b1*S22-b2*S12)/detS_safe;
    MX d_r_t=(S11*b2-S12*b1)/detS_safe;
    MX d_vy=(F1+m2*lr*d_r_nominal+m2*lt*cg*d_r_t)/A;
    // ================== ESO 扰动引入 ==================
    // MX d_r=d_r_nominal;
    MX d_r=d_r_nominal+h_dist;
    // ================== 运动学状态更新 ==================
    MX d_gamma=r_t-r;   
    MX d_x=vx_safe*cos(theta)-vy*sin(theta);
    MX d_y=vx_safe*sin(theta)+vy*cos(theta);
    MX d_theta=r;
    MX d_delta=(cmd_delta-delta)/nmpc_params_.T_lag;
    std::vector<MX> state_derivatives = {d_x,d_y,d_theta,d_vy,d_r,d_delta,d_r_t,d_gamma};
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
    solver_.P_dyn_params = solver_.opti.parameter(11); // 扩展为11维动力学参数

    // 控制量稀疏化
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

        if (integration_grade_ >= 0.5 && integration_grade_ < 1.5) { // Euler前向积分
            solver_.opti.subject_to(solver_.X(Slice(), k+1) == st + nmpc_params_.dt * vehicleDynamicsModel(st, con, solver_.P_vx, h, solver_.P_dyn_params));
        }

        else if (integration_grade_ >= 1.5 && integration_grade_ < 2.5) { // RK2积分
            // RK2积分（新增）
            MX k1 = vehicleDynamicsModel(st, con, solver_.P_vx, h, solver_.P_dyn_params);
            MX k2 = vehicleDynamicsModel(st + nmpc_params_.dt / 2.0 * k1, con, solver_.P_vx, h, solver_.P_dyn_params);
            solver_.opti.subject_to(solver_.X(Slice(), k+1) == st + nmpc_params_.dt * k2);
        }
        else if (integration_grade_ >= 3.5 && integration_grade_ < 4.5) { // RK4积分
            // RK4积分（保留原逻辑）
            MX k1 = vehicleDynamicsModel(st, con, solver_.P_vx, h, solver_.P_dyn_params);
            MX k2 = vehicleDynamicsModel(st + nmpc_params_.dt/2 * k1, con, solver_.P_vx, h, solver_.P_dyn_params);
            MX k3 = vehicleDynamicsModel(st + nmpc_params_.dt/2 * k2, con, solver_.P_vx, h, solver_.P_dyn_params);
            MX k4 = vehicleDynamicsModel(st + nmpc_params_.dt * k3, con, solver_.P_vx, h, solver_.P_dyn_params);
            solver_.opti.subject_to(solver_.X(Slice(), k+1) == st + nmpc_params_.dt/6 * (k1 + 2*k2 + 2*k3 + k4));
        }
        else {
            // 报错：integration_grade_不在有效范围内，无法选择积分方法
            throw std::runtime_error("integration_grade_不在有效范围内,无法选择积分方法");
        }

        // 代价函数（扩展挂车项）
        MX ref_x = solver_.P_waypoints(0, k+1), ref_y = solver_.P_waypoints(1, k+1);
        MX ref_theta = solver_.P_waypoints(2, k+1), ref_kappa = solver_.P_waypoints(3, k+1);
        // 体坐标系下，X(2) 与 ref_theta 均为连续、锚定在 0 附近的相对航向，
        // 不会跨越 ±pi 折叠边界，因此直接作差即可，无需 atan2(sin,cos) 折叠。
        // 移除折叠后，代价函数对航向误差全程光滑可导，消除 ±pi/2 附近的梯度突变。
        MX e_theta = solver_.X(2, k+1) - ref_theta;
        MX r_ref = solver_.P_vx * ref_kappa;
        MX gamma_dot_actual = solver_.X(6, k+1) - solver_.X(4, k+1);

        J += nmpc_params_.Q(0,0) * pow(solver_.X(0, k+1) - ref_x, 2);
        J += nmpc_params_.Q(1,1) * pow(solver_.X(1, k+1) - ref_y, 2);
        J += nmpc_params_.Q(2,2) * pow(e_theta, 2);
        J += nmpc_params_.Q(3,3) * pow(solver_.X(3, k+1), 2);
        J += nmpc_params_.Q(4,4) * pow(solver_.X(4, k+1) - r_ref, 2);

        // J += nmpc_params_.Q(6,6) * pow(solver_.X(6, k+1), 2);
        // J += nmpc_params_.Q(7,7) * pow(solver_.X(7, k+1), 2);
        // J += nmpc_params_.dgamma * pow(gamma_dot_actual, 2);
        J += nmpc_params_.R * pow(con, 2);
    }
    // 控制量平滑项（保留原逻辑）
    J += nmpc_params_.dR * pow(solver_.U_sparse(0) - solver_.P_u_prev, 2);
    for (int i=1; i<Nc; i++) J += nmpc_params_.dR * pow(solver_.U_sparse(i) - solver_.U_sparse(i-1), 2);

    // 硬约束（保留原逻辑）
    solver_.opti.subject_to(solver_.opti.bounded(nmpc_params_.delta_min, solver_.U_sparse, nmpc_params_.delta_max));

    solver_.opti.minimize(J);

    // IPOPT参数（Matlab实时性优化）
    Dict opts = {
        {"ipopt.print_level", 0}, 
        {"ipopt.sb", "yes"}, 
        {"ipopt.max_iter", ipopt_max_iter_},
        {"ipopt.tol", 1e-3},
        {"ipopt.acceptable_tol", ipopt_acceptable_tol_},
        {"ipopt.acceptable_iter", ipopt_acceptable_iter_},
        {"ipopt.mu_strategy", "adaptive"},
        {"print_time", 0},
        {"expand", true},

        {"ipopt.warm_start_init_point", "yes"},        // 明确告诉求解器接受热启动
        {"ipopt.warm_start_bound_push", ipopt_warm_start_bound_push_},        
        {"ipopt.warm_start_slack_bound_push", ipopt_warm_start_slack_bound_push_},
        {"ipopt.warm_start_mult_bound_push", ipopt_warm_start_mult_bound_push_}, 
        
        {"print_time", 0}
    };
    solver_.opti.solver("ipopt", opts);
}

// 求解NMPC
bool ESOTracker2::solveNMPC(const std::vector<double>& current_state, const casadi::DM& waypoints,
                          std::vector<double>& control_output) {

    auto start_time = std::chrono::high_resolution_clock::now();//记录开始的时间

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
        ROS_INFO("\033[32m[%s] NMPC 求解成功! 耗时: %.2f ms\033[0m", getName().c_str(), elapsed.count());
        
        solver_.sol_prev = std::make_unique<casadi::OptiSol>(sol);
        solver_.has_prev_sol = true;


        control_output[0] = static_cast<double>(sol.value(solver_.U_sparse(0)));
        return true;
    } catch (std::exception& e) {

         auto end_time = std::chrono::high_resolution_clock::now();//求解失败的耗时
       std::chrono::duration<double, std::milli> elapsed = end_time - start_time;
       ROS_WARN("\033[31m[%s] NMPC 求解失败! 耗时: %.2f ms, 原因: %s\033[0m", getName().c_str(), elapsed.count(), e.what());
        solver_.has_prev_sol = false;
        solver_.sol_prev = nullptr;
        return false;
    }
}

// ---------------------- [新增] 纯跟踪兜底保护实现 ----------------------
double ESOTracker2::computePurePursuitSteering(const race_msgs::Path& path,double curr_x, double curr_y,double curr_theta, double lookahead_dist) {
    if (path.points.empty()) return 0.0;
    // 1. 找最近点
    int nearest_idx = find_nearest_path_point(curr_x, curr_y, path);
    int target_idx = nearest_idx;
    // 2. 向前搜索预瞄距离内的目标点
    for (int i = nearest_idx; i < static_cast<int>(path.points.size()); ++i) {
        double dx = path.points[i].pose.position.x - curr_x;
        double dy = path.points[i].pose.position.y - curr_y;
        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist >= lookahead_dist) {
            target_idx = i;
            break;
        }
    }
    // 如果列表走完还没够到预瞄距离，就用最后一个点
    if (target_idx == nearest_idx) {
        target_idx = path.points.size() - 1;
    }
    // 3. 坐标转换到车体坐标系
    const auto& target_pt = path.points[target_idx].pose.position;
    double dx = target_pt.x - curr_x;
    double dy = target_pt.y - curr_y;
    // 旋转矩阵：将世界坐标误差转换为车体坐标
    double local_x = std::cos(curr_theta) * dx + std::sin(curr_theta) * dy;
    double local_y = -std::sin(curr_theta) * dx + std::cos(curr_theta) * dy;
    // 4. 纯跟踪公式计算
    double L = nmpc_params_.lf + nmpc_params_.lr; // 牵引车轴距
    double ld = std::max(lookahead_dist, std::sqrt(local_x*local_x + local_y*local_y));
    // 反正切计算前轮转角
    double delta_pp = std::atan2(2.0 * L * local_y, ld * ld);
    // 5. 限幅保护
    return std::max(nmpc_params_.delta_min, std::min(nmpc_params_.delta_max, delta_pp));
}

void ESOTracker2::esoCompute(double curr_vy,double curr_r,double curr_delta,double curr_r_t,
                             double curr_gamma,double curr_vx,double dt) {
    if (dt <= 1e-6) {return;}
    const double omega_o = 10.0;
    const double B1 = 2.0 * omega_o;
    const double B2 = omega_o * omega_o;
    if (!eso_initialized_) {
        eso_x1_ = curr_r;
        eso_x2_ = 0.0;
        eso_initialized_ = true;
    }
    const double d_r_nominal = calcNominalYawAccel(curr_vy,curr_r,curr_delta,curr_r_t,curr_gamma,curr_vx);
    const double error_eso = curr_r - eso_x1_;
    const double eso_x1_dot = d_r_nominal + eso_x2_ + B1 * error_eso;
    const double eso_x2_dot = B2 * error_eso;
    eso_x1_ += eso_x1_dot * dt;
    eso_x2_ += eso_x2_dot * dt;
    const double max_yaw_acc_dist = 5.0;  // rad/s^2
    eso_x2_ = std::max(-max_yaw_acc_dist,std::min(max_yaw_acc_dist, eso_x2_));
}

double ESOTracker2::calcNominalYawAccel(double curr_vy,double curr_r,double curr_delta,double curr_r_t,double curr_gamma,double curr_vx) {
    // ================== 参数读取 ==================
    const double m1  = nmpc_params_.m;
    const double Iz1 = nmpc_params_.Iz;
    const double lf  = nmpc_params_.lf;
    const double lr  = nmpc_params_.lr;
    const double Cf  = rls_Cf_est_;
    const double Cr  = rls_Cr_est_;
    const double Ct  = rls_Ct_est_;
    const double M   = nmpc_params_.m_t_total;
    const double Iz2 = nmpc_params_.Iz_t+ nmpc_params_.Kiz*(nmpc_params_.m_t_total- nmpc_params_.m_t);
    const double lt  = nmpc_params_.lt;
    const double L2  = nmpc_params_.L2;
    const double m2  = M-m1;
    // ================== 数值保护 ==================
    const double vx_safe = std::max(curr_vx, 0.5);
    const double cg = std::cos(curr_gamma);
    const double sg = std::sin(curr_gamma);
    // ================== 牵引车轮胎侧偏角与侧向力 ==================
    const double alpha_f = curr_delta - std::atan2(curr_vy + lf * curr_r, vx_safe);
    const double alpha_r = -std::atan2(curr_vy - lr * curr_r, vx_safe);
    const double Fyf = Cf * alpha_f;
    const double Fyr = Cr * alpha_r;
    // ================== 铰接点速度 ==================
    const double vx_h1 = vx_safe;
    const double vy_h1 = curr_vy-lr*curr_r;
    const double vx_h2 = vx_h1*cg+vy_h1*sg;
    const double vy_h2 = -vx_h1*sg+vy_h1*cg;
    // ================== 挂车轴速度与挂车侧向力 ==================
    const double vx2 = vx_h2;
    const double vy2 = vy_h2-L2*curr_r_t;
    const double vx2_safe = std::max(vx2, 0.5);
    const double alpha_t = -std::atan2(vy2, vx2_safe);
    const double Fyt = Ct * alpha_t;
    // ================== 广义力 F1, F2, F3 ==================
    const double Fyt_y1 = Fyt * cg;
    const double F1 = Fyf*std::cos(curr_delta)+Fyr+Fyt_y1-(m1+m2)*vx_safe*curr_r-m2*lt*curr_r_t*curr_r_t*sg;
    const double F2 = lf*Fyf*std::cos(curr_delta)-lr*Fyr-lr*Fyt_y1+m2*lr*vx_safe*curr_r+m2*lr*lt*curr_r_t*curr_r_t*sg;
    const double F3 = -L2*Fyt+m2*lt*curr_r*(vx_safe*cg+(curr_vy-lr*curr_r)*sg);
    const double A = m1 + m2;
    const double S11 = Iz1+m2*lr*lr-(m2*m2*lr*lr)/A;
    const double S12 = m2*lr*lt*cg-(m2*m2*lr*lt*cg)/A;
    const double S22 = Iz2+m2*lt*lt-(m2*m2*lt*lt)/A;
    const double b1 = F2+(m2*lr/A) * F1;
    const double b2 = F3+(m2*lt*cg/A) * F1;
    double detS = S11*S22-S12*S12;
    // ================== 防止除零 ==================
    if (std::abs(detS) < 1e-9) {detS = (detS >= 0.0) ? 1e-9 : -1e-9;}
    // ================== 求解牵引车横摆角加速度 ==================
    const double d_r_nominal=(b1*S22-b2*S12)/detS;
    return d_r_nominal;
}

} // namespace race_tracker

// 保留原插件注册（完全不变）
PLUGINLIB_EXPORT_CLASS(race_tracker::ESOTracker2, race_tracker::ControllerPluginBase)
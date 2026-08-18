#ifndef RACE_TRACKER_ESO_TRACKER_H
#define RACE_TRACKER_ESO_TRACKER_H

#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <memory>
#include <tf/transform_datatypes.h>
#include <deque>
#include <cstddef>
#include <fstream>
#include <array>
#include <string>

// ROS 插件和消息相关头文件
#include "race_tracker/controller_plugin_base.h"
#include <race_msgs/Control.h>
#include <race_msgs/VehicleStatus.h>
#include <race_msgs/Path.h>
#include <race_msgs/Flag.h>
#include <race_msgs/ESOEstimation.h>

namespace race_tracker {

// 监督层与纯跟踪参数
struct SupervisorParams {
    double startup_time;
    double blend_speed_low;
    double blend_speed_high;
};

// NMPC参数
struct NMPCParams {
    // --- 基础配置 ---
    double m;
    double Iz;
    double lf;
    double lr;
    double L;  // 当前质量节点下的轴距，始终由 lf + lr 更新
    double T_lag;
    double dt;
    int N;
    int Nc;
    int nx;
    int nu;

    // --- 转角幅值约束 ---
    double delta_max;
    double delta_min;
    double delta_rate_max;  // 前轮转角指令硬速率约束，rad/s

    // --- 随整车质量插值的车辆参数 ---
    // Iz、lf、lr、Cf、Cr 为当前质量下的实时插值结果；L 始终由 lf+lr 更新。
    double Cf;
    double Cr;
    std::vector<double> mass_interp_points;
    std::vector<double> Iz_interp_points;
    std::vector<double> lf_interp_points;
    std::vector<double> lr_interp_points;
    std::vector<double> Cf_interp_points;
    std::vector<double> Cr_interp_points;

    // --- 积分器 ---
    double integration_grade;
    double eso_disturbance_decay;  // ESO 扰动在预测域内的逐步保留系数 [0, 1]
    double eso_disturbance_tau_s;  // >0 时用 exp(-dt/tau) 生成上述系数；<=0 时使用旧系数

    // --- 代价函数权重 ---
    double Q_x, Q_y, Q_theta;
    double Q_vy, Q_r, Q_delta;
    double R;
    double dR;  // 控制增量软惩罚，不构成转角变化率硬约束
    int near_dense_control_steps;  // 预测域前部逐拍控制的步数，其余控制块自动均分

    double m_total;  // 质量插值的回退输入，kg


    Eigen::Matrix<double, 6, 6> Q;

    NMPCParams(); // 声明构造函数，在cpp中实现矩阵初始化
    void updateQMatrix();
};

// NMPC求解器结构
struct NMPSolver {
    casadi::Opti opti;
    casadi::MX X;
    casadi::MX U_sparse;
    casadi::MX P_x0;
    casadi::MX P_waypoints;
    casadi::MX P_vx;
    casadi::MX P_u_prev;
    casadi::MX P_h_hat;
    casadi::MX P_dyn_params;
    casadi::MX P_ay_slope_comp;
    casadi::MX U_full_feedback;
    casadi::MX U_full_command;
    std::vector<int> control_block_start;
    std::vector<int> control_block_length;
    std::unique_ptr<casadi::OptiSol> sol_prev;
    bool has_prev_sol;
};

// 核心控制器类继承自 ControllerPluginBase
class ESOTracker : public ControllerPluginBase {
public:
    ESOTracker();
    ~ESOTracker() override;

    // --- 核心 ROS 插件重载函数 ---
    bool initialize(ros::NodeHandle& nh) override;

    void computeControl(
        const race_msgs::VehicleStatusConstPtr& vehicle_status,
        const race_msgs::PathConstPtr& path,
        race_msgs::Control* control_msg,
        const double dt,
        const race_msgs::Flag::ConstPtr& flag) override;

    std::string getName() const override { return "ESOTracker"; }

private:
    // --- 算法核心函数  ---
    void buildNMPSolver();
    void initializeLocalLog();

    casadi::MX vehicleDynamicsModel(const casadi::MX& state, const casadi::MX& cmd_delta,
                                    const casadi::MX& vx, const casadi::MX& h_dist,
                                    const casadi::MX& dyn_params, const casadi::MX& ay_slope_comp);

    bool solveNMPC(const std::vector<double>& current_state, const casadi::DM& waypoints,
                   std::vector<double>& control_output);

    // 仅用于本地日志：缓存求解后的预测状态，不参与控制计算。
    void resetNmpcPredictionDiagnostics();


    void ukfEstimateVy(double curr_vx, double curr_delta, double curr_ay_corrected,
                       double curr_r, double lateral_disturbance, double dt,
                       bool measurement_is_new);

    // 根据 received_mass_ 对 Iz、lf、Cf、Cr 进行分段线性插值；区间外保持端点值。
    double interpolateWithClampedEnds(double mass,
                                      const std::vector<double>& mass_points,
                                      const std::vector<double>& value_points) const;
    bool validateMassInterpolationTables() const;
    void updateMassDependentParameters(double mass);

    void esoCompute(double curr_r, double curr_delta, double dt, bool measurement_is_new);

    double normalizeAngle(double angle);

    // --- ROS 与路径处理辅助函数  ---
    double quaternion_to_yaw(const geometry_msgs::Quaternion& q);
    int find_nearest_path_point(const double x0, const double y0, const double yaw0,
                                const race_msgs::Path& path);
    std::vector<double> calculate_cumulative_distance(const race_msgs::Path& path, int start_idx);
    std::vector<double> linear_interpolate(const std::vector<double>& s_original,
                                           const std::vector<double>& val_original,
                                           const std::vector<double>& s_target);
    casadi::DM interpolate_path_segment(const race_msgs::Path& path, const std::vector<double>& cum_dist,
                                        int start_idx, int end_idx, const std::vector<double>& s_target, double yaw0);
    casadi::DM process_race_path(const race_msgs::Path& input_path, const std::vector<double>& current_state);

    // 线性单轨稳态平衡前馈。返回 [delta_ff, vy_eq]；反馈和扰动观测继续负责模型失配。
    std::array<double, 2> computeSteadyStateFeedforward(double vx, double kappa) const;

    bool isNewVehicleMeasurement(double x, double y, double yaw, double vx, double vy,
                                 double r, double delta, double ay);


    // 侧向加速度零偏的准静态自校正：基于横向跟踪误差滑动窗口，非常缓慢地修正 ay 零偏估计
    void updateDynamicAyBias(double lateral_tracking_error);

private:
    ros::Publisher est_pub_;//发布话题
    double blend_alpha_;
    double nmpc_safe_cmd_;
    ros::Time start_time_;

    // 输出端一阶低通滤波，平滑方向盘高频抖动
    double output_lpf_tau_ = 0.0;   // 时间常数(s)，<=0 表示关闭
    double final_cmd_filt_ = 0.0;   // 滤波器状态
    bool   final_cmd_filt_init_ = false;


     // 动态预瞄参数
    double min_lookahead_distance_;
    double lookahead_speed_coeff_;
    int curvature_smoothing_steps_;          // 仅兼容旧配置/日志
    double curvature_smoothing_distance_m_;  // V6固定空间曲率平滑长度，不再随车速变化
    double lookahead_curvature_coeff_;  // 根据预瞄路径最大绝对曲率缩短预瞄距离，单位 m^2

    // V6名义稳态转角前馈
    bool use_equilibrium_feedforward_;
    double equilibrium_feedforward_gain_;
    double equilibrium_feedforward_limit_;
    double last_delta_ff_;

    // 路径投影在空间交叉处加入航向和车后点惩罚，避免纯欧氏最近点跳支路。
    double path_projection_heading_weight_m2_;
    double path_projection_heading_gate_rad_;
    double path_projection_rear_gate_m_;

    // --- 核心参数结构体 ---
    NMPCParams nmpc_params_;
    SupervisorParams supervisor_params_;
    NMPSolver solver_;

    double current_cmd_;

    // ESO观测器相关
    double eso_x1_;
    double eso_x2_;
    double model_r_comp_;//模型计算横摆角速度
    bool model_comp_initialized_;//是否初始化

    //输出模型计算量
    double Model_r1_;

    // UKF相关
    Eigen::Vector2d ukf_x_est_;
    Eigen::Matrix2d ukf_P_est_;

    // 标定特性
    double const_steer_bias_;

    // 横坡补偿
    bool use_slope_compensation_;
    double ay_slope_compensation_;
    double slope_compensation_coeff_;
    double slope_compensation_filter_tau_;
    bool ay_slope_compensation_initialized_;
    std::string slope_estimator_mode_;        // legacy_quasistatic 或 tire_force_residual
    bool slope_dynamic_gate_enabled_;
    double slope_gate_max_yaw_accel_;
    double slope_gate_max_steer_rate_;
    double slope_compensation_limit_;
    bool slope_gate_active_;
    bool slope_prev_valid_;
    double slope_prev_r_;
    double slope_prev_delta_;

    // UKF 与 NMPC 共用已估横向扰动，避免同一 ay 残差被重复解释为 vy。
    bool ukf_use_slope_disturbance_;
    double ukf_q_vy_;
    double ukf_q_r_;
    double ukf_r_ay_;
    double ukf_r_r_;
    double ukf_ay_innovation_limit_;
    double ukf_vy_abs_max_;
    double ukf_ay_innovation_raw_;
    double ukf_ay_innovation_used_;

    // 迭代时间
    double iter_time_ = 0.0;
    double total_control_time_ = 0.0;      // 整个控制周期耗时，单位ms

    // 侧向加速度零偏补偿
    bool use_ay_bias_compensation_;          // 总开关：false 时完全不做 ay 零偏补偿
    double const_ay_bias_;                  // 静态 ay 零偏初值/固定值，单位 m/s^2
    bool use_dynamic_ay_compensation_;      // true 时基于横向误差窗口进行准静态增量修正
    double ay_bias_estimate_;               // 当前 ay 零偏估计，动态模式下围绕 const_ay_bias_ 迭代
    double effective_ay_bias_;              // 本周期实际用于横坡补偿的 ay 零偏
    int dynamic_ay_error_window_size_;      // 横向误差滑动窗口长度，20Hz*10s 默认 200
    double dynamic_ay_error_threshold_;     // 窗口平均横向误差死区，单位 m
    double dynamic_ay_bias_learning_rate_;  // 每周期每米横向误差对应的 ay 偏置修正量
    double dynamic_ay_bias_max_step_;       // 单周期最大 ay 偏置修正量，防止过渡修正
    double dynamic_ay_bias_min_;            // ay 偏置估计下限
    double dynamic_ay_bias_max_;            // ay 偏置估计上限
    double dynamic_ay_bias_error_sign_;     // 横向误差均值到 ay 偏置修正方向的符号
    bool dynamic_ay_require_full_window_;   // true 时窗口填满后才允许更新
    std::deque<double> lateral_error_history_;

    bool auto_update_total_weight_; // 是否使用 EBS 整车质量更新插值输入
    double received_mass_ = 10000.0; // EBS接收到的整车质量，单位 kg；同时作为质量插值自变量

    // 在类中添加以下成员变量
    std::deque<double> pp_cmd_queue_;
    double control_time_;
    double control_delay_sec_;

    // 本地 CSV 诊断日志
    bool enable_local_log_ = false;
    std::string local_log_directory_ = "/tmp/eso_tracker_logs";
    int local_log_flush_interval_ = 100;
    int local_log_pending_rows_ = 0;
    std::string local_log_path_;
    std::ofstream local_log_stream_;

    // 分层归因日志的上一周期测量值，只用于计算原始差分/一步预测残差。
    bool diagnostic_prev_valid_ = false;
    double diagnostic_prev_delta_ = 0.0;
    double diagnostic_prev_vx_ = 0.0;
    double diagnostic_prev_vy_status_ = 0.0;
    double diagnostic_prev_r_ = 0.0;
    double diagnostic_prev_vy_ = 0.0;
    double diagnostic_prev_tracking_error_ = 0.0;
    double diagnostic_prev_geometric_error_ = 0.0;
    double diagnostic_prev_final_cmd_ = 0.0;
    int diagnostic_prev_nearest_idx_ = -1;

    // 车辆状态可能以保持值重复发布；重复测量只做观测器预测，不重复做测量校正。
    bool measurement_fingerprint_valid_ = false;
    double measurement_prev_x_ = 0.0;
    double measurement_prev_y_ = 0.0;
    double measurement_prev_yaw_ = 0.0;
    double measurement_prev_vx_ = 0.0;
    double measurement_prev_vy_ = 0.0;
    double measurement_prev_r_ = 0.0;
    double measurement_prev_delta_ = 0.0;
    double measurement_prev_ay_ = 0.0;
    bool last_measurement_is_new_ = true;
    bool last_observer_low_speed_reset_ = false;
    double observer_dynamic_min_speed_mps_ = 4.0;

    // NMPC 求解结果抽样：k=1、k=min(5,N)、k=N；状态顺序为 x,y,theta,vy,r,delta。
    std::array<double, 6> diagnostic_pred_k1_;
    std::array<double, 6> diagnostic_pred_k5_;
    std::array<double, 6> diagnostic_pred_kN_;
    std::array<double, 3> diagnostic_u_sparse_;

    // NMPC求解以及算法切换相关
    bool mpc_failure_flag_;
    bool using_pure_pursuit_flag_;
    bool require_over_take_flag_;
    bool using_mixed_mode_flag_;

    int mpc_failure_count_ = 0; // NMPC连续失败计数器
    int degrade_failure_times_; // NMPC连续失败次数阈值，超过该值则降级为纯跟踪模式
    int require_overtake_times_; // 连续要求超车次数阈值，超过该值则提示要求人工接管

};

} // namespace race_tracker

#endif // RACE_TRACKER_ESO_TRACKER_H

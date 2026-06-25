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
    double lookahead_distance;
};


// NMPC参数 
struct NMPCParams {
    // --- 基础配置 ---
    double m;
    double Iz;
    double lf;
    double lr;
    double T_lag;
    double dt;
    int N;
    int Nc;
    int nx;
    int nu;
    
    // --- 控制约束 ---
    double delta_max;
    double delta_min;
    double delta_c_max;

    // --- 轮胎参数 (含辨识上下限) ---
    double Cf;
    double Cr;
    double Cf_min;
    double Cf_max;
    double Cr_min;
    double Cr_max;

    // --- 积分器 ---
    double integration_grade;

    // --- 代价函数权重 ---
    double Q_x, Q_y, Q_theta;
    double Q_vy, Q_r, Q_delta;
    double R;
    double dR;
    double dR_dense;     // 对“稠密”转角序列(逐步)增量的惩罚，抑制段内阶梯抖动
    double R_ddelta;     // 对转角二阶差分的惩罚，专门抑制来回摆动(limit cycle)


    // 等效惯性量相关参数
    double m_total; // 车辆总质量，单位 kg
    double lg; // 第五轮到挂车等效轴距离，单位 m
    double lh; // 铰接点相对牵引车质心的纵向偏移，单位 m
    double Kiz; // 单位挂车质量增加的挂车横摆转动惯量，单位 kg·m²
 

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
    std::unique_ptr<casadi::OptiSol> sol_prev; 
    bool has_prev_sol;
};

// 核心控制器类继承自 ControllerPluginBase
class ESOTracker : public ControllerPluginBase {
public:
    ESOTracker();
    ~ESOTracker() override = default;

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

    casadi::MX vehicleDynamicsModel(const casadi::MX& state, const casadi::MX& cmd_delta,
                                    const casadi::MX& vx, const casadi::MX& h_dist,
                                    const casadi::MX& dyn_params, const casadi::MX& ay_slope_comp);

    bool solveNMPC(const std::vector<double>& current_state, const casadi::DM& waypoints,
                   std::vector<double>& control_output);

    
    void ukfEstimateVy(double curr_vx, double curr_delta, double curr_ay, double curr_r, double dt);
    
    void rlsIdentifyStiffness(double curr_vx, double vy_est, double curr_delta, 
                              double curr_r, double curr_ay, double dt);
    
    void esoCompute(double curr_r, double curr_delta, double dt);

    double normalizeAngle(double angle);

    // --- ROS 与路径处理辅助函数  ---
    double quaternion_to_yaw(const geometry_msgs::Quaternion& q);
    int find_nearest_path_point(const double x0, const double y0, const race_msgs::Path& path);
    std::vector<double> calculate_cumulative_distance(const race_msgs::Path& path, int start_idx);
    std::vector<double> linear_interpolate(const std::vector<double>& s_original, 
                                           const std::vector<double>& val_original, 
                                           const std::vector<double>& s_target);
    casadi::DM interpolate_path_segment(const race_msgs::Path& path, const std::vector<double>& cum_dist, 
                                        int start_idx, int end_idx, const std::vector<double>& s_target, double yaw0);
    casadi::DM process_race_path(const race_msgs::Path& input_path, const std::vector<double>& current_state);
    
    std::vector<double> vehicleStatusToStateVector(const race_msgs::VehicleStatus& status);

    // 构建状态估计与方案二对比
    void calculate_trailer_kinematics(double curr_vx, double curr_r, double dt);
    void ekfEstimateVy(double curr_vx, double curr_delta, double curr_ay, double curr_r, double M, double dt);

    // 侧向加速度零偏的准静态自校正：基于横向跟踪误差滑动窗口，非常缓慢地修正 ay 零偏估计
    void updateDynamicAyBias(double lateral_tracking_error);

private:
    ros::Publisher est_pub_;//发布话题
    bool is_high_speed_last_;
    double blend_alpha_;
    double nmpc_safe_cmd_;
    ros::Time start_time_;
    double last_final_cmd_;

    // 输出端一阶低通滤波，平滑方向盘高频抖动
    double output_lpf_tau_ = 0.0;   // 时间常数(s)，<=0 表示关闭
    double final_cmd_filt_ = 0.0;   // 滤波器状态
    bool   final_cmd_filt_init_ = false;
    
     // 动态预瞄参数
    double min_lookahead_distance_;
    double lookahead_speed_coeff_;

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
    double kappa_;
    double r_ref;
    double theta;
    double vy_model;

    // UKF相关
    Eigen::Vector2d ukf_x_est_;
    Eigen::Matrix2d ukf_P_est_;
    // 新增EKF对照用
    Eigen::Vector4d ekf_x_hat_;
    Eigen::Matrix4d ekf_P_;
    double vy_ekf_est_{0.0};
    // 挂车状态估计参照
    double r_tractor_filt_ = 0.0;
    bool r_filter_initialized_ = false;
    double gamma_ = 0.0;
    double r_t_ = 0.0;

    // RLS相关
    double rls_P_f_;
    double rls_theta_f_;
    double rls_P_r_;
    double rls_theta_r_;
    double rls_r_prev_;
    double rls_r_dot_pre_;
    double rls_Cf_est_;
    double rls_Cr_est_;

    // 标定特性
    double const_steer_bias_;

    // 横坡补偿
    bool use_slope_compensation_;
    double ay_slope_compensation_;
    double slope_compensation_coeff_;
    int slope_compensation_filter_window_size_;
    std::deque<double> ay_slope_compensation_history_;

    // 迭代时间
    double iter_time_ = 0.0;

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

    // 等效惯性量相关参数
    bool use_equivalent_inertia_; // 是否使用等效重量（方案二融合的部分，是否将挂车惯性量融合进来）
    bool auto_update_total_weight_; // 是否根据话题信息自动更新整车重量（包含挂车的整备质量）
    double m_eq_y_ = 0.0; // 等效重量，单位 kg
    double I_eq_ = 0.0; // 等效惯性量，单位 kg·m²

    // 在类中添加以下成员变量
    std::deque<double> pp_cmd_queue_;
    double control_time_;
    double control_delay_sec_; 
};

} // namespace race_tracker

#endif // RACE_TRACKER_ESO_TRACKER_H
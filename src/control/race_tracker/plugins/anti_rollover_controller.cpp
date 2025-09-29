#include "race_tracker/anti_rollover_controller.h"
#include <numeric>
#include <limits>
#include <ros/console.h>
using namespace casadi;
using namespace std;

namespace race_tracker {

bool AntiRolloverController::initialize(ros::NodeHandle& nh) {
    // 1. 打印原命名空间（确认基础路径）
    ROS_INFO("[AntiRolloverController] 父NodeHandle命名空间: %s", nh.getNamespace().c_str());
    // 2. 创建插件专属的子NodeHandle（匹配Launch中的ns="anti_rollover_controller"）
    ros::NodeHandle nh_nmpc(nh, "anti_rollover_controller");  // 子命名空间：父ns + "/anti_rollover_controller"
    ROS_INFO("[AntiRolloverController] 插件专属NodeHandle命名空间: %s", nh_nmpc.getNamespace().c_str());

    // 加载核心参数
    nh_nmpc.param("nx", nx_, 6); // 状态：[x, y, theta, gamma, ltr, dltr]
    nh_nmpc.param("nu", nu_, 1); // 输入：[delta]
    nh_nmpc.param("prediction_step", N_, 15);
    nh_nmpc.param("sparse_control_step", Nc_, 4);
    nh_nmpc.param("sampling_time", dt_, 0.1);
    nh_nmpc.param("tractor_wheelbase", L1_, 2.8);
    nh_nmpc.param("trailer_wheelbase", L2_, 11.5);
    nh_nmpc.param("gravity", g_, 9.806);
    nh_nmpc.param("max_acceleration", a_max_, 3.0);
    nh_nmpc.param("num_waypoints", n_waypoints_, 30);
    // 加载控制量边界
    nh_nmpc.param("steer_limit", delta_limit_, 0.4);
    // 加载状态量边界
    nh_nmpc.param("ltr_limit", ltr_limit_, 0.6);
    // 加载代价函数权重
    nh_nmpc.param("weight_position", w_pos_, 5.0);
    nh_nmpc.param("weight_heading", w_theta_, 3.0);
    nh_nmpc.param("weight_front_steer", w_delta_, 500.0);
    nh_nmpc.param("weight_terminal_position", w_term_pos_, 10.0);
    nh_nmpc.param("weight_terminal_heading", w_term_theta_, 5.0);
    nh_nmpc.param("weight_ltr_constraint", w_ltr_constraint_, 10000.0);
    // 加载动力学参数
    nh_nmpc.param("a1", a1_, -1.5);  // 示例默认值，需根据实际车型调整
    nh_nmpc.param("a2", a2_, -1.5); // 示例默认值
    nh_nmpc.param("b1", b1_, 0.05); // 关键：加载 b1_，设默认值避免随机

    // 打印验证
    printf("==========加载动力学参数==========\n");
    logParamLoad("a1", a1_, -1.5);
    logParamLoad("a2", a2_, -1.5);
    logParamLoad("b1", b1_, 0.05);
    // 打印加载的参数
    printf("==========加载核心参数==========\n");
    logParamLoad("nx", nx_, 6);
    logParamLoad("nu", nu_, 1);
    logParamLoad("prediction_step", N_, 15);
    logParamLoad("sparse_control_step", Nc_, 4);
    logParamLoad("sampling_time", dt_, 0.1);
    logParamLoad("tractor_wheelbase", L1_, 2.8);
    logParamLoad("trailer_wheelbase", L2_, 11.5);
    logParamLoad("gravity", g_, 9.806);
    logParamLoad("max_acceleration", a_max_, 3.0);
    logParamLoad("num_waypoints", n_waypoints_, 30);
    printf("==========加载控制量边界参数==========\n");
    logParamLoad("steer_limit", delta_limit_, 0.4);
    printf("==========加载状态量边界参数==========\n");
    logParamLoad("ltr_limit", ltr_limit_, 0.6);
    printf("==========加载权重参数==========\n");
    logParamLoad("weight_position", w_pos_, 5.0);
    logParamLoad("weight_heading", w_theta_, 3.0);
    logParamLoad("weight_front_steer", w_delta_, 500.0);
    logParamLoad("weight_terminal_position", w_term_pos_, 10.0);
    logParamLoad("weight_terminal_heading", w_term_theta_, 5.0);
    logParamLoad("weight_ltr_constraint", w_ltr_constraint_, 10000.0);


    // 初始化稀疏控制量分布
    steps_per_control_.resize(Nc_);
    int remaining_steps = N_;
    for(int i = 0; i < Nc_; ++i) {
        if(i < Nc_ - 1) {
            steps_per_control_[i] = i + 1;
            remaining_steps -= steps_per_control_[i];
        } else {
            steps_per_control_[i] = remaining_steps;
        }
    }

    // 验证总步数
    int total_steps = std::accumulate(steps_per_control_.begin(), steps_per_control_.end(), 0);
    if (total_steps != N_) {
        ROS_WARN("[%s] 稀疏控制量总步数(%d)与预测步长(%d)不匹配，将自动调整", 
                 getName().c_str(), total_steps, N_);
        steps_per_control_.back() += (N_ - total_steps);
    }

    // 定义符号变量与动力学模型
    casadi::MX X_sym = casadi::MX::sym("X", nx_);  // 状态变量 [x,y,theta,gamma,ltr,dltr]
    casadi::MX U_sym = casadi::MX::sym("U", nu_);  // 控制变量 [delta]
    casadi::MX vx_sym = casadi::MX::sym("vx", 1);  // 速度参数 [vx]
    casadi::MX theta_sym = vx_sym * casadi::MX::tan(U_sym(0)) / L1_;
    

    // 动力学方程（连续时间）
    // x_dot = vx * cos(theta)
    // y_dot = vx * sin(theta)
    // theta_dot = vx * tan(delta) / L1
    // gamma_dot = theta_dot - vx * tan(delta) / L2
    // ltr_dot = dltr 
    // dltr_dot = a1*ltr + a2*dltr + b1*vx*theta*gamma
    casadi::MX f_expr = casadi::MX::vertcat({
        vx_sym * casadi::MX::cos(X_sym(2)),  // x_dot
        vx_sym * casadi::MX::sin(X_sym(2)),  // y_dot
        theta_sym,  // theta_dot
        theta_sym - vx_sym/L2_ * casadi::MX::sin(X_sym(3)), // gamma_dot
        X_sym(5) ,  // ltr_dot
        a1_ * X_sym(4) + a2_ * X_sym(5) + b1_*vx_sym* X_sym(2)*X_sym(3)  // dltr_dot
    });
    f_func_ = casadi::Function("f_dynamics", {X_sym, U_sym, vx_sym}, {f_expr});

    // RK4离散化（数值积分）
    auto rk4_integrate = [&](casadi::MX x, casadi::MX u, casadi::MX v, double dt) {
        std::vector<casadi::MX> args_k1 = {x, u, v};
        casadi::MX k1 = f_func_(args_k1)[0];

        std::vector<casadi::MX> args_k2 = {x + dt/2*k1, u, v};
        casadi::MX k2 = f_func_(args_k2)[0];

        std::vector<casadi::MX> args_k3 = {x + dt/2*k2, u, v};
        casadi::MX k3 = f_func_(args_k3)[0];

        std::vector<casadi::MX> args_k4 = {x + dt*k3, u, v};
        casadi::MX k4 = f_func_(args_k4)[0];

        return x + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
    };

    // 构建NMPC优化问题
    opti_ = casadi::Opti();
    X_ = opti_.variable(nx_, N_+1);      // 状态序列（N+1个点：0→N）
    U_sparse_ = opti_.variable(nu_, Nc_); // 稀疏控制量（Nc个控制量）
    epsilon_ = opti_.variable(1);        // 松弛变量
    x0_ = opti_.parameter(nx_);         // 初始状态参数
    vx_ = opti_.parameter(1);          // 初始速度参数
    waypoints_ = opti_.parameter(4, n_waypoints_); // 参考路点参数

    // 生成完整控制序列（稀疏控制量扩展到N步）
    casadi::MX U_full = casadi::MX::zeros(nu_, N_); 
    int time_step = 0;
    for(int i = 0; i < Nc_; ++i) {
        int end_step = time_step + steps_per_control_[i];
        end_step = std::min(end_step, N_);
        U_full(casadi::Slice(), casadi::Slice(time_step, end_step)) = 
            casadi::MX::repmat(U_sparse_(casadi::Slice(), i), 1, end_step - time_step);
        time_step = end_step;
        if(time_step >= N_) break;
    }


    // 动力学约束（状态递推）
    for(int k = 0; k < N_; ++k) {
        opti_.subject_to(X_(casadi::Slice(), k+1) == rk4_integrate(X_(casadi::Slice(), k), U_full(casadi::Slice(), k), vx_, dt_));
    }

    // 初始状态约束
    opti_.subject_to(X_(casadi::Slice(), 0) == x0_);

    // 控制量边界约束
    opti_.subject_to(opti_.bounded(-delta_limit_, U_sparse_(0, casadi::Slice()), delta_limit_));
    // 状态量边界软约束
    opti_.subject_to(opti_.bounded(-ltr_limit_ - epsilon_, X_(4, casadi::Slice()), ltr_limit_ + epsilon_));
    opti_.subject_to(opti_.bounded(0.0, epsilon_, 1000));


    // 代价函数（状态跟踪+控制平滑）
    casadi::MX cost = 0;

    // 预测步代价（每个时间步的跟踪误差）
    for(int k = 0; k < N_; ++k) {
        // 当前状态位置
        casadi::MX curr_pos = X_(casadi::Slice(0,2), k);  // (x,y)
        
        // 计算到所有参考路点的距离平方，找到最近路点
        casadi::MX pos_diff = curr_pos - waypoints_(casadi::Slice(0,2), casadi::Slice());
        casadi::MX dist_sq = casadi::MX::sum1(pos_diff * pos_diff);
        casadi::MX min_idx = 0;
        for(int i = 1; i < n_waypoints_; ++i) {
            min_idx = casadi::MX::if_else(dist_sq(i) < dist_sq(min_idx), i, min_idx);
        }

        // 提取最近路点的参考信息
        casadi::MX ref_theta = waypoints_(2, min_idx);  // 参考航向角
        // 状态跟踪代价
        cost += w_pos_ * dist_sq(min_idx);
        cost += w_theta_ * casadi::MX::sumsqr(X_(2, k) - ref_theta - X_(2, 0));

        // 前轴转向角代价
        cost += w_delta_ * casadi::MX::sumsqr(U_full(0, k));

    }

    // 终端代价
    casadi::MX final_pos = X_(casadi::Slice(0,2), N_);
    casadi::MX final_pos_diff = final_pos - waypoints_(casadi::Slice(0,2), casadi::Slice());
    casadi::MX final_dist_sq = casadi::MX::sum1(final_pos_diff * final_pos_diff);
    casadi::MX final_min_idx = 0;
    for(int i = 1; i < n_waypoints_; ++i) {
        final_min_idx = casadi:: MX::if_else(final_dist_sq(i) < final_dist_sq(final_min_idx), i, final_min_idx);
    }
    casadi::MX final_ref_theta = waypoints_(2, final_min_idx);


    cost += w_term_pos_ * final_dist_sq(final_min_idx);
    cost += w_term_theta_ * casadi::MX::sumsqr(X_(2, N_) - final_ref_theta);

    // 松弛变量代价
    cost += w_ltr_constraint_* N_ * casadi::MX::sumsqr(epsilon_);

    // 最小化总代价
    opti_.minimize(cost);

    // 求解器配置（IPOPT）
    casadi::Dict solver_opts;
    solver_opts["ipopt.print_level"] = 0;
    solver_opts["print_time"] = true;
    solver_opts["ipopt.sb"] = "yes";
    solver_opts["ipopt.max_iter"] = 500;
    solver_opts["ipopt.tol"] = 1e-2;
    solver_opts["ipopt.acceptable_tol"] = 5e-2;
    solver_opts["ipopt.acceptable_iter"] = 10;
    solver_opts["ipopt.hessian_approximation"] = "limited-memory";
    solver_opts["ipopt.linear_solver"] = "ma27";

    opti_.solver("ipopt", solver_opts);

    // 初始化求解状态（关键修改：智能指针初始化为nullptr）
    sol_prev_ = nullptr;  // 显式初始化，避免野指针
    // 初始化求解状态
    has_prev_sol_ = false;

    ROS_INFO("[%s] NMPC控制器初始化完成", getName().c_str());
    return true;
}

void AntiRolloverController::computeControl(
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

    // 将车辆状态转换为状态向量
    std::vector<double> current_state = vehicleStatusToStateVector(*vehicle_status);

    // 提取当前速度
    double vx = vehicleStateToVx(*vehicle_status);

    // 处理路径生成参考路点
    casadi::DM waypoints_dm = process_race_path(*path, current_state, vx);

    // 求解NMPC
    std::vector<double> control_output(1); // [delta]
    if (!solveNMPC(current_state, waypoints_dm, control_output, vx)) {
        ROS_WARN("[%s] NMPC求解失败，使用上一次控制量: delta = %f", getName().c_str(), last_control_output_[0]);

        // 如果求解失败，使用上一次的控制量
        if (!last_control_output_.empty()) {
            control_output = last_control_output_;
        }
        else {
            // 如果没有上一次的控制量，使用默认值
            control_output = {0.0};
        }

    } else {
        // 如果求解成功，更新上一次的控制量
        last_control_output_ = control_output;
    }

    // 将求解结果转换为控制消息
    control_msg->lateral.steering_angle = control_output[0];
    
    // 设置双轴转向模式
    control_msg->steering_mode = race_msgs::Control::FRONT_STEERING_MODE;
    
    // 控制模式设置为加速度模式
    control_msg->control_mode = race_msgs::Control::THROTTLE_BRAKE_ONLY;
}

double AntiRolloverController::quaternion_to_yaw(const geometry_msgs::Quaternion& q) {
    tf::Quaternion tf_quat(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 rot_matrix(tf_quat);
    double roll, pitch, yaw;
    rot_matrix.getRPY(roll, pitch, yaw);
    return yaw;
}

int AntiRolloverController::find_nearest_path_point(const double x0, const double y0, const race_msgs::Path& path) {
    if (path.points.empty()) {
        ROS_ERROR("[%s] 路径为空", getName().c_str());
        return -1;
    }

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

std::vector<double> AntiRolloverController::calculate_cumulative_distance(const race_msgs::Path& path, int start_idx) {
    std::vector<double> cum_dist;
    if (start_idx < 0 || start_idx >= static_cast<int>(path.points.size())) {
        ROS_ERROR("[%s] 无效的起始索引: %d", getName().c_str(), start_idx);
        return cum_dist;
    }

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

std::vector<double> AntiRolloverController::linear_interpolate(const std::vector<double>& s_original, 
                                                     const std::vector<double>& val_original, 
                                                     const std::vector<double>& s_target) {
    std::vector<double> val_target;
    val_target.reserve(s_target.size());

    if (s_original.size() != val_original.size() || s_original.size() < 2) {
        ROS_WARN("[%s] 原始数据无效，使用常数插值", getName().c_str());
        double fill_val = (val_original.empty()) ? 0.0 : val_original[0];
        for (size_t i = 0; i < s_target.size(); ++i) {
            val_target.push_back(fill_val);
        }
        return val_target;
    }

    double s_min = s_original[0];
    double s_max = s_original.back();

    for (double s_t : s_target) {
        if (s_t <= s_min) {
            val_target.push_back(val_original[0]);
            continue;
        }
        if (s_t >= s_max) {
            val_target.push_back(val_original.back());
            continue;
        }

        size_t i = 0;
        while (i < s_original.size() - 1 && s_original[i+1] < s_t) {
            ++i;
        }

        double s_i = s_original[i];
        double s_j = s_original[i+1];
        double val_i = val_original[i];
        double val_j = val_original[i+1];
        
        double ratio = (s_t - s_i) / (s_j - s_i);
        double val_t = val_i + ratio * (val_j - val_i);
        val_target.push_back(val_t);
    }

    return val_target;
}

casadi::DM AntiRolloverController::interpolate_path_segment(const race_msgs::Path& path, const std::vector<double>& cum_dist, 
                                                  int start_idx, int end_idx, int n_waypoints, double yaw0) {
    std::vector<double> s_original, x_original, y_original, theta_original, v_original;
    for (int i = start_idx; i <= end_idx; ++i) {
        const auto& path_pt = path.points[i];
        int rel_idx = i - start_idx;
        
        s_original.push_back(cum_dist[rel_idx]);
        x_original.push_back(path_pt.pose.position.x);
        y_original.push_back(path_pt.pose.position.y);
        double original_yaw = quaternion_to_yaw(path_pt.pose.orientation) - yaw0;
        if (original_yaw > M_PI) {
            original_yaw -= 2 * M_PI;
        } else if (original_yaw < -M_PI) {
            original_yaw += 2 * M_PI;
        }
        theta_original.push_back(original_yaw);
        v_original.push_back(path_pt.velocity);
    }

    std::vector<double> s_target(n_waypoints);
    if (s_original.empty()) {
        ROS_ERROR("[%s] 原始累积距离为空", getName().c_str());
        return casadi::DM::zeros(4, n_waypoints);
    }
    double s_max = s_original.back();
    for (int i = 0; i < n_waypoints; ++i) {
        s_target[i] = s_max * static_cast<double>(i) / (n_waypoints - 1);
    }

    std::vector<double> x_interp = linear_interpolate(s_original, x_original, s_target);
    std::vector<double> y_interp = linear_interpolate(s_original, y_original, s_target);
    std::vector<double> theta_interp = linear_interpolate(s_original, theta_original, s_target);
    std::vector<double> v_interp = linear_interpolate(s_original, v_original, s_target);

    casadi::DM waypoints = casadi::DM::zeros(4, n_waypoints);
    for (int i = 0; i < n_waypoints; ++i) {
        waypoints(0, i) = x_interp[i];
        waypoints(1, i) = y_interp[i];
        waypoints(2, i) = theta_interp[i];
        waypoints(3, i) = v_interp[i];
    }

    return waypoints;
}

casadi::DM AntiRolloverController::process_race_path(const race_msgs::Path& input_path, const std::vector<double>& current_state, double vx0) {
    double x0 = current_state[0];
    double y0 = current_state[1];
    double yaw0 = current_state[2];

    int nearest_idx = find_nearest_path_point(x0, y0, input_path);
    if (nearest_idx == -1) {
        ROS_ERROR("[%s] 找不到最近路点", getName().c_str());
        return casadi::DM::zeros(4, n_waypoints_);
    }

    double pred_time = N_ * dt_;
    double max_distance = vx0 * pred_time + 0.5 * a_max_ * pred_time * pred_time;
    ROS_DEBUG("[%s] 当前速度: %.2f m/s, 预测时间: %.2f s, 最大跟踪距离: %.2f m",
             getName().c_str(), vx0, pred_time, max_distance);

    std::vector<double> cum_dist = calculate_cumulative_distance(input_path, nearest_idx);
    if (cum_dist.empty()) {
        ROS_ERROR("[%s] 累积距离计算失败", getName().c_str());
        return casadi::DM::zeros(4, n_waypoints_);
    }

    int end_idx = nearest_idx;
    for (size_t i = 0; i < cum_dist.size(); ++i) {
        if (cum_dist[i] > max_distance) {
            end_idx = nearest_idx + static_cast<int>(i);
            break;
        }
        if (i == cum_dist.size() - 1) {
            end_idx = nearest_idx + static_cast<int>(i);
            ROS_WARN("[%s] 路径末端未达最大距离，使用最后一个点（索引%d）", 
                     getName().c_str(), end_idx);
        }
    }
    end_idx = std::min(end_idx, static_cast<int>(input_path.points.size()) - 1);
    ROS_DEBUG("[%s] 截取路径段：索引%d → 索引%d（总距离%.2f m）",
             getName().c_str(), nearest_idx, end_idx, cum_dist[end_idx - nearest_idx]);

    return interpolate_path_segment(input_path, cum_dist, nearest_idx, end_idx, n_waypoints_, yaw0);
}

bool AntiRolloverController::solveNMPC(const std::vector<double>& current_state, const casadi::DM& waypoints,
                              std::vector<double>& control_output, double vx0) {
    if (current_state.size() != nx_) {
        ROS_ERROR("[%s] 状态向量维度不匹配: 期望%d, 实际%d", 
                 getName().c_str(), nx_, static_cast<int>(current_state.size()));
        return false;
    }

    try {
        // 设置参数
        opti_.set_value(x0_, current_state);
        opti_.set_value(waypoints_, waypoints);
        opti_.set_value(vx_, vx0);

        // 热启动
        if (has_prev_sol_) {
            opti_.set_initial(X_, sol_prev_->value(X_));
            opti_.set_initial(U_sparse_, sol_prev_->value(U_sparse_));
        }

        // 求解
        casadi::OptiSol sol_curr = opti_.solve();
        sol_prev_ = std::make_unique<casadi::OptiSol>(sol_curr);  // 智能指针创建实例（关键修改）
        has_prev_sol_ = true;

        // 提取第一个控制量
        casadi::DM u0 = sol_curr.value(U_sparse_(casadi::Slice(), 0));
        control_output[0] = static_cast<double>(u0(0));  // 前轴转向角

        // 提取松弛变量
        casadi::DM slack = sol_curr.value(epsilon_);
        double rollover_slack = static_cast<double>(slack(0));
        if (rollover_slack > 0.01) {
            ROS_WARN("[%s] 检测到 Rollover 风险，松弛变量值: %.4f", getName().c_str(), rollover_slack);
        } else {
            ROS_INFO("[%s] 暂无 Rollover 风险，松弛变量值: %.4f", getName().c_str(), rollover_slack);
        }
        return true;
    } catch (std::exception& e) {
        ROS_WARN("[%s] NMPC求解失败: %s", getName().c_str(), e.what());
        return false;
    }
}

std::vector<double> AntiRolloverController::vehicleStatusToStateVector(const race_msgs::VehicleStatus& status) {
    // 状态向量: [x, y, theta, gamma, ltr, dltr]
    std::vector<double> state(nx_, 0.0);
    
    state[0] = status.pose.position.x;               // x
    state[1] = status.pose.position.y;               // y
    state[2] = status.euler.yaw;                     // theta (偏航角)
    double gamma = status.euler.yaw - status.trailer.euler.yaw;
    if (gamma > M_PI){
        gamma -= 2 * M_PI;
    } else if (gamma < -M_PI){
        gamma += 2 * M_PI;
    }
    state[3] = gamma; // gamma (铰接角)
    state[4] = status.ltr_state.ltr;                // ltr
    state[5] = status.ltr_state.ltr_rate;           // dltr
    // 打印状态向量
    printf("状态向量: [x:%.4f, y:%.4f, theta:%.4f, gamma:%.4f, ltr:%.4f, dltr:%.4f]\n", 
           state[0], state[1], state[2], state[3], state[4], state[5]);
    
    return state;
}

double AntiRolloverController::vehicleStateToVx(const race_msgs::VehicleStatus& status) {
    return status.vel.linear.x;
}

} // namespace race_tracker

// 插件注册
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(race_tracker::AntiRolloverController, race_tracker::ControllerPluginBase)
    
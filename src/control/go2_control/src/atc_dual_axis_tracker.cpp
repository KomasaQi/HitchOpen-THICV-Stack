#include <casadi/casadi.hpp>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <numeric>
#include <limits>
#include <tf/transform_datatypes.h>  // 四元数转欧拉角
#include <race_msgs/Path.h>
#include <race_msgs/Control.h>
#include <race_msgs/VehicleStatus.h>
#include <race_msgs/PathPoint.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

using namespace casadi;
using namespace std;

// ====================== 辅助函数声明 ======================
/**
 * @brief 四元数转航向角（yaw角）
 * @param q 输入四元数
 * @return yaw角（弧度）
 */
double quaternion_to_yaw(const geometry_msgs::Quaternion& q);

/**
 * @brief 查找Path中距离当前位置最近的路点索引
 * @param x0 当前位置x坐标
 * @param y0 当前位置y坐标
 * @param path 输入的race_msgs::Path
 * @return 最近路点索引（-1表示Path为空）
 */
int find_nearest_path_point(const double x0, const double y0, const race_msgs::Path& path);

/**
 * @brief 计算从起始索引开始的Path点累积距离
 * @param path 输入的race_msgs::Path
 * @param start_idx 起始计算索引
 * @return 累积距离向量（第i个元素为start_idx+i点到start_idx点的总距离）
 */
vector<double> calculate_cumulative_distance(const race_msgs::Path& path, int start_idx);

/**
 * @brief 手动实现线性插值（替代CasADi::Interpolant，兼容旧版本）
 * @param s_original 原始自变量（累积距离）
 * @param val_original 原始因变量（如x/y/theta/v）
 * @param s_target 目标自变量（均匀分布的距离）
 * @return 插值后的因变量
 */
vector<double> linear_interpolate(const vector<double>& s_original, const vector<double>& val_original, 
                                 const vector<double>& s_target);

/**
 * @brief 对截取的Path段进行线性插值，生成指定数量的参考路点
 * @param path 原始输入Path
 * @param cum_dist 从起始点开始的累积距离向量
 * @param start_idx 截取段起始索引（原始Path中）
 * @param end_idx 截取段结束索引（原始Path中）
 * @param n_waypoints 目标插值路点数量
 * @return 插值后的参考路点（4行n_waypoints列：x,y,theta,velocity）
 */
DM interpolate_path_segment(const race_msgs::Path& path, const vector<double>& cum_dist, 
                           int start_idx, int end_idx, int n_waypoints);

/**
 * @brief 处理race_msgs::Path，生成NMPC所需的参考路点矩阵
 * @param input_path 输入的race_msgs::Path
 * @param current_state 当前车辆状态 [x,y,theta,vx,delta1,delta2]
 * @param n_waypoints 目标参考路点数量
 * @param a_max 预设最大加速度（m/s²）
 * @param N NMPC预测步长
 * @param dt NMPC采样时间（s）
 * @return 4行n_waypoints列的DM矩阵（x,y,theta,velocity）
 */
DM process_race_path(const race_msgs::Path& input_path, const vector<double>& current_state, 
                    int n_waypoints, double a_max, int N, double dt);

/**
 * @brief 构造示例race_msgs::Path（测试用，实际可从话题订阅）
 * @param num_original_points 原始Path点数量
 * @return 构造的示例Path（直线轨迹，x从0到60m，y=0，速度5到10m/s）
 */
race_msgs::Path create_example_race_path(int num_original_points);

// ====================== 辅助函数实现 ======================
double quaternion_to_yaw(const geometry_msgs::Quaternion& q) {
    tf::Quaternion tf_quat(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 rot_matrix(tf_quat);
    double roll, pitch, yaw;
    rot_matrix.getRPY(roll, pitch, yaw);  // Z-Y-X顺序，yaw为航向角
    return yaw;
}

int find_nearest_path_point(const double x0, const double y0, const race_msgs::Path& path) {
    if (path.points.empty()) {
        ROS_ERROR("[Path处理] 输入的race_msgs::Path为空！");
        return -1;
    }

    double min_dist_sq = numeric_limits<double>::max();
    int nearest_idx = 0;

    // 遍历所有Path点，计算欧氏距离平方（避免开根号，提高效率）
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

vector<double> calculate_cumulative_distance(const race_msgs::Path& path, int start_idx) {
    vector<double> cum_dist;
    if (start_idx < 0 || start_idx >= static_cast<int>(path.points.size())) {
        ROS_ERROR("[Path处理] 累积距离计算：起始索引无效（%d）！", start_idx);
        return cum_dist;
    }

    cum_dist.push_back(0.0);  // 起始点到自身距离为0
    double current_total = 0.0;

    // 从起始点开始，依次计算相邻点距离并累加
    for (int i = start_idx + 1; i < static_cast<int>(path.points.size()); ++i) {
        const auto& prev_pt = path.points[i-1].pose.position;
        const auto& curr_pt = path.points[i].pose.position;
        
        // 相邻点欧氏距离
        double dist = sqrt(pow(curr_pt.x - prev_pt.x, 2) + pow(curr_pt.y - prev_pt.y, 2));
        current_total += dist;
        cum_dist.push_back(current_total);
    }

    return cum_dist;
}

vector<double> linear_interpolate(const vector<double>& s_original, const vector<double>& val_original, 
                                 const vector<double>& s_target) {
    vector<double> val_target;
    val_target.reserve(s_target.size());

    // 处理原始数据长度不足的情况
    if (s_original.size() != val_original.size() || s_original.size() < 2) {
        ROS_WARN("[插值] 原始数据无效，返回常数插值！");
        double fill_val = (val_original.empty()) ? 0.0 : val_original[0];
        for (size_t i = 0; i < s_target.size(); ++i) {
            val_target.push_back(fill_val);
        }
        return val_target;
    }

    double s_min = s_original[0];
    double s_max = s_original.back();

    // 遍历所有目标插值点
    for (double s_t : s_target) {
        // 1. 边界处理：超出原始范围时直接取边界值
        if (s_t <= s_min) {
            val_target.push_back(val_original[0]);
            continue;
        }
        if (s_t >= s_max) {
            val_target.push_back(val_original.back());
            continue;
        }

        // 2. 查找s_t所在的区间（s_original[i] <= s_t < s_original[i+1]）
        size_t i = 0;
        while (i < s_original.size() - 1 && s_original[i+1] < s_t) {
            ++i;
        }

        // 3. 线性插值计算
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

DM interpolate_path_segment(const race_msgs::Path& path, const vector<double>& cum_dist, 
                           int start_idx, int end_idx, int n_waypoints) {
    // 提取截取段的原始数据（s:累积距离, x/y/theta/v:路点信息）
    vector<double> s_original, x_original, y_original, theta_original, v_original;
    for (int i = start_idx; i <= end_idx; ++i) {
        const auto& path_pt = path.points[i];
        int rel_idx = i - start_idx;  // 相对于起始点的索引
        
        s_original.push_back(cum_dist[rel_idx]);
        x_original.push_back(path_pt.pose.position.x);
        y_original.push_back(path_pt.pose.position.y);
        theta_original.push_back(quaternion_to_yaw(path_pt.pose.orientation));
        v_original.push_back(path_pt.velocity);
    }

    // 1. 创建目标插值的累积距离（均匀分布）
    vector<double> s_target(n_waypoints);
    if (s_original.empty()) {
        ROS_ERROR("[插值] 原始累积距离为空，返回零矩阵！");
        return DM::zeros(4, n_waypoints);
    }
    double s_max = s_original.back();
    for (int i = 0; i < n_waypoints; ++i) {
        s_target[i] = s_max * static_cast<double>(i) / (n_waypoints - 1);  // 0 ~ s_max 均匀分布
    }

    // 2. 对x/y/theta/v分别进行线性插值
    vector<double> x_interp = linear_interpolate(s_original, x_original, s_target);
    vector<double> y_interp = linear_interpolate(s_original, y_original, s_target);
    vector<double> theta_interp = linear_interpolate(s_original, theta_original, s_target);
    vector<double> v_interp = linear_interpolate(s_original, v_original, s_target);

    // 3. 转换为CasADi DM矩阵（4行n_waypoints列）
    DM waypoints = DM::zeros(4, n_waypoints);
    for (int i = 0; i < n_waypoints; ++i) {
        waypoints(0, i) = x_interp[i];
        waypoints(1, i) = y_interp[i];
        waypoints(2, i) = theta_interp[i];
        waypoints(3, i) = v_interp[i];
    }

    return waypoints;
}

DM process_race_path(const race_msgs::Path& input_path, const vector<double>& current_state, 
                    int n_waypoints, double a_max, int N, double dt) {
    // 1. 提取当前车辆状态（位置x0/y0，速度vx0）
    double x0 = current_state[0];
    double y0 = current_state[1];
    double vx0 = current_state[3];  // 当前纵向速度

    // 2. 查找最近路点
    int nearest_idx = find_nearest_path_point(x0, y0, input_path);
    if (nearest_idx == -1) {
        ROS_ERROR("[Path处理] 最近路点查找失败，返回零矩阵！");
        return DM::zeros(4, n_waypoints);
    }

    // 3. 计算最大跟踪距离（基于NMPC预测时域和最大加速度）
    double pred_time = N * dt;  // NMPC总预测时间
    double max_distance = vx0 * pred_time + 0.5 * a_max * pred_time * pred_time;  // 运动学距离公式
    ROS_INFO("[Path处理] 当前速度: %.2f m/s | 预测时间: %.2f s | 最大跟踪距离: %.2f m",
             vx0, pred_time, max_distance);

    // 4. 计算从最近点开始的累积距离
    vector<double> cum_dist = calculate_cumulative_distance(input_path, nearest_idx);
    if (cum_dist.empty()) {
        ROS_ERROR("[Path处理] 累积距离计算失败，返回零矩阵！");
        return DM::zeros(4, n_waypoints);
    }

    // 5. 确定截取段结束索引（第一个超过最大距离的点）
    int end_idx = nearest_idx;
    for (size_t i = 0; i < cum_dist.size(); ++i) {
        if (cum_dist[i] > max_distance) {
            end_idx = nearest_idx + static_cast<int>(i);
            break;
        }
        // 遍历到末尾仍未超过，使用最后一个点
        if (i == cum_dist.size() - 1) {
            end_idx = nearest_idx + static_cast<int>(i);
            ROS_WARN("[Path处理] 路径末端未达最大距离，使用最后一个点（索引%d）", end_idx);
        }
    }
    // 确保结束索引不超出Path范围
    end_idx = min(end_idx, static_cast<int>(input_path.points.size()) - 1);
    ROS_INFO("[Path处理] 截取路径段：索引%d → 索引%d（总距离%.2f m）",
             nearest_idx, end_idx, cum_dist[end_idx - nearest_idx]);

    // 6. 插值生成目标数量的参考路点
    DM waypoints_dm = interpolate_path_segment(input_path, cum_dist, nearest_idx, end_idx, n_waypoints);
    return waypoints_dm;
}

race_msgs::Path create_example_race_path(int num_original_points) {
    race_msgs::Path example_path;
    example_path.header.frame_id = "map";
    example_path.header.stamp = ros::Time::now();

    // 生成直线轨迹示例（x:0→60m, y:0, 速度:5→10m/s）
    double x_start = 0.0;
    double x_end = 60.0;
    double y_fixed = 0.0;
    double v_start = 5.0;
    double v_end = 10.0;

    for (int i = 0; i < num_original_points; ++i) {
        race_msgs::PathPoint pt;
        
        // 位置设置（线性分布）
        double ratio = static_cast<double>(i) / (num_original_points - 1);
        pt.pose.position.x = x_start + ratio * (x_end - x_start);
        pt.pose.position.y = y_fixed;
        pt.pose.position.z = 0.0;

        // 姿态设置（单位四元数，yaw=0，即朝向x轴正方向）
        pt.pose.orientation.x = 0.0;
        pt.pose.orientation.y = 0.0;
        pt.pose.orientation.z = 0.0;
        pt.pose.orientation.w = 1.0;

        // 速度设置（线性递增）
        pt.velocity = v_start + ratio * (v_end - v_start);

        // 曲率设置（直线曲率为0）
        pt.curvature = 0.0;

        example_path.points.push_back(pt);
    }

    ROS_INFO("[测试] 构造示例Path：%d个原始点（直线0→60m，速度5→10m/s）", num_original_points);
    return example_path;
}

// ====================== 主函数 ======================
int main(int argc, char * argv[]) {
    // 设置中文编码
    setlocale(LC_ALL, "");
    // 初始化ROS节点
    ros::init(argc, argv, "atc_dual_axis_tracker_node");    
    ros::NodeHandle nh;

    ROS_INFO("车辆轨迹跟踪NMPC求解器节点已启动（稀疏控制量版本，基于race_msgs::Path）");

    // ====== 1. 系统核心参数 ======
    int nx = 6;                // 状态维度 [x, y, theta, vx, delta1, delta2]
    int nu = 3;                // 控制量维度 [ax_des, delta1_des, delta2_des]
    int N = 20;                // NMPC预测步长
    int Nc = 4;                // 稀疏控制量步数
    double T_d1 = 0.3;         // 前轴转向动态时间常数
    double T_d2 = 0.3;         // 后轴转向动态时间常数
    double dt = 0.1;           // 采样时间 (s)
    double L = 2.8;            // 车辆轴距 (m)
    double g = 9.806;          // 重力加速度 (m/s²)
    double a_max = 6.0;        // 预设最大加速度 (m/s²，用户指定)
    int n_waypoints = 30;      // 目标参考路点数量
    int num_original_path_pt = 150;  // 示例Path的原始点数量

    // 稀疏控制量步数分布（前Nc-1步为1,2,...Nc-1，剩余步数给最后一个控制量）
    vector<int> steps_per_control(Nc);
    int remaining_steps = N;
    for(int i = 0; i < Nc; ++i) {
        if(i < Nc - 1) {
            steps_per_control[i] = i + 1;
            remaining_steps -= steps_per_control[i];
        } else {
            steps_per_control[i] = remaining_steps;
        }
    }

    // 打印控制量分布
    ROS_INFO("稀疏控制量分布:");
    int total_steps = 0;
    for(int i = 0; i < Nc; ++i) {
        total_steps += steps_per_control[i];
        ROS_INFO("控制量 u%d 作用 %d 个时间步", i, steps_per_control[i]);
    }
    ROS_INFO("总预测步长: %d (应等于 N=%d)", total_steps, N);

    // ====== 2. 定义符号变量与动力学模型 ======
    MX X_sym = MX::sym("X", nx);  // 状态变量 [x,y,theta,vx,delta1,delta2]
    MX U_sym = MX::sym("U", nu);  // 控制变量 [ax_des, delta1_des, delta2_des]
    
    // 计算侧向速度vy（双轴转向车辆简化模型）
    MX vy = X_sym(3) * (tan(X_sym(4)) + tan(X_sym(5))) / 2;

    // 动力学方程（连续时间）
    MX f_expr = MX::vertcat({
        X_sym(3) * cos(X_sym(2)) - vy * sin(X_sym(2)),  // x_dot: 纵向位移率
        X_sym(3) * sin(X_sym(2)) + vy * cos(X_sym(2)),  // y_dot: 侧向位移率
        X_sym(3) * (tan(X_sym(4)) - tan(X_sym(5))) / L,  // theta_dot: 航向角速率
        // 纵向加速度模型（驱动/制动效率差异）
        if_else(U_sym(0) > 0, (22/(X_sym(3)+12)-0.8)*U_sym(0), U_sym(0)*0.8*g),
        (U_sym(1) - X_sym(4)) / T_d1,  // delta1_dot: 前轴转向角速率
        (U_sym(2) - X_sym(5)) / T_d2   // delta2_dot: 后轴转向角速率
    });
    Function f_func("f_dynamics", {X_sym, U_sym}, {f_expr});

    // ====== 3. RK4离散化（数值积分）======
    auto rk4_integrate = [&](MX x, MX u, double dt) {
        // 显式构造std::vector<MX>参数，避免Function调用歧义
        std::vector<MX> args_k1 = {x, u};
        MX k1 = f_func(args_k1)[0];

        std::vector<MX> args_k2 = {x + dt/2*k1, u};
        MX k2 = f_func(args_k2)[0];

        std::vector<MX> args_k3 = {x + dt/2*k2, u};
        MX k3 = f_func(args_k3)[0];

        std::vector<MX> args_k4 = {x + dt*k3, u};
        MX k4 = f_func(args_k4)[0];

        return x + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
    };

    // ====== 4. 构建NMPC优化问题 ======
    Opti opti;
    MX X = opti.variable(nx, N+1);      // 状态序列（N+1个点：0→N）
    MX U_sparse = opti.variable(nu, Nc); // 稀疏控制量（Nc个控制量）
    MX x0 = opti.parameter(nx);         // 初始状态参数
    MX waypoints = opti.parameter(4, n_waypoints); // 参考路点参数（x,y,theta,v）

    // 4.1 生成完整控制序列（稀疏控制量扩展到N步）
    MX U_full = MX::zeros(nu, N);
    int time_step = 0;
    for(int i = 0; i < Nc; ++i) {
        int end_step = time_step + steps_per_control[i];
        end_step = min(end_step, N);  // 防止超出范围
        // 将第i个稀疏控制量复制到对应时间步
        U_full(Slice(), Slice(time_step, end_step)) = repmat(U_sparse(Slice(), i), 1, end_step - time_step);
        time_step = end_step;
        if(time_step >= N) break;
    }

    // 4.2 动力学约束（状态递推）
    for(int k = 0; k < N; ++k) {
        opti.subject_to(X(Slice(), k+1) == rk4_integrate(X(Slice(), k), U_full(Slice(), k), dt));
    }

    // 4.3 初始状态约束
    opti.subject_to(X(Slice(), 0) == x0);

    // 4.4 控制量边界约束
    opti.subject_to(opti.bounded(-1.0, U_sparse(0, Slice()), 1.0));    // 加速度 [-1,1] m/s²
    opti.subject_to(opti.bounded(-0.4, U_sparse(1, Slice()), 0.4));   // 前轴转向角 [-0.4,0.4] rad（≈23°）
    opti.subject_to(opti.bounded(-0.4, U_sparse(2, Slice()), 0.4));   // 后轴转向角 [-0.4,0.4] rad（≈23°）

    // ====== 5. 代价函数（状态跟踪+控制平滑） ======
    MX cost = 0;

    // 5.1 预测步代价（每个时间步的跟踪误差）
    for(int k = 0; k < N; ++k) {
        // 当前状态位置
        MX curr_pos = X(Slice(0,2), k);  // (x,y)
        
        // 计算到所有参考路点的距离平方，找到最近路点
        MX pos_diff = curr_pos - waypoints(Slice(0,2), Slice());  // 2×n_waypoints
        MX dist_sq = sum1(pos_diff * pos_diff);                  // 1×n_waypoints（每个路点的距离平方）
        MX min_idx = 0;
        for(int i = 1; i < n_waypoints; ++i) {
            min_idx = if_else(dist_sq(i) < dist_sq(min_idx), i, min_idx);  // 手动查找最近点索引
        }

        // 提取最近路点的参考信息
        MX ref_theta = waypoints(2, min_idx);  // 参考航向角
        MX ref_v = waypoints(3, min_idx);      // 参考速度

        // 状态跟踪代价（权重可调整）
        cost += 5.0 * dist_sq(min_idx);               // 位置跟踪权重
        cost += 3.0 * sumsqr(X(2, k) - ref_theta);    // 航向跟踪权重
        cost += 5.0 * sumsqr(X(3, k) - ref_v);        // 速度跟踪权重

        // 控制量平滑代价
        cost += 5.0 * sumsqr(U_full(0, k));           // 加速度平滑
        cost += 0.1 * sumsqr(X(4, k));                // 前轴转向角平滑
        cost += 0.2 * sumsqr(X(5, k));                // 后轴转向角平滑
    }

    // 5.2 终端代价（增强末端跟踪精度）
    MX final_pos = X(Slice(0,2), N);
    MX final_pos_diff = final_pos - waypoints(Slice(0,2), Slice());
    MX final_dist_sq = sum1(final_pos_diff * final_pos_diff);
    MX final_min_idx = 0;
    for(int i = 1; i < n_waypoints; ++i) {
        final_min_idx = if_else(final_dist_sq(i) < final_dist_sq(final_min_idx), i, final_min_idx);
    }
    MX final_ref_theta = waypoints(2, final_min_idx);
    MX final_ref_v = waypoints(3, final_min_idx);

    cost += 10.0 * final_dist_sq(final_min_idx);          // 终端位置权重
    cost += 5.0 * sumsqr(X(2, N) - final_ref_theta);      // 终端航向权重
    cost += 5.0 * sumsqr(X(3, N) - final_ref_v);          // 终端速度权重

    // 最小化总代价
    opti.minimize(cost);

    // ====== 6. 求解器配置（IPOPT） ======
    Dict solver_opts;
    solver_opts["ipopt.print_level"] = 0;                  // 关闭详细打印
    solver_opts["print_time"] = true;                      // 打印求解时间
    solver_opts["ipopt.sb"] = "yes";                       // 抑制冗余输出
    solver_opts["ipopt.max_iter"] = 500;                   // 最大迭代次数
    solver_opts["ipopt.tol"] = 1e-2;                       // 主精度阈值
    solver_opts["ipopt.acceptable_tol"] = 5e-2;            // 可接受精度阈值
    solver_opts["ipopt.acceptable_iter"] = 10;             // 可接受迭代次数（快速返回次优解）
    solver_opts["ipopt.hessian_approximation"] = "limited-memory";  // 有限内存Hessian（减少计算量）
    solver_opts["ipopt.linear_solver"] = "ma27";           // 线性求解器（适合中小型问题）

    opti.solver("ipopt", solver_opts);

    // ====== 7. 处理race_msgs::Path（核心功能） ======
    // 7.1 初始状态（[x,y,theta,vx,delta1,delta2]）
    vector<double> x0_val = {14.4, 2.0, 0.0, 9.0, 0.0, 0.0};  // 初始位置在Path中间段附近
    ROS_INFO("初始车辆状态：x=%.2f, y=%.2f, theta=%.2f rad, vx=%.2f m/s",
             x0_val[0], x0_val[1], x0_val[2], x0_val[3]);

    // 7.2 构造示例race_msgs::Path（实际使用时可替换为话题订阅）
    race_msgs::Path input_path = create_example_race_path(num_original_path_pt);

    // 7.3 处理Path生成参考路点矩阵
    DM waypoints_dm = process_race_path(input_path, x0_val, n_waypoints, a_max, N, dt);

    // ====== 8. NMPC求解与测试 ======
    // 8.1 设置初始参数
    opti.set_value(x0, x0_val);
    opti.set_value(waypoints, waypoints_dm);

    // 8.2 首次求解（无热启动）：OptiSol无默认构造，首次求解后赋值



    OptiSol sol_prev = opti.solve();
    bool has_prev_sol = true;
    ROS_INFO("首次求解成功！");


    // 8.3 多次求解测试（带热启动，模拟实时控制）
    int test_iterations = 50;  // 测试迭代次数
    double start_time = ros::Time::now().toSec();

    for (int i = 0; i < test_iterations; ++i) {
        // 给初始状态添加小扰动（模拟车辆状态变化）
        vector<double> x0_val_rand = x0_val;
        x0_val_rand[0] += (rand() % 100 - 50) * 0.05;  // x扰动 ±2.5m
        x0_val_rand[1] += (rand() % 100 - 50) * 0.01;  // y扰动 ±0.5m
        x0_val_rand[2] += (rand() % 100 - 50) * 0.01;  // theta扰动 ±0.5rad
        x0_val_rand[3] += (rand() % 100 - 50) * 0.01;  // vx扰动 ±0.5m/s

        // 更新初始状态参数
        DM waypoints_dm = process_race_path(input_path, x0_val, n_waypoints, a_max, N, dt);
        opti.set_value(waypoints, waypoints_dm);
        opti.set_value(x0, x0_val_rand);

        // 热启动（使用前一次求解结果作为初始猜测，加速收敛）
        if (has_prev_sol) {
            opti.set_initial(X, sol_prev.value(X));
            opti.set_initial(U_sparse, sol_prev.value(U_sparse));
        }

        // 求解
        try {
            OptiSol sol_curr = opti.solve();
            sol_prev = sol_curr;  // 更新前次解
            ROS_DEBUG("第%d次求解成功", i+1);
        } catch (exception& e) {
            ROS_WARN("第%d次求解失败：%s", i+1, e.what());
            continue;
        }
    }

    // 计算平均求解时间
    double avg_solve_time = (ros::Time::now().toSec() - start_time) * 1000 / test_iterations;
    ROS_INFO("========================================");
    ROS_INFO("测试完成！共迭代%d次，平均求解时间：%.2f ms", test_iterations, avg_solve_time);

    // ====== 9. 打印最终结果 ======
    OptiSol final_sol = sol_prev;
    ROS_INFO("========================================");
    ROS_INFO("最终优化结果：");
    ROS_INFO("稀疏控制量数量：%d（原始需%d个）", Nc, N);
    ROS_INFO("第一个控制输入：加速度=%.3f m/s²，前轴转角=%.3f rad，后轴转角=%.3f rad",
             static_cast<double>(final_sol.value(U_sparse(0,0))),
             static_cast<double>(final_sol.value(U_sparse(1,0))),
             static_cast<double>(final_sol.value(U_sparse(2,0))));

    // 打印稀疏控制量分布
    ROS_INFO("稀疏控制量作用范围：");
    time_step = 0;















    
    for(int i = 0; i < Nc; ++i) {
        int end_step = time_step + steps_per_control[i];
        end_step = min(end_step, N);
        DM u = final_sol.value(U_sparse(Slice(), i));
        ROS_INFO("u%d：加速度=%.3f，前轴转角=%.3f，后轴转角=%.3f → 作用步：%d~%d",
                 i, static_cast<double>(u(0)), static_cast<double>(u(1)), static_cast<double>(u(2)),
                 time_step, end_step - 1);
        time_step = end_step;
    }

    // 打印前10个预测步的状态（简化输出）
    ROS_INFO("前10个预测步状态（x,y,theta,vx,delta1,delta2）：");
    for(int k = 0; k < min(10, N+1); ++k) {
        DM xk = final_sol.value(X(Slice(), k));
        ROS_INFO("步%d：(%.2f, %.2f, %.2f, %.2f, %.2f, %.2f)",
                 k, static_cast<double>(xk(0)), static_cast<double>(xk(1)),
                 static_cast<double>(xk(2)), static_cast<double>(xk(3)),
                 static_cast<double>(xk(4)), static_cast<double>(xk(5)));
    }

    // ====== 10. 节点关闭 ======
    ros::shutdown();
    return 0;
}
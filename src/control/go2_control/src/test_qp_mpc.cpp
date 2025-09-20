#include <casadi/casadi.hpp>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <cmath>  // 用于atan2计算航向

// 车辆参数
struct VehicleParams {
    double L = 2.5;        // 轴距 (m)
    double dt = 0.1;       // 时间步长 (s)
    double v_ref = 10.0;   // 参考速度 (m/s)
};

// MPC 参数
struct MPCParams {
    int N = 15;             // 预测horizon
    double Q_xy = 50.0;    // 位置跟踪权重（增大，强化位置跟踪）
    double Q_psi = 20.0;   // 航向权重（新增，跟踪参考航向）
    double Q_v = 2.0;      // 速度权重
    double R_delta = 1.0;  // 前轮转角权重
    double R_a = 1.0;      // 加速度权重
    double R_dDelta = 5.0; // 转角变化率权重（新增，避免转角突变）
};

int main(int argc, char *argv[]) {
    // 设置中文编码
    setlocale(LC_ALL, "");
    // 初始化ROS节点
    ros::init(argc, argv, "vehicle_mpc_node");
    ros::NodeHandle nh;
    
    ROS_INFO("车辆MPC节点已启动");

    // 参数初始化
    VehicleParams vehicle_params;
    MPCParams mpc_params;
    
    // 当前状态 [x, y, psi, v] 和参考点
    std::vector<double> current_state = {0.0, 0.0, 0.0, 8.0};  // x,y,航向,速度
    std::vector<double> target_point = {20.0, 15.0};           // 目标点
    double psi_ref = atan2(target_point[1]-current_state[1], target_point[0]-current_state[0]); // 参考航向（指向目标点）
    
    using namespace casadi;
    
    // 创建符号变量
    SX state = SX::sym("state", 4 * mpc_params.N);  // 状态: [x0,y0,psi0,v0, x1,y1,psi1,v1, ...]
    SX control = SX::sym("control", 2 * mpc_params.N);  // 控制: [delta0,a0, delta1,a1, ...]
    SX param = SX::sym("param", 6);  // 参数: [x0,y0,psi0,v0, x_target,y_target]
    
    // 目标函数
    SX cost = 0;
    
    // 约束条件
    std::vector<SX> constraints;
    
    // 提取参数（当前状态和目标点）
    SX x0 = param(0);
    SX y0 = param(1);
    SX psi0 = param(2);
    SX v0 = param(3);
    SX x_target = param(4);
    SX y_target = param(5);
    
    // 生成参考轨迹（线性插值到目标点，包含参考航向）
    std::vector<SX> x_ref(mpc_params.N), y_ref(mpc_params.N), psi_ref_traj(mpc_params.N);
    for (int k = 0; k < mpc_params.N; ++k) {
        double t = static_cast<double>(k) / (mpc_params.N - 1);  // 注意：用N-1确保最后一步到达目标点
        x_ref[k] = x0 + t * (x_target - x0);
        y_ref[k] = y0 + t * (y_target - y0);
        psi_ref_traj[k] = psi_ref;  // 参考航向固定为指向目标点
    }
    
    // 遍历预测时域，构建模型和约束
    for (int k = 0; k < mpc_params.N; ++k) {
        // 当前预测状态（第k步）
        int state_idx = k * 4;
        SX x_k = state(state_idx + 0);
        SX y_k = state(state_idx + 1);
        SX psi_k = state(state_idx + 2);
        SX v_k = state(state_idx + 3);
        
        // 当前控制输入（第k步，控制量比状态少1步？不：N步状态对应N步控制，最后一步控制作用到N+1步，这里简化为N步控制）
        int ctrl_idx = k * 2;
        SX delta_k = control(ctrl_idx + 0);  // 前轮转角
        SX a_k = control(ctrl_idx + 1);      // 加速度
        
        // 1. 线性化自行车模型（修正：小角度近似，但保留正确的导数关系）
        SX x_dot = v_k * cos(psi_k);  // 恢复cos(psi)，避免x方向速度过大
        SX y_dot = v_k * sin(psi_k);  // 恢复sin(psi)，小角度下≈v*psi，但支持大角度
        SX psi_dot = (v_k / vehicle_params.L) * tan(delta_k);  // 恢复tan(delta)，小角度下≈delta
        SX v_dot = a_k;
        
        // 2. 状态递推约束（修正：下一状态 = 当前状态 + dt*导数）
        if (k == 0) {
            // 第0步状态：基于初始状态递推（约束初始预测状态=初始状态+第一步控制的作用）
            SX x1 = x0 + vehicle_params.dt * x_dot;
            SX y1 = y0 + vehicle_params.dt * y_dot;
            SX psi1 = psi0 + vehicle_params.dt * psi_dot;
            SX v1 = v0 + vehicle_params.dt * v_dot;
            // 约束：第0步预测状态 = 初始状态递推结果（确保初始状态正确）
            constraints.push_back(x_k - x1);
            constraints.push_back(y_k - y1);
            constraints.push_back(psi_k - psi1);
            constraints.push_back(v_k - v1);
        } else {
            // 第k步状态：基于第k-1步状态递推
            int prev_state_idx = (k-1) * 4;
            SX x_prev = state(prev_state_idx + 0);
            SX y_prev = state(prev_state_idx + 1);
            SX psi_prev = state(prev_state_idx + 2);
            SX v_prev = state(prev_state_idx + 3);
            // 递推下一状态（第k步状态 = 第k-1步状态 + dt*第k-1步的导数）
            SX x_next = x_prev + vehicle_params.dt * (v_prev * cos(psi_prev));
            SX y_next = y_prev + vehicle_params.dt * (v_prev * sin(psi_prev));
            SX psi_next = psi_prev + vehicle_params.dt * (v_prev / vehicle_params.L * tan(delta_k));
            SX v_next = v_prev + vehicle_params.dt * a_k;
            // 约束：第k步状态 = 递推结果
            constraints.push_back(x_k - x_next);
            constraints.push_back(y_k - y_next);
            constraints.push_back(psi_k - psi_next);
            constraints.push_back(v_k - v_next);
        }
        
        // 3. 目标函数（新增参考航向跟踪，调整权重）
        cost += mpc_params.Q_xy * ((x_k - x_ref[k])*(x_k - x_ref[k]) + (y_k - y_ref[k])*(y_k - y_ref[k]));  // 位置误差
        cost += mpc_params.Q_psi * (psi_k - psi_ref_traj[k])*(psi_k - psi_ref_traj[k]);  // 航向误差（关键新增）
        cost += mpc_params.Q_v * (v_k - vehicle_params.v_ref)*(v_k - vehicle_params.v_ref);  // 速度误差
        cost += mpc_params.R_delta * delta_k * delta_k;  // 转角控制代价
        
        // 4. 控制量变化率惩罚（避免转角突变，新增）
        if (k > 0) {
            SX delta_prev = control((k-1)*2 + 0);
            cost += mpc_params.R_dDelta * (delta_k - delta_prev)*(delta_k - delta_prev);
        }
        
        // 5. 控制量边界约束（即使去掉硬约束，也建议加软惩罚，这里先保留硬约束确保合理性）
        constraints.push_back(delta_k + 0.5);  // delta >= -0.5 rad（≈-28.6°）
        constraints.push_back(0.5 - delta_k);   // delta <= 0.5 rad
        constraints.push_back(a_k + 3.0);       // a >= -3 m/s²（刹车）
        constraints.push_back(3.0 - a_k);       // a <= 3 m/s²（加速）
    }
    
    // 创建QP问题
    SXDict qp;
    qp["x"] = SX::vertcat({state, control});  // 决策变量：状态 + 控制
    qp["f"] = cost;                           // 目标函数
    qp["g"] = SX::vertcat(constraints);       // 约束条件
    qp["p"] = param;                          // 参数（当前状态+目标点）
    
    // 创建QP求解器（qpOASES，增加收敛性参数）
    // 创建QP求解器
    Dict options;
    options["print_time"] = false;
    options["error_on_fail"] = false;  // 避免因不可行而崩溃
    Function solver = qpsol("solver", "qpoases", qp, options);
    
    // 设置约束界（根据constraints的顺序）
    int n_dynamic_constraints = 4 * mpc_params.N;  // 状态递推约束（每步4个）
    int n_control_constraints = 4 * mpc_params.N;  // 控制量边界约束（每步4个）
    int n_total_constraints = n_dynamic_constraints + n_control_constraints;
    
    std::vector<double> lbg(n_total_constraints, -1e9);
    std::vector<double> ubg(n_total_constraints, 1e9);
    
    // 状态递推约束：等于0（硬约束）
    for (int i = 0; i < n_dynamic_constraints; ++i) {
        lbg[i] = 0.0;
        ubg[i] = 0.0;
    }
    
    // 控制量边界约束：等于0（因为约束是delta+0.5 >=0 → lbg=0, ubg=inf）
    for (int i = n_dynamic_constraints; i < n_total_constraints; ++i) {
        lbg[i] = 0.0;
        ubg[i] = 1e9;
    }
    
    // 设置变量界（状态和控制的合理范围，避免优化器发散）
    std::vector<double> lbx(4*mpc_params.N + 2*mpc_params.N, -1e9);
    std::vector<double> ubx(4*mpc_params.N + 2*mpc_params.N, 1e9);
    // 状态界（可选，确保物理合理性）
    for (int k = 0; k < mpc_params.N; ++k) {
        lbx[k*4 + 3] = 0.0;    // 速度v >= 0（避免倒车）
        ubx[k*4 + 3] = 20.0;   // 速度v <= 20 m/s
        lbx[k*4 + 2] = -M_PI;  // 航向psi >= -π
        ubx[k*4 + 2] = M_PI;   // 航向psi <= π
    }
    // 控制量界（和硬约束一致，双重保障）
    for (int k = 0; k < mpc_params.N; ++k) {
        lbx[4*mpc_params.N + k*2 + 0] = -0.5;  // delta >= -0.5
        ubx[4*mpc_params.N + k*2 + 0] = 0.5;   // delta <= 0.5
        lbx[4*mpc_params.N + k*2 + 1] = -3.0;  // a >= -3
        ubx[4*mpc_params.N + k*2 + 1] = 3.0;   // a <= 3
    }
    
    // 准备求解参数（当前状态+目标点）
    std::vector<double> p_vec(6);
    p_vec[0] = current_state[0];  // x0
    p_vec[1] = current_state[1];  // y0
    p_vec[2] = current_state[2];  // psi0
    p_vec[3] = current_state[3];  // v0
    p_vec[4] = target_point[0];   // x_target
    p_vec[5] = target_point[1];   // y_target
    
    // 求解问题（用DMDict确保类型匹配）
    DMDict arg;
    arg["lbx"] = lbx;
    arg["ubx"] = ubx;
    arg["lbg"] = lbg;
    arg["ubg"] = ubg;
    arg["p"] = p_vec;
    // 可选：暖启动（用初始值引导优化器，避免delta=0）
    arg["x0"] = DM::zeros(4*mpc_params.N + 2*mpc_params.N);  // 初始值设为0，也可设小的非零值
    
    DMDict res = solver(arg);
    
    // 提取最优解
    DM solution = res.at("x");
    std::vector<double> state_opt = solution(Slice(0, 4*mpc_params.N)).get_elements();
    std::vector<double> control_opt = solution(Slice(4*mpc_params.N, solution.size1())).get_elements();
    
    // 输出结果（增加y方向和航向的信息）
    std::cout << "\n=== 车辆MPC求解结果 ===" << std::endl;
    std::cout << "当前状态: x=" << current_state[0] << ", y=" << current_state[1] 
              << ", ψ=" << current_state[2]*180/M_PI << "°, v=" << current_state[3] << " m/s" << std::endl;
    std::cout << "目标点: (" << target_point[0] << ", " << target_point[1] << "), 参考航向: " << psi_ref*180/M_PI << "°" << std::endl;
    std::cout << "第一个控制量: δ=" << control_opt[0] << " rad (" << control_opt[0]*180/M_PI << "°), a=" << control_opt[1] << " m/s²" << std::endl;
    std::cout << "预测轨迹 (前10步):" << std::endl;
    for (int k = 0; k < std::min(10, mpc_params.N); ++k) {
        std::cout << "  步" << k << ": x=" << std::fixed << std::setprecision(2) << state_opt[k*4] 
                  << ", y=" << state_opt[k*4+1] 
                  << ", ψ=" << state_opt[k*4+2]*180/M_PI << "°" 
                  << ", v=" << state_opt[k*4+3] << " m/s" << std::endl;
    }
    std::cout << "最优代价: " << std::fixed << std::setprecision(2) << res.at("f") << std::endl;
    
    ros::shutdown();
    return 0;
}
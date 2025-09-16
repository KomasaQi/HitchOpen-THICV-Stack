#include <casadi/casadi.hpp>
#include <iostream>
#include <ros/ros.h>
#include <vector>

using namespace casadi;

// 线性插值函数 (Casadi符号版本，不变)
MX interp1(const std::vector<double>& x, const std::vector<double>& y, const MX& xi) {
    MX yi = 0;
    int n = x.size();
    
    // 边界处理
    MX cond_low = xi <= x[0];
    MX cond_high = xi >= x.back();
    yi = if_else(cond_low, y[0], yi);
    yi = if_else(cond_high, y.back(), yi);
    
    // 中间插值
    for(int i = 0; i < n-1; ++i) {
        MX cond = (xi >= x[i]) && (xi <= x[i+1]);
        double dx = x[i+1] - x[i];
        double dy = y[i+1] - y[i];
        MX val = y[i] + (xi - x[i]) * dy / dx;
        yi = if_else(cond, val, yi);
    }
    
    return yi;
}

int main(int argc, char * argv[]) {
    // 设置中文编码
    setlocale(LC_ALL, "");
    // 初始化ROS节点
    ros::init(argc, argv, "vehicle_tracking_node");
    ros::NodeHandle nh;
    
    ROS_INFO("车辆轨迹跟踪NMPC求解器节点已启动（稀疏控制量版本，基于路径距离，优化版）");

    // ====== 1. 系统参数 ======
    int nx = 6;       // 状态维度 [x, y, theta, vx, delta1, delta2]
    int nu = 3;       // 控制量维度 [ax_des, delta1_des, delta2_des]
    int N = 20;       // 总预测步长
    int Nc = 4;       // 控制步数
    double T_d1 = 0.3; // 前轴动态时间常数
    double T_d2 = 0.3; // 后轴动态时间常数
    double V_d_max = M_PI / 4; // 最大转向速度
    double dt = 0.1; // 采样时间 (s)
    double L = 2.8;   // 车辆轴距 (m)
    double g = 9.806; // 重力加速度 (m/s^2)
    std::vector<double> drive_acc; // 驱动加速度曲线
    std::vector<double> drive_dcc; // 制动减速度曲线
    std::vector<double> drive_vel; // 速度节点


    int n_waypoints = 30; // 参考轨迹的路点数量

    // 控制量步数分布
    std::vector<int> steps_per_control(Nc);
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
    ROS_INFO("控制量分布:");
    int total = 0;
    for(int i = 0; i < Nc; ++i) {
        total += steps_per_control[i];
        ROS_INFO("控制量 u%d 作用 %d 个时间步", i, steps_per_control[i]);
    }
    ROS_INFO("总时间步数: %d (应等于 N=%d)", total, N);

    // ====== 2. 定义符号变量 ======
    MX X_sym = MX::sym("X", nx);  // 状态变量 [x, y, theta, vx, vy, delta1]
    MX U_sym = MX::sym("U", nu);  // 控制变量 [a, delta1_des]
    MX v_y = X_sym(3) * (tan(X_sym(4)) + tan(X_sym(5))) / 2;

    // 动力学函数
    MX f_expr = MX::vertcat({
        X_sym(3) * cos(X_sym(2)) - v_y * sin(X_sym(2)),
        X_sym(3) * sin(X_sym(2)) + v_y * cos(X_sym(2)),
        X_sym(3) * (tan(X_sym(4)) - tan(X_sym(5))) / L,
        if_else(U_sym(0)>0, (22/(X_sym(3)+12)-0.8)*U_sym(0),U_sym(0)*0.8*g),
        ((U_sym(1) - X_sym(4)) / T_d1),
        ((U_sym(2) - X_sym(5)) / T_d2)
    });
    Function f_func("f", {X_sym, U_sym}, {f_expr});

    // ====== 3. RK4 离散化 ======
    auto rk4 = [&](MX x, MX u, double dt) {
        MX k1 = f_func(std::vector<MX>{x, u})[0];
        MX k2 = f_func(std::vector<MX>{x + dt/2*k1, u})[0];
        MX k3 = f_func(std::vector<MX>{x + dt/2*k2, u})[0];
        MX k4 = f_func(std::vector<MX>{x + dt*k3, u})[0];
        return x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    };

    // ====== 4. 构建 NMPC 优化器 ======
    Opti opti;
    MX X = opti.variable(nx, N+1);      // 状态序列
    MX U_sparse = opti.variable(nu, Nc); // 稀疏控制量
    MX x0 = opti.parameter(nx);         // 初始状态
    MX waypoints = opti.parameter(4, n_waypoints); // 参考路点 [x, y, theta, v]

    // 构建完整控制序列
    MX U_full = MX::zeros(nu, N);
    int time_step = 0;
    for(int i = 0; i < Nc; ++i) {
        int end_step = time_step + steps_per_control[i];
        if(end_step > N) end_step = N;
        U_full(Slice(), Slice(time_step, end_step)) = repmat(U_sparse(Slice(), i), 1, end_step - time_step);
        time_step = end_step;
        if(time_step >= N) break;
    }

    // 动力学约束
    for(int k = 0; k < N; ++k) {
        opti.subject_to(X(Slice(), k+1) == rk4(X(Slice(), k), U_full(Slice(), k), dt));
    }

    // 初始条件
    opti.subject_to(X(Slice(), 0) == x0);

    // 控制约束
    opti.subject_to(opti.bounded(-1.0, U_sparse(0, Slice()), 1.0));    // 加速度约束
    opti.subject_to(opti.bounded(-0.4, U_sparse(1, Slice()), 0.4)); // 转向角约束（≈30°）
    opti.subject_to(opti.bounded(-0.4, U_sparse(2, Slice()), 0.4)); // 转向角约束（≈30°）
    
    // ====== 5. 优化后的代价函数（修复 argmin 问题） ======
    MX cost = 0;
    // 预测步代价：向量化计算最近路点
    for(int k = 0; k < N; ++k) {
        MX pos = X(Slice(0,2), k); // 当前状态位置 (x,y)
        // 计算到所有路点的距离平方
        MX pos_diff = pos - waypoints(Slice(0,2), Slice()); // 2xN_waypoints
        MX dist_sqr = sum1(pos_diff * pos_diff); // 1xN_waypoints
        MX min_dist_sqr = mmin(dist_sqr); // 最小距离平方

        // 手动实现 argmin：找到最小距离的索引
        MX min_idx = 0;
        for(int i = 1; i < n_waypoints; ++i) {
            min_idx = if_else(dist_sqr(i) < dist_sqr(min_idx), i, min_idx);
        }

        MX v_ref = waypoints(3, min_idx); // 参考速度
        MX theta_ref = waypoints(2, min_idx); // 参考航向
        // 状态跟踪代价（调整权重）
        cost += 5.0 * min_dist_sqr; // 位置跟踪
        cost += 3.0 * sumsqr(X(2, k) - theta_ref); // 航向跟踪
        cost += 5.0 * sumsqr(X(3, k) - v_ref); // 速度跟踪
        // 控制量正则化
        cost += 5.0 * sumsqr(U_full(0, k)); // 加速度平滑
        cost += 0.1 * sumsqr(X(4, k)); // 前轴转向角平滑
        cost += 0.2 * sumsqr(X(5, k)); // 后轴转向角平滑
    }

    // 终端代价：向量化计算最近路点
    MX pos_N = X(Slice(0,2), N); // 终端状态位置 (x,y)
    MX pos_diff_N = pos_N - waypoints(Slice(0,2), Slice());
    MX dist_sqr_N = sum1(pos_diff_N * pos_diff_N);
    MX min_dist_sqr_N = mmin(dist_sqr_N);

    // 手动实现 argmin
    MX min_idx_N = 0;
    for(int i = 1; i < n_waypoints; ++i) {
        min_idx_N = if_else(dist_sqr_N(i) < dist_sqr_N(min_idx_N), i, min_idx_N);
    }

    MX v_ref_N = waypoints(3, min_idx_N);
    MX theta_ref_N = waypoints(2, min_idx_N);
    // 终端代价权重（调整权重）
    cost += 10.0 * min_dist_sqr_N; // 终端位置跟踪
    cost += 5.0 * sumsqr(X(2, N) - theta_ref_N); // 终端航向跟踪
    cost += 5.0 * sumsqr(X(3, N) - v_ref_N); // 终端速度跟踪

    opti.minimize(cost);

    // ====== 6. 求解器设置 ======
    Dict opts;
    opts["ipopt.print_level"] = 0;         // 关闭IPOPT详细打印
    opts["print_time"] = true;             // 打印求解总时间
    opts["ipopt.sb"] = "yes";              // 抑制IPOPT的冗余输出
    opts["ipopt.max_iter"] = 500;           // 最大迭代次数
    opts["ipopt.tol"] = 1e-2;              // 主精度 tolerance
    opts["ipopt.acceptable_tol"] = 5e-2;   // 可接受精度 tolerance
    // === 新增: 在迭代次数达到 acceptable_iter 后，即便未收敛也返回 ===
    opts["ipopt.acceptable_iter"] = 10;    // 例如，在10次迭代后就允许返回次优解
    opts["ipopt.hessian_approximation"] = "limited-memory"; // 有限内存Hessian
    opts["ipopt.linear_solver"] = "ma27";  // 线性求解器

    opti.solver("ipopt", opts);

    // ====== 7. 设置参数值（圆形参考轨迹示例） ======
    // 初始状态 [x, y, theta, vx, delta1]
    std::vector<double> x0_val = {14.4, 2.0, 1.5, 9.0, 0.0, 0.0};

    // 生成圆形参考轨迹的路点
    std::vector<double> waypoints_data;
    waypoints_data.reserve(4 * n_waypoints);
    double R = 16.0;  // 圆半径 (m)
    double omega = 0.9; // 角速度 (rad/s)
    double total_time = 2 * M_PI / omega; // 完整圆形轨迹的时间 (s)
    for(int i = 0; i < n_waypoints; ++i) {
        double t = i * total_time / (n_waypoints - 1); // 路点时间戳
        waypoints_data.push_back(R * cos(omega * t));  // 路点x坐标
        waypoints_data.push_back(R * sin(omega * t));  // 路点y坐标
        waypoints_data.push_back(omega * t + M_PI/2);  // 路点航向
        waypoints_data.push_back(R * omega);           // 路点速度
    }
    // 计算初始最近路点索引
    DM waypoints_dm = DM::reshape(DM(waypoints_data), 4, n_waypoints);


    // 设置参数值
    opti.set_value(x0, x0_val);
    opti.set_value(waypoints, waypoints_dm);

    // 首次求解
    OptiSol sol_prev = opti.solve();
    bool has_prev_sol = false;

    // ====== 8. 求解与结果打印 ======
    opts["ipopt.max_iter"] = 50;           // 最大迭代次数
    opti.solver("ipopt", opts);

    double start_time = ros::Time::now().toSec();
    int test_num = 50;  // 测试50次
    for (int i = 0; i < test_num; i++) {
        // 给初始状态添加随机扰动
        auto x0_val_rand = x0_val;
        x0_val_rand[0] += (rand() % 100 - 50) * 0.05;
        x0_val_rand[1] += (rand() % 100 - 50) * 0.01;
        x0_val_rand[2] += (rand() % 100 - 50) * 0.01;
        x0_val_rand[3] += (rand() % 100 - 50) * 0.01;

        // 设置参数
        opti.set_value(x0, x0_val_rand);
        opti.set_value(waypoints, waypoints_dm);

        // 热启动：使用前次解
        opti.set_initial(X, sol_prev.value(X));
        opti.set_initial(U_sparse, sol_prev.value(U_sparse));
        try {
            OptiSol sol = opti.solve();
            sol_prev = sol; // 求解成功，更新prev
            has_prev_sol = true;
        }catch(std::exception &e){
            // ROS_ERROR("求解失败: " << e.what());
            std::cout << "求解失败: " << e.what() << std::endl;
            continue;
        }
    }
    double avg_solve_time = (ros::Time::now().toSec() - start_time) * 1000 / test_num;
    
    ROS_INFO_STREAM("求解成功！平均耗时: " << avg_solve_time << " ms");


    
    // 如果求解成功，打印结果
    OptiSol final_sol = opti.solve();
    ROS_INFO_STREAM("优化控制变量数量: " << Nc << " (原始需要: " << N << ")");
    ROS_INFO_STREAM("第一个控制输入: 加速度 = " << final_sol.value(U_sparse(0,0)) 
                    << " m/s², 前轴转角指令 = " << final_sol.value(U_sparse(1,0)) 
                    << " rad, 后轴转角指令 = " << final_sol.value(U_sparse(2,0)) << " rad");

    // 打印稀疏控制量分布
    ROS_INFO("稀疏控制量分布:");
    time_step = 0;
    for(int i = 0; i < Nc; ++i) {
        int end_step = time_step + steps_per_control[i];
        if(end_step > N) end_step = N;
        DM u = final_sol.value(U_sparse(Slice(), i));
        ROS_INFO("u%d: 加速度=%.3f, 前轴转角指令=%.3f, 后轴转角指令=%.3f, 作用于步 %d-%d",
                 i, (double)u(0), (double)u(1), (double)u(2), time_step, end_step-1);
        time_step = end_step;
    }

    // 打印前5个预测步的状态与控制
    for(int k = 0; k < N; ++k) {
        DM xk = final_sol.value(X(Slice(), k));
        DM uk = final_sol.value(U_full(Slice(), k));
        ROS_INFO("预测步%d: x = %.3f, y = %.3f, theta = %.3f, vx = %.3f, delta1 = %.3f, delta2 = %.3f",
                        k, (double)xk(0), (double)xk(1), (double)xk(2), (double)xk(3), (double)xk(4), (double)xk(5));
    }


    ros::shutdown();
    return 0;
}

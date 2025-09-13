#include <casadi/casadi.hpp>
#include <iostream>
#include <ros/ros.h>
#include <vector>

using namespace casadi;

int main(int argc, char * argv[]) {
    // 设置中文编码
    setlocale(LC_ALL, "");
    // 初始化ROS节点
    ros::init(argc, argv, "vehicle_tracking_node");
    ros::NodeHandle nh;
    
    ROS_INFO("车辆轨迹跟踪NMPC求解器节点已启动（稀疏控制量版本，基于路径距离）");

    // ====== 1. 系统参数 ======
    int nx = 4;       // 状态维度 [x, y, theta, v]
    int nu = 2;       // 控制量维度 [a, delta]
    int N = 30;       // 总预测步长
    int Nc = 6;       // 控制步数
    double dt = 0.1;  // 采样时间 (s)
    double L = 2.8;   // 车辆轴距 (m)
    int n_waypoints = 50; // 参考轨迹的路点数量

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
    MX X_sym = MX::sym("X", nx);  // 状态变量 [x, y, theta, v]
    MX U_sym = MX::sym("U", nu);  // 控制变量 [a, delta]

    // 动力学函数
    MX f_expr = MX::vertcat({
        X_sym(3) * cos(X_sym(2)),
        X_sym(3) * sin(X_sym(2)),
        X_sym(3) * tan(U_sym(1)) / L,
        U_sym(0)
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
    MX waypoints = opti.parameter(nx, n_waypoints); // 参考路点 [x, y, theta, v]

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
    opti.subject_to(opti.bounded(-8.0, U_sparse(0, Slice()), 6.0));    // 加速度约束
    opti.subject_to(opti.bounded(-0.523, U_sparse(1, Slice()), 0.523)); // 转向角约束

    // ====== 5. 代价函数 ======
    MX cost = 0;
    // 预测步代价
    for(int k = 0; k < N; ++k) {
        // 当前状态的 x, y 位置（2x1）
        MX pos = X(Slice(0,2), k);

        // 计算到每个线段的最小距离
        MX min_dist_sqr = MX(1e10); // 初始大值（标量）
        MX v_ref = MX(0.0); // 插值参考速度（标量）
        MX theta_ref = MX(0.0); // 插值参考航向角（标量）
        for(int i = 0; i < n_waypoints - 1; ++i) {
            // 线段的起点和终点
            MX p1 = waypoints(Slice(0,2), i);     // 2x1
            MX p2 = waypoints(Slice(0,2), i+1);   // 2x1
            MX v1 = waypoints(3, i);              // 标量
            MX v2 = waypoints(3, i+1);            // 标量
            MX theta1 = waypoints(2, i);          // 标量
            MX theta2 = waypoints(2, i+1);        // 标量

            // 计算点到线段的投影
            MX seg = p2 - p1; // 2x1 线段向量
            MX pos_p1 = pos - p1; // 2x1 点到起点向量
            MX seg_len_sqr = dot(seg, seg); // 标量（使用 dot 确保 1x1）
            MX proj_scalar = dot(pos_p1, seg); // 标量内积
            MX denom = seg_len_sqr + MX(1e-6); // 标量分母
            MX proj_param = proj_scalar / denom; // 标量投影参数
            MX t = fmax(fmin(proj_param, MX(1.0)), MX(0.0)); // 限制 t 在 [0,1]
            
            // 最近点坐标
            MX closest_point = p1 + t * seg; // 2x1
            MX dist_sqr = dot(pos - closest_point, pos - closest_point); // 标量距离平方

            // 更新最小距离和插值值
            MX cond = dist_sqr < min_dist_sqr;
            min_dist_sqr = if_else(cond, dist_sqr, min_dist_sqr);
            v_ref = if_else(cond, (MX(1.0) - t) * v1 + t * v2, v_ref);
            theta_ref = if_else(cond, (MX(1.0) - t) * theta1 + t * theta2, theta_ref);
        }

        // 状态跟踪代价
        cost += 10.0 * min_dist_sqr; // 位置到轨迹距离
        cost += 5.0 * sumsqr(X(2, k) - theta_ref); // 航向角偏差
        cost += 5.0 * sumsqr(X(3, k) - v_ref); // 速度偏差
        
        // 控制正则化
        cost += 5.0 * sumsqr(U_full(0, k)); // 加速度
        cost += 0.1 * sumsqr(U_full(1, k)); // 转向角
    }

    // 终端代价
    MX pos_N = X(Slice(0,2), N);
    MX min_dist_sqr_N = MX(1e10);
    MX v_ref_N = MX(0.0);
    MX theta_ref_N = MX(0.0);
    for(int i = 0; i < n_waypoints - 1; ++i) {
        MX p1 = waypoints(Slice(0,2), i);
        MX p2 = waypoints(Slice(0,2), i+1);
        MX v1 = waypoints(3, i);
        MX v2 = waypoints(3, i+1);
        MX theta1 = waypoints(2, i);
        MX theta2 = waypoints(2, i+1);

        MX seg = p2 - p1;
        MX pos_p1 = pos_N - p1;
        MX seg_len_sqr = dot(seg, seg);
        MX proj_scalar = dot(pos_p1, seg);
        MX denom = seg_len_sqr + MX(1e-6);
        MX proj_param = proj_scalar / denom;
        MX t = fmax(fmin(proj_param, MX(1.0)), MX(0.0));
        
        MX closest_point = p1 + t * seg;
        MX dist_sqr = dot(pos_N - closest_point, pos_N - closest_point); // 修复：使用 pos_N

        MX cond = dist_sqr < min_dist_sqr_N;
        min_dist_sqr_N = if_else(cond, dist_sqr, min_dist_sqr_N);
        v_ref_N = if_else(cond, (MX(1.0) - t) * v1 + t * v2, v_ref_N);
        theta_ref_N = if_else(cond, (MX(1.0) - t) * theta1 + t * theta2, theta_ref_N);
    }
    cost += 20.0 * min_dist_sqr_N; // 终端位置权重更高
    cost += 10.0 * sumsqr(X(2, N) - theta_ref_N); // 终端航向
    cost += 5.0 * sumsqr(X(3, N) - v_ref_N); // 终端速度

    opti.minimize(cost);

    // ====== 6. 求解器设置 ======
    Dict opts;
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = true;
    opts["ipopt.sb"] = "yes";
    opts["ipopt.max_iter"] = 50;
    opts["ipopt.tol"] = 1e-3;
    opts["ipopt.acceptable_tol"] = 1e-3;
    opts["ipopt.hessian_approximation"] = "limited-memory";
    opts["ipopt.linear_solver"] = "ma27";
    opti.solver("ipopt", opts);

    // ====== 7. 设置参数值 ======
    // 初始状态 [x, y, theta, v]
    std::vector<double> x0_val = {14.4, 0.3, 1.5, 9.0};

    // 生成参考轨迹：圆形轨迹示例
    std::vector<double> waypoints_data;
    waypoints_data.reserve(nx * n_waypoints);
    double R = 16.0;  // 圆半径
    double omega = 0.9;  // 角速度
    double total_time = 2 * M_PI / omega; // 完整圆形轨迹时间
    for(int i = 0; i < n_waypoints; ++i) {
        double t = i * total_time / (n_waypoints - 1);
        waypoints_data.push_back(R * cos(omega * t));  // x
        waypoints_data.push_back(R * sin(omega * t));  // y
        waypoints_data.push_back(omega * t + M_PI/2);  // theta
        waypoints_data.push_back(R * omega);           // v
    }

    // 设置参数
    opti.set_value(x0, x0_val);
    opti.set_value(waypoints, DM::reshape(DM(waypoints_data), nx, n_waypoints));

    // ====== 8. 求解 ======
    try {
        double start_time = ros::Time::now().toSec();
        int test_num = 50;
        for (int i = 0; i < test_num; i++) {
            OptiSol sol = opti.solve();
        }
        double avg_solve_time = (ros::Time::now().toSec() - start_time) * 1000 / test_num;
        
        OptiSol sol = opti.solve();
        
        ROS_INFO_STREAM("求解成功！平均耗时: " << avg_solve_time << " ms");
        ROS_INFO_STREAM("优化控制变量数量: " << Nc << " (原始需要: " << N << ")");
        ROS_INFO_STREAM("第一个控制输入: 加速度 = " << sol.value(U_sparse(0,0)) 
                      << " m/s², 转向角 = " << sol.value(U_sparse(1,0)) << " rad");

        // 打印稀疏控制量分布
        ROS_INFO("稀疏控制量分布:");
        time_step = 0;
        for(int i = 0; i < Nc; ++i) {
            int end_step = time_step + steps_per_control[i];
            if(end_step > N) end_step = N;
            DM u = sol.value(U_sparse(Slice(), i));
            ROS_INFO("u%d: 加速度=%.3f, 转向角=%.3f, 作用于步 %d-%d",
                     i, (double)u(0), (double)u(1), time_step, end_step-1);
            time_step = end_step;
        }

        // 打印前5个预测步的结果
        for(int k = 0; k < 5; ++k) {
            DM xk = sol.value(X(Slice(), k));
            DM uk = sol.value(U_full(Slice(), k));
            ROS_INFO_STREAM("预测步 " << k << ": "
                          << "x = " << xk(0) << ", y = " << xk(1) 
                          << ", theta = " << xk(2) << ", v = " << xk(3)
                          << "; 控制: a = " << uk(0) 
                          << ", delta = " << uk(1));
        }
    } catch (std::exception &e) {
        ROS_ERROR_STREAM("求解失败: " << e.what());
    }

    ros::shutdown();
    return 0;
}

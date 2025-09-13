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
    int N = 20;       // 总预测步长
    int Nc = 6;       // 控制步数
    double dt = 0.05;  // 采样时间 (s)
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
    opti.subject_to(opti.bounded(-0.523, U_sparse(1, Slice()), 0.523)); // 转向角约束（≈30°）

    // ====== 5. 代价函数（核心修改部分） ======
    MX cost = 0;
    // 预测步代价：每个步长找最近路点作为参考
    for(int k = 0; k < N; ++k) {
        // 当前状态的位置 (x,y)，2x1 向量
        MX pos = X(Slice(0,2), k);

        // 初始化：最小距离平方（用大值避免初始误判）、参考速度、参考航向
        MX min_dist_sqr = MX(1e10);  // 距离平方（避免开根号，提升效率）
        MX v_ref = MX(0.0);          // 参考速度（初始值无意义，会被更新）
        MX theta_ref = MX(0.0);      // 参考航向（初始值无意义，会被更新）

        // 遍历所有参考路点，找距离当前状态最近的路点
        for(int i = 0; i < n_waypoints; ++i) {
            // 第i个参考路点的信息
            MX wp_pos = waypoints(Slice(0,2), i);  // 路点i的 (x,y) 位置
            MX wp_v = waypoints(3, i);             // 路点i的参考速度
            MX wp_theta = waypoints(2, i);         // 路点i的参考航向

            // 计算当前状态到路点i的距离平方
            MX dist_sqr = dot(pos - wp_pos, pos - wp_pos);

            // 若当前路点更近，则更新最小距离和参考值
            MX is_closer = dist_sqr < min_dist_sqr;
            min_dist_sqr = if_else(is_closer, dist_sqr, min_dist_sqr);
            v_ref = if_else(is_closer, wp_v, v_ref);          // 直接用最近路点的速度
            theta_ref = if_else(is_closer, wp_theta, theta_ref);  // 直接用最近路点的航向
        }

        // 状态跟踪代价（权重与原逻辑一致，仅参考值来源改变）
        cost += 10.0 * min_dist_sqr;                // 位置跟踪：距离最近路点的距离
        cost += 5.0 * sumsqr(X(2, k) - theta_ref);  // 航向跟踪：与最近路点航向的偏差
        cost += 5.0 * sumsqr(X(3, k) - v_ref);      // 速度跟踪：与最近路点速度的偏差
        
        // 控制量正则化（原逻辑不变，抑制控制量过大）
        cost += 5.0 * sumsqr(U_full(0, k));         // 加速度平滑
        cost += 0.1 * sumsqr(U_full(1, k));         // 转向角平滑
    }

    // 终端代价：对最后一个预测步，同样找最近路点作为参考（权重更高）
    MX pos_N = X(Slice(0,2), N);  // 终端状态的位置 (x,y)
    MX min_dist_sqr_N = MX(1e10);
    MX v_ref_N = MX(0.0);
    MX theta_ref_N = MX(0.0);

    // 遍历所有路点，找终端位置的最近路点
    for(int i = 0; i < n_waypoints; ++i) {
        MX wp_pos = waypoints(Slice(0,2), i);
        MX wp_v = waypoints(3, i);
        MX wp_theta = waypoints(2, i);

        MX dist_sqr = dot(pos_N - wp_pos, pos_N - wp_pos);
        MX is_closer = dist_sqr < min_dist_sqr_N;

        min_dist_sqr_N = if_else(is_closer, dist_sqr, min_dist_sqr_N);
        v_ref_N = if_else(is_closer, wp_v, v_ref_N);
        theta_ref_N = if_else(is_closer, wp_theta, theta_ref_N);
    }

    // 终端代价权重（原逻辑不变，强调终端跟踪精度）
    cost += 20.0 * min_dist_sqr_N;                // 终端位置跟踪
    cost += 10.0 * sumsqr(X(2, N) - theta_ref_N);  // 终端航向跟踪
    cost += 5.0 * sumsqr(X(3, N) - v_ref_N);      // 终端速度跟踪




    opti.minimize(cost);

    // ====== 6. 求解器设置（原逻辑不变） ======
    Dict opts;
    opts["ipopt.print_level"] = 0;         // 关闭IPOPT详细打印
    opts["print_time"] = true;             // 打印求解总时间
    opts["ipopt.sb"] = "yes";              // 抑制IPOPT的冗余输出
    opts["ipopt.max_iter"] = 50;           // 最大迭代次数（平衡精度与速度）
    opts["ipopt.tol"] = 1e-2;              // 主精度 tolerance
    opts["ipopt.acceptable_tol"] = 5e-2;   // 可接受精度 tolerance
    opts["ipopt.hessian_approximation"] = "limited-memory"; // 有限内存Hessian（提升速度）
    opts["ipopt.linear_solver"] = "ma27";  // 线性求解器（适合中小型问题）

    opts["ipopt.warm_start_init_point"] = "yes";    // 启用热启动（利用前次解）
    opts["ipopt.warm_start_bound_push"] = 1e-6;     // 热启动边界松弛
    opts["ipopt.warm_start_slack_bound_push"] = 1e-6;
    



    opti.solver("ipopt", opts);

    // ====== 7. 设置参数值（原逻辑不变，圆形参考轨迹示例） ======
    // 初始状态 [x, y, theta, v]
    std::vector<double> x0_val = {14.4, 0.3, 1.5, 9.0};

    // 生成圆形参考轨迹的路点
    std::vector<double> waypoints_data;
    waypoints_data.reserve(nx * n_waypoints);
    double R = 16.0;  // 圆半径 (m)
    double omega = 0.9;// 角速度 (rad/s)
    double total_time = 2 * M_PI / omega; // 完整圆形轨迹的时间 (s)
    for(int i = 0; i < n_waypoints; ++i) {
        double t = i * total_time / (n_waypoints - 1); // 路点时间戳
        waypoints_data.push_back(R * cos(omega * t));  // 路点x坐标
        waypoints_data.push_back(R * sin(omega * t));  // 路点y坐标
        waypoints_data.push_back(omega * t + M_PI/2);  // 路点航向（垂直于半径向外）
        waypoints_data.push_back(R * omega);           // 路点速度（圆周运动速度 v=Rω）
    }



    // ====== 8. 求解与结果打印（原逻辑不变） ======
    try {
        // 将参数值传入优化器
        opti.set_value(x0, x0_val);
        opti.set_value(waypoints, DM::reshape(DM(waypoints_data), nx, n_waypoints));
        OptiSol sol = opti.solve();

        double start_time = ros::Time::now().toSec();
        int test_num = 50;  // 测试50次，计算平均求解时间
        for (int i = 0; i < test_num; i++) {
            // 给初始状态添加随机扰动
            auto x0_val_rand = x0_val;
            x0_val_rand[0] += (rand() % 100 - 50) * 0.01;
            x0_val_rand[1] += (rand() % 100 - 50) * 0.01;
            x0_val_rand[2] += (rand() % 100 - 50) * 0.01;
            x0_val_rand[3] += (rand() % 100 - 50) * 0.01;
            // 将参数值传入优化器
            opti.set_value(x0, x0_val_rand);
            opti.set_value(waypoints, DM::reshape(DM(waypoints_data), nx, n_waypoints));

            // // 关键：将前次的状态和控制解设为当前初始猜测
            opti.set_initial(X, sol.value(X));
            opti.set_initial(U_sparse, sol.value(U_sparse));

            OptiSol sol = opti.solve();
        }
        double avg_solve_time = (ros::Time::now().toSec() - start_time) * 1000 / test_num;
        
        
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

        // 打印前5个预测步的状态与控制
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
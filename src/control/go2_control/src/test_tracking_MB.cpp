#include <casadi/casadi.hpp>
#include <iostream>
#include <ros/ros.h>
#include <vector>

using namespace casadi;

int main(int argc, char * argv[] ){
    // 设置中文编码
    setlocale(LC_ALL, "");
    // 初始化ROS节点
    ros::init(argc, argv, "vehicle_tracking_node");
    ros::NodeHandle nh;
    
    ROS_INFO("车辆轨迹跟踪NMPC求解器节点已启动（稀疏控制量版本）");

    // ====== 1. 系统参数 ======
    int nx = 4;       // 状态维度 [x, y, theta, v]
    int nu = 2;       // 控制量维度 [a, delta]，a为加速度，delta为前轮转向角
    int N = 30;       // 总预测步长（时间步数）
    int Nc = 6;       // 实际需要优化的控制步数（控制变量数量）
    double dt = 0.1;  // 采样时间 (s)
    double L = 2.8;   // 车辆轴距 (m)

    // 计算每个控制量作用的时间步数（阶梯式递增）
    // 确保总步数等于N
    std::vector<int> steps_per_control(Nc);
    int remaining_steps = N;
    for(int i = 0; i < Nc; ++i) {
        // 前几个控制量使用较少步数，后面逐渐增加
        // 这里采用递增模式：1, 2, 3, ..., 最后一个控制量使用剩余所有步数
        if(i < Nc - 1) {
            steps_per_control[i] = i + 1;  // 1, 2, 3, 4, 5...
            remaining_steps -= steps_per_control[i];
        } else {
            steps_per_control[i] = remaining_steps;  // 最后一个控制量使用剩余步数
        }
    }

    // 打印控制量分布情况，验证是否正确
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

    // 定义车辆运动学模型动力学函数 f(x,u)
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
    MX X = opti.variable(nx, N+1);      // 状态序列（N+1个状态对应N个控制步）
    MX U_sparse = opti.variable(nu, Nc); // 稀疏控制量（仅Nc个控制变量）
    MX x0 = opti.parameter(nx);         // 初始状态
    MX x_ref = opti.parameter(nx, N+1); // 参考轨迹参数（包含N+1个参考点）

    // 构建完整控制序列（将稀疏控制量映射到所有时间步）
    MX U_full = MX::zeros(nu, N);  // 完整控制序列（N个控制量）
    int time_step = 0;
    
    for(int i = 0; i < Nc; ++i) {
        // 当前控制量作用的时间步范围
        int end_step = time_step + steps_per_control[i];
        // 确保不超过总步数
        if(end_step > N) end_step = N;
        
        // 将第i个稀疏控制量赋值给对应的时间步范围
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

    // 控制约束（应用于稀疏控制量）
    opti.subject_to(opti.bounded(-8.0, U_sparse(0, Slice()), 6.0));    // 加速度约束 [-8, 6] m/s^2
    opti.subject_to(opti.bounded(-0.523, U_sparse(1, Slice()), 0.523)); // 转向角约束 [-30°, 30°] (弧度)

    // ====== 5. 代价函数 ======
    MX cost = 0;
    for(int k = 0; k < N; ++k) {
        // 状态跟踪代价
        cost += 10.0 * sumsqr(X(Slice(0,2), k) - x_ref(Slice(0,2), k))  // x,y跟踪权重
              + 5.0 * sumsqr(X(2, k) - x_ref(2, k))                    // 航向角跟踪权重
              + 5.0 * sumsqr(X(3, k) - x_ref(3, k))                    // 速度跟踪权重
              + 5.0 * sumsqr(U_full(0, k))                             // 加速度正则化
              + 0.1 * sumsqr(U_full(1, k));                            // 转向角正则化
    }
    // 终端代价
    cost += 20.0 * sumsqr(X(Slice(0,2), N) - x_ref(Slice(0,2), N))
          + 10.0 * sumsqr(X(2, N) - x_ref(2, N))
          + 5.0 * sumsqr(X(3, N) - x_ref(3, N));
          
    opti.minimize(cost);

    // ====== 6. 求解器设置 ======
    Dict opts;
    // 1. 启用 CasADi 稀疏性检测（核心！让 CasADi 识别稀疏结构）

    opts["ipopt.print_level"] = 0;    // 关闭迭代输出
    opts["print_time"] = true;       // 关闭总耗时输出
    opts["ipopt.sb"] = "yes";         // 隐藏边界乘子
    opts["ipopt.max_iter"] = 50;     // 设置最大迭代次数
    opts ["ipopt.tol"] = 1e-3; // 降低收敛精度（实时控制足够）
    opts ["ipopt.acceptable_tol"] = 1e-3; // 可接受精度（进一步减少迭代）
    opts ["ipopt.hessian_approximation"] = "limited-memory"; // 用 L-BFGS 近似 Hessian（避免计算稠密 Hessian）
    // 启用稀疏矩阵存储和计算
    opts["ipopt.linear_solver"] = "ma27";  // 使用ma27稀疏求解器
    opti.solver("ipopt", opts);

    // ====== 7. 设置参数值 ======
    // 初始状态 [x, y, theta, v]
    std::vector<double> x0_val = {14.4, 0.3, 1.5, 9.0};    // 轻微偏移参考轨迹
    
    // 生成参考轨迹：圆形轨迹示例
    // 使用一维向量存储，按列优先顺序
    std::vector<double> x_ref_data;
    x_ref_data.reserve(nx * (N + 1));
    
    double R = 16.0;  // 圆半径
    double omega = 0.9;  // 角速度
    for(int k = 0; k <= N; ++k) {
        double t = k * dt;
        // 按列存储: [x0, y0, theta0, v0, x1, y1, theta1, v1, ...]
        x_ref_data.push_back(R * cos(omega * t));  // x参考
        x_ref_data.push_back(R * sin(omega * t));  // y参考
        x_ref_data.push_back(omega * t + M_PI/2);  // 航向角参考（切线方向）
        x_ref_data.push_back(R * omega);           // 速度参考
    }
    
    // 设置参数，使用DM矩阵进行转换
    opti.set_value(x0, x0_val);
    opti.set_value(x_ref, DM::reshape(DM(x_ref_data), nx, N+1));

    // ====== 8. 求解 ======
    try {
        // 记录求解时间
        double start_time = ros::Time::now().toSec();
        int test_num = 500;
        for (int i = 0; i < test_num; i++) {
            OptiSol sol = opti.solve();
        }
        double avg_solve_time = (ros::Time::now().toSec() - start_time) * 1000 / test_num;
        
        // 最后一次求解的结果
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

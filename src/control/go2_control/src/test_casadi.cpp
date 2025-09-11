#include <casadi/casadi.hpp>
#include <iostream>
#include <ros/ros.h>

using namespace casadi;

int main(int argc, char * argv[] ){
    // 设置中文编码
    setlocale(LC_ALL, "");
    // 初始化ROS节点
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    
    ROS_INFO("NMPC Solver Test Node Started");

    // ====== 1. 系统参数 ======
    int nx = 2;      // 状态维度 [theta, theta_dot]
    int nu = 1;      // 控制量维度 [u]
    int N = 20;      // 预测步长
    double dt = 0.05; // 采样时间

    // ====== 2. 定义符号变量 ======
    MX X_sym = MX::sym("X", nx);
    MX U_sym = MX::sym("U", nu);

    // 定义动力学函数 f(x,u)
    MX f_expr = MX::vertcat({X_sym(1), MX::sin(X_sym(0)) + U_sym(0)});
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
    MX X = opti.variable(nx, N+1); // 状态序列
    MX U = opti.variable(nu, N);   // 控制序列
    MX x0 = opti.parameter(nx);    // 初始状态
    MX x_ref = opti.parameter(nx); // 目标状态

    // 动力学约束
    for(int k=0; k<N; ++k) {
        opti.subject_to(X(Slice(), k+1) == rk4(X(Slice(), k), U(Slice(), k), dt));
    }

    // 初始条件
    opti.subject_to(X(Slice(),0) == x0);

    // 控制约束
    opti.subject_to(opti.bounded(-2.0, U, 2.0));

    // 状态约束（角度限制示例）
    opti.subject_to(opti.bounded(-3.14, X, 3.14));

    // ====== 5. 代价函数 ======
    MX cost = 0;
    for(int k=0; k<N; ++k){
        cost += sumsqr(X(Slice(), k) - x_ref) + 0.01*sumsqr(U(Slice(), k));
    }
    opti.minimize(cost);

    // ====== 6. 求解器设置（关闭 IPOPT 输出） ======
    Dict opts;
    opts["ipopt.print_level"] = 0;  // 关闭迭代输出
    opts["print_time"] = true;     // 关闭总耗时输出
    opts["ipopt.sb"] = "yes";       // 隐藏边界乘子
    
    // 启用稀疏矩阵存储和计算
    opts["ipopt.linear_solver"] = "ma27";  // 使用你安装的HSL中的MA27稀疏求解器
    opts["ipopt.print_info_string"] = "yes";  // 验证是否使用HSL
    opti.solver("ipopt", opts);

    // ====== 7. 设置参数值 ======
    std::vector<double> x0_val = {0.0, 0.0};       // 初始状态
    std::vector<double> x_ref_val = {3.14, 0.0};   // 目标状态 (倒立 π)
    opti.set_value(x0, x0_val);
    opti.set_value(x_ref, x_ref_val);


    // ====== 8. 求解 ======
    try {
        OptiSol sol = opti.solve();
        ROS_INFO_STREAM("最优控制输入 U[0] = " << sol.value(U(Slice(),0)));

        // 打印前 5 个预测步的控制量和状态
        for(int k=0; k<5; ++k){
            DM xk = sol.value(X(Slice(), k));
            DM uk = sol.value(U(Slice(), k));
            ROS_INFO_STREAM("Step " << k << ": x = " << xk << ", u = " << uk);
        }
    } catch (std::exception &e){
        ROS_ERROR_STREAM("求解失败: " << e.what());
    }

    ros::shutdown();
    return 0;
}

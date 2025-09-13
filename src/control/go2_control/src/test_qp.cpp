#include <casadi/casadi.hpp>
#include <iostream>
#include <ros/ros.h>
#include <vector>

int main(int argc, char * argv[] ){
    // 设置中文编码
    setlocale(LC_ALL, "");
    // 初始化ROS节点
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    
    ROS_INFO("QP Solver Test Node Started");

    // 创建问题变量
    casadi::SX x = casadi::SX::sym("x", 2);  // 2维变量x
    casadi::SX f = x(0)*x(0)+x(1)*x(1);  // 目标函数 f = x1^2 + x2^2
    // 创建约束条件
    casadi::SX g = x(0) + x(1) - 1;  // 约束条件 g = x1 + x2 - 1
    // 创建问题对象
    casadi::SXDict qp = { { "x", x }, { "f", f }, { "g", g } };
    // 创建求解器
    casadi::Dict opts;
    casadi::Function solver = casadi::qpsol("solver", "qpoases", qp, opts);
    // 设置求解器参数
    std::map<std::string, casadi::DM> args, res;
    args["lbg"] = 0.0;  // 约束下界
    args["ubg"] = 0.0;  // 约束上界
    // 求解问题
    res = solver(args);
    // 输出结果
    std::cout << "Optimal solution: " << res.at("x") << std::endl;
    std::cout << "Optimal cost: " << res.at("f") << std::endl;

    
    ros::shutdown();
    return 0;
}

#include <casadi/casadi.hpp>
#include <iostream>
#include <vector>
#include <ros/ros.h>

using namespace casadi;

// 定义车辆参数结构体（不变）
struct VehicleParams {
    double m;        // 质量 (kg)
    double Iz;       // 横摆转动惯量 (kg·m²)
    double a;        // 前轴到质心距离 (m)
    double b;        // 后轴到质心距离 (m)
    double k1;       // 前轴轮胎刚度 (N/rad)
    double k2;       // 后轴轮胎刚度 (N/rad)
    double T_d1;     // 前轴执行器时间常数 (s)
    double T_d2;     // 后轴执行器时间常数 (s)
    double T_da;     // 加速执行器时间常数 (s)
    double T_db;     // 制动执行器时间常数 (s)
    double T_La;     // 加速时延
    double T_Lb;     // 制动时延
    double K1;       // 前轴执行器增益
    double K2;       // 后轴执行器增益
    double T_L1;     // 前轴执行器时延
    double T_L2;     // 后轴执行器时延
    double brush;    // 刷子系数
    double max_steer_vel; // 车轮转向最大速度
    std::vector<double> drive_acc; // 驱动加速度曲线
    std::vector<double> drive_dcc; // 制动减速度曲线
    std::vector<double> drive_vel; // 速度节点
    double slope_acc; // 坡度加速度
    double g;        // 重力加速度
    // 车辆尺寸参数
    double length;    // 车辆轴距
    double width;     // 车辆宽度
    double wheelbase; // 轮距
    double wheel_radius; // 车轮半径
    double wheel_width;  // 车轮宽度
    double a_plus;     // 前轴前长度
    double b_plus;     // 后轴后长度
};

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

// 车辆动力学模型 (Casadi符号版本，不变)
MX vehicle_dynamics(const MX& t, const MX& xi, const std::vector<double>& input_t, 
                   const std::vector<std::vector<double>>& input_u, const VehicleParams& params) {
    // 状态分解
    MX psi = xi(2);        // 航向角 (rad)
    MX v_x = xi(3);        // 纵向速度 (m/s)
    MX v_y = xi(4);        // 横向速度 (m/s)
    MX omega_r = xi(5);    // 横摆角速度 (rad/s)
    MX delta1 = xi(6);     // 前轴实际转向角 (rad)
    MX delta2 = xi(7);     // 后轴实际转向角 (rad)
    MX a_x_power = xi(8);  // 车辆加速度 (m/s2)
    
    // 提取控制输入分量（用于插值）
    std::vector<double> u1, u2, u3;
    for(auto& u : input_u) {
        u1.push_back(u[0]);
        u2.push_back(u[1]);
        u3.push_back(u[2]);
    }
    
    // 控制输入插值（内部处理，无需外部传u）
    MX delta_des1 = interp1(input_t, u1, t);  // 前轴期望转向角
    MX delta_des2 = interp1(input_t, u2, t);  // 后轴期望转向角
    MX throttle_brake = interp1(input_t, u3, t); // 油门/制动
    
    // 防止v_x为零
    double min_vx = 0.1;
    MX v_x_safe = if_else(fabs(v_x) < min_vx, min_vx, v_x);
    
    // 1. 轮胎侧偏角
    MX alpha1 = atan2(v_y + params.a * omega_r, v_x_safe) - delta1;
    MX alpha2 = atan2(v_y - params.b * omega_r, v_x_safe) - delta2;
    
    // 2. 轮胎侧向力
    MX F_y1 = params.k1 * atan(alpha1 * params.brush) / params.brush;
    MX F_y2 = params.k2 * atan(alpha2 * params.brush) / params.brush;
    
    // 3. 动力学方程
    MX dx = v_x_safe * cos(psi) - v_y * sin(psi);
    MX dy = v_x_safe * sin(psi) + v_y * cos(psi);
    MX dpsi = omega_r;
    
    // 纵向动力学（加速度限制插值）
    MX acc_lim = interp1(params.drive_vel, params.drive_acc, v_x_safe) * params.g;
    MX dcc_lim = interp1(params.drive_vel, params.drive_dcc, v_x_safe) * params.g;
    MX target_acc = fabs(throttle_brake) * if_else(throttle_brake > 0, acc_lim, dcc_lim);
    MX da_x_power = if_else(target_acc > 0, (target_acc - a_x_power)/params.T_da, (target_acc - a_x_power)/params.T_db);
    MX dv_x = if_else(v_x > 0,a_x_power + params.slope_acc - (fabs(F_y1)*sin(delta1) + fabs(F_y2)*sin(delta2))/params.m,0);
    
    // 横向动力学
    MX dv_y = (F_y1*cos(delta1) + F_y2*cos(delta2))/params.m - v_x_safe*omega_r;
    
    // 横摆动力学
    MX d_omega_r = (params.a*F_y1*cos(delta1) - params.b*F_y2*cos(delta2))/params.Iz;
    
    // 执行器动态（带饱和）
    MX d_delta1 = fmin(fmax(params.K1*(delta_des1 - delta1)/params.T_d1, -params.max_steer_vel), params.max_steer_vel);
    MX d_delta2 = fmin(fmax(params.K2*(delta_des2 - delta2)/params.T_d2, -params.max_steer_vel), params.max_steer_vel);
    
    // 返回状态导数（9维）
    return MX::vertcat({dx, dy, dpsi, dv_x, dv_y, d_omega_r, d_delta1, d_delta2, da_x_power});
}

// 修复后的RK4数值积分（输入输出均为DM类型，调用编译后的数值函数）
DM rk4(const DM& x, const DM& t, double dt, const Function& f) {
    // 计算RK4的4个k值（调用f函数，输入t和x，输出dx）
    DM k1 = f(std::vector<DM>{t, x})[0];  // f(t, x) → dx
    DM k2 = f(std::vector<DM>{t, x + dt/2*k1})[0];
    DM k3 = f(std::vector<DM>{t, x + dt/2*k2})[0];
    DM k4 = f(std::vector<DM>{t, x + dt*k3})[0];
    
    // 计算下一个状态（数值类型DM）
    return x + dt/6 * (k1 + 2*k2 + 2*k3 + k4);
}

int main(int argc, char * argv[]) {
    // 设置中文编码 + 初始化ROS
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "vehicle_tracking_node");
    ros::NodeHandle nh;

    // 1. 车辆参数配置（不变）
    VehicleParams params;
    params.m = 760;    // 质量 (kg)
    params.Iz = 800;   // 横摆转动惯量 (kg·m²)
    params.a = 0.95;   // 前轴到质心距离 (m)
    params.b = 0.95;   // 后轴到质心距离 (m)
    params.k1 = -18800; // 前轴轮胎刚度 (N/rad)
    params.k2 = -18800; // 后轴轮胎刚度 (N/rad)
    params.T_d1 = 0.5; // 前轴执行器时间常数 (s)
    params.T_d2 = 0.5; // 后轴执行器时间常数 (s)
    params.T_da = 0.1; // 加速执行器时间常数 (s)
    params.T_db = 0.05; // 制动执行器时间常数 (s)
    params.T_La = 0.1; // 加速时延
    params.T_Lb = 0.1; // 制动时延
    params.K1 = 1.0;   // 前轴执行器增益
    params.K2 = 1.0;   // 后轴执行器增益
    params.T_L1 = 0.1; // 前轴执行器时延
    params.T_L2 = 0.1; // 后轴执行器时延
    params.brush = 1.0; // 刷子系数
    params.max_steer_vel = M_PI/4; // 车轮转向最大速度
    params.drive_acc = {0.6, 0.6, 0.45, 0.25, 0.1, 0.0, 0.0};
    params.drive_dcc = {-0.8, -0.8, -0.8, -0.8, -0.8, -0.8, -0.8};
    params.drive_vel = {0, 3, 6, 10, 15, 20, 25};
    params.slope_acc = 0.0;
    params.g = 9.806;  // 重力加速度
    
    // 车辆尺寸参数
    params.length = params.a + params.b;
    params.width = 1.7;
    params.wheelbase = 1.4;
    params.wheel_radius = 0.3;
    params.wheel_width = 0.2;
    params.a_plus = 0.3;
    params.b_plus = 0.3;
    
    // 2. 仿真设置（不变）
    std::vector<double> xi0 = {0, 0, 0, 12, 0, 0, 0, 0, 0}; // 初始状态
    std::vector<double> input_t = {0, 3, 5}; // 控制输入时间点
    std::vector<std::vector<double>> input_u = { // 控制输入值
        {0.2, -0.05, -1},
        {0.1, -0.1, -0.5},
        {0, 0, 0}
    };
    double t_start = 0.0, t_end = 5.0, dt = 0.05;
    int num_steps = (t_end - t_start) / dt + 1;
    
    // 3. 构建Casadi符号模型并编译为数值函数（核心修复）
    // 3.1 定义符号变量（t：时间，x：状态）
    MX t_sym = MX::sym("t");          // 标量时间
    MX x_sym = MX::sym("x", 9);       // 9维状态向量
    // 3.2 构建符号动力学模型（传入input_t/input_u/params，生成dx_sym）
    MX dx_sym = vehicle_dynamics(t_sym, x_sym, input_t, input_u, params);
    // 3.3 编译为数值函数：输入(t, x) → 输出(dx)
    Function f("vehicle_dynamics_func", {t_sym, x_sym}, {dx_sym});

    
    // 4. 运行数值仿真（全DM类型，无MX混用）
    DM x_current = DM(xi0); // 当前状态（数值类型）
    std::vector<DM> results;
    results.push_back(x_current);
    
    ROS_INFO("开始仿真...");
    for(int i = 0; i < num_steps - 1; ++i) {
        // 当前时间（转换为DM类型，匹配f函数输入）
        double t_current = t_start + i * dt;
        DM t_current_dm = DM(t_current);
        
        // 调用RK4积分（输入输出均为DM，类型匹配）
        x_current = rk4(x_current, t_current_dm, dt, f);
        
        // 保存结果
        results.push_back(x_current);
        
        // 打印进度（每10步更新一次）
        if(i % 10 == 0) {
            ROS_INFO_STREAM("仿真进度: " << std::fixed << std::setprecision(1) 
                          << (i * 100.0) / (num_steps - 1) << "%");
        }
    }
    ROS_INFO("仿真完成!");
    
    // 5. 输出部分结果（ROS日志打印）
    ROS_INFO("\n部分仿真结果 (t, x, y, psi, v_x):");
    ROS_INFO("-----------------------------------------");
    for(int i = 0; i < results.size(); i += 10) { // 每10步输出一次
        double t = t_start + i * dt;
        DM x = results[i];
        ROS_INFO_STREAM(std::fixed << std::setprecision(2)
                      << "t=" << t << ": x=" << x(0) << ", y=" << x(1) 
                      << ", psi=" << x(2) << ", v_x=" << x(3) << ", ax_power=" << x(8));
    }

    ros::shutdown();
    return 0;
}
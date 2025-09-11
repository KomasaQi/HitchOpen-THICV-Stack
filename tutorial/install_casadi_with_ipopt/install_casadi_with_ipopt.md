# Ubuntu 20.04 CasADi 的C++安装方法（源码编译 含Ipopt）

## [Ubuntu 20.04 CasADi 的C++安装方法（源码编译 含Ipopt）](https://blog.csdn.net/Li_Dongyi/article/details/151186844) 详细方法参考本文章

# ROS1版本的倒立摆控制程序

下面将为你提供适配ROS1的完整流程，包括包创建、配置文件和代码修改。

## 1. 新建ROS1包

```bash
catkin_create_pkg go2_control roscpp std_msgs
cd go2_control
mkdir -p src
```

## 2. 修改CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(go2_control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs
)

## Build test_casadi
add_executable(test_casadi src/test_casadi.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(test_casadi
  ${catkin_LIBRARIES}
  /usr/local/lib/libcasadi.so
)

## Include directories
target_include_directories(test_casadi PRIVATE
  ${catkin_INCLUDE_DIRS}
  /usr/local/include
)

## Install targets
install(TARGETS test_casadi
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

## 3. 确保环境变量配置

与ROS2相同，在~/.bashrc中加入：

```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

刷新环境变量：

```bash
source ~/.bashrc
```

## 4. 编写ROS1版本的test_casadi.cpp


``` cpp

#include <casadi/casadi.hpp>
#include <iostream>
#include <ros/ros.h>

using namespace casadi;

int main(int argc, char * argv[] ){
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
    opts["print_time"] = false;     // 关闭总耗时输出
    opts["ipopt.sb"] = "yes";       // 隐藏边界乘子
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

```


## 5. 编译和运行

在工作空间目录下编译：

```bash
catkin_make --pkg go2_control
```

编译后刷新环境变量：

```bash
source devel/setup.bash
```

运行节点：

```bash
rosrun go2_control test_casadi
```

## 主要修改说明

1. **包创建**：使用`catkin_create_pkg`替代`ros2 pkg create`
2. **CMakeLists.txt**：
   - 使用ROS1的`catkin_package`替代`ament_cmake`相关配置
   - 调整了包含目录和链接库的方式
   - 使用ROS1的安装路径变量`CATKIN_PACKAGE_BIN_DESTINATION`

3. **代码修改**：
   - 头文件从`rclcpp/rclcpp.hpp`改为`ros/ros.h`
   - 节点初始化使用`ros::init`和`ros::NodeHandle`
   - 日志输出使用ROS1的`ROS_INFO_STREAM`和`ROS_ERROR_STREAM`
   - 节点关闭使用`ros::shutdown()`

4. **编译和运行命令**：
   - 使用`catkin_make`替代`colcon build`
   - 使用`rosrun`替代`ros2 run`
   - 环境变量脚本为`devel/setup.bash`而非`install/setup.bash`

运行后，你应该能看到与ROS2版本类似的输出结果，只是日志格式会采用ROS1的风格。



下面是一些测试：

# 写一个倒立摆的例子 ROS2
新建一个包
``` bash
ros2 pkg create go2_control --build-type ament_cmake --dependencies rclcpp std_msgs
```
在CMakeists.txt中修改/补充：
``` cmake
project(nmpc_solver_test)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 添加一个可执行程序
add_executable(test_casadi src/test_casadi.cpp)
 
# 添加相关库文件链接到工程
target_link_libraries(test_casadi /usr/local/lib/libcasadi.so.3.7) 
target_include_directories(test_casadi PRIVATE /usr/local/include)
ament_target_dependencies(test_casadi rclcpp std_msgs)

install(TARGETS
  test_casadi
  DESTINATION lib/${PROJECT_NAME}
)
```
在Home的.bashrc中加入（加入bashrc，以后就不用手动加了）：
``` bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

刷新一下：
``` bash
source ~/.bashrc
```

然后编写test_casadi.cpp文件的内容，因为四足机器人在ROS2下运行，所以也写成了ROS2格式的;
``` cpp
#include <casadi/casadi.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

using namespace casadi;

int main(int argc, char * argv[] ){

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("test_node");

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
    opts["print_time"] = false;     // 关闭总耗时输出
    opts["ipopt.sb"] = "yes";       // 隐藏边界乘子
    opti.solver("ipopt", opts);

    // ====== 7. 设置参数值 ======
    std::vector<double> x0_val = {0.0, 0.0};       // 初始状态
    std::vector<double> x_ref_val = {3.14, 0.0};   // 目标状态 (倒立 π)
    opti.set_value(x0, x0_val);
    opti.set_value(x_ref, x_ref_val);

    // ====== 8. 求解 ======
    try {
        OptiSol sol = opti.solve();
        std::cout << "最优控制输入 U[0] = " << sol.value(U(Slice(),0)) << std::endl;

        // 打印前 5 个预测步的控制量和状态
        for(int k=0; k<5; ++k){

            DM xk = sol.value(X(Slice(), k));
            DM uk = sol.value(U(Slice(), k));
            std::cout << "Step " << k << ": x = " << xk << ", u = " << uk << std::endl;
        }
    } catch (std::exception &e){
        std::cerr << "求解失败: " << e.what() << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}
```
在工作空间目录下编译：

``` bash
colcon build --packages-select nmpc_solver_test
```

编译后刷新下setup.bash
``` bash
source install/setup.bash
```

（或者. install/setup.bash）

运行：
``` bash
ros2 run nmpc_solver_test test_casadi
```

结果：
```
最优控制输入 U[0] = 2
Step 0: x = [-2.64801e-34, -1.73414e-35], u = 2
Step 1: x = [0.00250052, 0.100042], u = 2
Step 2: x = [0.0100083, 0.200333], u = 2
Step 3: x = [0.0225422, 0.301126], u = 2
Step 4: x = [0.0401335, 0.402672], u = 2
```
cmake_minimum_required(VERSION 3.10)
project(mpc_qpoases)

set(CMAKE_CXX_STANDARD 17)

# 添加 pybind11 和 Eigen3
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)

# qpOASES 的路径
include_directories(/path/to/qpOASES/include)
link_directories(/path/to/qpOASES/lib)

# 添加 Python 模块
pybind11_add_module(MPC_Controller_qpOASES_Ycons MPC_Controller_qpOASES_Ycons.cpp)

# 链接 qpOASES 和 Eigen
target_link_libraries(MPC_Controller_qpOASES_Ycons PRIVATE qpOASES Eigen3::Eigen)

cmake_minimum_required(VERSION 3.0.2)
project(go2_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  race_msgs
  tf
  geometry_msgs
)
## 2. 关键：查找Casadi（自动处理头文件和库路径）
# 若find_package失败，需手动指定CasADi_DIR（见下文说明）
find_package(CasADi REQUIRED)
if(NOT CasADi_FOUND)
  message(FATAL_ERROR "Casadi not found! Please install Casadi first.")
endif()
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)





###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES go2_control
 CATKIN_DEPENDS roscpp std_msgs
 DEPENDS # CasADi
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/go2_control.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against


# 通过倒立摆的控制描述了一个简单的NMPC的框架
add_executable(test_casadi src/test_casadi.cpp)
target_link_libraries(test_casadi
  ${catkin_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
  /usr/local/lib/libcasadi.so
)
target_include_directories(test_casadi PRIVATE /usr/local/include)
install(TARGETS test_casadi
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# 针对4自由度运动学进行轨迹跟踪控制的案例
add_executable(test_tracking src/test_tracking.cpp)
target_link_libraries(test_tracking
  ${catkin_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
  /usr/local/lib/libcasadi.so
)
target_include_directories(test_tracking PRIVATE /usr/local/include)
install(TARGETS test_tracking
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# 采用Moving Block方法减少待优化变量的数量
add_executable(test_tracking_MB src/test_tracking_MB.cpp)
target_link_libraries(test_tracking_MB
  ${catkin_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
  /usr/local/lib/libcasadi.so
)
target_include_directories(test_tracking_MB PRIVATE /usr/local/include)
install(TARGETS test_tracking_MB
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 采用点到参考轨迹的插值距离和速度作为参考，而非每个时刻都固定点
add_executable(test_tracking_dist src/test_tracking_dist.cpp)
target_link_libraries(test_tracking_dist
  ${catkin_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
  /usr/local/lib/libcasadi.so
)
target_include_directories(test_tracking_dist PRIVATE /usr/local/include)
install(TARGETS test_tracking_dist
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# 为提速计算，简化参考点计算，直接采用最近点计算每个点的位置和参考速度
add_executable(test_tracking_simple_dist src/test_tracking_simple_dist.cpp)
target_link_libraries(test_tracking_simple_dist
  ${catkin_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
  /usr/local/lib/libcasadi.so
)
target_include_directories(test_tracking_simple_dist PRIVATE /usr/local/include)
install(TARGETS test_tracking_simple_dist
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 测试QP相关求解器是否安装成功
add_executable(test_qp src/test_qp.cpp)
target_link_libraries(test_qp
  ${catkin_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
  /usr/local/lib/libcasadi.so
)
target_include_directories(test_qp PRIVATE /usr/local/include/casadi)
install(TARGETS test_qp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 测试暖启动
add_executable(test_tracking_warmstart src/test_tracking_warmstart.cpp)
target_link_libraries(test_tracking_warmstart
  ${catkin_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
  /usr/local/lib/libcasadi.so
)
target_include_directories(test_tracking_warmstart PRIVATE /usr/local/include)
install(TARGETS test_tracking_warmstart
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 进一步对简化计算距离方案进行优化
add_executable(test_tracking_simple_dist_opt src/test_tracking_simple_dist_opt.cpp)
target_link_libraries(test_tracking_simple_dist_opt
  ${catkin_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
  /usr/local/lib/libcasadi.so
)
target_include_directories(test_tracking_simple_dist_opt PRIVATE /usr/local/include)
install(TARGETS test_tracking_simple_dist_opt
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# 对车辆模型进行仿真验证
add_executable(sim_vehicle_model src/sim_vehicle_model.cpp)
target_link_libraries(sim_vehicle_model
  ${catkin_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
  /usr/local/lib/libcasadi.so
)
target_include_directories(sim_vehicle_model PRIVATE /usr/local/include)
install(TARGETS sim_vehicle_model
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)




# 双轴转向轨迹跟踪
add_executable(dual_axis_tracker src/dual_axis_tracker.cpp)
target_link_libraries(dual_axis_tracker
  ${catkin_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
  /usr/local/lib/libcasadi.so
)
target_include_directories(dual_axis_tracker PRIVATE /usr/local/include)
install(TARGETS dual_axis_tracker
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 保证不收敛的时候也不会报错，而是返回次优解
add_executable(test_tracking_suboptimal src/test_tracking_suboptimal.cpp)
target_link_libraries(test_tracking_suboptimal
  ${catkin_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
  /usr/local/lib/libcasadi.so
)
target_include_directories(test_tracking_suboptimal PRIVATE /usr/local/include)
install(TARGETS test_tracking_suboptimal
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# 轨迹自动插值的双轴转向跟踪测试
add_executable(atc_dual_axis_tracker src/atc_dual_axis_tracker.cpp)
target_link_libraries(atc_dual_axis_tracker
  ${catkin_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
  /usr/local/lib/libcasadi.so
)
target_include_directories(atc_dual_axis_tracker PRIVATE /usr/local/include)
install(TARGETS atc_dual_axis_tracker
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html


## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_go2_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

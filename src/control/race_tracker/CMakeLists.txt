cmake_minimum_required(VERSION 3.0.2)
project(race_tracker)

# 新增：启用C++17编译（关键，解决std::clamp问题）
# add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  race_msgs
  geometry_msgs
  pluginlib
  tf
)
find_package(Eigen3 REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES race_tracker
  CATKIN_DEPENDS roscpp std_msgs race_msgs geometry_msgs pluginlib
)

find_package(CasADi REQUIRED)
if(NOT CasADi_FOUND)
  message(FATAL_ERROR "Casadi not found! Please install Casadi first.")
endif()

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES race_tracker
 CATKIN_DEPENDS roscpp std_msgs
 DEPENDS CasADi
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  include
)

# Build the main controller manager executable
add_executable(controller_manager src/controller_manager.cpp)
target_link_libraries(controller_manager ${catkin_LIBRARIES})

# Build the plugins
add_library(pure_pursuit_controller plugins/pure_pursuit_controller.cpp)
add_library(pid_controller plugins/pid_controller.cpp)
add_library(nmpc_controller plugins/nmpc_controller.cpp)
# Link against the base class library
target_link_libraries(pure_pursuit_controller ${catkin_LIBRARIES})
target_link_libraries(pid_controller ${catkin_LIBRARIES})
target_link_libraries(nmpc_controller ${catkin_LIBRARIES}
  ${catkin_INCLUDE_DIRS}
  /usr/local/lib/libcasadi.so)
target_include_directories(nmpc_controller PRIVATE /usr/local/include)
# Make the plugins known to ROS
install(TARGETS
  controller_manager
  pure_pursuit_controller
  pid_controller
  nmpc_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

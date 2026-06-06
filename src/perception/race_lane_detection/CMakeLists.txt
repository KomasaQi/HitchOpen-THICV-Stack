cmake_minimum_required(VERSION 3.0.2)
project(race_lane_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  race_msgs
)

## 导出 src/race_lane_detection Python 模块（供节点 import）
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs visualization_msgs cv_bridge race_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 安装 Python 节点与模块
catkin_install_python(PROGRAMS
  scripts/race_lane_detection_node.py
  scripts/download_weights.py
  scripts/debug_on_image.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装 launch / config / 权重目录
install(DIRECTORY launch config rviz weights
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)

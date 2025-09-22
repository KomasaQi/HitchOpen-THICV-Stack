cmake_minimum_required(VERSION 3.0.2)
project(pix_driver)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  automotive_msgs
  can_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
    FILES
    BatteryStatusFb.msg
    ChassisOdoFb.msg
    ChassisWheelRpmFb.msg
    ChassisWheelTorqueFb.msg
    SteerStatusFb.msg
    VehicleStatusFb.msg
    BrakeStatusFb.msg
    ChassisWheelAngleFb.msg
    ChassisWheelTirePressFb.msg
    DriveStatusFb.msg
    VehicleFltStatusFb.msg
    WorkStatusFb.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs  # Or other packages containing msgs
)

catkin_package(
 # INCLUDE_DIRS include
 LIBRARIES pix_driver
 CATKIN_DEPENDS roscpp rospy std_msgs message_generation
 # DEPENDS system_lib
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

add_executable(can_bridge src/can_bridge.cpp)
target_link_libraries(can_bridge
  ${catkin_LIBRARIES}
  boost_system
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS can_bridge
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

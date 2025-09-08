# 新建一个ROS软件包

在ROS中，软件包（Package）是一种组织代码的方式，它包含了相关的代码、配置文件、启动文件等。每个软件包都有一个唯一的名称，用于在ROS系统中进行识别和引用。

新建一个ROS软件包的步骤如下：
1. 打开终端，进入ROS工作空间的src目录下。
2. 执行以下命令新建一个软件包：
```bash
catkin_create_pkg <package_name> [dependencies]
```
其中，`<package_name>`是你要新建的软件包的名称，`[dependencies]`是该软件包的依赖项，多个依赖项之间用空格隔开。

例如，新建一个名为`my_package`的软件包，依赖于`roscpp`和`rospy`，可以执行以下命令：
```bash
catkin_create_pkg my_package roscpp rospy
```

下面通过2个例子来介绍如何新建一个软件包。

## C++软件包建立
新建一个C++软件包的步骤如下：
1. 打开终端，进入ROS工作空间的src目录下。
2. 执行以下命令新建一个C++软件包：
```bash
catkin_create_pkg <package_name> roscpp rospy std_msgs
```
其中，`<package_name>`是你要新建的软件包的名称，`roscpp`和`rospy`是该软件包的依赖项，`std_msgs`是该软件包的另一个依赖项。



# 创建carla_race_state_converter包的完整流程

## 1. 创建ROS包

首先，创建一个名为`carla_race_state_converter`的ROS包，依赖于必要的消息包：

```bash
cd ~/catkin_ws/src
catkin_create_pkg carla_race_state_converter roscpp std_msgs geometry_msgs nav_msgs carla_msgs race_msgs tf2 tf2_ros
cd ..
catkin_make
source devel/setup.bash
```

## 2. 创建转换节点

在`carla_race_state_converter/src`目录下创建`state_converter_node.cpp`文件：

``` cpp
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <carla_msgs/CarlaEgoVehicleStatus.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <sensor_msgs/Imu.h>
#include <race_msgs/VehicleStatus.h>
#include <race_msgs/Control.h>
#include <race_msgs/Euler.h>
#include <race_msgs/Lateral.h>
#include <race_msgs/WheelSpeed.h>

class StateConverter {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅者
    ros::Subscriber imu_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber speedometer_sub_;
    ros::Subscriber vehicle_status_sub_;
    
    // 发布者
    ros::Publisher vehicle_state_pub_;
    
    // 存储接收到的消息
    sensor_msgs::Imu imu_msg_;
    nav_msgs::Odometry odom_msg_;
    std_msgs::Float32 speedometer_msg_;
    carla_msgs::CarlaEgoVehicleStatus vehicle_status_msg_;
    
    // 消息接收标志
    bool imu_received_;
    bool odom_received_;
    bool speedometer_received_;
    bool vehicle_status_received_;
    
    // 车辆参数（可能需要根据实际车辆调整）
    double wheel_radius_;
    
public:
    StateConverter() : private_nh_("~"), 
                       imu_received_(false),
                       odom_received_(false),
                       speedometer_received_(false),
                       vehicle_status_received_(false) {
        // 获取车辆参数
        private_nh_.param<double>("wheel_radius", wheel_radius_, 0.37);
        
        // 初始化订阅者
        imu_sub_ = nh_.subscribe("/carla/ego_vehicle/imu", 10, &StateConverter::imuCallback, this);
        odom_sub_ = nh_.subscribe("/carla/ego_vehicle/odometry", 10, &StateConverter::odomCallback, this);
        speedometer_sub_ = nh_.subscribe("/carla/ego_vehicle/speedometer", 10, &StateConverter::speedometerCallback, this);
        vehicle_status_sub_ = nh_.subscribe("/carla/ego_vehicle/vehicle_status", 10, &StateConverter::vehicleStatusCallback, this);
        
        // 初始化发布者
        vehicle_state_pub_ = nh_.advertise<race_msgs::VehicleStatus>("/race/vehicle_state", 10);
        
        ROS_INFO("Carla Race State Converter initialized");
    }
    
    // 四元数转欧拉角
    race_msgs::Euler quaternionToEuler(const geometry_msgs::Quaternion& quat) {
        race_msgs::Euler euler;
        tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(euler.roll, euler.pitch, euler.yaw);
        return euler;
    }
    
    // 回调函数
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        imu_msg_ = *msg;
        imu_received_ = true;
        publishVehicleState();
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        odom_msg_ = *msg;
        odom_received_ = true;
        publishVehicleState();
    }
    
    void speedometerCallback(const std_msgs::Float32::ConstPtr& msg) {
        speedometer_msg_ = *msg;
        speedometer_received_ = true;
        publishVehicleState();
    }
    
    void vehicleStatusCallback(const carla_msgs::CarlaEgoVehicleStatus::ConstPtr& msg) {
        vehicle_status_msg_ = *msg;
        vehicle_status_received_ = true;
        publishVehicleState();
    }
    
    // 发布转换后的消息
    void publishVehicleState() {
        // 等待所有必要的消息都被接收
        if (!imu_received_ || !odom_received_ || !speedometer_received_ || !vehicle_status_received_) {
            return;
        }
        
        race_msgs::VehicleStatus state_msg;
        
        // 填充header
        state_msg.header = odom_msg_.header;
        state_msg.child_frame_id = "ego_vehicle";
        
        // 填充pose
        state_msg.pose = odom_msg_.pose.pose;
        
        // 计算并填充euler角
        state_msg.euler = quaternionToEuler(odom_msg_.pose.pose.orientation);
        
        // 填充速度信息
        state_msg.vel = odom_msg_.twist.twist;
        
        // 填充加速度信息（来自IMU）
        state_msg.acc.linear = imu_msg_.linear_acceleration;
        state_msg.acc.angular = imu_msg_.angular_velocity;
        
        // 填充转向信息
        state_msg.lateral.steering_angle = vehicle_status_msg_.control.steer;
        // 这里假设转向角速度无法直接获取，设置为0或需要额外计算
        state_msg.lateral.steering_angle_velocity = 0.0;
        // 双轴转向相关信息，CARLA默认可能不提供，设置为0
        state_msg.lateral.rear_wheel_angle = 0.0;
        state_msg.lateral.rear_wheel_angle_velocity = 0.0;
        
        // 填充车轮速度（假设四轮速度相同，根据车速计算）
        // 注意：这里是简化处理，实际应用中可能需要更精确的计算
        double wheel_angular_speed = speedometer_msg_.data / wheel_radius_;
        state_msg.wheel_speed.left_front = wheel_angular_speed;
        state_msg.wheel_speed.left_rear = wheel_angular_speed;
        state_msg.wheel_speed.right_front = wheel_angular_speed;
        state_msg.wheel_speed.right_rear = wheel_angular_speed;
        
        // 填充档位信息
        // CARLA的gear为1表示前进，-1表示倒车，这里映射到自定义的档位
        if (vehicle_status_msg_.control.reverse) {
            state_msg.gear = race_msgs::Control::GEAR_REVERSE;
        } else if (vehicle_status_msg_.control.gear == 0) {
            state_msg.gear = race_msgs::Control::GEAR_NEUTRAL;
        } else {
            state_msg.gear = vehicle_status_msg_.control.gear;
        }
        
        // 设置控制模式（默认设置为油门刹车模式）
        state_msg.control_mode = race_msgs::Control::THROTTLE_BRAKE_ONLY;
        
        // 填充手刹状态
        state_msg.hand_brake = vehicle_status_msg_.control.hand_brake;
        
        // 紧急状态（这里简化处理，根据刹车值判断）
        state_msg.emergency = (vehicle_status_msg_.control.brake > 0.98);
        
        // 离合状态（CARLA不直接提供，这里设为true）
        state_msg.clutch = true;
        
        // 转向模式（默认设为前轮转向）
        state_msg.steering_mode = race_msgs::Control::FRONT_STEERING_MODE;
        
        // 发布消息
        vehicle_state_pub_.publish(state_msg);
    }
};

int main(int argc, char**argv) {
    ros::init(argc, argv, "carla_race_state_converter");
    StateConverter converter;
    ros::spin();
    return 0;
}


```



## 3. 配置CMakeLists.txt

修改`carla_race_state_converter/CMakeLists.txt`文件：

```bash
cmake_minimum_required(VERSION 3.0.2)
project(carla_race_state_converter)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  carla_msgs
  race_msgs
  tf2
  tf2_ros
  sensor_msgs
)

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
#  INCLUDE_DIRS include
#  LIBRARIES carla_race_state_converter
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/carla_race_state_converter.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/state_converter_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
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
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_carla_race_state_converter.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
```

## 4. 配置package.xml

确保`package.xml`文件包含所有必要的依赖：

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>carla_race_state_converter</name>
  <version>0.0.0</version>
  <description>The carla_race_state_converter package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>nav_msgs</build_depend>
  <build_depend>carla_msgs</build_depend>
  <build_depend>race_msgs</build_depend>
  <build_depend>tf2</build_depend>
  <build_depend>tf2_ros</build_depend>
  <build_depend>sensor_msgs</build_depend>
  
  <exec_depend>roscpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>carla_msgs</exec_depend>
  <exec_depend>race_msgs</exec_depend>
  <exec_depend>tf2</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>

  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>

  <export>
    <!-- Other tools can request additional information be placed here -->
  </export>
</package>
```

## 5. 创建启动文件

在`carla_race_state_converter/launch`目录下创建`state_converter.launch`文件：

```xml
<launch>
  <node name="carla_race_state_converter" pkg="carla_race_state_converter" type="carla_race_state_converter_node" output="screen">
    <!-- 可以在这里设置参数，例如车轮半径 -->
    <param name="wheel_radius" value="0.33" type="double"/>
  </node>
</launch>
```

## 6. 编译和运行

```bash
# 编译包
cd ~/catkin_ws
catkin_make

# 刷新环境变量
source devel/setup.bash

# 运行转换节点
roslaunch carla_race_state_converter state_converter.launch
```

## 7. 验证结果

打开一个新的终端，检查是否正确发布了`/race/vehicle_state`话题：

```bash
# 检查话题是否存在
rostopic list | grep /race/vehicle_state

# 查看话题内容
rostopic echo /race/vehicle_state
```

## 实现说明

1. 节点订阅了四个必要的输入话题，并将它们的信息转换为`race_msgs/VehicleState`消息
2. 四元数到欧拉角的转换使用了tf2库
3. 车轮速度是通过车速和车轮半径计算的简化版本（实际应用中可能需要更精确的模型）
4. 对于CARLA中没有直接对应的字段（如后轮转向角），设置了合理的默认值
5. 控制模式默认为油门刹车模式，可以根据实际需求修改
6. 紧急状态通过刹车值是否接近1.0来判断

如果需要更精确的转换，你可能需要根据实际车辆参数调整代码中的转换逻辑，特别是车轮速度计算和转向相关参数。
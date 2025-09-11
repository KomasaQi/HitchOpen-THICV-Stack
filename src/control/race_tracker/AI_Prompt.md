# Race Messages for HitchOpen THICV Stack
This directory contains self defined message definitions used in the HitchOpen THICV Stack.

## Message Definitions
- [Control.msg](msg/Control.msg)
- [Path.msg](msg/Path.msg)
- [PathPoint.msg](msg/PathPoint.msg)
- [Lateral.msg](msg/Lateral.msg)
- [Longitudinal.msg](msg/Longitudinal.msg)
- [VehicleStatus.msg](msg/VehicleStatus.msg)
- [WheelSpeed.msg](msg/WheelSpeed.msg)
- [Euler.msg](msg/Euler.msg)


# Specific Message Definitions

## Control.msg
``` bash
# 控制命令消息
std_msgs/Header header
race_msgs/Longitudinal longitudinal
race_msgs/Lateral lateral
float64 throttle # 自定义的控制内容：油门 （0~1）
float64 brake  # 自定义的控制内容：刹车 （0~1）

# 档位
int8 gear
int8 GEAR_PARK = -2
int8 GEAR_REVERSE = -1
int8 GEAR_NEUTRAL = 0
int8 GEAR_1 = 1
int8 GEAR_2 = 2
int8 GEAR_3 = 3
int8 GEAR_4 = 4
int8 GEAR_5 = 5
int8 GEAR_6 = 6

uint8 control_mode # 控制模式
uint8 THROTTLE_BRAKE_ONLY = 0
uint8 DES_SPEED_ONLY = 1
uint8 DES_ACCEL_ONLY = 2
bool emergency # 紧急制动标志
bool hand_brake # 手刹标志
bool clutch # 离合标志

# 自定义的内容：是否双轴转向
uint8 steering_mode
uint8 FRONT_STEERING_MODE = 1
uint8 DUAL_STEERING_MODE = 2
```

## Path.msg
``` bash
# 路径消息，与Autoware的Path消息保持一致
std_msgs/Header header

# 路径点序列
race_msgs/PathPoint[] points
```

## PathPoint.msg
``` bash
# 路径点消息
geometry_msgs/Pose pose
# 该点的目标线速度 (m/s)
float64 velocity
# 该点的曲率 (1/m)
float64 curvature
```

## Lateral.msg
``` bash
# 目标转向角 (rad)
float64 steering_angle
# 目标转向角速度 (rad/s)
float64 steering_angle_velocity

# 自定义的内容：后轮转角 （rad）
float64 rear_wheel_angle
# 自定义的内容：后轮转角速度 (rad/s)
float64 rear_wheel_angle_velocity
```

## Longitudinal.msg
``` bash
# 目标线速度 (m/s)
float64 velocity
# 目标加速度 (m/s^2)
float64 acceleration
# 目标 jerk (m/s^3)
float64 jerk
```

## VehicleStatus.msg
``` bash
# 车辆状态消息
std_msgs/Header header # 时间戳和所处坐标系
string child_frame_id # 车辆自车坐标系名称
geometry_msgs/Pose pose # 车辆位置和姿态 包含position(xyz)和orientation(xyzw)
race_msgs/Euler euler # 车辆姿态 包含俯仰角、横滚角、偏航角
geometry_msgs/Twist vel # 平移旋转速度 包含速度、角速度
geometry_msgs/Twist acc # 平移旋转加速度 包含加速度、角加速度
race_msgs/Lateral lateral # 转向情况：包含前轮转角、角速度 后轮转角、角速度
race_msgs/WheelSpeed wheel_speed # 包含四个车轮的角速度 rad/s
int8 gear # 档位信息 -2：驻车 -1：倒车 0：空挡 1~6：前进档 参见Control.msg
uint8 control_mode # 控制模式，0：油门刹车模式，1：速度模式，2：加速度模式 参见Control.msg
bool hand_brake # 手刹状态
bool emergency # 是否紧急状态
bool clutch # 离合器状态
uint8 steering_mode # 转向模式，1：单轴转向，2：双轴转向 参见Control.msg
```

## WheelSpeed.msg
``` bash
float32 left_front # 单位：rad/s
float32 left_rear # 单位：rad/s
float32 right_front # 单位：rad/s
float32 right_rear # 单位：rad/s
```

## Euler.msg
``` bash
float64 roll # 横滚角
float64 pitch # 俯仰角
float64 yaw # 偏航角
```


## Flag.msg
``` bash
uint8 flag       # 比赛状态标志位
uint8 RED = 0    # 红色旗帜：紧急停止
uint8 GREEN = 1  # 绿色旗帜：正常运行且没有限速
uint8 BLACK = 2  # 黑色旗帜：比赛完成，紧急停止
uint8 G5 = 5     # 绿色旗帜且限速5km/h
uint8 G10 = 10   # 绿色旗帜且限速10km/h
uint8 G15 = 15   # 绿色旗帜且限速15km/h
uint8 G20 = 20   # 绿色旗帜且限速20km/h
uint8 G40 = 40   # 绿色旗帜且限速40km/h
uint8 G60 = 60   # 绿色旗帜且限速60km/h 
uint8 G80 = 80   # 绿色旗帜且限速80km/h
```

这是我自定义的消息包race_msgs当中的内容，我需要你根据这个消息包的格式，写一个HitchOpen-THICV-Stack/src/control/中的ROS1包：race_tracker。这个包中，横纵向控制是解耦的，需要有一个控制器调度主程序节点：controller_manager，还需要有对应的分别的横、纵向控制器作为插件形式存在（可以先写pure_pursuit横向控制器插件和pid_controller纵向控制器插件）。
控制器订阅race_msgs::Path类型的话题/race/local_path(当然可以要在launch文件中配置名称)
订阅race_msgs::VehicleStatus类型的话题/race/vehicle_state(当然可以要在launch文件中配置名称)
发布race_msgs::Control类型的话题/race/control(当然可以要在launch文件中配置名称)
我需要controller_manager节点预先生成一个control类型的消息，默认采用throttle_brake_only模式，向plugin节点传入车辆状态、期望轨迹、控制指令指针，就可以在plugin内自由配置控制指令。后调用的插件就可以修改前面插件的计算结果（如我要再加入一个abs_tcs插件的话，就可以进一步继承这个插件，然后放在pid_controller之后调用以覆盖对于throttle和brake的指令（如有必要））我希望只有一个插件类，所有插件都满足上述描述的接口。


请你给我完整的步骤和代码实现，教我一步步实现这个允许插件存在的代码架构。同时也仔细教我如何撰写匹配controller_manager的新的横纵向控制插件。

同学回答：
我将为您提供一个详细的步骤和代码实现，帮助您构建一个支持插件化横纵向控制器的 ROS 1 包 `race_tracker`。这个架构将允许您轻松地添加、修改或替换不同的控制器，例如您提到的 `pure_pursuit` 和 `pid_controller`。

---

### 整体架构设计

在开始编写代码之前，让我们先梳理一下整体的架构。

* **`race_tracker` ROS 包**: 包含所有控制相关的代码和配置文件。
* **`controller_manager` 节点**:
    * 作为主调度器，负责订阅车辆状态和局部路径，然后将这些数据传递给横向和纵向控制器插件。
    * 创建并管理控制器插件的实例。
    * 发布最终的控制指令。
    * 它将预先生成一个 `race_msgs::Control` 消息，并以指针形式传递给插件，允许插件直接修改控制指令。
* **控制器插件**:
    * 遵循一个统一的接口，确保 `controller_manager` 可以通用地调用它们。
    * 每个插件（例如 `pure_pursuit` 和 `pid_controller`）都将是这个接口的实现。
    * 它们从 `controller_manager` 接收车辆状态、路径数据和控制指令指针，然后执行各自的控制算法，并更新控制指令。

---

### 第一步：创建 ROS 包和目录结构

首先，在您的工作区 `src` 目录下创建 `race_tracker` 包，并设置必要的依赖--

### 整体架构设计

在开始编写代码之前，让我们先梳理一下整体的架构。

  * **`race_tracker` ROS 包**: 包含所有控制相关的代码和配置文件。
  * **`controller_manager` 节点**:
      * 作为主调度器，负责订阅车辆状态和局部路径，然后将这些数据传递给横向和纵向控制器插件。
      * 创建并管理控制器插件的实例。
      * 发布最终的控制指令。
      * 它将预先生成一个 `race_msgs::Control` 消息，并以指针形式传递给插件，允许插件直接修改控制指令。
  * **控制器插件**:
      * 遵循一个统一的接口，确保 `controller_manager` 可以通用地调用它们。
      * 每个插件（例如 `pure_pursuit` 和 `pid_controller`）都将是这个接口的实现。
      * 它们从 `controller_manager` 接收车辆状态、路径数据和控制指令指针，然后执行各自的控制算法，并更新控制指令。

-----

### 第一步：创建 ROS 包和目录结构

首先，在您的工作区 `src` 目录下创建 `race_tracker` 包，并设置必要的依赖。

```bash
cd ~/HitchOpen-THICV-Stack/src
catkin_create_pkg race_tracker roscpp std_msgs race_msgs geometry_msgs pluginlib
mkdir -p race_tracker/include/race_tracker
mkdir -p race_tracker/src
mkdir -p race_tracker/plugins
mkdir -p race_tracker/launch
```

  - **`roscpp`**: ROS C++ 客户端库。
  - **`std_msgs`**, **`geometry_msgs`**, **`race_msgs`**: 消息类型依赖。
  - \*\*\`pluginlib**`pluginlib`**: ROS 提供的用于动态加载插件的库，这是我们实现插件化架构的关键。

-----

### 第二步：定义控制器插件接口

我们需要一个统一的基类（接口），所有控制器插件都必须继承它。这个接口将定义控制器需要实现的方法。

在 \`race\_tracker/include/race`race_tracker/include/race_tracker` 目录下创建文件 `controller_plugin_base.h`：

```cpp
#ifndef RACE_TRACKER_CONTROLLER_PLUGIN_BASE_H
#define RACE_TRACKER_CONTROLLER_PLUGIN_BASE_H

#include <ros/ros.h>
#include <race_msgs/Control.h>
#include <race_msgs/VehicleStatus.h>
#include <race_msgs/Path.h>
#include <memory>

namespace race_tracker {

class ControllerPluginBase {
public:
    // Virtual destructor for proper memory management of derived classes
    virtual ~ControllerPluginBase() {}

    // Method to initialize the plugin with ROS node handle
    virtual void initialize(ros::NodeHandle& nh) = 0;

    // The main control method to be implemented by each plugin.
    // It takes vehicle status and path data, and a pointer to the control message
    // to allow modifications.
    virtual void computeControl(
        const race_msgs::VehicleStatusConstPtr& vehicle_status,
        const race_msgs::PathConstPtr& path,
        race_msgs::Control* control_msg) = 0;

    // Optional: Method to get the name of the plugin
    virtual std::string getName() const = 0;
};

} // namespace race_tracker

#endif // RACE_TRACKER_CONTROLLER_PLUGIN_BASE_H
```

这个接口定义了两个关键方法：

  * `initialize()`: 用于插件的初始化，例如加载参数、创建发布者/订阅者等。
  * `computeControl()`: 核心方法，每个插件都将在这里实现其控制逻辑。它接收车辆状态、路径和指向 `Control` 消息的指针，允许插件直接修改并传递其计算结果。

-----

### 第三步：实现具体的控制器插件

现在，我们可以基于上一步定义的接口实现具体的控制器。

#### 3.1 `PurePursuit` 横向控制器

在 \`race\_tracker`race_tracker/plugins` 目录下创建 `pure_pursuit_controller.cpp`。

```cpp
#include <race_tracker/controller_plugin_base.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

namespace race_tracker {

class PurePursuitController : public ControllerPluginBase {
public:
    PurePursuitController() {}

    void initialize(ros::NodeHandle& nh) override {
        // Load parameters for Pure Pursuit
        nh.param("pure_pursuit/lookahead_distance", lookahead_distance_, 5.0);
        nh.param("pure_pursuit/k_gain", k_gain_, 0.5);
        ROS_INFO_STREAM("PurePursuitController initialized with lookahead: " << lookahead_distance_);
    }

    void computeControl(
        const race_msgs::VehicleStatusConstPtr& vehicle_status,
        const race_msgs::PathConstPtr& path,
        race_msgs::Control* control_msg) override {

        if (path->points.empty()) {
            ROS_WARN_STREAM("Path is empty. Stopping.");
            control_msg->lateral.steering_angle = 0.0;
            return;
        }

        // Find the lookahead point on the path
        geometry_msgs::Point lookahead_point;
        bool found_lookahead = false;
        double min_dist_sq = std::numeric_limits<double>::max();
        
        for (const auto& path_point : path->points) {
            double dx = path_point.pose.position.x - vehicle_status->pose.position.x;
            double dy = path_point.pose.position.y - vehicle_status->pose.position.y;
            double dist_sq = dx*dx + dy*dy;

            // Simple search for the closest point
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
            }

            if (dist_sq > lookahead_distance_ * lookahead_distance_ && !found_lookahead) {
                lookahead_point = path_point.pose.position;
                found_lookahead = true;
            }
        }

        if (!found_lookahead) {
             // If lookahead point not found (e.g., end of path), use the last point
             lookahead_point = path->points.back().pose.position;
        }

        // Transform lookahead point to vehicle frame
        double vehicle_yaw = vehicle_status->euler.yaw;
        double tx = lookahead_point.x - vehicle_status->pose.position.x;
        double ty = lookahead_point.y - vehicle_status->pose.position.y;

        double x_veh = tx * cos(-vehicle_yaw) - ty * sin(-vehicle_yaw);
        double y_veh = tx * sin(-vehicle_yaw) + ty * cos(-vehicle_yaw);
        
        // Calculate steering angle
        double angle_to_point = atan2(y_veh, x_veh);
        double steering_angle = atan2(2.0 * 2.8 * sin(angle_to_point), lookahead_distance_); // 2.8 is assumed wheelbase

        // Update the control message
        control_msg->lateral.steering_angle = steering_angle;
    }

    std::string getName() const override {
        return "PurePursuitController";
    }

private:
    double lookahead_distance_;
    double k_gain_;
};

} // namespace race_tracker

// Register this class as a plugin
PLUGINLIB_EXPORT_CLASS(race_tracker::PurePursuitController, race_tracker::ControllerPluginBase)
```

**代码解释**:

  * `PLUGINLIB_EXPORT_CLASS`: 这是 `pluginlib` 宏，它使得这个类可以被动态加载。第一个参数是完整类名，第二个是基类名。
  * 在 \`computeControl`computeControl` 中，我们首先检查路径是否为空，然后遍历路径点找到一个合适的**预瞄点**。
  * 将预瞄点转换到车辆坐标系。
  * 根据纯追踪算法公式计算目标**转向角**，并更新 `control_msg->lateral.steering_angle`。

#### 3.2 `PID` 纵向控制器

在 `race_tracker/plugins` 目录下创建 `pid_controller.cpp`。

```cpp
#include <race_tracker/controller_plugin_base.h>
#include <pluginlib/class_list_macros.h>

namespace race_tracker {

class PIDController : public ControllerPluginBase {
public:
    PIDController() : last_error_(0.0), integral_error_(0.0) {}

    void initialize(ros::NodeHandle& nh) override {
        // Load PID gains
        nh.param("pid/kp", kp_, 0.5);
        nh.param("pid/ki", ki_, 0.01);
        nh.param("pid/kd", kd_, 0.1);
        nh.param("pid/max_throttle", max_throttle_, 0.8);
        nh.param("pid/max_brake", max_brake_, 0.8);
        ROS_INFO_STREAM("PIDController initialized with Kp: " << kp_ << ", Ki: " << ki_ << ", Kd: " << kd_);
    }

    void computeControl(
        const race_msgs::VehicleStatusConstPtr& vehicle_status,
        const race_msgs::PathConstPtr& path,
        race_msgs::Control* control_msg) override {

        if (path->points.empty()) {
            ROS_WARN_STREAM("Path is empty. Setting throttle/brake to zero.");
            control_msg->throttle = 0.0;
            control_msg->brake = 1.0;
            last_error_ = 0.0;
            integral_error_ = 0.0;
            return;
        }

        // Assume the target speed is the first point's velocity
        double current_vel = vehicle_status->vel.linear.x;
        double target_vel = path->points.front().velocity;

        double error = target_vel - current_vel;
        
        // PID calculation
        integral_error_ += error * (1.0/10.0); // Assuming 10Hz control loop
        double derivative_error = error - last_error_;

        double output = kp_ * error + ki_ * integral_error_ + kd_ * derivative_error;

        last_error_ = error;

        // Apply control output to throttle or brake
        if (output > 0) {
            control_msg->throttle = std::min(output, max_throttle_);
            control_msg->brake = 0.0;
        } else {
            control_msg->throttle = 0.0;
            control_msg->brake = std::min(-output, max_brake_);
        }
    }

    std::string getName() const override {
        return "PIDController";
    }

private:
    double kp_, ki_, kd_;
    double last_error_;
    double integral_error_;
    double max_throttle_;
    double max_brake_;
};

} // namespace race_tracker

// Register this class as a plugin
PLUGINLIB_EXPORT_CLASS(race_tracker::PIDController, race_tracker::ControllerPluginBase)
```

**代码解释**:

  * 与 `PurePursuitController` 类似，使用 \`PLUGINLIB\_EXPORT\_CLASS`PLUGINLIB_EXPORT_CLASS` 注册。
  * 在 `computeControl` 中，我们简单地将路径的第一个点的速度作为目标速度。
  * 计算当前速度与目标速度之间的误差，并使用经典的 PID 公式计算输出。
  * 根据输出的正负，将控制量映射到 `control_msg` 的 \`throttle`throttle` 或 `brake` 字段。

-----

### 第四步：实现 `controller_manager` 节点

这是整个架构的核心。它将动态加载并运行之前定义的插件。

在 `race_tracker/src` 目录下创建 `controller_manager.cpp`。

```cpp
#include <ros/ros.h>
#include <pluginlib/class_loader.h>

#include <race_tracker/controller_plugin_base.h>
#include <race_msgs/Control.h>
#include <race_msgs/Path.h>
#include <race_msgs/VehicleStatus.h>
#include <memory>
#include <vector>

class ControllerManager {
public:
    ControllerManager() : nh_("~") {
        // Create a ClassLoader for our base class
        try {
            plugin_loader_.reset(new pluginlib::ClassLoader<race_tracker::ControllerPluginBase>("race_tracker", "race_tracker::ControllerPluginBase"));
        } catch (pluginlib::PluginlibException& ex) {
            ROS_ERROR("Failed to create plugin class loader. Error: %s", ex.what());
            ros::shutdown();
            return;
        }

        // Get plugin names from ROS parameters
        nh_.getParam("plugins", plugin_names_);
        if (plugin_names_.empty()) {
            ROS_ERROR("No controller plugins specified in 'plugins' parameter. Shutting down.");
            ros::shutdown();
            return;
        }

        // Load each plugin and initialize it
        for (const auto& plugin_name : plugin_names_) {
            try {
                boost::shared_ptr<race_tracker::ControllerPluginBase> plugin_ptr = plugin_loader_->createInstance(plugin_name);
                plugin_ptr->initialize(nh_);
                plugins_.push_back(plugin_ptr);
                ROS_INFO("Successfully loaded and initialized plugin: %s", plugin_name.c_str());
            } catch (pluginlib::PluginlibException& ex) {
                ROS_ERROR("Failed to load plugin '%s'. Error: %s", plugin_name.c_str(), ex.what());
            }
        }
        
        // Subscribers and Publisher
        path_sub_ = nh_.subscribe("/race/local_path", 1, &ControllerManager::pathCallback, this);
        status_sub_ = nh_.subscribe("/race/vehicle_state", 1, &ControllerManager::statusCallback, this);
        control_pub_ = nh_.advertise<race_msgs::Control>("/race/control", 1);
        
        // Initialize control message
        control_msg_.gear = race_msgs::Control::GEAR_1;
        control_msg_.emergency = false;
        control_msg_.hand_brake = false;
        control_msg_.clutch = false;
        control_msg_.control_mode = race_msgs::Control::THROTTLE_BRAKE_ONLY; // Default mode
        control_msg_.steering_mode = race_msgs::Control::FRONT_STEERING_MODE;

        ROS_INFO("Controller Manager is ready.");
    }

private:
    void pathCallback(const race_msgs::PathConstPtr& path_msg) {
        last_path_ = path_msg;
        processControl();
    }

    void statusCallback(const race_msgs::VehicleStatusConstPtr& status_msg) {
        last_status_ = status_msg;
        processControl();
    }
    
    void processControl() {
        if (!last_path_ || !last_status_) {
            return; // Wait for both messages to arrive
        }

        // Create a copy of the path and status messages
        // to avoid concurrency issues if the callbacks are called again
        race_msgs::PathConstPtr path = last_path_;
        race_msgs::VehicleStatusConstPtr status = last_status_;
        
        // Pass the control message pointer to each plugin
        // The order of plugins in the 'plugins' list matters.
        for (const auto& plugin : plugins_) {
            plugin->computeControl(status, path, &control_msg_);
        }

        // Publish the final control message
        control_msg_.header.stamp = ros::Time::now();
        control_pub_.publish(control_msg_);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber status_sub_;
    ros::Publisher control_pub_;

    // Plugin management
    std::unique_ptr<pluginlib::ClassLoader<race_tracker::ControllerPluginBase>> plugin_loader_;
    std::vector<boost::shared_ptr<race_tracker::ControllerPluginBase>> plugins_;
    std::vector<std::string> plugin_names_;

    // Data from subscribers
    race_msgs::PathConstPtr last_path_;
    race_msgs::VehicleStatusConstPtr last_status_;
    race_msgs::Control control_msg_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_manager");
    ControllerManager cm;
    ros::spin();
    return 0;
}
```

**代码解释**:

  * \`pluginlib`pluginlib::ClassLoader`: 这是动态加载插件的核心类。它需要包名和基类名。
  * \`nh\_.getParam("plugins", plugin\_names\_)`nh_.getParam("plugins", plugin_names_)`: 从 ROS 参数服务器获取要加载的插件列表。这使得您无需修改代码即可更改控制器组合。
  * \`plugin\_loader\_-\>create`plugin_loader_->createInstance()`: 根据名称动态创建插件实例。
  * `processControl()`: 当接收到新的路径或状态消息时调用。它遍历所有已加载的插件，并按顺序调用它们的 `computeControl` 方法，将 `control_msg_` 的指针传递给它们。

-----  * **关键点**: 每个插件都将修改同一个 `control_msg_` 实例。因此，**插件的加载顺序至关重要**。例如，纵向控制器在前，横向控制器在后，或者您可以反过来。如果您未来添加一个像 `abs_tcs` 这样的插件，您可以将其放在最后，以确保它能覆盖之前计算的油门和刹车值。

-----

### 第五步：配置 `CMakeLists.txt` 和 `package.xml`

为了让 ROS 知道如何构建这些代码和插件，我们需要修改配置文件。

#### 5.1 修改 \`package.xml`package.xml`

在 `race_tracker/package.xml` 中添加 `pluginlib` 依赖。

```xml
<build_depend>pluginlib</build_depend>
<exec_depend>pluginlib</exec_depend>
```

#### 5.2 修改 `CMakeLists.txt`

在 `race_tracker/CMakeLists.txt` 中添加构建规则。

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(race_tracker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  race_msgs
  geometry_msgs
  pluginlib
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES race_tracker
  CATKIN_DEPENDS roscpp std_msgs race_msgs geometry_msgs pluginlib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

# Build the main controller manager executable
add_executable(controller_manager src/controller_manager.cpp)
target_link_libraries(controller_manager ${catkin_LIBRARIES})

# Build the plugins
add_library(pure_pursuit_controller plugins/pure_pursuit_controller.cpp)
add_library(pid_controller plugins/pid_controller.cpp)

# Link against the base class library
target_link_libraries(pure_pursuit_controller ${catkin_LIBRARIES})
target_link_libraries(pid_controller ${catkin_LIBRARIES})

# Make the plugins known to ROS
install(TARGETS
  controller_manager
  pure_pursuit_controller
  pid_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
```

-----

### 第六步：创建 `plugin_description.xml`

这个文件是 `pluginlib` 动态加载插件所必需的。它告诉 ROS 我们的插件在哪里，以及它们继承哪个基类。

在 `race_tracker` 根目录下创建 \`race`race_tracker_plugins.xml`。

```xml
<library path="lib/libpure_pursuit_controller">
    <class name="race_tracker::PurePursuitController"
           type="race_tracker::PurePursuitController"
           base_class_type="race_tracker::ControllerPluginBase">
        <description>
            A Pure Pursuit controller for lateral control.
        </description>
    </class>
</library>

<library path="lib/libpid_controller">
    <class name="race_tracker::PIDController"
           type="race_tracker::PIDController"
           base_class_type="race_tracker::ControllerPluginBase">
        <description>
            A simple PID controller for longitudinal control.
        </description>
    </class>
</library>
```

-----

### 第七步：创建 `launch` 文件

为了方便运行和配置，创建一个 `race_tracker/launch/controller.launch` 文件。

```xml
<launch>
    <node pkg="race_tracker" type="controller_manager" name="controller_manager" output="screen">
        <rosparam param="plugins">
            - race_tracker::PurePursuitController
            - race_tracker::PIDController
        </rosparam>
        
        <rosparam ns="pid">
            <param name="kp" type="double" value="0.5" />
            <param name="ki" type="double" value="0.01" />
            <param name="kd" type="double" value="0.1" />
        </rosparam>

        <rosparam ns="pure_pursuit">
            <param name="lookahead_distance" type="double" value="5.0" />
            <param name="k_gain" type="double" value="0.5" />
        </rosparam>
        
        </node>
</launch>
```

**关键点**:

  * `<rosparam param="plugins">`: 这个参数列表告诉 \`controller\_manager`controller_manager` 应该加载哪些插件。顺序很重要，因为它决定了 `computeControl` 的调用顺序。
  * `<rosparam ns="...">`: 每个插件的私有参数都可以在这里配置，并通过 `ros::NodeHandle("~")` 在插件的 `initialize` 方法中加载。

-----

### 第八步：编译与运行

现在您可以回到工作区根目录并进行编译。

```bash
cd ~/HitchOpen-THICV-Stack/
catkin_make
```

最后，运行您的控制器：

```bash
roslaunch race_tracker controller.launch
```

如果一切设置正确，您将看到日志输出表明 `controller_manager` 成功加载了 `PurePursuitController` 和 `PIDController`。然后，它将开始订阅 `/race/local_path` 和 `/race/vehicle_state`，并在接收到消息时发布 `/race/control`。


我采用这个方案成功编译成功也运行成功了，但是这位同学显然没有太听明白我的要求，而且实际控制基本上是全都是错乱的，根本看不出一点点受控的痕迹，只知道横冲直撞。我希望你对代码进行改进，着重将原本的pid_controller和pure_pursuit_controller拆分成header和cpp文件，并完善内容。并且这里的controller_manager好像不是按照周期进行控制的，相关参数也都没有可以进行设置。请你帮他改正代码问题。
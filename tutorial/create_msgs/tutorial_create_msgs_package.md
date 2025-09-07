# 创建并使用自定义ROS消息包的完整步骤


## 一、创建消息包
本文档介绍了如何创建和使用自己的ROS消息包。下面以`race_msgs`为例进行介绍。这里我们将在`HitchOpen-THICV-Stack/src/common/`目录下创建`race_msgs`消息包，并实现与Autoware相同的`Control`和`Path`消息类型。

### 1.1 创建消息包步骤概览
总体而言可以分为这样的几个步骤：
1. 创建新软件包，依赖项至少包括`std_msgs`、`roscpp`、`rosmsg`、`message_generation`、`message_runtime`、`rospy`等基础包。
2. 软件包添加**msg目录**，新建自定义消息文件，以<strong>.msg</strong>结尾。
3. 在CMakeLists.txt中，将新建立的<strong>.msg文件</strong>加入到`add_message_files`中。
4. 在CMakeLists.txt中，去掉`generate_messages()`注释符号，将依赖的其他消息包名称添加进去。
5. 在CMakeLists.txt中，将`message_generation`添加到`catkin_package`的`CATKIN_DEPENDS`中。
6. 在package.xml中，将`message_generation`和`message_runtime`添加到`<build_depend>`中和`<exec_depend>`中。
7. 编译消息包，生成新的自定义消息类型。



## 步骤1：创建工作空间和目录结构

首先确保工作空间和目录结构正确：

```bash
# 创建工作空间（如果尚未创建）
mkdir -p HitchOpen-THICV-Stack/src

# 进入src目录并创建common文件夹
cd HitchOpen-THICV-Stack/src
mkdir -p common
cd common
```

## 步骤2：创建race_msgs功能包

```bash
# 创建消息包，依赖roscpp、rospy、rosmsg、std_msgs等基础包
catkin_create_pkg race_msgs std_msgs geometry_msgs roscpp rosmsg message_generation message_runtime rospy 
```

## 步骤3：创建消息文件

进入`race_msgs`包的`msg`目录（如果没有则创建）：

```bash
cd race_msgs
mkdir -p msg
cd msg
```

**创建Control相关消息（对应autoware_control_msgs/msg/Control）**

创建`Longitudinal`文件：
``` bash
touch Longitudinal.msg
```
文件内容：
``` bash
# 目标线速度 (m/s)
float64 velocity
# 目标加速度 (m/s^2)
float64 acceleration
# 目标 jerk (m/s^3)
float64 jerk
```
创建`Lateral`文件：
``` bash
touch Lateral.msg
```
文件内容：
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

创建`Control.msg`文件：

```bash
touch Control.msg
```

文件内容如下：
``` bash
# 控制命令消息，与Autoware的Control消息保持一致
std_msgs/Header header

race_msgs/Longitudinal longitudinal
race_msgs/Lateral lateral

# 自定义的控制内容：油门 （0~1）
float64 throttle
# 自定义的控制内容：刹车 （0~1）
float64 brake 

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


# 紧急制动标志
bool emergency

# 自定义的内容：是否双轴转向
uint8 steering_mode
uint8 FRONT_STEERING_MODE = 1
uint8 DUAL_STEERING_MODE = 2

```

**创建Path相关消息（对应autoware_planning_msgs/msg/Path**

首先创建路径点消息`PathPoint.msg`：

```bash
touch PathPoint.msg
```

文件内容：
``` bash
# 路径点消息
geometry_msgs/PoseStamped pose

# 该点的目标线速度 (m/s)
float64 velocity

# 该点的目标加速度 (m/s^2)
float64 acceleration

# 该点的目标航向角变化率 (rad/s)
float64 heading_rate

# 该点的曲率 (1/m)
float64 curvature


```

然后创建路径消息`Path.msg`：

```bash
touch Path.msg
```

文件内容：
``` bash
# 路径消息，与Autoware的Path消息保持一致
std_msgs/Header header

# 路径点序列
race_msgs/PathPoint[] points  # 通过这种方式可以创建我们定义的其他消息的实例

# 路径是否为临时路径
bool is_avoidance_path
```

## 步骤4：修改package.xml

回到`race_msgs`目录，编辑`package.xml`文件：

```bash
cd ..  # 回到race_msgs目录
gedit package.xml
```

确保包含以下内容（已有的保留，添加缺少的）：

```xml
<package format="2">
  <name>race_msgs</name>
  <version>0.0.0</version>
  <description>The race_msgs package</description>
  
  <!-- 维护者信息 -->
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>MIT</license>

  <!-- 依赖项 -->
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rosmsg</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>message_generation</build_depend>
  
  <exec_depend>roscpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>message_runtime</exec_depend>
  <exec_depend>message_generation</exec_depend>   <!-- 这个要手动添加 -->

  <export>
    <build_type>catkin</build_type>
  </export>
</package>
```

## 步骤5：修改CMakeLists.txt

编辑`CMakeLists.txt`文件：

```bash
gedit CMakeLists.txt
```

内容如下：

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(race_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  message_runtime
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Longitudinal.msg
  Lateral.msg
  Control.msg
  PathPoint.msg
  Path.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)
```

## 步骤6：编译消息包

回到工作空间根目录进行编译：

```bash
cd ../../../../  # 回到HitchOpen-THICV-Stack目录
catkin_make
```

## 步骤7：设置环境变量

编译完成后，需要设置环境变量才能使用新创建的消息包：

```bash
# 在当前终端中生效
source devel/setup.bash

# 为了永久生效，可以将上述命令添加到.bashrc中
echo "source ~/HitchOpen-THICV-Stack/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 步骤8：验证消息包是否创建成功

```bash
# 检查消息是否存在
rosmsg show race_msgs/Control
rosmsg show race_msgs/Path
```

如果能正确显示消息结构，说明消息包创建成功。

### 目录结构总结

完成后，你的目录结构应该是这样的：

```
HitchOpen-THICV-Stack/
├── src/
│   └── common/
│       └── race_msgs/
│           ├── CMakeLists.txt
│           ├── package.xml
│           └── msg/
│               ├── Longitudinal.msg
│               ├── Lateral.msg
│               ├── Control.msg
│               ├── Path.msg
│               └── PathPoint.msg
├── build/
├── devel/
└── ...
```

现在你可以在其他ROS包中通过`#include "race_msgs/msg/control.hpp"`和`#include "race_msgs/msg/path.hpp"`来使用这些消息了，就像使用Autoware的原始消息一样。

### 注意事项

- 确保在使用新创建的消息之前，已经编译并设置了环境变量。
- 可以根据需要在`msg/`目录下添加其他自定义消息文件。
- 编译新的消息包后，需要重新设置环境变量才能生效。

## 二、使用自定义消息
创建并编译完了消息包的下一步，就是在其他ROS包中使用自定义消息。我们分两个部分讲解，分别是在ROS节点的源文件中导入消息包，以及在C++编译的过程中如何配置CMakeList.txt文件。

## 2.1 C++节点步骤总体概览
1. 在**节点代码**中，先**include**消息包的头文件。
2. 在发布或订阅话题的时候，将**话题中的消息类型**设置为自定义消息类型。
3. 按照新的消息结构，对消息包进行**赋值发送**或**读取解析**。
4. 在**CMakeList.txt**中，的`find_package`中，添加新消息报名称作为依赖。
5. 在**CMakeList.txt**中，节点编译规则中，添加一条`add_dependencies()`,将**新消息软件包名称_generate_messages_cpp**作为依赖项。
6. 在**package.xml**中，添加新消息包的依赖到`<build_depend>`和`<exec_depend>`中去。
7. 运行`catkin_make`重新编译。


## 2.1.1 在文件中导入消息包
我们编写好的并编译好的消息包在`HitchOpen-THICV-Stack/devel/include/race_msgs`目录下。在其他ROS包的源文件中，我们可以通过`#include "race_msgs/Control.h"`和`#include "race_msgs/Path.h"`来导入我们定义的消息。
``` c++
// C++导入消息包
#include "race_msgs/Control.h"
#include "race_msgs/Path.h"

// 具体生成消息的时候
race_msgs::Control control_msg;
race_msgs::Path path_msg;

```


这里如果在vscode中进行代码编写，可能会报错头文件找不到的问题，这时候我们就需要打开工作目录下`.vscode`文件夹下面的`c_cpp_properties.json`文件，来手动加入我们消息包所在的位置：
``` json
{
  "configurations": [
    {
      "browse": {
        "databaseFilename": "${default}",
        "limitSymbolsToIncludedHeaders": false
      },
      "includePath": [
        "/opt/ros/noetic/include/**",
        "/home/komasa/catkin_ws/src/ssr_pkg/include/**",
        "/home/komasa/catkin_ws/src/wpr_simulation/include/**",
        "/home/komasa/catkin_ws/src/my_planner/include/**",
        "/usr/include/**",
        "/home/komasa/catkin_ws/src/race_global_planner/include/**",
        "/home/komasa/catkin_ws/src/compass_nav_pkg/include/**",
        "/home/komasa/HitchOpen-THICV-Stack/devel/include/**" // 加入自定义消息包的路径
      ],
      "name": "ROS",
      "intelliSenseMode": "gcc-x64",
      "compilerPath": "/usr/bin/gcc",
      "cStandard": "gnu11",
      "cppStandard": "c++14"
    }
  ],
  "version": 4
}

```

再次提醒别忘了编译的时候所用的终端一定要source一下`HitchOpen-THICV-Stack/devel/setup.bash` ，否则会报错找不到消息包。强烈建议写入`~/.bashrc`中去。


## 2.1.2 在CMakeList.txt中配置依赖

在CMakeList.txt中，我们需要添加新消息包的依赖，以及新消息包的生成规则。
``` cmake
# 找到新消息包
find_package(race_msgs REQUIRED)

# 添加依赖项 比如对global_planning_node节点添加依赖
add_dependencies(global_planning_node race_msgs_generate_messages_cpp)

```
## 2.2 Python节点步骤总体概览

1. 在**节点代码**中，先**import**新定义的消息类型。
2. 在**发布或订阅话题**的时候，将**话题中的消息类型**设置为自定义消息类型。
3. 按照**新的消息结构**，对消息包进行**赋值发送**或**读取解析**。
4. 在**CMakeList.txt**的`find_package`中，添加新消息包的依赖项。
5. 在**package.xml**中，将新消息包的名称添加到`<build_depend>`和`<exec_depend>`中去。
6. 运行`catkin_make`重新编译，确保软件包进入ROS的包列表中。

## 2.2.1 在Python中导入消息包
在Python中导入消息包直接导入就行了，只要编译过应该能正常提示导入正常没啥问题。
``` python
# Python导入消息包
from race_msgs.msg import Control
from race_msgs.msg import Path

# 发布话题
pub = rospy.Publisher('/racer/control', Control, queue_size=10)

# 生成消息
msg = Control()
msg.longitudinal.velocity = 33.3
msg.lateral.steering_angle = 0

# 发布消息
pub.publish(msg)


```
## 2.2.2 在CMakeList.txt中添加依赖项
但是同样需要再CMakeList.txt中添加依赖项，比如对global_planning_node节点添加依赖：
``` cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  race_msgs # 加入自定义消息包的依赖
)
```

同样在**package.xml**中添加依赖项：
``` xml
<build_depend>race_msgs</build_depend>
<exec_depend>race_msgs</exec_depend>
```
# AngleNorm C++ 函数库使用说明文档

## 1. 库简介
`AngleNorm` 是一个轻量级C++函数库，用于计算两个角度之间的**周期性最短距离**，支持标量与标量、标量与向量、向量与向量的角度对比场景。核心特性包括：
- 支持两种角度单位：弧度（`rad`，默认）和角度（`deg`）。
- 可自定义角度周期范围（如默认的 `[-π, π]` 或常用的 `[0, 2π]`）。
- 基于Eigen库实现向量运算，适配ROS（Robot Operating System）环境，可无缝集成到ROS功能包中。
- 内置数值稳定性处理（如角度归一化、避免`acos`返回NaN），确保计算精度。


## 2. 依赖项
使用 `AngleNorm` 库前需确保环境已安装以下依赖：
| 依赖项       | 版本要求       | 用途说明                                  |
|--------------|----------------|-------------------------------------------|
| Eigen库      | 3.3及以上      | 提供向量运算支持（`VectorXd`、`Vector2d`）|
| C++标准      | C++11及以上    | 支持函数重载、匿名函数等特性              |
| ROS（可选）  | Noetic及以上   | 若在ROS环境使用，需适配ROS编译规则        |


## 3. 头文件（`angle_norm.h`）说明
库的接口定义集中在 `angle_norm.h` 中，所有函数封装在 `AngleUtils` 命名空间下，避免命名冲突。

### 3.1 头文件完整代码
```cpp
#ifndef ANGLE_NORM_H
#define ANGLE_NORM_H

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace AngleUtils {

/**
 * @brief 标量-标量版本：计算两个标量角度的周期性最短距离
 * @param angle1 第一个角度（标量，单位由unit指定）
 * @param angle2 第二个角度（标量，单位由unit指定）
 * @param limit 角度周期范围，默认：弧度时为[-M_PI, M_PI]，角度时为[-180, 180]
 * @param unit 角度单位，可选"rad"（弧度，默认）或"deg"（角度）
 * @return 角度间的周期性最短距离（与输入单位一致，非负）
 * @throws std::invalid_argument 若单位无效或周期范围非法（上限≤下限）
 */
double angle_norm(double angle1, double angle2, 
                 const Eigen::Vector2d& limit = Eigen::Vector2d(-M_PI, M_PI), 
                 const std::string& unit = "rad");

/**
 * @brief 标量-向量版本：计算一个标量角度与一个向量角度的周期性最短距离
 * @param angle1 第一个角度（标量，单位由unit指定）
 * @param angle2 第二个角度（Eigen向量，单位由unit指定）
 * @param limit 角度周期范围，默认：弧度时为[-M_PI, M_PI]，角度时为[-180, 180]
 * @param unit 角度单位，可选"rad"（弧度，默认）或"deg"（角度）
 * @return 最短距离向量（长度与angle2一致，非负）
 * @throws std::invalid_argument 若单位无效、周期范围非法或向量维度不匹配
 */
Eigen::VectorXd angle_norm(double angle1, const Eigen::VectorXd& angle2, 
                          const Eigen::Vector2d& limit = Eigen::Vector2d(-M_PI, M_PI), 
                          const std::string& unit = "rad");

/**
 * @brief 向量-标量版本：计算一个向量角度与一个标量角度的周期性最短距离
 * @param angle1 第一个角度（Eigen向量，单位由unit指定）
 * @param angle2 第二个角度（标量，单位由unit指定）
 * @param limit 角度周期范围，默认：弧度时为[-M_PI, M_PI]，角度时为[-180, 180]
 * @param unit 角度单位，可选"rad"（弧度，默认）或"deg"（角度）
 * @return 最短距离向量（长度与angle1一致，非负）
 * @throws std::invalid_argument 若单位无效、周期范围非法或向量维度不匹配
 */
Eigen::VectorXd angle_norm(const Eigen::VectorXd& angle1, double angle2, 
                          const Eigen::Vector2d& limit = Eigen::Vector2d(-M_PI, M_PI), 
                          const std::string& unit = "rad");

/**
 * @brief 向量-向量版本：计算两个同维度向量角度的周期性最短距离（元素级对比）
 * @param angle1 第一个角度（Eigen向量，单位由unit指定）
 * @param angle2 第二个角度（Eigen向量，单位由unit指定，需与angle1同维度）
 * @param limit 角度周期范围，默认：弧度时为[-M_PI, M_PI]，角度时为[-180, 180]
 * @param unit 角度单位，可选"rad"（弧度，默认）或"deg"（角度）
 * @return 最短距离向量（长度与输入向量一致，非负）
 * @throws std::invalid_argument 若单位无效、周期范围非法或向量维度不匹配
 */
Eigen::VectorXd angle_norm(const Eigen::VectorXd& angle1, const Eigen::VectorXd& angle2, 
                          const Eigen::Vector2d& limit = Eigen::Vector2d(-M_PI, M_PI), 
                          const std::string& unit = "rad");

} // namespace AngleUtils

#endif // ANGLE_NORM_H
```

### 3.2 关键接口说明
- **默认参数**：若不指定 `limit` 和 `unit`，库会自动使用默认配置（弧度单位，周期范围 `[-π, π]`），简化常规场景使用。
- **异常处理**：对无效单位（非"rad"/"deg"）、非法周期范围（上限≤下限）、向量维度不匹配等情况，会抛出 `std::invalid_argument` 异常，便于问题排查。


## 4. 核心功能实现原理
库的核心逻辑围绕“角度周期性”展开，关键步骤如下：
1. **单位转换**：若输入单位为角度（`deg`），先将所有角度（`angle1`/`angle2`）和周期范围（`limit`）转换为弧度，统一计算。
2. **角度归一化**：通过 `mod` 运算将角度映射到指定周期范围 `[limit(0), limit(1))`，确保角度在“单个周期内”对比。
3. **周期性距离计算**：  
   核心公式：`wrapped_diff = mod(direct_diff + range_span/2, range_span) - range_span/2`  
   该公式将角度差值映射到 `[-range_span/2, range_span/2]`，取绝对值后即为“周期性最短距离”（避免绕周期一周的长距离）。
4. **单位回转换**：若原始单位为角度，将计算结果转回角度，确保输出与输入单位一致。


## 5. 使用示例
以下示例覆盖常见使用场景，包含普通C++项目和ROS项目的验证代码。


### 5.1 示例1：标量-标量对比（默认弧度单位，范围 `[-π, π]`）
**场景**：验证接近周期边界的角度距离（3.14≈π 与 -π 的距离应接近0）。

```cpp
#include <iostream>
#include <cmath>
#include "angle_norm.h"

int main() {
    try {
        // 1. 定义两个标量角度（3.14≈π，-M_PI为-π）
        double angle1 = 3.14;
        double angle2 = -M_PI;

        // 2. 调用库函数（使用默认参数：弧度，[-π, π]）
        double dist = AngleUtils::angle_norm(angle1, angle2);

        // 3. 输出结果
        std::cout << "示例1：标量-标量（弧度）" << std::endl;
        std::cout << "角度1: " << angle1 << " rad, 角度2: " << angle2 << " rad" << std::endl;
        std::cout << "周期性最短距离: " << dist << " rad（约" << dist * 180 / M_PI << "°）" << std::endl;
        // 预期结果：≈0.001593 rad（因3.14与π的差值约为0.001593）
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
```


### 5.2 示例2：标量-向量对比（默认弧度单位）
**场景**：计算一个标量角度（0 rad）与多个向量角度的距离。

```cpp
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "angle_norm.h"

int main() {
    try {
        // 1. 定义标量角度和向量角度
        double angle_scalar = 0.0; // 标量角度
        Eigen::VectorXd angle_vec(4); // 向量角度（4个元素）
        angle_vec << 0, M_PI, 2*M_PI, 3*M_PI; // 输入角度

        // 2. 调用库函数（标量-向量版本）
        Eigen::VectorXd dist_vec = AngleUtils::angle_norm(angle_scalar, angle_vec);

        // 3. 输出结果
        std::cout << "\n示例2：标量-向量（弧度）" << std::endl;
        std::cout << "标量角度: " << angle_scalar << " rad" << std::endl;
        std::cout << "向量角度: " << angle_vec.transpose() << " rad" << std::endl;
        std::cout << "最短距离向量: " << dist_vec.transpose() << " rad" << std::endl;
        // 预期结果：[0, π, 0, π]（2π和3π归一后分别为0和π）
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
```


### 5.3 示例3：角度单位对比（`deg`，默认范围 `[-180, 180]`）
**场景**：验证180°与-180°的距离（两者在周期中重合，距离接近0）。

```cpp
#include <iostream>
#include <Eigen/Dense>
#include "angle_norm.h"

int main() {
    try {
        // 1. 定义两个角度（角度单位）
        double angle1_deg = 180.0;
        double angle2_deg = -180.0;

        // 2. 调用库函数（指定单位为"deg"，使用默认范围[-180, 180]）
        double dist_deg = AngleUtils::angle_norm(angle1_deg, angle2_deg, Eigen::Vector2d(-180, 180), "deg");

        // 3. 输出结果
        std::cout << "\n示例3：角度单位（deg）" << std::endl;
        std::cout << "角度1: " << angle1_deg << "°, 角度2: " << angle2_deg << "°" << std::endl;
        std::cout << "周期性最短距离: " << dist_deg << "°" << std::endl;
        // 预期结果：≈0°（180°与-180°在[-180, 180]范围中是同一点）
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
```


### 5.4 示例4：自定义周期范围（`[0, 2π]`，弧度）
**场景**：在自定义周期范围 `[0, 2π]` 下，计算0.5 rad与3.5 rad的距离。

```cpp
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include "angle_norm.h"

int main() {
    try {
        // 1. 定义角度和自定义周期范围
        double angle1 = 0.5;
        double angle2 = 3.5;
        Eigen::Vector2d custom_limit(0, 2*M_PI); // 自定义范围：[0, 2π]

        // 2. 调用库函数（指定自定义范围）
        double dist_custom = AngleUtils::angle_norm(angle1, angle2, custom_limit);

        // 3. 输出结果
        std::cout << "\n示例4：自定义周期范围（[0, 2π]）" << std::endl;
        std::cout << "角度1: " << angle1 << " rad, 角度2: " << angle2 << " rad" << std::endl;
        std::cout << "自定义范围: [" << custom_limit(0) << ", " << custom_limit(1) << "] rad" << std::endl;
        std::cout << "周期性最短距离: " << dist_custom << " rad" << std::endl;
        // 预期结果：≈3.0 rad（直接差值3.0，小于周期跨度2π≈6.28的一半，故最短距离为3.0）
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
```


### 5.5 示例5：ROS环境下的验证（集成到ROS功能包）
**场景**：在ROS Noetic环境中，将 `AngleNorm` 库集成到 `race_global_static_planner` 功能包，通过 `angle_demo` 节点验证功能。

#### 5.5.1 ROS功能包结构
``` bash
HitchOpen-THICV-Stack/
├─ src/
│  └─ planning/
│     └─ race_global_static_planner/  # ROS功能包
│        ├─ src/
│        │  ├─ angle_norm.cpp        # 库实现文件
│        │  ├─ main.cpp              # 示例5的验证代码（即示例1-4的整合）
│        │  └─ ...（其他功能文件）
│        ├─ include/
│        │  └─ race_global_static_planner/
│        │     └─ angle_norm.h       # 库头文件
│        └─ CMakeLists.txt           # ROS编译配置
```

#### 5.5.2 CMakeLists.txt关键配置（ROS）
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(race_global_static_planner)

# 找到依赖（Eigen + ROS核心）
find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)

# 声明头文件路径
catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# 1. 编译AngleNorm库（供其他模块复用）
add_library(race_global_static_planner
  src/angle_norm.cpp
  # ...（其他功能文件，如curvature.cpp）
)

target_link_libraries(race_global_static_planner
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# 2. 编译验证节点angle_demo
add_executable(angle_demo
  src/main.cpp  # 示例1-4的整合代码
)

target_link_libraries(angle_demo
  race_global_static_planner  # 链接AngleNorm库
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
```

#### 5.5.3 ROS环境下运行验证
1. 编译功能包：
   ```bash
   cd ~/HitchOpen-THICV-Stack
   catkin_make --cmake-args -DCATKIN_WHITELIST_PACKAGES="race_global_static_planner"
   ```
2. 加载ROS环境：
   ```bash
   source devel/setup.bash
   ```
3. 运行验证节点：
   ```bash
   rosrun race_global_static_planner angle_demo
   ```
4. 预期输出：
   终端会依次打印示例1-4的结果，无报错即说明库功能正常。


## 6. 编译与运行指南
### 6.1 场景1：普通C++项目（非ROS）
#### 编译命令（g++）
假设所有文件（`angle_norm.h`、`angle_norm.cpp`、`main.cpp`）在同一目录：
```bash
# 编译命令：g++ 源代码文件 -o 可执行文件名 -I Eigen头文件路径 -std=c++11
g++ main.cpp angle_norm.cpp -o angle_demo -I /usr/include/eigen3 -std=c++11
```
- `-I /usr/include/eigen3`：指定Eigen头文件路径（需根据实际安装路径调整）。
- `-std=c++11`：指定C++标准（至少C++11）。

#### 运行命令
```bash
./angle_demo  # Linux/macOS
# 或 angle_demo.exe  # Windows（MinGW环境）
```


### 6.2 场景2：ROS项目（如示例5）
#### 编译命令
```bash
# 进入ROS工作空间
cd ~/HitchOpen-THICV-Stack
# 仅编译目标功能包（速度快）
catkin_make --cmake-args -DCATKIN_WHITELIST_PACKAGES="race_global_static_planner"
```

#### 运行命令
```bash
# 加载ROS环境（每次新终端需执行，或添加到~/.bashrc永久生效）
source devel/setup.bash
# 运行验证节点
rosrun race_global_static_planner angle_demo
```

## 8. 注意事项
1. **单位一致性**：输入角度的单位必须与 `unit` 参数一致（如 `unit="deg"` 时，输入需为角度值），否则计算结果会严重偏差。
2. **周期范围合法性**：`limit` 必须满足“下限 < 上限”（如 `[0, 2π]` 合法，`[2π, 0]` 非法），否则会抛出异常。
3. **向量维度匹配**：向量-向量对比时，`angle1` 和 `angle2` 必须同维度，否则会抛出维度不匹配异常。
4. **数值稳定性**：库已处理 `acos` 输入超出 `[-1, 1]` 的情况（通过 `std::max`/`std::min` 限制），无需额外处理。
项目目录：
komasa@KLAPTOP:~/HitchOpen-THICV-Stack/src/planning/race_global_static_planner$ 

# 全局规划轨迹数据格式说明（ENU坐标系）
本文档通过两个Markdown表格说明全局规划轨迹的数据格式，数据最终以CSV格式存储，坐标系采用**东北天（ENU）坐标系**，用于描述给定偏移下的全局规划轨迹信息。


## 1. 全局规划轨迹数据格式说明表
该表格详细定义了数据文件中每一行、每一列的字段含义、数据类型及约束条件，便于理解数据逻辑。

| 表格行类型       | 列序号 | 字段名称               | 字段含义                                                                 | 数据类型   | 取值范围/特殊说明                                                                 |
|------------------|--------|------------------------|--------------------------------------------------------------------------|------------|-----------------------------------------------------------------------------------|
| 表头行（元数据） | 1      | 固定标识               | 用于标识数据格式的固定值，不可修改                                       | 整数       | 固定为 `10`                                                                       |
| 表头行（元数据） | 2      | 有效路径点个数         | 描述当前轨迹中包含的有效轨迹点总数（与列3值相同，用于数据校验）           | 整数       | 非负整数（示例中为 `53700`）                                                     |
| 表头行（元数据） | 3      | 有效路径点个数（重复） | 重复列2的“有效路径点个数”，用于提升数据传输/存储的容错性                 | 整数       | 与列2值完全一致（示例中为 `53700`）                                               |
| 表头行（元数据） | 4      | 原点纬度               | 轨迹坐标系（ENU）原点的地理纬度（WGS84坐标系）                            | 浮点数     | 范围：-90°~90°（示例中为 `39.7932`，单位：度）                                   |
| 表头行（元数据） | 5      | 原点经度               | 轨迹坐标系（ENU）原点的地理经度（WGS84坐标系）                            | 浮点数     | 范围：-180°~180°（示例中为 `-86.2389`，单位：度）                                |
| 表头行（元数据） | 6      | 原点海拔               | 轨迹坐标系（ENU）原点的地理海拔（WGS84坐标系）                            | 浮点数     | 无固定范围（示例中为 `0.266973`，单位：米）                                       |
| 数据行（轨迹点） | 1      | 原点偏移X坐标          | 轨迹点相对于ENU坐标系原点的**东向（E）** 偏移量                           | 浮点数     | 无固定范围（单位：米）                                                           |
| 数据行（轨迹点） | 2      | 原点偏移Y坐标          | 轨迹点相对于ENU坐标系原点的**北向（N）** 偏移量                           | 浮点数     | 无固定范围（单位：米）                                                           |
| 数据行（轨迹点） | 3      | 原点偏移Z坐标          | 轨迹点相对于ENU坐标系原点的**天向（U）** 偏移量                           | 浮点数     | 无固定范围（单位：米）                                                           |
| 数据行（轨迹点） | 4      | 参考航向角             | 轨迹点处的期望航向角，以ENU坐标系东向为0°，逆时针为正                     | 浮点数     | 范围：`-π ~ π`（单位：弧度）                                                      |
| 数据行（轨迹点） | 5      | 期望车速               | 轨迹点处的期望行驶速度                                                     | 浮点数     | 非负整数（单位：米/秒，示例中采用匀速设计）                                       |
| 数据行（轨迹点） | 6      | 轨迹点曲率             | 轨迹点处的路径曲率，曲率=1/半径（曲率为正表示左转弯，负表示右转弯）         | 浮点数     | 无固定范围（单位：1/米，示例中采用平缓轨迹设计，曲率值较小）                     |


## 2. 全局规划轨迹数据示例表（ENU坐标系）
该表格为符合上述格式的示例数据，包含1行元数据（表头行）和9行轨迹点数据，后续可根据实际轨迹点个数扩展数据行，最终可直接导出为CSV文件使用。

| 固定标识 | 有效路径点个数 | 有效路径点个数（重复） | 原点纬度（度） | 原点经度（度） | 原点海拔（米） |
|----------|----------------|------------------------|----------------|----------------|----------------|
| 10       | 53700          | 53700                  | 39.7932        | -86.2389       | 0.266973       |
|----------|----------------|------------------------|----------------|----------------|----------------|
| X偏移（米） | Y偏移（米） | Z偏移（米） | 参考航向角（rad） | 期望车速（m/s） | 轨迹点曲率（1/米） |
| 969.8734082 | -1865.704287 | 0 | 0.50 | 20.0 | 0.001 |
| 969.7111365 | -1865.587377 | 0 | 0.49 | 20.0 | 0.001 |
| 969.5488648 | -1865.470467 | 0 | 0.48 | 20.0 | 0.001 |
| 969.3865931 | -1865.353557 | 0 | 0.47 | 20.0 | 0.001 |
| 969.2243215 | -1865.236648 | 0 | 0.46 | 20.0 | 0.001 |
| 969.0620498 | -1865.119738 | 0 | 0.45 | 20.0 | 0.001 |
| 968.8997781 | -1865.002828 | 0 | 0.44 | 20.0 | 0.001 |
| 968.7375064 | -1864.885918 | 0 | 0.43 | 20.0 | 0.001 |


## 3. CSV存储说明
上述数据示例表可直接转换为CSV文件，CSV文件的**表头顺序**与“数据格式说明表”的“字段名称”对应，具体CSV表头及首行数据如下（示例）：
```csv
固定标识,有效路径点个数,有效路径点个数（重复）,原点纬度（度）,原点经度（度）,原点海拔（米）
10,53700,53700,39.7932,-86.2389,0.266973
969.8734082,-1865.704287,0,0.50,20.0,0.001
969.7111365,-1865.587377,0,0.49,20.0,0.001
969.5488648,-1865.470467,0,0.48,20.0,0.001
```
CSV文件中无需保留Markdown表格的合并/分隔行，仅需按“元数据行→轨迹点数据行”的顺序存储所有数据即可。


# Curvature 曲率计算库使用说明

## 1. 库简介

Curvature是一个用于计算二维轨迹点曲率的C++库，基于Eigen库实现向量运算，能够高效计算轨迹上各点的曲率值，适用于路径规划、机器人导航等领域。

## 2. 依赖项

- Eigen库：用于向量运算，需要在项目中配置Eigen库路径

## 3. 类与方法说明

### 3.1 Curvature类

该类提供轨迹曲率计算的核心功能，包含以下公共方法：

| 方法 | 说明 |
|------|------|
| `std::vector<double> curvature(const std::vector<Eigen::Vector2d>& path)` | 计算轨迹上各点的曲率，返回与轨迹点数量相同的曲率向量 |

私有辅助方法：
- `rotate(const Eigen::Vector2d& vec0, double th)`：将向量按指定角度旋转
- `distance(const Eigen::Vector2d& x1, const Eigen::Vector2d& x2)`：计算两点间的欧氏距离

## 4. 使用示例

### 4.1 基本使用示例

以下是一个简单的使用示例，展示如何创建轨迹点并计算其曲率：

```cpp
#include <iostream>
#include <vector>
#include "curvature.h"
#include <Eigen/Dense>

int main() {
    try {
        // 创建Curvature对象
        Curvature curvatureCalculator;
        
        // 1. 创建示例轨迹点 - 圆形轨迹
        std::vector<Eigen::Vector2d> path;
        const double radius = 10.0;  // 圆半径
        const int numPoints = 10;    // 点数量
        
        for (int i = 0; i < numPoints; ++i) {
            double angle = 2 * M_PI * i / (numPoints - 1);  // 0到2π的角度
            double x = radius * cos(angle);
            double y = radius * sin(angle);
            path.emplace_back(x, y);
        }
        
        // 2. 计算曲率
        std::vector<double> curvatures = curvatureCalculator.curvature(path);
        
        // 3. 输出结果
        std::cout << "轨迹点数量: " << path.size() << std::endl;
        std::cout << "曲率计算结果:" << std::endl;
        std::cout << "索引\tX坐标\tY坐标\t曲率值\t半径估计" << std::endl;
        
        for (size_t i = 0; i < path.size(); ++i) {
            double radiusEstimate = (curvatures[i] != 0) ? 1.0 / fabs(curvatures[i]) : INFINITY;
            printf("%zu\t%.2f\t%.2f\t%.6f\t%.2f\n", 
                   i, 
                   path[i].x(), 
                   path[i].y(), 
                   curvatures[i],
                   radiusEstimate);
        }
    }
    catch (const std::exception& e) {
        std::cerr << "发生错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
```

### 4.2 从文件读取轨迹点并计算曲率

```cpp
#include <iostream>
#include <fstream>
#include <vector>
#include "curvature.h"
#include <Eigen/Dense>

// 从CSV文件读取轨迹点
bool readPathFromCSV(const std::string& filename, std::vector<Eigen::Vector2d>& path) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false;
    }
    
    std::string line;
    // 跳过表头
    std::getline(file, line);
    
    while (std::getline(file, line)) {
        size_t commaPos = line.find(',');
        if (commaPos == std::string::npos) continue;
        
        try {
            double x = std::stod(line.substr(0, commaPos));
            double y = std::stod(line.substr(commaPos + 1));
            path.emplace_back(x, y);
        }
        catch (...) {
            std::cerr << "解析行失败: " << line << std::endl;
        }
    }
    
    return true;
}

int main() {
    try {
        // 创建Curvature对象
        Curvature curvatureCalculator;
        
        // 1. 从文件读取轨迹点
        std::vector<Eigen::Vector2d> path;
        if (!readPathFromCSV("trajectory.csv", path)) {
            return 1;
        }
        
        // 检查轨迹点数量
        if (path.size() < 3) {
            std::cerr << "轨迹点数量不足，至少需要3个点" << std::endl;
            return 1;
        }
        
        // 2. 计算曲率
        std::vector<double> curvatures = curvatureCalculator.curvature(path);
        
        // 3. 将结果写入文件
        std::ofstream outFile("curvature_results.csv");
        if (outFile.is_open()) {
            outFile << "x,y,curvature" << std::endl;
            for (size_t i = 0; i < path.size(); ++i) {
                outFile << path[i].x() << "," 
                        << path[i].y() << "," 
                        << curvatures[i] << std::endl;
            }
            outFile.close();
            std::cout << "曲率计算完成，结果已保存到 curvature_results.csv" << std::endl;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "发生错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
```



## 5. 注意事项

1. **输入要求**：轨迹点数量必须不少于3个，否则会抛出`invalid_argument`异常
2. **边界处理**：第一个点和最后一个点的曲率值分别使用第二个点和倒数第二个点的曲率值
3. **曲率含义**：
   - 曲率值的绝对值越大，表示曲线弯曲程度越大
   - 正曲率表示向左转弯，负曲率表示向右转弯
   - 曲率的倒数为曲率半径
4. **数值稳定性**：库内部已处理可能的数值不稳定问题（如确保`cosA`在[-1, 1]范围内）
5. **单位**：曲率单位为1/长度单位（与输入点的坐标单位一致）

## 6. 编译说明

编译时需要链接Eigen库，示例CMakeLists.txt片段：

```cmake
cmake_minimum_required(VERSION 3.10)
project(curvature_example)

set(CMAKE_CXX_STANDARD 11)

# 寻找Eigen库
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# 添加可执行文件
add_executable(curvature_demo main.cpp curvature.cpp)
```


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


# 目前准备好的文件有：
1. `curvature.h`：包含曲率计算的类定义
2. `curvature.cpp`：包含曲率计算的实现
3. `angle_norm.h`：包含角度归一化的类定义
4. `angle_norm.cpp`：包含角度归一化的实现
5. `CMakeLists.txt`：用于编译项目的CMake配置文件

# 待完成：
1. `race_global_static_planner.h`：
``` cpp
#ifndef RACE_GLOBAL_STATIC_PLANNER_H
#define RACE_GLOBAL_STATIC_PLANNER_H

#include <ros/ros.h>
// 待补充


#endif // RACE_GLOBAL_STATIC_PLANNER_H
```

2. `race_global_static_planner.cpp`：
``` cpp
// 引入头文件
#include "race_global_static_planner.h"
#include <tf/tf.h> // tf相关的头文件
#include <tf/transform_listener.h> // 自动监听系统中的tf话题，可以调用这里提供的函数完成坐标系转换
#include <tf/transform_datatypes.h> // 提供一些坐标系转换的函数
#include <opencv2/highgui/highgui.hpp> //显示图片窗口
#include <opencv2/imgproc/imgproc.hpp> //绘制图片
#include "curvature.h"
#include <race_msgs/Path.h>
#include <race_msgs/PathPoint.h>
#include <race_msgs/Euler.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <race_msgs/Control.h>
#include <race_msgs/VehicleStatus.h>
#include <race_msgs/Lateral.h>
#include <race_msgs/Longitudinal.h>
#include <race_msgs/WheelSpeed.h>
#include "angle_norm.h"
// 待补充
```

我希望根据以上文件，请你帮我完成`race_global_static_planner.h`和`race_global_static_planner.cpp`的编写。
静态规划器的实现逻辑是：

读取全局路径csv文件，默认保存在$(find race_global_static_planner)/config/corsa.csv中，文件格式如上所述。
你需要将这个全局路径发布为/race/global_path的话题，其类型为race_msgs/Path，所以这个path当中自然要包含曲率信息、航向角信息和期望速度信息，这些都来自于csv文件，可以直接赋值。race_global_static_planner的相关参数应该在$(find race_global_static_planner)/config/params.yaml文件中配置，这个文件还没有请你也帮我生成。
/race/global_path的发布频率就是一个可以设置的参数，默认为1Hz。同时可以选择是否加入可视化，默认为true,如果可视化就需要同时再发布一个/race/global_path_vis的话题，其类型为nav_msgs/Path，可视化的内容就是全局路径, 自然就只包含每个点的位姿信息，其他类似速度、曲率等信息无法可视化就不需要了。

vehicle status中的位置坐标已经转换到了enu坐标系下面，所以你不需要做状态转换，也不需要关注gnss相关话题。

同时还需要发布一个局部路径规划以及可选的可视化话题。/race/local_path和/race/local_path_vis，类型描述同上,同样可以选择是否加入可视化和发布频率。
根据当前车辆的x y z坐标和航向角，找到全局路径上最近的一个点，作为当前车辆的参考点。航向角计算距离这里可以调用现有的自定义angle_norm函数库，在上面已经介绍具体用法。我建议可以添加参数来定义x y z heading在计算距离中的权重，这样方便调试。（一般x y是等权重的，当时也可以和分开（建议，增加调整自由度）），所有权重默认为1 1 0.5 0.2。找到最近点以后向后找满足给定距离的所有点（参数），也向前找到满足给定距离的所有点（参数），（这个向前向后的给定距离是真的沿着路径的距离，也别想太复杂，就是点到点的距离的累加）最终将从身后到前面满足要求的所有点作为一个局部路径输出。全局路径读取后建议就放在函数中作为一个全局变量，每次只需要从中复制相应内容到局部路径既可以。

下面请你帮我完成`race_global_static_planner.h`和`race_global_static_planner.cpp`的编写。



# HitchOpen-THICV-Stack 清华大学智能网联汽车课题组Komasa的自动驾驶软件栈

## 1. 项目介绍
Hello，我是Komasa Qi，这个仓库主要是整理了关于液罐车小车平台和HitchOpen世界AI竞速锦标赛中相关代码的一个集成仓库，包含了感知决策规划控制相关代码的完整部署。里面也有很多我的相关笔记来备忘，方便在新机器上快速部署相关算法。

**系统推荐配置：**
- 操作系统：Ubuntu 20.04 64位 x86架构
- ROS版本：Noetic
- 处理器：Intel i7-11800H 及以上
- 内存：16GB 及以上
- 显卡：RTX 3080 及以上
- 存储：512GB SSD 及以上

相关仿真是跑在CARLA 0.9.13版本上的，仿真桥相关代码在`src/simulation/carla_race_bridge`目录下。[Ubuntu 20.04 ROS Noetic安装Carla 0.9.13 Linux编译版笔记](/tutorial/install_carla/Linux安装Carla编译版教程-小晶.docx)

实车代码一方面是运行在我们的液罐车模型小车上，这是一辆半挂式液罐车，另一方面会运行在HitchOpen天门山下山赛的PIX Hooke底盘上，这辆小车是四轮转向的，所以相关代码自定义消息中预留了后轮转向的控制接口和状态反馈。

## 2. 项目结构
本项目如上所述包含了感知决策规划控制相关代码，同时也包含了一些相关的笔记。项目结构如下：
``` cmake
HitchOpen-THICV-Stack/
├── README.md
├── tutorial/  # 相关笔记
├── src/       # 代码目录
|   ├── common/          # 通用代码
|   |   ├── race_msgs/                    # 赛车相关自定义话题
|   |   ├── race_steering/                # 赛车命令行手动控制
|   ├── perception/      # 感知算法
|   ├── localization/    # 定位算法

|   ├── mapping/         # 地图构建
|   |   ├── liorf/                        # LIO-SAM改进版
|   |   ├── lio-sam/                      # LIO激光惯性里程计建图
|   ├── decision/        # 决策算法
|   ├── planning/        # 规划算法
|   ├── control/         # 控制算法
|   ├── simulation/      # 仿真相关
|   |   ├── carla_race_state_converter/   # 状态转换代码
|   |   ├── carla_race_msgs_to_control/   # 控制指令转换代码
|   ├── launch/          # 启动文件目录
|   |   ├── carla_race_bridge/   # 仿真桥启动文件
├── config/    # 配置文件目录
├── launch/    # 启动文件目录
├── scripts/   # 一些脚本
└── ...
```

## 3. 安装运行

首先下载本项目
``` bash
cd ~
git clone https://github.com/KomasaQi/HitchOpen-THICV-Stack.git
```

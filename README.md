# HitchOpen-THICV-Stack 清华大学智能网联汽车课题组自动驾驶软件栈


## 1. 项目介绍
Hello，我是Komasa Qi，这个仓库主要是整理了关于液罐车小车平台和HitchOpen世界AI竞速锦标赛中相关代码的一个集成仓库，包含了感知决策规划控制相关代码的完整部署。里面也有很多我的相关笔记来备忘，方便在新机器上快速部署相关算法。

**系统推荐配置：**
- 操作系统：Ubuntu 20.04 64位 x86架构 
- ROS版本：Noetic
- 内存：16GB 及以上
- 显卡：RTX 3080 及以上
- 存储：256 GB SSD 及以上

相关仿真是跑在CARLA 0.9.13版本上的，仿真桥相关代码在`src/simulation/carla_race_bridge`目录下。[Ubuntu 20.04 ROS Noetic安装Carla 0.9.13 Linux编译版笔记](/tutorial/install_carla/Linux安装Carla编译版教程-小晶.docx)

实车代码一方面是运行在我们的液罐车模型小车上，这是一辆半挂式液罐车，另一方面会运行在HitchOpe山下山赛的PIX Hooke底盘上，这辆小车是四轮转向的，所以相关代码自定义消息中预留了后轮转向的控制接口和状态反馈。

## 2. 项目结构
本项目如上所述包含了感知决策规划控制相关代码，同时也包含了一些相关的笔记。项目结构如下：
``` 
HitchOpen-THICV-Stack/
├── README.md
├── tutorial/  # 相关笔记
├── src/       # 代码目录
|   ├── common/          # 通用代码
|   |   ├── race_msgs/                    # 赛车相关自定义话题
|   |   └── race_steering/                # 赛车命令行手动控制
|   ├── perception/      # 感知算法
|   ├── localization/    # 定位算法
|   |   └── liorf_localization/           # 基于liorf的定位算法
|   ├── mapping/         # 地图构建
|   |   ├── liorf/                        # LIO-SAM改进版
|   |   └── lio-sam/                      # LIO激光惯性里程计建图
|   ├── decision/        # 决策算法
|   ├── planning/        # 规划算法
|   |   ├── race_global_static_planner/   # 全局静态规划器
|   |   └── race_tracker/                 # 路径跟踪控制器
|   ├── control/         # 控制算法
|   |   ├── go2_control/                  # casadi构建控制器示例
|   |   └── race_tracker/                 # 轨迹跟踪控制器
|   |       └── plugins/                  # 轨迹跟踪控制器插件
|   |           ├── nmpc_controller.cpp   # 双轴转向横纵向耦合NMPC插件
|   |           ├── pure_pursuit.cpp      # 纯跟踪横向控制
|   |           └── pid_controller.cpp    # PID纵向控制器插件
|   ├── simulation/      # 仿真相关
|   |   ├── carla_race_state_converter/   # 状态转换代码
|   |   └── carla_race_msgs_to_control/   # 控制指令转换代码
|   ├── launch/          # 启动文件目录
|   |   └── carla_race_bridge/   # 仿真桥启动文件
├── config/    # 配置文件目录
├── launch/    # 启动文件目录
├── scripts/   # 一些脚本
└── ...
```
## 3. 算法说明
本仓库涉及的算法下面将一一说明

### 3.1 建图与定位算法


### 3.2 环境感知算法


### 3.3 行为决策算法


### 3.4 轨迹规划算法


### 3.5 运动控制算法


## 4. 安装运行

首先下载本项目
``` bash
cd ~
git clone https://github.com/KomasaQi/HitchOpen-THICV-Stack.git
```

### 4.1 比赛计时器 Competition Timer
`competition_timer`是控制代码运行的必要前提，为车辆运行提供安全保障，在运行前务必进行相关设置。

手动更改比赛计时器的旗帜状态为GREEN：
``` bash
rosparam set /competition_timer/flag GREEN
```

## Contributors:
- 戚笑景 Komasa Qi （清华大学）
- 何瑞坤 He Ruikun （辽宁工业大学）
- 邹恒多 Zou Hengduo （清华大学）
- 扶尚宇 Fu Shangyu （清华大学）
- 邱逸凡 Qiu Yifan （中国农业大学）
- 黄梓谦 Huang Ziqian （华南理工大学）
- 冷佳桐 Leng Jiatong （清华大学）
- 李珂 Li Ke （重庆大学） 
- 蒋涛 Jiang Tao （重庆大学）

感谢以上所有参与项目的贡献者，我们的项目是一个开源项目，欢迎所有的贡献者参与进来。
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

## 2. 项目结构
本项目如上所述包含了感知决策规划控制相关代码，同时也包含了一些相关的笔记。项目结构如下：
``` cmake
HitchOpen-THICV-Stack/
├── README.md
├── tutorial/  # 相关笔记
├── src/       # 代码目录
|   ├── common/          # 通用代码
|   ├── perception/      # 感知算法
|   ├── localization/    # 定位算法
|   ├── decision/    # 决策算法
|   ├── planning/    # 规划算法
|   ├── control/     # 控制算法
|   ├── simulation/  # 仿真相关
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

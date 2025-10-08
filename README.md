# HitchOpen-THICV-Stack : **清华大学智能网联汽车课题组自动驾驶软件栈**


## 1. 项目介绍
Hello，我是Komasa Qi，这个仓库主要是整理了关于液罐车小车平台和HitchOpen世界AI竞速锦标赛中相关代码的一个集成仓库，包含了感知决策规划控制相关代码的完整部署。同时也收录了关于Onsite自动驾驶挑战赛第六赛道我们提出的多轴转向车辆自适应转向中心控制相关代码，以及昆明机场项目多节挂车相关项目。里面也有很多我的相关笔记来备忘，方便在新机器上快速部署相关算法。

**系统推荐配置：**
- 操作系统：Ubuntu 20.04 64位 x86架构 
- ROS版本：Noetic 
- cmake 版本：3.16.3
- 内存：16GB 及以上
- 显卡：RTX 3080 及以上
- 存储：256 GB SSD 及以上

![HitchOpen-THICV-Stack](/tutorial/images/HitchOpen_THICV_Stack.png)

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
|   |   ├── race_msgs/                            # 赛车相关自定义话题
|   |   └── race_steering/                        # 赛车命令行手动控制
|   ├── perception/      # 感知算法
|   |   └── patchworkpp/                          # 路面与非路面点云分割
|   ├── localization/    # 定位算法
|   |   └── liorf_localization/                   # 基于liorf的定位算法
|   ├── mapping/         # 地图构建
|   |   ├── liorf/                                # LIO-SAM改进版
|   |   └── lio-sam/                              # LIO激光惯性里程计建图
|   ├── planning/        # 规划算法
|   |   ├── race_global_static_planner/           # 全局静态规划器
|   |   └── race_tracker/                         # 路径跟踪控制器
|   ├── control/         # 控制算法
|   |   ├── go2_control/                          # casadi构建控制器示例
|   |   └── race_tracker/                         # 轨迹跟踪控制器
|   |       └── plugins/                          # 轨迹跟踪控制器插件
|   |           ├── nmpc_controller.cpp           # 双轴转向横纵向耦合NMPC插件
|   |           ├── nmpc_lateral_controller.cpp   # 双轴转向横向NMPC插件
|   |           ├── pure_pursuit.cpp              # 纯跟踪横向控制
|   |           └── pid_controller.cpp            # PID纵向控制器插件
|   ├── simulation/      # 仿真相关
|   |   ├── carla_race_state_converter/           # 状态转换代码
|   |   └── carla_race_msgs_to_control/           # 控制指令转换代码
|   ├── implementation/  # 仿真相关
|   |   ├── race_connector/                       # 远程电脑连接
|   |   ├── messages/                             # PIX底盘各种消息包
|   |   ├── pix_driver/                           # PIX底盘驱动
|   |   ├── pix_molde/                            # PIX底盘rivz可视化模型
|   |   ├── pix_race_bridge/                      # PIX底盘与本项目的桥接
|   |   └── sensors/                              # 传感器相关代码 gnss rplidar
|   └── launch/          # 启动文件目录
|       └── carla_race_bridge/                    # 仿真桥启动文件
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
所有运动控制算法都包含在`race_tracker`包的`controller_manager`中，任何控制算法都满足`controller_plugin_base.h`当中定义的插件接口形式，以插件存在并被按照相关launch文件中设定的顺序被调用。通过这种架构可以方便地实现横纵向控制器解耦或耦合的方案共存，也可以很容易地通过插件调用的顺序设置来覆盖前一个插件的命令，如进行ABS/TCS相关操作等。所有相关插件在`race_tracker/plugins`目录下，新撰写的插件也应该符合对应的接口，从而方便地进行拓展。与MPC相关的部分推荐通过CasADi来构建，一些示例性代码在`/src/control/go2_control`软件包下实现。
- **1.nmpc_controller ----横纵向耦合运动学双轴转向轨迹跟踪控制器**
- **2.nmpc_lateral_controller ----横向运动学双轴转向NMPC控制器**
- **3.pure_pursuit ----纯跟踪横向控制器**
- **4.pid_controller ----PID纵向控制器**


### 3.6 比赛计时器 Competition Timer
`competition_timer`是控制代码运行的必要前提，为车辆运行提供安全保障，在运行前务必进行相关设置。整个程序运行的流程可以用状态机描述：）

**等待比赛开始** --> **进入起点范围** --> **在跑圈中** --> **比赛结束**

可以通过设置launch文件来调整比赛是否为跑圈制，还是单纯跑到终点就算结束。可以在launch文件中设置起终点（终点参数只在非跑圈模式下生效）和圈数等参数，然后可以启动比赛计时器launch文件。

启动比赛计时器：
``` bash
roslaunch competition_timer competition_timer.launch # 可以自行设置其他参数
```

手动更改比赛计时器的旗帜状态为GREEN：
``` bash
rosparam set /competition_timer/flag GREEN # 设置比赛状态，可选：GREEN RED BLACK G5 G10 G15 G20 G40 G60 G80
```

- `RED` : 紧急状态，车辆应立即停止运行。
- `BLACK` : 比赛结束状态，车辆应立即停止运行。
- `GREEN` : 比赛正常进行状态，车辆可以正常运行并且没有限速。
- `Gxx` : 带有数字的比赛旗帜表示限速，单位是`km/h`，比如`G20`表示正常进行比赛但是限速`20km/h`。


## 4. 安装运行

### 4.1 安装依赖项
#### 4.1.1 安装ROS Noetic、配置系统源和Python源等
强烈推荐鱼香ROS的一键安装脚本，运行下面的命令以后就可以自动安装ROS、配置环境等
``` bash
sudo wget http://fishros.com/install -O fishros && sudo chmod +x fishros && sudo ./fishros # 下载并启动小鱼ROS安装脚本
```
确认ROS noetic版本安装成功，并进行了换源操作（换系统源并清理第三方源），如果需要可以继续安装Vscode等必要工具。

#### 4.1.2 安装CARLA 0.9.13 
Carla版本并不一定，根据实际情况选择，比如也可以是0.9.15等。注意如果安装不同版本，在下方教程中对应下载的一些文件的名称中对应的版本号也需要修改成对应数字。
详细安装教程请参考[Ubuntu 20.04 ROS Noetic安装Carla 0.9.13 Linux编译版笔记](/tutorial/install_carla/Linux安装Carla编译版教程-小晶.docx)
请确保`carla-ros-bridge`也一同安装。在安装过程中报的错误如果上述文档中提及的参考网站没有收录，那么通常可以直接问豆包来解决。在给AI复制我们的错误信息时，尽可能包含更多的输出信息，防止漏掉实际错误内容导致无法解决问题。
安装好**CARLA 0.9.13**和`carla-ros-bridge`后，可以通过下面的案例来进行验证。

运行carla
``` bash
cd ~/carla # 切换到你的carla安装目录，如果跟着上面教程来就是这里
make launch # 启动carla
```
启动`carla-ros-bridge`并生成一个可以控制的自车。
``` bash
roslaunch carla_ros_bridge carla_ros_bridge.launch
```
如果能够正常启动，没有报错，可以控制车辆，那么就说明安装成功了。（手动控制车辆需要在弹出的窗口中按下`B`键切换切换手动控制模式，按下`H`键可以查看更多帮助）
#### 4.1.3 安装gtsam地图依赖
gtsam是一个用于解决机器人定位和导航问题的库，本项目中使用了gtsam来进行定位和建图，是一个必要的前置需求。
``` bash
cd ~ # 进入用户目录，避免权限问题
git clone https://github.com/borglab/gtsam.git # 下载gtsam库
cd gtsam # 切换到gtsam目录
git checkout 4.0.3  # 切换到4.0.3稳定版，否则后续程序会报错！！！！！！！！！！
mkdir build && cd build # 切换到build目录
# 配置CMake（重点：关闭Python绑定以加速编译，保留Boost支持）
cmake  -DGTSAM_BUILD_EXAMPLES=OFF   -DGTSAM_BUILD_TESTS=OFF   -DGTSAM_BUILD_PYTHON=OFF  -DGTSAM_WITH_EIGEN=ON -DGTSAM_USE_BOOST=ON  -DCMAKE_INSTALL_PREFIX=/usr/local ..
sudo make -j$(nproc) # 编译gtsam
sudo make install   # 安装gtsam
```
### 4.1.4 安装Casadi
Casadi是一个用于数值优化的库，本项目中使用了Casadi来构建NMPC控制器，是一个必要的前置需求。
详细的安装教程请参考[从源码安装Casadi与ipopt教程](/tutorial/install_casadi_with_ipopt/install_casadi_with_ipopt.md)，其参考网页为[Ubuntu 20.04 CasADi 的C++安装方法（源码编译）](https://blog.csdn.net/Li_Dongyi/article/details/151186844)，同时也参考了`韩硕师兄`的笔记中的一节[Casadi配置各种求解器（ipopt、osqp、qpoases）](https://hs867785578.github.io/my_note/C%2B%2B/Casadi%E9%85%8D%E7%BD%AE%E5%90%84%E7%A7%8D%E6%B1%82%E8%A7%A3%E5%99%A8%EF%BC%88ipopt%E3%80%81osqp%E3%80%81qpoases%EF%BC%89/)。具体流程总结一下是这样：
- **1. 安装基本依赖**
``` bash
sudo apt install gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends
sudo apt install ipython3 python3-dev python3-numpy python3-scipy python3-matplotlib --install-recommends
sudo apt install swig --install-recommends
```
- **2. 安装Ipopt**

通用问题的非线性优化求解器Ipopt，用于求解NMPC控制器的优化问题，是本项目中Casadi安装的前提。Ipopt需要安装ASL,BLAS,LAPACK,HSL,MUMPS等求解器。
``` bash
mkdir -p ~/opt && cd ~/opt # -p表示如果目录不存在则创建
```
- **2.1 安装ASL**

ASL是Ipopt的一个依赖项，需要先安装ASL，否则后续安装Ipopt会失败。我们新建一个目录专门用来存放Casadi相关的优化库：
``` bash
cd ~/opt # 进入优化库集中存储目录
git clone https://github.com/coin-or/ASL.git # 下载ASL库
cd ASL # 切换到ASL目录
mkdir build && cd build # 切换到build目录
cmake .. # 配置CMake
make -j$(nproc) # 编译ASL
sudo make install # 安装ASL
```
出现Libraries have been installed in:/usr/local/lib就说明安装成功。
- **2.2 安装BLAS,LAPACK**
``` bash
sudo apt install libblas-dev  liblapack-dev
```
- **2.3 安装HSL**
HSL是Ipopt的一个求解器，我们用到了`ma27`求解器。首先克隆HSL的仓库：
``` bash
cd ~/opt # 进入优化库集中存储目录
git clone https://github.com/coin-or-tools/ThirdParty-HSL.git
```
这个求解器是一般来说需要在[COIN-HSL](https://link.zhihu.com/?target=https%3A//licences.stfc.ac.uk/product/coin-hsl)购买，但是COIN-HSL是一个商业软件，我们这里只能使用免费的`ma27`求解器。当然也可以申请免费的高级学术版。这里我们将免费版的内容放在[/tutorial/install_casadi_with_ipopt目录下面](/tutorial/install_casadi_with_ipopt/),复制`coinhsl`文件夹到刚刚下载好的`ThirdParty-HSL`目录下，然后进行下一步。(如果是arm架构的话，复制同一个文件夹下的那个压缩包，然后解压到刚刚说的位置，重命名为coinhsl文件夹)
``` bash
sudo ./configure
sudo make -j$(nproc)
sudo make install

```
- **2.4 安装MUMPS**
MUMPS是Ipopt的默认求解器，我们虽然最终没有用到`mumps`求解器，但是在测试过程中依赖。首先克隆MUMPS的仓库：
``` bash
cd ~/opt # 进入优化库集中存储目录
git clone https://github.com/coin-or-tools/ThirdParty-Mumps.git
cd ThirdParty-Mumps
sudo ./get.Mumps 
sudo ./configure
sudo make -j$(nproc)
sudo make install
sudo apt install libmumps-seq-dev
```
- **2.5 安装Ipopt**
``` bash
cd ~/opt # 进入优化库集中存储目录
git clone https://github.com/coin-or/Ipopt.git
cd Ipopt
mkdir build && cd build
../configure --with-mumps --with-mumps-cflags="-I/usr/local/include/coin-or/mumps" --with-mumps-lflags="-L/usr/local/lib -lcoinmumps"  --with-hsl-cflags="-I/usr/local/include/coin-or/hsl" --with-hsl-lflags="-L/usr/local/lib -lcoinhsl"
sudo make
sudo make install

```
- **3. 安装osqp(0.6.0版本，casadi只支持0.6)**
``` bash
cd ~/opt # 进入优化库集中存储目录
git clone https://github.com/osqp/osqp.git
cd osqp
git checkout -b 0.6.0 v0.6.0
git submodule update --init --recursive
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```
- **4. 安装qpoases**

如果安装过程没有报错就最好，如果报错了也可以不安装，但是在稍后编译casadi的时候记得关闭qpoases的编译选项。这里在ubuntu20.04下可以切换到更早的版本，否则会报错。
``` bash
git clone https://github.com/coin-or/qpOASES.git
cd qpOASES
git checkout release/3.2.1 # 具体版本还没确认过，如过还不行可以切换更早版本（比如：releases/3.1.0，releases/3.1.1，releases/3.2.0， stable/3.0， stable/3.1， stable/3.2）
git submodule update --init --recursive
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```
- **5. 安装casadi**
``` bash
cd ~/opt # 进入优化库集中存储目录
git clone https://github.com/casadi/casadi.git
cd casadi
mkdir build
cd build
cmake -DWITH_PYTHON=ON -DWITH_PYTHON3=ON -DWITH_IPOPT=ON -DWITH_MUMPS=ON -DWITH_OSQP=ON -DWITH_LAPACK=ON -DWITH_QPOASES=ON .. # 如果qpoases报错没安装成功，那么就去掉-DWITH_QPOASES=ON这个选项
make -j$(nproc)
sudo make install
```
安装的目录默认是`/usr/local/lib`，如果需要修改安装目录，可以在cmake的时候加上`-DCMAKE_INSTALL_PREFIX=/usr/local/lib`。安装好之后需要在`~/.bashrc`文件中设置2个变量和一个地址：
``` bash
gedit ~/.bashrc
```
在文件末尾添加下面代码并**保存**再关闭
``` bash
export CasADi_INCLUDE_DIRS=/usr/local/include
export CasADi_LIBRARIES=/usr/local/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```
这样在编译相关ROS代码的时候就可以找到CasADi的头文件和库文件了。


#### 4.1.5 安装其他依赖项并配置系统安装环境
在进行操作之前强烈建议进行鱼香ROS的换源操作，包括系统源和Python源等，否则后续可能会遇到一些问题。
``` bash
sudo apt update
sudo apt install -y ros-noetic-ros-base libceres-dev\
ros-noetic-tf ros-noetic-pcl-conversions ros-noetic-cv-bridge \
ros-noetic-visualization-msgs ros-noetic-geometry-msgs ros-noetic-nav-msgs \
libpcl-dev libopencv-dev libgeographic-dev ros-noetic-robot-localization ros-noetic-robot-state-publisher ros-noetic-navigation ros-noetic-ackermann-msgs
```
在编译定位与建图相关的包时，若不进行此修改会遇到FLANN相关的错误，需要进行如下配置：
``` bash
sudo gedit /usr/include/flann/util/serialization.h
```
大概在文件的第34行左右
``` c++
template <typename Archive, typename T>
struct access {
    static void serialize(Archive& ar, T& type){
        // type.serialize(ar); // × 这一句导致了问题
        serialize(ar, type); // √ 修改成这，注意type挪到了括号内部
    }
};
```


### 4.2 编译安装本项目

``` bash
cd ~
git clone https://github.com/KomasaQi/HitchOpen-THICV-Stack.git  # 首先下载本项目
cd HitchOpen-THICV-Stack
catkin_make # 进行编译
```
如果一切顺利，编译完成一切就都完美结束了，但是大概率不会，会报错。比如找不到`race_msgs::XXXX`之类的文件，但不要着急，这是因为编译顺序的问题，其中一些包之间有相互依赖的关系，需要先编译依赖的包，再编译当前包才不会出错，但是往往不会按照这个顺序来就导致了错误。这个时候是需要**再重复在刚刚的终端运行**：
``` bash
catkin_make
```
你可能会发现，这次报错的内容和上次不一样了，这说明问题有所解决，但是不一定能解决所有的问题，所以需要重复运行`catkin_make`直到所有的包都编译通过为止。如果你反复运行了5次以上都提示同样的报错，那么建议将报错内容**完整地附带上下文地**复制给豆包让她帮咱看看是什么错，大概率是小错误缺少包之类的，可以解决。

编译好`[100%]xxxx`通过以后，别忘了将本项目的编译内容加载到环境中。
``` bash 
gedit ~/.bashrc
```
在文件末尾添加下面代码并**保存**再关闭
``` bash
source ~/HitchOpen-THICV-Stack/devel/setup.bash
```
这样在运行ROS代码的时候就可以找到本项目的内容了。记得新建一个终端再运行代码，否则很可能加载了也不起作用（是个系统bug）。

### 4.3 相关资源下载
一些定位与建图相关的ROSBAG资源可以在[百度云盘数据库](https://pan.baidu.com/s/1-sAB_cNlYPqTjDuaFgz9pg)下载，提取码：`ejmu`。

本项目的一些相关文件也放在[清华云盘-极限AI挑战赛THU](https://cloud.tsinghua.edu.cn/d/35fe77d97a684d77aa1a/)，比如clash-for-linux等小组件。

很多知识都可以在`韩硕师兄`的个人笔记中找到答案[Hans's Notebook](https://hs867785578.github.io/my_note/)


### 4.4 运行本项目
下面提供了一些运行的案例可以参考。
---
#### 4.4.1 Carla车辆轨迹跟踪仿真
在运行前需要先启动Carla模拟器，就打开默认的`Town10HD_Opt`地图, 如图所示，启动并保持运行状态。
![Carla Town10HD_Opt](/tutorial/images/runing_carla.png)

然后在本项目的终端中运行：(强烈建议安装terminator, 可以在一个终端中打开多个窗口，`sudo apt install terminator`)。让我们在第一个终端中启动`carla-ros-bridge`并生成一辆**Tesla.Model3**自车：
``` bash
roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch spawn_point:="-114.0,-30.0,1.5,0.0,0.0,-90.0" # 设置起点前产生车辆
```
然后在新的终端中运行带有一阶+纯时延动力学的转向、刹车、油门动力学的`carla_race_bridge`，以将本仓库自定义的`race_msgs`消息包中的消息与`carla-ros-bridge`中的消息进行转换：
``` bash
roslaunch carla_race_bridge carla_race_bridge_with_dynamics.launch
```
再在新的一个终端中运行静态全局规划代码：
``` bash
roslaunch race_global_static_planner race_global_carla_town10.launch
```
之后在新的一个终端中运行轨迹跟踪控制代码：（NMPC双轴转向+PID纵向速度跟踪）
``` bash
roslaunch race_tracker excel_controller_carla.launch
```



![track_sim_run](/tutorial/images/tracking_sim_run.png)


此时应该会显示收到RED Flag，启动了紧急停止，车辆并不会运动，我们需要打开`competition_timer`来控制车辆运行。
``` bash
roslaunch competition_timer competition_timer.launch
```
我们需要设置比赛旗帜的颜色来控制车辆状态。如：
手动更改比赛计时器的旗帜状态为GREEN：
``` bash
rosparam set /competition_timer/flag GREEN # 设置比赛状态，可选：GREEN RED BLACK G5 G10 G15 G20 G40 G60 G80
```
上述除了设置旗帜的内容可以通过一行代码启动：
``` bash
roslaunch simple_racing simple_tracking_carla_town10_with_dynamics.launch
```
然后就可以来设置一个GREEN旗帜，车辆就会开始运动了。
---

#### 4.4.2 Carla车辆定位+轨迹跟踪仿真
这是最为全面的一个仿真案例，采用liorf_localization进行定位。

在进行定位前需要[☞下载`Carla Town10HD_Opt`的点云地图文件](https://cloud.tsinghua.edu.cn/d/35fe77d97a684d77aa1a/files/?p=%2Fmaps%2FCarla_Map_Town10%2FGlobalMap.pcd)，放在`launch/simple_racing/maps/Carla_Map_Town10`目录下。

我们可以启动一下该启动的内容：
``` bash
roslaunch simple_racing simple_tracking_carla_town10_lidar_loc.launch
```

启动后应该是类似下图的界面，但是定位信息不存在。我们需要手动指定一下初始位置估计，方便算法来收敛。在左侧全局试图的`RVIZ`中，点击`绿色箭头后的 2D Pose Estimation`，然后在如下图所示的地图下方轨迹中断位置点击鼠标左键，并向右拖动指定航向为向右，稍后应该即可收敛到正确的位置，并在右上方小的自车视角RVIZ中查看到实时白色点云信息更新。

![track_sim_run_lidar_loc](/tutorial/images/complete_tracking.png)

此时可以设置比赛计时器参数（在launch文件中已经启动了competition_timer）:由于定位模块更新频率收传感器限制并不快，且存在误差，在高速容易导致误差累计车辆失稳;所以这里经过测试，设置G60参数来限制车速。
``` bash
rosparam set /competition_timer/flag G60 # 防止车速过快定位失效
```
---
#### 4.4.3 CICV车辆定位测试（Velodyne VLP-32C + FDI Link Gnss）
这是用实车数据进行的定位仿真案例，采用liorf_localization进行定位仿真。采集的是国汽智联园区外部的感知数据。在进行定位前需要[☞下载`Cicv`的点云地图文件`GlobalMap.pcd`（注意不要改名字）](https://cloud.tsinghua.edu.cn/d/35fe77d97a684d77aa1a/files/?p=%2Fmaps%2FCicv_Map_Outside%2FGlobalMap.pcd)，放在`launch/simple_racing/maps/Cicv_Map_Outside`目录下。同时[☞下载`pix_moving.bag`的传感器实时录制文件](https://cloud.tsinghua.edu.cn/d/35fe77d97a684d77aa1a/files/?p=%2Fmaps%2FCicv_Map_Outside%2Fpix_moving.bag)，也放在`launch/simple_racing/maps/Cicv_Map_Outside`目录下（或其他任何你能找到的文件路径）。

我们可以启动一下位姿发布：
``` bash
roslaunch lio_sam cicv_robot_state_publisher.launch
```
再启动我们的liorf_localization：
``` bash
roslaunch liorf_localization run_cicv_localization.launch
```
此时拖动界面到全景预览的这个视角位置，然后找到左下角中红色方框框柱的位置，将视野移动过去，在这里我们要手动指定一下初始位置估计，方便算法来收敛。
![start gui](/tutorial/images/cicv_map.png)
在左侧全局试图的`RVIZ`中，点击`绿色箭头后的 2D Pose Estimation`，然后在如下图所示的地图下方轨迹中断位置点击鼠标左键，并向右拖动指定航向为向图示所示位置。大概就位置就在路边公交车那一侧开始数第三辆车中间的位置具体如下图所示：
![init pose estimation](/tutorial/images/init_pose_estimation_cicv.png)

接下来我们就可以播放rosbag包了：
``` bash
cd $(rospack find simple_racing)/maps/Cicv_Map_Outside/
rosbag play pix_moving.bag
```
如果不出意外，定位效果应该会一切正常如下图所示，直到定位到bag包播放结束。
![localization result](/tutorial/images/cicv_loc_result.png)

---
#### 4.4.4 天门山预赛车辆启动流程
**1. 启动梯子**

打开电脑第一步先启动一个终端并运行下列命令打开clash梯子
``` bash
cd ~/下载/clash-for-linux
sudo bash start.sh
```
一般情况下小车上电以后需要稍等一分钟左右，等待网络模块加载完毕才能上网，否则会出现连接超时的情况。**如果超时可以稍等一分钟再尝试。**

如果出现2个`[ok]`一切顺利就可以继续，如果出现无法连接的情况可以考虑是否是订阅链接出现了问题是否需要更换。更换订阅链接的方法是在`~/下载/clash-for-linux`文件夹下打开`.env`文件，如果看不到请确保勾选查看所有隐藏文件的选项。替换**引号内的链接**为你准备好的新链接即可。替换好保存后再重新启动上述命令打开clash，如果全部ok就继续。

**2. 移动车辆**

在启动之前需要确认当前位置，一般我们发车的位置在山顶；也有时开下去后需要从山底定位（当然可以不关闭定位直接切换规划器路径再开上去就行）。比如当前位置在山顶P房，需要准备开始比赛，建议开车到起点线之前，这样确保两件事：

 - **周围特征信息和建图时类似**（否则周围特征变化大定位容易失败）
 - **GPS信号良好** （否则初始GPS位置会偏移严重，干扰后续定位）

**3. 确保地图文件和路径规划无误**

确保开到位置以后，我们需要确保`点云地图文件`和`全局路径文件`是正确的。
打开文件管理器，进入`~/HitchOpen-THICV-Stack/src/launch/simple_racing/maps/Tianmen_Map/`文件夹，下面应该会有几个文件：`UpStart.pcd`、`DownStart.pcd`、`GlobalMap.pcd`、`BeforeTunnel.pcd`，区别在于0高度平面的位置不一样。因为初始定位的时候在rviz中指定的初始位置默认是在0高度平面上的，所以需要**保证当前车辆的位置在0高度平面附近**。

- `UpStart.pcd`的基准平面在山顶P房旁的露天小空地上；
- `DownStart.pcd`的基准平面在山脚下的狐仙剧场；
- `BeforeTunnel.pcd`基准平面在下了蛋糕弯的第一个S形隧道前（相对山顶-97.35m）。
- `GlobalMap.pcd`是我们定位节点默认要求的全局点云文件名称。

比如我现在在山顶上，所以我需要将原有的`GlobalMap.pcd`删除掉（如果存在这个文件），并将`UpStart.pcd`复制一份并将文件名仍然修改为`GlobalMap.pcd`。

下一步确保我们的路径规划是从山上往下没问题的：文件管理器进入`~/HitchOpen-THICV-Stack/src/planning/race_global_static_planner/config/tianmen_down_pre_map`文件夹，这里面应该会有几个文件，包含
- `Tianmen_raceline_Oct_20ok.csv`、`Tianmen_raceline_Oct_autoup.csv`，分别是从山上往下开的，以及自动上山的（从山下往上开）。

我们在山上，需要开始比赛，所以选择从山上往下开的。复制`Tianmen_raceline_Oct_20ok.csv`文件，并将其修改为没有后缀的`Tianmen_raceline_Oct.csv`（如果这个文件存在，请先删除）。


**4. 启动驱动并进行定位**

`ctrl + Alt + T`调出终端窗口，右键操作水平、竖直分屏，打开5~6个窗口方便后续操作。

首先启动**总体硬件驱动+定位**：
``` bash
roslaunch simple_racing tianmen_racing.launch 
```
此时会启动2个rviz界面，我们关注有小车的那个界面。稍等30s左右会加载出点云地图。根据当前位置估计进行初始位置与朝向的赋予。等待1min左右如果顺利的话就会刷新出位姿（如果rviz界面变成黑色，别慌，大概是当前摄像机焦点不在定位的位置，我们手动在视野中寻找一番就可以）

**5. 启动计时器**

在新终端窗口中运行计时器。
``` bash
roslaunch competition_timer tianmen_down_pre_timer.launch
```


**6. 打开轨迹跟踪控制器**

在新终端窗口中打开控制器。
``` bash
roslaunch race_tracker simple_controller_carla.launch 
```

**7. 可以出发**

在新终端窗口手动更改比赛计时器的旗帜状态为G20：
``` bash
rosparam set /competition_timer/flag G20 # 设置比赛状态，可选：GREEN RED BLACK G5 G10 G15 G20 G40 G60 G80
```
需要急停时用todesk窗口在这个终端中让状态变成`RED`即可，车辆会立即停止。
``` bash
rosparam set /competition_timer/flag RED 
```

---
## Contributors:

- 戚笑景 Komasa Qi （清华大学）
- 何瑞坤 He Ruikun （辽宁工业大学）
- 邹恒多 Zou Hengduo （清华大学）
- 扶尚宇 Fu Shangyu （清华大学）
- 邱逸凡 Qiu Yifan （中国农业大学）
- 黄梓谦 Huang Ziqian （华南理工大学）
- 冷佳桐 Leng Jiatong （清华大学）
- 李珂 Li Ke （重庆大学） 
- 蒋涛 Javier Jiang （重庆大学）


感谢以上所有参与项目的贡献者，我们的项目是一个开源项目，欢迎所有的贡献者参与进来。
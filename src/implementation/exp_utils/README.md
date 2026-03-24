# exp_utils

`exp_utils` 是一个用于实验辅助的小工具包，当前已提供 `vehicle_state_recorder.py`，用于将 `/race/vehicle_state` 话题中的车辆状态完整记录到 CSV 文件中，便于后续离线分析、绘图、回放与实验归档。后续你还可以继续在这个包里扩展其他实验工具。

---

## 1. 当前功能

目前本包已包含以下功能：

### 1.1 vehicle_state_recorder

用于订阅 `race_msgs/VehicleStatus` 类型的 `/race/vehicle_state` 话题，并将其中全部状态量展开后记录到 CSV 文件。

它具备以下特点：

- 支持输入实验名称；
- 自动在 CSV 文件名后附加**开始记录时间**；
- 自动避免覆盖历史实验文件；
- 自动创建输出目录；
- 将 `VehicleStatus` 中的嵌套字段全部展开为独立列，便于 MATLAB、Python、Excel 直接读取分析；
- 提供默认 `yaml` 配置文件与 `launch` 文件，调用方便。

---

## 2. 目录结构建议

建议你的 `exp_utils` 包目录至少包含如下内容：

```bash
exp_utils/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── vehicle_state_recorder.yaml
├── launch/
│   └── vehicle_state_recorder.launch
└── scripts/
    └── vehicle_state_recorder.py
```

---

## 3. 使用前准备

### 3.1 依赖

本节点依赖：

- `rospy`
- `race_msgs`
- ROS1 环境

你已经说明当前已经创建了引用 `race_msgs` 的 `exp_utils` 包，因此通常只需保证工作空间已经正确编译并 source。

例如：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

如果你使用的是 `catkin build`，则改为：

```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

---

## 4. 功能说明

### 4.1 输出文件命名规则

输出的 CSV 文件名格式如下：

```bash
实验名称_开始记录时间.csv
```

例如：

```bash
double_lane_change_20260324_153015.csv
```

若同名文件已存在，程序会自动继续追加序号，例如：

```bash
double_lane_change_20260324_153015_001.csv
```

这样可以避免覆盖其他实验记录，也能清楚区分每次实验的开始时间。

---

### 4.2 默认记录内容

当前节点会完整记录 `race_msgs/VehicleStatus` 中的全部状态量，包括但不限于：

- `header`
- `child_frame_id`
- `pose`
- `euler`
- `vel`
- `acc`
- `lateral`
- `wheel_speed`
- `gear`
- `control_mode`
- `hand_brake`
- `emergency`
- `clutch`
- `steering_mode`
- `tracking`
- `throttle_fb`
- `brake_fb`
- `trailer`
- `ltr_state`

同时还会额外记录：

- 写入序号 `record_seq`
- 当前系统时间 `record_wall_time_iso`
- 当前记录时刻对应的 ROS 时间 `record_ros_time`

这样后续做实验对齐、日志同步和多源数据关联会更方便。

---

## 5. 参数说明

默认参数文件位于：

```bash
config/vehicle_state_recorder.yaml
```

其主要参数如下：

| 参数名 | 含义 | 默认值 |
|---|---|---|
| `topic_name` | 订阅的话题名称 | `/race/vehicle_state` |
| `experiment_name` | 实验名称 | `default_experiment` |
| `output_dir` | CSV 输出目录 | `~/.ros/exp_utils/vehicle_state_logs` |
| `flush_every_n` | 每接收多少条消息后写盘一次 | `1` |
| `use_wall_time_in_filename` | 文件名是否使用系统时间 | `true` |

说明：

- 一般推荐 `use_wall_time_in_filename=true`，因为这样更适合管理真实实验记录；
- `flush_every_n=1` 最稳妥，每条都立即写盘，适合实验场景；
- 若消息频率很高、担心磁盘写入压力，可以设置为更大的值，比如 `10` 或 `50`。

---

## 6. 启动方式

### 6.1 使用 launch 文件启动

最推荐的启动方式：

```bash
roslaunch exp_utils vehicle_state_recorder.launch
```

### 6.2 在启动时指定实验名称

例如：

```bash
roslaunch exp_utils vehicle_state_recorder.launch experiment_name:=slalom_test
```

生成的文件可能为：

```bash
~/.ros/exp_utils/vehicle_state_logs/slalom_test_20260324_153015.csv
```

### 6.3 同时修改输出目录

```bash
roslaunch exp_utils vehicle_state_recorder.launch \
    experiment_name:=brake_test \
    output_dir:=/home/user/exp_logs
```

### 6.4 修改订阅话题

如果后续你的车辆状态话题名称变化，也可以直接覆盖：

```bash
roslaunch exp_utils vehicle_state_recorder.launch \
    topic_name:=/my_vehicle/state
```

---

## 7. 直接运行脚本方式

除了 launch 方式，也可以直接运行节点：

```bash
rosrun exp_utils vehicle_state_recorder.py _experiment_name:=test1
```

例如：

```bash
rosrun exp_utils vehicle_state_recorder.py \
    _topic_name:=/race/vehicle_state \
    _experiment_name:=fishhook_test \
    _output_dir:=/home/user/exp_logs
```

注意在 `rosrun` 下，私有参数要写成 `_参数名:=参数值` 的形式。

---

## 8. CSV 字段设计说明

由于 `VehicleStatus` 中存在大量嵌套消息，节点内部会自动将其展开为扁平化列名。例如：

- `pose.position.x` → `pose_position_x`
- `vel.angular.z` → `vel_angular_z`
- `trailer.euler.yaw` → `trailer_euler_yaw`
- `ltr_state.ltr_rate` → `ltr_state_ltr_rate`

这样设计的好处是：

1. 不需要后处理时再解析嵌套结构；
2. MATLAB / pandas / Excel 读取更直接；
3. 更适合后续画图、滤波、对比实验与自动分析脚本。

---

## 9. 典型使用流程

下面给出一个常见实验流程示例：

### 第一步：启动 ROS 环境

```bash
source ~/catkin_ws/devel/setup.bash
```

### 第二步：启动你的车辆系统

确保 `/race/vehicle_state` 正常发布。

可以先检查：

```bash
rostopic echo /race/vehicle_state
```

### 第三步：启动记录器

```bash
roslaunch exp_utils vehicle_state_recorder.launch experiment_name:=dlc_01
```

### 第四步：完成实验后结束节点

按 `Ctrl+C` 结束后，CSV 会自动关闭并保存。

---

## 10. 建议

### 10.1 实验命名建议

建议实验名称尽量简洁且有辨识度，例如：

- `dlc_01`
- `fishhook_speed15`
- `brake_test_lowmu`
- `ramp_entry_case3`

这样后续做批量分析时会更方便。

### 10.2 关于时间戳

当前文件名中的时间是“开始记录的系统时间”，主要用于实验管理。

而 CSV 内容里同时保留了：

- 消息自身的 `header.stamp`
- 当前记录时刻的 ROS 时间 `record_ros_time`
- 当前系统时间字符串 `record_wall_time_iso`

因此无论你后续是做仿真对齐，还是做真实实验日志归档，都会比较方便。

---

## 11. 可能的后续扩展方向

既然 `exp_utils` 以后还会继续扩展，下面是一些很适合继续加入的工具方向：

- 控制命令记录器（记录 `/control`、`/cmd` 等话题）；
- 多话题同步记录器；
- 自动实验编号管理工具；
- 实验结束后自动生成统计摘要；
- CSV 转 MAT / NPZ 工具；
- 记录与视频时间戳对齐工具；
- 一键绘图分析脚本。

这样这个包后面会逐渐变成一个比较完整的实验辅助工具箱。

---

## 12. 注意事项

1. 请确保 `vehicle_state_recorder.py` 具有可执行权限：

```bash
chmod +x scripts/vehicle_state_recorder.py
```

2. 请确保 `package.xml` 和 `CMakeLists.txt` 中已经声明了对 `rospy` 与 `race_msgs` 的依赖。

3. 若使用 `roslaunch` 时提示找不到脚本，一般是以下几种原因：

- 没有给 `scripts/vehicle_state_recorder.py` 可执行权限；
- 工作空间没有重新编译；
- 没有重新 `source devel/setup.bash`。

---

## 13. 最小自检方法

你可以用下面的方法快速检查是否工作正常：

```bash
rostopic list | grep vehicle_state
roslaunch exp_utils vehicle_state_recorder.launch experiment_name:=debug_test
```

然后检查输出目录下是否生成类似文件：

```bash
~/.ros/exp_utils/vehicle_state_logs/debug_test_20260324_153015.csv
```

打开前几行确认表头和数据是否正常即可。

---

## 14. 当前版本说明

当前版本提供：

- `vehicle_state_recorder.py`
- `vehicle_state_recorder.yaml`
- `vehicle_state_recorder.launch`
- 本中文 README

后续你可以继续在 `exp_utils` 中逐步加入新的实验工具，而无需重新组织包结构。

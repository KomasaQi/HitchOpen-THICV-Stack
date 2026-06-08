# race_lane_detection

单目前向 RGB 车道线感知包（ROS1 Noetic / Ubuntu 20.04）。订阅 CARLA 前视相机话题，
在线检测车道线，逆透视（IPM）到自车坐标系（FLU：x前 y左 z上），用可配置阶数的多项式
拟合**所有车道线**与**所有车道中心线**（含相邻车道），并发布结构化结果。

本包后续将接替 `race_global_static_planner`：把静态全局轨迹替换为实时车道中心线轨迹，
并据 `left_lane_available / right_lane_available` 判据规划左/右换道。当前版本已输出
`race_msgs/Path` 形式的自车道中心线轨迹，可被下游 `race_tracker` 直接跟踪。

## 1. 依赖

```bash
sudo apt install ros-noetic-cv-bridge python3-opencv
pip3 install numpy
# 学习后端(可选, 推荐):
pip3 install onnxruntime-gpu    # 有 CUDA 时; 否则 pip3 install onnxruntime
```

## 2. 编译

将本包放在 `~/HitchOpen-THICV-Stack/src/perception/race_lane_detection/`，
并确保已更新 `src/common/race_msgs`（新增 `LaneLine.msg`、`LaneDetection.msg`）。

```bash
cd ~/HitchOpen-THICV-Stack
catkin_make            # 或 catkin build
source devel/setup.bash
```

## 3. 选择检测后端

`detector_backend` 参数：
- `auto`（默认）：UFLDv2 权重就绪则用学习后端，否则自动回退规则后端。
- `learning`：仅用 UFLDv2（需 onnx 权重 + onnxruntime）。
- `classic`：仅用规则后端（HLS/梯度阈值 + IPM 鸟瞰 + 滑窗），无需 GPU，CPU 约 30fps。

### 模型与相机分辨率建议（重要）

UFLDv2 的网络输入尺寸是**固定的**，与相机分辨率无关（节点内部会自动 resize）：
- **CULane**：输入 320×1600，4 车道，适合直道/城市道路。
- **CurveLanes**：输入 800×1600，10 车道，**弯道更稳，推荐**。

CARLA 相机建议设为 **1600×800（2:1）**，正好匹配 CurveLanes 的 800×1600 输入比例，
resize 时几乎无形变，效果最好。配 `image_size_x:1600, image_size_y:800, fov:90`。
若用 CULane(320×1600)，比例 5:1 形变较大，建议相机仍设宽幅(如 1280×720)。

`ufld_dataset: auto` 会按 onnx 输出张量形状（4 车道→culane，10 车道→curvelanes）
自动判别并设置正确的行/列锚与解码分支，一般无需手动指定。

### 下载学习后端权重

```bash
rosrun race_lane_detection download_weights.py --model curvelanes_res34
# 按提示从 PINTO model zoo 获取 onnx, 放入 weights/ 后:
# 在 config/params.yaml 设置 onnx_path 指向该文件
```

### 排查检测效果（脱离 ROS）

```bash
rosrun race_lane_detection debug_on_image.py \
    --image /path/to/frame.png \
    --onnx  $(rospack find race_lane_detection)/weights/ufldv2_curvelanes_res34_800x1600.onnx \
    --fov 90 --out /tmp/lane_debug.png
```
输出图中**红点**=网络原始检测点、**绿线**=拟合车道边界、**橙线**=中心线。
红点乱 → 模型/裁剪/分辨率问题；红点对但绿/橙线偏 → IPM 外参或 `ground_z` 问题。
运行节点时 `/race/lane_vis_image` 也会叠加这些红点，可在 RViz/rqt_image_view 中查看。

## 4. 运行

```bash
# 规则后端(开箱即用):
roslaunch race_lane_detection lane_detection.launch detector_backend:=classic open_rviz:=true

# 学习后端:
roslaunch race_lane_detection lane_detection.launch \
    detector_backend:=learning \
    onnx_path:=$(rospack find race_lane_detection)/weights/ufldv2_culane_res18_800x1600.onnx
```

## 5. 话题接口

订阅
- `/carla/ego_vehicle/rgb_front/image` (sensor_msgs/Image, bgra8)
- `/carla/ego_vehicle/rgb_front/camera_info` (sensor_msgs/CameraInfo)

发布
- `/race/lane_detection` (**race_msgs/LaneDetection**) 结构化结果
- `/race/lane_center_path` (**race_msgs/Path**) 自车道中心线轨迹
- `/race/lane_vis_image` (sensor_msgs/Image) 叠加可视化
- `/race/lane_markers` (visualization_msgs/MarkerArray) RViz 标记

## 6. 消息格式（race_msgs）

`LaneLine.msg`（单条线）
- `coeffs`：`y(x)=c0+c1*x+...+c_order*x^order`，低次在前；默认补零到 5 阶（6 个系数），每个保留 6 位小数。
- `lane_id`：边界线 0=自车道左边线、1=自车道右边线，向左递减、向右递增；中心线 0=自车道、向左负、向右正。
- 还含 `order / x_start / x_end / fit_rmse / confidence / num_points / points`。

`LaneDetection.msg`（综合结果）
- `lane_lines[]`、`center_lines[]`（均从左到右排序）
- `ego_center_index`、`left_lane_available`、`right_lane_available`
- `ego_lateral_offset`（左正）、`ego_heading_error`（左正）
- `detect_latency_ms`、`source`

## 7. 坐标与几何

相机外参默认安装于自车系 `(3.5, 0, 2.0)`，pitch=roll=yaw=0，FOV 90°。
逆透视采用平地假设（地面 `ground_z`）。CARLA RGB 相机为 `bgra8`，其光学坐标已对齐
标准 CV 约定，本包据此构建 `ego↔optical` 旋转。若相机离地高度或地面高度不同，
调整 `cam_z` 与 `ground_z`。

## 8. 关键可调参数（config/params.yaml）

| 参数 | 说明 | 默认 |
|------|------|------|
| `fit_order` | 拟合阶数(≤`max_fit_order`) | 3 |
| `report_order` | 系数补零输出阶数 | 5 |
| `coeff_decimals` | 系数小数位 | 6 |
| `lane_width` | 标称车道宽(m) | 3.5 |
| `process_rate` | 处理频率(Hz) | 15 |
| `ground_z` | 地面在自车系的 z(m) | 0.0 |

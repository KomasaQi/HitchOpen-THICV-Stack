#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
race_lane_detection_node.py
---------------------------
单目前向 RGB 车道线感知 ROS1 节点。

订阅:
  ~camera_info_topic (sensor_msgs/CameraInfo)  相机内参
  ~image_topic       (sensor_msgs/Image)       前视 RGB 图像 (CARLA: bgra8)

发布:
  ~lane_topic        (race_msgs/LaneDetection)  结构化车道线检测结果
  ~ego_center_path_topic (race_msgs/Path)       自车道中心线轨迹(供下游直接跟踪)
  ~vis_image_topic   (sensor_msgs/Image)        叠加可视化(可选)
  ~markers_topic     (visualization_msgs/MarkerArray) RViz 车道线标记(可选)

后端:
  detector_backend: "learning" | "classic" | "auto"
  学习后端权重缺失或加载失败时, auto 模式自动回退到 classic。
"""

import math
import time

import numpy as np
import rospy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Header

from race_msgs.msg import LaneLine, LaneDetection, Path, PathPoint

try:
    from visualization_msgs.msg import Marker, MarkerArray
    _HAS_VIS_MARKER = True
except Exception:
    _HAS_VIS_MARKER = False

try:
    import cv2
    from cv_bridge import CvBridge
    _HAS_CV = True
except Exception:
    _HAS_CV = False

from race_lane_detection.geometry import GroundProjector
from race_lane_detection.classic_detector import ClassicLaneDetector
from race_lane_detection.ufld_detector import UFLDv2Detector
from race_lane_detection.lane_assembler import assemble


class LaneDetectionNode(object):
    def __init__(self):
        # ----------------------- 参数 ----------------------- #
        gp = lambda k, d: rospy.get_param("~" + k, d)
        self.image_topic = gp("image_topic",
                              "/carla/ego_vehicle/rgb_front/image")
        self.cam_info_topic = gp("camera_info_topic",
                                 "/carla/ego_vehicle/rgb_front/camera_info")
        self.lane_topic = gp("lane_topic", "/race/lane_detection")
        self.ego_path_topic = gp("ego_center_path_topic",
                                 "/race/lane_center_path")
        self.vis_image_topic = gp("vis_image_topic", "/race/lane_vis_image")
        self.markers_topic = gp("markers_topic", "/race/lane_markers")
        self.child_frame_id = gp("ego_frame_id", "ego_vehicle")

        # 相机外参（自车 FLU 系）
        self.cam_xyz = (gp("cam_x", 3.5), gp("cam_y", 0.0), gp("cam_z", 2.0))
        self.cam_rpy = (gp("cam_roll", 0.0), gp("cam_pitch", 0.0),
                        gp("cam_yaw", 0.0))
        self.fov_deg = gp("cam_fov_deg", 90.0)
        # 地面在自车系中的 z 高度(米)。若自车原点在地面则为0；
        # 若自车原点取在某离地高度处, 则填入地面相对该原点的 z(通常为负)。
        self.ground_z = gp("ground_z", 0.0)

        # 拟合参数
        self.fit_order = int(gp("fit_order", 3))
        self.max_order = int(gp("max_fit_order", 5))
        self.report_order = int(gp("report_order", 5))
        self.coeff_decimals = int(gp("coeff_decimals", 6))
        self.lane_width = float(gp("lane_width", 3.5))

        # 运行参数
        self.backend = gp("detector_backend", "auto")  # learning|classic|auto
        self.process_rate = float(gp("process_rate", 15.0))
        self.publish_vis = bool(gp("publish_vis_image", True))
        self.publish_markers = bool(gp("publish_markers", True))

        # ----------------------- 几何/后端 ----------------------- #
        self.projector = GroundProjector(self.cam_xyz, self.cam_rpy,
                                         self.ground_z)
        # 无 camera_info 时按 fov 兜底(需要图像尺寸, 收到首帧后再定)
        self._intrinsics_ready = False

        classic_cfg = dict(lane_width=self.lane_width)
        self.classic = ClassicLaneDetector(self.projector, classic_cfg)

        ufld_cfg = dict(
            onnx_path=gp("onnx_path", ""),
            use_gpu=bool(gp("use_gpu", True)),
            dataset=gp("ufld_dataset", "auto"),       # auto|culane|curvelanes
            crop_ratio=gp("ufld_crop_ratio", None),   # None/"" -> 用数据集默认
            local_window=int(gp("ufld_local_window", 32)),
            min_lane_pts=int(gp("ufld_min_lane_pts", 6)),
        )
        cr = ufld_cfg.get("crop_ratio", None)
        if cr is None or cr == "" or (isinstance(cr, str) and not cr.strip()):
            ufld_cfg.pop("crop_ratio", None)
        else:
            ufld_cfg["crop_ratio"] = float(cr)
        self.ufld = UFLDv2Detector(self.projector, ufld_cfg)

        self._resolve_backend()

        # ----------------------- ROS I/O ----------------------- #
        self.bridge = CvBridge() if _HAS_CV else None
        self._last_img = None
        self._last_img_stamp = None

        self.pub_lane = rospy.Publisher(self.lane_topic, LaneDetection,
                                        queue_size=5)
        self.pub_path = rospy.Publisher(self.ego_path_topic, Path,
                                        queue_size=5)
        self.pub_vis = None
        if self.publish_vis and _HAS_CV:
            self.pub_vis = rospy.Publisher(self.vis_image_topic, Image,
                                           queue_size=2)
        self.pub_markers = None
        if self.publish_markers and _HAS_VIS_MARKER:
            self.pub_markers = rospy.Publisher(self.markers_topic, MarkerArray,
                                               queue_size=2)

        rospy.Subscriber(self.cam_info_topic, CameraInfo, self._cb_info,
                         queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self._cb_image,
                         queue_size=1, buff_size=2 ** 24)

        period = 1.0 / max(1.0, self.process_rate)
        self.timer = rospy.Timer(rospy.Duration(period), self._on_timer)

        rospy.loginfo("[lane_detection] backend=%s onnx=%s",
                      self._active_backend, ufld_cfg["onnx_path"] or "(none)")
        rospy.loginfo("[lane_detection] image=%s info=%s",
                      self.image_topic, self.cam_info_topic)

    # ------------------------------------------------------------------ #
    def _resolve_backend(self):
        if self.backend == "classic":
            self._active_backend = "classic"
        elif self.backend == "learning":
            self._active_backend = "learning"
            if not self.ufld.ready:
                rospy.logwarn("[lane_detection] learning backend not ready "
                              "(missing onnx / onnxruntime); will fall back "
                              "to classic at runtime.")
        else:  # auto
            self._active_backend = "learning" if self.ufld.ready else "classic"

    def _source_enum(self):
        if self._active_backend == "learning":
            return LaneDetection.SOURCE_LEARNING
        return LaneDetection.SOURCE_CLASSIC

    # ----------------------- callbacks ----------------------- #
    def _cb_info(self, msg):
        if self._intrinsics_ready:
            return
        K = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        if K[0, 0] <= 0:
            return
        self.projector.set_intrinsics_from_K(K, msg.width, msg.height)
        self.classic._build_bev_grid()
        self._intrinsics_ready = True
        rospy.loginfo("[lane_detection] intrinsics from camera_info: "
                      "fx=%.1f size=%dx%d", K[0, 0], msg.width, msg.height)

    def _cb_image(self, msg):
        if not _HAS_CV:
            return
        try:
            # CARLA RGB 为 bgra8; 统一转 bgr8
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn_throttle(5.0, "[lane_detection] cv_bridge: %s", str(e))
            return
        self._last_img = img
        self._last_img_stamp = msg.header.stamp
        if not self._intrinsics_ready:
            h, w = img.shape[:2]
            self.projector.set_intrinsics_from_fov(w, h, self.fov_deg)
            self.classic._build_bev_grid()
            self._intrinsics_ready = True
            rospy.logwarn("[lane_detection] no camera_info yet; using fov=%.1f "
                          "fallback intrinsics (%dx%d)", self.fov_deg, w, h)

    # ----------------------- main loop ----------------------- #
    def _on_timer(self, _evt):
        if self._last_img is None or not self.projector.ready:
            return
        img = self._last_img
        stamp = self._last_img_stamp or rospy.Time.now()
        t0 = time.time()

        lanes_ego = self._run_backend(img)
        self._last_lanes_ego = lanes_ego
        res = assemble(lanes_ego,
                       fit_order=self.fit_order,
                       max_order=self.max_order,
                       report_order=self.report_order,
                       decimals=self.coeff_decimals,
                       lane_width=self.lane_width)
        latency = (time.time() - t0) * 1000.0

        self._publish(res, stamp, latency)
        if self.pub_vis is not None:
            self._publish_vis(img, res, stamp)
        if self.pub_markers is not None:
            self._publish_markers(res, stamp)

    def _run_backend(self, img):
        if self._active_backend == "learning" and self.ufld.ready:
            lanes = self.ufld.detect(img)
            if lanes:
                return lanes
            # 学习后端本帧无输出, auto/learning 均回退 classic 兜底
            return self.classic.detect(img)
        return self.classic.detect(img)

    # ----------------------- message build ----------------------- #
    def _make_laneline(self, info):
        m = LaneLine()
        m.type = info.type
        m.lane_id = info.lane_id
        m.coeffs = list(info.coeffs_lo)
        m.order = int(info.order)
        m.x_start = float(info.x0)
        m.x_end = float(info.x1)
        m.fit_rmse = float(info.rmse)
        m.confidence = float(info.conf)
        m.num_points = int(info.n)
        m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                    for p in info.pts]
        return m

    def _publish(self, res, stamp, latency):
        msg = LaneDetection()
        msg.header = Header(stamp=stamp, frame_id=self.child_frame_id)
        msg.child_frame_id = self.child_frame_id
        msg.lane_lines = [self._make_laneline(b) for b in res["boundaries"]]
        msg.center_lines = [self._make_laneline(c) for c in res["centers"]]
        msg.ego_center_index = int(res["ego_center_index"])
        msg.left_lane_available = bool(res["left_avail"])
        msg.right_lane_available = bool(res["right_avail"])
        msg.ego_lateral_offset = float(res["lat_offset"])
        msg.ego_heading_error = float(res["head_err"])
        msg.detect_latency_ms = float(latency)
        msg.source = self._source_enum()
        self.pub_lane.publish(msg)

        # 自车道中心线 -> race_msgs/Path（与 planner 接口一致, 供下游跟踪）
        path = Path()
        path.header = Header(stamp=stamp, frame_id=self.child_frame_id)
        eci = res["ego_center_index"]
        if eci >= 0:
            ec = res["centers"][eci]
            xs = np.linspace(max(0.0, ec.x0), ec.x1, 50)
            ys = np.array([ec.y_at(x) for x in xs])
            # 航向角 = atan(dy/dx) 数值差分
            dyx = np.gradient(ys, xs)
            headings = np.arctan(dyx)
            # 曲率 kappa = y'' / (1+y'^2)^1.5
            d2 = np.gradient(dyx, xs)
            kappa = d2 / np.power(1.0 + dyx ** 2, 1.5)
            for i in range(len(xs)):
                pp = PathPoint()
                pp.pose = Pose()
                pp.pose.position.x = float(xs[i])
                pp.pose.position.y = float(ys[i])
                pp.pose.position.z = 0.0
                # yaw -> 四元数(仅 z)
                qz = math.sin(headings[i] / 2.0)
                qw = math.cos(headings[i] / 2.0)
                pp.pose.orientation.z = qz
                pp.pose.orientation.w = qw
                pp.velocity = 0.0
                pp.curvature = float(kappa[i])
                path.points.append(pp)
        self.pub_path.publish(path)

    # ----------------------- visualization ----------------------- #
    def _publish_vis(self, img, res, stamp):
        vis = img.copy()
        # 先画原始检测点(红), 直观判断检测/IPM是否正确
        raw = getattr(self, "_last_lanes_ego", None)
        if raw:
            for pts in raw:
                pts3 = np.array(pts, dtype=np.float64)
                uv = self.projector.ground_to_pixels(pts3)
                uv = uv[np.isfinite(uv).all(axis=1)]
                for p in uv:
                    cv2.circle(vis, (int(p[0]), int(p[1])), 3, (0, 0, 255), -1)
        colors = {0: (0, 255, 0), 1: (0, 200, 255)}  # boundary green, center orange
        for info in res["boundaries"] + res["centers"]:
            xs = np.linspace(max(0.5, info.x0), info.x1, 40)
            ys = np.array([info.y_at(x) for x in xs])
            pts3 = np.stack([xs, ys, np.full_like(xs, self.ground_z)], axis=1)
            uv = self.projector.ground_to_pixels(pts3)
            uv = uv[np.isfinite(uv).all(axis=1)]
            c = colors.get(info.type, (255, 255, 255))
            for j in range(1, len(uv)):
                p1 = (int(uv[j - 1, 0]), int(uv[j - 1, 1]))
                p2 = (int(uv[j, 0]), int(uv[j, 1]))
                cv2.line(vis, p1, p2, c, 2)
        txt = "L:%s  R:%s  src:%s" % (res["left_avail"], res["right_avail"],
                                      self._active_backend)
        cv2.putText(vis, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2)
        try:
            out = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
            out.header = Header(stamp=stamp, frame_id=self.child_frame_id)
            self.pub_vis.publish(out)
        except Exception:
            pass

    def _publish_markers(self, res, stamp):
        arr = MarkerArray()
        mid = 0
        for info in res["boundaries"] + res["centers"]:
            mk = Marker()
            mk.header = Header(stamp=stamp, frame_id=self.child_frame_id)
            mk.ns = "lane_boundary" if info.type == 0 else "lane_center"
            mk.id = mid
            mid += 1
            mk.type = Marker.LINE_STRIP
            mk.action = Marker.ADD
            mk.scale.x = 0.12
            if info.type == 0:
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.1, 0.9, 0.1, 0.9
            else:
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.6, 0.0, 0.9
            xs = np.linspace(max(0.0, info.x0), info.x1, 40)
            for x in xs:
                p = Point()
                p.x = float(x)
                p.y = float(info.y_at(x))
                p.z = 0.0
                mk.points.append(p)
            mk.lifetime = rospy.Duration(0.3)
            arr.markers.append(mk)
        self.pub_markers.publish(arr)


def main():
    rospy.init_node("race_lane_detection")
    if not _HAS_CV:
        rospy.logfatal("[lane_detection] cv_bridge/opencv unavailable; "
                       "install ros-noetic-cv-bridge and python3-opencv.")
        return
    LaneDetectionNode()
    rospy.spin()


if __name__ == "__main__":
    main()

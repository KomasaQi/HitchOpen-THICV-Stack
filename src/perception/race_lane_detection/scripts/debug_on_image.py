#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
debug_on_image.py
-----------------
脱离 ROS 直接在一张图片上跑学习后端并可视化, 用于快速排查检测/IPM问题。

用法:
  rosrun race_lane_detection debug_on_image.py \
      --image /path/to/frame.png \
      --onnx  $(rospack find race_lane_detection)/weights/ufldv2_curvelanes_res34_800x1600.onnx \
      --fov 90 --out /tmp/lane_debug.png

输出图中: 红点=网络原始检测点(反投影回图像), 绿线=拟合车道边界, 橙线=车道中心线。
若红点本身就乱 -> 模型/裁剪/分辨率问题; 红点对但绿线偏 -> IPM外参/ground_z问题。
"""
import argparse, os, sys
import numpy as np

try:
    import cv2
except Exception:
    print("need opencv"); sys.exit(1)

# 允许直接运行(非catkin环境)时找到模块
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
from race_lane_detection.geometry import GroundProjector
from race_lane_detection.ufld_detector import UFLDv2Detector
from race_lane_detection.lane_assembler import assemble


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True)
    ap.add_argument("--onnx", required=True)
    ap.add_argument("--fov", type=float, default=90.0)
    ap.add_argument("--cam_x", type=float, default=3.5)
    ap.add_argument("--cam_y", type=float, default=0.0)
    ap.add_argument("--cam_z", type=float, default=2.0)
    ap.add_argument("--ground_z", type=float, default=0.0)
    ap.add_argument("--dataset", default="auto")
    ap.add_argument("--crop_ratio", type=float, default=-1.0)
    ap.add_argument("--out", default="/tmp/lane_debug.png")
    a = ap.parse_args()

    img = cv2.imread(a.image)
    if img is None:
        print("cannot read", a.image); sys.exit(1)
    h, w = img.shape[:2]
    gp = GroundProjector((a.cam_x, a.cam_y, a.cam_z), (0, 0, 0), a.ground_z)
    gp.set_intrinsics_from_fov(w, h, a.fov)
    cfg = dict(onnx_path=a.onnx, use_gpu=False, dataset=a.dataset)
    if a.crop_ratio >= 0:
        cfg["crop_ratio"] = a.crop_ratio
    det = UFLDv2Detector(gp, cfg)
    if not det.ready:
        print("detector not ready (onnx/onnxruntime?)"); sys.exit(1)
    print("dataset=%s lanes=%d input=%dx%d" %
          (det.dataset, det.num_lanes, det.input_h, det.input_w))

    lanes = det.detect(img)
    print("decoded lanes:", len(lanes))
    res = assemble(lanes, fit_order=3, report_order=5, decimals=6)

    vis = img.copy()
    for pts in lanes:
        uv = gp.ground_to_pixels(np.array(pts))
        uv = uv[np.isfinite(uv).all(1)]
        for p in uv:
            cv2.circle(vis, (int(p[0]), int(p[1])), 3, (0, 0, 255), -1)
    for info in res["boundaries"] + res["centers"]:
        xs = np.linspace(max(0.5, info.x0), info.x1, 40)
        ys = np.array([info.y_at(x) for x in xs])
        uv = gp.ground_to_pixels(np.stack([xs, ys, np.full_like(xs, a.ground_z)], 1))
        uv = uv[np.isfinite(uv).all(1)]
        c = (0, 255, 0) if info.type == 0 else (0, 200, 255)
        for j in range(1, len(uv)):
            cv2.line(vis, (int(uv[j-1, 0]), int(uv[j-1, 1])),
                     (int(uv[j, 0]), int(uv[j, 1])), c, 2)
    cv2.imwrite(a.out, vis)
    print("saved ->", a.out)
    print("boundaries lane_ids:", [b.lane_id for b in res["boundaries"]])
    print("centers lane_ids:", [c.lane_id for c in res["centers"]])


if __name__ == "__main__":
    main()

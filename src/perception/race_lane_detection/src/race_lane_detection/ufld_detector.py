#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ufld_detector.py
----------------
Ultra-Fast-Lane-Detection-v2 (TPAMI 2022) 学习后端，ONNX Runtime 推理。

解码严格遵循官方 pred2coords（row/col 双分支 + 局部 soft-argmax + exist 门控）。
同时支持：
  * CULane 模型:    loc_row [1,200,72,4],  loc_col [1,100,81,4]   (4 车道, 输入 320x1600)
  * CurveLanes 模型: loc_row [1,200,72,10], loc_col [1,100,81,10]  (10车道, 输入 800x1600)

与官方一致的关键点：
  1. 预处理：对原图按 crop_ratio 裁掉顶部，再 resize 到模型输入尺寸；行/列锚以
     “裁剪前原图尺寸 (ori_h, ori_w)” 为基准映射回像素坐标。
  2. row 分支：对每个行锚 k，在 loc_row 的 grid 维取 argmax，再在峰值 ±window 内做
     softmax 期望(+0.5)，得到列位置 u；纵坐标 v = row_anchor[k]*ori_h。
  3. col 分支：对每个列锚 k 取 grid argmax + 局部期望，得到行位置 v；
     横坐标 u = col_anchor[k]*ori_w。
  4. exist 用 argmax(类别维)，并对每条车道做有效点数门控
     (row: > num_cls_row/2; col: > num_cls_col/4)。
  5. 车道索引分支划分按车道数自适应：内侧车道走 row，最外两侧走 col。
"""

import os
import numpy as np

try:
    import cv2
except Exception:
    cv2 = None

IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def _softmax(x, axis=0):
    x = x - np.max(x, axis=axis, keepdims=True)
    e = np.exp(x)
    return e / (np.sum(e, axis=axis, keepdims=True) + 1e-12)


DATASET_PRESETS = {
    "culane": dict(
        crop_ratio=0.6,
        row_anchor=(0.42, 1.0), col_anchor=(0.0, 1.0),
        row_lane_idx=[1, 2], col_lane_idx=[0, 3],
    ),
    "curvelanes": dict(
        crop_ratio=0.8,
        row_anchor=(0.4, 1.0), col_anchor=(0.0, 1.0),
        row_lane_idx=None, col_lane_idx=None,   # 运行时自适应
    ),
}


def _detect_dataset(num_lanes):
    return "curvelanes" if num_lanes >= 8 else "culane"


class UFLDv2Detector(object):
    def __init__(self, projector, cfg=None):
        self.gp = projector
        cfg = cfg or {}
        self.onnx_path = cfg.get("onnx_path", "")
        self.dataset = cfg.get("dataset", "auto")
        self.local_window = int(cfg.get("local_window", 32))
        self.min_lane_pts = int(cfg.get("min_lane_pts", 6))
        self.ori_w = 0
        self.ori_h = 0
        self.sess = None
        self.input_name = None
        self.out_names = None
        self._provider = "none"
        self._cfg = cfg
        self._init_session(cfg)

    def _init_session(self, cfg):
        if not self.onnx_path or not os.path.isfile(self.onnx_path):
            return
        try:
            import onnxruntime as ort
        except Exception:
            return
        providers = []
        if cfg.get("use_gpu", True) and \
                "CUDAExecutionProvider" in ort.get_available_providers():
            providers.append("CUDAExecutionProvider")
        providers.append("CPUExecutionProvider")
        try:
            self.sess = ort.InferenceSession(self.onnx_path, providers=providers)
            self.input_name = self.sess.get_inputs()[0].name
            self.out_names = [o.name for o in self.sess.get_outputs()]
            ishape = self.sess.get_inputs()[0].shape
            self.input_h = int(ishape[2])
            self.input_w = int(ishape[3])
            self._provider = self.sess.get_providers()[0]
            self._configure_from_output()
        except Exception:
            self.sess = None

    def _configure_from_output(self):
        shapes = {o.name: o.shape for o in self.sess.get_outputs()}
        lr = shapes.get("loc_row")
        lc = shapes.get("loc_col")
        if lr is None or lc is None:
            for n, s in shapes.items():
                if len(s) == 4 and int(s[1]) >= 150:
                    lr = s
                elif len(s) == 4 and 50 <= int(s[1]) < 150:
                    lc = s
        self.num_grid_row = int(lr[1])
        self.num_row = int(lr[2])
        self.num_lanes = int(lr[3])
        self.num_grid_col = int(lc[1])
        self.num_col = int(lc[2])

        ds = self.dataset
        if ds == "auto":
            ds = _detect_dataset(self.num_lanes)
        preset = DATASET_PRESETS.get(ds, DATASET_PRESETS["culane"])
        self.dataset = ds
        ra = preset["row_anchor"]
        ca = preset["col_anchor"]
        self.row_anchor = np.linspace(ra[0], ra[1], self.num_row)
        self.col_anchor = np.linspace(ca[0], ca[1], self.num_col)
        self.crop_ratio = self._cfg.get("crop_ratio", preset["crop_ratio"])

        ri = preset["row_lane_idx"]
        ci = preset["col_lane_idx"]
        if ri is None or ci is None:
            L = self.num_lanes
            ci = [0, L - 1]
            ri = [j for j in range(L) if j not in ci]
        self.row_lane_idx = [j for j in ri if j < self.num_lanes]
        self.col_lane_idx = [j for j in ci if j < self.num_lanes]

    @property
    def ready(self):
        return self.sess is not None and self.gp.ready

    def _preprocess(self, img_bgr):
        h, w = img_bgr.shape[:2]
        self.ori_h, self.ori_w = h, w
        cut_h_ori = int(h * (1.0 - self.crop_ratio))
        crop = img_bgr[cut_h_ori:, :, :]
        rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
        rsz = cv2.resize(rgb, (self.input_w, self.input_h),
                         interpolation=cv2.INTER_CUBIC)
        x = rsz.astype(np.float32) / 255.0
        x = (x - IMAGENET_MEAN) / IMAGENET_STD
        x = np.transpose(x, (2, 0, 1))[None, ...].astype(np.float32)
        return np.ascontiguousarray(x)

    def _local_expect(self, loc_lane_k, peak, num_grid):
        lo = max(0, peak - self.local_window)
        hi = min(num_grid - 1, peak + self.local_window) + 1
        idx = np.arange(lo, hi)
        seg = loc_lane_k[lo:hi]
        p = _softmax(seg, axis=0)
        return float(np.sum(p * idx) + 0.5)

    def _decode(self, preds):
        loc_row = preds["loc_row"][0]
        loc_col = preds["loc_col"][0]
        exist_row = preds["exist_row"][0]
        exist_col = preds["exist_col"][0]

        max_row = loc_row.argmax(0)
        max_col = loc_col.argmax(0)
        valid_row = exist_row.argmax(0)
        valid_col = exist_col.argmax(0)

        Gr = self.num_grid_row
        Gc = self.num_grid_col
        lanes = []

        for i in self.row_lane_idx:
            if i >= loc_row.shape[2]:
                continue
            if valid_row[:, i].sum() <= self.num_row / 2.0:
                continue
            us, vs = [], []
            for k in range(self.num_row):
                if not valid_row[k, i]:
                    continue
                u = self._local_expect(loc_row[:, k, i], int(max_row[k, i]), Gr)
                u = u / (Gr - 1) * self.ori_w
                v = self.row_anchor[k] * self.ori_h
                us.append(u); vs.append(v)
            if len(us) >= 2:
                lanes.append(np.stack([np.array(us), np.array(vs)], axis=1))

        for i in self.col_lane_idx:
            if i >= loc_col.shape[2]:
                continue
            if valid_col[:, i].sum() <= self.num_col / 4.0:
                continue
            us, vs = [], []
            for k in range(self.num_col):
                if not valid_col[k, i]:
                    continue
                v = self._local_expect(loc_col[:, k, i], int(max_col[k, i]), Gc)
                v = v / (Gc - 1) * self.ori_h
                u = self.col_anchor[k] * self.ori_w
                us.append(u); vs.append(v)
            if len(us) >= 2:
                lanes.append(np.stack([np.array(us), np.array(vs)], axis=1))

        return lanes

    def detect(self, img_bgr):
        if not self.ready or cv2 is None:
            return []
        x = self._preprocess(img_bgr)
        outs = self.sess.run(self.out_names, {self.input_name: x})
        preds = {n: o for n, o in zip(self.out_names, outs)}
        if "loc_row" not in preds:
            return []
        lanes_px = self._decode(preds)
        lanes_ego = []
        for px in lanes_px:
            if px.shape[0] < self.min_lane_pts:
                continue
            pts = self.gp.pixels_to_ground(px)
            pts = pts[np.isfinite(pts).all(axis=1)]
            pts = pts[(pts[:, 0] > 1.0) & (pts[:, 0] < 80.0) &
                      (np.abs(pts[:, 1]) < 20.0)]
            if pts.shape[0] >= 2:
                lanes_ego.append(pts)
        return lanes_ego

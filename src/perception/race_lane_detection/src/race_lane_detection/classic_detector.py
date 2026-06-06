#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
classic_detector.py
-------------------
基于规则的车道线检测后端（无需 GPU，始终可用，作为学习后端的兜底）。
流程：颜色/梯度阈值 -> 逆透视到 BEV 俯视图 -> 直方图找基 -> 滑窗追踪 ->
逐车道提取像素 -> 反投影回自车地面系 -> 由上层做多项式拟合。

该后端直接在“自车地面系的鸟瞰网格”里做滑窗，避免显式标定 BEV 单应：
我们用 GroundProjector 把一组规则地面采样网格投到图像取样，从而得到 BEV。
"""

import numpy as np

try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None


class ClassicLaneDetector(object):
    def __init__(self, projector, cfg=None):
        self.gp = projector
        cfg = cfg or {}
        # BEV 采样范围（自车地面系，米）
        self.x_min = cfg.get("bev_x_min", 4.0)
        self.x_max = cfg.get("bev_x_max", 45.0)
        self.y_abs = cfg.get("bev_y_abs", 12.0)   # 左右各 12m
        self.res = cfg.get("bev_res", 0.10)        # 每像素 0.1m
        self.n_windows = cfg.get("n_windows", 18)
        self.margin = cfg.get("win_margin", 12)    # 滑窗半宽(BEV像素)
        self.minpix = cfg.get("win_minpix", 25)
        self.lane_width = cfg.get("lane_width", 3.5)
        self._build_bev_grid()

    def _build_bev_grid(self):
        self.bev_w = int(round(2 * self.y_abs / self.res))
        self.bev_h = int(round((self.x_max - self.x_min) / self.res))
        # BEV 行 r -> 自车 x（远处在上方，r=0 对应 x_max）
        xs = self.x_max - (np.arange(self.bev_h) + 0.5) * self.res
        # BEV 列 c -> 自车 y（左为正，c=0 对应 y=+y_abs）
        ys = self.y_abs - (np.arange(self.bev_w) + 0.5) * self.res
        gx, gy = np.meshgrid(xs, ys, indexing="ij")     # (H,W)
        gz = np.zeros_like(gx)
        grid = np.stack([gx.ravel(), gy.ravel(), gz.ravel()], axis=1)
        self._bev_grid_ego = grid
        self._bev_shape = (self.bev_h, self.bev_w)
        self._bev_grid_valid = None  # lazy until intrinsics ready

    def _bev_map(self):
        """返回把图像 remap 成 BEV 所需的 map_x, map_y。依赖内参已就绪。"""
        uv = self.gp.ground_to_pixels(self._bev_grid_ego)
        mapx = uv[:, 0].reshape(self._bev_shape).astype(np.float32)
        mapy = uv[:, 1].reshape(self._bev_shape).astype(np.float32)
        return mapx, mapy

    def _threshold(self, img_bgr):
        """车道线像素阈值：白/黄 颜色 + Sobel 梯度。返回 0/255 单通道。"""
        hls = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HLS)
        l = hls[:, :, 1]
        s = hls[:, :, 2]
        # 亮度高（白线）
        white = (l > 160).astype(np.uint8)
        # 黄色：HSV 更稳
        hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
        yellow = cv2.inRange(hsv, (15, 60, 80), (40, 255, 255)) > 0
        # 梯度
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        sx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sx = np.abs(sx)
        sx = np.uint8(255 * sx / (sx.max() + 1e-6))
        grad = (sx > 35)
        mask = (white.astype(bool) | yellow | grad).astype(np.uint8) * 255
        return mask

    def detect(self, img_bgr):
        """
        返回 list[ np.ndarray(M,3) ]：每条车道线的自车地面系采样点。
        """
        if cv2 is None or not self.gp.ready:
            return []
        mask = self._threshold(img_bgr)
        mapx, mapy = self._bev_map()
        bev = cv2.remap(mask, mapx, mapy, interpolation=cv2.INTER_NEAREST,
                        borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        H, W = bev.shape
        # 直方图找车道基：取下半部分（近处）列和
        hist = np.sum(bev[H // 2:, :] > 0, axis=0).astype(np.float64)
        if hist.sum() < 1:
            return []
        bases = self._find_peaks(hist, min_dist=int(self.lane_width / self.res * 0.6))
        nz = bev.nonzero()
        nzy = np.array(nz[0])  # row
        nzx = np.array(nz[1])  # col
        lanes_px = []
        win_h = H // self.n_windows
        for base in bases:
            cur = base
            idxs = []
            for w in range(self.n_windows):
                ylo = H - (w + 1) * win_h
                yhi = H - w * win_h
                xlo = int(cur - self.margin)
                xhi = int(cur + self.margin)
                good = ((nzy >= ylo) & (nzy < yhi) &
                        (nzx >= xlo) & (nzx < xhi)).nonzero()[0]
                if good.size:
                    idxs.append(good)
                    if good.size > self.minpix:
                        cur = int(np.mean(nzx[good]))
            if not idxs:
                continue
            idxs = np.concatenate(idxs)
            if idxs.size < self.minpix:
                continue
            rows = nzy[idxs]
            cols = nzx[idxs]
            # BEV 像素 -> 自车地面系
            ego_x = self.x_max - (rows + 0.5) * self.res
            ego_y = self.y_abs - (cols + 0.5) * self.res
            pts = np.stack([ego_x, ego_y, np.zeros_like(ego_x)], axis=1)
            lanes_px.append(pts)
        return lanes_px

    @staticmethod
    def _find_peaks(hist, min_dist=20, max_lanes=6):
        peaks = []
        h = hist.copy()
        thr = max(hist.max() * 0.25, 3.0)
        while len(peaks) < max_lanes:
            i = int(np.argmax(h))
            if h[i] < thr:
                break
            peaks.append(i)
            lo = max(0, i - min_dist)
            hi = min(len(h), i + min_dist)
            h[lo:hi] = 0
        return sorted(peaks)

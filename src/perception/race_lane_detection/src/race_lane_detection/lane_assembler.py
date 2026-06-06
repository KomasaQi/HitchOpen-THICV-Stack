#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lane_assembler.py
-----------------
把检测后端给出的“一组自车地面系车道线点集”整理为结构化结果：
  * 按横向位置从左到右排序车道边界线，并赋 lane_id（0=自车道左边线,1=自车道右边线...）
  * 由相邻边界线中点生成车道中心线
  * 标定自车所在车道中心线索引
  * 给出左右换道可行性（基于是否存在相邻车道及其检测质量）
  * 计算自车相对自车道中心线的横向偏移与航向偏差
"""

import numpy as np
from race_lane_detection.geometry import fit_poly_xy, pad_coeffs


class LaneInfo(object):
    def __init__(self, coeffs_lo, order, rmse, x0, x1, n, conf, pts, ltype, lane_id):
        self.coeffs_lo = coeffs_lo
        self.order = order
        self.rmse = rmse
        self.x0 = x0
        self.x1 = x1
        self.n = n
        self.conf = conf
        self.pts = pts          # (M,3) ego
        self.type = ltype       # 0=boundary,1=center
        self.lane_id = lane_id

    def y_at(self, x):
        c = self.coeffs_lo
        return sum(c[i] * (x ** i) for i in range(len(c)))


def _lateral_at(pts, x_ref):
    """估计点集在 x≈x_ref 处的横向 y，用于排序。"""
    x = pts[:, 0]
    y = pts[:, 1]
    j = int(np.argmin(np.abs(x - x_ref)))
    return float(y[j])


def assemble(lanes_ego, fit_order=3, max_order=5, report_order=5,
             decimals=6, lane_width=3.5, x_ref=8.0,
             min_points=4, min_conf=0.25):
    """
    输入: lanes_ego = list[(M,3)]  自车地面系车道线点
    返回: dict(boundaries=[LaneInfo], centers=[LaneInfo], ego_center_index,
              left_avail, right_avail, lat_offset, head_err)
    """
    fitted = []
    for pts in lanes_ego:
        pts = np.asarray(pts, dtype=np.float64)
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.shape[0] < min_points:
            continue
        r = fit_poly_xy(pts, order=fit_order, max_order=max_order)
        if r is None:
            continue
        coeffs_lo, used_order, rmse, x0, x1, n = r
        span = max(1e-3, x1 - x0)
        # 置信度：点数 + 跨度 + 拟合残差 的简单综合
        conf = (
            min(1.0, n / 30.0) * 0.4 +
            min(1.0, span / 30.0) * 0.3 +
            max(0.0, 1.0 - rmse / 0.6) * 0.3
        )
        if conf < min_conf:
            continue
        ylat = _lateral_at(pts, x_ref)
        fitted.append((ylat, coeffs_lo, used_order, rmse, x0, x1, n, conf, pts))

    if not fitted:
        return dict(boundaries=[], centers=[], ego_center_index=-1,
                    left_avail=False, right_avail=False,
                    lat_offset=0.0, head_err=0.0)

    # 从左(y大)到右(y小)排序
    fitted.sort(key=lambda t: -t[0])

    # 找出夹住自车(y=0)的两条边界，自车道左/右边线
    ylats = [t[0] for t in fitted]
    # 自车道左边线 = 最接近且 y>=0 中最小者；右边线 = 最接近且 y<0 中最大者
    left_idx = None
    right_idx = None
    for i, yl in enumerate(ylats):
        if yl >= 0 and (left_idx is None or yl < ylats[left_idx]):
            left_idx = i
        if yl < 0 and (right_idx is None or yl > ylats[right_idx]):
            right_idx = i
    # 若全在一侧，退化处理：取最接近 0 的作为参考
    if left_idx is None and right_idx is None:
        left_idx = int(np.argmin(np.abs(ylats)))

    # 赋 lane_id：自车道左边线=0，右边线=1，向左递减，向右递增
    boundaries = []
    base = left_idx if left_idx is not None else right_idx
    for i, t in enumerate(fitted):
        ylat, coeffs_lo, used_order, rmse, x0, x1, n, conf, pts = t
        if left_idx is not None:
            lane_id = i - left_idx  # left_idx -> 0
        else:
            lane_id = i - right_idx + 1
        b = LaneInfo(pad_coeffs(coeffs_lo, report_order, decimals),
                     used_order, rmse, x0, x1, n, round(conf, 4),
                     pts, 0, int(lane_id))
        boundaries.append(b)

    # 由相邻边界线生成中心线
    centers = []
    for i in range(len(boundaries) - 1):
        bL = boundaries[i]
        bR = boundaries[i + 1]
        x0 = max(bL.x0, bR.x0)
        x1 = min(bL.x1, bR.x1)
        if x1 - x0 < 2.0:
            continue
        xs = np.linspace(x0, x1, 40)
        ys = 0.5 * (np.array([bL.y_at(x) for x in xs]) +
                    np.array([bR.y_at(x) for x in xs]))
        cpts = np.stack([xs, ys, np.zeros_like(xs)], axis=1)
        r = fit_poly_xy(cpts, order=fit_order, max_order=max_order)
        if r is None:
            continue
        c_lo, c_ord, c_rmse, cx0, cx1, cn = r
        # 中心线 lane_id：自车道(夹住y=0)记 0，向左负，向右正
        cid = bL.lane_id  # 左边界 lane_id 即该车道编号(0为自车道)
        conf = round(0.5 * (bL.conf + bR.conf), 4)
        centers.append(LaneInfo(pad_coeffs(c_lo, report_order, decimals),
                                c_ord, c_rmse, cx0, cx1, cn, conf,
                                cpts, 1, int(cid)))

    # 自车道中心线索引：lane_id==0
    ego_center_index = -1
    for k, c in enumerate(centers):
        if c.lane_id == 0:
            ego_center_index = k
            break
    if ego_center_index < 0 and centers:
        # 退化：取中心 y 最接近 0 者
        ego_center_index = int(np.argmin([abs(c.y_at(x_ref)) for c in centers]))
        centers[ego_center_index].lane_id = 0

    # 换道可行性：存在 lane_id<0 / >0 的中心线且置信度达标
    left_avail = any(c.lane_id < 0 and c.conf > 0.35 for c in centers)
    right_avail = any(c.lane_id > 0 and c.conf > 0.35 for c in centers)

    # 自车相对自车道中心线的横向偏移(左正)与航向误差(左正)
    lat_offset = 0.0
    head_err = 0.0
    if ego_center_index >= 0:
        ec = centers[ego_center_index]
        lat_offset = float(ec.y_at(0.0))           # x=0 处中心线横向位置即 -自车偏移
        # 中心线在前方的切线方向相对车体x轴的夹角
        c = ec.coeffs_lo
        dydx = c[1] if len(c) > 1 else 0.0          # y'(0)
        head_err = float(np.arctan(dydx))
    return dict(boundaries=boundaries, centers=centers,
                ego_center_index=ego_center_index,
                left_avail=left_avail, right_avail=right_avail,
                lat_offset=lat_offset, head_err=head_err)

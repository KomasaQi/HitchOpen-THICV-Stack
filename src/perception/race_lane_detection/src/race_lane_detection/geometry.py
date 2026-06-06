#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
geometry.py
-----------
图像像素 <-> 自车地面坐标系(FLU: x前 y左 z上) 之间的逆透视变换(IPM)。

约定（与 CARLA ros-bridge 一致）：
  * 相机外参：安装于自车系 (x,y,z)=(3.5,0,2.0)，pitch=roll=yaw=0，朝向前方。
  * CARLA RGB 相机内部已将光学坐标对齐到标准 CV 约定，其相对车体的旋转为：
        optical_x = -body_y (指向图像右)
        optical_y = -body_z (指向图像下)
        optical_z =  body_x (指向前方/光轴)
  * 内参由 camera_info 提供（K 矩阵）；若无则按 fov 计算。

地面平面假设 z=0（相对自车，路面与自车底盘同高的近似；CARLA 中可用相机高度修正）。
"""

import numpy as np


# 自车体坐标 -> 相机光学坐标 的旋转（pitch=roll=yaw=0 时为常量）
R_BODY2OPT = np.array([
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
    [1.0, 0.0, 0.0],
], dtype=np.float64)


class GroundProjector(object):
    """
    基于针孔模型 + 平地假设，将图像像素反投影到自车地面坐标系。
    """

    def __init__(self, cam_xyz=(3.5, 0.0, 2.0),
                 cam_rpy=(0.0, 0.0, 0.0),
                 ground_z=0.0):
        self.C = np.array(cam_xyz, dtype=np.float64)        # 相机在自车系中的位置
        self.ground_z = float(ground_z)
        self.set_rpy(cam_rpy)
        self.K = None
        self.K_inv = None
        self.width = None
        self.height = None

    # ------------------------------------------------------------------ #
    def set_rpy(self, rpy):
        """设置相机相对自车的安装姿态(roll,pitch,yaw)，构建 body->optical 旋转。"""
        roll, pitch, yaw = [float(a) for a in rpy]
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        # body 安装旋转 R_mount (ego -> 安装后的相机车体系)，ZYX
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
        R_mount = Rz @ Ry @ Rx
        # ego -> optical:  先按安装姿态逆旋到相机车体，再 body->optical
        self.R_ego2opt = R_BODY2OPT @ R_mount.T
        self.R_opt2ego = self.R_ego2opt.T

    def set_intrinsics_from_K(self, K, width=None, height=None):
        self.K = np.array(K, dtype=np.float64).reshape(3, 3)
        self.K_inv = np.linalg.inv(self.K)
        if width is not None:
            self.width = int(width)
        if height is not None:
            self.height = int(height)

    def set_intrinsics_from_fov(self, width, height, fov_deg=90.0):
        f = width / (2.0 * np.tan(np.deg2rad(fov_deg) / 2.0))
        K = np.array([[f, 0, width / 2.0],
                      [0, f, height / 2.0],
                      [0, 0, 1.0]], dtype=np.float64)
        self.set_intrinsics_from_K(K, width, height)

    @property
    def ready(self):
        return self.K_inv is not None

    # ------------------------------------------------------------------ #
    def pixels_to_ground(self, uv):
        """
        将一批像素 (N,2)[u,v] 反投影到自车地面平面 z=ground_z。
        返回 (N,3) 自车系坐标；位于相机后方或地平线以上的点返回 NaN。
        """
        uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
        n = uv.shape[0]
        ones = np.ones((n, 1))
        pix_h = np.hstack([uv, ones])                       # (N,3)
        # 光学系下的方向向量（未归一化）
        rays_opt = (self.K_inv @ pix_h.T).T                 # (N,3)
        # 转到自车系方向
        rays_ego = (self.R_opt2ego @ rays_opt.T).T          # (N,3)
        # 射线参数化: P = C + t * dir, 求与 z=ground_z 的交点
        dz = rays_ego[:, 2]
        t = (self.ground_z - self.C[2]) / dz
        valid = (t > 0) & np.isfinite(t)
        pts = self.C[None, :] + t[:, None] * rays_ego
        pts[~valid] = np.nan
        return pts

    def ground_to_pixels(self, pts_ego):
        """自车系地面点 (N,3) -> 像素 (N,2)。光轴后方点返回 NaN。"""
        pts_ego = np.asarray(pts_ego, dtype=np.float64).reshape(-1, 3)
        rel = pts_ego - self.C[None, :]
        p_opt = (self.R_ego2opt @ rel.T).T
        z = p_opt[:, 2]
        uvw = (self.K @ p_opt.T).T
        out = np.full((pts_ego.shape[0], 2), np.nan)
        ok = z > 1e-6
        out[ok, 0] = uvw[ok, 0] / uvw[ok, 2]
        out[ok, 1] = uvw[ok, 1] / uvw[ok, 2]
        return out


def fit_poly_xy(pts_ego, order=3, max_order=5):
    """
    用 y(x) 多项式拟合自车系点集。x=前向, y=左向。
    返回 (coeffs_low_to_high, used_order, rmse, x_start, x_end, n_used)。
    coeffs 长度 = used_order+1，低次在前。
    """
    pts = np.asarray(pts_ego, dtype=np.float64)
    pts = pts[np.isfinite(pts).all(axis=1)]
    if pts.shape[0] < 2:
        return None
    x = pts[:, 0]
    y = pts[:, 1]
    # 去重排序，避免病态
    idx = np.argsort(x)
    x, y = x[idx], y[idx]
    order = int(max(1, min(order, max_order)))
    # 阶数不能超过 (点数-1)
    use_order = int(min(order, max(1, len(x) - 1)))
    # numpy.polyfit 返回高次在前；转成低次在前
    try:
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", np.RankWarning)
            coeffs_hi = np.polyfit(x, y, use_order)
    except Exception:
        return None
    coeffs_lo = coeffs_hi[::-1].copy()
    y_hat = np.polyval(coeffs_hi, x)
    rmse = float(np.sqrt(np.mean((y_hat - y) ** 2)))
    return coeffs_lo, use_order, rmse, float(x.min()), float(x.max()), int(len(x))


def pad_coeffs(coeffs_lo, report_order=5, decimals=6):
    """将系数补零到 report_order+1 长度并按指定小数位四舍五入。低次在前。"""
    out = np.zeros(report_order + 1, dtype=np.float64)
    n = min(len(coeffs_lo), report_order + 1)
    out[:n] = coeffs_lo[:n]
    return [round(float(c), decimals) for c in out]

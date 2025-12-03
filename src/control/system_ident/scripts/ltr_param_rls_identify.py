#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化版 LTR-RLS 辨识节点
核心：yk = LTR，仅辨识 LTR 与输入 u（vx*dtheta）的关系
不输出 a1/a2/b1，仅输出离散系数和传递函数
支持算法：经典/平方根/指数遗忘/变量遗忘RLS
支持自定义 AR（LTR历史阶数）、X（u历史阶数）
"""
import numpy as np
import rospy
from collections import deque
from race_msgs.msg import VehicleStatus
from d2c_library import diff_eq_to_continuous_tf as d2c

# -------------------------- 1. ROS参数配置（保留核心，移除dLTR相关） --------------------------
def get_ros_params():
    rospy.init_node("ltr_param_rls_simple", anonymous=True)
    params = {
        # 核心配置
        "rls_method": rospy.get_param("~rls_method", "classic").lower(),
        "n_ar": int(rospy.get_param("~n_ar", 2)),  # LTR历史阶数（AR阶数）
        "n_x": int(rospy.get_param("~n_x", 2)),  # u历史阶数（X阶数）
        "Ts": float(rospy.get_param("~Ts", 0.025)),  # 采样步长
        
        # RLS算法参数
        "init_P_gain": float(rospy.get_param("~init_P_gain", 2000.0)),
        "rho_exp": float(rospy.get_param("~rho_exp", 0.99)),
        
        # 变量遗忘RLS专属参数
        "var_N": int(rospy.get_param("~var_N", 500)),
        "var_Nmin": int(rospy.get_param("~var_Nmin", 50)),
        "var_sig02": float(rospy.get_param("~var_sig02", 0.02)),
    }
    # 合法性校验
    params["rls_method"] = params["rls_method"] if params["rls_method"] in ["classic", "sqrt", "exp", "var"] else "classic"
    params["n_ar"] = max(1, params["n_ar"])
    params["n_x"] = max(1, params["n_x"])
    params["Ts"] = max(0.001, params["Ts"])
    return params

# -------------------------- 2. 全局变量初始化（移除dLTR相关） --------------------------
params = get_ros_params()
rls_method = params["rls_method"]
n_ar = params["n_ar"]  # AR阶数：y(k-1), y(k-2), ..., y(k-n_ar)
n_x = params["n_x"]    # X阶数：u(k-1), u(k-2), ..., u(k-n_x)
Ts = params["Ts"]

phi_dim = n_ar + n_x  # 回归向量维度：AR阶数 + X阶数（仅y历史+u历史）

# 数据缓存：仅保留LTR（y）和u的历史，长度=max(AR,X)+1（确保取到k-1~k-n）
max_cache_len = max(n_ar, n_x) + 1
y_cache = deque(maxlen=max_cache_len)  # y=LTR的历史缓存
u_cache = deque(maxlen=max_cache_len)  # u=vx*dtheta的历史缓存

# RLS核心状态变量
theta0 = np.zeros((phi_dim, 1), dtype=np.float64)  # 待辨识参数（离散系数）
P0 = np.eye(phi_dim, dtype=np.float64) * params["init_P_gain"]
S0 = np.linalg.cholesky(P0) if rls_method == "sqrt" else None
rho = params["rho_exp"] if rls_method in ["exp", "var"] else 0.0
Sigma = params["var_sig02"] * params["var_N"] if rls_method == "var" else 0.0
rhomin = 1 - 1/params["var_Nmin"] if rls_method == "var" else 0.0

# -------------------------- 3. 工具函数（传递函数格式化，保留） --------------------------
def tf_to_string(num, den):
    def coef_to_poly(coefs):
        terms = []
        for i, c in enumerate(coefs):
            c_rounded = round(c, 4)
            if c_rounded == 0:
                continue
            sign = "+" if c_rounded > 0 and i > 0 else ""
            if i == 0:
                term = f"{sign}{c_rounded}"
            elif i == 1:
                term = f"{sign}{c_rounded}s"
            else:
                term = f"{sign}{c_rounded}s^{i}"
            terms.append(term)
        return " ".join(terms) if terms else "0"
    num_str = coef_to_poly(num)
    den_str = coef_to_poly(den)
    return f"G(s) = {num_str} / {den_str}"

# -------------------------- 4. RLS核心算法（无修改，仅适配新的phi维度） --------------------------
def classic_rls(phi, yk):
    global theta0, P0
    phi = phi.reshape(-1, 1)
    denominator = float(phi.T @ P0 @ phi + 1)
    K = P0 @ phi / denominator
    theta = theta0 + K * (yk - float(phi.T @ theta0))
    P = P0 - P0 @ phi @ phi.T @ P0 / denominator
    theta0 = theta
    P0 = P
    return theta

def sqrt_rls(phi, yk):
    global theta0, S0
    phi = phi.reshape(-1, 1)
    f = S0.T @ phi
    g = float(1 / (f.T @ f + 1))
    a = float(1 / (1 + np.sqrt(g)))
    K = S0 @ f * g
    theta = theta0 + K * (yk - float(phi.T @ theta0))
    S = S0 @ (np.eye(phi_dim) - g * a * (f @ f.T))
    theta0 = theta
    S0 = S
    return theta

def exp_rls(phi, yk):
    global theta0, P0, rho
    phi = phi.reshape(-1, 1)
    denominator = float(phi.T @ P0 @ phi + rho)
    K = P0 @ phi / denominator
    theta = theta0 + K * (yk - float(phi.T @ theta0))
    P = (P0 - P0 @ phi @ phi.T @ P0 / (float(phi.T @ P0 @ phi) + 1)) / rho
    theta0 = theta
    P0 = P
    return theta

def var_rls(phi, yk):
    global theta0, P0, rho, Sigma, rhomin
    phi = phi.reshape(-1, 1)
    ek = yk - float(phi.T @ theta0)
    K = P0 @ phi / (float(phi.T @ P0 @ phi) + 1)
    theta = theta0 + K * ek
    rho = float(1 - (1 - float(phi.T @ K)) * (ek ** 2) / Sigma)
    rho = max(rho, rhomin)
    P = (P0 - P0 @ phi @ phi.T @ P0 / (float(phi.T @ P0 @ phi) + 1)) / rho
    theta0 = theta
    P0 = P
    return theta

# -------------------------- 5. 离散系数构造（适配y=LTR与u的关系） --------------------------
def build_discrete_coefs(theta):
    """
    构造差分方程系数（适配d2c函数）
    差分方程形式：y(k) - a1*y(k-1) - a2*y(k-2) - ... = b1*u(k-1) + b2*u(k-2) + ...
    a系数：[1, -a1, -a2, ...]（长度=n_ar+1）
    b系数：[0, b1, b2, ...]（长度=n_x+1）
    其中：theta = [a1, a2, ..., b1, b2, ...]（前n_ar个是y历史系数，后n_x个是u历史系数）
    """
    a = [1.0]  # a0=1
    # 前n_ar个参数是y(k-1), y(k-2)...的系数（a1, a2...），对应a系数为 -a1, -a2...
    for i in range(n_ar):
        a.append(-theta[i, 0])
    # 后n_x个参数是u(k-1), u(k-2)...的系数（b1, b2...），对应b系数为 b1, b2...
    b = [0.0]  # b0=0（无u(k)项）
    for i in range(n_x):
        b.append(theta[n_ar + i, 0])
    return a, b

# -------------------------- 6. ROS回调函数（核心修改：仅处理LTR和u） --------------------------
def vehicle_state_callback(msg):
    # 1. 仅提取LTR（yk=y）和输入u，移除dLTR相关
    yk = msg.ltr_state.ltr  # 当前输出：y(k) = LTR
    vx = msg.vel.linear.x
    dtheta = msg.vel.angular.z
    u = vx * dtheta  # 当前输入：u(k) = vx*dtheta

    # 2. 数据有效性校验
    if np.isnan(yk) or np.isnan(u):
        rospy.logwarn(f"[{rls_method}-RLS] 收到NaN数据，跳过")
        return
    if abs(vx) > 50:
        rospy.logwarn(f"[{rls_method}-RLS] 车速异常：{vx:.2f}m/s，跳过")
        return

    # 3. 更新缓存（仅y和u）
    y_cache.append(yk)
    u_cache.append(u)

    # 4. 缓存足够时构造回归向量（需取到y(k-1)~y(k-n_ar)和u(k-1)~u(k-n_x)）
    if len(y_cache) < max_cache_len or len(u_cache) < max_cache_len:
        return

    # 5. 构造回归向量phi = [y(k-1), y(k-2), ..., y(k-n_ar), u(k-1), u(k-2), ..., u(k-n_x)]
    phi = []
    # 添加y的历史项（k-1, k-2, ..., k-n_ar）：缓存倒数第2到倒数第n_ar+1个元素
    for i in range(1, n_ar + 1):
        phi.append(y_cache[-i-1])
    # 添加u的历史项（k-1, k-2, ..., k-n_x）：缓存倒数第2到倒数第n_x+1个元素
    for i in range(1, n_x + 1):
        phi.append(u_cache[-i-1])
    phi = np.array(phi, dtype=np.float64)

    # 6. 调用RLS算法更新参数
    try:
        if rls_method == "classic":
            theta = classic_rls(phi, yk)
        elif rls_method == "sqrt":
            theta = sqrt_rls(phi, yk)
        elif rls_method == "exp":
            theta = exp_rls(phi, yk)
        elif rls_method == "var":
            theta = var_rls(phi, yk)
        else:
            theta = theta0
    except Exception as e:
        rospy.logerr(f"[{rls_method}-RLS] 算法错误：{str(e)}")
        return

    # 7. 构造离散系数，转换为传递函数
    tf_str = "传递函数转换失败"
    try:
        a_discrete, b_discrete = build_discrete_coefs(theta)
        num, den = d2c(a_discrete, b_discrete, Ts)
        tf_str = tf_to_string(num, den)
    except Exception as e:
        rospy.logwarn(f"[{rls_method}-RLS] 传递函数转换错误：{str(e)}")

    # 8. 简化输出：仅打印关键信息（无a1/a2/b1）
    log_msg = f"[{rls_method}-RLS] 时间戳: {msg.header.stamp.to_sec():.3f} | "
    log_msg += f"y(k)={yk:.4f} | u(k)={u:.4f} | "
    if rls_method in ["exp", "var"]:
        log_msg += f"rho={float(rho):.4f} | "
    log_msg += f"参数个数={phi_dim}\n"
    log_msg += f"[{rls_method}-RLS] 离散差分方程系数：\n"
    log_msg += f"  a = {[round(c,4) for c in a_discrete]} （y(k) - a1*y(k-1) - ... = b1*u(k-1) + ...）\n"
    log_msg += f"  b = {[round(c,4) for c in b_discrete]}\n"
    log_msg += f"[{rls_method}-RLS] {tf_str}"
    rospy.loginfo(log_msg)

# -------------------------- 7. 主函数（简化启动信息） --------------------------
if __name__ == "__main__":
    rospy.loginfo("="*60)
    rospy.loginfo(f"简化版 LTR-RLS 辨识节点启动！")
    rospy.loginfo(f"核心配置：")
    rospy.loginfo(f"  RLS算法：{rls_method}")
    rospy.loginfo(f"  AR阶数（y历史项数）：{n_ar} | X阶数（u历史项数）：{n_x}")
    rospy.loginfo(f"  采样步长：{Ts:.3f}s | 回归向量维度（参数个数）：{phi_dim}")
    rospy.loginfo(f"  初始P增益：{params['init_P_gain']}")
    if rls_method == "exp":
        rospy.loginfo(f"  指数遗忘因子：rho={float(rho):.4f}")
    elif rls_method == "var":
        rospy.loginfo(f"  变量遗忘：N={params['var_N']}, Nmin={params['var_Nmin']}, sig02={params['var_sig02']}")
    rospy.loginfo("="*60)
    rospy.loginfo(f"订阅话题：/race/vehicle_state（仅提取 LTR 和 vx*dtheta）...")

    # 订阅话题
    rospy.Subscriber(
        "/race/vehicle_state",
        VehicleStatus,
        vehicle_state_callback,
        queue_size=30
    )

    rospy.spin()
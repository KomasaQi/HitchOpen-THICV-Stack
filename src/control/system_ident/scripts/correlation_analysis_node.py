#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Real-time Correlation Analysis ROS Node
Function: Plot ACF/PACF of LTR and CCF between LTR and u(vx*dtheta) in a sliding window
Configurable: Window duration, max lag order, update frequency
Stable Version: Fix animation return value + attribute error
"""
import numpy as np
import rospy
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
from threading import Lock
from race_msgs.msg import VehicleStatus
import statsmodels.api as sm

# -------------------------- 1. ROS Parameter Configuration --------------------------
def get_ros_params():
    params = {
        "window_duration": rospy.get_param("~window_duration", 30.0),  # Sliding window duration (s)
        "max_lag": rospy.get_param("~max_lag", 20),                   # Max lag order (user config)
        "update_freq": rospy.get_param("~update_freq", 1.0),          # Plot update frequency (Hz)
        "topic_name": rospy.get_param("~topic_name", "/race/vehicle_state"),  # Subscribed topic
        "use_input": rospy.get_param("~use_input", True),             # Whether to use input u=vx*dtheta
    }
    # Parameter validation
    params["window_duration"] = max(5.0, params["window_duration"])
    params["max_lag"] = max(1, min(50, params["max_lag"]))
    params["update_freq"] = max(0.5, min(10.0, params["update_freq"]))
    return params

# -------------------------- 2. Global Data & Thread Lock --------------------------
params = get_ros_params()
window_duration = params["window_duration"]
user_max_lag = params["max_lag"]  # User-configured max lag (e.g., 20 → 0~20, 21 points)
update_interval = 1.0 / params["update_freq"]
use_input = params["use_input"]

# Data cache: (timestamp, LTR, u)
data_cache = deque()
cache_lock = Lock()

# Plot-related global variables（明确初始化所有线条）
fig, axes = None, None
acf_line, pacf_line, ccf_line = None, None, None
acf_ci_upper_line, acf_ci_lower_line = None, None
pacf_ci_upper_line, pacf_ci_lower_line = None, None
ccf_ci_upper_line, ccf_ci_lower_line = None, None

# 存储所有需要返回的Artist对象（确保动画函数始终有返回值）
all_artists = []

# -------------------------- 3. Data Cache Management --------------------------
def clean_expired_data(current_time):
    """Remove data older than window duration"""
    with cache_lock:
        while data_cache and (current_time - data_cache[0][0]) > window_duration:
            data_cache.popleft()

def add_data(timestamp, ltr, u):
    """Thread-safe data addition to cache"""
    with cache_lock:
        data_cache.append((timestamp, ltr, u))

# -------------------------- 4. Correlation Calculation Functions --------------------------
def calculate_acf(x, max_lag):
    """Calculate Autocorrelation Function (ACF) + 95% confidence interval (scalar)"""
    n = len(x)
    if n <= max_lag * 2:  # 至少需要2倍滞后阶数的样本
        rospy.logwarn_throttle(5, f"ACF: Insufficient data (n={n} < 2*max_lag={max_lag*2})")
        return np.zeros(max_lag+1), 0.0  # 返回默认ACF和置信区间标量
    
    # Normalized ACF calculation (0~max_lag, 长度max_lag+1)
    x_mean = np.mean(x)
    acf = np.correlate(x - x_mean, x - x_mean, mode='full')
    acf = acf[n-1:n-1+max_lag+1] / acf[n-1]  # 归一化（lag 0=1）
    
    # 95%置信区间（标量，基于白噪声假设）
    ci = 1.96 / np.sqrt(n)
    return acf, ci

def calculate_pacf(x, max_lag):
    """Calculate Partial Autocorrelation Function (PACF) + 95% CI (array)"""
    n = len(x)
    valid_max_lag = min(max_lag, (n // 2) - 1)  # 不超过样本量50%
    if valid_max_lag < 1:
        rospy.logwarn_throttle(5, f"PACF: Insufficient data (n={n}, valid max lag={valid_max_lag})")
        # 返回默认值（长度max_lag+1）
        pacf = np.zeros(max_lag+1)
        ci_upper = np.zeros(max_lag+1)
        ci_lower = np.zeros(max_lag+1)
        return pacf, ci_upper, ci_lower
    
    try:
        # 计算有效滞后阶数的PACF和CI
        pacf_result = sm.tsa.stattools.pacf(x, nlags=valid_max_lag, alpha=0.05)
        pacf = np.zeros(max_lag+1)
        ci_upper = np.zeros(max_lag+1)
        ci_lower = np.zeros(max_lag+1)
        
        # 填充有效滞后阶数的结果，剩余部分填充0
        pacf[:valid_max_lag+1] = pacf_result[0]
        ci_lower[:valid_max_lag+1] = pacf_result[1][:, 0]
        ci_upper[:valid_max_lag+1] = pacf_result[1][:, 1]
        
        return pacf, ci_upper, ci_lower
    except Exception as e:
        rospy.logerr_throttle(5, f"PACF calculation error: {str(e)}")
        return np.zeros(max_lag+1), np.zeros(max_lag+1), np.zeros(max_lag+1)

def calculate_ccf(x, y, max_lag):
    """Calculate Cross-Correlation Function (CCF) + 95% CI (scalar)"""
    n = len(x)
    if n != len(y) or n <= max_lag * 2:
        rospy.logwarn_throttle(5, f"CCF: Insufficient data (n={n} < 2*max_lag={max_lag*2})")
        return np.zeros(2*max_lag+1), 0.0  # 返回默认CCF和置信区间标量
    
    # Normalize x and y (zero mean, unit variance)
    x_norm = (x - np.mean(x)) / np.std(x) if np.std(x) != 0 else x - np.mean(x)
    y_norm = (y - np.mean(y)) / np.std(y) if np.std(y) != 0 else y - np.mean(y)
    
    # CCF计算（-max_lag~+max_lag，长度2*max_lag+1）
    ccf = np.correlate(x_norm, y_norm, mode='full')
    ccf = ccf[n-1-max_lag:n-1+max_lag+1] / n  # 归一化
    
    # 95%置信区间（标量）
    ci = 1.96 / np.sqrt(n)
    return ccf, ci

# -------------------------- 5. ROS Callback Function --------------------------
def vehicle_state_callback(msg):
    """Subscribe to vehicle state and extract LTR + u=vx*dtheta"""
    timestamp = msg.header.stamp.to_sec()
    ltr = msg.ltr_state.ltr
    
    # Extract input u if enabled
    if use_input:
        vx = msg.vel.linear.x
        dtheta = msg.vel.angular.z
        u = vx * dtheta
    else:
        u = np.nan
    
    # Validate data (排除NaN)
    if not np.isnan(ltr) and (not use_input or not np.isnan(u)):
        add_data(timestamp, ltr, u)
        clean_expired_data(timestamp)

# -------------------------- 6. Plot Initialization --------------------------
def init_plot():
    """Initialize matplotlib plot + 收集所有Artist对象"""
    global fig, axes, acf_line, pacf_line, ccf_line
    global acf_ci_upper_line, acf_ci_lower_line
    global pacf_ci_upper_line, pacf_ci_lower_line
    global ccf_ci_upper_line, ccf_ci_lower_line, all_artists
    
    # Use non-blocking backend (兼容旧版matplotlib)
    plt.switch_backend('TkAgg')
    
    # Create subplot layout
    if use_input:
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        ax_acf, ax_pacf, ax_ccf = axes[0,0], axes[0,1], axes[1,0]
        axes[1,1].axis('off')  # Hide empty subplot
    else:
        fig, axes = plt.subplots(2, 1, figsize=(10, 8))
        ax_acf, ax_pacf = axes[0], axes[1]
        ax_ccf = None
    
    # -------------------------- ACF Subplot --------------------------
    ax_acf.set_title(f'LTR Autocorrelation (ACF) - Window {window_duration}s', fontsize=10)
    ax_acf.set_xlabel('Lag Order')
    ax_acf.set_ylabel('ACF Value')
    ax_acf.set_xlim(-0.5, user_max_lag + 0.5)
    ax_acf.set_ylim(-1.2, 1.2)
    ax_acf.grid(True, alpha=0.3)
    # ACF线条（x: 0~user_max_lag，长度user_max_lag+1）
    acf_lags = np.arange(0, user_max_lag+1)
    acf_line, = ax_acf.plot(acf_lags, np.zeros_like(acf_lags), 'b-o', linewidth=2, markersize=4)
    # ACF置信区间（水平直线）
    acf_ci_upper_line = ax_acf.axhline(y=0.05, color='r', linestyle='--', alpha=0.5, label='95% CI')
    acf_ci_lower_line = ax_acf.axhline(y=-0.05, color='r', linestyle='--', alpha=0.5)
    ax_acf.legend()
    
    # -------------------------- PACF Subplot --------------------------
    ax_pacf.set_title(f'LTR Partial Autocorrelation (PACF) - Window {window_duration}s', fontsize=10)
    ax_pacf.set_xlabel('Lag Order')
    ax_pacf.set_ylabel('PACF Value')
    ax_pacf.set_xlim(-0.5, user_max_lag + 0.5)
    ax_pacf.set_ylim(-1.2, 1.2)
    ax_pacf.grid(True, alpha=0.3)
    # PACF线条
    pacf_line, = ax_pacf.plot(acf_lags, np.zeros_like(acf_lags), 'g-s', linewidth=2, markersize=4)
    # PACF置信区间（线条）
    pacf_ci_upper_line, = ax_pacf.plot(acf_lags, np.zeros_like(acf_lags), 'r--', alpha=0.5, label='95% CI')
    pacf_ci_lower_line, = ax_pacf.plot(acf_lags, np.zeros_like(acf_lags), 'r--', alpha=0.5)
    ax_pacf.legend()
    
    # -------------------------- CCF Subplot (if enabled) --------------------------
    ccf_artists = []
    if use_input and ax_ccf is not None:
        ax_ccf.set_title(f'LTR vs u(vx*dtheta) Cross-Correlation (CCF)', fontsize=10)
        ax_ccf.set_xlabel('Lag Order (Neg: u leads LTR, Pos: LTR leads u)')
        ax_ccf.set_ylabel('CCF Value')
        ccf_lags = np.arange(-user_max_lag, user_max_lag+1)
        ax_ccf.set_xlim(-user_max_lag - 0.5, user_max_lag + 0.5)
        ax_ccf.set_ylim(-1.2, 1.2)
        ax_ccf.grid(True, alpha=0.3)
        ax_ccf.set_xticks(ccf_lags[::2])
        # CCF线条
        ccf_line, = ax_ccf.plot(ccf_lags, np.zeros_like(ccf_lags), 'm-^', linewidth=2, markersize=4)
        # CCF置信区间
        ccf_ci_upper_line = ax_ccf.axhline(y=0.05, color='r', linestyle='--', alpha=0.5, label='95% CI')
        ccf_ci_lower_line = ax_ccf.axhline(y=-0.05, color='r', linestyle='--', alpha=0.5)
        ax_ccf.legend()
        ccf_artists = [ccf_line, ccf_ci_upper_line, ccf_ci_lower_line]
    
    # 收集所有需要返回的Artist对象（确保非空）
    all_artists = [
        acf_line, acf_ci_upper_line, acf_ci_lower_line,
        pacf_line, pacf_ci_upper_line, pacf_ci_lower_line
    ] + ccf_artists
    
    # Adjust layout
    plt.tight_layout()
    return fig, axes

# -------------------------- 7. Plot Update Function --------------------------
def update_plot(frame):
    """Update plot data + 确保所有分支返回非空Artist序列"""
    global acf_line, pacf_line, ccf_line
    global acf_ci_upper_line, acf_ci_lower_line
    global pacf_ci_upper_line, pacf_ci_lower_line
    global ccf_ci_upper_line, ccf_ci_lower_line
    
    # 清除之前的文本提示（避免重叠）
    for ax in axes.flat:
        for text in ax.texts:
            text.remove()
    
    # 1. 读取缓存数据（线程安全）
    with cache_lock:
        data_empty = len(data_cache) == 0
    
    if data_empty:
        # 无数据时显示提示
        for ax in axes.flat:
            ax.text(0.5, 0.5, 'No data available\nWaiting for topic...', 
                    ha='center', va='center', transform=ax.transAxes)
        return all_artists  # 必须返回Artist序列
    
    # 2. 清理过期数据并提取有效数据
    current_time = rospy.Time.now().to_sec()
    clean_expired_data(current_time)
    
    with cache_lock:
        ltr_data = np.array([d[1] for d in data_cache])
        u_data = np.array([d[2] for d in data_cache]) if use_input else None
    
    n_data = len(ltr_data)
    min_required_data = max(user_max_lag * 2, 30)  # 至少2倍滞后阶数或30个样本
    if n_data < min_required_data:
        # 数据不足时显示提示
        for ax in axes.flat:
            ax.text(0.5, 0.5, f'Insufficient data\nCurrent: {n_data} | Required: ≥{min_required_data}',
                    ha='center', va='center', transform=ax.transAxes)
        return all_artists  # 必须返回Artist序列
    
    # 3. 计算相关性系数
    acf_lags = np.arange(0, user_max_lag+1)
    acf, acf_ci = calculate_acf(ltr_data, user_max_lag)
    pacf, pacf_ci_upper, pacf_ci_lower = calculate_pacf(ltr_data, user_max_lag)
    
    # CCF相关
    ccf, ccf_ci = None, 0.0
    ccf_lags = np.arange(-user_max_lag, user_max_lag+1)
    if use_input and u_data is not None and not np.isnan(u_data).any():
        ccf, ccf_ci = calculate_ccf(ltr_data, u_data, user_max_lag)
    
    # 4. 更新图表（确保x/y长度一致）
    # ACF
    acf_line.set_data(acf_lags, acf)
    acf_ci_upper_line.set_ydata(acf_ci)
    acf_ci_lower_line.set_ydata(-acf_ci)
    # PACF
    pacf_line.set_data(acf_lags, pacf)
    pacf_ci_upper_line.set_data(acf_lags, pacf_ci_upper)
    pacf_ci_lower_line.set_data(acf_lags, pacf_ci_lower)
    # CCF
    if use_input and ccf_line is not None and ccf is not None:
        ccf_line.set_data(ccf_lags, ccf)
        ccf_ci_upper_line.set_ydata(ccf_ci)
        ccf_ci_lower_line.set_ydata(-ccf_ci)
    
    # 5. 更新标题
    axes[0,0].set_title(f'LTR Autocorrelation (ACF) - Window {window_duration}s | Data: {n_data}', fontsize=10)
    axes[0,1].set_title(f'LTR Partial Autocorrelation (PACF) - Window {window_duration}s | Data: {n_data}', fontsize=10)
    if use_input and axes[1,0] is not None:
        axes[1,0].set_title(f'LTR vs u(vx*dtheta) Cross-Correlation (CCF) | Data: {n_data}', fontsize=10)
    
    return all_artists  # 始终返回完整的Artist序列

# -------------------------- 8. Main Function --------------------------
if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node("correlation_analysis_node", anonymous=True)
    rospy.loginfo("="*60)
    rospy.loginfo("Real-time Correlation Analysis Node Started!")
    rospy.loginfo(f"Configuration:")
    rospy.loginfo(f"  Sliding Window Duration: {window_duration}s")
    rospy.loginfo(f"  Max Lag Order: {user_max_lag}")
    rospy.loginfo(f"  Plot Update Frequency: {params['update_freq']}Hz")
    rospy.loginfo(f"  Subscribed Topic: {params['topic_name']}")
    rospy.loginfo(f"  Use Input u(vx*dtheta): {'Yes' if use_input else 'No'}")
    rospy.loginfo("="*60)
    
    # Initialize plot
    try:
        fig, axes = init_plot()
        rospy.loginfo("Plot initialized successfully. Waiting for data...")
    except Exception as e:
        rospy.logerr(f"Plot initialization failed: {str(e)}")
        rospy.signal_shutdown("Plot init failed")
        exit(1)
    
    # Subscribe to ROS topic
    rospy.Subscriber(
        params["topic_name"],
        VehicleStatus,
        vehicle_state_callback,
        queue_size=50
    )
    
    # Start matplotlib animation（禁用blit，确保稳定性）
    animation = FuncAnimation(
        fig,
        update_plot,
        interval=update_interval * 1000,  # Convert to milliseconds
        blit=False,  # 禁用blit，避免返回值兼容问题
        cache_frame_data=False,
        repeat=True  # 持续更新
    )
    
    # Show plot (non-blocking mode)
    plt.ion()
    plt.show(block=False)
    
    # Keep node running
    try:
        while not rospy.is_shutdown():
            plt.pause(0.01)  # Allow plot to update
            rospy.sleep(0.1)
    except KeyboardInterrupt:
        rospy.loginfo("Node terminated by user")
    finally:
        plt.close(fig)
        rospy.loginfo("Plot closed. Node exited.")
# 单前轴横向 NMPC：实现核对与优化建议

面向当前仓库中的 `NMPCLateralController`（`plugins/nmpc_lateral_controller.cpp` + `excel_controller.yaml` + `excel_controller_carla.launch`）。双轴已改为单前轴后，本文只写**还没改对的点**和**值得做的算法优化**，便于评估后再动代码。

---

## 1. 当前结构（改完之后）

横向与纵向是拆开的：

| 插件顺序 | 作用 |
|---|---|
| `NMPCLateralController` | 只出前轮转角，`FRONT_STEERING_MODE` |
| `PIDController`（后执行） | 油门/刹车，并把 `control_mode` 改成 `THROTTLE_BRAKE_ONLY` |

因此 NMPC 里写的 `DES_ACCEL_ONLY` **会被 PID 覆盖**，实车/仿真走的是「NMPC 转角 + PID 速度」，不是加速度模式。这是架构如此，不是单前轴改漏了。

预测模型（质心取轴距中点时的运动学自行车）：

\[
v_y=\frac{v_x\tan\delta_f}{2},\quad
\dot\theta=\frac{v_x\tan\delta_f}{L},\quad
\dot\delta_f=\frac{\delta_{fdes}-\delta_f}{T_{d1}},\quad
\dot v_x=0
\]

`excel_controller.yaml` 现为 `nx=5, nu=1, dt=0.05, n_waypoints=15`。Launch 里 `control_freq=20` Hz，与 `dt=0.05` **对齐**（改之前 `dt=0.1` 对 20 Hz 会差一倍）。

---

## 2. 必须先核对的疏漏

### 2.1 航向终端代价与过程代价不一致

过程代价：

```text
X(2,k) - ref_theta - X(2,0)
```

路点 yaw 在插值时已减过当前 `yaw0`，这一项相当于用**相对航向**跟踪相对参考，说得通。

终端代价却是：

```text
X(2,N) - final_ref_theta
```

**没有**减 `X(2,0)`。`X(2,N)` 仍是绝对航向，`final_ref_theta` 是相对航向，量纲混用，高速弯道容易把车「掰」到错误航向。

建议：终端与过程统一成 `(X(2,N)-X(2,0)) - final_ref_theta`，并对误差做 `atan2(sin,cos)` 包到 \((-\pi,\pi]\)。

### 2.2 权重读了但没用

`w_v_`、`w_ax_`、`w_term_v_` 在 `initialize` 里加载，代价函数里**从未出现**。模型又强制 `\dot v_x=0`，NMPC 不可能跟踪路径速度。纵向完全交给 PID，这三项应删除或明确注释「预留」，避免调参误以为生效。

### 2.3 每步「最近路点」作参考，不是沿轨迹对齐

预测第 `k` 步状态对 15 个路点做 `min ||p-p_ref||`。车会倾向吸到几何最近点，而不是「沿路径再走 `k·dt·v`」。可能出现：

- 切弯（corner cutting）
- 参考点在预测窗内来回跳，IPOPT 难收敛

更稳的做法：按弧长取 `s_k = s_0 + v_x·k·dt`（或累计预测位移）插值出 `(x,y,θ)_ref(k)`，不要每步全局最近邻。

### 2.4 控制量没有变化率约束

只限制 `\delta_{fdes} \in [-\delta_{max},\delta_{max}]`。`w_delta_cmd1_` 罚的是 `u^2`（指令绝对值），不是 `\Delta u`。求解器可以在相邻稀疏段之间把转角打满，实车转向电机跟不上。

建议至少加：

\[
|\delta_{fdes}(i)-\delta_{fdes}(i-1)| \le \dot\delta_{max}\, \Delta t_{\mathrm{seg}}
\]

第一段相对当前实测 `\delta_f` 也要限幅。

### 2.5 求解时限与 `ma27`

`ipopt.max_iter=500`，`print_time=true`，线性解算器 `ma27`（HSL）。20 Hz 周期只有 50 ms，一次求解超时会整周期拖死，失败则锁死上一拍转角。

建议：`max_cpu_time≈0.04`，`max_iter` 降到 50–100，无 HSL 时回退 `mumps`；`print_time` 默认关掉。

### 2.6 其它小坑

- `computeControl` 的 `dt` **未使用**，积分步永远是 yaml 的 `dt_`。若日后改 `control_freq` 而不改 yaml，模型又会错拍。
- `prediction_step` yaml 为 **20**，代码默认 15，以 yaml 为准。预测时域 `N·dt=1.0 s`；60 km/h 只看前约 17 m，大曲率可能偏短。
- `w_delta1_=500` 对 `w_pos_=5` 过大，容易「不敢打方向」。单前轴比双轴少一个自由度，更需要重新标权重，不要沿用双轴 500。
- 全局最近路点：路径自交或闭环时可能跳到错误段。应限制在当前索引向前的窗口内搜索。
- Launch 注释仍写「双轴横向控制器」，仅文档问题。

`anti_rollover_controller.yaml` 里也有同名 `nmpc_lateral_controller:` 但那是另一套状态（LTR），**不要**和本插件混用同一组 `nx/nu`。

---

## 3. 模型层面（单前轴后更明显）

双轴时前后转角可对消一部分侧偏；单前轴后运动学误差会直接进横向。仓库 tutorial 里其实写的是带轮胎的 2DoF 动力学，**当前实现仍是运动学 + `\dot v_x=0`**。

| 工况 | 运动学是否够用 |
|---|---|
| 低速、中等曲率 | 一般够 |
| 60 km/h + 小半径 | `vy=vx tanδ/2` 忽略侧偏，横摆会偏乐观 |
| 质心不在轴距中点 | `/2` 应改成 `l_r/L` |

中期优化（按收益）：

1. **先改参考对齐 + 航向包角 + 转角速率**（不换模型）。
2. 质心到后轴距离 `l_r` 参数化，替换写死的 `/2`。
3. 若实测侧偏大：自行车动力学（`β, r`）或把 `vy` 用 `β\approx l_r/L·δ` 的小角形式，仍保持 `nu=1`。
4. 不要把纵向重新塞进这个插件，除非拿掉 PID；否则 `\dot v_x=0` 与真实加速同时存在，预测轨迹在加速段会偏短。

---

## 4. 建议的改动优先级

**P0（正确性，建议先改）**

1. 终端航向与过程航向同一相对定义，并做 2π 包角。  
2. IPOPT 加 CPU 时限；确认目标机有 `ma27` 或改 `mumps`。  
3. 调参：把 `weight_front_steer` 从 500 降一个数量级做对比（如 20–50），看横向误差是否下降。

**P1（跟踪质量）**

4. 路点按预测弧长索引，替换每步最近邻。  
5. 增加 `\Delta\delta` 约束；代价改为罚 `u_k-u_{k-1}`。  
6. 最近点搜索加前向窗口。

**P2（工况扩展）**

7. `l_r` 可配；高速再考虑动力学自行车。  
8. 预测时域随车速：`T = clip(d_des / v_x, Tmin, Tmax)`，或提高 `N` 而不是盲目加密 `dt`。  
9. 热启动对稀疏控制做 shift（当前整段复制上一解，车速变化时较差）。

---

## 5. 实车/仿真时建议看的量

- IPOPT 耗时是否经常 `> 40 ms`，失败率。  
- `steering_angle` 相邻周期差分是否打满。  
- 横向误差、航向误差；航向若出现 ±2π 跳变，就是包角问题。  
- PID 目标速度与 NMPC 用的 `vx` 是否同一时刻；加速时运动学预测是否明显短于真实。

---

## 6. 结论

单前轴改动（状态/控制维数、模型、约束、输出模式、yaml）主路径是自洽的，和 20 Hz 控制周期也已对齐。当前最大风险不在「少了后轮」，而在：**航向终端代价写错坐标系、参考点用最近邻、转角无速率限制、权重仍按双轴、求解可能超时**。建议按 P0→P1 改，再视 40→60 km/h 横向误差决定要不要上 `l_r` 或动力学模型。

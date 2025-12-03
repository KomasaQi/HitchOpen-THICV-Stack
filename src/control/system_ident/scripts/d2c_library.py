#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
差分方程转连续时间传递函数（与MATLAB d2c完全对齐版）
核心修正：保持Z域分子分母原始次数，双线性变换时统一(1-cs)幂次
"""

# ===================== 多项式运算辅助函数（无修改） =====================
def poly_mult(p1, p2):
    """多项式乘法：p1 * p2（低次到高次，如[1,2]*(3,4) = 3+10s+8s² → [3,10,8]）"""
    len1, len2 = len(p1), len(p2)
    result = [0.0] * (len1 + len2 - 1)
    for i in range(len1):
        for j in range(len2):
            result[i + j] += p1[i] * p2[j]
    return result

def poly_pow(p, k):
    """多项式幂：p^k（k为非负整数，低次到高次）"""
    if k == 0:
        return [1.0]  # 任何多项式的0次幂为1（常数项1）
    result = p.copy()
    for _ in range(1, k):
        result = poly_mult(result, p)
    return result

def poly_add(p_list):
    """多个多项式相加（低次到高次），自动补零对齐长度"""
    if not p_list:
        return [0.0]
    max_len = max(len(p) for p in p_list)
    padded = [p + [0.0]*(max_len - len(p)) for p in p_list]
    result = [sum(col) for col in zip(*padded)]
    while len(result) > 1 and abs(result[-1]) < 1e-10:
        result.pop()
    return result

def poly_scale(p, scalar):
    """多项式标量乘法：p * scalar（低次到高次）"""
    return [coeff * scalar for coeff in p]

# ===================== 差分方程→Z域传递函数（核心修正） =====================
def diff_eq_to_z_poly(a, b):
    """
    差分方程转Z域多项式（分子B(z)、分母A(z)）
    正确逻辑：保持分子分母原始次数，不强制补零
    参数：
        a: 输出侧系数 [a0, a1, ..., an]（对应y[k], y[k-1], ..., y[k-n]）
        b: 输入侧系数 [b0, b1, ..., bm]（对应u[k], u[k-1], ..., u[k-m]）
    返回：
        B_poly: 分子多项式（低次到高次，z^0→z^m，长度m+1）
        A_poly: 分母多项式（低次到高次，z^0→z^n，长度n+1）
    """
    m = len(b) - 1  # 输入延迟阶数（分子B(z)的次数）
    n = len(a) - 1  # 输出延迟阶数（分母A(z)的次数）
    
    # 分子B(z) = b0 z^m + b1 z^(m-1) + ... + bm → 低次到高次：[bm, bm-1, ..., b0]
    B_poly = [float(b[m - i]) for i in range(m + 1)]
    
    # 分母A(z) = a0 z^n + a1 z^(n-1) + ... + an → 低次到高次：[an, an-1, ..., a0]
    A_poly = [float(a[n - j]) for j in range(n + 1)]
    
    return B_poly, A_poly

# ===================== Z域→S域（双线性逆变换，核心修正） =====================
def z_to_s_bilinear(B_poly, A_poly, T):
    """
    双线性逆变换：Z域传递函数→连续S域传递函数（与MATLAB d2c对齐）
    正确逻辑：分子乘(1-cs)^m，分母乘(1-cs)^n，再同乘(1-cs)^(D)（D=|m-n|）消去分母
    """
    m = len(B_poly) - 1  # 分子B(z)的次数
    n = len(A_poly) - 1  # 分母A(z)的次数
    D = abs(m - n)       # 分子分母次数差
    c = T / 2.0          # 双线性变换参数（c=T/2）
    poly_1cs = [1.0, c]   # 1 + c*s（低次到高次）
    poly_1mcs = [1.0, -c] # 1 - c*s（低次到高次）

    # 步骤1：计算分子 Num_raw = B(z) * (1 - c s)^m
    Num_raw = [0.0]
    for i in range(m + 1):
        b_i = B_poly[i]  # B(z)的z^i项系数（低次到高次）
        # (1+cs)^i * (1-cs)^(m - i) = (1+cs)^i * (1-cs)^(m-i)
        p1 = poly_pow(poly_1cs, i)
        p2 = poly_pow(poly_1mcs, m - i)
        term = poly_mult(p1, p2)
        Num_raw = poly_add([Num_raw, poly_scale(term, b_i)])

    # 步骤2：计算分母 Den_raw = A(z) * (1 - c s)^n
    Den_raw = [0.0]
    for j in range(n + 1):
        a_j = A_poly[j]  # A(z)的z^j项系数（低次到高次）
        p1 = poly_pow(poly_1cs, j)
        p2 = poly_pow(poly_1mcs, n - j)
        term = poly_mult(p1, p2)
        Den_raw = poly_add([Den_raw, poly_scale(term, a_j)])

    # 步骤3：同乘(1-cs)^D，消去分母中的(1-cs)因子（D=|m-n|）
    if m > n:
        # 分母乘(1-cs)^D
        Den_s = poly_mult(Den_raw, poly_pow(poly_1mcs, D))
        Num_s = Num_raw
    else:
        # 分子乘(1-cs)^D
        Num_s = poly_mult(Num_raw, poly_pow(poly_1mcs, D))
        Den_s = Den_raw

    # 步骤4：分母归一化（最高次项系数为1）+ 符号统一（与MATLAB一致）
    den_max_coeff = Den_s[-1] if Den_s else 1.0
    if abs(den_max_coeff) > 1e-10:
        scale = 1 / den_max_coeff
        Num_s = poly_scale(Num_s, scale)
        Den_s = poly_scale(Den_s, scale)

    return Num_s, Den_s

# ===================== 核心整合函数 =====================
def diff_eq_to_continuous_tf(a, b, T):
    """差分方程直接转换为连续时间传递函数（与MATLAB d2c完全对齐）"""
    B_poly, A_poly = diff_eq_to_z_poly(a, b)
    num_s, den_s = z_to_s_bilinear(B_poly, A_poly, T)
    return num_s, den_s

def tf_to_string(num, den):
    """辅助函数：高次到低次显示传递函数（与MATLAB格式一致）"""
    def poly_to_str(coeffs):
        terms = []
        max_degree = len(coeffs) - 1
        for degree in range(max_degree, -1, -1):
            coeff = coeffs[degree]
            if abs(coeff) < 1e-10:
                continue
            # 处理系数显示（与MATLAB一致，保留4位小数）
            coeff_str = f"{coeff:.4f}".rstrip('0').rstrip('.') if abs(coeff) != int(abs(coeff)) else f"{int(coeff)}"
            if coeff_str in ('1', '-1') and degree != 0:
                coeff_str = coeff_str[:-1]
            # 处理s的次数
            if degree == 0:
                term = coeff_str
            elif degree == 1:
                term = f"{coeff_str}s"
            else:
                term = f"{coeff_str}s^2" if degree == 2 else f"{coeff_str}s^{degree}"
            terms.append(term)
        return " + ".join(terms).replace("+ -", "- ") or "0"
    
    return f"G(s) = {poly_to_str(num)} / {poly_to_str(den)}"

# ===================== 测试案例（与MATLAB完全一致） =====================
if __name__ == "__main__":
    print("="*60)
    print("差分方程→连续时间传递函数（与MATLAB d2c对齐版）")
    print("="*60)

    # ---------------------- 案例1：积分器 G(s) = 1/s ----------------------
    print("\n【案例1】已知连续系统：积分器 G(s) = 1/s")
    print("离散差分方程（T=1）：y[k] - y[k-1] = 0.5u[k] + 0.5u[k-1]")
    a1 = [1, -1]    # 输出侧系数：y[k] - y[k-1] → [a0, a1] = [1, -1]
    b1 = [0.5, 0.5] # 输入侧系数：0.5u[k] + 0.5u[k-1] → [b0, b1] = [0.5, 0.5]
    T1 = 1.0
    num1, den1 = diff_eq_to_continuous_tf(a1, b1, T1)
    print(f"输入系数 a = {a1}")
    print(f"输入系数 b = {b1}")
    print(f"采样周期 T = {T1}")
    print(f"转换后分子系数（低次到高次）：{[round(c,4) for c in num1]}")
    print(f"转换后分母系数（低次到高次）：{[round(c,4) for c in den1]}")
    print(f"传递函数：{tf_to_string(num1, den1)}")

    # ---------------------- 案例2：一阶系统 G(s) = 1/(s+1) ----------------------
    print("\n" + "-"*50)
    print("【案例2】已知连续系统：一阶系统 G(s) = 1/(s+1)")
    print("离散差分方程（T=1）：3y[k] - y[k-1] = u[k] + u[k-1]")
    a2 = [3, -1]    # 输出侧系数：3y[k] - y[k-1] → [a0, a1] = [3, -1]
    b2 = [1, 1]     # 输入侧系数：u[k] + u[k-1] → [b0, b1] = [1, 1]
    T2 = 1.0
    num2, den2 = diff_eq_to_continuous_tf(a2, b2, T2)
    print(f"输入系数 a = {a2}")
    print(f"输入系数 b = {b2}")
    print(f"采样周期 T = {T2}")
    print(f"转换后分子系数（低次到高次）：{[round(c,4) for c in num2]}")
    print(f"转换后分母系数（低次到高次）：{[round(c,4) for c in den2]}")
    print(f"传递函数：{tf_to_string(num2, den2)}")

    # ---------------------- 案例3：自定义二阶差分方程 ----------------------
    print("\n" + "-"*50)
    print("【案例3】自定义差分方程：2y[k] + 5y[k-1] = 3u[k] + 7u[k-1] + 2u[k-2]")
    print("采样周期 T = 0.5")
    a3 = [2, 5]       # 输出侧系数：2y[k] + 5y[k-1] → [a0, a1] = [2, 5]
    b3 = [3, 7, 2]    # 输入侧系数：3u[k] +7u[k-1]+2u[k-2] → [b0,b1,b2] = [3,7,2]
    T3 = 0.5
    num3, den3 = diff_eq_to_continuous_tf(a3, b3, T3)
    print(f"输入系数 a = {a3}")
    print(f"输入系数 b = {b3}")
    print(f"采样周期 T = {T3}")
    print(f"转换后分子系数（低次到高次）：{[round(c,4) for c in num3]}")
    print(f"转换后分母系数（低次到高次）：{[round(c,4) for c in den3]}")
    print(f"传递函数：{tf_to_string(num3, den3)}")

    print("\n" + "="*60)
    
    
    print("="*70)
    print("差分方程→连续传递函数 补充测试案例（Python版）")
    print("="*70)

    # ---------------------- 案例4：0阶差分方程（无延迟） ----------------------
    print("\n【案例4】0阶差分方程（无延迟）")
    print("差分方程：5y[k] = 2u[k]（T=0.2）")
    print("预期连续传递函数：G(s) = 2/5 = 0.4")
    a4 = [5]         # 输出侧：5y[k]（无延迟）
    b4 = [2]         # 输入侧：2u[k]（无延迟）
    T4 = 0.2
    num4, den4 = diff_eq_to_continuous_tf(a4, b4, T4)
    print(f"输入系数 a = {a4}")
    print(f"输入系数 b = {b4}")
    print(f"采样周期 T = {T4}")
    print(f"分子系数（低次到高次）：{[round(c,4) for c in num4]}")
    print(f"分母系数（低次到高次）：{[round(c,4) for c in den4]}")
    print(f"传递函数：{tf_to_string(num4, den4)}")

    # ---------------------- 案例5：2阶输出延迟+1阶输入延迟 ----------------------
    print("\n" + "-"*60)
    print("【案例5】2阶输出延迟+1阶输入延迟")
    print("差分方程：y[k] + 3y[k-1] + 2y[k-2] = 4u[k] + 6u[k-1]（T=0.3）")
    a5 = [1, 3, 2]   # 输出侧：y[k]+3y[k-1]+2y[k-2]
    b5 = [4, 6]      # 输入侧：4u[k]+6u[k-1]
    T5 = 0.3
    num5, den5 = diff_eq_to_continuous_tf(a5, b5, T5)
    print(f"输入系数 a = {a5}")
    print(f"输入系数 b = {b5}")
    print(f"采样周期 T = {T5}")
    print(f"分子系数（低次到高次）：{[round(c,4) for c in num5]}")
    print(f"分母系数（低次到高次）：{[round(c,4) for c in den5]}")
    print(f"传递函数：{tf_to_string(num5, den5)}")

    # ---------------------- 案例6：3阶输入延迟+2阶输出延迟 ----------------------
    print("\n" + "-"*60)
    print("【案例6】3阶输入延迟+2阶输出延迟")
    print("差分方程：2y[k] + 4y[k-1] + y[k-2] = u[k] + 3u[k-1] + 5u[k-2] + 2u[k-3]（T=0.1）")
    a6 = [2, 4, 1]   # 输出侧：2y[k]+4y[k-1]+y[k-2]
    b6 = [1, 3, 5, 2]# 输入侧：u[k]+3u[k-1]+5u[k-2]+2u[k-3]
    T6 = 0.1
    num6, den6 = diff_eq_to_continuous_tf(a6, b6, T6)
    print(f"输入系数 a = {a6}")
    print(f"输入系数 b = {b6}")
    print(f"采样周期 T = {T6}")
    print(f"分子系数（低次到高次）：{[round(c,4) for c in num6]}")
    print(f"分母系数（低次到高次）：{[round(c,4) for c in den6]}")
    print(f"传递函数：{tf_to_string(num6, den6)}")

    # ---------------------- 案例7：纯延迟差分方程 ----------------------
    print("\n" + "-"*60)
    print("【案例7】纯延迟差分方程（1步延迟）")
    print("差分方程：y[k] = u[k-1]（T=0.5）")
    print("预期连续传递函数：G(s) = (1 - e^(-0.5s))/(0.5s)（近似，tustin变换结果）")
    a7 = [1]         # 输出侧：y[k]
    b7 = [0, 1]      # 输入侧：0*u[k] + 1*u[k-1]（1步延迟）
    T7 = 0.5
    num7, den7 = diff_eq_to_continuous_tf(a7, b7, T7)
    print(f"输入系数 a = {a7}")
    print(f"输入系数 b = {b7}")
    print(f"采样周期 T = {T7}")
    print(f"分子系数（低次到高次）：{[round(c,4) for c in num7]}")
    print(f"分母系数（低次到高次）：{[round(c,4) for c in den7]}")
    print(f"传递函数：{tf_to_string(num7, den7)}")

    print("\n" + "="*70)
    
    # ---------------------- 案例8：纯延迟差分方程 ----------------------
    print("\n" + "-"*60)
    print("【案例8】纯延迟差分方程（1步延迟）")

    a8 = [1.0, -1.9786, 0.9802]         # 输出侧：y[k]
    b8 = [0.0004470, 0.0004440]      # 输入侧：0*u[k] + 1*u[k-1]（1步延迟）
    T8 = 0.01
    num8, den8 = diff_eq_to_continuous_tf(a8, b8, T8)
    print(f"输入系数 a = {a8}")
    print(f"输入系数 b = {b8}")
    print(f"采样周期 T = {T8}")
    print(f"分子系数（低次到高次）：{[round(c,4) for c in num8]}")
    print(f"分母系数（低次到高次）：{[round(c,4) for c in den8]}")
    print(f"传递函数：{tf_to_string(num8, den8)}")

    print("\n" + "="*70)
    
    
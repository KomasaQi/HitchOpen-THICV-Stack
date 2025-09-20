import numpy as np
from getParam_SP import getParam_SP
from c2d_zoh import c2d_zoh


def TrailorTruck_5DOF_SP(state0, Ts):
    # 定义模型参数
    global a, b, c, d, e, Tw1, Tw2, h1, h2, hc1, hc2
    global m1, m2, m1s, m2s, g, I1zz, I1xx, I1xz, I2zz, I2xx, I2xz
    g = 9.806
    global k1, k2, k3, kr1, kr2, c1, c2, k12, m0, mp, h0, hp, lp, co

    # 若参数未定义，则初始化
    if 'a' not in globals():
        # 摆模型参数初始化
        x = [10890, 1.5, 0.13, 0.97, 0.13]
        sim_param = getParam_SP(x)
        m0, mp, h0, hp, lp, g, co = sim_param

        # 车辆模型参数    # TODO 和壮一起确定参数
        
        # L1 = (5 + 6.27) / 2
        # L2 = (10.15 + 11.5) / 2
        # a = 1.385      
        # b = L1 - a
        # c = 5.635 - a
        # e = 5.5
        # d = L2 - e
        # Tw1 = (2.03 + 1.863 * 2) / 3
        # Tw2 = 1.863
        # hm1s = 1.02
        # hm2s = 1.00
        # hhitch = 1.1
        # hroll1 = 0.2314
        # hroll2 = 0.9630
        # h1 = hm1s - hroll1
        # h2 = hm2s - hroll2
        # hc1 = hhitch - hroll1
        # hc2 = hhitch - hroll2
        # m1s = 6310
        # m1 = m1s + 570 + 785 * 2
        # m2s = 20387
        # m2 = m2s + 665 * 2
        # I1xx = 6879
        # I1xz = 130
        # I1zz = 19665
        # I2xx = 9960
        # I2xz = 0
        # I2zz = 331380
        # k1, k2, k3 = -1e4 * np.array([300, 300, 470]) * 1
        # kr1, kr2, k12 = 18.7e5, 9.14e5, 68.3e5
        # c1, c2 = 64.4e3, 197.7e3
        
        L1 = 0.24
        L2 = 0.34
        L3 = 0.29
        L4 = 0.482
        L5 = 0.695
        a = 0.102    
        b = L1 - a
        c = L3 - a
        e = 0.386
        d = ( L4 + L5 ) / 2 - e
        Tw1 = 0.145
        Tw2 = 0.135
        hm1s = 0.15          # 牵引车质心高度
        hm2s = 0.20          # 挂车质心高度
        hhitch = 0.116       #挂钩的高度
        
        hroll1 = 0.085      #牵引车侧倾中心高度
        hroll2 = 0.125      #挂车侧倾中心高度
        h1 = hm1s - hroll1
        h2 = hm2s - hroll2
        hc1 = hhitch - hroll1
        hc2 = hhitch - hroll2
        m1s = 3.5
        m1 = 4
        m2s = 18.500
        m2 = 24.000
        ###################
        I1xx = 0.089
        I1xz = 0
        I1zz = 0.214000
        I2xx = 0.53800
        I2xz = 0
        I2zz = 1.456890
        k1, k2, k3 = np.array([4.7042, 4.7042, 8.3792])
        #k1, k2, k3 = np.array([2.0845, 4.7042, 15.3792]) * 1
        ####################
        kr1, kr2, k12 = 11,11,11
        c1, c2 = 30,30
        

    # 状态变量
    dF2, f1, vx1, vx2 = state0[7].item(), state0[10].item(), state0[11].item(), state0[11].item()
    # print('vx1',vx2)


    # 动力学矩阵 M
    m14 = -m1s * h1 * c - I1xz
    m21 = m1 * vx1 * hc1 - m1s * h1 * vx1
    m24 = I1xx + 2 * m1s * h1**2 - m1s * h1 * hc1
    m55 = m2 * vx2 * hc2 - m2s * h2 * vx2
    m58 = I2xx + 2 * m2s * h2**2 - m2s * h2 * hc2

    M = np.array([
        [m1 * vx1 * c, I1zz, 0, m14, 0, 0, 0, 0],
        [m21, -I1xz, 0, m24, 0, 0, 0, 0],
        [m1 * vx1, 0, 0, -m1s * h1, m2 * vx2, 0, 0, -m2s * h2],
        [0, 0, 0, 0, m2 * vx2 * e, -I2zz, 0, I2xz - m2s * h2 * e],
        [0, 0, 0, 0, m55, -I2xz, 0, m58],
        [1, -c / vx1, 0, -hc1 / vx1, -1, -e / vx2, 0, hc2 / vx2],
        [0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0]
    ])
    # Acm0 矩阵
    a11 = (c + a) * k1 + (c - b) * k2
    a12 = a * (c + a) * k1 / vx1 - b * (c - b) * k2 / vx1 - m1 * vx1 * c
    a22 = (a * k1 - b * k2) * hc1 / vx1 + (m1s * h1 - m1 * hc1) * vx1
    a23 = m1s * g * h1 - kr1 - k12
    a32 = (a * k1 - b * k2) / vx1 - m1 * vx1
    a36 = -d * k3 / vx2 - m2 * vx2
    a46 = -d * (e + d) * k3 / vx2 - m2 * vx2 * e
    a56 = (m2s * h2 - m2 * hc2) * vx2 - d * k3 * hc2 / vx2
    a57 = m2s * g * h2 - kr2 - k12

    # 动力学矩阵 Acm0 和输入矩阵 Bcm0
    Acm0 = np.array([
        [a11, a12, 0, 0, 0, 0, 0, 0],
        [(k1 + k2) * hc1, a22, a23, -c1, 0, 0, k12, 0],
        [k1 + k2, a32, 0, 0, k3, a36, 0, 0],
        [0, 0, 0, 0, (e + d) * k3, a46, 0, 0],
        [0, 0, k12, 0, k3 * hc2, a56, a57, -c2],
        [0, -1, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 1],
    ])
    # Bcm0 = np.zeros(8)  # 替换你的 Bcm0 计算
    Bcm0 = np.array([-(c + a) * k1, -k1 * hc1, -k1, 0, 0, 0, 0, 0]).reshape(-1, 1)

    
    M_ = np.zeros((10, 10))
    M_[8, 8] = 1
    M_[9, :] = [0, 0, 0, 0, vx2 / lp, 0, 0, -hp / lp, 0, 1]  # 定义th动力学
   

    Fycoeff = 1
    Mxcoeff = -1

    M_[2, :] = [0, 0, 0, 0, -(mp + m0) * vx2, 0, 0, (m0 + h0 + mp * hp), 0, -mp * lp] * Fycoeff
    M_[3, :] = M_[2, :] * e
    # print([0, 0, 0, 0, (m0 * h0 + mp * hp) * vx2, 0, 0, -(m0 + h0 ** 2 + mp * hp ** 2), 0, mp * lp * hp])
    M_[4, :] = M_[2, :] * hc2 + [0, 0, 0, 0, (m0 * h0 + mp * hp) * vx2, 0, 0, -(m0 + h0 ** 2 + mp * hp ** 2), 0, mp * lp * hp]
    # M_[4, :] = M_[2, :] * hc2 + [0, 0, 0, 0, -(m0 * h0 + mp * hp) * vx2, 0, 0, (m0 + h0 ** 2 + mp * hp ** 2), 0, -mp * lp * hp]* Mxcoeff

    # print()
    
    # 定义零矩阵 A_
    A_ = np.zeros((10, 10))

    A_[8, 9] = 1
    A_[9, :] = [0, 0, 0, 0, 0, -vx2 / lp, -vx2 / lp, 0, -g / lp, -co]
    A_[2, :] = [0, 0, 0, 0, 0, (mp + m0) * vx2, 0, 0, 0, 0] * Fycoeff
    A_[3, :] = A_[2, :] * e
    A_[4, :] = A_[2, :] * hc2 + [0, 0, 0, 0, 0, -(m0 * h0 + mp * hp) * vx2, -(m0 * h0 + mp * hp) * g, 0, mp * lp * g, 0]
    # A_[4, :] = A_[2, :] * hc2 + [0, 0, 0, 0, 0, (m0 * h0 + mp * hp) * vx2, (m0 * h0 + mp * hp) * g, 0, -mp * lp * g, 0] * Mxcoeff
    
    
    combined_M = np.block([
    [M, np.zeros((8, 2))],
    [np.zeros((2, 8)), np.zeros((2, 2))]
    ]) + M_

    combined_A = np.block([
        [Acm0, np.zeros((8, 2))],
        [np.zeros((2, 10))]
    ]) + A_

    combined_B = np.vstack([Bcm0, np.zeros((2, 1))])

    Acm = np.linalg.solve(combined_M, combined_A)
    Bcm = np.linalg.solve(combined_M, combined_B)
    
    Trans = np.diag([vx1, 1, 1, 1, vx2, 1, 1, 1, 1, 1])
    
    Ac = np.block([
        [np.dot(np.dot(Trans, Acm),np.linalg.inv(Trans)), np.zeros((10, 4))],
        [0,1,0,0,0,0,0,0,0,0,0,0,0,0],
        [np.zeros((1, 14))],
        [np.cos(f1), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, np.sin(f1), 0, 0],
        [-np.sin(f1), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, np.cos(f1), 0, 0]
    ])
    Bc = np.vstack([
        np.dot(Trans, Bcm),
        np.zeros((4, 1))
    ])


    # 离散化
    A, B = c2d_zoh(Ac, Bc, Ts)

    # 观测矩阵
    C = np.vstack([
        np.hstack([np.zeros((2, 12)), np.eye(2)]),  # Y1 X1
        np.hstack([np.zeros((1, 10)), np.array([[1]]), np.zeros((1, 3))]),  # f1
        np.hstack([np.zeros((2, 8)), np.eye(2), np.zeros((2, 4))]),  # th dth
        2 / (np.mean([Tw1, Tw2]) * (m1 + m2) * g) * np.hstack([
            [0, 0, -kr1, -c1, 0, 0, -kr2, -c2],
            np.zeros(6)
        ])  # LTR
    ])

    # 观测量 observe
    observe = np.dot(C, state0)
    
    return A, B, C, observe
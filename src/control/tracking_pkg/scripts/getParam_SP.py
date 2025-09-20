def getParam_SP(x):
# 根据变量生成传输参数
    TotMass = 14462
    m1 = x[0]  # 参数1
    m0 = TotMass - m1
    h0 = x[1]  # 参数2
    hp = x[2]  # 参数3
    lp = x[3]  # 参数4
    g = 9.806
    coeff = x[4]  # 参数5
    exh = 0.77  # 额外的高度是从罐底到简化力的中心：车轴心 的距离
    sim_param = [m0, m1, h0 + exh, hp + exh, lp, g, coeff]
    return sim_param
import numpy as np
from scipy.linalg import expm
def c2d_zoh(Ac, Bc, Ts):

    nx = Ac.shape[0]  # 状态维度
    nu = Bc.shape[1]  # 输入维度

    
    # 构造扩展矩阵 [Ac Bc; 0 0]
    augmented_matrix = np.vstack([
        np.hstack([Ac, Bc]) * Ts,  # 第一行是 [Ac, Bc] 乘以 Ts
        np.zeros((nu, nx + nu))   # 第二行为零矩阵
    ])
    
    M = expm(augmented_matrix)

    # 提取离散时间的 A 和 B 矩阵
    A = M[:nx, :nx]
    B = M[:nx, nx:]

    return A, B
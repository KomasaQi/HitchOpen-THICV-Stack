import pandas as pd

def read_csv(filepath):
    """读取 CSV 文件为 numpy 矩阵"""
    return pd.read_csv(filepath, header=None).values
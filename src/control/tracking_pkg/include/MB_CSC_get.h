#ifndef MB_CSC_GET_H
#define MB_CSC_GET_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <string>

using namespace Eigen;
using namespace std;

// 函数声明


// 生成具有递增的数列，并使其总和为指定的值
std::vector<int> Nums_get(int Np, int Nc, int incre_deg);

// 根据给定参数生成MB和CSC压缩映射矩阵
extern "C" {
void MB_CSC_get(int Np, int Nc, int Nr, int Nu, int Ny, int incre_MB, int incre_CSC,
                MatrixXd &Tt, MatrixXd &Pipi, MatrixXd &Pi);
}

 MatrixXd kroneckerProduct(const MatrixXd& A, const MatrixXd& B);

#endif // MATRIX_UTILS_H

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <ctime>
#include <fstream>
#include <sstream>
#include <cmath>
#include <numeric>
#include <algorithm>
#include "MB_CSC_get.h"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

using namespace Eigen;
using namespace std;
namespace py = pybind11;

//
std::vector<int> Nums_get(int Np, int Nc, int incre_deg) {
    std::vector<int> Nums(Nc, 0); // 初始化结果数组
    Nums[0] = 1; // 将第一个元素设置为1

    // 计算递增因子
    double total_sum = 0.0;
    for (int i = 2; i <= Nc; i++) {
        total_sum += std::pow(i, incre_deg);
    }
    double factor = (Np - Nc) / total_sum; // 计算递增因子

    // 生成递增数字序列
    for (int i = 1; i < Nc; i++) {
        Nums[i] = static_cast<int>(std::round(factor * std::pow(i + 1, incre_deg))) + 1;
    }

    // 调整数字序列，确保总和为 Np
    int diff = Np - std::accumulate(Nums.begin(), Nums.end(), 0);
    if (diff != 0) {
        auto max_it = std::max_element(Nums.begin(), Nums.end()); // 找到最大值的索引
        *max_it += diff; // 调整最大值
    }

    return Nums;
}

extern "C" {
void MB_CSC_get(int Np, int Nc, int Nr, int Nu, int Ny, int incre_MB, int incre_CSC,
                MatrixXd &Tt, MatrixXd &Pipi, MatrixXd &Pi) {
    // 获取 MB 压缩映射数组
    std::vector<int> Nums = Nums_get(Np, Nc, incre_MB);

    int idx = 0;
    MatrixXd T = MatrixXd::Zero(Np, Nc);
    for (int i = 0; i < Nc; i++) {
        for (int j = 0; j < Nums[i]; j++) {
            T(idx, i) = 1;
            idx++;
        }
    }
    Tt = kroneckerProduct(T, MatrixXd::Identity(Nu, Nu));
    cout << "MB压缩映射数组为:";
    for (auto val : Nums) cout << val << " ";
    cout << endl;

    // 获取 CSC 压缩映射数组
    Nums = Nums_get(Np, Nr, incre_CSC);

    idx = 0;
    Pi = MatrixXd::Zero(Nr, Np);
    for (int i = 0; i < Nr; i++) {
        Pi(i, idx) = 1;
        idx += Nums[i];
    }
    Pipi = kroneckerProduct(Pi, MatrixXd::Identity(Ny, Ny));
    cout << "CSC压缩映射数组为:";
    for (auto val : Nums) cout << val << " ";
    cout << endl;
}

}

MatrixXd kroneckerProduct(const MatrixXd& A, const MatrixXd& B) {
    int rowsA = A.rows();
    int colsA = A.cols();
    int rowsB = B.rows();
    int colsB = B.cols();
    
    MatrixXd C = MatrixXd::Zero(rowsA * rowsB, colsA * colsB);
    
    for (int i = 0; i < rowsA; ++i) {
        for (int j = 0; j < colsA; ++j) {
            C.block(i * rowsB, j * colsB, rowsB, colsB) = A(i, j) * B;
        }
    }

    return C;
}

/ 定义Python绑定模块
PYBIND11_MODULE(MB_CSC_get, m) {
    m.doc() = "MB CSC compression mapping module"; // 模块文档

    // 绑定 Nums_get 函数
    m.def("Nums_get", 
          [](int Np, int Nc, int incre_deg) -> std::vector<int> {
              return Nums_get(Np, Nc, incre_deg);
          },
          py::arg("Np"), py::arg("Nc"), py::arg("incre_deg"),
          "Generate MB/CSC compression arrays");

    // 绑定 MB_CSC_get 函数
    m.def("MB_CSC_get",
          [](int Np, int Nc, int Nr, int Nu, int Ny, int incre_MB, int incre_CSC) -> py::tuple {
              Eigen::MatrixXd Tt, Pipi, Pi;
              MB_CSC_get(Np, Nc, Nr, Nu, Ny, incre_MB, incre_CSC, Tt, Pipi, Pi);
              return py::make_tuple(Tt, Pipi, Pi);
          },
          py::arg("Np"), py::arg("Nc"), py::arg("Nr"), py::arg("Nu"), py::arg("Ny"),
          py::arg("incre_MB"), py::arg("incre_CSC"),
          "Compute MB and CSC compression mapping matrices");
}
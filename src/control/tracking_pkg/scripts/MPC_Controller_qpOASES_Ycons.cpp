#include <iostream>
#include <qpOASES.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
//#include<eigen3/unsupported/Eigen/KroneckerProduct>    
#include <vector>
#include "MB_CSC_get.h"
// #include <chrono> //需要计时部分代码的时候解锁即可
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

USING_NAMESPACE_QPOASES
using namespace Eigen;
using namespace std;
namespace py = pybind11;

extern "C" {

     double* MPC_Controller_qpOASES_Ycons(double* a0, double* b0, double* c0, double* x0, double* u0,
                                         double* Q0, double* R0, int Nx, int Nu, int Ny, int Np,
                                         int Nc, double* uconstrain0, double* yconstrain0,
                                         double* Yr0, double rho, double* Tt0, double* Pipi0, int Nr, 
                                          MatrixXd &du, MatrixXd &Y, double* epsilon) {

        // 将a,b,c重建为Eigen格式的二维矩阵
        MatrixXd a = Map<MatrixXd>(a0, Nx, Nx);
        MatrixXd b = Map<MatrixXd>(b0, Nx, Nu);
        MatrixXd c = Map<MatrixXd>(c0, Ny, Nx);

        // 将x,u重建为Eigen格式的向量
        VectorXd x = Map<VectorXd>(x0, Nx);
        VectorXd u = Map<VectorXd>(u0, Nu);

        // 将Q,R重建为Eigen格式的向量
        MatrixXd Qq1 = Map<MatrixXd>(Q0, Ny, Ny);
       // MatrixXd Qq1 = Q.replicate(1, 1).asDiagonal();

        VectorXd R = Map<VectorXd>(R0, Nu);

        // 将uconstrain,yconstrain重建为Eigen格式的二维矩阵
        MatrixXd uconstrain = Map<MatrixXd>(uconstrain0, Nu, 4);
        MatrixXd yconstrain = Map<MatrixXd>(yconstrain0, Ny, 2);

        // 将Yr重建为Eigen格式的向量
        VectorXd Yr = Map<VectorXd>(Yr0, Ny*Np);

        MatrixXd Pipi = Map<MatrixXd>(Pipi0,Ny*Nr, Ny*Np);
        MatrixXd Tt = Map<MatrixXd>(Tt0, Np, Nc);
       
       
       // VectorXd du = Map<VectorXd>(du0, Nu);
    
        // 构建控制矩阵 A
        MatrixXd A(Nx + Nu, Nx + Nu);  //19*19
        A.topLeftCorner(Nx, Nx) = a;
        A.topRightCorner(Nx, Nu) = b;
        A.bottomLeftCorner(Nu, Nx).setZero();
        A.bottomRightCorner(Nu, Nu) = MatrixXd::Identity(Nu, Nu);

        // 构建控制矩阵 B
        MatrixXd B(Nx + Nu, Nu);   //19*1
        B.topRows(Nx) = b;
        B.bottomRows(Nu) = MatrixXd::Identity(Nu, Nu);

        // 构建控制矩阵 C
        MatrixXd C(Ny, Nx + Nu);  //6*19
        C.leftCols(Nx) = c;
        C.rightCols(Nu).setZero();

        // 新的控制量 ksai(k) = [x(k), u(k-1)]'
        VectorXd ksai(Nx + Nu);
        ksai << x, u;              //19*1

       std::vector<MatrixXd> powersOfA(Np+1);
        powersOfA[0] = MatrixXd::Identity(Nx+Nu, Nx+Nu);
        for (int i = 1; i <= Np; i++) {
            powersOfA[i] = powersOfA[i-1] * A;
        }

        // 初始化 psai 矩阵并按逻辑填充
        MatrixXd psai = MatrixXd::Zero(Ny * Np, Nx + Nu);
        for (int i = 0; i < Np; i++) {
        psai.block(i * Ny, 0, Ny, Nx + Nu) = C * powersOfA[i + 1];
        }
       

        // 初始化 theta 矩阵并按逻辑填充
        MatrixXd theta = MatrixXd::Zero(Np * Ny, Np * Nu);
        for (int i = 0; i < Np; i++) {
            for (int j = 0; j <= i; j++) {
                theta.block(i * Ny, j * Nu, Ny, Nu) = C * powersOfA[i - j] * B;
            }
        }//theta 300*50,Tt 50*15

        // 最后乘以 Tt
        theta *= Tt;//300*15   //
        
        double GAMMA = 0.97; 

        VectorXd E = psai * ksai; 

      VectorXd xNeyeNp = VectorXd::Zero(Np); 
        for (int i = 0; i < Np; ++i) { 
        xNeyeNp(i) = pow(GAMMA, i); 
        } 

        xNeyeNp(Np - 1) *= 10; 

        Eigen::MatrixXd xNeyeNp_diag = xNeyeNp.asDiagonal(); 

       MatrixXd Qq = kroneckerProduct(xNeyeNp_diag, Qq1); 
        MatrixXd Rr = R.replicate(Nc, 1).asDiagonal(); 

        MatrixXd Qq_theta = Qq * theta; //300*15

       
        MatrixXd H(Nc * Nu + 1, Nc * Nu + 1); // 定义H矩阵 //16*16
        H.setZero(); //16*16的0矩阵
        H.block(0,0,Nc*Nu,Nc*Nu)=theta.transpose() * Qq_theta + Rr;// 15*300 * 300*15 + 15*15  
        H(Nc*Nu,Nc*Nu)=rho; 
        // 确保矩阵对称 
        H = (H + H.transpose()) / 2.0;

        VectorXd g(Nu*Nc+1);
        g.setZero();
        g.head(Nu*Nc)=(E - Yr).transpose()*Qq_theta;

       
        MatrixXd At_tmp(Nc, Nc); 
        At_tmp.setZero(); 

        // 填充 At_tmp 的下三角为 1
        for (int i = 0; i < Nc; i++) {
            At_tmp.block(i, 0, 1, i + 1).setOnes(); 
        }

        MatrixXd At(Nu * Nc + 2 * Ny * Nr, Nu * Nc + 1); // 定义 At 矩阵，大小为 (Nu * Nc + 2 * Ny * Nr) 行和 (Nu * Nc + 1) 列
        At.setZero(); // 将 At 矩阵初始化为零矩阵

        // 设置 At 的第一个块矩阵
        At.block(0, 0, Nu * Nc, Nu * Nc) =kroneckerProduct(At_tmp, MatrixXd::Identity(Nu, Nu)); 

        // 设置第一个块的额外一列为零
        At.block(0, Nu * Nc, Nu * Nc, 1).setZero(); // 将 At 矩阵的第 Nu * Nc 列设置为零

        // 设置 At 的中间块，使用 Pipi * theta 和 -1 向量
        At.block(Nu * Nc, 0, Ny * Nr, Nu * Nc) = Pipi * theta; 
        // 将 At 矩阵的 (Nu * Nc) 到 (Nu * Nc + Ny * Nr) 行的前 (Nu * Nc) 列块填充为 Pipi * theta
        At.block(Nu * Nc, Nu * Nc, Ny * Nr, 1) = -VectorXd::Ones(Ny * Nr); 
        // 将 At 矩阵的第 (Nu * Nc) 到 (Nu * Nc + Ny * Nr) 行的最后一列设置为 -1 向量

        // 设置 At 的最后一个块，使用 Pipi * theta 和 +1 向量
        At.block(Nu * Nc + Ny * Nr, 0, Ny * Nr, Nu * Nc) = Pipi * theta; 
        // 将 At 矩阵的最后一部分块 (Nu * Nc + Ny * Nr) 到最后行的前 (Nu * Nc) 列填充为 Pipi * theta
        At.block(Nu * Nc + Ny * Nr, Nu * Nc, Ny * Nr, 1) = VectorXd::Ones(Ny * Nr); 
        // 将 At 矩阵的最后一部分块的最后一列设置为 +1 向量


        // 控制量及其变化量的限制
        VectorXd Umin(Nc*Nu); // 定义Umin向量15*1
        VectorXd Umax(Nc*Nu); // 定义Umax向量15*1
        VectorXd dUmin(Nc*Nu + 1); // 定义dUmin向量15*2
        VectorXd dUmax(Nc*Nu + 1); // 定义dUmax向量15*2

         Umin << uconstrain.col(0).replicate(Nc, 1);
        Umax << uconstrain.col(1).replicate(Nc, 1);
        double epsilon_limit = 1e3;

        dUmin.head(Nc * Nu) << uconstrain.col(2).replicate(Nc, 1);
        dUmin(Nc * Nu) = 0;

        dUmax.head(Nc * Nu) << uconstrain.col(3).replicate(Nc, 1);
        dUmax(Nc * Nu) = epsilon_limit;

        VectorXd Ut(Nc*Nu); // 定义Ut向量
        Ut << u.replicate(Nc, 1);
        
        // 输出量约束
        VectorXd Ymin(Ny*Nr); // 定义Ymin向量
        VectorXd Ymax(Ny*Nr); // 定义Ymax向量

        // 设置Ymin向量
        Ymin << yconstrain.col(0).replicate(Nr, 1);
        // 设置Ymax向量
        Ymax << yconstrain.col(1).replicate(Nr, 1);

        VectorXd lbA(Nu*Nc+2*Ny*Nr);
        VectorXd ubA(Nu*Nc+2*Ny*Nr);
        
        lbA.head(Nu*Nc) << Umin-Ut;
        ubA.head(Nu*Nc) << Umax-Ut;

        lbA.segment(Nu*Nc,Ny*Nr) = -1e10*VectorXd::Ones(Ny * Nr);
        ubA.segment(Nu*Nc,Ny*Nr) = Ymax-Pipi*E;

        lbA.segment(Nu*Nc+Ny*Nr,Ny*Nr) = Ymin-Pipi*E;
        ubA.segment(Nu*Nc+Ny*Nr,Ny*Nr) = 1e10*VectorXd::Ones(Ny * Nr);
        

        /* Setting up QProblem object. */
        QProblem example(Nu*Nc+1,Nu*Nc+2*Ny*Nr);

        Options options;
        /* 将选项设置为MPC模式 */
        options.setToMPC();
        options.printLevel = PL_LOW;  // 设置打印级别
        example.setOptions( options );

        /* Solve first QP. */
        int_t nWSR = 20;
        MatrixXd At_transpose = At.transpose(); // 转置矩阵 test

        example.init( H.data(),g.data(),At_transpose.data(),dUmin.data(),dUmax.data(),lbA.data(),ubA.data(), nWSR );


       // 获取解
        real_t xOpt[Nu * Nc + 1];
        example.getPrimalSolution(xOpt);
 
        du = VectorXd::Map(xOpt, Nu);
        Y = E + theta * VectorXd::Map(xOpt, Nu * Nc);  // 计算输出 Y
        *epsilon = xOpt[Nu * Nc];  // epsilon是解的最后一个元素

        return 0;
        
       
    }


}
// 创建Python绑定模块
PYBIND11_MODULE(MPC_Controller_qpOASES_Ycons, m) {
    m.doc() = "MPC Controller using qpOASES"; // 模块文档

    // 绑定函数到Python模块
    m.def("MPC_Controller_qpOASES_Ycons", 
          [](const Eigen::MatrixXd &a, const Eigen::MatrixXd &b, const Eigen::MatrixXd &c,
             const Eigen::VectorXd &x, const Eigen::VectorXd &u,
             const Eigen::MatrixXd &Q, const Eigen::VectorXd &R,
             int Nx, int Nu, int Ny, int Np, int Nc,
             const Eigen::MatrixXd &uconstrain, const Eigen::MatrixXd &yconstrain,
             const Eigen::VectorXd &Yr, double rho, const Eigen::MatrixXd &Tt,
             const Eigen::MatrixXd &Pipi, int Nr) -> py::tuple {
              
              Eigen::MatrixXd du;
              Eigen::MatrixXd Y;
              double epsilon = 0.0;

              // 调用原C++函数
              MPC_Controller_qpOASES_Ycons(a.data(), b.data(), c.data(), x.data(), u.data(),
                                           Q.data(), R.data(), Nx, Nu, Ny, Np, Nc,
                                           uconstrain.data(), yconstrain.data(),
                                           Yr.data(), rho, Tt.data(), Pipi.data(), Nr,
                                           du, Y, &epsilon);

              // 将结果打包成Python返回值
              return py::make_tuple(du, Y, epsilon);
          },
          py::arg("a"), py::arg("b"), py::arg("c"), py::arg("x"), py::arg("u"),
          py::arg("Q"), py::arg("R"), py::arg("Nx"), py::arg("Nu"), py::arg("Ny"),
          py::arg("Np"), py::arg("Nc"), py::arg("uconstrain"), py::arg("yconstrain"),
          py::arg("Yr"), py::arg("rho"), py::arg("Tt"), py::arg("Pipi"), py::arg("Nr"),
          "MPC Controller implemented with qpOASES");
}

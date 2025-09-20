#include <casadi/casadi.hpp>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <chrono>
#include <stdexcept>

using namespace casadi;
using namespace std::chrono;

namespace MPCUtils {
    // Kronecker product for DM (used in precompute) and SX (fallback)
    DM kron(const DM& A, const DM& B) {
        int a_rows = A.rows(), a_cols = A.columns();
        int b_rows = B.rows(), b_cols = B.columns();
        DM R = DM::zeros(a_rows * b_rows, a_cols * b_cols);
        for (int i = 0; i < a_rows; ++i) {
            for (int j = 0; j < a_cols; ++j) {
                DM block = A(i, j) * B;
                for (int ii = 0; ii < b_rows; ++ii)
                    for (int jj = 0; jj < b_cols; ++jj)
                        R(i * b_rows + ii, j * b_cols + jj) = block(ii, jj);
            }
        }
        return R;
    }

    SX kron(const SX& A, const SX& B) {
        int a_rows = A.size1(), a_cols = A.size2();
        int b_rows = B.size1(), b_cols = B.size2();
        SX result = SX::zeros(a_rows * b_rows, a_cols * b_cols);
        for (int i = 0; i < a_rows; ++i) {
            for (int j = 0; j < a_cols; ++j) {
                result(Slice(i * b_rows, (i + 1) * b_rows), Slice(j * b_cols, (j + 1) * b_cols)) = A(i, j) * B;
            }
        }
        return result;
    }

    // matrix power for DM
    DM matrix_power(const DM& A, int n) {
        if (n < 0) throw std::invalid_argument("Power must be non-negative");
        if (n == 0) return DM::eye(A.rows());
        DM result = A;
        for (int i = 1; i < n; ++i) result = mtimes(result, A);
        return result;
    }

    // matrix power for SX
    SX matrix_power(const SX& A, int n) {
        if (n < 0) throw std::invalid_argument("Power must be non-negative");
        if (n == 0) return SX::eye(A.size1());
        SX result = A;
        for (int i = 1; i < n; ++i) result = mtimes(result, A);
        return result;
    }
} // namespace MPCUtils

// MPC controller class to precompute heavy stuff and reuse solver
class MPCController {
public:
    // constructor: precompute constant matrices and build CasADi qpsol once
    MPCController(const DM& a_dm, const DM& b_dm, const DM& c_dm,
                  int Np, int Nc, const DM& Q_dm, const DM& R_dm,
                  const DM& uconstrain_dm, const DM& yconstrain_dm, double rho,
                  int max_iter = 500) :
        Np_(Np), Nc_(Nc), rho_(rho), max_iter_(max_iter) {

        // sizes
        Nx_ = a_dm.rows();
        Nu_ = b_dm.columns();
        Ny_ = c_dm.rows();

        // Build augmented DM matrices
        DM A_aug = DM::vertcat({
            DM::horzcat({a_dm, b_dm}),
            DM::horzcat({DM::zeros(Nu_, Nx_), DM::eye(Nu_)})
        });
        DM B_aug = DM::vertcat({b_dm, DM::eye(Nu_)});
        DM C_aug = DM::horzcat({c_dm, DM::zeros(Ny_, Nu_)});

        // Precompute powers and CA^k and theta blocks (DM)
        std::vector<DM> A_pows(Np_ + 1);
        A_pows[0] = DM::eye(Nx_ + Nu_);
        for (int k = 1; k <= Np_; ++k) {
            A_pows[k] = mtimes(A_pows[k - 1], A_aug);
        }

        // psai_const: stacked C*A_aug^(i+1)  (size Ny*Np x (Nx+Nu))
        DM psai_const = DM::zeros(Ny_ * Np_, Nx_ + Nu_);
        for (int i = 0; i < Np_; ++i) {
            DM CAk = mtimes(C_aug, A_pows[i + 1]);
            psai_const(Slice(i * Ny_, (i + 1) * Ny_), Slice()) = CAk;
        }

        // theta_const: mapping from dU (Nc*Nu) to stacked outputs (Np*Ny)
        DM theta_const = DM::zeros(Np_ * Ny_, Nc_ * Nu_);
        for (int i = 0; i < Np_; ++i) {
            for (int j = 0; j <= i && j < Nc_; ++j) {
                DM term = mtimes(C_aug, mtimes(A_pows[i - j], B_aug));
                theta_const(Slice(i * Ny_, (i + 1) * Ny_), Slice(j * Nu_, (j + 1) * Nu_)) = term;
            }
        }

        // Qq and Rr
        DM Qq = MPCUtils::kron(DM::eye(Np_), Q_dm);
        DM Rr = MPCUtils::kron(DM::eye(Nc_), R_dm);

        // Cache constants used later
        psai_const_ = psai_const;
        theta_const_ = theta_const;
        B_aug_ = B_aug;
        C_aug_ = C_aug;
        A_aug_ = A_aug;
        Qq_ = Qq;
        Rr_ = Rr;
        uconstrain_dm_ = uconstrain_dm;
        yconstrain_dm_ = yconstrain_dm;

        // Build constant parts for QP: H (top-left) and At (structure)
        DM H_topleft = mtimes(theta_const_.T(), mtimes(Qq_, theta_const_)) + Rr_;
        // extend with slack var (last row/col)
        DM H = DM::zeros(Nu_ * Nc_ + 1, Nu_ * Nc_ + 1);
        H(Slice(0, Nu_ * Nc_), Slice(0, Nu_ * Nc_)) = H_topleft;
        H(Nu_ * Nc_, Nu_ * Nc_) = rho_;
        // symmetrize
        H = (H + H.T()) * 0.5;

        // At: building same as original: first dU cumulative constraints, then theta/slack, then +theta/-theta, eye
        // build At_tmp (Nc x Nc lower triangular ones)
        DM At_tmp = DM::zeros(Nc_, Nc_);
        for (int i = 0; i < Nc_; ++i)
            for (int j = 0; j <= i; ++j)
                At_tmp(i, j) = 1.0;

        DM At_block1 = MPCUtils::kron(At_tmp, DM::eye(Nu_));

        DM At = DM::vertcat({
            DM::horzcat({At_block1, DM::zeros(Nu_ * Nc_, 1)}),
            DM::horzcat({theta_const, -DM::ones(Ny_ * Np_, 1)}),
            DM::horzcat({theta_const, DM::ones(Ny_ * Np_, 1)}),
            DM::eye(Nu_ * Nc_ + 1)
        });

        // store constants
        H_dm_ = H;
        At_dm_ = At;
        Umin_dm_ = MPCUtils::kron(DM::ones(Nc_, 1), uconstrain_dm(Slice(), 0));
        Umax_dm_ = MPCUtils::kron(DM::ones(Nc_, 1), uconstrain_dm(Slice(), 1));
        dUmin_prefix_ = MPCUtils::kron(DM::ones(Nc_, 1), uconstrain_dm(Slice(), 2));
        dUmax_prefix_ = MPCUtils::kron(DM::ones(Nc_, 1), uconstrain_dm(Slice(), 3));
        Ymin_prefix_ = MPCUtils::kron(DM::ones(Np_, 1), yconstrain_dm(Slice(), 0));
        Ymax_prefix_ = MPCUtils::kron(DM::ones(Np_, 1), yconstrain_dm(Slice(), 1));

        // Build CasADi qpsol only once with these constant H and At embedded.
        SX dU_slack = SX::sym("dUslack", Nu_ * Nc_ + 1);
        SX g_sym = SX::sym("g_sym", Nu_ * Nc_ + 1);

        SX f_sx = 0.5 * mtimes(dU_slack.T(), mtimes(SX(H_dm_), dU_slack)) + mtimes(g_sym.T(), dU_slack);

        SXDict qp;
        qp["x"] = dU_slack;
        qp["p"] = g_sym;
        qp["f"] = f_sx;
        qp["g"] = mtimes(SX(At_dm_), dU_slack);

        Dict opts;
        opts["qpoases_print_level"] = "none";
        opts["print_time"] = false;
        opts["qpoases_max_iter"] = max_iter_;
        // create solver
        solver_ = qpsol("solver", "qpoases", qp, opts);

        // sizes for bounds vector l and u built later:
        int total_g_rows = At_dm_.rows();
        // pre-allocate DM vectors for l,u with correct sizes
        l_dm_ = DM::zeros(total_g_rows, 1);
        u_dm_ = DM::zeros(total_g_rows, 1);
    }

    // solve given numeric current state x_dm and current input u_dm and Yr numeric, optional warm start
    // returns pair: du (first Nu entries) and full dU_out (Nu*Nc+1)
    std::pair<DM, DM> solve(const DM& x_dm, const DM& u_dm, const DM& Yr_dm,
                            const DM* warm_start = nullptr,
                            double& assembly_ms_out = *new double(0.0),
                            double& solver_ms_out = *new double(0.0)) {
        auto t1 = high_resolution_clock::now();

        DM ksi = DM::vertcat({x_dm, u_dm});
        DM E_dm = mtimes(psai_const_, ksi);

        DM Ut_dm = MPCUtils::kron(DM::ones(Nc_, 1), u_dm);
        
        DM rhs = E_dm - Yr_dm;
        DM g_main = mtimes(theta_const_.T(), mtimes(Qq_, rhs));
        DM g_dm = DM::vertcat({g_main, DM::zeros(1, 1)});

        DM l_part1 = Umin_dm_ - Ut_dm;
        DM u_part1 = Umax_dm_ - Ut_dm;
        DM u_part2 = Ymax_prefix_ - E_dm;
        DM l_part3 = Ymin_prefix_ - E_dm;
        DM dUmin = DM::vertcat({dUmin_prefix_, -1e5});
        DM dUmax = DM::vertcat({dUmax_prefix_, 1e5});

        l_dm_ = DM::vertcat({l_part1, -DM::inf(Ny_ * Np_), l_part3, dUmin});
        u_dm_ = DM::vertcat({u_part1, u_part2, DM::inf(Ny_ * Np_), dUmax});
        
        auto t2 = high_resolution_clock::now();
        double assembly_ms = duration_cast<microseconds>(t2 - t1).count() / 1000.0;
        assembly_ms_out = assembly_ms;
        
        DMDict args;
        args["lbg"] = l_dm_;
        args["ubg"] = u_dm_;
        args["p"] = g_dm;

        if (warm_start) {
            args["x0"] = *warm_start;
        }

        auto t3 = high_resolution_clock::now();
        DMDict res;
        try {
            res = solver_(args);
        } catch (std::exception& e) {
            std::cerr << "QP solver failed: " << e.what() << std::endl;
            DM zero = DM::zeros(Nu_, 1);
            return {zero, DM::zeros(Nu_ * Nc_ + 1, 1)};
        }
        auto t4 = high_resolution_clock::now();
        double solver_ms = duration_cast<microseconds>(t4 - t3).count() / 1000.0;
        solver_ms_out = solver_ms;

        DM sol = res.at("x");
        DM du = sol(Slice(0, Nu_));
        return {du, sol};
    }

private:
    int Nx_, Nu_, Ny_, Np_, Nc_;
    double rho_;
    int max_iter_;

    // cached DM matrices
    DM psai_const_, theta_const_, A_aug_, B_aug_, C_aug_, Qq_, Rr_;
    DM uconstrain_dm_, yconstrain_dm_;
    DM H_dm_, At_dm_;
    DM Umin_dm_, Umax_dm_, dUmin_prefix_, dUmax_prefix_, Ymin_prefix_, Ymax_prefix_;
    DM l_dm_, u_dm_;

    Function solver_;
};

// ---- main ----
int main(int argc, char* argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "mpc_controller_node");
    ros::NodeHandle nh;
    ROS_INFO("MPC Controller Node Started");

    // Model parameters
    double Ts = 0.05, fr = M_PI / 6, vr = 15, deltar = 0.3, l = 2.7;
    DM a_dm = DM::vertcat({
        std::vector<double>{1.0, 0.0, -Ts * vr * sin(fr)},
        std::vector<double>{0.0, 1.0, Ts * vr * cos(fr)},
        std::vector<double>{0.0, 0.0, 1.0}
    });
    DM b_dm = DM::vertcat({
        std::vector<double>{cos(fr), 0.0},
        std::vector<double>{sin(fr), 0.0},
        std::vector<double>{tan(deltar) / l, vr / (l * pow(cos(deltar), 2))}
    }) * Ts;
    DM c_dm = DM::eye(3);
    int Ny = (int)c_dm.rows();

    // Initial states and controls
    DM x_dm = DM::vertcat({1.0, 2.0, 0.4});
    DM u_dm = DM::vertcat({0.0, 0.0});

    // MPC parameters
    int Np = 20, Nc = 3;
    DM Q_dm = DM::diag(DM::vertcat({1.0, 1.0, 1.2}));
    DM R_dm = DM::diag(DM::vertcat({0.1, 0.5}));
    double rho = 100;
    DM Yr_dm = DM::zeros(Np * Ny, 1);
    double rate_delta = 90.0 / 180.0 * M_PI;
    double dt = Ts;

    DM uconstrain_dm = DM::horzcat({
        DM::vertcat({-10.0, -30.0 * M_PI / 180.0}),
        DM::vertcat({10.0, 30.0 * M_PI / 180.0}),
        DM::vertcat({-0.2 * 9.806 * dt, -rate_delta}),
        DM::vertcat({0.2 * 9.806 * dt, rate_delta})
    });
    DM yconstrain_dm = DM::horzcat({
        DM::vertcat({-1.0, -0.05, -0.1}),
        DM::vertcat({1.0, 0.05, 0.1})
    });

    // Create controller
    std::cout << "Constructing MPC controller..." << std::endl;
    auto t0 = high_resolution_clock::now();
    MPCController mpc(a_dm, b_dm, c_dm, Np, Nc, Q_dm, R_dm, uconstrain_dm, yconstrain_dm, rho, 500);
    auto t1 = high_resolution_clock::now();
    std::cout << "Precompute time: " << duration_cast<milliseconds>(t1 - t0).count() << " ms" << std::endl;
    
    DM warm_x = DM::zeros( (int)(u_dm.rows() * Nc + 1), 1 );

    // Run MPC multiple times
    int run_iter = 50;
    double sum_assembly = 0.0, sum_solver = 0.0;
    DM du, dU;
    for (int i = 0; i < run_iter; ++i) {
        double assembly_ms = 0.0, solver_ms = 0.0;
        std::tie(du, dU) = mpc.solve(x_dm, u_dm, Yr_dm, &warm_x, assembly_ms, solver_ms);
        sum_assembly += assembly_ms;
        sum_solver += solver_ms;
        warm_x = dU;
    }

    std::cout << "Last MPC du = " << du << std::endl;
    std::cout << "Average assembly time: " << (sum_assembly / run_iter) << " ms" << std::endl;
    std::cout << "Average solver call time: " << (sum_solver / run_iter) << " ms" << std::endl;
    std::cout << "Average total time: " << (sum_assembly / run_iter + sum_solver / run_iter) << " ms" << std::endl;

    ros::shutdown();
    return 0;
}

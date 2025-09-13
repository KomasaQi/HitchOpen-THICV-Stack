from acados_template import AcadosOcp, AcadosOcpSolver
import numpy as np
import casadi as ca

# 定义系统参数
nx = 2      # 状态维度 [theta, theta_dot]
nu = 1      # 控制量维度 [u]
N = 20      # 预测步长
dt = 0.05   # 采样时间

# 创建 OCP 对象
ocp = AcadosOcp()

# 定义符号变量
X = ca.SX.sym('X', nx)
U = ca.SX.sym('U', nu)
P = ca.SX.sym('P', nx)  # 参数变量，用于 x_ref

# 动力学函数
f_expr = ca.vertcat(X[1], ca.sin(X[0]) + U[0])
ocp.model.f_expl_expr = f_expr
ocp.model.x = X
ocp.model.u = U
ocp.model.p = P  # 设置模型参数
ocp.model.name = 'pendulum_nmpc'

# RK4 离散化
k1 = f_expr
k2 = ca.substitute(f_expr, X, X + dt/2*k1)
k3 = ca.substitute(f_expr, X, X + dt/2*k2)
k4 = ca.substitute(f_expr, X, X + dt*k3)
f_discrete = X + dt/6*(k1 + 2*k2 + 2*k3 + k4)
ocp.model.f_impl_expr = X + U * 0  # 显式模型
ocp.model.disc_dyn_expr = f_discrete

# 设置维度
ocp.dims.nx = nx
ocp.dims.nu = nu
ocp.dims.np = nx  # 参数维度，与 x_ref 一致

# 代价函数
Q = np.diag([1.0, 1.0])  # 状态权重
R = np.diag([0.01])      # 控制权重
ocp.cost.cost_type = 'NONLINEAR_LS'
ocp.model.cost_y_expr = ca.vertcat(X - P, U)  # y = [X - x_ref, U]
ocp.cost.W = np.block([[Q, np.zeros((nx, nu))],
                       [np.zeros((nu, nx)), R]])
ocp.cost.yref = np.zeros(nx + nu)  # 参考值为 0，因为 x_ref 通过参数 P 传入
ocp.cost.W_e = np.zeros((nx, nx))  # 终端代价
ocp.parameter_values = np.zeros(nx)  # 初始参数值（x_ref）

# 约束
ocp.constraints.lbu = np.array([-2.0])  # 控制下界
ocp.constraints.ubu = np.array([2.0])   # 控制上界
ocp.constraints.lbx = np.array([-3.14, -np.inf])  # 状态下界
ocp.constraints.ubx = np.array([3.14, np.inf])    # 状态上界
ocp.constraints.idxbu = np.array([0])
ocp.constraints.idxbx = np.array([0, 1])
ocp.constraints.x0 = np.array([0.0, 0.0])  # 初始状态

# 求解器设置
ocp.solver_options.tf = N * dt
ocp.solver_options.N_horizon = N  # 使用 N_horizon 替代 dims.N
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
ocp.solver_options.integrator_type = 'DISCRETE'
ocp.solver_options.nlp_solver_type = 'SQP'
ocp.solver_options.qp_solver_cond_N = N

# 生成 C 代码
ocp.code_export_directory = 'c_generated_code'
acados_ocp_solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')

print("C 代码生成成功！")

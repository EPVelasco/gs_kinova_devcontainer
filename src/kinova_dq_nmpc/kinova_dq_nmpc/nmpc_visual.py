import numpy as np
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from casadi import Function
from casadi import jacobian
from kinova_dq_nmpc.forward_kinematics import jacobian_casadi
from acados_template import AcadosModel
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver, AcadosSim
from casadi import Function, MX, vertcat, sin, cos, fabs, DM
from kinova_dq_nmpc.quaternion_casadi import Quaternion
from kinova_dq_nmpc.dq_quaternion_casadi import DualQuaternion

jacobian_robot = jacobian_casadi()

# Defining Dual Quaternion informtatio
qw_1 = ca.MX.sym('qw_1', 1, 1)
qx_1 = ca.MX.sym('qx_1', 1, 1)
qy_1 = ca.MX.sym('qy_1', 1, 1)
qz_1 = ca.MX.sym('qz_1', 1, 1)
q_1 = ca.vertcat(qw_1, qx_1, qy_1, qz_1)
dw_1 = ca.MX.sym("dw_1", 1, 1)
dx_1 = ca.MX.sym("dx_1", 1, 1)
dy_1 = ca.MX.sym("dy_1", 1, 1)
dz_1 = ca.MX.sym("dz_1", 1, 1)
d_1 = ca.vertcat(dw_1, dx_1, dy_1, dz_1)

# Creating auxiliar variables
dual_1 = ca.vertcat(qw_1, qx_1, qy_1, qz_1, dw_1,dx_1, dy_1, dz_1)

# Defining Desired Frame
qw_1d = ca.MX.sym('qw_1d', 1, 1)
qx_1d = ca.MX.sym('qx_1d', 1, 1)
qy_1d = ca.MX.sym('qy_1d', 1, 1)
qz_1d = ca.MX.sym('qz_1d', 1, 1)
q_1d = ca.vertcat(qw_1d, qx_1d, qy_1d, qz_1d)

dw_1d = ca.MX.sym("dw_1d", 1, 1)
dx_1d = ca.MX.sym("dx_1d", 1, 1)
dy_1d = ca.MX.sym("dy_1d", 1, 1)
dz_1d = ca.MX.sym("dz_1d", 1, 1)
d_1d = ca.vertcat(dw_1d, dx_1d, dy_1d, dz_1d)

# Symbolic Variables
dual_1d = ca.vertcat(qw_1d, qx_1d, qy_1d, qz_1d, dw_1d, dx_1d, dy_1d, dz_1d)

# Defining the desired Velocity using symbolics
vx_1d = ca.MX.sym("vx_1d", 1, 1)
vy_1d = ca.MX.sym("vy_1d", 1, 1)
vz_1d = ca.MX.sym("vz_1d", 1, 1)

wx_1d = ca.MX.sym("wx_1d", 1, 1)
wy_1d = ca.MX.sym("wy_1d", 1, 1)
wz_1d = ca.MX.sym("wz_1d", 1, 1)

Vd = ca.vertcat(0.0, vx_1d, vy_1d, vz_1d)
Wd = ca.vertcat(0.0, wx_1d, wy_1d, wz_1d)

# Symbolic variables desired velocities
w_1d = ca.vertcat(wx_1d, wy_1d, wz_1d, vx_1d, vy_1d, vz_1d)

# Defining the control gains using symbolic variables
kr1 = ca.MX.sym("kr1", 1, 1)
kr2 = ca.MX.sym("kr2", 1, 1)
kr3 = ca.MX.sym("kr3", 1, 1)

kd1 = ca.MX.sym("kd1", 1, 1)
kd2 = ca.MX.sym("kd2", 1, 1)
kd3 = ca.MX.sym("kd3", 1, 1)

Kr = ca.vertcat(0.0, kr1, kr2, kr3)
Kd = ca.vertcat(0.0, kd1, kd2, kd3)


# Creating states of the current dualquaternion
Q1 = DualQuaternion(q_real= Quaternion(q = q_1), q_dual = Quaternion(q = d_1))

# Creating the desired quaternion
Q1d = DualQuaternion(q_real= Quaternion(q = q_1d), q_dual = Quaternion(q = d_1d))

# Creating the Desired dualquaternion twist
W1d = DualQuaternion(q_real= Quaternion(q = Wd), q_dual= Quaternion(q= Vd))

def dual_quat_casadi():
    # Function that obtains the elements of the dual quaternion  real an dual part
    values = Q1.get[:, 0]
    dualquaternion_f = Function('dualquaternion_f', [dual_1], [values])
    return dualquaternion_f

def dualquat_trans_casadi():
    # Functions that computes the translation from a dual quaternion
    values = Q1.get_trans.get[:, 0]
    f_trans = Function('f_trans', [dual_1], [values])
    return f_trans

def dualquat_get_real_casadi():
    # Function that get the real part form the dual quaternon
    values = Q1.get_real.get[:, 0]
    f_real = Function('f_real', [dual_1], [values])
    return f_real

def dualquat_get_dual_casadi():
    # Function that gets the dual part of the dualquaternion
    values = Q1.get_dual.get[:, 0]
    f_dual = Function('f_dual', [dual_1], [values])
    return f_dual

def dualquat_quat_casadi():
    # Function that get the quaternion of the dualquaternion, this elemens is the same as the real part
    values = Q1.get_quat.get[:, 0]
    f_quat = Function('f_quat', [dual_1], [values])
    return f_quat

# Creating Functions
get_real = dualquat_get_real_casadi()
get_dual = dualquat_get_dual_casadi()
get_trans = dualquat_trans_casadi()
get_quat = dualquat_quat_casadi()

def ManipoulatorModel()-> AcadosModel:
    # Dynamics of the quadrotor based on unit quaternions
    # INPUT
    # L                                                          - system parameters(mass, Inertias and gravity)
    # OUTPUT                           
    # model                                                      - Acados model
    model_name = 'kinova'
    constraint = ca.types.SimpleNamespace()

    # Defining Desired Frame
    qw_1d = ca.MX.sym('qw_1d', 1, 1)
    qx_1d = ca.MX.sym('qx_1d', 1, 1)
    qy_1d = ca.MX.sym('qy_1d', 1, 1)
    qz_1d = ca.MX.sym('qz_1d', 1, 1)

    dw_1d = ca.MX.sym("dw_1d", 1, 1)
    dx_1d = ca.MX.sym("dx_1d", 1, 1)
    dy_1d = ca.MX.sym("dy_1d", 1, 1)
    dz_1d = ca.MX.sym("dz_1d", 1, 1)

    theta_1 = ca.MX.sym('theta_1', 1, 1)
    theta_2 = ca.MX.sym('theta_2', 1, 1)
    theta_3 = ca.MX.sym('theta_3', 1, 1)
    theta_4 = ca.MX.sym('theta_4', 1, 1)
    theta_5 = ca.MX.sym("theta_5", 1, 1)
    theta_6 = ca.MX.sym("theta_6", 1, 1)
    theta_7 = ca.MX.sym("theta_7", 1, 1)
    
    X = ca.vertcat(qw_1d, qx_1d, qy_1d, qz_1d, dw_1d, dx_1d, dy_1d, dz_1d, theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, theta_7)

    # Split States of the system
    theta = X[8:15, 0]
    dualquat_1 = X[0:8, 0]

    # Auxiliary variables implicit function
    qw_1dot = ca.MX.sym('qw_1dot', 1, 1)
    qx_1dot = ca.MX.sym('qx_1dot', 1, 1)
    qy_1dot = ca.MX.sym('qy_1dot', 1, 1)
    qz_1dot = ca.MX.sym('qz_1dot', 1, 1)

    dw_1dot = ca.MX.sym("dw_1dot", 1, 1)
    dx_1dot = ca.MX.sym("dx_1dot", 1, 1)
    dy_1dot = ca.MX.sym("dy_1dot", 1, 1)
    dz_1dot = ca.MX.sym("dz_1dot", 1, 1)

    theta_1_dot = ca.MX.sym('theta_1_dot', 1, 1)
    theta_2_dot = ca.MX.sym('theta_2_dot', 1, 1)
    theta_3_dot = ca.MX.sym('theta_3_dot', 1, 1)
    theta_4_dot = ca.MX.sym('theta_4_dot', 1, 1)
    theta_5_dot = ca.MX.sym("theta_5_dot", 1, 1)
    theta_6_dot = ca.MX.sym("theta_6_dot", 1, 1)
    theta_7_dot = ca.MX.sym("theta_7_dot", 1, 1)

    X_dot = ca.vertcat(qw_1dot, qx_1dot, qy_1dot, qz_1dot, dw_1dot, dx_1dot, dy_1dot, dz_1dot, theta_1_dot, theta_2_dot, theta_3_dot, theta_4_dot, theta_5_dot, theta_6_dot, theta_7_dot)

    # Control Actions
    theta_1_c= ca.MX.sym('theta_1_c')
    theta_2_c= ca.MX.sym('theta_2_c')
    theta_3_c= ca.MX.sym('theta_3_c')
    theta_4_c= ca.MX.sym('theta_4_c')
    theta_5_c= ca.MX.sym('theta_5_c')
    theta_6_c= ca.MX.sym('theta_6_c')
    theta_7_c= ca.MX.sym('theta_7_c')

    u = ca.vertcat(theta_1_c, theta_2_c, theta_3_c, theta_4_c, theta_5_c, theta_6_c, theta_7_c)

    J = jacobian_robot(theta)
    dualdot = J@u
    theta_dot = u

    norm_q = ca.norm_2(get_quat(X[0:8]))
    constraint.norm = Function("norm", [X], [norm_q])
    constraint.expr = ca.vertcat(norm_q)
    constraint.min = 1.0
    constraint.max = 1.0
    
    # Explicit and implicit functions
    f_expl = ca.vertcat(dualdot, theta_dot)
    f_impl = X_dot - f_expl
    p = ca.MX.sym('p', X.shape[0] + u.shape[0], 1)


    # Algebraic variables
    z = []

    # Dynamics
    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl

    model.x = X
    model.xdot = X_dot

    model.u = u
    model.p = p

    model.name = model_name
    return model, get_trans, get_quat, constraint, error_dual, ln_dual

def ImageJacobian(u_pixel_1, v_pixel_1, z_camera_frame_1, u_max, v_max, f):
    # Centered pixels
    u_c_1 = u_pixel_1 - u_max
    v_c_1 = v_pixel_1 - v_max

    # Point in camera frame
    x_camera_1 = u_c_1 * z_camera_frame_1 / f
    y_camera_1 = v_c_1 * z_camera_frame_1 / f

    # --- Jacobian (3x6) built with CasADi ---
    j11 = -(f / z_camera_frame_1)
    j12 = 0
    j13 = (u_c_1 / z_camera_frame_1)
    j14 = (u_c_1 * v_c_1) / f
    j15 = -(f**2 + u_c_1**2) / f
    j16 = v_c_1

    j21 = 0
    j22 = -(f / z_camera_frame_1)
    j23 = (v_c_1 / z_camera_frame_1)
    j24 = (f**2 + v_c_1**2) / f
    j25 = -(u_c_1 * v_c_1) / f
    j26 = -u_c_1

    j31 = 0
    j32 = 0
    j33 = -1
    j34 = -y_camera_1
    j35 = x_camera_1
    j36 = 0

    J1 = ca.vertcat(
        ca.hcat([j11, j12, j13, j14, j15, j16]),
        ca.hcat([j21, j22, j23, j24, j25, j26]),
        ca.hcat([j31, j32, j33, j34, j35, j36])
    )
    return J1

def ImageJacobianTheta(p1, p2):
    u1 = p1[0, 0]
    v1 = p1[1, 0]

    # Get pixels values 
    u2 = p2[0, 0]
    v2 = p2[1, 0]

    j11 = -(v1 - v2)/((u1 - u2)**2 + (v1 - v2)**2)
    j12 = (u1 - u2)/((u1 - u2)**2 + (v1 - v2)**2)
    j13 = (v1 - v2)/((u1 - u2)**2 + (v1 - v2)**2)
    j14 = -(u1 - u2)/((u1 - u2)**2 + (v1 - v2)**2)

    J1 =  ca.hcat([j11, j12, j13, j14])
    return J1

def ImageJacobianr(p1, p2):
    # Get pixels values 
    u1 = p1[0, 0]
    v1 = p1[1, 0]

    # Get pixels values 
    u2 = p2[0, 0]
    v2 = p2[1, 0]

    h = 240
    w = 320

    j11 = ((v1 - v2)*(h*v1 - h*v2 + 2*u1*u2 - u1*w + u2*w + 2*v1*v2 - 2*u2**2 - 2*v1**2))/(2*((u1 - u2)**2 + (v1 - v2)**2)**(3/2))
    j12 = -((u1 - u2)*(h*v1 - h*v2 - 2*u1*u2 - u1*w + u2*w - 2*v1*v2 + 2*u1**2 + 2*v2**2))/(2*((u1 - u2)**2 + (v1 - v2)**2)**(3/2))
    j13 = -((v1 - v2)*(h*v1 - h*v2 - 2*u1*u2 - u1*w + u2*w - 2*v1*v2 + 2*u1**2 + 2*v2**2))/(2*((u1 - u2)**2 + (v1 - v2)**2)**(3/2))
    j14 = ((u1 - u2)*(h*v1 - h*v2 + 2*u1*u2 - u1*w + u2*w + 2*v1*v2 - 2*u2**2 - 2*v1**2))/(2*((u1 - u2)**2 + (v1 - v2)**2)**(3/2))
    J =  ca.hcat([j11, j12, j13, j14])
    return J

def ImageJacobianphi(p1, p2, z1, z2):
    # Get pixels values 
    u1 = p1[0, 0]
    v1 = p1[1, 0]

    # Get pixels values 
    u2 = p2[0, 0]
    v2 = p2[1, 0]

    j11 = (z1 - z2)/((u1 - u2)**2 + (z1 - z2)**2)
    j12 = 0.0
    j13 = -(z1 - z2)/((u1 - u2)**2 + (z1 - z2)**2)
    j14 = 0.0
    j15 = -(u1 - u2)/((u1 - u2)**2 + (z1 - z2)**2)
    j16 = (u1 - u2)/((u1 - u2)**2 + (z1 - z2)**2)
    J =  ca.hcat([j11, j12, j13, j14, j15, j16])
    return J
    
def CameraModel()-> AcadosModel:
    # Dynamics of the quadrotor based on unit quaternions
    # INPUT
    # L                                                          - system parameters(mass, Inertias and gravity)
    # OUTPUT                           
    # model                                                      - Acados model
    model_name = 'camera'
    variables = ca.types.SimpleNamespace()

    # --- Camera / image params ---
    u_max = 320/2
    v_max = 240/2
    f = 100

    # Parameters of the camera
    variables.u_max = u_max
    variables.v_max = v_max
    variables.f = f

    # --- States (pixels + depth) ---
    u_pixel_1 = ca.MX.sym('u_pixel_1')
    v_pixel_1 = ca.MX.sym('v_pixel_1')
    uv_pixel_1 = ca.vertcat(u_pixel_1, v_pixel_1)
    z_camera_frame_1 = ca.MX.sym('z_camera_frame_1')

    # --- States (pixels + depth) ---
    u_pixel_2 = ca.MX.sym('u_pixel_2')
    v_pixel_2 = ca.MX.sym('v_pixel_2')
    uv_pixel_2 = ca.vertcat(u_pixel_2, v_pixel_2)
    z_camera_frame_2 = ca.MX.sym('z_camera_frame_2')

    theta = ca.MX.sym('theta')
    r = ca.MX.sym('r')
    phi = ca.MX.sym('phi') 


    X = ca.vertcat(u_pixel_1, v_pixel_1, z_camera_frame_1, u_pixel_2, v_pixel_2, z_camera_frame_2, theta, r, phi)

    u_pixel_dot_1 = ca.MX.sym('u_pixel_dot_1') 
    v_pixel_dot_1 = ca.MX.sym('v_pixel_dot_1') 
    z_camera_frame_dot_1 = ca.MX.sym('z_camera_frame_dot_1') 

    u_pixel_dot_2 = ca.MX.sym('u_pixel_dot_2') 
    v_pixel_dot_2 = ca.MX.sym('v_pixel_dot_2') 
    z_camera_frame_dot_2 = ca.MX.sym('z_camera_frame_dot_2') 

    theta_dot = ca.MX.sym('theta_dot')
    r_dot = ca.MX.sym('r_dot')
    phi_dot = ca.MX.sym('phi_dot')

    X_dot = ca.vertcat(u_pixel_dot_1, v_pixel_dot_1, z_camera_frame_dot_1, u_pixel_dot_2, v_pixel_dot_2, z_camera_frame_dot_2, theta_dot, r_dot, phi_dot)

    # --- Camera spatial velocity (vx, vy, vz, wx, wy, wz) ---
    vx_c = ca.MX.sym('vx_c')
    vy_c = ca.MX.sym('vy_c')
    vz_c = ca.MX.sym('vz_c')
    wx_c = ca.MX.sym('wx_c')
    wy_c = ca.MX.sym('wy_c')
    wz_c = ca.MX.sym('wz_c')
    u = ca.vertcat(vx_c, vy_c, vz_c, wx_c, wy_c, wz_c)

    J1 = ImageJacobian(u_pixel_1, v_pixel_1, z_camera_frame_1, u_max, v_max, f)
    J2 = ImageJacobian(u_pixel_2, v_pixel_2, z_camera_frame_2, u_max, v_max, f)

    features_1_dot = J1@u
    features_2_dot = J2@u

    # Extract only Jacobian of the features
    J1uv = J1[0:2, :]
    J2uv = J2[0:2, :]
    Juv = ca.vertcat(J1uv, J2uv)

    # Compute jacobian Theta
    Jtheta = ImageJacobianTheta(uv_pixel_1, uv_pixel_2)
    Jr = ImageJacobianr(uv_pixel_1, uv_pixel_2)
    Jphi = ImageJacobianphi(uv_pixel_1, uv_pixel_2, z_camera_frame_1, z_camera_frame_2)

    theta_feature_dot = Jtheta@Juv@u
    r_feature_dot = Jr@Juv@u
    phi_features_dot = Jphi@u

    # Explicit and implicit functions
    f_expl = ca.vertcat(features_1_dot, features_2_dot, theta_feature_dot, r_feature_dot, phi_features_dot)
    f_impl = X_dot - f_expl
    p = ca.MX.sym('p', X.shape[0] + u.shape[0], 1)

    # Algebraic variables
    z = []

    # Dynamics
    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl

    model.x = X
    model.xdot = X_dot

    model.u = u
    model.p = p
    model.name = model_name

    return model, variables

def solver(N_prediction, theta_1_max, theta_1_min, theta_2_max, theta_2_min, theta_3_max, theta_3_min, theta_4_max, theta_4_min, theta_5_max, theta_5_min, theta_6_max, theta_6_min, theta_7_max, theta_7_min, ts, t_N, x0):
        # get dynamical model
        model, get_trans, get_quat, constraint, dual_error, ln = ManipoulatorModel()
        
        # Optimal control problem
        ocp = AcadosOcp()
        ocp.model = model

        # Get size of the system
        nx = model.x.size()[0]
        nu = model.u.size()[0]
        ny = nx + nu

        # Set Dimension of the problem
        ocp.p = model.p
        ocp.dims.N = N_prediction

        # Definition of the cost functions (EXTERNAL)
        ocp.cost.cost_type = "EXTERNAL"
        ocp.cost.cost_type_e = "EXTERNAL" 

        # some variables
        x = ocp.model.x
        u = ocp.model.u
        p = ocp.model.p

        R = MX.zeros(7, 7)
        R[0, 0] = 20/theta_1_max
        R[1, 1] = 20/theta_2_max
        R[2, 2] = 20/theta_3_max
        R[3, 3] = 20/theta_4_max
        R[4, 4] = 20/theta_5_max
        R[5, 5] = 20/theta_6_max
        R[6, 6] = 20/theta_7_max

        Q_dual = MX.zeros(8, 8)
        Q_dual[0, 0] = 0.1
        Q_dual[1, 1] = 0.1
        Q_dual[2, 2] = 0.1
        Q_dual[3, 3] = 0.1

        Q_dual[4, 4] = 2.0
        Q_dual[5, 5] = 2.0
        Q_dual[6, 6] = 2.0
        Q_dual[7, 7] = 2.0

        dual = x[0:8]
        theta = x[8:15]
        
        dual_d = p[0:8]
        theta_d = p[8:15]
        theta_d_dot = p[15:22]
        
        error_dual = dual_error(dual_d, dual)
        ln_error = ln(error_dual)
        u_error = theta_d_dot - u

        ocp.model.cost_expr_ext_cost = 10*(ln_error.T@Q_dual@ln_error) + 0.01*(u_error.T @ R @ u_error) # gains of Cost function 
        ocp.model.cost_expr_ext_cost_e =  10*(ln_error.T@Q_dual@ln_error)

        ref_params = np.array([1.0, 0.0, 0.0, 0.0,    # Primary part dualquaternion
                                     0.0, 0.0, 0.0, 0.0,    # Dual part dualquaternion
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,        # theta
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,        # theta dot c
                                     ])  

        ocp.parameter_values = ref_params

        ocp.constraints.constr_type = 'BGH'

        ## Set constraints
        # Theta 1  no limits in angles
        # Thetadot 1  1.39

        # Theta 2  [-2.24 2.24]rad
        # Thetadot 2  1.39

        # Theta 3  no limits in angles
        # Thetadot 3  1.39

        # Theta 4  [-2.57 2.57]
        # Thetadot 4  1.3963

        # Theta 5  no limits in angles
        # Thetadot 5  1.2218

        # Theta 6  [-2.09 2.09]
        # Thetadot 6 1.2218

        # Theta 7
        # Thetadot 7 1.2218

        #ocp.constraints.ubx = ubx
        #ocp.constraints.lbx = lbx
        #ocp.constraints.ubx_0 = ubx
        #ocp.constraints.lbx_0 = lbx
        #ocp.constraints.ubx_e = ubx
        #ocp.constraints.lbx_e = lbx

        ocp.constraints.lbu = np.array([theta_1_min, theta_2_min, theta_3_min, theta_4_min, theta_5_min, theta_6_min, theta_7_min])
        ocp.constraints.ubu = np.array([theta_1_max, theta_2_max, theta_3_max, theta_4_max, theta_5_max, theta_6_max, theta_7_max])
        ocp.constraints.idxbu = np.array([0, 1, 2, 3, 4, 5, 6])
        ocp.constraints.x0 = x0

        ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM" 
        ocp.solver_options.qp_solver_cond_N = N_prediction // 4
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.Tsim = ts
        ocp.solver_options.tf = t_N
        return ocp

def solverCamera(N_prediction, ts, t_N, x0):
        # get dynamical model
        model, variables = CameraModel()
        
        # Optimal control problem
        ocp = AcadosOcp()
        ocp.model = model

        # Get size of the system
        nx = model.x.size()[0]
        nu = model.u.size()[0]
        ny = nx + nu

        # Set Dimension of the problem
        ocp.p = model.p
        ocp.dims.N = N_prediction

        # Definition of the cost functions (EXTERNAL)
        ocp.cost.cost_type = "EXTERNAL"
        ocp.cost.cost_type_e = "EXTERNAL" 

        # some variables
        x = ocp.model.x
        u = ocp.model.u
        p = ocp.model.p

        # Maximum velocities of the end effector
        vx_max = 1.0
        vy_max = 0.1
        vz_max = 0.01

        wx_max = 0.0
        wy_max = 0.01
        wz_max = 1.0

        R = MX.zeros(6, 6)
        R[0, 0] = 20/vx_max
        R[1, 1] = 20/vy_max
        R[2, 2] = 20/vz_max
        R[4, 4] = 20/wy_max
        R[5, 5] = 20/wz_max

        # Extracts the z values of the system
        z1 = x[2]*10 # Just multiplying to have a better conditiones problem the values are quite small
        z2 = x[5]*10 # Just multiplying to have a better conditiones problem the values are quite small
        z_average = (z1 + z2) / 2

        zd = 0.195*10
        ze = zd-z_average

        theta = x[6]
        r = x[7]
        phi = 1000*x[8]    ## Scaling factor this angle is too small
        r_normalized = r/variables.v_max

        # Velocity x  ---------------------------------------- Verify this mapping this is only for simulation purposes 
        xi = np.array([10*x[6], 100*x[8], r_normalized]) 
        velocity_x = (0.001)/(1 + xi.T@xi)

        # Desired Velocities
        Vd = MX.zeros(6, 1)
        Vd[0, 0] = velocity_x
        Vd[1, 0] = 0.0
        Vd[2, 0] = 0.0
        Vd[3, 0] = 0.0
        Vd[4, 0] = 0.0
        Vd[5, 0] = 0.0

        velocity_error = Vd - u


        ocp.model.cost_expr_ext_cost = 50*(theta*theta) + velocity_error.T@R@velocity_error + 0.1*(r_normalized*r_normalized) + 1*(ze*ze) + 50*(phi*phi)
        ocp.model.cost_expr_ext_cost_e =  50*(theta*theta) + 0.1*(r_normalized*r_normalized) + 1*(ze*ze)+ 50*(phi*phi)

        ref_params = np.array([2.39245010e+02, 1.29000000e+02, 1.96842924e-01,
                               2.76755005e+02, 1.29000000e+02, 1.96842998e-01,
                               0, 9.00000000e+00, 0,
                               0, 0, 0, 0, 0, 0
                                     ])  

        ocp.parameter_values = ref_params

        ocp.constraints.constr_type = 'BGH'

        # Constrains in z 0.182 0.197 
        # Verify this values on the real robot
        z_min  = 0.1815
        z_max = 0.197

        ubx = np.array([z_max, z_max])
        lbx = np.array([z_min, z_min])

        # Contrains in the states of the nmpc
        ocp.constraints.ubx = ubx
        ocp.constraints.lbx = lbx
        ocp.constraints.ubx_0 = ubx
        ocp.constraints.lbx_0 = lbx
        ocp.constraints.ubx_e = ubx
        ocp.constraints.lbx_e = lbx
        ocp.constraints.idxbx = np.array([2, 5])
        ocp.constraints.idxbx_0 = np.array([2, 5])
        ocp.constraints.idxbx_e = np.array([2, 5])

        ## Contraints in the control actions
        ocp.constraints.lbu = np.array([-vx_max, -vy_max, -vz_max, -wx_max, -wy_max, -wz_max])
        ocp.constraints.ubu = np.array([vx_max, vy_max, vz_max, wx_max, wy_max, wz_max])
        ocp.constraints.idxbu = np.array([0, 1, 2, 3, 4, 5])
        ocp.constraints.x0 = x0

        ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM" 
        ocp.solver_options.qp_solver_cond_N = N_prediction // 4
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.Tsim = ts
        ocp.solver_options.tf = t_N
        return ocp

def error_dual(qd, q):
    qd_conjugate = ca.vertcat(qd[0], -qd[1], -qd[2], -qd[3], qd[4], -qd[5], -qd[6], -qd[7])
    quat_d_data = qd_conjugate[0:4]
    dual_d_data =  qd_conjugate[4:8]

    H_r_plus = ca.vertcat(ca.horzcat(quat_d_data[0], -quat_d_data[1], -quat_d_data[2], -quat_d_data[3]),
                                ca.horzcat(quat_d_data[1], quat_d_data[0], -quat_d_data[3], quat_d_data[2]),
                                ca.horzcat(quat_d_data[2], quat_d_data[3], quat_d_data[0], -quat_d_data[1]),
                                ca.horzcat(quat_d_data[3], -quat_d_data[2], quat_d_data[1], quat_d_data[0]))

    H_d_plus = ca.vertcat(ca.horzcat(dual_d_data[0], -dual_d_data[1], -dual_d_data[2], -dual_d_data[3]),
                                ca.horzcat(dual_d_data[1], dual_d_data[0], -dual_d_data[3], dual_d_data[2]),
                                ca.horzcat(dual_d_data[2], dual_d_data[3], dual_d_data[0], -dual_d_data[1]),
                                ca.horzcat(dual_d_data[3], -dual_d_data[2], dual_d_data[1], dual_d_data[0]))
    zeros = ca.DM.zeros(4, 4)
    Hplus = ca.vertcat(ca.horzcat(H_r_plus, zeros),
                        ca.horzcat(H_d_plus, H_r_plus))

    q_e_aux = Hplus @ q

    q_error = q_e_aux

    return q_error

def ln_dual(q_error):
    q_error_real = q_error[0:4, 0]
    q_error_real_c = ca.vertcat(q_error_real[0, 0], -q_error_real[1, 0], -q_error_real[2, 0], -q_error_real[3, 0])
    q_error_dual = q_error[4:8, 0]

    ## Real Part
    norm = ca.norm_2(q_error_real[1:4] + ca.np.finfo(np.float64).eps)
    angle = 2*ca.atan2(norm, q_error_real[0])

    ## Dual Part
    H_error_dual_plus = ca.vertcat(ca.horzcat(q_error_dual[0, 0], -q_error_dual[1, 0], -q_error_dual[2, 0], -q_error_dual[3, 0]),
                                ca.horzcat(q_error_dual[1, 0], q_error_dual[0, 0], -q_error_dual[3, 0], q_error_dual[2, 0]),
                                ca.horzcat(q_error_dual[2, 0], q_error_dual[3, 0], q_error_dual[0, 0], -q_error_dual[1, 0]),
                                ca.horzcat(q_error_dual[3, 0], -q_error_dual[2, 0], q_error_dual[1, 0], q_error_dual[0, 0]))

    trans_error = 2 * H_error_dual_plus@q_error_real_c
    # Computing log map
    ln_quaternion = ca.vertcat(0.0, (1/2)*angle*q_error_real[1, 0]/norm, (1/2)*angle*q_error_real[2, 0]/norm, (1/2)*angle*q_error_real[3, 0]/norm)
    ln_trans = ca.vertcat(0.0, (1/2)*trans_error[1, 0], (1/2)*trans_error[2, 0], (1/2)*trans_error[3, 0])

    q_e_ln = ca.vertcat(ln_quaternion, ln_trans)

    return q_e_ln
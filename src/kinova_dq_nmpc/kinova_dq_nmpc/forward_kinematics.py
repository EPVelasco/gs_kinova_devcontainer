import numpy as np
import casadi as ca
import numpy as np
from kinova_dq_nmpc.quaternion_casadi import Quaternion
from kinova_dq_nmpc.dq_quaternion_casadi import DualQuaternion
from casadi import Function
from casadi import jacobian

# Base link 
base_position = np.array([0, 0, 0])
base_rpy = np.array([0, 0, 0])

# Shoulder link 
# Joint 1
shoulder_link_position = np.array([0, 0, 0.15643])
shoulder_link_rpy = np.array([3.1416, 0, 0])

# Half arm 1 link
# Joint 2
half_arm_1_link_position = np.array([0, 0.005375, -0.12838])
half_arm_1_link_rpy = np.array([1.5708, 0, 0])

# Half arm 2 link
# Joint 3
half_arm_2_link_position = np.array([0, -0.21038, -0.006375])
half_arm_2_link_rpy = np.array([-1.5708, 0, 0])

# Forarm link
# Joint 4
forearm_link_position = np.array([0, 0.006375, -0.21038])
forearm_link_rpy = np.array([1.5708, 0, 0])

# Spherical Wrist 1 link
# Joint 5
spherical_wrist_1_link_position = np.array([0, -0.20843, -0.006375])
spherical_wrist_1_link_rpy = np.array([-1.5708, 0, 0])

# Spherical Wrist 2 link
# Joint 6
spherical_wrist_2_link_position = np.array([0, 0.00017505, -0.10593])
spherical_wrist_2_link_rpy = np.array([1.5708, 0, 0])

# Bracelet link
# Joint 7
bracelet_link_position = np.array([0 -0.10593 -0.00017505])
bracelet_link_rpy = np.array([-1.5708, 0, 0])

# End effector link
end_effector_link_position = np.array([0, 0, -0.061525])
end_effector_link_rpy = np.array([3.1416, 0, 0])

# Auxiliar values to create the dual quaternion
# Link 1 and Joint 1
aux_1_1 = np.pi
n_1_1 = ca.MX([1.0, 0.0, 0.0])
q_1_1 = ca.vertcat(ca.cos(aux_1_1/2), ca.sin(aux_1_1/2)@n_1_1)
t_1_1= ca.MX([0.0, 0, 0, 0.15643])

theta_1 = ca.MX.sym('theta_1', 1, 1)
n_1 = ca.MX([0.0, 0.0, 1.0])
q_1 = ca.vertcat(ca.cos(theta_1/2), ca.sin(theta_1/2)@n_1)
t_1 = ca.MX([0.0, 0.0, 0.0, 0.0])


# Auxiliar Transformation Link2
aux_2_1 = np.pi/2
n_2_1 = ca.MX([1.0, 0.0, 0.0])
q_2_1 = ca.vertcat(ca.cos(aux_2_1/2), ca.sin(aux_2_1/2)@n_2_1)
t_2_1 = ca.MX([0.0, 0, 0.005375, -0.12838])

# Joint Link 2
theta_2 = ca.MX.sym('theta_2', 1, 1)
n_2 = ca.MX([0.0, 0.0, 1.0])
q_2 = ca.vertcat(ca.cos(theta_2/2), ca.sin(theta_2/2)@n_2)
t_2 = ca.MX([0.0, 0.0, 0.0, 0.0])

# Joint Link 3
aux_3_1 = -np.pi/2
n_3_1 = ca.MX([1.0, 0.0, 0.0])
q_3_1 = ca.vertcat(ca.cos(aux_3_1/2), ca.sin(aux_3_1/2)@n_3_1)
t_3_1 = ca.MX([0.0, 0, -0.21038, -0.006375])

theta_3 = ca.MX.sym('theta_3', 1, 1)
n_3 = ca.MX([0.0, 0.0, 1.0])
q_3 = ca.vertcat(ca.cos(theta_3/2), ca.sin(theta_3/2)@n_3)
t_3 = ca.MX([0.0, 0.0, 0.0, 0.0])

# Auxiliar Transformation Link4
aux_4_1 = np.pi/2
n_4_1 = ca.MX([1.0, 0.0, 0.0])
q_4_1 = ca.vertcat(ca.cos(aux_4_1/2), ca.sin(aux_4_1/2)@n_4_1)
t_4_1 = ca.MX([0.0, 0, 0.006375, -0.21038])

# Joint link 4
theta_4 = ca.MX.sym('theta_4', 1, 1)
n_4 = ca.MX([0.0, 0.0, 1.0])
q_4 = ca.vertcat(ca.cos(theta_4/2), ca.sin(theta_4/2)@n_4)
t_4 = ca.MX([0.0, 0.0, 0.0, 0.0])

# Auxiliar Transformation Link5
aux_5_1 = -np.pi/2
n_5_1 = ca.MX([1.0, 0.0, 0.0])
q_5_1 = ca.vertcat(ca.cos(aux_5_1/2), ca.sin(aux_5_1/2)@n_5_1)
t_5_1 = ca.MX([0.0, 0, -0.20843, -0.006375])

# Joint 5
theta_5 = ca.MX.sym('theta_5', 1, 1)
n_5 = ca.MX([0.0, 0.0, 1.0])
q_5 = ca.vertcat(ca.cos(theta_5/2), ca.sin(theta_5/2)@n_5)
t_5 = ca.MX([0.0, 0.0, 0.0, 0.0])

# Auxiliar Transformation Link6
aux_6_1 = np.pi/2
n_6_1 = ca.MX([1.0, 0.0, 0.0])
q_6_1 = ca.vertcat(ca.cos(aux_6_1/2), ca.sin(aux_6_1/2)@n_6_1)
t_6_1 = ca.MX([0.0, 0, 0.00017505, -0.10593])

# Joint 6
theta_6 = ca.MX.sym('theta_6', 1, 1)
n_6 = ca.MX([0.0, 0.0, 1.0])
q_6 = ca.vertcat(ca.cos(theta_6/2), ca.sin(theta_6/2)@n_6)
t_6 = ca.MX([0.0, 0.0, 0.0, 0.0])

# Auxiliar Transformation Link6
aux_7_1 = -np.pi/2
n_7_1 = ca.MX([1.0, 0.0, 0.0])
q_7_1 = ca.vertcat(ca.cos(aux_7_1/2), ca.sin(aux_7_1/2)@n_7_1)
t_7_1 = ca.MX([0.0, 0, -0.10593, -0.00017505])

# Joint 7
theta_7= ca.MX.sym('theta_7', 1, 1)
n_7 = ca.MX([0.0, 0.0, 1.0])
q_7 = ca.vertcat(ca.cos(theta_7/2), ca.sin(theta_7/2)@n_7)
t_7 = ca.MX([0.0, 0.0, 0.0, 0.0])

# End effector
theta_8= ca.MX([np.pi])
n_8 = ca.MX([1.0, 0.0, 0.0])
q_8 = ca.vertcat(ca.cos(theta_8/2), ca.sin(theta_8/2)@n_8)
t_8 = ca.MX([0.0, 0, 0, -0.061525])

# DualQuaternion from axis and position
Q1_1_pose =  DualQuaternion.from_pose(quat = q_1_1, trans = t_1_1)
Q1_pose =  DualQuaternion.from_pose(quat = q_1, trans = t_1)

Q2_1_pose =  DualQuaternion.from_pose(quat = q_2_1, trans = t_2_1)
Q2_pose =  DualQuaternion.from_pose(quat = q_2, trans = t_2)

Q3_1_pose =  DualQuaternion.from_pose(quat = q_3_1, trans = t_3_1)
Q3_pose =  DualQuaternion.from_pose(quat = q_3, trans = t_3)

Q4_1_pose =  DualQuaternion.from_pose(quat = q_4_1, trans = t_4_1)
Q4_pose =  DualQuaternion.from_pose(quat = q_4, trans = t_4)

Q5_1_pose =  DualQuaternion.from_pose(quat = q_5_1, trans = t_5_1)
Q5_pose =  DualQuaternion.from_pose(quat = q_5, trans = t_5)

Q6_1_pose =  DualQuaternion.from_pose(quat = q_6_1, trans = t_6_1)
Q6_pose =  DualQuaternion.from_pose(quat = q_6, trans = t_6)

Q7_1_pose =  DualQuaternion.from_pose(quat = q_7_1, trans = t_7_1)
Q7_pose =  DualQuaternion.from_pose(quat = q_7, trans = t_7)

Q8_pose =  DualQuaternion.from_pose(quat = q_8, trans = t_8)


def forward_kinematics_casadi_link1():
    # Compute the Pose based of the quaternion and the trasnation
    tranformation = Q1_1_pose * Q1_pose
    values = tranformation.get[:, 0]
    f_pose = Function('f_pose', [theta_1], [values])
    return f_pose

def forward_kinematics_casadi_link2():
    # Compute the Pose based of the quaternion and the trasnation
    tranformation = Q2_1_pose * Q2_pose
    values = tranformation.get[:, 0]
    f_pose = Function('f_pose', [theta_2], [values])
    return f_pose

def forward_kinematics_casadi_link3():
    # Compute the Pose based of the quaternion and the trasnation
    tranformation = Q3_1_pose * Q3_pose
    values = tranformation.get[:, 0]
    f_pose = Function('f_pose', [theta_3], [values])
    return f_pose

def forward_kinematics_casadi_link4():
    # Compute the Pose based of the quaternion and the trasnation
    tranformation = Q4_1_pose  * Q4_pose
    values = tranformation.get[:, 0]
    f_pose = Function('f_pose', [theta_4], [values])
    return f_pose

def forward_kinematics_casadi_link5():
    # Compute the Pose based of the quaternion and the trasnation
    tranformation = Q5_1_pose  * Q5_pose
    values = tranformation.get[:, 0]
    f_pose = Function('f_pose', [theta_5], [values])
    return f_pose

def forward_kinematics_casadi_link6():
    # Compute the Pose based of the quaternion and the trasnation
    tranformation = Q6_1_pose  * Q6_pose
    values = tranformation.get[:, 0]
    f_pose = Function('f_pose', [theta_6], [values])
    return f_pose

def forward_kinematics_casadi_link7():
    # Compute the Pose based of the quaternion and the trasnation
    tranformation = Q7_1_pose  * Q7_pose
    values = tranformation.get[:, 0]
    f_pose = Function('f_pose', [theta_7], [values])
    return f_pose

def forward_kinematics_casadi_link8():
    # Compute the Pose based of the quaternion and the trasnation
    tranformation = Q8_pose
    values = tranformation.get[:, 0]
    f_pose = Function('f_pose', [], [values])
    return f_pose

def forward_kinematics_casadi():
    # Compute the Pose based of the quaternion and the trasnation
    tranformation = Q1_1_pose * Q1_pose * Q2_1_pose * Q2_pose * Q3_1_pose * Q3_pose * Q4_1_pose  * Q4_pose * Q5_1_pose  * Q5_pose * Q6_1_pose  * Q6_pose * Q7_1_pose  * Q7_pose * Q8_pose
    values = tranformation.get[:, 0]
    f_pose = Function('f_pose', [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, theta_7], [values])
    return f_pose

def jacobian_casadi():
    # Compute Jacobian using Dual quaternions
    tranformation = Q1_1_pose * Q1_pose * Q2_1_pose * Q2_pose * Q3_1_pose * Q3_pose * Q4_1_pose  * Q4_pose * Q5_1_pose  * Q5_pose * Q6_1_pose  * Q6_pose * Q7_1_pose  * Q7_pose * Q8_pose
    symbolic_actions = ca.vertcat(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, theta_7)
    J = ca.jacobian(tranformation.get[:, 0], symbolic_actions)
    f_jacobian = Function("Function_J", [symbolic_actions], [J])
    return f_jacobian
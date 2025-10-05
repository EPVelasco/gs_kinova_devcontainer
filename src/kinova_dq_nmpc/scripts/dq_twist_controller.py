#!/usr/bin/env python3
import rospy
import numpy as np
import casadi as ca

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from kortex_driver.msg import Base_JointSpeeds, JointSpeed

# --- kinova_dq_nmpc imports (tuyos) ---
from kinova_dq_nmpc.forward_kinematics import (
    forward_kinematics_casadi_link1, forward_kinematics_casadi_link2,
    forward_kinematics_casadi_link3, forward_kinematics_casadi_link4,
    forward_kinematics_casadi_link5, forward_kinematics_casadi_link6,
    forward_kinematics_casadi_link7, forward_kinematics_casadi_link8,
    forward_kinematics_casadi, jacobian_casadi, dualquat_from_pose_casadi
)
from kinova_dq_nmpc.ode_acados import (
    dualquat_trans_casadi, dualquat_quat_casadi, dual_velocity_casadi,
    velocities_from_twist_casadi, error_dual_aux_casadi, ln_dual_aux_casadi,
    rotation_casadi
)

# ======= Utilidades DQ / FK / Jacobiano =======
get_trans = dualquat_trans_casadi()
get_quat  = dualquat_quat_casadi()
dual_twist = dual_velocity_casadi()              # mapea (wb, vi) -> dual twist
velocity_from_twist = velocities_from_twist_casadi()
forward_kinematics_f = forward_kinematics_casadi()
jacobian = jacobian_casadi()
dualquat_from_pose = dualquat_from_pose_casadi()
error_dual = error_dual_aux_casadi()
ln_dual = ln_dual_aux_casadi()
rotation_i = rotation_casadi()                   # rota ω_b -> ω_i si lo necesitas

# ======= Estado global =======
joint_angles = np.zeros(7)
_last_twist_msg = None
_last_twist_time = None

# ======= Callbacks =======
def joint_state_callback(msg: JointState):
    global joint_angles
    if len(msg.position) >= 7:
        joint_angles = np.array(msg.position[:7])
    else:
        rospy.logwarn("JointState con menos de 7 articulaciones.")

def ee_twist_callback(msg: TwistStamped):
    global _last_twist_msg, _last_twist_time
    _last_twist_msg = msg
    _last_twist_time = msg.header.stamp if msg.header.stamp!=rospy.Time() else rospy.Time.now()

# ======= Helpers =======
def clamp_joint_speeds(omega, vmax):
    omega = np.asarray(omega).copy()
    omega = np.clip(omega, -vmax, vmax)                 # límite por eje
    max_norm = np.linalg.norm(omega)
    if max_norm > vmax * 1.5:                           # límite global suave
        omega *= (vmax * 1.5) / max_norm
    return omega

def get_params():
    params = {}
    params["lambda_damp"] = rospy.get_param("~lambda_damp", 1e-4)
    params["vmax"]        = rospy.get_param("~vmax", 0.7)           # rad/s por junta
    params["sample_time"] = rospy.get_param("~sample_time", 0.01)   # 100 Hz por defecto
    params["ee_twist_topic"] = rospy.get_param("~ee_twist_topic", "/dq_control/ee_twist")
    params["twist_timeout"]  = rospy.get_param("~twist_timeout", 0.2)  # s sin comandos -> paro
    params["warmup_cycles"]  = rospy.get_param("~warmup_cycles", 20)
    return params

def send_joint_velocity(pub, omega):
    msg = Base_JointSpeeds()
    for i in range(omega.shape[0]):
        sp = JointSpeed()
        sp.joint_identifier = i
        sp.value = float(omega[i])   # Kortex usa rad/s
        sp.duration = 0
        msg.joint_speeds.append(sp)
    pub.publish(msg)

def compute_dual_twist_from_msg(twist_msg: TwistStamped):
    """
    Convención: ω en marco cuerpo (body), v en marco inercial (world).
    TwistStamped no codifica el frame; asegúrate de publicar con esta convención.
    """
    wb = ca.DM([twist_msg.twist.angular.x,
                twist_msg.twist.angular.y,
                twist_msg.twist.angular.z])    # ω_b

    vi = ca.DM([twist_msg.twist.linear.x,
                twist_msg.twist.linear.y,
                twist_msg.twist.linear.z])     # v_i (inercial)

    # Construimos el dual twist 8x1 como [0, wb; 0, vi]
    dual_tw = ca.vertcat(0.0, wb, 0.0, vi)     # (8,1)
    return dual_tw

def dq_diff_ik(dual_twist_cmd, q, lam=1e-4):
    """
    dual_twist_cmd: DM (8x1)
    q: np.array (7,)
    """
    J = jacobian(q)                 # (8x7)
    I = ca.DM.eye(8)
    # Pseudo-inversa amortiguada (estable)
    Jpinv = J.T @ ca.pinv(J @ J.T + lam * I)   # (7x8)
    dq = 2.0 * (Jpinv @ dual_twist_cmd)        # (7x1)   (factor 2 típico en formulación DQ)
    return np.array(dq).reshape((7,))

# ======= Main =======
def main():
    p = get_params()

    # ROS I/O
    joint_states_sub = rospy.Subscriber("/my_gen3/joint_states", JointState, joint_state_callback, queue_size=50)
    ee_twist_sub = rospy.Subscriber(p["ee_twist_topic"], TwistStamped, ee_twist_callback, queue_size=50)
    joint_velocity_pub = rospy.Publisher("/my_gen3/in/joint_velocity", Base_JointSpeeds, queue_size=10)

    # Frecuencia de bucle
    hz = max(1, int(round(1.0 / p["sample_time"])))
    rate = rospy.Rate(hz)

    rospy.loginfo("DQ Diff-IK control por Twist. Topic: %s", p["ee_twist_topic"])

    # Warmup para llenar joint_states
    for _ in range(p["warmup_cycles"]):
        if rospy.is_shutdown(): break
        rate.sleep()

    try:
        while not rospy.is_shutdown():
            now = rospy.Time.now()

            # Watchdog de comandos
            fresh = False
            if _last_twist_time is not None:
                age = (now - _last_twist_time).to_sec()
                fresh = (age <= p["twist_timeout"])

            if fresh and _last_twist_msg is not None:
                # 1) leer estados actuales
                q_cur = joint_angles.copy()

                # 2) construir dual twist (8x1) desde el TwistStamped
                dual_tw_cmd = compute_dual_twist_from_msg(_last_twist_msg)

                # 3) IK diferencial en DQ -> velocidades articulares
                qdot = dq_diff_ik(dual_tw_cmd, q_cur, lam=p["lambda_damp"])

                # 4) saturación
                qdot = clamp_joint_speeds(qdot, p["vmax"])

                # 5) publicar a Kortex
                send_joint_velocity(joint_velocity_pub, qdot)

            else:
                # Sin comandos frescos: seguridad -> parar
                send_joint_velocity(joint_velocity_pub, np.zeros(7))

            rate.sleep()

    finally:
        # Paro seguro
        send_joint_velocity(joint_velocity_pub, np.zeros(7))
        rospy.loginfo("DQ Diff-IK detenido: velocidades a cero.")

if __name__ == "__main__":
    try:
        rospy.init_node("dq_diffik_twist_control", disable_signals=True, anonymous=True)
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass

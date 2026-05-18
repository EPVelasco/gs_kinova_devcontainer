#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from kortex_driver.msg import Base_JointSpeeds, JointSpeed

# Jacobiano geométrico 6x7 del Kinova Gen3
from kinova_gen3.kinematics.jacobian import jacobian as kinova_jacobian


class DQTwistController(object):
    def __init__(self):
        rospy.init_node("dq_twist_controller", anonymous=True)

        # Parámetros
        self.robot_name = rospy.get_param("~robot_name", "my_gen3")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.ee_frame = rospy.get_param("~ee_frame", "gs_camera_pcloud") # frame del sensor para simulacion
        self.twist_topic = rospy.get_param("~twist_topic", "/dq_control/ee_twist")

        self.damping = rospy.get_param("~damping", 0.03)
        self.max_joint_speed = rospy.get_param("~max_joint_speed", 0.1)
        self.cmd_timeout = rospy.get_param("~cmd_timeout", 0.25)
        self.publish_rate = rospy.get_param("~publish_rate", 100.0)

        # Orden esperado de articulaciones
        self.expected_joint_names = rospy.get_param(
            "~joint_names",
            [
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
                "joint_7",
            ],
        )

        # Estado interno
        self.joint_positions = np.zeros(7, dtype=float)
        self.have_joint_state = False

        self.last_twist_msg = None
        self.last_twist_time = None

        self.tf_listener = tf.TransformListener()

        # Subs / pubs
        self.joint_sub = rospy.Subscriber(
            "/" + self.robot_name + "/joint_states",
            JointState,
            self.joint_state_cb,
            queue_size=10,
        )

        self.twist_sub = rospy.Subscriber(
            self.twist_topic,
            TwistStamped,
            self.twist_cb,
            queue_size=10,
        )

        self.joint_vel_pub = rospy.Publisher(
            "/" + self.robot_name + "/in/joint_velocity",
            Base_JointSpeeds,
            queue_size=10,
        )

        rospy.loginfo("dq_twist_controller iniciado")
        rospy.loginfo("  twist_topic: %s", self.twist_topic)
        rospy.loginfo("  base_frame : %s", self.base_frame)
        rospy.loginfo("  ee_frame   : %s", self.ee_frame)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def joint_state_cb(self, msg):
        """
        Reordena las articulaciones por nombre.
        Esto evita errores si el JointState no viene en el orden esperado.
        """
        if len(msg.name) == 0 or len(msg.position) == 0:
            return

        name_to_pos = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                name_to_pos[name] = msg.position[i]

        q = []
        missing = []
        for name in self.expected_joint_names:
            if name in name_to_pos:
                q.append(name_to_pos[name])
            else:
                missing.append(name)

        if missing:
            rospy.logwarn_throttle(
                2.0,
                "Faltan articulaciones en JointState: %s. "
                "Se usará fallback por índice si es posible." % str(missing),
            )

            if len(msg.position) >= 7:
                self.joint_positions = np.array(msg.position[:7], dtype=float)
                self.have_joint_state = True
            return

        self.joint_positions = np.array(q, dtype=float)
        self.have_joint_state = True

    def twist_cb(self, msg):
        self.last_twist_msg = msg
        self.last_twist_time = rospy.Time.now()

    # ------------------------------------------------------------------
    # Utilidades
    # ------------------------------------------------------------------
    def get_rotation_base_to_ee(self):
        """
        Devuelve R_base_ee, que rota un vector expresado en ee_frame
        hacia base_frame:
            v_base = R_base_ee @ v_ee
        """
        try:
            self.tf_listener.waitForTransform(
                self.base_frame,
                self.ee_frame,
                rospy.Time(0),
                rospy.Duration(0.2),
            )

            (trans, quat) = self.tf_listener.lookupTransform(
                self.base_frame,
                self.ee_frame,
                rospy.Time(0),
            )

            T = tf.transformations.quaternion_matrix(quat)
            R = T[:3, :3]
            return R

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(
                1.0,
                "No se pudo obtener TF entre %s y %s" % (self.base_frame, self.ee_frame),
            )
            return None

    def convert_twist_to_base_frame(self, twist_msg):
        """
        Convierte el twist recibido al frame base.

        Convenciones:
        - Si frame_id == base_link:
            se asume que lineal y angular ya están en base.
        - Si frame_id == end_effector_link:
            se asume que lineal y angular están ambas en efector final.
        """
        frame_id = twist_msg.header.frame_id.strip()

        v = np.array([
            twist_msg.twist.linear.x,
            twist_msg.twist.linear.y,
            twist_msg.twist.linear.z
        ], dtype=float)

        w = np.array([
            twist_msg.twist.angular.x,
            twist_msg.twist.angular.y,
            twist_msg.twist.angular.z
        ], dtype=float)

        if frame_id in ["", self.base_frame, "base_link", "world", "base"]:
            v_base = v
            w_base = w
            return v_base, w_base

        elif frame_id in [self.ee_frame, "end_effector_link", "tool", "ee"]:
            R_base_ee = self.get_rotation_base_to_ee()
            if R_base_ee is None:
                return None, None

            v_base = np.dot(R_base_ee, v)
            w_base = np.dot(R_base_ee, w)
            return v_base, w_base

        else:
            rospy.logwarn_throttle(
                1.0,
                "frame_id '%s' no reconocido. Se asume %s." % (frame_id, self.base_frame),
            )
            v_base = v
            w_base = w
            return v_base, w_base

    def compute_qdot_dls(self, q, v_base, w_base):
        """
        Resuelve:
            xdot = J qdot
        con Damped Least Squares

        Asume que J = kinova_jacobian(q) devuelve 6x7 con orden:
            [vx, vy, vz, wx, wy, wz]
        en base_frame.
        """
        xdot = np.hstack((v_base, w_base)).reshape((6, 1))

        J = np.array(kinova_jacobian(q), dtype=float)

        if J.shape != (6, 7):
            raise RuntimeError(
                "El Jacobiano debe ser 6x7. Se obtuvo shape=%s" % (str(J.shape),)
            )

        lam = self.damping
        JJt = np.dot(J, J.T)
        inv = np.linalg.inv(JJt + (lam ** 2) * np.eye(6))
        qdot = np.dot(J.T, np.dot(inv, xdot)).reshape((7,))

        return qdot

    def saturate_qdot(self, qdot):
        qdot = np.array(qdot, dtype=float)

        # saturación componente a componente
        qdot = np.clip(qdot, -self.max_joint_speed, self.max_joint_speed)

        return qdot

    def publish_joint_speeds(self, qdot):
        msg = Base_JointSpeeds()

        for i in range(7):
            js = JointSpeed()
            js.joint_identifier = i
            js.value = float(qdot[i])
            js.duration = 0
            msg.joint_speeds.append(js)

        self.joint_vel_pub.publish(msg)

    def publish_zero(self):
        self.publish_joint_speeds(np.zeros(7))

    def cmd_is_fresh(self):
        if self.last_twist_time is None:
            return False
        dt = (rospy.Time.now() - self.last_twist_time).to_sec()
        return dt <= self.cmd_timeout

    # ------------------------------------------------------------------
    # Loop principal
    # ------------------------------------------------------------------
    def run(self):
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown():
            try:
                if (not self.have_joint_state) or (self.last_twist_msg is None) or (not self.cmd_is_fresh()):
                    self.publish_zero()
                    rate.sleep()
                    continue

                q = self.joint_positions.copy()

                v_base, w_base = self.convert_twist_to_base_frame(self.last_twist_msg)
                if v_base is None or w_base is None:
                    self.publish_zero()
                    rate.sleep()
                    continue

                qdot = self.compute_qdot_dls(q, v_base, w_base)
                qdot = self.saturate_qdot(qdot)

                self.publish_joint_speeds(qdot)
                rate.sleep()

            except Exception as e:
                rospy.logerr_throttle(1.0, "Error en dq_twist_controller: %s" % str(e))
                self.publish_zero()
                rate.sleep()


if __name__ == "__main__":
    controller = DQTwistController()
    controller.run()
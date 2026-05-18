#!/usr/bin/env python3
import rospy
import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from kortex_driver.msg import Base_JointSpeeds, JointSpeed

# TF utilities for obtaining the base->end-effector transform
import tf
from tf.transformations import quaternion_matrix

from kinova_gen3.kinematics.jacobian import jacobian as kinova_jacobian


class CartesianVelocityController(object):
    """
    Cartesian velocity controller for Kinova Gen3.
    The node receives an end-effector velocity expressed in the tool frame,
    converts it to base frame using the adjoint representation,
    and computes joint velocities via damped least-squares inverse kinematics.
    """

    def __init__(self):

        # ------------------------ Parameters ------------------------
        self.robot_name = rospy.get_param("~robot_name", "my_gen3")
        self.damping = rospy.get_param("~damping", 0.01)  # Damping factor λ for DLS IK
        self.max_joint_speed = rospy.get_param("~max_joint_speed", 0.1)  # rad/s

        # Reference frames
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.ee_frame   = rospy.get_param("~ee_frame",   "gs_camera_pcloud")
        self.tf_listener = tf.TransformListener()

        # Joint state tracking
        self.joint_names = ["joint_1", "joint_2", "joint_3",
                            "joint_4", "joint_5", "joint_6", "joint_7"]
        self.q = np.zeros(7)
        self.has_joint_state = False

        # End-effector twist expressed in the EE frame (body twist)
        self.v_ee_body = np.zeros(6)

        # ------------------------ ROS Interfaces ------------------------
        rospy.Subscriber("/my_gen3/joint_states", JointState, self.joint_state_cb)
        rospy.Subscriber("/ee_cartesian_vel", Twist, self.cartesian_vel_cb)

        topic_joint_vel = "/" + self.robot_name + "/in/joint_velocity"
        self.pub_joint_vel = rospy.Publisher(topic_joint_vel,
                                             Base_JointSpeeds,
                                             queue_size=10)

        rospy.loginfo("Using base_frame=%s, ee_frame=%s",
                      self.base_frame, self.ee_frame)

    # ======================== Callbacks ============================

    def joint_state_cb(self, msg):
        """
        JointState callback: updates the vector q[7].
        Ensures joint ordering consistent with the Kinova model.
        """
        q_tmp = np.zeros(7)
        name_to_pos = dict(zip(msg.name, msg.position))

        for i, name in enumerate(self.joint_names):
            if name in name_to_pos:
                q_tmp[i] = name_to_pos[name]
            else:
                # If a joint name is missing, skip the update
                return

        self.q = q_tmp
        self.has_joint_state = True

    def cartesian_vel_cb(self, msg):
        """
        Receives an end-effector twist expressed in the EE frame.
        """
        self.v_ee_body = np.array([
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z
        ])

    # =================== Adjoint Transformation =====================

    def get_adjoint_base_ee(self):
        """
        Computes the adjoint matrix Ad_{T_base_ee} for transforming a twist
        from the end-effector frame to the base frame:
            V_base = Ad(T_base_ee) * V_ee
        """
        try:
            self.tf_listener.waitForTransform(
                self.base_frame, self.ee_frame,
                rospy.Time(0), rospy.Duration(0.05)
            )
            (trans, rot) = self.tf_listener.lookupTransform(
                self.base_frame, self.ee_frame, rospy.Time(0)
            )
        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
            rospy.logwarn_throttle(
                2.0,
                "TF %s -> %s unavailable: %s",
                self.base_frame, self.ee_frame, str(e)
            )
            return None

        # Translation and rotation
        p = np.array(trans).reshape((3,))
        R = quaternion_matrix(rot)[0:3, 0:3]

        # Skew-symmetric matrix of p
        px = np.array([[0,      -p[2],  p[1]],
                       [p[2],    0,    -p[0]],
                       [-p[1],   p[0],  0   ]])

        # Full adjoint representation matrix (6x6)
        Ad = np.zeros((6, 6))
        Ad[0:3, 0:3] = R
        Ad[3:6, 3:6] = R
        Ad[3:6, 0:3] = px.dot(R)

        return Ad

    # =================== IK: Twist -> Joint Velocities ===============

    def compute_dq_from_twist(self):
        """
        Computes joint velocities from a body-frame end-effector twist using:
            dq = Jᵀ (J Jᵀ + λ² I)⁻¹ v_s

        where:
            v_body : twist expressed in EE frame
            v_s    : same twist expressed in base frame: v_s = Ad * v_body
            J      : spatial Jacobian expressed in base frame
        """
        if not self.has_joint_state:
            return np.zeros(7)

        # Get adjoint transform base<-ee
        Ad = self.get_adjoint_base_ee()
        if Ad is None:
            return np.zeros(7)

        v_body = self.v_ee_body.copy().reshape((6,))
        if np.linalg.norm(v_body) < 1e-6:
            return np.zeros(7)

        # Transform twist to base frame
        v_s = Ad.dot(v_body)

        # Joint configuration
        q = self.q.copy()

        # Spatial Jacobian J(q) (6x7)
        J = np.asarray(kinova_jacobian(q)).reshape((6, 7))

        # Damped least-squares inverse
        lam2 = self.damping ** 2
        JJt = J.dot(J.T)
        A = JJt + lam2 * np.eye(6)

        # Solve (J Jᵀ + λ²I) x = v_s
        x = np.linalg.solve(A, v_s)

        # Compute dq = Jᵀ x
        dq = J.T.dot(x)

        # Joint speed saturation
        dq = np.clip(dq, -self.max_joint_speed, self.max_joint_speed)

        return dq

    # ======================== ROS Publisher =========================

    def publish_joint_speeds(self, dq):
        """
        Publishes joint velocities to the Kortex driver interface.
        """
        msg = Base_JointSpeeds()
        for i in range(7):
            js = JointSpeed()
            js.joint_identifier = int(i)
            js.value = float(dq[i])
            js.duration = 0   # Required: uint32
            msg.joint_speeds.append(js)
        self.pub_joint_vel.publish(msg)

    # ============================== Loop ============================

    def spin(self):
        rate = rospy.Rate(100.0)  # Control frequency
        while not rospy.is_shutdown():
            dq = self.compute_dq_from_twist()
            self.publish_joint_speeds(dq)
            rate.sleep()


# ===================================================================

if __name__ == "__main__":
    rospy.init_node("ee_cartesian_velocity_controller")
    ctrl = CartesianVelocityController()
    rospy.loginfo("Cartesian EE velocity controller initialized (Twist in EE frame)")
    ctrl.spin()
#!/usr/bin/env python
import rospy
import numpy as np
import tf.transformations as tft
from geometry_msgs.msg import Wrench, Twist
from gazebo_msgs.msg import LinkStates

class PD6DOFController:
    def __init__(self):
        rospy.init_node("gelsight_pd_6dof_controller")

        # Ganancias PD
        self.Kp_lin = np.array([2.0, 2.0, 2.0])
        self.Kd_lin = np.array([0.3, 0.3, 0.3])
        self.Kp_ang = np.array([0.01, 0.01, 0.01])
        self.Kd_ang = np.array([0.001, 0.001, 0.001])
        self.max_torque = 0.01

        self.link_name = "gelsight_sensor::base_link"

        self.vel_des = np.zeros(6)  # [vx, vy, vz, wx, wy, wz] en base_link
        self.vel_act = np.zeros(6)  # actual también en base_link
        self.prev_error = np.zeros(6)
        self.prev_time = rospy.Time.now()

        self.R_base_to_world = np.identity(3)  # inicializar como identidad

        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.link_states_cb)
        self.pub = rospy.Publisher("/applied_force", Wrench, queue_size=1)

        self.control_loop()

    def cmd_vel_cb(self, msg):
        self.vel_des[0:3] = [msg.linear.x, msg.linear.y, msg.linear.z]
        self.vel_des[3:6] = [msg.angular.x, msg.angular.y, msg.angular.z]

    def link_states_cb(self, msg):
        try:
            idx = msg.name.index(self.link_name)
            twist = msg.twist[idx]
            pose = msg.pose[idx]

            # Rotación: world → base_link
            q = pose.orientation
            quat = [q.x, q.y, q.z, q.w]
            R_world_to_base = tft.quaternion_matrix(quat)[0:3, 0:3].T
            self.R_base_to_world = R_world_to_base.T  # Guardar para el control_loop

            # Velocidades en world
            v_world = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
            w_world = np.array([twist.angular.x, twist.angular.y, twist.angular.z])

            # Transformar a base_link
            v_base = R_world_to_base.dot(v_world)
            w_base = R_world_to_base.dot(w_world)

            self.vel_act[0:3] = v_base
            self.vel_act[3:6] = w_base

        except ValueError:
            pass

    def control_loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - self.prev_time).to_sec()
            if dt == 0.0:
                rate.sleep()
                continue

            # Control PD
            error = self.vel_des - self.vel_act
            d_error = (error - self.prev_error) / (dt + 1e-9)

            force = self.Kp_lin * error[0:3] + self.Kd_lin * d_error[0:3]
            torque = self.Kp_ang * error[3:6] + self.Kd_ang * d_error[3:6]
            torque = np.clip(torque, -self.max_torque, self.max_torque)

            # Transformar a frame world
            force_world = self.R_base_to_world.dot(force)
            torque_world = self.R_base_to_world.dot(torque)

            # Publicar en world
            wrench = Wrench()
            wrench.force.x, wrench.force.y, wrench.force.z = force_world
            wrench.torque.x, wrench.torque.y, wrench.torque.z = torque_world
            self.pub.publish(wrench)

            self.prev_error = error
            self.prev_time = now
            rate.sleep()

if __name__ == "__main__":
    try:
        PD6DOFController()
    except rospy.ROSInterruptException:
        pass

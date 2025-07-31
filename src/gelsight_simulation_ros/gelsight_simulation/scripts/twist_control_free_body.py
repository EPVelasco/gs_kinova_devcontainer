#!/usr/bin/env python
import rospy
import numpy as np
import tf
import tf.transformations as tft
from geometry_msgs.msg import Wrench, Twist
from gazebo_msgs.msg import LinkStates, ModelStates

class GelsightController:
    def __init__(self):
        rospy.init_node("gelsight_combined_controller")

        # === TF Broadcaster ===
        self.br = tf.TransformBroadcaster()
        self.last_stamp = rospy.Time(0)
        self.last_pose = None

        # === PD Controller gains ===
        self.Kp_lin = np.array([2.0, 2.0, 2.0])       # Proportional gains (linear)
        self.Kd_lin = np.array([0.3, 0.3, 0.3])       # Derivative gains (linear)
        self.Kp_ang = np.array([0.01, 0.01, 0.01])    # Proportional gains (angular)
        self.Kd_ang = np.array([0.001, 0.001, 0.001]) # Derivative gains (angular)
        self.max_torque = 0.01                        # Max torque per axis

        # === Sensor model and link names ===
        self.model_name = 'gelsight_sensor'
        self.link_name = self.model_name + '::base_link'

        # Desired and actual velocity (vx, vy, vz, wx, wy, wz)
        self.vel_des = np.zeros(6)
        self.vel_act = np.zeros(6)

        self.prev_error = np.zeros(6)
        self.prev_time = rospy.Time.now()

        # Rotation matrix from base_link to world
        self.R_base_to_world = np.identity(3)

        # === Subscribers and Publisher ===
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_cb)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.link_states_cb)
        self.pub = rospy.Publisher("/applied_control_force_gs", Wrench, queue_size=1)

        # Start control loop
        self.control_loop()

    def cmd_vel_cb(self, msg):
        # Receive desired linear and angular velocities
        self.vel_des[0:3] = [msg.linear.x, msg.linear.y, msg.linear.z]
        self.vel_des[3:6] = [msg.angular.x, msg.angular.y, msg.angular.z]

    def model_states_cb(self, msg):
        # Publish TF from world to base_link using Gazebo pose
        try:
            idx = msg.name.index(self.model_name)
            pose = msg.pose[idx]
            position = (pose.position.x, pose.position.y, pose.position.z)
            orientation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

            stamp = rospy.Time.now()
            # Ensure stamp increases to avoid TF_REPEATED_DATA warning
            if stamp <= self.last_stamp:
                stamp = self.last_stamp + rospy.Duration.from_sec(1e-6)

            self.br.sendTransform(position, orientation, stamp, "base_link", "world")
            self.last_stamp = stamp
            self.last_pose = (position, orientation)

        except ValueError:
            pass

    def link_states_cb(self, msg):
        # Extract actual velocity from Gazebo link_states and convert to base_link frame
        try:
            idx = msg.name.index(self.link_name)
            twist = msg.twist[idx]
            pose = msg.pose[idx]

            # Convert orientation to rotation matrix
            q = pose.orientation
            quat = [q.x, q.y, q.z, q.w]
            R_world_to_base = tft.quaternion_matrix(quat)[0:3, 0:3].T
            self.R_base_to_world = R_world_to_base.T

            # Velocities in world frame
            v_world = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
            w_world = np.array([twist.angular.x, twist.angular.y, twist.angular.z])

            # Convert to base_link frame
            v_base = R_world_to_base.dot(v_world)
            w_base = R_world_to_base.dot(w_world)

            self.vel_act[0:3] = v_base
            self.vel_act[3:6] = w_base

        except ValueError:
            pass

    def control_loop(self):
        # Main control loop running at 50 Hz
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - self.prev_time).to_sec()
            if dt == 0.0:
                rate.sleep()
                continue

            # PD control
            error = self.vel_des - self.vel_act
            d_error = (error - self.prev_error) / (dt + 1e-9)

            force = self.Kp_lin * error[0:3] + self.Kd_lin * d_error[0:3]
            torque = self.Kp_ang * error[3:6] + self.Kd_ang * d_error[3:6]
            torque = np.clip(torque, -self.max_torque, self.max_torque)

            # Transform to world frame
            force_world = self.R_base_to_world.dot(force)
            torque_world = self.R_base_to_world.dot(torque)

            # Publish as Wrench
            wrench = Wrench()
            wrench.force.x, wrench.force.y, wrench.force.z = force_world
            wrench.torque.x, wrench.torque.y, wrench.torque.z = torque_world
            self.pub.publish(wrench)

            self.prev_error = error
            self.prev_time = now
            rate.sleep()

if __name__ == "__main__":
    try:
        GelsightController()
    except rospy.ROSInterruptException:
        pass

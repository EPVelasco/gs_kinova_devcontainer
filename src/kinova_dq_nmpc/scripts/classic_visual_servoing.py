#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np

class ZeroCmdFromFeatures:
    def __init__(self):
        # Publisher for cmd_vel
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber for features
        rospy.Subscriber('/gs_feature_coords', Float32MultiArray, self.features_callback)

        rospy.loginfo("Visual Servoing Controller")

        # Define focal distance 
        self.f = 1

        # Camera u_max and v _max
        self.v_max = 240/2
        self.u_max = 320/2

    def jacobian(self, p, z):
        # Get pixels values 
        u = p[0, 0]
        v = p[1, 0]


        # Get pixels respect to the center of the image
        u_c = u - self.u_max
        v_c = v - self.v_max

        j11 = -(u_c * v_c)/self.f
        j12 = (self.f**2 + u_c**2)/self.f
        j13 = v_c
        j14 = (self.f/z)
        j15 = 0.0
        j16 = (u_c/z)

        j21 = -(self.f**2 + v_c**2)/self.f
        j22 = (u_c * v_c)/self.f
        j23 = -u_c
        j24 = 0.0
        j25 = (self.f/z)
        j26 = (v_c/z)

        J = np.array([[j11, j12, j13, j14, j15, j16], [j21, j22, j23, j24, j25, j26]])

        return J
        
    def features_callback(self, msg):
        # Extract Point from the data
        p1 = np.array([msg.data[4], msg.data[5]]).reshape(2,1)
        z1 = msg.data[8]

        p2 = np.array([msg.data[6], msg.data[7]]).reshape(2,1)
        z2 = msg.data[9]

        p3 = np.array([msg.data[2], msg.data[3]]).reshape(2,1)
        z3 = z2

        # Compute Jacobians
        J_p1 = self.jacobian(p1, z1)  # expect 2×6
        J_p2 = self.jacobian(p2, z2)
        J_p3 = self.jacobian(p3, z3)

        J = np.vstack((J_p1, J_p2, J_p3))  # 6×6

        # Desired features (centered)
        desired = np.array([self.u_max, self.v_max]).reshape(2,1)
        features = np.vstack((p1, p2, p3))              
        desired_features = np.vstack((desired, desired, desired))

        error = desired_features - features  # 6×1
        # Gain matrix
        K = 0.01*np.diag([0.0, 1.0, 0.0, 1.0, 0.0, 1.0])  # 6×6

        # Control law
        u = np.linalg.pinv(J) @ (K @ error)
        print(u)

        # Create a Twist with all zeros
        zero_twist = Twist()
        zero_twist.linear.x = u[3, 0]
        zero_twist.linear.y = u[4, 0]
        zero_twist.linear.z = u[5, 0]
        zero_twist.angular.x = u[0, 0]
        zero_twist.angular.y = u[1, 0]
        zero_twist.angular.z = u[2, 0]

        # Publish zero command
        self.cmd_pub.publish(zero_twist)
        rospy.loginfo("Published cmd_vel")

if __name__ == '__main__':
    rospy.init_node('visual_servoing')
    ZeroCmdFromFeatures()
    rospy.spin()
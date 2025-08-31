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
        self.f = 100

        # Camera u_max and v _max
        self.v_max = 240/2
        self.u_max = 320/2

        self.h = 240/2
        self.w = 320/2

    def jacobian(self, p, z):
        # Get pixels values 
        u = p[0, 0]
        v = p[1, 0]

        # Get pixels respect to the center of the image
        u_c = u - self.u_max
        v_c = v - self.v_max

        ## Compute point in the camera frame at the moment of contact
        x = u_c*z/self.f
        y = v_c*z/self.f
  

        j11 = -(self.f/z)
        j12 = 0.0
        j13 = (u_c/z)
        j14 = (u_c * v_c)/self.f
        j15 = -(self.f**2 + u_c**2)/self.f
        j16 = v_c

        j21 = 0.0
        j22 = -(self.f/z)
        j23 = (v_c/z)
        j24 = (self.f**2 + v_c**2)/self.f
        j25 = -(u_c * v_c)/self.f
        j26 = -u_c

        j31 = 0.0
        j32 = 0.0
        j33 =-1.0
        j34 = -y
        j35 = x
        j36 = 0.0


        J = np.array([[j11, j12, j13, j14, j15, j16], [j21, j22, j23, j24, j25, j26], [j31, j32, j33, j34, j35, j36]])
        #J = np.array([[j11, j12, j13, j14, j15, j16], [j21, j22, j23, j24, j25, j26]])

        return J

    def jacobian_decopled(self, p, z):
        # Get pixels values 
        u = p[0, 0]
        v = p[1, 0]

        # Get pixels respect to the center of the image
        u_c = u - self.u_max
        v_c = v - self.v_max

        ## Compute point in the camera frame at the moment of contact
        x = u_c*z/self.f
        y = v_c*z/self.f
  

        j11 = -(self.f/z)
        j12 = 0.0
        j13 = (u_c/z)
        j14 = (u_c * v_c)/self.f
        j15 = -(self.f**2 + u_c**2)/self.f
        j16 = v_c

        j21 = 0.0
        j22 = -(self.f/z)
        j23 = (v_c/z)
        j24 = (self.f**2 + v_c**2)/self.f
        j25 = -(u_c * v_c)/self.f
        j26 = -u_c

        J = np.array([[j11, j12, j13, j14, j15, j16], [j21, j22, j23, j24, j25, j26]])
        J_xy = np.array([[j11, j12, j14, j15], [j21, j22, j24, j25]])
        J_z = np.array([[j13, j16], [j23, j26]])
        return J_xy, J_z

    def theta_jacobian(self, p1, p2):
        # Get pixels values 
        u1 = p1[0, 0]
        v1 = p1[1, 0]

        # Get pixels values 
        u2 = p2[0, 0]
        v2 = p2[1, 0]
        ## [-(v1 - v2)/((u1 - u2)^2 + (v1 - v2)^2), (u1 - u2)/((u1 - u2)^2 + (v1 - v2)^2), (v1 - v2)/((u1 - u2)^2 + (v1 - v2)^2), -(u1 - u2)/((u1 - u2)^2 + (v1 - v2)^2), 0, 0]

        j11 = -(v1 - v2)/((u1 - u2)**2 + (v1 - v2)**2)
        j12 = (u1 - u2)/((u1 - u2)**2 + (v1 - v2)**2)
        j13 = (v1 - v2)/((u1 - u2)**2 + (v1 - v2)**2)
        j14 = -(u1 - u2)/((u1 - u2)**2 + (v1 - v2)**2)
        J = np.array([[j11, j12, j13, j14]])
        return J

    def phi_jacobian(self, p1, z1, p2, z2):
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

        ## [(z1 - z2)/((u1 - u2)^2 + (z1 - z2)^2), 0, -(z1 - z2)/((u1 - u2)^2 + (z1 - z2)^2), 0, -(u1 - u2)/((u1 - u2)^2 + (z1 - z2)^2), (u1 - u2)/((u1 - u2)^2 + (z1 - z2)^2)]
        J = np.array([[j11, j12, j13, j14, j15, j16]])
        return J

    def r_jacobian(self, p1, p2):
        # Get pixels values 
        u1 = p1[0, 0]
        v1 = p1[1, 0]

        # Get pixels values 
        u2 = p2[0, 0]
        v2 = p2[1, 0]

        h = self.h
        w = self.w

        j11 = ((v1 - v2)*(h*v1 - h*v2 + 2*u1*u2 - u1*w + u2*w + 2*v1*v2 - 2*u2**2 - 2*v1**2))/(2*((u1 - u2)**2 + (v1 - v2)**2)**(3/2))
        j12 = -((u1 - u2)*(h*v1 - h*v2 - 2*u1*u2 - u1*w + u2*w - 2*v1*v2 + 2*u1**2 + 2*v2**2))/(2*((u1 - u2)**2 + (v1 - v2)**2)**(3/2))
        j13 = -((v1 - v2)*(h*v1 - h*v2 - 2*u1*u2 - u1*w + u2*w - 2*v1*v2 + 2*u1**2 + 2*v2**2))/(2*((u1 - u2)**2 + (v1 - v2)**2)**(3/2))
        j14 = ((u1 - u2)*(h*v1 - h*v2 + 2*u1*u2 - u1*w + u2*w + 2*v1*v2 - 2*u2**2 - 2*v1**2))/(2*((u1 - u2)**2 + (v1 - v2)**2)**(3/2))
        J = np.array([[j11, j12, j13, j14]])
        return J

    def r_theta_control(self, p1, z1, p2, z2):
        # Get pixels values 
        u1 = p1[0, 0]
        v1 = p1[1, 0]

        # Get pixels respect to the center of the image
        u1_c = u1 - self.u_max
        v1_c = v1 - self.v_max

        # Get pixels values 
        u2 = p2[0, 0]
        v2 = p2[1, 0]

        # Get pixels respect to the center of the image
        u2_c = u2 - self.u_max
        v2_c = v2 - self.v_max

        # Compute theta
        du = u2_c - u1_c
        dv = v2_c - v1_c
        dz = (z2 - z1)
        #du_aux = 0.0001*(u2_c - u1_c)
        du_aux = (u2_c - u1_c)

        # Angle of the feature
        theta = np.arctan2(dv, du)
        theta = theta

        # Inclination angles
        phi = np.arctan2(dz, du_aux)

        # Distance to the center 
        um = (u1_c + u2_c)/2
        vm = (v1_c + v2_c)/2
        r = um*np.sin(theta) + vm*np.cos(theta)


        # Jacobian Control
        jacobian_theta = self.theta_jacobian(p1, p2)
        jacobian_phi = self.phi_jacobian(p1, z1,  p2, z2)

        #J_features = np.vstack((jacobian_theta, jacobian_phi))  # 6×6

        # Compute Jacobians
        J_xy_p1, J_z_p1 = self.jacobian_decopled(p1, z1)
        J_xy_p2, J_z_p2 = self.jacobian_decopled(p2, z2)


        J_image_xy = np.vstack((J_xy_p1, J_xy_p2)) 
        J_image_z = np.vstack((J_z_p1, J_z_p2)) 

        J_image_xy_theta = jacobian_theta@ J_image_xy
        J_image_xy_z = jacobian_theta@ J_image_z

        
        #J_image = np.vstack((J_p1, J_p2))  # 4×6
        ##J_image_theta = jacobian_theta@J_image

        ## Full Jacobian
        J = J_image_xy_z

        ### Desired features (centered)
        #desired = np.array([0.0, 0.0]).reshape(2,1)
        ##features = np.array([theta, phi]).reshape(2,1)              

        ## Desired Features pixels
        #desired = np.array([0.0, 0.0, 0.195]).reshape(3,1)
        #featuresnormalized = np.vstack((u1_c, v1_c, z1, u2_c, v2_c, z2))              
        #desired_features = np.vstack((desired, desired))

        ## Desired Features
        desired = np.array([0.1]).reshape(1,1)
        featuresnormalized = np.array([theta]).reshape(1,1)
        error_theta = desired - featuresnormalized  # 1×1
        #### Gain matrix
        K = 0.5*np.diag([1.0])  # 1×1

        #### Control law
        I = np.eye(2, 2)
        J_inv = np.linalg.pinv(J)
        K2 = 100*np.diag([1.0, 1.0])  # 6×6
        null_space = np.array([[0.0], [0.0]])
        twist_z = J_inv @ (K @ error_theta)  + (I - J_inv@J)@K2@null_space



        # Control Classic features
        desired = np.array([0.0, 30.0]).reshape(2,1)
        featuresnormalized = np.vstack((u1_c, v1_c))              
        error_p1 = desired - featuresnormalized  # 1×1
        I = np.eye(4, 4)
        K = 0.5*np.diag([0.0, 1.0])  # 1×1
        K2 = 100*np.diag([0.0, 0.0, 1.0, 1.0])  # 6×6
        null_space = np.array([[0.0], [0.0], [0.0], [0.0]])

        ## Control law
        u = np.linalg.pinv(J_xy_p1) @ (K @ error_p1 - J_z_p1@twist_z)+ (I - np.linalg.pinv(J_xy_p1)@J_xy_p1)@K2@null_space
        return u, twist_z
        
        
    def features_callback(self, msg):
        # Extract Point from the data p1 is the point that is in the too the way right
        p1 = np.array([msg.data[4], msg.data[5]]).reshape(2,1)
        z1 = msg.data[8]
        z1 = 10*z1

        # Extract Point from the data p1 is the point that is in the too the way left
        p2 = np.array([msg.data[6], msg.data[7]]).reshape(2,1)
        z2 = msg.data[9]
        z2 = 10*z2

        u, twist_z = self.r_theta_control(p2, z2, p1, z1)

        # Create a Twist with all zeros
        zero_twist = Twist()
        zero_twist.linear.x = u[0, 0]
        zero_twist.linear.y = u[1, 0]
        zero_twist.linear.z = twist_z[0, 0]

        zero_twist.angular.x = u[2, 0]
        zero_twist.angular.y = u[3, 0]
        zero_twist.angular.z = twist_z[1, 0]

        # Publish zero command
        self.cmd_pub.publish(zero_twist)
        rospy.loginfo("Published cmd_vel")

if __name__ == '__main__':
    rospy.init_node('visual_servoing')
    ZeroCmdFromFeatures()
    rospy.spin()
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

        self.h = 240
        self.w = 320

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
        return J

    def jacobian_decopled(self, p, z):
        # Get pixels values 
        u = p[0, 0]
        v = p[1, 0]

        # Get pixels respect to the center of the image
        u_c = u - self.u_max
        v_c = v - self.v_max

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
    def control_theta_z(self, p1, z1, p2, z2, gain, desired_theta, desired_z):
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

        # Angle of the feature
        theta = np.arctan2(dv, du)

        # Compute Jacobians decoupled
        J_xy_p1, J_z_p1 = self.jacobian_decopled(p1, z1)
        J_xy_p2, J_z_p2 = self.jacobian_decopled(p2, z2)

        # Coupled jacobians of the decopled ones
        J_image_z = np.vstack((J_z_p1, J_z_p2)) 

        # Jacobian theta
        jacobian_theta = self.theta_jacobian(p1, p2)

        J_image_xy_z = jacobian_theta@ J_image_z

        # Full Jacobian
        J = J_image_xy_z

        # Desired Features
        desired = np.array([desired_theta]).reshape(1,1)
        featuresnormalized = np.array([theta]).reshape(1,1)
        error_theta = desired - featuresnormalized  # 1×1

        #### Gain matrix
        K = gain*np.diag([1.0])  # 1×1

        #### Control law
        I = np.eye(2, 2)
        J_inv = np.linalg.pinv(J)
        K2 = 1*np.diag([1.0, 1.0])  # 6×6

        z_m = (z1+z2)/2.0
        # Be careful the way we define the error of position for z this si the inverse behavior of a z in the camera

        null_space = np.array([[z_m - desired_z], [0.0]])
        twist_z = J_inv @ (K @ error_theta)  + (I - J_inv@J)@K2@null_space
        return twist_z, error_theta
        
    def r_theta_control(self, p1, z1, p2, z2, gain_r, desired_r, gain_theta, desired_theta, desired_z):
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

        # Angle of the feature
        theta = np.arctan2(dv, du)
        phi = np.arctan2(dz, du)

        # Distance to the center 
        um = (u1_c + u2_c)/2
        vm = (v1_c + v2_c)/2
        r = um*np.sin(theta) + vm*np.cos(theta)

        jacobian_r = self.r_jacobian(p1, p2)

        # Compute Jacobians
        J_xy_p1, J_z_p1 = self.jacobian_decopled(p1, z1)
        J_xy_p2, J_z_p2 = self.jacobian_decopled(p2, z2)

        # Coupled jacobians of the decopled ones
        J_image_xy = np.vstack((J_xy_p1, J_xy_p2)) 
        J_image_z = np.vstack((J_z_p1, J_z_p2)) 

        # Jacobian of the theta and z
        J_r_decoupled_xy = jacobian_r@J_image_xy
        J_r_decoupled_z = jacobian_r@J_image_z

        # You can modify the desired theta and z (Be careful with the gain)
        twist_z, error_theta = self.control_theta_z(p1=p1, z1=z1, p2=p2, z2=z2, gain=gain_theta, desired_theta=desired_theta, desired_z=desired_z)

        # Desired Features
        desired = np.array([desired_r]).reshape(1,1)
        featuresnormalized = np.array([r]).reshape(1,1)
        error_r = desired - featuresnormalized  # 1×1

        # Gain matrix
        K = gain_r*np.diag([1.0])  # 1×1

        I = np.eye(4, 4)
        K2 = 1*np.diag([1, 0.0, 1.0, 100.0])  # 6×6

        # Compute error phi
        desired_phi = 0.0
        error_phi = desired_phi - phi

        # Vector for velocity regularization
        xi = np.array([2*error_theta[0, 0], 100*error_phi]) 
        velocity_x = (0.005)/(1 + np.linalg.norm(xi))
        null_space = np.array([[velocity_x], [0.0], [0.0], [error_phi]])

        ## Control law
        u = np.linalg.pinv(J_r_decoupled_xy) @ (K @ error_r - J_r_decoupled_z@twist_z)+ (I - np.linalg.pinv(J_r_decoupled_xy)@J_r_decoupled_xy)@K2@null_space
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

        # Here We do the control (We are not controlling pitch )
        u, twist_z = self.r_theta_control(p1=p2, z1=z2, p2=p1, z2=z1, gain_r=0.2, desired_r=0.0, gain_theta=0.5, desired_theta=0.0, desired_z=0.195)

        # limits
        MAX_LIN = 0.03   # m/s  (30 mm/s)
        MAX_ANG = 2.0    # rad/s

        def sat(val, lim):
            return float(np.clip(val, -lim, lim))

         # Create a Twist to satured velocites

        zero_twist = Twist()
        zero_twist.linear.x  = sat(u[0, 0],      MAX_LIN)
        zero_twist.linear.y  = sat(u[1, 0],      MAX_LIN)
        zero_twist.linear.z  = sat(twist_z[0, 0], MAX_LIN)

        zero_twist.angular.x = sat(u[2, 0],       MAX_ANG)
        zero_twist.angular.y = sat(u[3, 0],       MAX_ANG)
        zero_twist.angular.z = sat(twist_z[1, 0], MAX_ANG)

        # Publish zero command
        self.cmd_pub.publish(zero_twist)
        rospy.loginfo("Published cmd_vel")

if __name__ == '__main__':
    rospy.init_node('visual_servoing')
    ZeroCmdFromFeatures()
    rospy.spin()

#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from kinova_dq_nmpc.utils import add
from kinova_dq_nmpc.dq_quaternion_casadi import DualQuaternion
import casadi as ca
from kinova_dq_nmpc.forward_kinematics import forward_kinematics_casadi_link1, forward_kinematics_casadi_link2, forward_kinematics_casadi_link3, forward_kinematics_casadi_link4, forward_kinematics_casadi_link5, forward_kinematics_casadi_link6, forward_kinematics_casadi_link7, forward_kinematics_casadi_link8, forward_kinematics_casadi
from kinova_dq_nmpc.forward_kinematics import jacobian_casadi

from kinova_dq_nmpc.ode_acados import dualquat_trans_casadi, dualquat_quat_casadi, rotation_casadi, rotation_inverse_casadi, dual_velocity_casadi, dual_quat_casadi, velocities_from_twist_casadi
import numpy as np

from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# Creating Funtions based on Casadi
get_trans = dualquat_trans_casadi()
get_quat = dualquat_quat_casadi()
dual_twist = dual_velocity_casadi()
velocity_from_twist = velocities_from_twist_casadi()
forward_kinematics_link1 = forward_kinematics_casadi_link1()
forward_kinematics_link2 = forward_kinematics_casadi_link2()
forward_kinematics_link3 = forward_kinematics_casadi_link3()
forward_kinematics_link4 = forward_kinematics_casadi_link4()
forward_kinematics_link5 = forward_kinematics_casadi_link5()
forward_kinematics_link6 = forward_kinematics_casadi_link6()
forward_kinematics_link7 = forward_kinematics_casadi_link7()
forward_kinematics_link8 = forward_kinematics_casadi_link8()
forward_kinematics_f = forward_kinematics_casadi()
jacobian = jacobian_casadi()


def tranform_data(origin_frame, child_frame, dqd, tf_pub):
    # Compute quaternion and translations
    t_d = get_trans(dqd)
    q_d = get_quat(dqd)

    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = origin_frame
    t.child_frame_id = child_frame


    t.transform.translation.x = t_d[1]
    t.transform.translation.y = t_d[2]
    t.transform.translation.z = t_d[3]
    t.transform.rotation.x = q_d[1]
    t.transform.rotation.y = q_d[2]
    t.transform.rotation.z = q_d[3]
    t.transform.rotation.w = q_d[0]
    tf_pub.sendTransform(t)
    return None

def main():
    # Sample Time Defintion
    sample_time = 0.03
    t_f = 60000

    # Time defintion aux variable
    t = np.arange(0, t_f + sample_time, sample_time)
    delta_t = np.zeros((1, t.shape[0]), dtype=np.double)

    # Frequency of the simulation
    hz = int(1/(sample_time))
    loop_rate = rospy.Rate(hz)

    # Message Ros
    rospy.loginfo_once("DualQuaternion.....")

    # Initial States of the robot manipulator
    # Link1
    theta_1 = 7.849925267144897e-06
    theta_2 = 0.2602010330758926
    theta_3 = 3.140026891628009
    theta_4 = -2.27011089841832
    theta_5 = -5.963913216611161e-05
    theta_6 = 0.9597031693450511
    theta_7 = 1.5691806478271388


    link_1 = forward_kinematics_link1(theta_1)
    link_2 = forward_kinematics_link2(theta_2)
    link_3 = forward_kinematics_link3(theta_3)
    link_4 = forward_kinematics_link4(theta_4)
    link_5 = forward_kinematics_link5(theta_5)
    link_6 = forward_kinematics_link6(theta_6)
    link_7 = forward_kinematics_link7(theta_7)

    ## There is a bug since the function I created does not require input parameters
    link_8_aux = forward_kinematics_link8()
    link_8 = link_8_aux['o0']

    ## Function of the full pose base on angles
    ## This is the forward kinematicsbase on dual quaternions
    pose = forward_kinematics_f(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, theta_7)
    
    ## Aux variables    
    X_1 = np.zeros((8, t.shape[0] + 1), dtype=np.double)
    X_2 = np.zeros((8, t.shape[0] + 1), dtype=np.double)
    X_3 = np.zeros((8, t.shape[0] + 1), dtype=np.double)
    X_4 = np.zeros((8, t.shape[0] + 1), dtype=np.double)
    X_5 = np.zeros((8, t.shape[0] + 1), dtype=np.double)
    X_6 = np.zeros((8, t.shape[0] + 1), dtype=np.double)
    X_7 = np.zeros((8, t.shape[0] + 1), dtype=np.double)
    X_8 = np.zeros((8, t.shape[0] + 1), dtype=np.double)

    # Empty space for the end-effector and the joints 
    X_pose = np.zeros((8, t.shape[0] + 1), dtype=np.double)
    theta_vector = np.zeros((7, t.shape[0] + 1), dtype=np.double)

    # Set up initial conditions in the empty matrices
    X_1[:, 0] = np.array(link_1).reshape((8, ))
    X_2[:, 0] = np.array(link_2).reshape((8, ))
    X_3[:, 0] = np.array(link_3).reshape((8, ))
    X_4[:, 0] = np.array(link_4).reshape((8, ))
    X_5[:, 0] = np.array(link_5).reshape((8, ))
    X_6[:, 0] = np.array(link_6).reshape((8, ))
    X_7[:, 0] = np.array(link_7).reshape((8, ))
    X_8[:, 0] = np.array(link_8).reshape((8, ))

    ## Init Pose and joints
    X_pose[:, 0] = np.array(pose).reshape((8, ))
    theta_vector[0, :] = theta_1
    theta_vector[1, :] = theta_2
    theta_vector[2, :] = theta_3
    theta_vector[3, :] = theta_4
    theta_vector[4, :] = theta_5
    theta_vector[5, :] = theta_6
    theta_vector[6, :] = theta_7


    ## Message
    message_ros = "Kinova Forward Kinematics "

    manipulador_frames = TransformBroadcaster()

    # Simulation loop
    for k in range(0, t.shape[0]):
        tic = rospy.get_time()

        J = jacobian(theta_vector[:, k])

        # Update Matrices
        X_1[:, k+1] = np.array(link_1).reshape((8, ))
        X_2[:, k+1] = np.array(link_2).reshape((8, ))
        X_3[:, k+1] = np.array(link_3).reshape((8, ))
        X_4[:, k+1] = np.array(link_4).reshape((8, ))
        X_5[:, k+1] = np.array(link_5).reshape((8, ))
        X_6[:, k+1] = np.array(link_6).reshape((8, ))
        X_7[:, k+1] = np.array(link_7).reshape((8, ))
        X_8[:, k+1] = np.array(link_8).reshape((8, ))

        X_pose[:, k+1] = np.array(pose).reshape((8, ))

        # Send Data throught Ros
        tranform_data("world", "link_1", X_1[:, k+1], manipulador_frames)
        tranform_data("link_1", "link_2", X_2[:, k+1], manipulador_frames)
        tranform_data("link_2", "link_3", X_3[:, k+1], manipulador_frames)
        tranform_data("link_3", "link_4", X_4[:, k+1], manipulador_frames)
        tranform_data("link_4", "link_5", X_5[:, k+1], manipulador_frames)
        tranform_data("link_5", "link_6", X_6[:, k+1], manipulador_frames)
        tranform_data("link_6", "link_7", X_7[:, k+1], manipulador_frames)
        tranform_data("link_7", "end", X_8[:, k+1], manipulador_frames)

        ## Full transformation
        tranform_data("world", "pose", X_pose[:, k+1], manipulador_frames)

        # Sample time restriction
        loop_rate.sleep()

        # Save the appropriate sample time
        toc_solver = rospy.get_time() - tic

        # Delta time
        delta_t[:, k] = toc_solver

        # Show delta time 
        rospy.loginfo(message_ros + str(delta_t[:, k]))

    
if __name__ == '__main__':
    try:
        # Initialization Node
        rospy.init_node("dual_duaternions_kinova", disable_signals=True, anonymous=True)
        main()

    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass
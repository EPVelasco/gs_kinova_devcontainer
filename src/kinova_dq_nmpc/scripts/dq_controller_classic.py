#!/usr/bin/env python3
import rospy
from kinova_dq_nmpc.forward_kinematics import forward_kinematics_casadi_link1, forward_kinematics_casadi_link2, forward_kinematics_casadi_link3, forward_kinematics_casadi_link4, forward_kinematics_casadi_link5, forward_kinematics_casadi_link6, forward_kinematics_casadi_link7, forward_kinematics_casadi_link8, forward_kinematics_casadi
from kinova_dq_nmpc.forward_kinematics import jacobian_casadi
from kinova_dq_nmpc.ode_acados import dualquat_trans_casadi, dualquat_quat_casadi, dual_velocity_casadi, velocities_from_twist_casadi, error_dual_aux_casadi, ln_dual_aux_casadi, rotation_casadi
from kinova_dq_nmpc.forward_kinematics import dualquat_from_pose_casadi
import numpy as np
import casadi as ca

from sensor_msgs.msg import JointState

from kortex_driver.msg import Base_JointSpeeds, JointSpeed



## Dual quaternions Kinematic chain and Jacobian using casadi
## Extract position and orientation from dualquaternion
get_trans = dualquat_trans_casadi()
get_quat = dualquat_quat_casadi()

## Function that uses angular velocity body frame and linear inertial frame and map them to the body
dual_twist = dual_velocity_casadi()

## Function that maps twist body frame to angular velocity body frame and linear velocity inertial frame
velocity_from_twist = velocities_from_twist_casadi()

## Forward kinematic of each link (We do not need this, it is only for verification purposes)
forward_kinematics_link1 = forward_kinematics_casadi_link1()
forward_kinematics_link2 = forward_kinematics_casadi_link2()
forward_kinematics_link3 = forward_kinematics_casadi_link3()
forward_kinematics_link4 = forward_kinematics_casadi_link4()
forward_kinematics_link5 = forward_kinematics_casadi_link5()
forward_kinematics_link6 = forward_kinematics_casadi_link6()
forward_kinematics_link7 = forward_kinematics_casadi_link7()
forward_kinematics_link8 = forward_kinematics_casadi_link8()

## Full forward kinamtics of the system based on the joint angles compute the pose as dualquaternoin
forward_kinematics_f = forward_kinematics_casadi()

## Jacobian of the full forward Kinematics
jacobian = jacobian_casadi()

## Dual quaternion from the pose
dualquat_from_pose = dualquat_from_pose_casadi()

## Init Joint values
joint_angles = np.zeros(7)

## Dual quaternion error and Projectins to tangent space
error_dual = error_dual_aux_casadi()
ln_dual = ln_dual_aux_casadi()

# Rotation Funtion
rotation_i = rotation_casadi()

## Calback to get the angles of each joint
def joint_state_callback(msg):
    global joint_angles

    if len(msg.position) >= 7:
        joint_angles = np.array(msg.position[:7])
    else:
        rospy.logwarn("Received joint state with fewer than 7 joints.")

# Function to send velocities to the robot considerign each joint
def send_joint_velocity(joint_velocity_pub_, omega):
    # Joint velocities empty
    joint_speeds = Base_JointSpeeds()

    # Foor loop to assign the velocity to each joint
    for i in range(0, omega.shape[0]):
        speed = JointSpeed()
        speed.joint_identifier = i
        speed.value = omega[i]
        speed.duration = 0
        joint_speeds.joint_speeds.append(speed)
    ## Sending velocity
    joint_velocity_pub_.publish(joint_speeds)
    
def get_desired_frame():
    ##### ------------------  Here you shoudl include something to get the desired pose from rviz or Gazebo ##
    #### Remember this is pose regulatio, this is not pose tracking if you want pose tracking you shoudl provide the desired qual quaternion dot #########
    # Get system Positions and quaternions

    t = np.array([0.54, 0.0, 0.5])
    #r = np.array([0.9238795, 0, 0, 0.3826834])
    r = np.array([0, 0.7, 0.7, 0.0])

    # Initial Dualquaternion
    dual_d = dualquat_from_pose(r[0], r[1], r[2],  r[3], t[0], t[1], t[2])
    return dual_d

# Function that compute the  dual error and its projection
def dual_error(dual_d, dual):
    dual_error = error_dual(dual_d, dual)
    ln_e = ln_dual(dual_error)
    rot_e = ln_e[0:4, 0]
    trans_e = ln_e[4:8, 0]
    return trans_e, rot_e

## CLassic controller
def controller(dual_d, dual, q):
    # Create a diagonal matrix
    J = jacobian(q)
    I = np.eye(8, 8)
    aux = (J@J.T + 0.00000001*I)
    J_1 = J.T@ca.pinv(aux)
    # Compute error
    dual_error = error_dual(dual_d, dual)
    ln = ln_dual(dual_error)

    wb = ln[1:4, 0]
    vi = ln[5:8, 0]

    wi = rotation_i(dual[0:4], wb)

    w_i_twist = ca.vertcat(0.0, wi, 0.0, vi)

    # Error manifold
    unit = ca.DM.zeros(8, 1)
    unit[0, 0] = 1.0

    # Control Actions
    control = 2*J_1@(0.1*(unit - dual_error))
    #control = 2*J_1@(w_i_twist)
    control_reshape = np.array(control).reshape((7,))
    return control_reshape

def main():
    # Topics 
    # Jointstates
    joint_states_sub_ = rospy.Subscriber("/my_gen3/joint_states", 
                                        JointState, joint_state_callback)     

    # Publisher Joint velocites
    joint_velocity_pub_ = rospy.Publisher('/my_gen3/in/joint_velocity', Base_JointSpeeds, queue_size=10)


    # Sample Time Defintion
    sample_time = 0.05
    t_f = 30

    # Time defintion aux variable
    t = np.arange(0, t_f + sample_time, sample_time)
    delta_t = np.zeros((1, t.shape[0]), dtype=np.double)

    # Frequency of the simulation
    hz = int(1/(sample_time))
    loop_rate = rospy.Rate(hz)

    # Message Ros
    rospy.loginfo_once("DualQuaternionControl.....")

    ## Message
    message_ros = "Kinova Classic Controller "

    # Empty control actions
    omega_control = np.zeros((7, t.shape[0]), dtype = np.double)

    # Emppty current joint values
    q = np.zeros((7, t.shape[0] + 1), dtype = np.double)

    # Empty dual quaternion
    d = np.zeros((8, t.shape[0] + 1), dtype = np.double)
    
    ## Init System Short For Loop
    for k in range(0, 20):
        tic = rospy.get_time()
        # Sample time restriction
        loop_rate.sleep()
        # Save the appropriate sample time
        toc_solver = rospy.get_time() - tic
        # Show delta time 
        rospy.loginfo("Init System")
    
    # Set Initial states 
    q[:, 0] = joint_angles 
    d[:, 0] = np.array(forward_kinematics_f(q[0, 0], q[1, 0], q[2, 0], q[3, 0], q[4, 0], q[5, 0], q[6, 0])).reshape((8, ))

    # Set Desired state
    dual_d = get_desired_frame()

    # Simulation loop (We should remove this to call back function only)
    for k in range(0, t.shape[0]):
        tic = rospy.get_time()
        # Check for the shortest path
        #error_dual_no_filter = error_dual(dual_d, d[:, k])
        #if error_dual_no_filter[0] >= 0.0:
        #    dual_d = dual_d
        #else:
        #    dual_d = -dual_d
        # Compute the norm of the error
        translation_e, rotation_e = dual_error(dual_d, d[:, k]) 
        orientation_cost = rotation_e.T@rotation_e
        translation_cost = translation_e.T@translation_e
        # Control law
        omega_control[:, k] = controller(dual_d, d[:, k], q[:, k])

        # Send COntrol actions
        send_joint_velocity(joint_velocity_pub_, omega_control[:, k])

        # Sample time restriction
        loop_rate.sleep()

        # Save the appropriate sample time
        toc_solver = rospy.get_time() - tic

        # Delta time
        delta_t[:, k] = toc_solver

        # Show delta time 
        #rospy.loginfo("Control of the system")
        rospy.loginfo(message_ros + " " + str(orientation_cost) + " " + str(translation_cost))
        q[:, k + 1] = joint_angles 
        d[:, k + 1] = np.array(forward_kinematics_f(q[0, k+1], q[1, k+1], q[2, k+1], q[3, k+1], q[4, k+1], q[5, k+1], q[6, k+1])).reshape((8, ))
    
    ## Set Velocities to zero
    send_joint_velocity(joint_velocity_pub_, np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

if __name__ == '__main__':
    try:
        # Initialization Node
        rospy.init_node("dual_duaternions_kinova_control", disable_signals=True, anonymous=True)
        main()

    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass
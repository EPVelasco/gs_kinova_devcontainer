#!/usr/bin/env python3

import sys
import time
import rospy

from kortex_driver.srv import *
from kortex_driver.msg import *

from sensor_msgs.msg import JointState
import angles   # sudo apt-get install ros-noetic-angles

class ExampleJointVelocityControl:
    def __init__(self):
        try:
            rospy.init_node('example_joint_velocity_control')

            self.HOME_ACTION_IDENTIFIER = 2
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")
            self.print_rate_hz = rospy.get_param('~print_rate_hz', 5.0)  # imprime cada 0.2s por defecto
            self._last_print = rospy.Time(0)

            rospy.loginfo("Using robot_name %s", self.robot_name)

            # Subscripción a /joint_states
            self.joint_states_sub = rospy.Subscriber(
                '/' + self.robot_name + '/joint_states',
                JointState,
                self.cb_joint_states,
                queue_size=10
            )

            # el resto de inicialización original...
            self.action_topic_sub = rospy.Subscriber(
                "/" + self.robot_name + "/action_topic",
                ActionNotification,
                self.cb_action_topic
            )
            self.last_action_notif_type = None

            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            self.joint_velocity_pub = rospy.Publisher(
                '/' + self.robot_name + '/in/joint_velocity',
                Base_JointSpeeds,
                queue_size=10
            )

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(
                activate_publishing_of_action_notification_full_name, OnNotificationActionTopic
            )

        except Exception as e:
            rospy.logerr("Init error: %s", e)
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    # ✅ Nuevo: imprimir todos los joints con posición raw, envuelta y velocidad
    def cb_joint_states(self, msg: JointState):
        if self.print_rate_hz > 0:
            period = rospy.Duration(1.0 / self.print_rate_hz)
            if rospy.Time.now() - self._last_print < period:
                return
            self._last_print = rospy.Time.now()

        lines = []
        for i, pos_raw in enumerate(msg.position):
            name = msg.name[i] if i < len(msg.name) else f"joint_{i+1}"
            vel = msg.velocity[i] if i < len(msg.velocity) else float('nan')
            pos_wrapped = angles.normalize_angle(pos_raw)
            lines.append(f"{name}: pos_raw={pos_raw:.4f}, pos_wrapped={pos_wrapped:.4f}, vel={vel:.4f}")
        rospy.loginfo("\n" + "\n".join(lines))

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if self.last_action_notif_type == ActionEvent.ACTION_END:
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif self.last_action_notif_type == ActionEvent.ACTION_ABORT:
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        self.last_action_notif_type = None
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        else:
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()

    def example_subscribe_to_a_robot_notification(self):
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")
            rospy.sleep(1.0)
            return True

    def send_joint_speeds(self, speeds):
        joint_speeds = Base_JointSpeeds()
        for i in range(len(speeds)):
            speed = JointSpeed()
            speed.joint_identifier = i
            speed.value = speeds[i]
            speed.duration = 0
            joint_speeds.joint_speeds.append(speed)
        self.joint_velocity_pub.publish(joint_speeds)

    def main(self):
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/joint_velocity_control_python")
        except:
            pass

        if success:
            success &= self.example_clear_faults()
            success &= self.example_home_the_robot()
            success &= self.example_subscribe_to_a_robot_notification()

            if success:
                rospy.loginfo("Starting joint velocity control...")
                rate = rospy.Rate(10)  # 10 Hz
                while not rospy.is_shutdown():
                    self.send_joint_speeds([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                    rate.sleep()

        rospy.set_param("/kortex_examples_test_results/joint_velocity_control_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")

if __name__ == "__main__":
    ex = ExampleJointVelocityControl()
    ex.main()

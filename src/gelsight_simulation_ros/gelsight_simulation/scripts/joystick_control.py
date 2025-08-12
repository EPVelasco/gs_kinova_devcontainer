#!/usr/bin/env python

# First, you need to launch the node rosrun joy joy_node
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickToCmdVel:
    def __init__(self):
        rospy.init_node("joystick_to_cmd_vel")

        # Joystick axis configuration
        self.axis_linear_x = 1    # left stick vertical (forward/backward)
        self.axis_linear_y = 0    # left stick horizontal (left/right)
        self.axis_linear_z = 4    # right trigger (up/down)

        self.axis_angular_x = 1   # right stick horizontal
        self.axis_angular_y = 0   # right stick vertical
        self.axis_angular_z = 3   # alternative axis for rotation (e.g., another button or stick)

        self.button_enable_rotation_xy = 5  # Button 5 to enable rotation in X and Y

        # Scales
        self.max_v = 0.01   # m/s
        self.max_w = 0.5    # rad/s

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/joy", Joy, self.joy_callback)

    def joy_callback(self, msg):
        twist = Twist()

        # Linear movement
        twist.linear.z = self.max_v * msg.axes[self.axis_linear_z] 
        twist.angular.z = self.max_w * msg.axes[self.axis_angular_z]

        # Angular movement
        if msg.buttons[self.button_enable_rotation_xy]:
            twist.angular.x = self.max_w * msg.axes[self.axis_linear_x]
            twist.angular.y = self.max_w * msg.axes[self.axis_linear_y]
        else:            
            twist.linear.x = self.max_v * msg.axes[self.axis_linear_x]
            twist.linear.y = self.max_v * msg.axes[self.axis_linear_y]

        self.pub.publish(twist)

if __name__ == "__main__":
    try:
        JoystickToCmdVel()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

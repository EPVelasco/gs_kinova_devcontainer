#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Instructions
msg = """
Control the robot with the keyboard:
---------------------------
w: forward
s: backward
a: left
d: right
x: stop
q: quit
"""

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('keyboard_cmd_vel')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    
    linear_speed = 0.01  # m/s
    twist = Twist()

    print(msg)

    try:
        while not rospy.is_shutdown():
            key = get_key()
            if key == 'w':
                twist.linear.x = linear_speed
                twist.linear.y = 0.0
            elif key == 's':
                twist.linear.x = -linear_speed
                twist.linear.y = 0.0
            elif key == 'a':
                twist.linear.x = 0.0
                #twist.angular.z = 0.05
                twist.linear.y = 0.02
            elif key == 'd':
                twist.linear.x = 0.0
                #twist.angular.z = -0.05
                twist.linear.y = -0.02
            elif key == 'x':  # stop
                twist = Twist()
            elif key == 'q':  # quit
                break
            else:
                continue

            pub.publish(twist)
            rate.sleep()

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

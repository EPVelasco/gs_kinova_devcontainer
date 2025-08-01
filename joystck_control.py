#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoystickToCmdVel:
    def __init__(self):
        rospy.init_node("joystick_to_cmd_vel")

        # Configuración de ejes del joystick
        self.axis_linear_x = 1    # stick izquierdo vertical (adelante/atrás)
        self.axis_linear_y = 0    # stick izquierdo horizontal (izq/der)
        self.axis_linear_z = 4    # gatillo derecho (subir/bajar)

        self.axis_angular_x = 1   # stick derecho horizontal
        self.axis_angular_y = 0   # stick derecho vertical
        self.axis_angular_z = 3   # eje alternativo para giro (por ejemplo, otro botón o stick)

        self.button_enable_rotation_xy = 5  # Botón 5 para activar rotación en X e Y

        # Escalas
        self.max_v = 0.01   # m/s
        self.max_w = 0.5   # rad/s

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/joy", Joy, self.joy_callback)

    def joy_callback(self, msg):
        twist = Twist()

        # Movimiento lineal
       
        twist.linear.z = self.max_v * msg.axes[self.axis_linear_z] * 0.1
        twist.angular.z = self.max_w * msg.axes[self.axis_angular_z]

        # Movimiento angular
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

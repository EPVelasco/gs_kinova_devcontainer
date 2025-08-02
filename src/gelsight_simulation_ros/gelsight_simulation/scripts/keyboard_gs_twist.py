#!/usr/bin/env python
import rospy
import pygame
from geometry_msgs.msg import Twist

class KeyboardToCmdVel:
    def __init__(self):
        rospy.init_node("keyboard_to_cmd_vel")

        # Initialize pygame window
        pygame.init()
        self.screen = pygame.display.set_mode((400, 150))
        pygame.display.set_caption("Keyboard Teleop for /cmd_vel")

        # Font for on-screen instructions
        self.font = pygame.font.SysFont("Arial", 18)
        self.text_lines = [
            self.font.render("        Click this window to enable keyboard input        ", True, (255, 255, 255)),
            self.font.render("        Then use the keys to control the robot    ", True, (255, 255, 255))
        ]

        # Get rectangles for centering
        self.text_rects = []
        for i, line in enumerate(self.text_lines):
            rect = line.get_rect(center=(200, 50 + i * 30))
            self.text_rects.append(rect)

        self.clock = pygame.time.Clock()

        # Velocity scales
        self.max_v = 0.01  # linear velocity in m/s
        self.max_w = 0.5   # angular velocity in rad/s

        # ROS publisher
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def run(self):
        while not rospy.is_shutdown():
            twist = Twist()

            # Fill background and render instructions
            self.screen.fill((0, 0, 0))
            for line, rect in zip(self.text_lines, self.text_rects):
                self.screen.blit(line, rect)
            pygame.display.flip()

            # Get pressed keys
            keys = pygame.key.get_pressed()

            # Linear motion
            if keys[pygame.K_w]: twist.linear.x = self.max_v
            if keys[pygame.K_s]: twist.linear.x = -self.max_v
            if keys[pygame.K_a]: twist.linear.y = self.max_v
            if keys[pygame.K_d]: twist.linear.y = -self.max_v
            if keys[pygame.K_q]: twist.linear.z = self.max_v
            if keys[pygame.K_e]: twist.linear.z = -self.max_v

            # Angular motion
            if keys[pygame.K_LEFT]: twist.angular.z = self.max_w
            if keys[pygame.K_RIGHT]: twist.angular.z = -self.max_w
            if keys[pygame.K_UP]: twist.angular.x = self.max_w
            if keys[pygame.K_DOWN]: twist.angular.x = -self.max_w
            if keys[pygame.K_z]: twist.angular.y = self.max_w
            if keys[pygame.K_c]: twist.angular.y = -self.max_w

            # Publish Twist message
            self.pub.publish(twist)

            # Process pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    rospy.signal_shutdown("Window closed")

            # Maintain loop at 20 Hz
            self.clock.tick(20)

if __name__ == "__main__":
    try:
        teleop = KeyboardToCmdVel()
        teleop.run()
    except rospy.ROSInterruptException:
        pass

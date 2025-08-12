#!/usr/bin/env python
import rospy
import pygame
from geometry_msgs.msg import Twist

class KeyboardToCmdVel:
    def __init__(self):
        rospy.init_node("keyboard_to_cmd_vel")

        # Velocity scales
        self.max_v = 0.01  # linear velocity in m/s
        self.max_w = 0.5   # angular velocity in rad/s

        # Initialize pygame window (taller to fit instructions)
        pygame.init()
        self.screen = pygame.display.set_mode((600, 420))
        pygame.display.set_caption("Keyboard Teleop for /cmd_vel")

        # Monospaced font for alignment
        self.font = pygame.font.SysFont("Courier New", 16)

        # Build instruction text using current scales
        v = self.max_v
        w = self.max_w
        self.text_lines = [
            "CLICK THIS WINDOW TO ENABLE KEYBOARD INPUT",
            "",
            "LINEAR MOTION (m/s):",
            f"  W: +X ({v:+.3f})     S: -X ({-v:+.3f})",
            f"  A: +Y ({v:+.3f})     D: -Y ({-v:+.3f})",
            f"  Q: +Z ({v:+.3f})     E: -Z ({-v:+.3f})",
            "",
            "ANGULAR MOTION (rad/s):",
            f"  LEFT:  +Z yaw ({w:+.3f})   RIGHT: -Z yaw ({-w:+.3f})",
            f"  UP:    +X roll({w:+.3f})   DOWN:  -X roll({-w:+.3f})",
            f"  Z:     +Y pitch({w:+.3f})  C:     -Y pitch({-w:+.3f})",
            "",
            "NOTES:",
            "  • Linear axes: X=forward/back, Y=left/right, Z=up/down",
            "  • Angular axes: roll(X), pitch(Y), yaw(Z)",
            "  • Messages are published to /cmd_vel at 20 Hz",
        ]

        # Pre-render text surfaces and center horizontally
        self.text_surfaces = [self.font.render(line, True, (255, 255, 255)) for line in self.text_lines]
        self.text_rects = []
        for i, surf in enumerate(self.text_surfaces):
            rect = surf.get_rect(center=(260, 30 + i * 24))
            self.text_rects.append(rect)

        self.clock = pygame.time.Clock()

        # ROS publisher
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def run(self):
        while not rospy.is_shutdown():
            twist = Twist()

            # Draw background and instructions
            self.screen.fill((0, 0, 0))
            for surf, rect in zip(self.text_surfaces, self.text_rects):
                self.screen.blit(surf, rect)
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
            if keys[pygame.K_LEFT]:  twist.angular.z = self.max_w
            if keys[pygame.K_RIGHT]: twist.angular.z = -self.max_w
            if keys[pygame.K_UP]:    twist.angular.x = self.max_w
            if keys[pygame.K_DOWN]:  twist.angular.x = -self.max_w
            if keys[pygame.K_z]:     twist.angular.y = self.max_w
            if keys[pygame.K_c]:     twist.angular.y = -self.max_w

            # Publish Twist message
            self.pub.publish(twist)

            # Handle window events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    rospy.signal_shutdown("Window closed")

            # 20 Hz loop
            self.clock.tick(20)

if __name__ == "__main__":
    try:
        teleop = KeyboardToCmdVel()
        teleop.run()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
import rospy
import pygame
from geometry_msgs.msg import Twist

class KeyboardToCmdVel:
    def __init__(self):
        rospy.init_node("keyboard_to_cmd_vel")

        # --- Velocity scales and increments ---
        self.max_v = 0.01      # linear velocity (m/s)
        self.max_w = 0.5       # angular velocity (rad/s)
        self.step_v = 0.0001   # single-tap increment for linear (m/s)
        self.step_w = 0.005    # single-tap increment for angular (rad/s)

        # Debounced hold behavior for +/- (press -> 1 step, then repeat after delay)
        self.hold_delay_s = 0.30     # time before repeating starts
        self.repeat_rate_hz = 10.0   # how many repeats per second after delay

        self.v_min, self.v_max = 0.0, 5.0    # linear limits (m/s)
        self.w_min, self.w_max = 0.0, 10.0   # angular limits (rad/s)

        # Which scale to adjust with +/-: 'linear' or 'angular'
        self.adjust_target = 'linear'

        # Initialize pygame
        pygame.init()
        self.screen = pygame.display.set_mode((800, 540))
        pygame.display.set_caption("Keyboard Teleop for /cmd_vel")

        # Monospaced fonts
        self.font = pygame.font.SysFont("Courier New", 18)
        self.small = pygame.font.SysFont("Courier New", 16)

        self.clock = pygame.time.Clock()

        # ROS publisher
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Last Twist for display
        self.last_twist = Twist()

        # Internal state for debounced +/- hold
        self.plus_held = False
        self.minus_held = False
        self.plus_next_time = 0.0
        self.minus_next_time = 0.0

        # --- NEW: Publishing enable/disable flag ---
        self.publish_enabled = True

    def now(self):
        """Current time in seconds from pygame (monotonic)."""
        return pygame.time.get_ticks() / 1000.0

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def _apply_step(self, sign):
        """Apply one step to the selected target with given sign (+1 or -1)."""
        if self.adjust_target == 'linear':
            self.max_v = self.clamp(self.max_v + sign * self.step_v, self.v_min, self.v_max)
        else:
            self.max_w = self.clamp(self.max_w + sign * self.step_w, self.w_min, self.w_max)

    def handle_key_events(self, event):
        """Process events: quit, ESC, TAB, +/- and P press & release."""
        if event.type == pygame.QUIT:
            rospy.signal_shutdown("Window closed")
            return
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                rospy.signal_shutdown("ESC pressed")
                return
            if event.key == pygame.K_TAB:
                self.adjust_target = 'angular' if self.adjust_target == 'linear' else 'linear'
                return

            # --- NEW: toggle publishing with P ---
            if event.key == pygame.K_p:
                self.publish_enabled = not self.publish_enabled
                return

            # Detect + press (main row or keypad)
            if event.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                # Single-tap step immediately
                self._apply_step(+1)
                # Arm hold repeat
                self.plus_held = True
                self.plus_next_time = self.now() + self.hold_delay_s
                return

            # Detect - press (main row or keypad)
            if event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                self._apply_step(-1)
                self.minus_held = True
                self.minus_next_time = self.now() + self.hold_delay_s
                return

        if event.type == pygame.KEYUP:
            # Release cancels repeat
            if event.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                self.plus_held = False
            if event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                self.minus_held = False

    def handle_hold_repeat(self):
        """Repeat steps for +/- after the initial delay, at a fixed repeat rate."""
        t = self.now()
        step_period = 1.0 / self.repeat_rate_hz

        if self.plus_held and t >= self.plus_next_time:
            self._apply_step(+1)
            self.plus_next_time += step_period  # schedule next repeat

        if self.minus_held and t >= self.minus_next_time:
            self._apply_step(-1)
            self.minus_next_time += step_period

    def build_text_lines(self, twist):
        # Display linear scale in mm/s (screen only), angular in rad/s
        v_mm = self.max_v * 1000.0
        w = self.max_w
        tgt = "LINEAR (m/s)" if self.adjust_target == 'linear' else "ANGULAR (rad/s)"
        status = "ENABLED" if self.publish_enabled else "DISABLED"
        status_hint = "(press P to toggle)"

        # Convert published twist (m/s) to mm/s for display lines only
        linx_mm = twist.linear.x * 1000.0
        liny_mm = twist.linear.y * 1000.0
        linz_mm = twist.linear.z * 1000.0

        hold_info = f"(hold delay {self.hold_delay_s:.2f}s, repeat {self.repeat_rate_hz:.1f} Hz)"

        lines = [
            "CLICK THIS WINDOW TO CAPTURE KEYBOARD INPUT | ESC: exit",
            f"PUBLISHING: {status} {status_hint}",
            "",
            f"CURRENT SCALES ->  max_v: {v_mm:.3f} mm/s   |   max_w: {w:.3f} rad/s",
            f"ADJUST WITH +/- ->  Current target: {tgt}   (TAB to toggle) {hold_info}",
            "",
            "MOTION CONTROLS:",
            f"  LINEAR (mm/s):   W:+X({v_mm:+.3f})   S:-X({-v_mm:+.3f})",
            f"                   A:+Y({v_mm:+.3f})   D:-Y({-v_mm:+.3f})",
            f"                   Q:+Z({v_mm:+.3f})   E:-Z({-v_mm:+.3f})",
            f"  ANGULAR (rad/s): LEFT:+yaw({w:+.3f}) RIGHT:-yaw({-w:+.3f})",
            f"                   UP:+roll({w:+.3f})  DOWN:-roll({-w:+.3f})",
            f"                   Z:+pitch({w:+.3f})  C:-pitch({-w:+.3f})",
            "",
            "PUBLISHED VELOCITIES (/cmd_vel at 20 Hz):",
            f"  linear.x: {linx_mm:+.3f}  linear.y: {liny_mm:+.3f}  linear.z: {linz_mm:+.3f}   [mm/s]",
            f"  angular.x: {twist.angular.x:+.3f}  angular.y: {twist.angular.y:+.3f}  angular.z: {twist.angular.z:+.3f}   [rad/s]",
        ]
        return lines

    def draw_text_block(self, lines):
        """Draws the instruction and status text on screen."""
        self.screen.fill((0, 0, 0))
        # --- Red banner if publishing disabled ---
        if not self.publish_enabled:
            pygame.draw.rect(self.screen, (120, 0, 0), pygame.Rect(0, 0, 800, 24))
            banner = self.small.render("PUBLISHING DISABLED (press P to enable)", True, (255, 255, 255))
            self.screen.blit(banner, (20, 4))
            y = 36
        else:
            y = 24

        for line in lines:
            surf = self.font.render(line, True, (255, 255, 255))
            rect = surf.get_rect(topleft=(20, y))
            self.screen.blit(surf, rect)
            y += 26

        footer = self.small.render(
            "Tip: Hold motion keys. +/- uses debounced repeat (delay then steady rate).",
            True, (180, 180, 180)
        )
        self.screen.blit(footer, (20, y + 6))
        pygame.display.flip()

    def run(self):
        while not rospy.is_shutdown():
            # 1) Handle events (quit, ESC, TAB, +/- press & release)
            for event in pygame.event.get():
                self.handle_key_events(event)

            # 2) Apply debounced +/- repeats (after delay, at fixed rate)
            self.handle_hold_repeat()

            # 3) Build Twist from pressed motion keys
            keys = pygame.key.get_pressed()
            twist = Twist()

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

            # 4) Publish only if enabled
            if self.publish_enabled:
                self.pub.publish(twist)
            self.last_twist = twist

            # 5) Draw UI
            lines = self.build_text_lines(self.last_twist)
            self.draw_text_block(lines)

            # 6) 20 Hz loop
            self.clock.tick(20)

if __name__ == "__main__":
    try:
        teleop = KeyboardToCmdVel()
        teleop.run()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
import rospy
import pygame
from geometry_msgs.msg import TwistStamped

class KeyboardToTwistStamped:
    def __init__(self):
        rospy.init_node("keyboard_to_twiststamped")

        # --- ROS params ---
        self.topic       = rospy.get_param("~topic", "/dq_control/ee_twist")
        self.rate_hz     = float(rospy.get_param("~rate_hz", 20.0))
        # Convención de marcos (solo informativa en frame_id; el consumidor debe saberlo)
        self.linear_frame  = rospy.get_param("~linear_frame",  "world")
        self.angular_frame = rospy.get_param("~angular_frame", "tool")

        # --- Velocity scales and increments ---
        self.max_v  = float(rospy.get_param("~max_v",  0.01))    # m/s
        self.max_w  = float(rospy.get_param("~max_w",  0.5))     # rad/s
        self.step_v = float(rospy.get_param("~step_v", 0.0001))  # m/s per tap
        self.step_w = float(rospy.get_param("~step_w", 0.005))   # rad/s per tap

        # Debounced hold behavior for +/- (press -> 1 step, then repeat after delay)
        self.hold_delay_s  = float(rospy.get_param("~hold_delay_s", 0.30))
        self.repeat_rate_hz = float(rospy.get_param("~repeat_rate_hz", 10.0))

        self.v_min, self.v_max = 0.0, 5.0     # linear limits (m/s)
        self.w_min, self.w_max = 0.0, 10.0    # angular limits (rad/s)

        # Which scale to adjust with +/-: 'linear' or 'angular'
        self.adjust_target = 'linear'

        # Initialize pygame
        pygame.init()
        self.screen = pygame.display.set_mode((820, 560))
        pygame.display.set_caption("Keyboard Teleop for TwistStamped")

        # Monospaced fonts
        self.font  = pygame.font.SysFont("Courier New", 18)
        self.small = pygame.font.SysFont("Courier New", 16)
        self.clock = pygame.time.Clock()

        # ROS publisher
        self.pub = rospy.Publisher(self.topic, TwistStamped, queue_size=10)

        # Last TwistStamped for display
        self.last_msg = TwistStamped()
        self.last_msg.header.frame_id = self._frame_id_text()

        # Internal state for debounced +/- hold
        self.plus_held = False
        self.minus_held = False
        self.plus_next_time = 0.0
        self.minus_next_time = 0.0

    # -------- Helpers --------
    def _frame_id_text(self):
        return "lin:{},ang:{}".format(self.linear_frame, self.angular_frame)

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

    # -------- Event handling --------
    def handle_key_events(self, event):
        """Process events: quit, ESC, TAB, and debounced +/- press & release."""
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

            # Detect + press (main row or keypad)
            if event.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                self._apply_step(+1)  # immediate single tap
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
            self.plus_next_time += step_period

        if self.minus_held and t >= self.minus_next_time:
            self._apply_step(-1)
            self.minus_next_time += step_period

    # -------- UI --------
    def build_text_lines(self, msg):
        # Display linear scale in mm/s (screen only), angular in rad/s
        v_mm = self.max_v * 1000.0
        w = self.max_w
        tgt = "LINEAR (m/s)" if self.adjust_target == 'linear' else "ANGULAR (rad/s)"

        # Convert published twist (m/s) to mm/s for display lines only
        linx_mm = msg.twist.linear.x * 1000.0
        liny_mm = msg.twist.linear.y * 1000.0
        linz_mm = msg.twist.linear.z * 1000.0

        hold_info = f"(hold delay {self.hold_delay_s:.2f}s, repeat {self.repeat_rate_hz:.1f} Hz)"
        lines = [
            "CLICK THIS WINDOW TO CAPTURE KEYBOARD INPUT | ESC: exit",
            "",
            f"Publishing: {self.topic}  @ {self.rate_hz:.1f} Hz   [TwistStamped]",
            f"frame_id: {self._frame_id_text()}",
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
            "PUBLISHED VELOCITIES:",
            f"  linear.x: {linx_mm:+.3f}  linear.y: {liny_mm:+.3f}  linear.z: {linz_mm:+.3f}   [mm/s]",
            f"  angular.x: {msg.twist.angular.x:+.3f}  angular.y: {msg.twist.angular.y:+.3f}  angular.z: {msg.twist.angular.z:+.3f}   [rad/s]",
        ]
        return lines

    def draw_text_block(self, lines):
        """Draws the instruction and status text on screen."""
        self.screen.fill((0, 0, 0))
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

    # -------- Main loop --------
    def run(self):
        rate = rospy.Rate(self.rate_hz)
        try:
            while not rospy.is_shutdown():
                # 1) Handle events (quit, ESC, TAB, +/- press & release)
                for event in pygame.event.get():
                    self.handle_key_events(event)

                # 2) Apply debounced +/- repeats (after delay, at fixed rate)
                self.handle_hold_repeat()

                # 3) Build TwistStamped from pressed motion keys
                msg = TwistStamped()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = self._frame_id_text()

                keys = pygame.key.get_pressed()

                # Linear motion
                if keys[pygame.K_w]: msg.twist.linear.x = self.max_v
                if keys[pygame.K_s]: msg.twist.linear.x = -self.max_v
                if keys[pygame.K_a]: msg.twist.linear.y = self.max_v
                if keys[pygame.K_d]: msg.twist.linear.y = -self.max_v
                if keys[pygame.K_q]: msg.twist.linear.z = self.max_v
                if keys[pygame.K_e]: msg.twist.linear.z = -self.max_v

                # Angular motion
                if keys[pygame.K_LEFT]:  msg.twist.angular.z = self.max_w
                if keys[pygame.K_RIGHT]: msg.twist.angular.z = -self.max_w
                if keys[pygame.K_UP]:    msg.twist.angular.x = self.max_w
                if keys[pygame.K_DOWN]:  msg.twist.angular.x = -self.max_w
                if keys[pygame.K_z]:     msg.twist.angular.y = self.max_w
                if keys[pygame.K_c]:     msg.twist.angular.y = -self.max_w

                # 4) Publish and draw UI
                self.pub.publish(msg)
                self.last_msg = msg

                lines = self.build_text_lines(self.last_msg)
                self.draw_text_block(lines)

                # 5) loop rate
                self.clock.tick(self.rate_hz)
                rate.sleep()
        finally:
            pygame.quit()

if __name__ == "__main__":
    try:
        teleop = KeyboardToTwistStamped()
        teleop.run()
    except rospy.ROSInterruptException:
        pass

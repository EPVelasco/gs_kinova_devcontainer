#!/usr/bin/env python3
import math
import rospy
import pygame

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from kortex_driver.msg import Base_JointSpeeds, JointSpeed


class KeyboardToJointTeleop(object):
    def __init__(self):
        # ---------------- ROS params ----------------
        self.robot_name = rospy.get_param("~robot_name", "my_gen3")
        self.topic_joint_vel = rospy.get_param(
            "~topic_joint_vel", "/{}/in/joint_velocity".format(self.robot_name)
        )
        self.topic_joint_states = rospy.get_param(
            "~topic_joint_states", "/{}/joint_states".format(self.robot_name)
        )
        self.topic_gripper = rospy.get_param("~topic_gripper", "/gripper_cmd")
        self.rate_hz = float(rospy.get_param("~rate_hz", 40.0))

        # velocidad manual por teclado [rad/s]
        self.current_joint_speed = float(rospy.get_param("~current_joint_speed", 0.20))
        self.step_joint_speed = float(rospy.get_param("~step_joint_speed", 0.02))
        self.min_joint_speed = float(rospy.get_param("~min_joint_speed", 0.02))
        self.max_joint_speed = float(rospy.get_param("~max_joint_speed", 0.50))

        # control automático hacia pose objetivo
        self.kp_auto = float(rospy.get_param("~kp_auto", 0.9))
        self.auto_max_speed = float(rospy.get_param("~auto_max_speed", 0.25))
        self.goal_tolerance_deg = float(rospy.get_param("~goal_tolerance_deg", 0.1))
        self.goal_tolerance = math.radians(self.goal_tolerance_deg)

        # pinza
        self.gripper_step = float(rospy.get_param("~gripper_step", 0.05))
        self.gripper = 0.0

        # poses predefinidas en grados        
               
        default_home_deg = [0.0, 15.0, 180.0, 230.0, 0.0, 55.0, 90.0]
        default_initial_deg = [-38.854, 62.034, 197.570, -84.351, 190.869, 6.709, 27.438]

        self.home_pose_deg = rospy.get_param("~home_pose_deg", default_home_deg)
        self.initial_pose_deg = rospy.get_param("~initial_pose_deg", default_initial_deg)

        if len(self.home_pose_deg) != 7 or len(self.initial_pose_deg) != 7:
            raise ValueError("Las poses ~home_pose_deg y ~initial_pose_deg deben tener 7 valores.")

        self.home_pose = [math.radians(v) for v in self.home_pose_deg]
        self.initial_pose = [math.radians(v) for v in self.initial_pose_deg]

        # joint limits suaves solo para error angular [-pi, pi]
        self.joint_names = [
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"
        ]
        self.q = [0.0] * 7
        self.has_joint_state = False

        # modo
        self.mode = "manual"   # manual | auto_home | auto_initial
        self.target_pose = None
        self.status_text = "Manual"

        # -------- pygame --------
        pygame.init()
        self.screen = pygame.display.set_mode((1100, 820))
        pygame.display.set_caption("Kinova Joint Teleop (ROS1)")
        self.font = pygame.font.SysFont("Courier New", 18)
        self.small = pygame.font.SysFont("Courier New", 16)
        self.clock = pygame.time.Clock()

        # -------- ROS --------
        self.pub_joint_vel = rospy.Publisher(self.topic_joint_vel, Base_JointSpeeds, queue_size=10)
        self.pub_gripper = rospy.Publisher(self.topic_gripper, Float64, queue_size=10)
        rospy.Subscriber(self.topic_joint_states, JointState, self.joint_state_cb)

        rospy.loginfo("Keyboard joint teleop iniciado")
        rospy.loginfo("Joint velocity topic: %s", self.topic_joint_vel)
        rospy.loginfo("Joint states topic:   %s", self.topic_joint_states)

    # -------------------------------------------------
    @staticmethod
    def clamp(x, lo, hi):
        return max(lo, min(hi, x))

    @staticmethod
    def wrap_to_pi(a):
        return math.atan2(math.sin(a), math.cos(a))

    def joint_state_cb(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))
        q_new = [0.0] * 7
        for i, name in enumerate(self.joint_names):
            if name not in name_to_pos:
                return
            q_new[i] = name_to_pos[name]
        self.q = q_new
        self.has_joint_state = True

    def build_joint_speed_msg(self, dq):
        msg = Base_JointSpeeds()
        for i in range(7):
            js = JointSpeed()
            js.joint_identifier = i
            js.value = float(dq[i])
            js.duration = 0
            msg.joint_speeds.append(js)
        return msg

    def publish_joint_speeds(self, dq):
        self.pub_joint_vel.publish(self.build_joint_speed_msg(dq))

    def stop_all_joints(self):
        self.publish_joint_speeds([0.0] * 7)

    def start_auto_motion(self, target_pose, label):
        if not self.has_joint_state:
            self.status_text = "Sin joint_states: no se puede lanzar {}".format(label)
            return
        self.target_pose = list(target_pose)
        self.mode = label
        self.status_text = "Moviendo a {}".format(label)

    def compute_auto_dq(self):
        if self.target_pose is None or not self.has_joint_state:
            return [0.0] * 7, True

        dq = [0.0] * 7
        done = True
        for i in range(7):
            err = self.wrap_to_pi(self.target_pose[i] - self.q[i])
            if abs(err) > self.goal_tolerance:
                done = False
            v = self.kp_auto * err
            v = self.clamp(v, -self.auto_max_speed, self.auto_max_speed)
            dq[i] = v

        return dq, done

    def handle_keydown(self, event):
        if event.key == pygame.K_ESCAPE:
            rospy.signal_shutdown("ESC pulsado")
            return

        if event.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
            self.current_joint_speed = self.clamp(
                self.current_joint_speed + self.step_joint_speed,
                self.min_joint_speed,
                self.max_joint_speed,
            )
            return

        if event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
            self.current_joint_speed = self.clamp(
                self.current_joint_speed - self.step_joint_speed,
                self.min_joint_speed,
                self.max_joint_speed,
            )
            return

        if event.key == pygame.K_h:
            self.start_auto_motion(self.home_pose, "HOME")
            return

        if event.key == pygame.K_i:
            self.start_auto_motion(self.initial_pose, "INITIAL")
            return

        if event.key == pygame.K_SPACE:
            self.mode = "manual"
            self.target_pose = None
            self.status_text = "Manual"
            self.stop_all_joints()
            return

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rospy.signal_shutdown("Ventana cerrada")
                return
            if event.type == pygame.KEYDOWN:
                self.handle_keydown(event)

    def compute_manual_dq(self, keys):
        dq = [0.0] * 7
        s = self.current_joint_speed

        # Joint 1
        if keys[pygame.K_1]:
            dq[0] = +s
        elif keys[pygame.K_q]:
            dq[0] = -s

        # Joint 2
        if keys[pygame.K_2]:
            dq[1] = +s
        elif keys[pygame.K_w]:
            dq[1] = -s

        # Joint 3
        if keys[pygame.K_3]:
            dq[2] = +s
        elif keys[pygame.K_e]:
            dq[2] = -s

        # Joint 4
        if keys[pygame.K_4]:
            dq[3] = +s
        elif keys[pygame.K_r]:
            dq[3] = -s

        # Joint 5
        if keys[pygame.K_5]:
            dq[4] = +s
        elif keys[pygame.K_t]:
            dq[4] = -s

        # Joint 6
        if keys[pygame.K_6]:
            dq[5] = +s
        elif keys[pygame.K_y]:
            dq[5] = -s

        # Joint 7
        if keys[pygame.K_7]:
            dq[6] = +s
        elif keys[pygame.K_u]:
            dq[6] = -s

        # pinza opcional
        if keys[pygame.K_g]:
            self.gripper = self.clamp(self.gripper + self.gripper_step, 0.0, 1.0)
        if keys[pygame.K_f]:
            self.gripper = self.clamp(self.gripper - self.gripper_step, 0.0, 1.0)

        return dq

    def joint_lines(self):
        lines = []
        for i in range(7):
            q_deg = math.degrees(self.q[i]) if self.has_joint_state else 0.0
            h_deg = self.home_pose_deg[i]
            ini_deg = self.initial_pose_deg[i]
            lines.append(
                "J{}  actual={:+8.2f} deg   home={:+8.2f}   initial={:+8.2f}".format(
                    i + 1, q_deg, h_deg, ini_deg
                )
            )
        return lines

    def build_text_lines(self, dq_cmd):
        dq_txt = " ".join(["{:+.2f}".format(v) for v in dq_cmd])
        lines = [
            "CLICK EN ESTA VENTANA PARA CAPTURAR EL TECLADO | ESC: salir",
            "",
            "Modo: {}".format(self.status_text),
            "Topic joint velocity: {}".format(self.topic_joint_vel),
            "Topic joint states:   {}".format(self.topic_joint_states),
            "",
            "Velocidad manual actual: {:.3f} rad/s   (+ / - para cambiar)".format(self.current_joint_speed),
            "Control automatico: kp={:.3f}  max={:.3f} rad/s  tolerancia={:.2f} deg".format(
                self.kp_auto, self.auto_max_speed, self.goal_tolerance_deg
            ),
            "",
            "CONTROLES MANUALES POR JOINT:",
            "  J1: 1 / Q     J2: 2 / W     J3: 3 / E     J4: 4 / R",
            "  J5: 5 / T     J6: 6 / Y     J7: 7 / U",
            "",
            "POSES PREDEFINIDAS:",
            "  H -> HOME      I -> INITIAL      SPACE -> cancelar auto y parar",
            "",
            "PINZA:",
            "  G -> cerrar    F -> abrir",
            "  gripper_cmd = {:.2f}".format(self.gripper),
            "",
            "VELOCIDADES ARTICULARES ENVIADAS [rad/s]:",
            "  {}".format(dq_txt),
            "",
            "ESTADO ARTICULAR:",
        ]
        lines.extend(self.joint_lines())
        return lines

    def draw_text_block(self, lines):
        self.screen.fill((0, 0, 0))
        y = 24
        for line in lines:
            surf = self.font.render(line, True, (255, 255, 255))
            rect = surf.get_rect(topleft=(20, y))
            self.screen.blit(surf, rect)
            y += 26

        footer = self.small.render(
            "Nota: H e I usan joint_states + control proporcional simple por velocidades.",
            True,
            (180, 180, 180),
        )
        self.screen.blit(footer, (20, y + 8))
        pygame.display.flip()

    def run(self):
        try:
            while not rospy.is_shutdown():
                self.handle_events()
                if rospy.is_shutdown():
                    break

                keys = pygame.key.get_pressed()

                if self.mode in ("HOME", "INITIAL"):
                    dq_cmd, done = self.compute_auto_dq()
                    if done:
                        self.mode = "manual"
                        self.target_pose = None
                        self.status_text = "Objetivo alcanzado"
                        dq_cmd = [0.0] * 7
                else:
                    dq_cmd = self.compute_manual_dq(keys)

                self.publish_joint_speeds(dq_cmd)

                msg_g = Float64()
                msg_g.data = float(self.gripper)
                self.pub_gripper.publish(msg_g)

                lines = self.build_text_lines(dq_cmd)
                self.draw_text_block(lines)
                self.clock.tick(self.rate_hz)
        finally:
            self.stop_all_joints()
            pygame.quit()
            rospy.loginfo("Saliendo de KeyboardToJointTeleop")


def main():
    rospy.init_node("teleop_kinova_joints", anonymous=False)
    node = KeyboardToJointTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

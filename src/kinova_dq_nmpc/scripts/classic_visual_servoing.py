#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np

class ClassicVisualServoingNS:
    def __init__(self):
        # Publisher para cmd_vel
        self.cmd_pub = rospy.Publisher('/ee_cartesian_vel', Twist, queue_size=10)

        # Subscriber de las features
        rospy.Subscriber('/gs_feature_coords', Float32MultiArray, self.features_callback)

        rospy.loginfo("Classic Visual Servoing (IBVS) + null-space (z y phi) con watchdog 0.05 s")

        # Parámetros de cámara
        self.f = 100.0       # distancia focal (en píxeles)
        self.h = 240.0       # alto de imagen
        self.w = 320.0       # ancho de imagen
        self.cont = 0
        # Centro de imagen
        self.v_max = self.h / 2.0
        self.u_max = self.w / 2.0

        # Ganancias del control
        self.gain_r = 0.005
        self.gain_theta = 0.5

        # Ganancias espacio nulo phi/x y z
        self.gain_phi = 0.00
        self.desired_z = 0.195  # mismo que en tu código original

        # Features deseadas
        self.desired_r = 0.0
        self.desired_theta = 0.0
        self.desired_phi = 0.0

        # ==== WATCHDOG ====
        self.timeout = 0.001  # 50 ms
        self.last_msg_time = None

        # Timer que comprueba periódicamente si se ha superado el timeout
        # self.watchdog_timer = rospy.Timer(
        #     rospy.Duration(0),  # comprueba cada 10 ms
        #     self.watchdog_callback
        # )

    # Jacobiana de un punto (3x6)
    def jacobian_point(self, p, z):
        u = p[0, 0]
        v = p[1, 0]

        u_c = u - self.u_max
        v_c = v - self.v_max

        x = u_c * z / self.f
        y = v_c * z / self.f

        j11 = -(self.f / z)
        j12 = 0.0
        j13 = (u_c / z)
        j14 = (u_c * v_c) / self.f
        j15 = -(self.f**2 + u_c**2) / self.f
        j16 = v_c

        j21 = 0.0
        j22 = -(self.f / z)
        j23 = (v_c / z)
        j24 = (self.f**2 + v_c**2) / self.f
        j25 = -(u_c * v_c) / self.f
        j26 = -u_c

        j31 = 0.0
        j32 = 0.0
        j33 = -1.0
        j34 = -y
        j35 = x
        j36 = 0.0

        J = np.array([
            [j11, j12, j13, j14, j15, j16],
            [j21, j22, j23, j24, j25, j26],
            [j31, j32, j33, j34, j35, j36]
        ])
        return J

    # Jacobiana de theta wrt (u1, v1, u2, v2) -> 1x4
    def theta_jacobian(self, p1, p2):
        u1 = p1[0, 0]; v1 = p1[1, 0]
        u2 = p2[0, 0]; v2 = p2[1, 0]

        denom = (u1 - u2)**2 + (v1 - v2)**2
        if denom < 1e-8:
            return np.zeros((1, 4))

        j11 = -(v1 - v2) / denom
        j12 = (u1 - u2) / denom
        j13 = (v1 - v2) / denom
        j14 = -(u1 - u2) / denom

        return np.array([[j11, j12, j13, j14]])

    # Jacobiana de r wrt (u1, v1, u2, v2) -> 1x4
    def r_jacobian(self, p1, p2):
        u1 = p1[0, 0]
        v1 = p1[1, 0]
        u2 = p2[0, 0]
        v2 = p2[1, 0]

        h = self.h
        w = self.w

        denom = ((u1 - u2)**2 + (v1 - v2)**2)**(3.0 / 2.0)
        if denom < 1e-8:
            return np.zeros((1, 4))

        j11 = ((v1 - v2) * (h * v1 - h * v2 + 2 * u1 * u2 - u1 * w + u2 * w +
                2 * v1 * v2 - 2 * u2**2 - 2 * v1**2)) / (2 * denom)

        j12 = -((u1 - u2) * (h * v1 - h * v2 - 2 * u1 * u2 - u1 * w + u2 * w -
                2 * v1 * v2 + 2 * u1**2 + 2 * v2**2)) / (2 * denom)

        j13 = -((v1 - v2) * (h * v1 - h * v2 - 2 * u1 * u2 - u1 * w + u2 * w -
                2 * v1 * v2 + 2 * u1**2 + 2 * v2**2)) / (2 * denom)

        j14 = ((u1 - u2) * (h * v1 - h * v2 + 2 * u1 * u2 - u1 * w + u2 * w +
                2 * v1 * v2 - 2 * u2**2 - 2 * v1**2)) / (2 * denom)

        return np.array([[j11, j12, j13, j14]])

    def classic_control_with_nullspace(self, p1, z1, p2, z2):
        """
        Control servovisual clásico:
        - Tarea principal: s = [r, theta]^T
        - Espacio nulo 1: controlar z ~ desired_z
        - Espacio nulo 2: regularizar v_x y corregir phi
        """

        # Coordenadas respecto al centro
        u1 = p1[0, 0]; v1 = p1[1, 0]
        u2 = p2[0, 0]; v2 = p2[1, 0]

        u1_c = u1 - self.u_max
        v1_c = v1 - self.v_max
        u2_c = u2 - self.u_max
        v2_c = v2 - self.v_max

        du = u2_c - u1_c
        dv = v2_c - v1_c
        dz = z2 - z1

        # Ángulo del borde
        theta = np.arctan2(dv, du)

        # Punto medio
        um = 0.5 * (u1_c + u2_c)
        vm = 0.5 * (v1_c + v2_c)

        # Distancia r al centro
        r = um * np.sin(theta) + vm * np.cos(theta)

        # Ángulo phi
        phi = np.arctan2(dz, du)

        # Profundidad media
        z_m = 0.5 * (z1 + z2)

        # Jacobianas wrt (u1, v1, u2, v2)
        J_r_uv = self.r_jacobian(p1, p2)          # 1x4
        J_theta_uv = self.theta_jacobian(p1, p2)  # 1x4

        # Jacobiana de features [r, theta] wrt (u1, v1, u2, v2)
        J_su = np.vstack((J_r_uv, J_theta_uv))    # 2x4

        # Jacobianas de los puntos wrt velocidad de cámara
        J1 = self.jacobian_point(p1, z1)   # 3x6
        J2 = self.jacobian_point(p2, z2)   # 3x6

        # L_uv: 4x6
        L_uv = np.vstack((J1[0:2, :], J2[0:2, :]))

        # Jacobiana completa de features principales: L_s: 2x6
        L_s = J_su @ L_uv

        # ===== TAREA PRINCIPAL: r, theta =====
        s = np.array([[r],
                      [theta]])

        s_des = np.array([[self.desired_r],
                          [self.desired_theta]])

        error = s_des - s                   # 2x1
        e_r = error[0, 0]
        e_theta_feat = error[1, 0]
        error_theta_orig = self.desired_theta - theta

        K = np.diag([self.gain_r, self.gain_theta])  # 2x2

        L_s_pinv = np.linalg.pinv(L_s)
        v_primary = L_s_pinv @ (K @ error)          # 6x1

        # Proyector al espacio nulo
        I6 = np.eye(6)
        N_s = I6 - L_s_pinv @ L_s

        # ===== ESPACIO NULO 1: control de z =====
        ns_z = np.array([[0.0],
                         [0.0],
                         [z_m - self.desired_z],
                         [0.0],
                         [0.0],
                         [0.0]])

        K2_z = np.diag([0.0, 0.0, 0.001, 0.0, 0.0, 0.0])
        v_z_ns = N_s @ (K2_z @ ns_z)

        # ===== ESPACIO NULO 2: v_x y phi =====
        error_phi = self.desired_phi - phi
        xi = np.array([2.0 * error_theta_orig, 10 * error_phi])
        velocity_x = 0.005 / (1.0 + np.linalg.norm(xi))

        ns_xphi = np.array([[velocity_x],  # v_x
                            [0.0],        # v_y
                            [0.0],        # v_z
                            [0.0],        # w_x
                            [error_phi],  # w_y
                            [0.0]])       # w_z

        K2_xphi = np.diag([0.01, 0.0, 0.0, 0.01, 0.01, 0.0])
        v_xphi_ns = N_s @ (K2_xphi @ ns_xphi)

        # ===== VELOCIDAD FINAL =====
        v_cam = v_primary + v_z_ns + v_xphi_ns

        return v_cam, r, theta, phi, e_r, e_theta_feat, error_phi, z_m

    def features_callback(self, msg):
        # Actualizar instante de último mensaje recibido
        
        
        self.last_msg_time = rospy.get_time()
        while(self.cont<200):
            self.cont = 1+self.cont
            twist = Twist()
            twist.linear.x  = 0
            twist.linear.y  = 0
            twist.linear.z  = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0

            self.cmd_pub.publish(twist)
            
         

        # p1: punto derecha (según tu convención actual)
        p1 = np.array([msg.data[4], msg.data[5]]).reshape(2, 1)
        z1 = msg.data[8] * 10.0

        # p2: punto izquierda
        p2 = np.array([msg.data[6], msg.data[7]]).reshape(2, 1)
        z2 = msg.data[9] * 10.0

        # === Chequeo básico de pérdida de características: NaN/inf ===
        arr = np.array([p1[0, 0], p1[1, 0], z1,
                        p2[0, 0], p2[1, 0], z2], dtype=float)

        if np.any(~np.isfinite(arr)):
            twist_zero = Twist()
            self.cmd_pub.publish(twist_zero)
            rospy.logwarn_throttle(1.0, "Features no válidas (NaN/inf) — enviando velocidades cero")
            return

        # Control clásico + espacio nulo
        v_cam, r, theta, phi, e_r, e_theta_feat, e_phi, z_m = \
            self.classic_control_with_nullspace(p1, z1, p2, z2)

        MAX_LIN = 0.03
        MAX_ANG = 2.0

        def sat(val, lim):
            return float(np.clip(val, -lim, lim))

        twist = Twist()
        twist.linear.x  = 0.1*v_cam[0, 0]
        twist.linear.y  = 0.1*v_cam[1, 0]
        twist.linear.z  = 0.1*v_cam[2, 0]
        twist.angular.x = 0.1*v_cam[3, 0]
        twist.angular.y = 0.1*v_cam[4, 0]
        twist.angular.z = 0.1*v_cam[5, 0]

        self.cmd_pub.publish(twist)

        rospy.loginfo_throttle(
            0.1,
            "Classic+NS | r=%.3f, theta=%.3f, phi=%.3f, z_m=%.3f | "
            "er=%.3f, e_theta=%.3f, ephi=%.3f" %
            (r, theta, phi, z_m, e_r, e_theta_feat, e_phi)
        )

    # def watchdog_callback(self, event):
    #     """
    #     Si no se reciben mensajes en más de 'timeout' segundos,
    #     publicar velocidades cero.
    #     """
    #     current_time = rospy.get_time()

    #     # Si nunca ha llegado un mensaje, o ya ha pasado el timeout:
    #     if self.last_msg_time is None or (current_time - self.last_msg_time) > self.timeout:
    #         twist_zero = Twist()
    #         self.cmd_pub.publish(twist_zero)
    #         rospy.logwarn_throttle(
    #             1.0,
    #             "Watchdog: sin features durante > %.3f s — enviando velocidades cero" % self.timeout
    #         )

if __name__ == '__main__':
    rospy.init_node('visual_servoing_classic_ns')
    ClassicVisualServoingNS()
    rospy.spin()


# Mismas ganancias que el desacoplado:

# #!/usr/bin/env python3
# import rospy
# from std_msgs.msg import Float32MultiArray
# from geometry_msgs.msg import Twist
# import numpy as np

# class ClassicVisualServoingNS:
#     def __init__(self):
#         # Publisher para cmd_vel
#         self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

#         # Subscriber de las features
#         rospy.Subscriber('/gs_feature_coords', Float32MultiArray, self.features_callback)

#         rospy.loginfo("Classic Visual Servoing (IBVS) + null-space (z y phi)")

#         # Parámetros de cámara
#         self.f = 100.0       # distancia focal (en píxeles)
#         self.h = 240.0       # alto de imagen
#         self.w = 320.0       # ancho de imagen

#         # Centro de imagen
#         self.v_max = self.h / 2.0
#         self.u_max = self.w / 2.0

#         # === Ganancias del control (alineadas con el código original) ===
#         self.gain_r = 0.2        # igual que gain_r en r_theta_control
#         self.gain_theta = 0.5    # igual que gain_theta en control_theta_z

#         # Ganancias espacio nulo phi/x y z
#         self.gain_phi = 0.0      # de momento no se usa explícitamente
#         self.desired_z = 0.197   # igual que desired_z del código original

#         # Features deseadas
#         self.desired_r = 0.0
#         self.desired_theta = 0.0
#         self.desired_phi = 0.0

#     # Jacobiana de un punto (3x6)
#     def jacobian_point(self, p, z):
#         u = p[0, 0]
#         v = p[1, 0]

#         u_c = u - self.u_max
#         v_c = v - self.v_max

#         x = u_c * z / self.f
#         y = v_c * z / self.f

#         j11 = -(self.f / z)
#         j12 = 0.0
#         j13 = (u_c / z)
#         j14 = (u_c * v_c) / self.f
#         j15 = -(self.f**2 + u_c**2) / self.f
#         j16 = v_c

#         j21 = 0.0
#         j22 = -(self.f / z)
#         j23 = (v_c / z)
#         j24 = (self.f**2 + v_c**2) / self.f
#         j25 = -(u_c * v_c) / self.f
#         j26 = -u_c

#         j31 = 0.0
#         j32 = 0.0
#         j33 = -1.0
#         j34 = -y
#         j35 = x
#         j36 = 0.0

#         J = np.array([
#             [j11, j12, j13, j14, j15, j16],
#             [j21, j22, j23, j24, j25, j26],
#             [j31, j32, j33, j34, j35, j36]
#         ])
#         return J

#     # Jacobiana de theta wrt (u1, v1, u2, v2) -> 1x4
#     def theta_jacobian(self, p1, p2):
#         u1 = p1[0, 0]; v1 = p1[1, 0]
#         u2 = p2[0, 0]; v2 = p2[1, 0]

#         denom = (u1 - u2)**2 + (v1 - v2)**2
#         if denom < 1e-8:
#             return np.zeros((1, 4))

#         j11 = -(v1 - v2) / denom
#         j12 = (u1 - u2) / denom
#         j13 = (v1 - v2) / denom
#         j14 = -(u1 - u2) / denom

#         return np.array([[j11, j12, j13, j14]])

#     # Jacobiana de r wrt (u1, v1, u2, v2) -> 1x4
#     def r_jacobian(self, p1, p2):
#         u1 = p1[0, 0]
#         v1 = p1[1, 0]
#         u2 = p2[0, 0]
#         v2 = p2[1, 0]

#         h = self.h
#         w = self.w

#         denom = ((u1 - u2)**2 + (v1 - v2)**2)**(3.0 / 2.0)
#         if denom < 1e-8:
#             return np.zeros((1, 4))

#         j11 = ((v1 - v2) * (h * v1 - h * v2 + 2 * u1 * u2 - u1 * w + u2 * w +
#                 2 * v1 * v2 - 2 * u2**2 - 2 * v1**2)) / (2 * denom)

#         j12 = -((u1 - u2) * (h * v1 - h * v2 - 2 * u1 * u2 - u1 * w + u2 * w -
#                 2 * v1 * v2 + 2 * u1**2 + 2 * v2**2)) / (2 * denom)

#         j13 = -((v1 - v2) * (h * v1 - h * v2 - 2 * u1 * u2 - u1 * w + u2 * w -
#                 2 * v1 * v2 + 2 * u1**2 + 2 * v2**2)) / (2 * denom)

#         j14 = ((u1 - u2) * (h * v1 - h * v2 + 2 * u1 * u2 - u1 * w + u2 * w +
#                 2 * v1 * v2 - 2 * u2**2 - 2 * v1**2)) / (2 * denom)

#         return np.array([[j11, j12, j13, j14]])

#     def classic_control_with_nullspace(self, p1, z1, p2, z2):
#         """
#         Control servovisual clásico:
#         - Tarea principal: s = [r, theta]^T
#         - Espacio nulo 1: controlar z ~ desired_z
#         - Espacio nulo 2: regularizar v_x y corregir phi
#         """

#         # Coordenadas respecto al centro
#         u1 = p1[0, 0]; v1 = p1[1, 0]
#         u2 = p2[0, 0]; v2 = p2[1, 0]

#         u1_c = u1 - self.u_max
#         v1_c = v1 - self.v_max
#         u2_c = u2 - self.u_max
#         v2_c = v2 - self.v_max

#         du = u2_c - u1_c
#         dv = v2_c - v1_c
#         dz = z2 - z1

#         # Ángulo del borde
#         theta = np.arctan2(dv, du)

#         # Punto medio
#         um = 0.5 * (u1_c + u2_c)
#         vm = 0.5 * (v1_c + v2_c)

#         # Distancia r al centro
#         r = um * np.sin(theta) + vm * np.cos(theta)

#         # Ángulo phi
#         phi = np.arctan2(dz, du)

#         # Profundidad media
#         z_m = 0.5 * (z1 + z2)

#         # Jacobianas wrt (u1, v1, u2, v2)
#         J_r_uv = self.r_jacobian(p1, p2)          # 1x4
#         J_theta_uv = self.theta_jacobian(p1, p2)  # 1x4

#         # Jacobiana de features [r, theta] wrt (u1, v1, u2, v2)
#         J_su = np.vstack((J_r_uv, J_theta_uv))    # 2x4

#         # Jacobianas de los puntos wrt velocidad de cámara
#         J1 = self.jacobian_point(p1, z1)   # 3x6
#         J2 = self.jacobian_point(p2, z2)   # 3x6

#         # L_uv: 4x6
#         L_uv = np.vstack((J1[0:2, :], J2[0:2, :]))

#         # Jacobiana completa de features principales: L_s: 2x6
#         L_s = J_su @ L_uv

#         # ===== TAREA PRINCIPAL: r, theta =====
#         s = np.array([[r],
#                       [theta]])

#         s_des = np.array([[self.desired_r],
#                           [self.desired_theta]])

#         error = s_des - s                   # 2x1
#         e_r = error[0, 0]
#         e_theta_feat = error[1, 0]
#         error_theta_orig = self.desired_theta - theta

#         # K equivalente a gain_r y gain_theta del código original
#         K = np.diag([self.gain_r, self.gain_theta])  # 2x2

#         L_s_pinv = np.linalg.pinv(L_s)
#         v_primary = L_s_pinv @ (K @ error)          # 6x1

#         # Proyector al espacio nulo
#         I6 = np.eye(6)
#         N_s = I6 - L_s_pinv @ L_s

#         # ===== ESPACIO NULO 1: control de z (como en control_theta_z) =====
#         ns_z = np.array([[0.0],
#                          [0.0],
#                          [z_m - self.desired_z],
#                          [0.0],
#                          [0.0],
#                          [0.0]])

#         K2_z = np.diag([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
#         v_z_ns = N_s @ (K2_z @ ns_z)

#         # ===== ESPACIO NULO 2: v_x y phi (como en r_theta_control) =====
#         error_phi = self.desired_phi - phi

#         # MISMO esquema que en el código original: 2*e_theta y 100*e_phi
#         xi = np.array([2.0 * error_theta_orig, 100.0 * error_phi])
#         velocity_x = 0.005 / (1.0 + np.linalg.norm(xi))

#         ns_xphi = np.array([[velocity_x],  # v_x
#                             [0.0],        # v_y
#                             [0.0],        # v_z
#                             [0.0],        # w_x
#                             [error_phi],  # w_y
#                             [0.0]])       # w_z

#         # K2_xphi respeta la relación 1 : 1 : 100 (como [1,0,1,100] en el original)
#         K2_xphi = np.diag([1.0, 0.0, 0.0, 1.0, 100.0, 0.0])
#         v_xphi_ns = N_s @ (K2_xphi @ ns_xphi)

#         # ===== VELOCIDAD FINAL =====
#         v_cam = v_primary + v_z_ns + v_xphi_ns

#         return v_cam, r, theta, phi, e_r, e_theta_feat, error_phi, z_m

#     def features_callback(self, msg):
#         # p1: punto derecha
#         p1 = np.array([msg.data[4], msg.data[5]]).reshape(2, 1)
#         z1 = msg.data[8] * 10.0

#         # p2: punto izquierda
#         p2 = np.array([msg.data[6], msg.data[7]]).reshape(2, 1)
#         z2 = msg.data[9] * 10.0

#         v_cam, r, theta, phi, e_r, e_theta_feat, e_phi, z_m = \
#             self.classic_control_with_nullspace(p1, z1, p2, z2)

#         MAX_LIN = 0.03
#         MAX_ANG = 2.0

#         def sat(val, lim):
#             return float(np.clip(val, -lim, lim))

#         twist = Twist()
#         twist.linear.x  = sat(v_cam[0, 0], MAX_LIN)
#         twist.linear.y  = sat(v_cam[1, 0], MAX_LIN)
#         twist.linear.z  = sat(v_cam[2, 0], MAX_LIN)

#         twist.angular.x = sat(v_cam[3, 0], MAX_ANG)
#         twist.angular.y = sat(v_cam[4, 0], MAX_ANG)
#         twist.angular.z = sat(v_cam[5, 0], MAX_ANG)

#         self.cmd_pub.publish(twist)

#         rospy.loginfo_throttle(
#             0.1,
#             "Classic+NS | r=%.3f, theta=%.3f, phi=%.3f, z_m=%.3f | "
#             "er=%.3f, e_theta=%.3f, ephi=%.3f" %
#             (r, theta, phi, z_m, e_r, e_theta_feat, e_phi)
#         )

# if __name__ == '__main__':
#     rospy.init_node('visual_servoing_classic_ns')
#     ClassicVisualServoingNS()
#     rospy.spin()

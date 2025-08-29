#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time

class ImageSubscriberPublisher:
    def __init__(self):
        rospy.init_node('image_subscriber_publisher', anonymous=True)

        # Parámetros
        # factor de reducción (0 = extremos originales; 0.8 = encoge 80% del semisegmento hacia el centro)
        self.shrink_frac = float(rospy.get_param('~shrink_frac', 0.4))
        self.shrink_frac = max(0.0, min(0.95, self.shrink_frac))  # límite sano

        # Dimensiones por defecto (se actualizan con la imagen real)
        self.image_h = 240
        self.image_w = 320

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber('/gelsight/depth/image_raw', Image, self.image_callback, queue_size=1)
        self.image_pub = rospy.Publisher('/gs_feature_img', Image, queue_size=1)
        self.coord_pub = rospy.Publisher('/gs_feature_coords', Float32MultiArray, queue_size=1)

        rospy.loginfo("Node initialized: Subscribed to /gelsight/depth/image_raw and publishing to /gs_feature_img, /gs_feature_coords")

    def publish_feature_coords(self, r, alpha, x_center, y_center, point1, point2, depth_data_p1, depth_data_p2):
        values = [
            float(r), float(alpha),
            float(x_center), float(y_center),
            float(point1[0]), float(point1[1]),
            float(point2[0]), float(point2[1]),
            float(depth_data_p1), float(depth_data_p2)
        ]
        if any(not math.isfinite(v) for v in values):
            return
        msg = Float32MultiArray()
        msg.data = values
        self.coord_pub.publish(msg)

    @staticmethod
    def _clamp_point_to_image(pt_xy, w, h):
        x = int(round(pt_xy[0]))
        y = int(round(pt_xy[1]))
        x_clamped = max(0, min(w - 1, x))
        y_clamped = max(0, min(h - 1, y))
        return x_clamped, y_clamped

    def image_callback(self, msg):
        rospy.loginfo_once(f"Image encoding: {msg.encoding}")

        # Convertir a OpenCV (float32)
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Actualizar dimensiones reales
        self.image_h, self.image_w = cv_image.shape[:2]
        h, w = self.image_h, self.image_w

        # === Preproceso ===
        depth_min = 0.018
        depth_max = 0.02
        depth_filtered = np.where((cv_image >= depth_min) & (cv_image <= depth_max), cv_image, 0.0)

        depth_normalized = np.clip((depth_filtered - depth_min) / (depth_max - depth_min), 0, 1)
        cv_image_8 = (depth_normalized * 255).astype(np.uint8)
        cv_image_8 = 255 - cv_image_8

        _, binary_thresh = cv2.threshold(cv_image_8, 12, 255, cv2.THRESH_BINARY)

        # Contornos externos
        contours_thresh, _ = cv2.findContours(binary_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Imagen de salida en color
        filtered = cv2.cvtColor(cv_image_8, cv2.COLOR_GRAY2BGR)

        # Defaults
        min_area = 100
        x_center, y_center = 0.0, 0.0
        point1, point2 = (0.0, 0.0), (0.0, 0.0)
        alpha = 0.0
        r = 0.0

        for cnt in contours_thresh:
            if cv2.contourArea(cnt) < min_area or len(cnt) < 2:
                continue

            # === Ajuste de línea robusta al contorno ===
            # fitLine devuelve (vx, vy, x0, y0) con (vx,vy) ~ unitario
            line = cv2.fitLine(cnt, cv2.DIST_L2, 0, 0.01, 0.01)
            vx, vy, x0, y0 = [float(v) for v in line]

            # Vector unitario del eje mayor (dirección de la línea)
            norm = math.hypot(vx, vy)
            if norm <= 1e-9:
                continue
            ux, uy = vx / norm, vy / norm

            # Proyectar TODOS los puntos del contorno sobre la línea para obtener extremos dentro del contorno
            pts = cnt.reshape(-1, 2).astype(np.float32)  # (N,2)
            # parámetros t a lo largo de la línea (p = p0 + t*u)
            t_vals = (pts[:, 0] - x0) * ux + (pts[:, 1] - y0) * uy
            t_min = float(np.min(t_vals))
            t_max = float(np.max(t_vals))

            # Extremos del segmento "dentro" del contorno, sobre la línea
            p_min = (x0 + t_min * ux, y0 + t_min * uy)
            p_max = (x0 + t_max * ux, y0 + t_max * uy)

            # Reducir el segmento por factor constante hacia su centro
            s = self.shrink_frac  # 0..0.95
            cx, cy = ( (p_min[0] + p_max[0]) * 0.5, (p_min[1] + p_max[1]) * 0.5 )
            # punto1 más "arriba" en la dirección +u, punto2 en -u (simétricos)
            half_len = 0.5 * (t_max - t_min)
            half_len_shrunk = (1.0 - s) * half_len  # cuanto nos alejamos del centro
            p1 = (cx + half_len_shrunk * ux, cy + half_len_shrunk * uy)
            p2 = (cx - half_len_shrunk * ux, cy - half_len_shrunk * uy)

            # alpha: ángulo de la línea normalizado a [-pi/2, pi/2]
            alpha = math.atan2(uy, ux)
            if alpha > math.pi / 2:
                alpha -= math.pi
            elif alpha < -math.pi / 2:
                alpha += math.pi

            # Centroide simple del segmento (para publicar como centro)
            x_center, y_center = float(cx), float(cy)

            # Clamp de puntos a la imagen para dibujar y leer profundidad
            x1c, y1c = self._clamp_point_to_image(p1, w, h)
            x2c, y2c = self._clamp_point_to_image(p2, w, h)

            point1 = (float(x1c), float(y1c))
            point2 = (float(x2c), float(y2c))

            # Dibujos de ayuda
            # Dibuja la línea encogida (segmento entre p1 y p2)
            cv2.line(filtered, (x1c, y1c), (x2c, y2c), (0, 0, 255), 2)
            cv2.circle(filtered, (x1c, y1c), 4, (255, 255, 255), -1)
            cv2.putText(filtered, "p1", (x1c + 3, y1c - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.circle(filtered, (x2c, y2c), 4, (255, 0, 255), -1)
            cv2.putText(filtered, "p2", (x2c + 3, y2c - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1, cv2.LINE_AA)
            cv2.circle(filtered, (int(round(cx)), int(round(cy))), 3, (0, 255, 0), -1)

            # Profundidad en esos puntos (imagen float32 original)
            depth_data_p1 = float(cv_image[y1c, x1c])
            depth_data_p2 = float(cv_image[y2c, x2c])

            # r: distancia con signo desde el centro de la imagen a la recta que une p1 y p2 (misma fórmula)
            r = (math.sin(alpha) * (point1[0] + point2[0] - w) +
                 math.cos(alpha) * (point1[1] + point2[1] - h)) / 2.0

            # Publicar datos
            self.publish_feature_coords(r, alpha, x_center, y_center, point1, point2, depth_data_p1, depth_data_p2)

            # Si solo quieres el contorno más relevante, podrías hacer break
            # break

        # Publicar imagen con overlays
        filtered_msg = self.bridge.cv2_to_imgmsg(filtered, encoding='bgr8')
        self.image_pub.publish(filtered_msg)
        rospy.loginfo_once("Image published.")

if __name__ == '__main__':
    try:
        node = ImageSubscriberPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

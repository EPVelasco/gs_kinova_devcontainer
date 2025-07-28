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

        self.image_h = 240
        self.image_w = 320

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber('/gelsight/depth/image_raw', Image, self.image_callback, queue_size=1)
        self.image_pub = rospy.Publisher('/gs_new_img', Image, queue_size=1)
        self.coord_pub = rospy.Publisher('/gs_feature_coords', Float32MultiArray, queue_size=1)

        rospy.loginfo("Node initialized: Subscribed to /gs_depth_image and publishing to /gs_new_img")

    def publish_feature_coords(self, x_center, y_center, point1, point2, alpha, depth_data_p1, depth_data_p2, r):
        values = [
            float(x_center), float(y_center),
            float(point1[0]), float(point1[1]),
            float(point2[0]), float(point2[1]),
            float(alpha), float(depth_data_p1), float(depth_data_p2),
            float(r)
        ]

        if any(not math.isfinite(v) for v in values):
            return

        msg = Float32MultiArray()
        msg.data = values
        self.coord_pub.publish(msg)

    def image_callback(self, msg):
        rospy.loginfo_once(f"Image encoding: {msg.encoding}")
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        start = time.time()

        # Filtrar valores fuera del rango [0.0, 0.02]
        depth_min = 0.018
        depth_max = 0.02
        depth_filtered = np.where((cv_image >= depth_min) & (cv_image <= depth_max), cv_image, 0.0)

        # Escalar a [0, 255]
        depth_normalized = np.clip((depth_filtered - depth_min) / (depth_max - depth_min), 0, 1)
        cv_image_8 = (depth_normalized * 255).astype(np.uint8)
        cv_image_8 = 255 - cv_image_8
        # Umbral binario (e.g., ignora todo lo menor a 5% del rango)
        _, binary_thresh = cv2.threshold(cv_image_8, 12, 255, cv2.THRESH_BINARY)
        
        
        thinned = cv2.ximgproc.thinning(binary_thresh)
        edges = cv2.Canny(thinned, 100, 120)
        kernel = np.ones((3, 3), np.uint8)
        dilated = cv2.dilate(edges, kernel, iterations=3)
        erode = cv2.erode(binary_thresh, kernel, iterations=10)
        contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)       
  


        contours_thresh, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered = cv2.cvtColor(cv_image_8, cv2.COLOR_GRAY2BGR)
        h, w = cv_image.shape[:2]

        min_area = 0
        x_center, y_center = 0, 0
        point1, point2 = (0, 0), (0, 0)
        alpha = 0

        for cnt in contours_thresh:
            if cv2.contourArea(cnt) >= min_area:
                try:
                    ellipse = cv2.fitEllipse(cnt)
                    (x_center, y_center), (a_len, b_len), angle = ellipse
                    cv2.ellipse(filtered, ellipse, (0, 255, 0), 1)

                    if a_len < b_len:
                        a_len, b_len = b_len, a_len
                        angle += 90.0

                    a = a_len / 2.0
                    alpha = math.radians(angle)

                    f = 0.2
                    dx_new = a * (1 - f) * math.cos(alpha)
                    dy_new = a * (1 - f) * math.sin(alpha)

                    point1 = (int(x_center + dx_new), int(y_center + dy_new))
                    point2 = (int(x_center - dx_new), int(y_center - dy_new))

                    cv2.circle(filtered, point1, 2, (255, 255, 255), -1)
                    cv2.circle(filtered, point2, 2, (255, 0, 255), -1)

                except cv2.error:
                    continue

                point1 = (min(max(point1[1], 0), self.image_h - 1), min(max(point1[0], 0), self.image_w - 1))
                point2 = (min(max(point2[1], 0), self.image_h - 1), min(max(point2[0], 0), self.image_w - 1))

                if 0 <= y_center < h and 0 <= x_center < w:
                    depth_data_p1 = float(cv_image_8[point1[0], point1[1]])
                    depth_data_p2 = float(cv_image_8[point2[0], point2[1]])
                else:
                    rospy.logwarn("Center out of image bounds.")
                    depth_data_p1 = 0.0
                    depth_data_p2 = 0.0

                r = (math.sin(alpha)*(point1[0]+point2[0]-w) + math.cos(alpha)*(point1[1]+point2[1]-h)) / 2.0
                self.publish_feature_coords(x_center, y_center, point1, point2, math.pi-alpha, depth_data_p1, depth_data_p2, r)

        filtered_msg = self.bridge.cv2_to_imgmsg(thinned, encoding='mono8')
        self.image_pub.publish(filtered_msg)
        rospy.loginfo_once("Image published.")

        rospy.loginfo("Process time: %.2f ms" % (1000 * (time.time() - start)))

if __name__ == '__main__':
    try:
        node = ImageSubscriberPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

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

        # Image dimensions
        self.image_h = 240
        self.image_w = 320

        # OpenCV-ROS bridge
        self.bridge = CvBridge()

        # Subscribers and publishers
        self.image_sub = rospy.Subscriber('/gelsight/depth/image_raw', Image, self.image_callback, queue_size=1)
        self.image_pub = rospy.Publisher('/gs_feature_img', Image, queue_size=1)
        self.coord_pub = rospy.Publisher('/gs_feature_coords', Float32MultiArray, queue_size=1)

        rospy.loginfo("Node initialized: Subscribed to /gs_depth_image and publishing to /gs_new_img")

    def publish_feature_coords(self, r, alpha, x_center, y_center, point1, point2, depth_data_p1, depth_data_p2):
        """
        Publish extracted geometric and depth data as Float32MultiArray.
        """
        values = [
            float(r), float(alpha),
            float(x_center), float(y_center),
            float(point1[0]), float(point1[1]),
            float(point2[0]), float(point2[1]),
            float(depth_data_p1), float(depth_data_p2)
        ]

        # Avoid publishing if any value is not finite
        if any(not math.isfinite(v) for v in values):
            return

        msg = Float32MultiArray()
        msg.data = values
        self.coord_pub.publish(msg)

    def image_callback(self, msg):
        rospy.loginfo_once(f"Image encoding: {msg.encoding}")

        # Convert ROS image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        start = time.time()

        # === Preprocessing ===

        # Filter out depth values outside the range [0.018, 0.02] meters
        depth_min = 0.018
        depth_max = 0.02
        depth_filtered = np.where((cv_image >= depth_min) & (cv_image <= depth_max), cv_image, 0.0)

        # Normalize filtered image to 0–255 range, then invert (brighter = closer)
        depth_normalized = np.clip((depth_filtered - depth_min) / (depth_max - depth_min), 0, 1)
        cv_image_8 = (depth_normalized * 255).astype(np.uint8)
        cv_image_8 = 255 - cv_image_8

        # Threshold to generate binary mask of the contact region
        _, binary_thresh = cv2.threshold(cv_image_8, 12, 255, cv2.THRESH_BINARY)

        # Find external contours in the binary image
        contours_thresh, _ = cv2.findContours(binary_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered = cv2.cvtColor(cv_image_8, cv2.COLOR_GRAY2BGR)
        h, w = cv_image.shape[:2]

        # Initialize default variables
        min_area = 100
        x_center, y_center = 0, 0
        point1, point2 = (0, 0), (0, 0)
        alpha = 0

        for cnt in contours_thresh:
            # Skip small contours
            if cv2.contourArea(cnt) >= min_area:
                # Fit a minimum area rectangle around the contour
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = box.astype(np.intp)
                cv2.drawContours(filtered, [box], 0, (0, 0, 255), 2)

                (x_center, y_center), (w_rect, h_rect), raw_angle = rect

                # Ensure the longer side is treated as width to get a stable angle
                if w_rect < h_rect:
                    w_rect, h_rect = h_rect, w_rect
                    raw_angle += 90.0

                # Convert angle to radians
                alpha = math.radians(raw_angle)

                # Normalize angle to [-pi/2, pi/2]
                if alpha > math.pi / 2:
                    alpha -= math.pi
                elif alpha < -math.pi / 2:
                    alpha += math.pi

                # Invert angle logic (positive slope → negative angle)
                #alpha = -alpha

                # Draw center point of the rectangle
                x_center = int(x_center)
                y_center = int(y_center)
               
                
                # Compute midpoints of the shorter sides, slightly pulled toward center
                # f = 0.6  # shrink factor (0 = corners, 1 = center)
                f = h_rect/100.0           # f factor with edge height depends on
                f = 1.0 - max(0.2, min(f, 0.9)) # f factor limitations
                                
                dx_minor = (w_rect / 2.0) * (1 - f) * math.cos(-alpha)
                dy_minor = (w_rect / 2.0) * (1 - f) * math.sin(-alpha)

                point1 = ((x_center + dx_minor), (y_center - dy_minor))  # upper point
                point2 = ((x_center - dx_minor), (y_center + dy_minor))  # lower point               

                # Retrieve raw depth at the two points (from original float32 image)
                depth_data_p1 = float(cv_image[int(point1[1]), int(point1[0])]) if (0 <= point1[1] < self.image_h and 0 <= point1[0] < self.image_w) else 0.0
                depth_data_p2 = float(cv_image[int(point2[1]), int(point2[0])]) if (0 <= point2[1] < self.image_h and 0 <= point2[0] < self.image_w) else 0.0

                # Calculate R = signed distance from image center to the line formed by point1 and point2
                h, w = self.image_h, self.image_w
                r = (math.sin(alpha) * (point1[0] + point2[0] - w) +
                     math.cos(alpha) * (point1[1] + point2[1] - h)) / 2.0
                
                # dif_y = point2[1] - point1[1]
                # dif_x = point2[0] - point1[0]
                # angle = math.atan(dif_y/dif_x)                     

                # Publish feature data
                self.publish_feature_coords(r, alpha, x_center, y_center, point1, point2, depth_data_p1, depth_data_p2)
                
                # Draw points
                
                cv2.circle(filtered, (int(point1[0]),int(point1[1])), 4, (255, 255, 255), -1)
                cv2.putText(filtered, "point 1", (5,200), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
                
                cv2.circle(filtered, (int(point2[0]),int(point2[1])), 4, (255, 0, 255), -1)
                cv2.putText(filtered, "point 2", (5,220), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1, cv2.LINE_AA)
                
                cv2.circle(filtered, (int(x_center), int(y_center)), 3, (0, 255, 0), -1)
         


        # Convert final image with visual features back to ROS and publish
        filtered_msg = self.bridge.cv2_to_imgmsg(filtered, encoding='bgr8')
        self.image_pub.publish(filtered_msg)
        rospy.loginfo_once("Image published.")

if __name__ == '__main__':
    try:
        node = ImageSubscriberPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

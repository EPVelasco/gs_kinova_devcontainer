#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker

rospy.init_node("gel_marker")
pub = rospy.Publisher("visualization_marker", Marker, queue_size=1)
rate = rospy.Rate(10)

marker = Marker()
marker.header.frame_id = "gs_gel_link"  # o tu frame
marker.type = Marker.MESH_RESOURCE
marker.mesh_resource = "package://gelsight_description/meshes/remain/gs_gel.stl"
marker.action = Marker.ADD
marker.scale.x = 0.001
marker.scale.y = 0.001
marker.scale.z = 0.001
marker.color.r = 0.8
marker.color.g = 0.8
marker.color.b = 1.0
marker.color.a = 1.0  # Opacidad completa
marker.pose.orientation.w = 1.0

while not rospy.is_shutdown():
    marker.header.stamp = rospy.Time.now()
    pub.publish(marker)
    rate.sleep()

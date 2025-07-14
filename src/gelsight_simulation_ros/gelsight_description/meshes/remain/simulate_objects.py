#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker

rospy.init_node('stl_marker_pub')
pub = rospy.Publisher('/visualization_marker', Marker, queue_size=1)

marker = Marker()
marker.header.frame_id = "world"    # frame base, por ejemplo "world" o "base_link"
marker.header.stamp = rospy.Time.now()
marker.ns = "objects"
marker.id = 0
marker.type = Marker.MESH_RESOURCE
marker.action = Marker.ADD
marker.pose.position.x = 0.5
marker.pose.position.y = 0.5
marker.pose.position.z = 0.0
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0
marker.scale.x = 0.001
marker.scale.y = 0.001
marker.scale.z = 0.001
marker.color.a = 1.0
marker.color.r = 0.0
marker.color.g = 1.0
marker.color.b = 0.0
marker.mesh_resource = "file:///home/catkin_ws/src/ros_kortex/kortex_gazebo/worlds/model_simulation/mesh/aluminum_profile.stl"
marker.mesh_use_embedded_materials = True

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    marker.header.stamp = rospy.Time.now()
    pub.publish(marker)
    rate.sleep()

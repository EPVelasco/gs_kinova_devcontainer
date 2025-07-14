#!/usr/bin/env python
import sys
import rospy
from visualization_msgs.msg import Marker
import tf

# ---------------------------
# Leer argumentos
# ---------------------------
object_name = sys.argv[1]
x = sys.argv[2]
y = sys.argv[3]
z = sys.argv[4]
roll = sys.argv[5]
pitch = sys.argv[6]
yaw = sys.argv[7]

# Paths STL
if object_name == "aluminum_profile":
    stl_file = "file:///home/catkin_ws/src/ros_kortex/kortex_gazebo/worlds/model_simulation/mesh/aluminum_profile.stl"
elif object_name == "curve_1":
    stl_file = "file:///home/catkin_ws/src/ros_kortex/kortex_gazebo/worlds/model_simulation/mesh/curve_figure_1.stl"
elif object_name == "curve_2":
    stl_file = "file:///home/catkin_ws/src/ros_kortex/kortex_gazebo/worlds/model_simulation/mesh/curve_figure_2.stl"
else:
    stl_file = ""

# ---------------------------
# Crear world file
# ---------------------------
content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <model name="{object_name}">
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>{stl_file}</uri>
              <scale>0.001 0.001 0.001</scale>
            </mesh>
          </geometry>
        </visual>
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>{stl_file}</uri>
              <scale>0.001 0.001 0.001</scale>
            </mesh>
          </geometry>
        </collision>
        <inertial>
          <mass value="1.0"/>
          <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
        </inertial>
      </link>
      <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
    </model>
  </world>
</sdf>
"""

with open("/tmp/generated_world.world", "w") as f:
    f.write(content)

# ---------------------------
# Inicializar ROS
# ---------------------------
rospy.init_node("combined_marker_node")
pub = rospy.Publisher("visualization_marker", Marker, queue_size=2)
rate = rospy.Rate(10)

quat = tf.transformations.quaternion_from_euler(float(roll), float(pitch), float(yaw))

# ---------------------------
# Marker objeto principal
# ---------------------------
if stl_file != "":
    # crear marker y publicarlo
    object_marker = Marker()
    object_marker.header.frame_id = "base_link"
    object_marker.ns = "objects"
    object_marker.id = 0
    object_marker.type = Marker.MESH_RESOURCE
    object_marker.mesh_resource = stl_file
    object_marker.action = Marker.ADD
    object_marker.scale.x = 0.001
    object_marker.scale.y = 0.001
    object_marker.scale.z = 0.001
    object_marker.color.r = 0.8
    object_marker.color.g = 0.8
    object_marker.color.b = 1.0
    object_marker.color.a = 1.0
    object_marker.pose.position.x = float(x)
    object_marker.pose.position.y = float(y)
    object_marker.pose.position.z = float(z)

    # Convertir orientación
    import tf
    quat = tf.transformations.quaternion_from_euler(float(roll), float(pitch), float(yaw))
    object_marker.pose.orientation.x = quat[0]
    object_marker.pose.orientation.y = quat[1]
    object_marker.pose.orientation.z = quat[2]
    object_marker.pose.orientation.w = quat[3]

else:
    print("⚠️⚠️⚠️⚠️⚠️  STL dosen't exist.")


gel_marker = Marker()
gel_marker.header.frame_id = "gs_gel_link" 
gel_marker.ns = "gs_gel"
gel_marker.id = 1
gel_marker.type = Marker.MESH_RESOURCE
gel_marker.mesh_resource = "package://gelsight_description/meshes/remain/gs_gel.stl"
gel_marker.action = Marker.ADD
gel_marker.scale.x = 0.001
gel_marker.scale.y = 0.001
gel_marker.scale.z = 0.001
gel_marker.color.r = 0.8
gel_marker.color.g = 0.8
gel_marker.color.b = 1.0
gel_marker.color.a = 0.8
gel_marker.pose.orientation.w = 1.0


while not rospy.is_shutdown():
    object_marker.header.stamp = rospy.Time.now()
    gel_marker.header.stamp = rospy.Time.now()

    pub.publish(object_marker)
    pub.publish(gel_marker)

    rate.sleep()

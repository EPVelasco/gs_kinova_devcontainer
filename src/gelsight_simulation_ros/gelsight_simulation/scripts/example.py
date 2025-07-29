#!/usr/bin/env python  
import rospy
import tf
from gazebo_msgs.msg import ModelStates

class TfPublisher:
    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.last_stamp = rospy.Time(0)
        self.last_pose = None

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)

    def callback(self, msg):
        try:
            idx = msg.name.index('gelsight_sensor')
            pose = msg.pose[idx]
            position = (pose.position.x, pose.position.y, pose.position.z)
            orientation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            stamp = rospy.Time.now()

            # No publicar si el tiempo no avanza o la pose no cambia
            if stamp == self.last_stamp and self.last_pose == (position, orientation):
                return

            self.br.sendTransform(position,
                                  orientation,
                                  stamp,
                                  "base_link",
                                  "world")

            self.last_stamp = stamp
            self.last_pose = (position, orientation)

        except ValueError:
            pass

if __name__ == "__main__":
    rospy.init_node('gelsight_tf_broadcaster')
    TfPublisher()
    rospy.spin()

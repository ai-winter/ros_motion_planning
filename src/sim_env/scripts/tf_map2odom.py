#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import Transform, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


class TFMap2Odom:
    def __init__(self) -> None:
        self.odom_frame_id = rospy.get_param("~odom_frame_id", "odom")

        self.tf_pub = tf2_ros.TransformBroadcaster()
        rospy.Timer(rospy.Duration(0.05), self.timer_callback)

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"
        odom.child_frame_id = self.odom_frame_id
        odom.pose.pose.orientation.w = 1

        self.tf = TransformStamped(
            header=Header(frame_id=odom.header.frame_id, stamp=odom.header.stamp),
            child_frame_id=odom.child_frame_id,
            transform=Transform(translation=odom.pose.pose.position, rotation=odom.pose.pose.orientation),
        )

    def timer_callback(self, event) -> None:
        self.tf.header.stamp = rospy.Time.now()
        self.tf_pub.sendTransform(self.tf)


# Start the node
if __name__ == "__main__":
    rospy.init_node("tf_map2odom_node")

    node = TFMap2Odom()

    rospy.spin()

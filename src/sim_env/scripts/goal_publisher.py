#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


class GoalPublisher:
    def __init__(self) -> None:
        self.robot1_goal_pub = rospy.Publisher("/robot1/move_base_simple/goal", PoseStamped, queue_size=10)
        self.robot2_goal_pub = rospy.Publisher("/robot2/move_base_simple/goal", PoseStamped, queue_size=10)
        self.robot3_goal_pub = rospy.Publisher("/robot3/move_base_simple/goal", PoseStamped, queue_size=10)
        self.robot4_goal_pub = rospy.Publisher("/robot4/move_base_simple/goal", PoseStamped, queue_size=10)

        self.robot1_goal = self.getPoseStamped(-1.0, -1.0, np.math.pi)
        self.robot2_goal = self.getPoseStamped(1.0, -1.0, -np.math.pi / 2)
        self.robot3_goal = self.getPoseStamped(1.0, 1.0, 0.0)
        self.robot4_goal = self.getPoseStamped(-1.0, 1.0, np.math.pi / 2)

    @staticmethod
    def getPoseStamped(x, y, theta):
        res = PoseStamped()
        res.pose.position.x = x
        res.pose.position.y = y

        q = quaternion_from_euler(0, 0, theta)
        res.pose.orientation.x = q[0]
        res.pose.orientation.y = q[1]
        res.pose.orientation.z = q[2]
        res.pose.orientation.w = q[3]

        res.header.frame_id = "map"
        res.header.stamp = rospy.Time.now()

        return res

    def pub(self) -> None:
        self.robot1_goal_pub.publish(self.robot1_goal)
        self.robot2_goal_pub.publish(self.robot2_goal)
        self.robot3_goal_pub.publish(self.robot3_goal)
        self.robot4_goal_pub.publish(self.robot4_goal)


if __name__ == "__main__":
    rospy.init_node("goal_publisher")

    gp = GoalPublisher()
    rospy.sleep(1.0)
    gp.pub()

    rospy.loginfo("Goal Published!")

#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64


NODE_NAME = "pid_position_control"
WAYPOINT_TOPIC = "/waypoint"


def waypoint_callback(pose):
    pass


def pos_callback(ps):
    pass


def main():
    rospy.init_node(NODE_NAME, anonymous=False)
    pos_sub = rospy.Subscriber(POSITION_TOPIC, PoseStamped, pos_callback)
    waypoint_sub = rospy.Subscriber(WAYPOINT_TOPIC, Pose, waypoint_callback)


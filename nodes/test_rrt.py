#!/usr/bin/env python

import math
import rospy
import tf
import roshelper
import random

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry


NODE_NAME = "test_rrt_node"
n = roshelper.Node(NODE_NAME, anonymous=False)

SETPOINT_TOPIC = "/setpoint_goal"
ODOM_TOPIC = "/odometry/filtered"
POLYGON_TOPIC = "/bounding_poly"


@n.entry_point()
class Landing(object):

    def __init__(self):

        self.polygon1 = PolygonStamped()
        self.polygon1.header.frame_id = "map"
        points1 = [[0,0],[6,0], [6,2], [2,2], [2, 4], [6, 4], [6,6], [0,6]]
        for point in points1:
            new_point = Point32()
            new_point.x = point[0]
            new_point.y = point[1]
            self.polygon1.polygon.points.append(new_point)

        self.polygon2 = PolygonStamped()
        self.polygon2.header.frame_id = "map"
        points2 = [[0,0],[6,0],[6,6],[0,6],[0,4],[5,4],[5,2],[0,2]]
        for point in points2:
            new_point = Point32()
            new_point.x = point[0]
            new_point.y = point[1]
            self.polygon2.polygon.points.append(new_point)

        self.pose = Odometry()
        self.pose.header.frame_id = "map"
        self.pose.pose.pose.position.x = 5.5
        self.pose.pose.pose.position.y = 1

        self.pose2 = Odometry()
        self.pose2.header.frame_id = "map"
        self.pose2.pose.pose.position.x = 4.9
        self.pose2.pose.pose.position.y = 1.1

        self.setpoint = PoseStamped()
        self.setpoint.header.frame_id = "map"
        self.setpoint.pose.position.x = 5.5
        self.setpoint.pose.position.y = 5

        self.ticker = 0


    @n.publisher(ODOM_TOPIC, Odometry)
    def publish_pose(self):
        if self.ticker % 40 < 20:
            return self.pose
        else:
            return self.pose2

    @n.publisher(POLYGON_TOPIC, PolygonStamped)
    def publish_poly(self):
        self.ticker = (self.ticker + 1) % 120
        if self.ticker < 60:
            return self.polygon1
        else:
            return self.polygon2

    @n.publisher(SETPOINT_TOPIC, PoseStamped)
    def publish_setpoint(self):
        return self.setpoint

    @n.main_loop(frequency=30)
    def run(self):
        self.publish_pose()
        self.publish_poly()
        self.publish_setpoint()


if __name__ == "__main__":
    n.start(spin=True)
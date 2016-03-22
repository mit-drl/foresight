#!/usr/bin/env python

import rospy
from collections import defaultdict
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid


NODE_NAME = "info_planner"
POSE_TOPIC = "/mavros/setpoint_position/local"
MAP_TOPIC = "/car/map"


class InfoPlanner(object):

    def __init__(self):
        self.pose = None
        self.pub = None
        self.sub = None
        self.og = None
        self.time_grid = defaultdict(lambda: defaultdict(float))

    def start(self):
        self.pub = rospy.Publisher(POSE_TOPIC, PoseStamped, queue_size=1)
        self.sub = rospy.Subscriber(
            MAP_TOPIC, OccupancyGrid, self.map_callback)
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            pass

    def map_callback(self, og):
        self.og = og


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=False)
    infopl = InfoPlanner()
    infopl.start()

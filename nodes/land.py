#!/usr/bin/env python

import roshelper
import math
import rospy
import tf
import time
from std_msgs.msg import Empty
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from foresight.msg import PoseArrayWithTimes
from foresight.msg import ForesightState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

NODE_NAME = "lander"
n = roshelper.Node(NODE_NAME, anonymous=False)

STATE_TOPIC = "/state"
GOING_HOME = 0
WAITING = 1
LANDING = 2


@n.entry_point()
class Lander(object):

    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.fixed_frame_id = rospy.get_param("~fixed_frame_id", "body")
        self.thresh = rospy.get_param("~dist_thresh", 0.1)
        self.home = PoseStamped()
        self.home.header.frame_id = self.fixed_frame_id
        self.home.header.stamp = rospy.Time.now()
        self.home.pose.position.x = 2.900
        self.home.pose.position.y = 0.0
        self.home.pose.position.z = 1.5
        self.mode = GOING_HOME
        self.start_time = 0
        self.waiting_time = 3
        self.pose = None
        self.enabled = False

    @n.subscriber("/odometry/filtered", Odometry)
    def odom_sub(self, odom):
        self.pose = odom.pose

    @n.subscriber(STATE_TOPIC, ForesightState)
    def state_sub(self, fs):
        self.enabled = self.fs.state == ForesightState.LANDING

    @n.publisher("/setpoint_goal", PoseStamped)
    def go_home(self):
        return self.home

    @n.publisher("/bebop/land", Empty)
    def land(self):
        return Empty()

    def dist_to_goal(self):
        pos = self.pose.pose.position
        spos = self.home.pose.position
        x_dist = pow(pos.x - spos.x, 2)
        y_dist = pow(pos.y - spos.y, 2)
        z_dist = pow(pos.z - spos.z, 2)
        return math.sqrt(x_dist + y_dist + z_dist)

    @n.main_loop(frequency=100)
    def run(self):
        if self.enabled:
            if self.mode == GOING_HOME:
                if self.pose is not None and self.dist_to_goal() < self.thresh:
                    rospy.loginfo("Waiting")
                    self.mode = WAITING
                    self.start_time = time.time()
                else:
                    self.go_home()
            if self.mode == WAITING:
                if time.time() - self.start_time > self.waiting_time:
                    rospy.loginfo("Landing")
                    self.mode = LANDING
                    self.land()
            if self.mode == LANDING:
                self.land()

if __name__ == "__main__":
    n.start(spin=True)

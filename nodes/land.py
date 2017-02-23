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
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

NODE_NAME = "lander"
n = roshelper.Node(NODE_NAME, anonymous=False)


GOING_HOME = 0
WAITING = 1
LANDING = 2


@n.entry_point()
class Lander(object):

    def __init__(self):
        self.frame_id = rospy.get_param("frame_id", "base_link")
        self.home = PoseStamped()
        self.home.header.frame_id = "body"
        self.home.header.stamp = rospy.Time.now()
        self.home.pose.position.x = 2.900
        self.home.pose.position.y = 0.0
        self.home.pose.position.z = 1.5
        self.mode = GOING_HOME
        self.start_time = 0
        self.waiting_time = 3
        self.pose = None

    @n.subscriber("/odometry/filtered", Odometry)
    def odom_sub(self, odom):
        self.pose = odom.pose

    @n.publisher("/waypoints", PoseArrayWithTimes)
    def go_home(self):
        pawt = PoseArrayWithTimes()
        center_pose = Pose()
        center_pose.position.x = 4.5
        center_pose.position.y = 0
        center_pose.position.z = 1.5
        # pawt.pose_array.poses.append(center_pose)
        pawt.pose_array.poses.append(self.home.pose)
        pawt.pose_array.header = self.home.header
        # pawt.wait_times.append(0.1)
        pawt.wait_times.append(0.1)
        return pawt

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
        if self.mode == GOING_HOME:
            if self.pose is not None and self.dist_to_goal() < 0.1:
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

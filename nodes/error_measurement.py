#!/usr/bin/env python

import sys
import time
import copy
import rospy
import roshelper
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from moveit_msgs.msg import RobotState
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray


NODE_NAME = "error_measurement"
n = roshelper.Node(NODE_NAME, anonymous=False)

TRUTH_TOPIC = "set_quad_pose"

EKF_TOPIC = "ekf_pose"
ALEX_TOPIC = "odometry/filtered"
ALEX_ERROR_TOPIC = "error/alex"
EKF_ERROR_TOPIC = "error/ekf"

@n.entry_point()
class Error(object):

    def __init__(self):
        self.data = dict()

    @n.subscriber(TRUTH_TOPIC, Odometry)
    def odom_sub(self, odom):
        self.data['truth'] = odom

    @n.subscriber(EKF_TOPIC, Odometry)
    def ekf_sub(self, ps):
        self.data['ekf'] = ps

    @n.subscriber(ALEX_TOPIC, Odometry)
    def alex_sub(self, odom):
        self.data['alex'] = odom

    @n.publisher(Pose)
    def error_pub(self, comp):
        truth = self.data['truth']

        error = Pose()
        c = comp.pose.pose.position
        t = truth.pose.pose.position
        error.position.x = c.x - t.x
        error.position.y = c.y - t.y
        error.position.z = c.z - t.z

        return error
        
    @n.main_loop(frequency=100)
    def run(self):
        if len(self.data) == 3:
            self.error_pub(self.data['alex']).publish(ALEX_ERROR_TOPIC)
            self.error_pub(self.data['ekf']).publish(EKF_ERROR_TOPIC)
            self.data = dict()

if __name__ == "__main__":
    n.start(spin=True)
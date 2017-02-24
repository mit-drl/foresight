#!/usr/bin/env python

import roshelper
import rospy
from foresight.msg import ForesightState
from geometry_msgs.msg import PoseStamped

NODE_NAME = "hovering"
n = roshelper.Node(NODE_NAME, anonymous=False)

STATE_TOPIC = "/state"


@n.entry_point()
class Hovering(object):

    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.fixed_frame_id = rospy.get_param("~fixed_frame_id", "body")
        self.home = PoseStamped()
        self.home.header.frame_id = self.fixed_frame_id
        self.home.header.stamp = rospy.Time.now()
        self.home.pose.position.x = 4.500
        self.home.pose.position.y = 0.0
        self.home.pose.position.z = 1.5
        self.enabled = False

    @n.subscriber(STATE_TOPIC, ForesightState)
    def state_sub(self, fs):
        self.enabled = fs.state == ForesightState.HOVERING

    @n.publisher("/setpoint_goal", PoseStamped)
    def go_home(self):
        return self.home

    @n.main_loop(frequency=100)
    def run(self):
        if self.enabled:
            self.go_home()


if __name__ == "__main__":
    n.start(spin=True)

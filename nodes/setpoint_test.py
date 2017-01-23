#!/usr/bin/env python

import roshelper
from geometry_msgs.msg import PoseStamped

n = roshelper.Node("setpoint_test_node", anonymous=False)


@n.publisher("/setpoint_pose", PoseStamped)
def pub_setpoint(x, y, z):
    ps = PoseStamped()
    ps.header.frame_id = "odom"
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = z
    return ps


@n.entry_point(frequency=30)
def run():
    pub_setpoint(0.5, 0.5, 2)


if __name__ == "__main__":
    n.start(spin=True)

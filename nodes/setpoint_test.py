#!/usr/bin/env python

import roshelper
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion

n = roshelper.Node("setpoint_test_node", anonymous=False)


@n.publisher("/setpoint_pose", PoseStamped)
def pub_setpoint(x, y, z, yaw):
    ps = PoseStamped()
    ps.header.frame_id = "odom"
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = z
    quat = Quaternion()
    fake_quat= tf.transformations.quaternion_from_euler(0,0,yaw)
    quat.x = fake_quat[0]
    quat.y = fake_quat[1]
    quat.z = fake_quat[2]
    quat.w = fake_quat[3]
    ps.pose.orientation = quat
    return ps


@n.entry_point(frequency=30)
def run():
    pub_setpoint(0.5, 0.5, 2, 1.57)


if __name__ == "__main__":
    n.start(spin=True)

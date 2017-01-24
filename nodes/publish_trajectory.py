#!/usr/bin/env python

import roshelper
import tf
import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion

n = roshelper.Node("trajectory_node", anonymous=False)


@n.publisher("/trajectory", PoseArray)
def pub_setpoint():
    points = [0, 0, 0, 0, 0, 0, 0]
    points[0] = [0.0, 0.0, 2, 0]
    points[1] = [0.75, 0.0, 2, 0]
    points[2] = [0.75, 0.75, 2, 0]
    points[3] = [0.0, 0.75, 2, 0]
    points[4] = [0.0, 0.0, 2, 0]
    points[5] = [0.0, 0.0, 2, 1.57]
    points[6] = [0.0, 0.0, 2, 3.14]

    poses = []

    for point in points:
        p = Pose()
        p.position.x = point[0]
        p.position.y = point[1]
        p.position.z = point[2]

        quat = Quaternion()
        fake_quat= tf.transformations.quaternion_from_euler(0,0,point[3])
        quat.x = fake_quat[0]
        quat.y = fake_quat[1]
        quat.z = fake_quat[2]
        quat.w = fake_quat[3]
        p.orientation = quat
        poses.append(p)

    traj = PoseArray()
    traj.header.frame_id = "odom"
    traj.poses = poses

    return traj

@n.entry_point(frequency=30)
def run():
    pub_setpoint()


if __name__ == "__main__":
    n.start(spin=True)

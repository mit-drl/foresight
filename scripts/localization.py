#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, PoseArray


class Localization(object):

    def __init__(self, K):
        self.K = K
        self.april_pose = Pose()
        self.april_xs = list()
        self.april_ys = list()
        self.april_zs = list()
        self.lsd_pose = Pose()
        self.lsd_xs = list()
        self.lsd_ys = list()
        self.lsd_zs = list()

    def apriltags_callback(self, pose_array):
        if len(pose_array.poses) > 0:
            self.april_pose = pose_array.poses[0]
            self.april_xs.append(self.april_pose.position.x)
            self.april_ys.append(self.april_pose.position.y)
            self.april_zs.append(self.april_pose.position.z)
            if len(self.april_xs) > self.K:
                self.april_xs.pop(0)
                self.april_ys.pop(0)
                self.april_zs.pop(0)

    def lsd_callback(self, pose_stamped):
        self.lsd_pose = pose_stamped.pose
        self.lsd_xs.append(self.lsd_pose.position.x)
        self.lsd_ys.append(self.lsd_pose.position.y)
        self.lsd_zs.append(self.lsd_pose.position.z)
        if len(self.lsd_xs) > self.K:
            self.lsd_xs.pop(0)
            self.lsd_ys.pop(0)
            self.lsd_zs.pop(0)

    def get_pose(self):
        return self.pose

    def get_lsd_pose(self):
        if len(self.april_xs) == self.K and len(self.lsd_xs) == self.K:
            p_x = np.polyfit(np.array(self.april_xs), np.array(self.lsd_xs), 1)
            p_y = np.polyfit(np.array(self.april_ys), np.array(self.lsd_ys), 1)
            p_z = np.polyfit(np.array(self.april_zs), np.array(self.lsd_zs), 1)
            x = p_x[0] * self.lsd_pose.position.x + p_x[1]
            y = p_y[0] * self.lsd_pose.position.y + p_y[1]
            z = p_z[0] * self.lsd_pose.position.z + p_z[1]
            lsd_pose_fixed = PoseStamped()
            lsd_pose_fixed.header.frame_id = "tag_0"
            lsd_pose_fixed.pose.position.x = x
            lsd_pose_fixed.pose.position.y = y
            lsd_pose_fixed.pose.position.z = z
            return lsd_pose_fixed
        else:
            return PoseStamped()


def main():
    rospy.init_node("fs_localization", anonymous=False)
    loc = Localization(10)
    rospy.Subscriber("/lsd_slam/pose", PoseStamped, loc.lsd_callback)
    rospy.Subscriber("/tag_detections_pose", PoseArray, loc.apriltags_callback)
    pub = rospy.Publisher("/localization/lsd_pose", PoseStamped, queue_size=10)
    r = rospy.Rate(30)
    while True:
        pub.publish(loc.get_lsd_pose())
        r.sleep()


if __name__ == "__main__":
    main()

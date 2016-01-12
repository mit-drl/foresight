#!/usr/bin/env python

import rospy
import numpy as np
from pairedtimeseries import PairedTimeSeries
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray


class Localization(object):

    def __init__(self, K):
        self.K = K
        self.april_pose = Pose()
        self.april_time = 0
        self.lsd_time = 0
        self.ptsx = PairedTimeSeries(self.K)
        self.ptsy = PairedTimeSeries(self.K)
        self.ptsz = PairedTimeSeries(self.K)
        self.april_xs = list()
        self.april_ys = list()
        self.april_zs = list()
        self.lsd_pose = Pose()
        self.lsd_xs = list()
        self.lsd_ys = list()
        self.lsd_zs = list()
        self.p_x = [0, 0]
        self.p_y = [0, 0]
        self.p_z = [0, 0]

    def apriltags_callback(self, pose_array):
        if len(pose_array.poses) > 0:
            self.april_pose = pose_array.poses[0]
            self.april_time = pose_array.header.stamp.nsecs
            self.april_xs.append(self.april_pose.position.x)
            self.april_ys.append(self.april_pose.position.y)
            self.april_zs.append(self.april_pose.position.z)
            self.ptsx.add_y(self.april_pose.position.x, self.april_time)
            self.ptsy.add_y(self.april_pose.position.y, self.april_time)
            self.ptsz.add_y(self.april_pose.position.z, self.april_time)
            if len(self.april_xs) > self.K:
                self.april_xs.pop(0)
                self.april_ys.pop(0)
                self.april_zs.pop(0)

    def lsd_callback(self, pose_stamped):
        self.lsd_pose = pose_stamped.pose
        self.lsd_time = pose_stamped.header.stamp.nsecs
        self.lsd_xs.append(self.lsd_pose.position.x)
        self.lsd_ys.append(self.lsd_pose.position.y)
        self.lsd_zs.append(self.lsd_pose.position.z)
        self.ptsx.add_x(self.lsd_pose.position.x, self.lsd_time)
        self.ptsy.add_x(self.lsd_pose.position.y, self.lsd_time)
        self.ptsz.add_x(self.lsd_pose.position.z, self.lsd_time)
        if len(self.lsd_xs) > self.K:
            self.lsd_xs.pop(0)
            self.lsd_ys.pop(0)
            self.lsd_zs.pop(0)

    def get_pose(self):
        return self.pose

    def get_lsd_pose(self):
        # if len(self.april_xs) == self.K and len(self.lsd_xs) == self.K:
        x_xs, x_ys = self.ptsx.get_time_series()
        y_xs, y_ys = self.ptsx.get_time_series()
        z_xs, z_ys = self.ptsz.get_time_series()
        if len(x_xs) > 2:
            self.p_x = np.polyfit(x_xs, x_ys, 1)
            self.p_y = np.polyfit(y_xs, y_ys, 1)
            self.p_z = np.polyfit(z_xs, z_ys, 1)
            # self.p_x = np.polyfit(np.array(self.lsd_xs),
            #                       np.array(self.april_xs), 1)
            # self.p_y = np.polyfit(np.array(self.lsd_ys),
            #                       np.array(self.april_ys), 1)
            # self.p_z = np.polyfit(np.array(self.lsd_zs),
            #                       np.array(self.april_zs), 1)
            x = self.p_x[0] * self.lsd_pose.position.x + self.p_x[1]
            y = self.p_y[0] * self.lsd_pose.position.y + self.p_y[1]
            z = self.p_z[0] * self.lsd_pose.position.z + self.p_z[1]
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
    loc = Localization(100)
    rospy.Subscriber("/lsd_slam/pose", PoseStamped, loc.lsd_callback)
    rospy.Subscriber("/tag_detections_pose", PoseArray, loc.apriltags_callback)
    pub = rospy.Publisher("/localization/lsd_pose", PoseStamped, queue_size=10)
    r = rospy.Rate(30)
    while True:
        pub.publish(loc.get_lsd_pose())
        r.sleep()


if __name__ == "__main__":
    main()

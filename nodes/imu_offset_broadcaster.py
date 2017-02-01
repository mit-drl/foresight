#!/usr/bin/env python

import roshelper
import rospy
import tf
import math
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler


NODE_NAME = "imu_offset_broadcaster"
n = roshelper.Node(NODE_NAME, anonymous=False)


@n.entry_point()
class ImuOffsetBroadcaster(object):

    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.tfl = tf.TransformListener()
        self.ori = None
        self.yaw_zero = None
        self.car_zero = None
        self.yaw = None
        self.imu = None
        self.odom = Odometry()
        self.odom_offset = Odometry()
        self.odom_offset.header.frame_id = "golfcartlw/base_link"
        self.odom_offset.child_frame_id = "base_link"

    @n.subscriber("/bebop/odom_cov", Odometry)
    def odom_sub(self, odom):
        if self.car_zero is None:
            return

        self.odom = odom
        ori = odom.pose.pose.orientation
        quat = [ori.x, ori.y, ori.z, ori.w]
        r, p, self.yaw = euler_from_quaternion(quat)

        if self.yaw_zero is None:
            self.yaw_zero = self.yaw

        # dyaw = self.car_zero + self.yaw - self.yaw_zero
        dyaw = self.yaw - self.yaw_zero
        qx, qy, qz, qw = quaternion_from_euler(r, p, dyaw)
        self.dyaw = dyaw
        self.quat = [qx, qy, qz, qw]
        self.odom_offset.pose = self.odom.pose
        self.odom_offset.pose.pose.position.x += 1
        # self.ps.pose.pose.position.x += 1
        # self.ps.pose.pose.orientation.x = qx
        # self.ps.pose.pose.orientation.y = qy
        # self.ps.pose.pose.orientation.z = qz
        # self.ps.pose.pose.orientation.w = qw

    @n.subscriber("/golfcartlw/imu/data", Imu)
    def imu_sub(self, imu):
        if self.imu is None:
            ori = imu.orientation
            quat = [ori.x, ori.y, ori.z, ori.w]
            _, _, yaw = euler_from_quaternion(quat)

            if self.car_zero is None:
                self.car_zero = yaw

    @n.publisher("/odom_offset_cov", Odometry)
    def odom_offset_pub(self):
        return self.odom_offset

    @n.main_loop(frequency=30)
    def run(self):
        if self.yaw_zero is not None:
            self.odom_offset_pub()
            trans = (3.5, 0, 0)
            quat = quaternion_from_euler(0, 0, -self.yaw_zero)
            self.br.sendTransform(
                trans,
                quat,
                rospy.Time.now(),
                "odom", "golfcartlw/base_link")


if __name__ == "__main__":
    n.start(spin=True)

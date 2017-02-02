#!/usr/bin/env python

import roshelper
import rospy
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler


NODE_NAME = "imu_offset_publisher"
n = roshelper.Node(NODE_NAME, anonymous=False)


@n.entry_point()
class ImuOffsetPublisher(object):

    def __init__(self):
        self.yaw_zero = None
        self.odom_offset = Odometry()
        self.odom_offset.header.frame_id = "golfcartlw/base_link"
        self.odom_offset.child_frame_id = "base_link"
        self.x0 = rospy.get_param("~x0")
        self.y0 = rospy.get_param("~y0")
        self.z0 = rospy.get_param("~z0")

    def rotate_xy(self, x, y, rad):
        xp = x * math.cos(rad) - y * math.sin(rad)
        yp = y * math.cos(rad) + x * math.sin(rad)
        return xp, yp

    @n.subscriber("/bebop/odom_cov", Odometry)
    def odom_sub(self, odom):
        ori = odom.pose.pose.orientation
        ori_quat = [ori.x, ori.y, ori.z, ori.w]
        r, p, yaw = euler_from_quaternion(ori_quat)

        if self.yaw_zero is None:
            self.yaw_zero = yaw

        dyaw = yaw - self.yaw_zero
        quat = quaternion_from_euler(r, p, dyaw)
        pos = odom.pose.pose.position
        self.odom_offset_pub(pos, quat)

    @n.publisher("/odom_offset_cov", Odometry)
    def odom_offset_pub(self, pos, quat):
        dx, dy = self.rotate_xy(pos.x, pos.y, -self.yaw_zero)
        self.odom_offset.header.stamp = rospy.Time.now()
        self.odom_offset.pose.pose.position.x = self.x0 + dx
        self.odom_offset.pose.pose.position.y = self.y0 + dy
        self.odom_offset.pose.pose.position.z = pos.z + self.z0
        self.odom_offset.pose.pose.orientation.x = quat[0]
        self.odom_offset.pose.pose.orientation.y = quat[1]
        self.odom_offset.pose.pose.orientation.z = quat[2]
        self.odom_offset.pose.pose.orientation.w = quat[3]
        return self.odom_offset

    @n.main_loop(frequency=30)
    def run(self):
        pass

if __name__ == "__main__":
    n.start(spin=True)

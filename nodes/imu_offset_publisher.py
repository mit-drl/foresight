#!/usr/bin/env python

import roshelper
import rospy
import math
import tf
import tf2_ros
import tf2_geometry_msgs as tf2_geom
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler


NODE_NAME = "imu_offset_publisher"
n = roshelper.Node(NODE_NAME, anonymous=False)


@n.entry_point()
class ImuOffsetPublisher(object):

    def __init__(self):
        self.yaw_zero = None
        self.car_frame_id = rospy.get_param("car_frame_id",
                                            "body")
        self.frame_id = rospy.get_param("frame_id", "base_link")
        self.odom_offset = Odometry()
        self.odom = Odometry()
        self.odom_offset.header.frame_id = self.car_frame_id
        self.odom_offset.child_frame_id = self.frame_id
        self.tf_buffer = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tf_buffer)

    def rotate_xy(self, x, y, rad):
        xp = x * math.cos(rad) - y * math.sin(rad)
        yp = y * math.cos(rad) + x * math.sin(rad)
        return xp, yp

    def cov_mat(self, ps_cov, odom):
        mat = [0] * 36
        mat[:8] = ps_cov.pose.covariance[:8]
        mat[8:] = odom.pose.covariance[8:]
        return mat

    @n.subscriber("/bebop/odom_cov", Odometry)
    def odom_sub(self, odom):
        ori = odom.pose.pose.orientation
        ori_quat = [ori.x, ori.y, ori.z, ori.w]
        r, p, yaw = euler_from_quaternion(ori_quat)

        if self.yaw_zero is None:
            self.yaw_zero = yaw

        dyaw = yaw - self.yaw_zero
        quat = quaternion_from_euler(r, p, dyaw)
        self.relative_quat = quat
        self.odom = odom
        # pos = odom.pose.pose.position
        # self.odom_offset_pub(pos, quat)

    @n.subscriber("/uwb_pose_cov", PoseWithCovarianceStamped)
    def uwb_pose_cov_sub(self, ps_cov):
        ps = tf2_geom.PoseStamped()
        ps.header = ps_cov.header
        ps.pose = ps_cov.pose.pose
        ps_tf = self.tf_buffer.transform(ps, self.car_frame_id)
        x = ps_tf.pose.position.x  # + self.x0
        y = ps_tf.pose.position.y  # + self.y0
        z = self.odom.pose.pose.position.z  # + self.z0
        self.odom_offset.header.stamp = rospy.Time.now()
        self.odom_offset.pose.pose.position.x = x
        self.odom_offset.pose.pose.position.y = y
        self.odom_offset.pose.pose.position.z = z
        self.odom_offset.pose.pose.orientation.x = self.relative_quat[0]
        self.odom_offset.pose.pose.orientation.y = self.relative_quat[1]
        self.odom_offset.pose.pose.orientation.z = self.relative_quat[2]
        self.odom_offset.pose.pose.orientation.w = self.relative_quat[3]
        self.odom_offset.pose.covariance = self.cov_mat(ps_cov, self.odom)
        self.pub_odom_offset()

    @n.publisher("/odom_offset_cov", Odometry)
    def pub_odom_offset(self):
        return self.odom_offset

    # @n.publisher("/odom_offset_cov", Odometry)
    def odom_offset_pub_old(self, pos, quat):
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

#!/usr/bin/env python

import roshelper
import rospy
import math
import tf
import tf2_ros
import tf2_geometry_msgs as tf2_geom
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler


NODE_NAME = "relative_odom_publisher"
n = roshelper.Node(NODE_NAME, anonymous=False)


@n.entry_point()
class RelativeOdomPublisher(object):

    def __init__(self):
        self.yaw_zero = None
        self.car_yaw_zero = None
        self.car_dyaw = 0
        self.car_frame_id = rospy.get_param("~car_frame_id",
                                            "body")
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.odom_offset = Odometry()
        self.odom_offset_2d = Odometry()
        self.odom_offset_alt = Odometry()
        self.odom = Odometry()
        self.odom_offset.header.frame_id = self.car_frame_id
        self.odom_offset.child_frame_id = self.frame_id
        self.odom_offset_2d.header.frame_id = self.car_frame_id
        self.odom_offset_2d.child_frame_id = self.frame_id
        self.odom_offset_alt.header.frame_id = "odom"
        self.odom_offset_alt.child_frame_id = self.frame_id
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

        dyaw = yaw - self.yaw_zero - self.car_dyaw
        quat = quaternion_from_euler(r, p, dyaw)
        self.relative_quat = quat
        self.odom = odom

    @n.subscriber("/white_prius/odometry/filtered/odom", Odometry)
    def car_odom_sub(self, odom):
        ori = odom.pose.pose.orientation
        ori_quat = [ori.x, ori.y, ori.z, ori.w]
        _, _, yaw = euler_from_quaternion(ori_quat)

        if self.car_yaw_zero is None:
            self.car_yaw_zero = yaw

        self.car_dyaw = yaw - self.car_yaw_zero

    @n.subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged",
                  Ardrone3PilotingStateAltitudeChanged)
    def altitude_sub(self, alt):
        cov = [0] * 36
        cov[17] = 0.05
        self.odom_offset_alt.header.stamp = rospy.Time.now()
        self.odom_offset_alt.pose.pose.position.z = alt.altitude
        self.odom_offset_alt.pose.covariance = cov
        self.pub_odom(self.odom_offset_alt).publish("/odom_alt")

    @n.subscriber("/uwb_pose_cov_3d", PoseWithCovarianceStamped)
    def uwb_pose_cov_sub_3d(self, ps_cov):
        if self.yaw_zero is None or self.car_yaw_zero is None:
            return

        ps = tf2_geom.PoseStamped()
        ps.header = ps_cov.header
        ps.pose = ps_cov.pose.pose
        ps_tf = self.tf_buffer.transform(ps, self.car_frame_id)
        x = ps_tf.pose.position.x  # + self.x0
        y = ps_tf.pose.position.y  # + self.y0
        z = ps_tf.pose.position.z
        self.odom_offset.header.stamp = rospy.Time.now()
        self.odom_offset.pose.pose.position.x = x
        self.odom_offset.pose.pose.position.y = y
        self.odom_offset.pose.pose.position.z = z
        self.odom_offset.pose.pose.orientation.x = self.relative_quat[0]
        self.odom_offset.pose.pose.orientation.y = self.relative_quat[1]
        self.odom_offset.pose.pose.orientation.z = self.relative_quat[2]
        self.odom_offset.pose.pose.orientation.w = self.relative_quat[3]
        self.odom_offset.pose.covariance = self.cov_mat(ps_cov, self.odom)
        self.pub_odom(self.odom_offset).publish("/odom_uwb_3d")

    @n.subscriber("/uwb_pose_cov_2d", PoseWithCovarianceStamped)
    def uwb_pose_cov_sub_2d(self, ps_cov):
        if self.yaw_zero is None or self.car_yaw_zero is None:
            return

        ps = tf2_geom.PoseStamped()
        ps.header = ps_cov.header
        ps.pose = ps_cov.pose.pose
        ps_tf = self.tf_buffer.transform(ps, self.car_frame_id)
        x = ps_tf.pose.position.x  # + self.x0
        y = ps_tf.pose.position.y  # + self.y0
        z = self.odom.pose.pose.position.z  # + self.z0
        self.odom_offset_2d.header.stamp = rospy.Time.now()
        self.odom_offset_2d.pose.pose.position.x = x
        self.odom_offset_2d.pose.pose.position.y = y
        self.odom_offset_2d.pose.pose.position.z = z
        self.odom_offset_2d.pose.pose.orientation.x = self.relative_quat[0]
        self.odom_offset_2d.pose.pose.orientation.y = self.relative_quat[1]
        self.odom_offset_2d.pose.pose.orientation.z = self.relative_quat[2]
        self.odom_offset_2d.pose.pose.orientation.w = self.relative_quat[3]
        self.odom_offset_2d.pose.covariance = self.cov_mat(ps_cov, self.odom)
        self.pub_odom(self.odom_offset_2d).publish("/odom_uwb_2d")

    @n.publisher(Odometry)
    def pub_odom(self, odom):
        return odom

    @n.main_loop(frequency=30)
    def run(self):
        pass

if __name__ == "__main__":
    n.start(spin=True)

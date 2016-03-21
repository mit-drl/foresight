#!/usr/bin/env python

import rospy
import math
import tf
# import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from mavros_msgs.msg import OpticalFlowRad


NODE_NAME = "px4flow_odom"
OPTICAL_FLOW_TOPIC = "/foresight/optical_flow_rad/filtered"
IMU_TOPIC = "/imu/data"
ODOM_TOPIC = "/foresight/px4flow/odom"
ODOM_FRAME_ID = "odom"
ODOM_CHILD_FRAME_ID = "base_link"


class OpticalFlowOdom(object):

    def __init__(self, optical_flow_topic, imu_topic, odom_topic,
                 odom_frame_id, odom_child_frame_id):
        self.optical_flow_topic = optical_flow_topic
        self.imu_topic = imu_topic
        self.odom_topic = odom_topic
        self.odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size=10)
        self.br = tf.TransformBroadcaster()
        self.optical_flow_sub = None
        self.imu_sub = None
        self.odom = Odometry()
        self.odom.header.frame_id = odom_frame_id
        self.odom.child_frame_id = odom_child_frame_id
        self.o_cov = [0] * 9

    def start(self):
        self.optical_flow_sub = rospy.Subscriber(
            self.optical_flow_topic, OpticalFlowRad,
            self.optical_flow_callback)
        self.imu_sub = rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)
        return self

    def covariance_matrix(self, x_p, y_p, z_p, x_r, y_r, z_r):
        return [x_p, 0, 0, 0, 0, 0,
                0, y_p, 0, 0, 0, 0,
                0, 0, z_p, 0, 0, 0,
                0, 0, 0, x_r, 0, 0,
                0, 0, 0, 0, y_r, 0,
                0, 0, 0, 0, 0, z_r]

    def optical_flow_callback(self, ofr):
        h = ofr.distance
        r_x = ofr.integrated_x
        r_y = ofr.integrated_y
        x_rel = 2 * h * math.tan(r_x / 2.0)
        y_rel = 2 * h * math.tan(r_y / 2.0)
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [self.odom.pose.pose.orientation.x,
             self.odom.pose.pose.orientation.y,
             self.odom.pose.pose.orientation.z,
             self.odom.pose.pose.orientation.w])
        x_abs = x_rel * math.cos(yaw) - y_rel * math.sin(yaw)
        y_abs = x_rel * math.sin(yaw) + y_rel * math.cos(yaw)
        self.odom.header.seq += 1
        self.odom.header.stamp = rospy.Time.now()
        self.odom.pose.pose.position.x -= x_abs
        self.odom.pose.pose.position.y -= y_abs
        self.odom.pose.pose.position.z = h

    def imu_callback(self, imu):
        self.odom.pose.pose.orientation = imu.orientation
        self.o_cov = imu.orientation_covariance

    def publish_odom(self):
        self.odom.pose.covariance = self.covariance_matrix(
            1e-2, 1e-2, 1e-2, self.o_cov[0], self.o_cov[4], self.o_cov[8])
        self.odom_pub.publish(self.odom)
        return self


def main():
    rospy.init_node(NODE_NAME, anonymous=False)
    hz = rospy.get_param("~frequency", 30)
    optical_flow_topic = rospy.get_param("~optical_flow_topic",
                                         OPTICAL_FLOW_TOPIC)
    imu_topic = rospy.get_param("~imu_topic", IMU_TOPIC)
    odom_topic = rospy.get_param("~odom_topic", ODOM_TOPIC)
    odom_frame_id = rospy.get_param("~odom_frame_id", ODOM_FRAME_ID)
    odom_child_frame_id = rospy.get_param("~odom_child_frame_id",
                                          ODOM_FRAME_ID)
    r = rospy.Rate(hz)
    ofo = OpticalFlowOdom(optical_flow_topic,
                          imu_topic,
                          odom_topic,
                          odom_frame_id,
                          odom_child_frame_id).start()
    while not rospy.is_shutdown():
        ofo.publish_odom()
        r.sleep()


if __name__ == "__main__":
    main()

#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from mavros_msgs.msg import OpticalFlowRad


NODE_NAME = "px4flow_odom"
OPTICAL_FLOW_TOPIC = "/mavros/px4flow/raw/optical_flow_rad"
IMU_TOPIC = "/mavros/imu/data"
ODOM_TOPIC = "/foresight/px4flow/odom"
ODOM_FRAME_ID = "odom"
ODOM_CHILD_FRAME_ID = "base_link"


class OpticalFlowOdom(object):

    def __init__(self, optical_flow_topic, imu_topic, odom_topic):
        self.optical_flow_topic = optical_flow_topic
        self.imu_topic = imu_topic
        self.odom_topic = odom_topic
        self.odom_pub = rospy.Publisher(
            odom_topic, Odometry, queue_size=10)
        self.optical_flow_sub = None
        self.odom = Odometry()
        self.odom.header.frame_id = ODOM_FRAME_ID
        self.odom.child_frame_id = ODOM_CHILD_FRAME_ID
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
        self.odom.header.seq += 1
        self.odom.header.stamp = rospy.Time.now()
        self.odom.pose.pose.position.x -= 2 * h * math.tan(r_y / 2.0)
        self.odom.pose.pose.position.y += 2 * h * math.tan(r_x / 2.0)
        self.odom.pose.pose.position.z = ofr.distance

    def imu_callback(self, imu):
        self.odom.pose.pose.orientation.x = imu.orientation.x
        self.odom.pose.pose.orientation.y = imu.orientation.y
        self.odom.pose.pose.orientation.z = imu.orientation.z
        self.odom.pose.pose.orientation.w = imu.orientation.w
        self.o_cov = imu.orientation_covariance

    def publish_odom(self):
        self.odom.pose.covariance = self.covariance_matrix(
            1, 1, 1e6, self.o_cov[0], self.o_cov[4], self.o_cov[8])
        self.odom_pub.publish(self.odom)


def main():
    rospy.init_node(NODE_NAME, anonymous=False)
    ofo = OpticalFlowOdom(OPTICAL_FLOW_TOPIC, IMU_TOPIC, ODOM_TOPIC).start()
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        ofo.publish_odom()
        r.sleep()


if __name__ == "__main__":
    main()

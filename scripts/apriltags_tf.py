#!/usr/bin/env python

import rospy
import tf
from apriltags_ros.msg import AprilTagDetectionArray

NODE_NAME = "apriltags_pose"
FREQUENCY = 30
LANDING_PAD_ID = "/landing_pad"
USB_CAM_ID = "/usb_cam"
ODOM_ID = "/odom"
MAP_ID = "/map"
BASE_LINK_ID = "/base_link"
TAG_DETECTIONS_TOPIC = "/tag_detections"
TF_TIMEOUT = 0.3


class AprilTagsTransformer(object):

    def __init__(self, frequency):
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(frequency)
        self.tf_exs = (tf.LookupException,
                       tf.ConnectivityException,
                       tf.ExtrapolationException)
        self.sub = None
        self.latest_odom_tf = (0, 0, 0)

    def start(self):
        self.sub = rospy.Subscriber(
            TAG_DETECTIONS_TOPIC, AprilTagDetectionArray, self.apriltags_cb)
        return self
        # while not rospy.is_shutdown():
        #     self.run()
        #     self.rate.sleep()

    def apriltags_cb(self, tag_array):
        tags = tag_array.detections
        try:
            tob_args = [ODOM_ID, BASE_LINK_ID, rospy.Time(0)]
            (xo, yo, zo), _ = self.listener.lookupTransform(*tob_args)
            ct_args = [USB_CAM_ID, LANDING_PAD_ID]
            common_time = self.listener.getLatestCommonTime(*ct_args)
            delta_t = (rospy.Time.now() - common_time).to_sec()
            if delta_t < TF_TIMEOUT:
                tlc_args = [LANDING_PAD_ID, USB_CAM_ID, rospy.Time(0)]
                (xm, ym, zm), _ = self.listener.lookupTransform(*tlc_args)
                self.latest_odom_tf = (xo + xm, yo + ym, zm - zo)
            elif len(tags) > 0 and tags[0].id > 0:
                ttl = [str(tags[0].id), LANDING_PAD_ID, rospy.Time(0)]
                (xtl, ytl, ztl), _ = self.listener.lookupTransform(*ttl)
                xt = tags[0].pose.pose.position.x
                yt = tags[0].pose.pose.position.y
                zt = tags[0].pose.pose.position.z
                self.latest_odom_tf = (xo + xtl - xt, yo + ytl - yt,
                                       (ztl + zt) - zo)
        except tf.Exception:
            rospy.loginfo("No AprilTags found")

        self.br.sendTransform(self.latest_odom_tf,
                              (0, 0, 0, 1),
                              rospy.Time.now(),
                              ODOM_ID, MAP_ID)

    def run(self):
        try:
            ct_args = [USB_CAM_ID, LANDING_PAD_ID]
            common_time = self.listener.getLatestCommonTime(*ct_args)
            delta_t = (rospy.Time.now() - common_time).to_sec()
            if delta_t < TF_TIMEOUT:
                tlc_args = [LANDING_PAD_ID, USB_CAM_ID, rospy.Time(0)]
                tob_args = [ODOM_ID, BASE_LINK_ID, rospy.Time(0)]
                (xm, ym, zm), _ = self.listener.lookupTransform(*tlc_args)
                (xo, yo, zo), _ = self.listener.lookupTransform(*tob_args)
                self.latest_odom_tf = (xo + xm, yo + ym, zm - zo)
        except tf.Exception:
            rospy.loginfo("No AprilTags found")

        self.br.sendTransform(self.latest_odom_tf,
                              (0, 0, 0, 1),
                              rospy.Time.now(),
                              ODOM_ID, MAP_ID)
        return self


def main():
    rospy.init_node(NODE_NAME, anonymous=False)
    att = AprilTagsTransformer(30)
    att.start()
    rospy.spin()


if __name__ == "__main__":
    main()

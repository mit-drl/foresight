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

    def apriltags_cb(self, tag_array):
        tags = tag_array.detections
        if len(tags) > 0:
            tag = tags[0]
            ps = self.listener.transformPose("odom", tag.pose)
            pos = ps.pose.position
            quat = ps.pose.orientation
            self.br.sendTransform((-pos.x, -pos.y, 0),
                                  (0, 0, 0, 1),
                                  rospy.Time.now(), "odom", "car/hood_link")


def main():
    rospy.init_node(NODE_NAME, anonymous=False)
    att = AprilTagsTransformer(30)
    att.start()
    rospy.spin()


if __name__ == "__main__":
    main()

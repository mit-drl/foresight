#!/usr/bin/env python

import rospy
import tf


NODE_NAME = "apriltags_pose"
FREQUENCY = 30
LANDING_PAD_ID = "/landing_pad"
USB_CAM_ID = "/usb_cam"
ODOM_ID = "/odom"
MAP_ID = "/map"
BASE_LINK_ID = "/base_link"
TAG_DETECTIONS_TOPIC = "/tag_detections_pose"


class AprilTagsTransformer(object):

    def __init__(self, frequency):
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(frequency)
        self.tf_exs = (tf.LookupException,
                       tf.ConnectivityException,
                       tf.ExtrapolationException)

    def start(self):
        while not rospy.is_shutdown():
            self.run()
            self.rate.sleep()

    def run(self):
        try:
            (xm, ym, zm), rm = self.listener.lookupTransform(
                LANDING_PAD_ID, USB_CAM_ID, rospy.Time(0))
            (xo, yo, zo), ro = self.listener.lookupTransform(
                ODOM_ID, BASE_LINK_ID, rospy.Time(0))
            self.br.sendTransform(
                (xo - xm, yo - ym, zm - zo),
                (0, 0, 0, 1), rospy.Time.now(),
                ODOM_ID, MAP_ID)
        except self.tf_exs:
            print "well fuck"


def main():
    rospy.init_node(NODE_NAME, anonymous=False)
    att = AprilTagsTransformer(30)
    att.start()


if __name__ == "__main__":
    main()

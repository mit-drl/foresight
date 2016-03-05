#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
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
        self.tfl = tf.TransformListener()
        self.rate = rospy.Rate(frequency)
        self.sub = None
        self.trans = (0, 0, 0)
        self.quat = (0, 0, 0, 1)

    def start(self):
        self.sub = rospy.Subscriber(
            TAG_DETECTIONS_TOPIC, AprilTagDetectionArray, self.apriltags_cb)
        self.run()
        return self

    def apriltags_cb(self, tag_array):
        tags = tag_array.detections
        if len(tags) > 0:
            tag = tags[0]
            odom_org = PoseStamped()
            odom_org.pose.orientation.w = 1
            odom_org.header.frame_id = "odom"
            ps = tag.pose
            pos = ps.pose.position
            quat = ps.pose.orientation
            self.br.sendTransform((pos.x, pos.y, pos.z),
                                  (quat.x, quat.y, quat.z, quat.w),
                                  rospy.Time.now(), "car/hood_tag",
                                  "quad/back_camera_link")
            tp = self.tfl.transformPose("car/hood_tag", odom_org)
            pos = tp.pose.position
            quat = tp.pose.orientation
            self.trans = (pos.x, pos.y, pos.z)
            self.quat = (quat.x, quat.y, quat.z, quat.w)

    def run(self):
        while not rospy.is_shutdown():
            self.br.sendTransform(self.trans, self.quat,
                                  rospy.Time.now(),
                                  "odom", "car/hood_link")
            self.rate.sleep()



def main():
    rospy.init_node(NODE_NAME, anonymous=False)
    att = AprilTagsTransformer(30)
    att.start()
    rospy.spin()


if __name__ == "__main__":
    main()

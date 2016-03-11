#!/usr/bin/env python

import rospy
import tf
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler


NODE_NAME = "stabilized_tf_publisher"


class StabilizedTransformer(object):

    def __init__(self, frequency):
        self.br = tf.TransformBroadcaster()
        self.tfl = tf.TransformListener()
        self.rate = rospy.Rate(frequency)

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.tfl.waitForTransform(
                    "quad/base_link", "odom",
                    rospy.Time(), rospy.Duration(0.1))
                tr, quat = self.tfl.lookupTransform(
                    "odom", "quad/base_link", rospy.Time())
                _, _, yaw = euler_from_quaternion(quat)
                quat_yaw = quaternion_from_euler(0, 0, yaw)
                self.br.sendTransform(
                    tr, quat_yaw, rospy.Time.now(),
                    "quad/base_link_stab", "odom")
            except tf.Exception:
                print "TF ERROR"
            self.rate.sleep()


def main():
    rospy.init_node(NODE_NAME, anonymous=False)
    att = StabilizedTransformer(100)
    att.run()
    rospy.spin()


if __name__ == "__main__":
    main()

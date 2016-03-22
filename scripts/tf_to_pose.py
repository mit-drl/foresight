#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped


NODE_NAME = "tf_to_pose"


class TFToPose(object):

    def __init__(self, frequency):
        self.tfl = tf.TransformListener()
        self.rate = rospy.Rate(frequency)
        self.pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped,
                                   queue_size=2)
        self.sub = None
        self.lr = 0.2
        self.pose = PoseStamped()
        self.pose.header.frame_id = "map"
        self.pose.header.seq = 0

    def quat_to_list(self, quat):
        return [quat.x, quat.y, quat.z, quat.w]

    def set_mov(self, obj, var, val):
        setattr(obj, var, (1 - self.lr) * getattr(obj, var) + self.lr * val)
        return self

    def start(self):
        while not rospy.is_shutdown():
            try:
                self.tfl.waitForTransform(
                    "map", "quad/base_link",
                    rospy.Time(), rospy.Duration(0.1))
                tr, quat = self.tfl.lookupTransform(
                    "map", "quad/base_link", rospy.Time())
                self.pose.header.seq += 1
                self.pose.header.stamp = rospy.Time.now()
                self.set_mov(self.pose.pose.position, "x", tr[0])
                self.set_mov(self.pose.pose.position, "y", tr[1])
                self.set_mov(self.pose.pose.position, "z", tr[2])
                self.pose.pose.orientation.x = quat[0]
                self.pose.pose.orientation.y = quat[1]
                self.pose.pose.orientation.z = quat[2]
                self.pose.pose.orientation.w = quat[3]
                self.pub.publish(self.pose)
            except tf.Exception:
                print "TF ERROR"
            self.rate.sleep()


def main():
    rospy.init_node(NODE_NAME, anonymous=False)
    att = TFToPose(100)
    att.start()
    rospy.spin()


if __name__ == "__main__":
    main()

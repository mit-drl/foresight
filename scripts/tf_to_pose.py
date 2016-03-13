#!/usr/bin/env python

import rospy
import tf
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped


NODE_NAME = "tf_to_pose"


class TFToPose(object):

    def __init__(self, frequency):
        self.tfl = tf.TransformListener()
        self.rate = rospy.Rate(frequency)
        self.pub = rospy.Publisher("/mavros/mocap/pose", PoseStamped,
                                   queue_size=2)
        self.pose = PoseStamped()
        self.pose.header.frame_id = "map"
        self.pose.header.seq = 0

    def quat_to_list(self, quat):
        return [quat.x, quat.y, quat.z, quat.w]

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.tfl.waitForTransform(
                    "map", "quad/base_link",
                    rospy.Time(), rospy.Duration(0.1))
                tr, quat = self.tfl.lookupTransform(
                    "map", "car/hood_tag", rospy.Time())
                map_to_tag = euler_from_quaternion(quat)[2]
                tr, quat = self.tfl.lookupTransform(
                    "car/hood_tag", "quad/base_link_yaw", rospy.Time())
                tag_to_quad = euler_from_quaternion(quat)[2]
                yaw = quaternion_from_euler(0, 0, map_to_tag + tag_to_quad)
                tr, quat = self.tfl.lookupTransform(
                    "map", "quad/base_link", rospy.Time())
                self.pose.header.seq += 1
                self.pose.header.stamp = rospy.Time.now()
                self.pose.pose.position.x = tr[0]
                self.pose.pose.position.y = tr[1]
                self.pose.pose.position.z = tr[2]
                self.pose.pose.orientation.x = yaw[0]
                self.pose.pose.orientation.y = yaw[1]
                self.pose.pose.orientation.z = yaw[2]
                self.pose.pose.orientation.w = yaw[3]
                self.pub.publish(self.pose)
            except tf.Exception:
                print "TF ERROR"
            self.rate.sleep()


def main():
    rospy.init_node(NODE_NAME, anonymous=False)
    att = TFToPose(100)
    att.run()
    rospy.spin()


if __name__ == "__main__":
    main()

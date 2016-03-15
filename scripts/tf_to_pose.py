#!/usr/bin/env python

import rospy
import tf
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64


NODE_NAME = "tf_to_pose"


class TFToPose(object):

    def __init__(self, frequency):
        self.tfl = tf.TransformListener()
        self.rate = rospy.Rate(frequency)
        self.pub = rospy.Publisher("/mavros/mocap/pose", PoseStamped,
                                   queue_size=2)
        self.sub = None
        self.yaw_offset = 0
        self.pose = PoseStamped()
        self.pose.header.frame_id = "map"
        self.pose.header.seq = 0

    def start(self):
        self.sub = rospy.Subscriber("/foresight/yaw_offset", Float64,
                                    self.offset_cb)
        self.run()

    def offset_cb(self, fl_yaw):
        self.yaw_offset = fl_yaw.data

    def quat_to_list(self, quat):
        return [quat.x, quat.y, quat.z, quat.w]

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.tfl.waitForTransform(
                    "map", "quad/base_link",
                    rospy.Time(), rospy.Duration(0.1))
                tr, quat = self.tfl.lookupTransform(
                    "map", "quad/base_link", rospy.Time())
                roll, pitch, yaw_abs = euler_from_quaternion(quat)
                print yaw_abs, self.yaw_offset
                yaw_quat = quaternion_from_euler(roll, pitch,
                                                 yaw_abs + self.yaw_offset)
                self.pose.header.seq += 1
                self.pose.header.stamp = rospy.Time.now()
                self.pose.pose.position.x = tr[0]
                self.pose.pose.position.y = tr[1]
                self.pose.pose.position.z = tr[2]
                self.pose.pose.orientation.x = yaw_quat[0]
                self.pose.pose.orientation.y = yaw_quat[1]
                self.pose.pose.orientation.z = yaw_quat[2]
                self.pose.pose.orientation.w = yaw_quat[3]
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

#!/usr/bin/env python

import rospy
import tf
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion


NODE_NAME = "tf_to_pose"


class TFToPose(object):

    def __init__(self, frequency):
        self.tfl = tf.TransformListener()
        self.rate = rospy.Rate(frequency)
        self.pub = rospy.Publisher("/mavros/mocap/pose", PoseStamped,
                                   queue_size=2)
        self.imu = Quaternion()
        self.sub = None
        self.roll = 0
        self.pitch = 0
        self.pose = PoseStamped()
        self.pose.header.frame_id = "map"
        self.pose.header.seq = 0

    def start(self):
        self.sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_cb)
        self.run()

    def quat_to_list(self, quat):
        return [quat.x, quat.y, quat.z, quat.w]

    def imu_cb(self, imu):
        ori = self.quat_to_list(imu.orientation)
        self.roll, self.pitch, _ = euler_from_quaternion(ori)

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.tfl.waitForTransform(
                    "map", "quad/base_link_stab",
                    rospy.Time(), rospy.Duration(0.1))
                tr, quat = self.tfl.lookupTransform(
                    "map", "quad/base_link_stab", rospy.Time())
                _, _, yaw = euler_from_quaternion(quat)
                quat = quaternion_from_euler(self.roll, self.pitch, yaw)
                self.pose.header.seq += 1
                self.pose.header.stamp = rospy.Time.now()
                self.pose.pose.position.x = tr[0]
                self.pose.pose.position.y = tr[1]
                self.pose.pose.position.z = tr[2]
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

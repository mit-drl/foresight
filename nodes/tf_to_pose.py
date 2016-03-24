#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped


NODE_NAME = "tf_to_pose"


class TFToPose(object):

    def __init__(self, frequency, pose_topic, fixed_frame, child_frame):
        self.pose_topic = pose_topic
        self.fixed_frame = fixed_frame
        self.child_frame = child_frame
        self.tfl = tf.TransformListener()
        self.rate = rospy.Rate(frequency)
        self.pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped,
                                   queue_size=2)
        self.pub = rospy.Publisher(pose_topic, PoseStamped,
                                   queue_size=2)
        self.sub = None
        self.lr = 0.1
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
                    self.fixed_frame, self.child_frame,
                    rospy.Time(), rospy.Duration(0.1))
                tr, quat = self.tfl.lookupTransform(
                    self.fixed_frame, self.child_frame, rospy.Time())
                self.pose.header.seq += 1
                self.pose.header.stamp = rospy.Time.now()
                self.set_mov(self.pose.pose.position, "x", tr[0])
                self.set_mov(self.pose.pose.position, "y", tr[1])
                self.set_mov(self.pose.pose.position, "z", tr[2])
                self.set_mov(self.pose.pose.orientation, "x", quat[0])
                self.set_mov(self.pose.pose.orientation, "y", quat[1])
                self.set_mov(self.pose.pose.orientation, "z", quat[2])
                self.set_mov(self.pose.pose.orientation, "w", quat[3])
                self.pub.publish(self.pose)
            except tf.Exception:
                print "TF ERROR"
            self.rate.sleep()


def main():
    rospy.init_node(NODE_NAME, anonymous=False)
    pose_topic = rospy.get_param("~pose_topic", "/mavros/vision_pose/pose")
    fixed_frame = rospy.get_param("~fixed_frame", "map")
    child_frame = rospy.get_param("~child_frame", "quad/base_link")
    att = TFToPose(100, pose_topic, fixed_frame, child_frame)
    att.start()
    rospy.spin()


if __name__ == "__main__":
    main()

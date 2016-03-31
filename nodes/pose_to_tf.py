#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped


NODE_NAME = "pose_to_tf"


class PoseToTF(object):

    def __init__(self, frequency, pose_topic, fixed_frame, child_frame):
        self.pose_topic = pose_topic
        self.fixed_frame = fixed_frame
        self.child_frame = child_frame
        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(frequency)
        self.sub = None
        self.pose = None

    def quat_to_list(self, quat):
        return [quat.x, quat.y, quat.z, quat.w]

    def pose_cb(self, ps):
        self.pose = ps

    def start(self):
        self.sub = rospy.Subscriber(self.pose_topic, PoseStamped,
                                    self.pose_cb)
        while not rospy.is_shutdown():
            if not self.pose is None:
                self.br.sendTransform(
                    [self.pose.pose.position.x,
                     self.pose.pose.position.y,
                     self.pose.pose.position.z],
                    self.quat_to_list(self.pose.pose.orientation),
                    rospy.Time.now(),
                    self.child_frame, self.fixed_frame)
            self.rate.sleep()


def main():
    rospy.init_node(NODE_NAME, anonymous=False)
    pose_topic = rospy.get_param("~pose_topic", "/mavros/vision_pose/pose")
    fixed_frame = rospy.get_param("~fixed_frame", "map")
    child_frame = rospy.get_param("~child_frame", "quad/base_link")
    att = PoseToTF(30, pose_topic, fixed_frame, child_frame)
    att.start()
    rospy.spin()


if __name__ == "__main__":
    main()

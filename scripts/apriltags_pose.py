#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray


NODE_NAME = "apriltags_pose"
POSE_TOPIC = "/mavros/mocap/pose"
TAG_DETECTIONS_TOPIC = "/tag_detections_pose"
pub = None


def apriltags_callback(pose_array):
    if len(pose_array.poses) > 0 and not pub is None:
        ps = PoseStamped()
        ps.header = pose_array.header
        ps.pose = pose_array.poses[0]
        pub.publish(ps)


def main():
    global pub
    rospy.init_node(NODE_NAME, anonymous=False)
    pub = rospy.Publisher(POSE_TOPIC, PoseStamped, queue_size=10)
    rospy.Subscriber(TAG_DETECTIONS_TOPIC, PoseArray, apriltags_callback)
    rospy.spin()


if __name__ == "__main__":
    main()

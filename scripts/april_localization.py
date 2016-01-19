#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray


pub = None


def apriltags_callback(pose_array):
    if len(pose_array.poses) > 0 and not pub is None:
        ps = PoseStamped()
        ps.header = pose_array.header
        ps.pose = pose_array.poses[0]
        pub.publish(ps)


def main():
    global pub
    rospy.init_node("fs_localization", anonymous=False)
    pub = rospy.Publisher("/mavros/mocap/pose", PoseStamped, queue_size=10)
    rospy.Subscriber("/tag_detections_pose", PoseArray, apriltags_callback)
    rospy.spin()


if __name__ == "__main__":
    main()

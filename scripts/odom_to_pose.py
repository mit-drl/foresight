#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

NODE_NAME = "odom_to_pose"
ODOM_TOPIC = "/odom"
POSE_TOPIC_MOCAP = "/mavros/mocap/pose"
POSE_TOPIC_VISION = "/mavros/vision/pose"
ODOM_ID = "odom"
seq = 0


def odom_callback(odom, pubs):
    global seq
    ps = PoseStamped()
    ps.pose = odom.pose.pose
    ps.header.seq = seq
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = ODOM_ID
    seq += 1
    pubs[0].publish(ps)
    pubs[1].publish(ps)


def main():
    rospy.init_node(NODE_NAME, anonymous=False)
    mocap_pub = rospy.Publisher(POSE_TOPIC_MOCAP, PoseStamped, queue_size=10)
    vision_pub = rospy.Publisher(POSE_TOPIC_VISION, PoseStamped, queue_size=10)
    odom_sub = rospy.Subscriber(ODOM_TOPIC, Odometry, odom_callback, (mocap_pub,
                                vision_pub))
    rospy.spin()


if __name__ == "__main__":
    main()

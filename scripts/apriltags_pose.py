#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseArray


NODE_NAME = "apriltags_pose"
POSE_TOPIC = "/foresight/landing_pad/pose"
TAG_DETECTIONS_TOPIC = "/tag_detections_pose"
pub = None
br = None
seq = 0


def covariance_matrix(x_p, y_p, z_p, x_r, y_r, z_r):
    return [x_p, 0, 0, 0, 0, 0,
            0, y_p, 0, 0, 0, 0,
            0, 0, z_p, 0, 0, 0,
            0, 0, 0, x_r, 0, 0,
            0, 0, 0, 0, y_r, 0,
            0, 0, 0, 0, 0, z_r]


def apriltags_callback(pose_array):
    global seq
    if len(pose_array.poses) > 0 and not pub is None:
        ps = PoseWithCovarianceStamped()
        ps.header.stamp = rospy.Time.now()
        ps.header.seq = seq
        ps.header.frame_id = "odom"
        ps.pose.pose.position.x = -pose_array.poses[0].position.x
        ps.pose.pose.position.y = -pose_array.poses[0].position.y
        ps.pose.pose.position.z = pose_array.poses[0].position.z
        ps.pose.covariance = covariance_matrix(1e-2, 1e-2, 1e-2, 0, 0, 0)
        # pub.publish(ps)
        seq += 1


def main():
    global pub, br
    rospy.init_node(NODE_NAME, anonymous=False)
    pub = rospy.Publisher(POSE_TOPIC, PoseWithCovarianceStamped, queue_size=10)
    br = tf.TransformBroadcaster()
    rospy.Subscriber(TAG_DETECTIONS_TOPIC, PoseArray, apriltags_callback)
    rospy.spin()


if __name__ == "__main__":
    main()

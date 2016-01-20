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
new_data = True
listener = None
ps = PoseWithCovarianceStamped()


def covariance_matrix(x_p, y_p, z_p, x_r, y_r, z_r):
    return [x_p, 0, 0, 0, 0, 0,
            0, y_p, 0, 0, 0, 0,
            0, 0, z_p, 0, 0, 0,
            0, 0, 0, x_r, 0, 0,
            0, 0, 0, 0, y_r, 0,
            0, 0, 0, 0, 0, z_r]


def apriltags_callback(pose_array):
    global ps
    if len(pose_array.poses) > 0 and not pub is None:
        ps.header.stamp = rospy.Time.now()
        ps.header.seq += 1
        ps.header.frame_id = "map"
        ps.pose.pose.position.x = -pose_array.poses[0].position.x
        ps.pose.pose.position.y = -pose_array.poses[0].position.y
        ps.pose.pose.position.z = pose_array.poses[0].position.z
        ps.pose.covariance = covariance_matrix(1e-2, 1e-2, 1e-2, 0, 0, 0)

    # try:
    (xm, ym, zm), rm = listener.lookupTransform("/base_link",
                                                "/landing_pad",
                                                rospy.Time(0))
    (xo, yo, zo), ro = listener.lookupTransform("/odom", "/base_link",
                                                rospy.Time(0))
    br.sendTransform((xo - xm,
                        yo - ym,
                        zo - zm),
                        (0, 0, 0, 1), rospy.Time.now(),
                        "odom", "map")
    # except (tf.LookupException, tf.ConnectivityException,
    #         tf.ExtrapolationException):


def main():
    global pub, br, listener
    rospy.init_node(NODE_NAME, anonymous=False)
    pub = rospy.Publisher(POSE_TOPIC, PoseWithCovarianceStamped, queue_size=10)
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    rospy.Subscriber(TAG_DETECTIONS_TOPIC, PoseArray, apriltags_callback)
    rospy.spin()


if __name__ == "__main__":
    main()

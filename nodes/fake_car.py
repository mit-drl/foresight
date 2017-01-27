#!/usr/bin/env python

import roshelper
import rospy
import math
import planar
import tf
from tf.transformations import euler_from_quaternion
from foresight.msg import PolygonArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import PolygonStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


""" Default parameters """
MAP_FRAME = "car/base_link"
QUAD_FRAME = "base_link"
CAMERA_FRAME = "camera_base_link"
SCAN_BREAK_THRESH = 1.0

""" Default topic names """
SCAN_TOPIC = "/scan"
BLIND_SPOT_MARKERS_TOPIC = "/blind_spot_markers"
BLIND_SPOTS_TOPIC = "/blind_spots"
SCAN_POLYGON_TOPIC = "/bounding_poly"

NODE_NAME = "fake_car"
n = roshelper.Node(NODE_NAME, anonymous=False)


@n.entry_point()
class FakeCarPublisher(object):

    def __init__(self):
        self.br = tf.TransformBroadcaster()
        self.tfl = tf.TransformListener()

    @n.main_loop(frequency=30)
    def run(self):
        quat = [-0.0032697421595,
        -0.00402461444292,
        -0.571335132937,
        0.820700479552]

        _, _, yaw = euler_from_quaternion(quat)
        x = -1.34 * math.cos(yaw)
        y = -1.34 * math.sin(yaw)

        self.br.sendTransform((x, y, 0),
            quat,
            rospy.Time.now(),
            "laser_base_link",
            "odom")


if __name__ == "__main__":
    n.start(spin=True)

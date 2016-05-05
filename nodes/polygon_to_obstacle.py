#!/usr/bin/env python

import rospy
import planar
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from teb_local_planner.msg import ObstacleMsg


NODE_NAME = "poly_to_obstacle"
MAP_FRAME = "map"
QUAD_FRAME = "base_link"
OBSTACLES_TOPIC = "/teb/obstacles"
SCAN_POLYGON_TOPIC = "/scan_polygon"
POSE_TOPIC = "/mavros/local_position/pose"
ODOM_TOPIC = "/odom"
CF_STEP = 0.3


class PolygonToObstacle(object):

    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", MAP_FRAME)
        self.quad_frame = rospy.get_param("~quad_frame", QUAD_FRAME)
        self.seq = 0

    def start(self):
        self.obstacle_pub = rospy.Publisher(
            OBSTACLES_TOPIC, ObstacleMsg, queue_size=1)
        self.odom_pub = rospy.Publisher(
            ODOM_TOPIC, Odometry, queue_size=1)
        self.scan_polygon_sub = rospy.Subscriber(
            SCAN_POLYGON_TOPIC, PolygonStamped,
            self.scan_polygon_cb, queue_size=1)
        self.pose_sub = rospy.Subscriber(
            POSE_TOPIC, PoseStamped,
            self.pose_cb, queue_size=1)

    def crow_flies(self, start, end):
        dr = (end - start).normalized()
        cur = start
        while cur.distance_to(end) > CF_STEP:
            cur += CF_STEP * dr
            yield cur

    def scan_polygon_cb(self, ps):
        obs = list()
        pts = ps.polygon.points
        for i in xrange(len(pts)):
            p = planar.Vec2(pts[i - 1].x, pts[i - 1].y)
            q = planar.Vec2(pts[i - 1].x, pts[i - 1].y)
            obs += list(self.crow_flies(p, q))
        self.publish_obstacles(obs)

    def publish_obstacles(self, obs):
        ob = ObstacleMsg()
        ob.header.stamp = rospy.get_rostime()
        ob.header.frame_id = self.map_frame
        ob.header.seq = self.seq
        for p_ob in obs:
            pt = Point32()
            pt.x = p_ob.x
            pt.y = p_ob.y
            sing_poly = PolygonStamped()
            sing_poly.header.frame_id = self.map_frame
            sing_poly.header.stamp = rospy.get_rostime()
            sing_poly.header.seq = self.seq
            sing_poly = PolygonStamped()
            sing_poly.polygon.points.append(pt)
        self.seq += 1
        self.obstacle_pub.publish(ob)

    def pose_cb(self, ps):
        odom = Odometry()
        odom.header.frame_id = self.map_frame
        odom.header.stamp = rospy.get_rostime()
        odom.child_frame_id = self.quad_frame
        odom.pose.pose.position.x = ps.pose.position.x
        odom.pose.pose.position.y = ps.pose.position.y
        odom.pose.pose.position.z = ps.pose.position.z
        odom.pose.pose.orientation.x = ps.pose.orientation.x
        odom.pose.pose.orientation.y = ps.pose.orientation.y
        odom.pose.pose.orientation.z = ps.pose.orientation.z
        odom.pose.pose.orientation.w = ps.pose.orientation.w
        self.odom_pub.publish(odom)


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=False)
    pto = PolygonToObstacle()
    pto.start()
    rospy.spin()

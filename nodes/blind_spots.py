#!/usr/bin/env python

import roshelper
import rospy
import math
import planar
import tf
from foresight.msg import PolygonArray
from sensor_msgs.msg import LaserScan
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

NODE_NAME = "blind_spots"
n = roshelper.Node(NODE_NAME, anonymous=False)


@n.entry_point()
class BlindSpotPublisher(object):

    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", MAP_FRAME)
        self.quad_frame = rospy.get_param("~quad_frame", QUAD_FRAME)
        self.camera_frame = rospy.get_param("~camera_frame", CAMERA_FRAME)
        self.scan_break_thresh = rospy.get_param(
            "~scan_break_thresh", SCAN_BREAK_THRESH)
        self.polys = None
        self.tfl = tf.TransformListener()

    @n.subscriber(SCAN_TOPIC, LaserScan)
    def laser_sub(self, scan):
        pts = self.get_laser_pts(scan)
        self.polys = self.get_blind_polys(pts)
        self.pub_blind_spot_markers(self.polys)
        self.pub_blind_polys(self.polys)
        self.pub_bounding_poly(pts)

    @n.publisher(BLIND_SPOT_MARKERS_TOPIC, MarkerArray)
    def pub_blind_spot_markers(self, polys):
        markers = MarkerArray()
        for i, poly in enumerate(polys):
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = self.map_frame
            marker.type = Marker.LINE_STRIP
            marker.id = i
            marker.action = Marker.ADD
            marker.scale.x = 0.03
            marker.lifetime = rospy.Duration(0.1)
            marker.color.a = 1.0
            marker.color.b = 0.2
            marker.color.r = 0.8
            for v in poly:
                marker.points.append(self.vec_to_point(v))
            marker.points.append(self.vec_to_point(poly[0]))
            markers.markers.append(marker)
        return markers

    @n.publisher(BLIND_SPOTS_TOPIC, PolygonArray)
    def pub_blind_polys(self, b_polys):
        polys = PolygonArray()
        polys.header.frame_id = self.map_frame
        polys.header.stamp = rospy.Time.now()
        for poly in b_polys:
            p_poly = Polygon()
            for v in poly:
                p_poly.points.append(self.vec_to_point(v))
            polys.polygons.append(p_poly)
        return polys

    @n.publisher(SCAN_POLYGON_TOPIC, PolygonStamped)
    def pub_bounding_poly(self, pts):
        poly = PolygonStamped()
        poly.header.stamp = rospy.Time.now()
        poly.header.frame_id = self.map_frame
        for v in pts:
            poly.polygon.points.append(self.vec_to_point(v))
        return poly

    @n.main_loop(frequency=30)
    def run(self):
        pass

    def get_blind_polys(self, pts):
        polys = list()
        for p, q in self.get_laser_breaks(pts):
            dr = (p - q).perpendicular().scaled_to(0.2 * p.distance_to(q))
            polys.append([p, q, q + dr, p + dr])
        return polys

    def get_laser_pts(self, scan):
        pts = list()
        self.tfl.waitForTransform(scan.header.frame_id, self.map_frame,
                                  rospy.Time(), rospy.Duration(1))
        for i, r in enumerate(scan.ranges):
            if r < scan.range_max and r > scan.range_min:
                angle = scan.angle_min + i * scan.angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                pt = self.make_point_stamped(x, y, scan.header.frame_id)
                pt_tf = self.tfl.transformPoint(self.map_frame, pt)
                cur_pt = planar.Vec2(pt_tf.point.x, pt_tf.point.y)
                pts.append(cur_pt)
        return pts

    def get_laser_breaks(self, pts):
        last_pt = pts[0]
        for i, cur_pt in enumerate(pts):
            if last_pt.distance_to(cur_pt) > self.scan_break_thresh:
                yield last_pt, cur_pt
            last_pt = cur_pt

    def vec_to_point(self, vec):
        point = Point()
        point.x = vec.x
        point.y = vec.y
        return point

    def make_point_stamped(self, x, y, frame_id):
        p = PointStamped()
        p.header.frame_id = frame_id
        p.point.x = x
        p.point.y = y
        return p


if __name__ == "__main__":
    n.start(spin=True)

#!/usr/bin/env python

import roshelper
import rospy
import math
import planar
from foresight.msg import PolygonArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import PolygonStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

NODE_NAME = "blind_spots"
n = roshelper.Node(NODE_NAME, anonymous=False)


@n.entry_point()
class BlindSpotPublisher(object):

    def __init__(self, map_frame, scan_break_thresh):
        self.map_frame = map_frame
        self.quad_frame = rospy.get_param("~quad_frame", "base_link")
        self.camera_frame = rospy.get_param("~camera_frame",
                                            "camera_base_link")
        self.scan_break_thresh = scan_break_thresh
        self.polys = None

    @n.subscriber("/scan", LaserScan)
    def laser_sub(self, scan):
        pts = self.get_laser_pts(scan)
        self.polys = self.get_blind_polys(pts)
        self.pub_blind_spot_markers(self.polys)
        self.pub_blind_polys(self.polys)
        self.pub_bounding_poly(pts)

    @n.publisher("/blind_spot_markers", MarkerArray)
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
            marker.color.b = 0.8
            marker.color.r = 0.2
            for v in poly:
                marker.points.append(self.vec_to_point(v))
            marker.points.append(self.vec_to_point(poly[0]))
            markers.markers.append(marker)
        return markers

    @n.publisher("/blind_spots", PolygonArray)
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

    @n.publisher("/bounding_poly", PolygonStamped)
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
        for i, r in enumerate(scan.ranges):
            if r < scan.range_max and r > scan.range_min:
                angle = scan.angle_min + i * scan.angle_increment
                cur_pt = planar.Vec2(r * math.cos(angle), r * math.sin(angle))
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


if __name__ == "__main__":
    n.start(spin=True)

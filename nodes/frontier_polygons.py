#!/usr/bin/env python

import rospy
import math
import tf
import planar
import sensor_msgs.point_cloud2 as pc2
from collections import defaultdict, deque
from foresight.msg import PolygonArray
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Point
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Polygon
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import ChannelFloat32
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker


NODE_NAME = "frontier_publisher"
PC_TOPIC = "/frontier"
MAP_FRAME = "map"
SCAN_TOPIC = "/merged_cloud"
SCAN_POLYGON_TOPIC = "/scan_polygon"
MARKER_TOPIC = "/blind_spots_marker"
BS_TOPIC = "/blind_spots"


class FrontierPublisher(object):

    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", MAP_FRAME)
        self.rate = rospy.Rate(rospy.get_param("~frequency", 30))
        self.scan_break_thresh = rospy.get_param("~scan_break_thresh", 1)
        self.tfl = tf.TransformListener()

    def start(self):
        self.pc_pub = rospy.Publisher(PC_TOPIC, PointCloud, queue_size=1)
        self.blind_spots_pub = rospy.Publisher(
            BS_TOPIC, PolygonArray, queue_size=1)
        self.marker_pub = rospy.Publisher(
            MARKER_TOPIC, MarkerArray, queue_size=1)
        self.polygon_pub = rospy.Publisher(
            SCAN_POLYGON_TOPIC, PolygonStamped, queue_size=1)
        self.scan_sub = rospy.Subscriber(
            SCAN_TOPIC, PointCloud2, self.laser_callback, queue_size=1)

    def laser_callback(self, scan):
        poly = self.pointcloud_to_polygon(scan)
        b_polys = self.get_blind_polygons(scan, poly)
        self.publish_polygon_markers(b_polys)
        self.publish_blind_polygons(b_polys)
        self.publish_planar_polygon(poly)

    def get_blind_polygons(self, scan, poly):
        polys = list()
        for p, q in self.get_laser_breaks(scan, poly):
            dr = (p - q).perpendicular().scaled_to(0.5 * p.distance_to(q))
            polys.append([p, q, q + dr, p + dr])
        return polys

    def get_laser_breaks(self, scan, poly):
        last_pt = None
        pts = pc2.read_points(
            scan, skip_nans=True,
            field_names=("x", "y", "z"))
        for pt in pts:
            ps = self.arr_to_point_stamped(pt, scan.header.frame_id)
            ps_tf = self.tfl.transformPoint(self.map_frame, ps)
            point = planar.Vec2(ps_tf.point.x, ps_tf.point.y)
            if last_pt is None:
                last_pt = point
            elif last_pt.distance_to(point) > self.scan_break_thresh:
                yield last_pt, point
            last_pt = point

    def publish_blind_polygons(self, b_polys):
        polys = PolygonArray()
        polys.header.frame_id = self.map_frame
        polys.header.stamp = rospy.Time.now()
        for poly in b_polys:
            p_poly = Polygon()
            for v in poly:
                p_poly.points.append(self.vec_to_point32(v))
            polys.polygons.append(p_poly)
        self.blind_spots_pub.publish(polys)

    def publish_planar_polygon(self, p_poly):
        poly = PolygonStamped()
        poly.header.stamp = rospy.Time.now()
        poly.header.frame_id = self.map_frame
        for v in p_poly:
            poly.polygon.points.append(self.vec_to_point32(v))
        self.polygon_pub.publish(poly)

    def publish_polygon_markers(self, polys):
        markers = MarkerArray()
        for i, poly in enumerate(polys):
            marker = Marker()
            marker.header.stamp = rospy.Time.now()
            marker.header.frame_id = self.map_frame
            marker.type = Marker.LINE_STRIP
            marker.id = i
            marker.action = Marker.ADD
            marker.scale.x = 0.1
            marker.lifetime = rospy.Duration(1.0)
            marker.color.a = 1.0
            marker.color.b = 0.8
            for v in poly:
                marker.points.append(self.vec_to_point(v))
            marker.points.append(self.vec_to_point(poly[0]))
            markers.markers.append(marker)
        self.marker_pub.publish(markers)

    """ Conversions """

    def vec_to_point32(self, vec):
        point = Point32()
        point.x = vec.x
        point.y = vec.y
        return point

    def vec_to_point(self, vec):
        point = Point()
        point.x = vec.x
        point.y = vec.y
        return point

    def arr_to_point_stamped(self, arr, frame_id):
        ps = PointStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = rospy.Time()
        ps.point.x = arr[0]
        ps.point.y = arr[1]
        return ps

    def arr_to_point_stamped_tf(self, arr, frame_id):
        ps = self.arr_to_point_stamped(arr, frame_id)
        return self.tfl.transformPoint(self.map_frame, ps)

    def arrs_to_vecs_tf(self, arrs, frame_id):
        vecs = list()
        for arr in arrs:
            ps = self.arr_to_point_stamped_tf(arr, frame_id)
            vecs.append(planar.Vec2(ps.point.x, ps.point.y))
        return vecs

    def pointcloud_to_polygon(self, pc):
        pts = pc2.read_points(
            pc, skip_nans=True,
            field_names=("x", "y", "z"))
        vecs = self.arrs_to_vecs_tf(pts, pc.header.frame_id)
        return planar.Polygon(vecs, is_convex=False, is_simple=True)


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=False)
    infopl = FrontierPublisher()
    infopl.start()
    rospy.spin()

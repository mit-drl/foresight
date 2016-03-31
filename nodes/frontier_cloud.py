#!/usr/bin/env python

import rospy
import math
import tf
import planar
import sensor_msgs.point_cloud2 as pc2
from collections import defaultdict, deque
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import ChannelFloat32


NODE_NAME = "frontier_publisher"
PC_TOPIC = "/frontier"
MAP_FRAME = "map"
SCAN_TOPIC = "/merged_cloud_filtered"
NBR_DIST = 0.3
CF_STEP = 0.3
NORMAL_HORIZON = 0.8


class FrontierPublisher(object):

    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", MAP_FRAME)
        self.rate = rospy.Rate(rospy.get_param("~frequency", 30))
        self.scan_break_thresh = rospy.get_param("~scan_break_thresh", 1)
        self.pc_pub = None
        self.scan_sub = None
        self.time_grid = defaultdict(lambda: defaultdict(lambda: 0))
        self.tfl = tf.TransformListener()

    def start(self):
        self.pc_pub = rospy.Publisher(PC_TOPIC, PointCloud, queue_size=1)
        self.scan_sub = rospy.Subscriber(SCAN_TOPIC, PointCloud2,
                                         self.laser_callback, queue_size=1)

    def laser_callback(self, scan):
        self.time_grid.clear()
        poly = self.pointcloud_to_polygon(scan)
        for brk in self.get_laser_breaks(scan, poly):
            self.set_grid_val(brk.x, brk.y, 1)
        self.publish_time_grid()

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
                for bpt in self.horizon_points(last_pt, point, poly):
                    yield bpt
            last_pt = point

    def horizon_points(self, p, q, poly):
        perp = (p - q).perpendicular()
        horizon = perp.scaled_to(NORMAL_HORIZON)
        for bpt in self.crow_flies(p, q):
            for hpt in self.crow_flies(bpt, bpt + horizon):
                if not poly.contains_point(hpt):
                    yield hpt

    def crow_flies(self, start, end):
        dr = (end - start).normalized()
        cur = start
        while cur.distance_to(end) > CF_STEP:
            cur += CF_STEP * dr
            yield cur

    def points_in_poly(self, poly, step):
        q = deque([poly.centroid])
        seen = set([poly.centroid.x, poly.centroid.y])
        nbrs = [(step, 0), (0, step), (step, step),
                (-step, 0), (0, -step), (-step, -step),
                (step, -step), (-step, step)]
        while len(q) > 0 and not rospy.is_shutdown():
            p = q.popleft()
            for nbr in nbrs:
                nbr_t = (p.x + nbr[0], p.y + nbr[1])
                nbr_p = planar.Vec2(*nbr_t)
                if poly.contains_point(nbr_p) and not nbr_t in seen:
                    seen.add(nbr_t)
                    q.append(nbr_p)
            yield p

    def publish_time_grid(self):
        pc = PointCloud()
        ch = ChannelFloat32()
        pc.header.stamp = rospy.Time.now()
        pc.header.frame_id = self.map_frame
        ch.name = "time"
        for x in self.time_grid.keys():
            for y in self.time_grid[x].keys():
                p32 = self.xy_to_point32(x, y)
                pc.points.append(p32)
                ch.values.append(self.time_grid[x][y])
        pc.channels.append(ch)
        self.pc_pub.publish(pc)

    """ Getters and setters for time grid """

    def get_discrete(self, x, y):
        x_hat = NBR_DIST * math.floor(x / NBR_DIST)
        y_hat = NBR_DIST * math.floor(y / NBR_DIST)
        return x_hat, y_hat

    def get_grid_val(self, x, y):
        xp, yp = self.get_discrete(x, y)
        return self.time_grid[xp][yp]

    def set_grid_val(self, x, y, val):
        xp, yp = self.get_discrete(x, y)
        self.time_grid[xp][yp] = val

    """ Conversions """

    def arrs_to_vecs(self, arrs):
        vecs = list()
        for arr in arrs:
            vecs.append(planar.Vec2(arr[0], arr[1]))
        return vecs

    def vec_to_point32(self, vec):
        point = Point32()
        point.x = vec.x
        point.y = vec.y
        return point

    def xy_to_point32(self, x, y):
        point = Point32()
        point.x = x
        point.y = y
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

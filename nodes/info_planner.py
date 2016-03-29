#!/usr/bin/env python

import rospy
import camproj
import math
import tf
import numpy as np
import planar
import time
import scipy.optimize as opt
import sensor_msgs.point_cloud2 as pc2
from tf.transformations import euler_matrix
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from collections import defaultdict, deque
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import ChannelFloat32


NODE_NAME = "info_planner"
POSE_TOPIC = "/mavros/setpoint_position/local"
POSE_SUB_TOPIC = "/mavros/local_position/pose"
PC_TOPIC = "/timegrid"
MAP_TOPIC = "/map"
MAP_FRAME = "map"
QUAD_FRAME = "quad/base_link"
CAM_FRAME = "quad/back_camera_link"
POLYGON_TOPIC = "/projection"
SCAN_TOPIC = "/merged_cloud"
BREAKS_TOPIC = "/breaks_pc"
NBR_DIST = 1
CF_STEP = 0.5


class InfoPlanner(object):

    def __init__(self):
        fov_v = rospy.get_param("~fov_v", math.pi / 2)
        fov_h = rospy.get_param("~fov_h", math.pi / 2)
        self.map_frame = rospy.get_param("~map_frame", MAP_FRAME)
        self.quad_frame = rospy.get_param("~quad_frame", QUAD_FRAME)
        self.camera_frame = rospy.get_param("~camera_frame", CAM_FRAME)
        self.min_alt = rospy.get_param("~min_alt", 1)
        self.max_alt = rospy.get_param("~max_alt", 2)
        self.bound_rel_xy = rospy.get_param("~bound_rel_xy", 1)
        self.scan_break_thresh = rospy.get_param("~scan_break_thresh", 1)
        self.rate = rospy.Rate(rospy.get_param("~frequency", 30))
        self.cam = camproj.CameraProjection(fov_v, fov_h)
        self.pose = None
        self.pub = None
        self.polygon_pub = None
        self.pc_pub = None
        self.scan_sub = None
        self.breaks_pub = None
        self.time_grid = defaultdict(lambda: defaultdict(lambda: -1))
        self.tfl = tf.TransformListener()
        self.start_time = None

    def start(self):
        self.pub = rospy.Publisher(POSE_TOPIC, PoseStamped, queue_size=1)
        self.polygon_pub = rospy.Publisher(POLYGON_TOPIC, PolygonStamped,
                                           queue_size=1)
        self.pc_pub = rospy.Publisher(PC_TOPIC, PointCloud, queue_size=1)
        self.breaks_pub = rospy.Publisher(BREAKS_TOPIC, PointCloud, queue_size=1)
        self.scan_sub = rospy.Subscriber(SCAN_TOPIC, PointCloud2,
                                         self.laser_callback)
        # self.pose_sub = rospy.Subscriber(POSE_SUB_TOPIC, PoseStamped,
        #                                  self.pose_callback)
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            try:
                projection = self.get_current_projection()
                pc = self.update_time_grid(projection)
                self.pc_pub.publish(pc)
                self.publish_polygon(projection)
            except tf.Exception:
                print "TF ERROR"
            self.rate.sleep()

    def pose_callback(self, ps):
        bp = self.find_best_point(ps)
        set_ps = self.state_to_pose(bp)
        self.pub.publish(set_ps)

    def laser_callback(self, scan):
        breaks = self.get_laser_breaks(scan)
        # print len(breaks)
        # rospy.sleep(0.3)

    def get_laser_breaks(self, scan):
        pc = PointCloud()
        ch = ChannelFloat32()
        pc.header.stamp = rospy.Time.now()
        pc.header.frame_id = "map"
        ch.name = "break_id"
        breaks = list()
        last_pt = None
        pts = pc2.read_points(scan, skip_nans=True,
                              field_names=("x", "y", "z"))
        for pt in pts:
            ps = self.arr_to_point_stamped(pt, scan.header.frame_id)
            ps_tf = self.tfl.transformPoint(self.map_frame, ps)
            point = planar.Vec2(ps_tf.point.x, ps_tf.point.y)
            if last_pt is None:
                last_pt = point
            elif last_pt.distance_to(point) > self.scan_break_thresh:
                p32 = self.vec_to_point32(last_pt)
                q32 = self.vec_to_point32(point)
                pc.points.append(p32)
                pc.points.append(q32)
                ch.values.append(1)
                breaks.append((last_pt, point))
                for bpt in self.crow_flies(last_pt, point):
                    pc.points.append(self.vec_to_point32(bpt))
            last_pt = point
        pc.channels.append(ch)
        self.breaks_pub.publish(pc)
        return breaks

    def crow_flies(self, start, end):
        dr = (end - start).normalized()
        cur = start
        while cur.distance_to(end) > CF_STEP:
            cur += CF_STEP * dr
            yield cur

    def find_best_point(self, ps):
        init = self.pose_to_state(ps)
        bounds = [(init[0] - self.bound_rel_xy, init[0] + self.bound_rel_xy),
                  (init[1] - self.bound_rel_xy, init[1] + self.bound_rel_xy),
                  (self.min_alt, self.max_alt), (0, 2 * math.pi)]
        opt_res = opt.minimize(self.objective, init, method="SLSQP",
                               bounds=bounds)
        return opt_res.x

    def objective(self, state):
        projection = self.get_projection(state)
        improvement = 0
        new_time = time.time()
        for p in self.points_in_poly(projection, NBR_DIST):
            improvement += (new_time - self.time_grid[p.x][p.y])
        return -improvement

    def update_time_grid(self, projection):
        pc = PointCloud()
        ch = ChannelFloat32()
        pc.header.stamp = rospy.Time.now()
        pc.header.frame_id = "map"
        ch.name = "time"
        if self.start_time is None:
            self.start_time = time.time()
        t = time.time() - self.start_time
        for p in self.points_in_poly(projection, NBR_DIST):
            self.time_grid[p.x][p.y] = t
            p32 = Point32()
            p32.x = p.x
            p32.y = p.y
            pc.points.append(p32)
            ch.values.append(t)
        pc.channels.append(ch)
        return pc

    def get_relative_pose(self, parent_frame, child_frame):
        self.tfl.waitForTransform(
            parent_frame, child_frame, rospy.Time(),
            rospy.Duration(0.1))
        tr, _ = self.tfl.lookupTransform(
            parent_frame, child_frame, rospy.Time())
        _, quat = self.tfl.lookupTransform(
            parent_frame, child_frame, rospy.Time())
        ps = PoseStamped()
        ps.pose.position.x = -tr[0]
        ps.pose.position.y = -tr[1]
        ps.pose.position.z = -tr[2]
        ps.pose.orientation.x = quat[0]
        ps.pose.orientation.y = quat[1]
        ps.pose.orientation.z = quat[2]
        ps.pose.orientation.w = quat[3]
        return ps

    def get_projection(self, state):
        pose_mq = self.state_to_pose(state)
        pose_qm = self.tfl.transformPose(self.quad_frame, pose_mq)
        pose_cq = self.get_relative_pose(self.quad_frame, self.camera_frame)
        rot_qm, trans_qm = self.pose_to_matrix(pose_qm)
        rot_cq, trans_cq = self.pose_to_matrix(pose_cq)
        projection = self.cam.get_projection(rot_qm, rot_cq,
                                             trans_qm, trans_cq)
        return projection

    def get_current_projection(self):
        pose_qm = self.get_relative_pose(self.map_frame, self.quad_frame)
        pose_cq = self.get_relative_pose(self.quad_frame, self.camera_frame)
        rot_qm, trans_qm = self.pose_to_matrix(pose_qm)
        rot_cq, trans_cq = self.pose_to_matrix(pose_cq)
        projection = self.cam.get_projection(rot_qm, rot_cq,
                                             trans_qm, trans_cq)
        return projection

    def publish_polygon(self, projection):
        poly = PolygonStamped()
        poly.header.stamp = rospy.Time.now()
        poly.header.frame_id = MAP_FRAME
        for v in projection:
            p = Point32()
            p.x = v[0]
            p.y = v[1]
            poly.polygon.points.append(p)
        self.polygon_pub.publish(poly)

    def points_in_poly(self, projection, step):
        vecs = self.arrs_to_vecs(projection)
        poly = planar.Polygon(vecs, is_simple=True, is_convex=True)
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

    def state_to_pose(self, state):
        quat = quaternion_from_euler(0, 0, state[3])
        pose_mq = PoseStamped()
        pose_mq.header.frame_id = self.map_frame
        pose_mq.header.stamp = rospy.Time()
        pose_mq.pose.position.x = state[0]
        pose_mq.pose.position.y = state[1]
        pose_mq.pose.position.z = state[2]
        pose_mq.pose.orientation.x = quat[0]
        pose_mq.pose.orientation.y = quat[1]
        pose_mq.pose.orientation.z = quat[2]
        pose_mq.pose.orientation.w = quat[3]
        return pose_mq

    def pose_to_state(self, pose):
        quat = [pose.pose.orientation.x, pose.pose.orientation.y,
                pose.pose.orientation.z, pose.pose.orientation.z]
        _, _, yaw = euler_from_quaternion(quat)
        pos = pose.pose.position
        state = np.array([pos.x, pos.y, pos.z, yaw])
        return state

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

    def arr_to_point_stamped(self, arr, frame_id):
        ps = PointStamped()
        ps.header.frame_id = frame_id
        ps.point.x = arr[0]
        ps.point.y = arr[1]
        ps.point.z = arr[2]

    def pose_to_matrix(self, ps):
        trans = np.matrix([-ps.pose.position.x,
                           -ps.pose.position.y,
                           ps.pose.position.z]).T
        r, p, y = euler_from_quaternion([ps.pose.orientation.x,
                                         ps.pose.orientation.y,
                                         ps.pose.orientation.z,
                                         ps.pose.orientation.w])
        rot = np.matrix(euler_matrix(-r, -p, y)[:3, :3]).T
        return rot, trans

    def polar_to_cart(self, angle, dist, frame_id):
        rel_point = PointStamped()
        rel_point.header.frame_id = frame_id
        rel_point.point.x = dist * math.cos(angle)
        rel_point.point.y = dist * math.sin(angle)
        abs_point = self.tfl.transformPoint(self.map_frame, rel_point)
        return abs_point.point.x, abs_point.point.y


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=False)
    infopl = InfoPlanner()
    infopl.start()

#!/usr/bin/env python

import rospy
import camproj
import math
import tf
import numpy as np
import planar
import scipy.optimize as opt
import scipy.spatial as spatial
import shapely.geometry as geom
import networkx as nx
import heapq
from tf.transformations import euler_matrix
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from collections import defaultdict, deque
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from foresight.msg import PolygonArray
from geometry import Point
from geometry import SpaceHeapValue


NODE_NAME = "info_planner"
POSE_TOPIC = "/mavros/setpoint_position/local"
POSE_SUB_TOPIC = "/mavros/local_position/pose"
MAP_FRAME = "map"
QUAD_FRAME = "base_link"
CAM_FRAME = "back_camera_link"
POLYGON_TOPIC = "/projection"
FRONTIER_TOPIC = "/blind_spots"
OPT_POLYGON_TOPIC = "/opt_projection"
SCAN_POLYGON_TOPIC = "/scan_polygon"
POSE_ARRAY_TOPIC = "/optimal_path"


class InfoPlanner(object):

    def __init__(self):
        fov_v = rospy.get_param("~fov_v", 0.2 * math.pi)
        fov_h = rospy.get_param("~fov_h", 0.2 * math.pi)
        self.map_frame = rospy.get_param("~map_frame", MAP_FRAME)
        self.quad_frame = rospy.get_param("~quad_frame", QUAD_FRAME)
        self.camera_frame = rospy.get_param("~camera_frame", CAM_FRAME)
        self.min_alt = rospy.get_param("~min_alt", 1)
        self.perc_opt_thresh = rospy.get_param("~optimality_threshold", 0.7)
        self.max_time = rospy.get_param("~max_execution_time", 5.0)
        self.max_speed = rospy.get_param("~max_speed", 1.0)
        self.timeout = rospy.get_param("~timeout", 0.2)
        self.rate = rospy.Rate(rospy.get_param("~frequency", 100))
        self.cam = camproj.CameraProjection(fov_v, fov_h)
        self.pose = None
        self.tree = None
        self.opt_ps = None
        self.poly = None
        self.last_opt = None
        self.tfl = tf.TransformListener()

    def start(self):
        self.pose_pub = rospy.Publisher(POSE_TOPIC, PoseStamped, queue_size=1)
        self.path_pub = rospy.Publisher(
            POSE_ARRAY_TOPIC, PoseArray, queue_size=1)
        self.polygon_pub = rospy.Publisher(
            POLYGON_TOPIC, PolygonStamped, queue_size=1)
        self.opt_polygon_pub = rospy.Publisher(
            OPT_POLYGON_TOPIC, PolygonStamped, queue_size=1)
        self.frontier_sub = rospy.Subscriber(
            FRONTIER_TOPIC, PolygonArray,
            self.frontier_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber(
            POSE_SUB_TOPIC, PoseStamped,
            self.pose_callback, queue_size=1)
        self.scan_polygon_sub = rospy.Subscriber(
            SCAN_POLYGON_TOPIC, PolygonStamped,
            self.scan_polygon_cb, queue_size=1)
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            if not self.opt_ps is None:
                self.pose_pub.publish(self.opt_ps)
            self.rate.sleep()

    def scan_polygon_cb(self, ps):
        arrs = self.points_to_arrs(ps.polygon.points)
        self.poly = geom.Polygon(arrs)

    def pose_callback(self, ps):
        projection = self.get_current_projection()
        self.publish_projection(projection, self.polygon_pub)
        self.pose = ps

    def frontier_callback(self, polys):
        pt_polys = list()
        multi_polygon = None
        for poly in polys.polygons:
            pts = self.points_to_arrs(poly.points)
            geom_poly = geom.Polygon(pts)
            if multi_polygon is None:
                multi_polygon = geom_poly
            else:
                multi_polygon = multi_polygon.union(geom_poly)
        shvs = self.find_path(self.pose, multi_polygon, self.poly, 0.3,
                              self.timeout)
        self.publish_pose_array(shvs)

    def find_best_point(self, ps, polys):
        init = self.pose_to_state(ps)
        options = {"disp": False, "maxiter": None, "maxfev": 20}
        kwargs = {"options": options, "method": "Powell", "args": (polys,)}
        opt_res = opt.minimize(self.objective, init, **kwargs)
        self.last_opt = opt_res.x
        return opt_res.x

    def get_residual_polys(self, pt, yaw, polys):
        state = np.array([pt.x, pt.y, yaw])
        projection = self.get_projection(state)
        proj = np.array(projection).reshape(4, 2)
        proj_poly = geom.Polygon(proj)
        return polys.difference(proj_poly)

    def find_path(self, ps, bs_polys, free_poly, step, timeout):
        pt = self.pose_to_geom_point(ps)
        opt_res = self.find_best_yaw(pt, bs_polys)
        res_polys = self.get_residual_polys(pt, opt_res.x, bs_polys)
        first_value = SpaceHeapValue() \
            .set_point(pt) \
            .set_area(-opt_res.fun) \
            .set_current_time(0) \
            .set_polygons(bs_polys) \
            .set_yaw(opt_res.x)
        hq = [first_value]
        seen = set()
        parents = dict()
        nbrs = [(step, 0), (0, step), (-step, 0), (0, -step)]
        start_time = rospy.get_time()
        while len(hq) > 0 and not rospy.is_shutdown():
            shv = heapq.heappop(hq)
            if rospy.get_time() - start_time >= timeout:
                return self.backtrack_path(parents, shv)
            area = shv.get_area()
            pt = shv.get_point()
            polys = shv.get_polygons()
            ct = shv.get_current_time()
            for nbr in nbrs:
                nbr_p = Point(pt.x + nbr[0], pt.y + nbr[1])
                next_time = ct + pt.distance(nbr_p) / self.max_speed
                cfree = free_poly.contains(nbr_p)
                within_time = next_time < self.max_time
                if cfree and within_time:
                    yaw_res = self.find_best_yaw(nbr_p, polys)
                    res_ps = self.get_residual_polys(nbr_p, yaw_res.x, polys)
                    nbr_shv = SpaceHeapValue() \
                        .set_point(nbr_p) \
                        .set_current_time(next_time) \
                        .set_yaw(yaw_res.x) \
                        .set_polygons(res_ps) \
                        .set_area(-yaw_res.fun)
                    parents[nbr_shv] = shv
                    optimality = 1 - res_ps.area / bs_polys.area
                    if optimality >= self.perc_opt_thresh:
                        return self.backtrack_path(parents, nbr_shv)
                    heapq.heappush(hq, nbr_shv)

    def backtrack_path(self, parents, shv):
        cur = shv
        path = [shv]
        while cur in parents.keys():
            cur = parents[cur]
            path.append(cur)
        return list(reversed(path))

    def find_best_yaw(self, pt, polys):
        init = math.pi
        args = (list(pt), polys)
        options = {"disp": False, "maxiter": None}
        kwargs = {"options": options, "method": "Bounded", "args": args,
                  "bounds": (0, 2 * math.pi)}
        opt_res = opt.minimize_scalar(self.yaw_objective, init, **kwargs)
        return opt_res

    def yaw_objective(self, yaw, state_2d, polys):
        state = np.array(state_2d + [yaw])
        return self.objective(state, polys)

    def objective(self, state, polys):
        obj = 0
        projection = self.get_projection(state)
        proj = np.array(projection).reshape(4, 2)
        poly = geom.Polygon(proj)
        obj -= polys.intersection(poly).area
        return obj

    def sampled_poly_graph(self, poly, step):
        q = deque([poly.centroid, poly.centroid])
        seen = set([(poly.centroid.x, poly.centroid.y)])
        nbrs = [(step, 0), (0, step), (-step, 0), (0, -step)]
        G = nx.Graph()
        while len(q) > 0 and not rospy.is_shutdown():
            p = q.popleft()
            for nbr in nbrs:
                nbr_t = (p.x + nbr[0], p.x + nbr[1])
                nbr_p = geom.Point(*nbr_t)
                if poly.contains(nbr_p) and not nbr_t in seen:
                    seen.add(nbr_t)
                    q.append(nbr_p)
                    G.add_edge((p.x, p.y), nbr_t)
        return G

    def get_relative_pose(self, parent_frame, child_frame):
        self.tfl.waitForTransform(
            parent_frame, child_frame, rospy.Time(),
            rospy.Duration(0.1))
        tr, _ = self.tfl.lookupTransform(
            parent_frame, child_frame, rospy.Time())
        _, quat = self.tfl.lookupTransform(
            child_frame, parent_frame, rospy.Time())
        r, p, y = euler_from_quaternion(quat)
        nquat = quaternion_from_euler(r, p, -y)
        ps = PoseStamped()
        ps.pose.position.x = -tr[0]
        ps.pose.position.y = -tr[1]
        ps.pose.position.z = -tr[2]
        ps.pose.orientation.x = nquat[0]
        ps.pose.orientation.y = nquat[1]
        ps.pose.orientation.z = nquat[2]
        ps.pose.orientation.w = nquat[3]
        return ps

    def get_inverse_pose(self, pose, frame_id):
        pos = pose.pose.position
        quat = pose.pose.orientation
        r, p, y = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        inv_quat = quaternion_from_euler(-r, -p, y)
        inv_pose = PoseStamped()
        inv_pose.header.frame_id = frame_id
        inv_pose.pose.position.x = -pos.x
        inv_pose.pose.position.y = -pos.y
        inv_pose.pose.position.z = -pos.z
        inv_pose.pose.orientation.x = inv_quat[0]
        inv_pose.pose.orientation.y = inv_quat[1]
        inv_pose.pose.orientation.z = inv_quat[2]
        inv_pose.pose.orientation.w = inv_quat[3]
        return inv_pose

    def get_projection(self, state):
        pose_mq = self.state_to_pose(state)
        pose_qm = self.get_inverse_pose(pose_mq, "base_link")
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

    """ Publisher helpers """

    def publish_projection(self, projection, pub):
        poly = PolygonStamped()
        poly.header.stamp = rospy.Time.now()
        poly.header.frame_id = self.map_frame
        for v in projection:
            p = Point32()
            p.x = v[0]
            p.y = v[1]
            poly.polygon.points.append(p)
        pub.publish(poly)

    def publish_pose_array(self, shvs):
        pa = PoseArray()
        pa.header.stamp = rospy.Time.now()
        pa.header.frame_id = self.map_frame
        for shv in shvs:
            pose = Pose()
            pose = self.point_yaw_to_pose(shv.point, shv.yaw)
            pa.poses.append(pose)
        self.path_pub.publish(pa)

    """ Conversions """

    def state_to_pose(self, state):
        quat = quaternion_from_euler(0, 0, state[2])
        pose_mq = PoseStamped()
        pose_mq.header.frame_id = self.map_frame
        pose_mq.header.stamp = rospy.Time()
        pose_mq.pose.position.x = state[0]
        pose_mq.pose.position.y = state[1]
        pose_mq.pose.position.z = self.min_alt
        pose_mq.pose.orientation.x = quat[0]
        pose_mq.pose.orientation.y = quat[1]
        pose_mq.pose.orientation.z = quat[2]
        pose_mq.pose.orientation.w = quat[3]
        return pose_mq

    def point_yaw_to_pose(self, pt, yaw):
        quat = quaternion_from_euler(0, 0, yaw)
        pose_mq = Pose()
        pose_mq.position.x = pt.x
        pose_mq.position.y = pt.y
        pose_mq.position.z = self.min_alt
        pose_mq.orientation.x = quat[0]
        pose_mq.orientation.y = quat[1]
        pose_mq.orientation.z = quat[2]
        pose_mq.orientation.w = quat[3]
        return pose_mq

    def pose_to_state(self, pose):
        quat = [pose.pose.orientation.x, pose.pose.orientation.y,
                pose.pose.orientation.z, pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(quat)
        pos = pose.pose.position
        state = np.array([pos.x, pos.y, yaw])
        return state

    def pose_to_geom_point(self, ps):
        return Point(ps.pose.position.x, ps.pose.position.y)

    def points_to_arrs(self, ps):
        arrs = np.zeros((len(ps), 2))
        for i in xrange(len(ps)):
            arrs[i][0] = ps[i].x
            arrs[i][1] = ps[i].y
        return arrs

    def pose_to_matrix(self, ps):
        trans = np.matrix([-ps.pose.position.x,
                           -ps.pose.position.y,
                           ps.pose.position.z]).T
        r, p, y = euler_from_quaternion([ps.pose.orientation.x,
                                         ps.pose.orientation.y,
                                         ps.pose.orientation.z,
                                         ps.pose.orientation.w])
        rot = np.matrix(euler_matrix(-r, -p, -y)[:3, :3]).T
        return rot, trans


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=False)
    infopl = InfoPlanner()
    infopl.start()
    rospy.spin()

#!/usr/bin/env python

import rospy
import camproj
import math
import tf
import numpy as np
import planar
import scipy.optimize as opt
import scipy.spatial as spatial
from tf.transformations import euler_matrix
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from collections import defaultdict, deque
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud


NODE_NAME = "info_planner"
POSE_TOPIC = "/mavros/setpoint_position/local"
POSE_SUB_TOPIC = "/mavros/local_position/pose"
MAP_FRAME = "map"
QUAD_FRAME = "quad/base_link"
CAM_FRAME = "quad/back_camera_link"
POLYGON_TOPIC = "/projection"
FRONTIER_TOPIC = "/frontier"
OPT_POLYGON_TOPIC = "/opt_projection"
NBR_DIST = 0.5


class InfoPlanner(object):

    def __init__(self):
        fov_v = rospy.get_param("~fov_v", 0.2 * math.pi)
        fov_h = rospy.get_param("~fov_h", 0.2 * math.pi)
        self.map_frame = rospy.get_param("~map_frame", MAP_FRAME)
        self.quad_frame = rospy.get_param("~quad_frame", QUAD_FRAME)
        self.camera_frame = rospy.get_param("~camera_frame", CAM_FRAME)
        self.min_alt = rospy.get_param("~min_alt", 2)
        self.max_alt = rospy.get_param("~max_alt", 4)
        self.bound_rel_xy = rospy.get_param("~bound_rel_xy", 30)
        self.rate = rospy.Rate(rospy.get_param("~frequency", 100))
        self.cam = camproj.CameraProjection(fov_v, fov_h)
        self.pose = None
        self.pose_pub = None
        self.polygon_pub = None
        self.opt_polygon_pub = None
        self.pc_pub = None
        self.tree = None
        self.opt_ps = None
        self.time_grid = defaultdict(lambda: defaultdict(lambda: 0))
        self.tfl = tf.TransformListener()

    def start(self):
        self.pose_pub = rospy.Publisher(POSE_TOPIC, PoseStamped, queue_size=1)
        self.polygon_pub = rospy.Publisher(
            POLYGON_TOPIC, PolygonStamped, queue_size=1)
        self.opt_polygon_pub = rospy.Publisher(
            OPT_POLYGON_TOPIC, PolygonStamped, queue_size=1)
        self.frontier_sub = rospy.Subscriber(
            FRONTIER_TOPIC, PointCloud, self.frontier_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber(
            POSE_SUB_TOPIC, PoseStamped,
            self.pose_callback, queue_size=1)
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            if not self.opt_ps is None:
                self.pose_pub.publish(self.opt_ps)
            self.rate.sleep()

    def pose_callback(self, ps):
        projection = self.get_current_projection()
        self.publish_projection(projection, self.polygon_pub)
        self.pose = ps

    def frontier_callback(self, fpc):
        self.time_grid.clear()
        self.set_grid_with_pc(fpc, 1)
        self.update_tree_with_pc(fpc)
        bp = self.find_best_point(self.pose)
        set_ps = self.state_to_pose(bp)
        opt_projection = self.get_projection(bp)
        self.opt_ps = set_ps
        self.publish_projection(opt_projection, self.opt_polygon_pub)
        self.pose_pub.publish(set_ps)

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

    def find_best_point(self, ps):
        init = self.pose_to_state(ps)
        best_opt = None
        for yaw in np.linspace(0, 2 * math.pi, 4):
            init[2] = yaw
            opt_res = opt.minimize(self.objective, init, args=[init], method="BFGS")
            if best_opt is None or opt_res.fun < best_opt.fun:
                best_opt = opt_res
        return best_opt.x

    def objective(self, state, init):
        obj = 0
        projection = self.get_projection(state)
        poly = self.projection_to_polygon(
            projection, is_convex=True, is_simple=True)
        dist, _ = self.tree.query(state[:2])
        dist_to_quad = pow(state[0] - init[0], 2) + pow(state[1] - init[1], 2)
        for p in self.points_in_poly(poly, NBR_DIST):
            gv = self.get_grid_val(p.x, p.y)
            if gv > 0:
                obj -= 10 * gv
        if obj < 0:
            return obj + dist_to_quad
        else:
            return 10 * dist + dist_to_quad

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
        pose_qm = self.get_inverse_pose(pose_mq, "opt_quad/base_link")
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
        poly.header.frame_id = MAP_FRAME
        for v in projection:
            p = Point32()
            p.x = v[0]
            p.y = v[1]
            poly.polygon.points.append(p)
        pub.publish(poly)

    def publish_planar_polygon(self, p_poly):
        poly = PolygonStamped()
        poly.header.stamp = rospy.Time.now()
        poly.header.frame_id = self.map_frame
        for v in p_poly:
            p = self.vec_to_point32(v)
            poly.polygon.points.append(p)
        self.polygon_pub.publish(poly)

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
        return self

    def set_grid_with_pc(self, pc, val):
        for pt in pc.points:
            self.set_grid_val(pt.x, pt.y, val)
        return self

    def update_tree_with_pc(self, pc):
        arrs = self.point32s_to_arrs(pc.points)
        self.tree = spatial.KDTree(arrs)

    """ Conversions """

    def projection_to_polygon(self, projection, **kwargs):
        vecs = self.arrs_to_vecs(projection)
        return planar.Polygon(vecs, **kwargs)

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

    def pose_to_state(self, pose):
        quat = [pose.pose.orientation.x, pose.pose.orientation.y,
                pose.pose.orientation.z, pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(quat)
        pos = pose.pose.position
        state = np.array([pos.x, pos.y, yaw])
        return state

    def state_to_vec(self, state):
        return planar.Vec2(state[0], state[1])

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

    def point32_to_arr(self, p32):
        return np.array([p32.x, p32.y])

    def point32s_to_arrs(self, p32s):
        arrs = np.zeros((len(p32s), 2))
        for i in xrange(len(p32s)):
            arrs[i][0] = p32s[i].x
            arrs[i][1] = p32s[i].y
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

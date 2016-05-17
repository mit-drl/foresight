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
from tf.transformations import euler_matrix
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from collections import defaultdict, deque
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from foresight.msg import PolygonArray


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


class InfoPlanner(object):

    def __init__(self):
        fov_v = rospy.get_param("~fov_v", 0.2 * math.pi)
        fov_h = rospy.get_param("~fov_h", 0.2 * math.pi)
        self.map_frame = rospy.get_param("~map_frame", MAP_FRAME)
        self.quad_frame = rospy.get_param("~quad_frame", QUAD_FRAME)
        self.camera_frame = rospy.get_param("~camera_frame", CAM_FRAME)
        self.min_alt = rospy.get_param("~min_alt", 1)
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
        for poly in polys.polygons:
            pts = self.points_to_arrs(poly.points)
            pt_polys.append(geom.Polygon(pts))
        bp = self.find_best_point(self.pose, pt_polys)
        set_ps = self.state_to_pose(bp)
        opt_projection = self.get_projection(bp)
        self.opt_ps = set_ps
        self.publish_projection(opt_projection, self.opt_polygon_pub)
        self.pose_pub.publish(set_ps)

    def find_best_point(self, ps, polys):
        init = self.pose_to_state(ps)
        options = {"disp": False, "maxiter": None, "maxfev": 20}
        kwargs = {"options": options, "method": "Powell", "args": (polys,)}
        opt_res = opt.minimize(self.objective, init, **kwargs)
        self.last_opt = opt_res.x
        return opt_res.x

    def objective(self, state, polys):
        obj = 0
        projection = self.get_projection(state)
        proj = np.array(projection).reshape(4, 2)
        poly = geom.Polygon(proj)
        if self.poly.contains(geom.Point(state[0], state[1])):
            for p in polys:
                obj -= p.intersection(poly).area
        else:
            obj = float("inf")
        return obj

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
        poly.header.frame_id = MAP_FRAME
        for v in projection:
            p = Point32()
            p.x = v[0]
            p.y = v[1]
            poly.polygon.points.append(p)
        pub.publish(poly)

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

    def pose_to_state(self, pose):
        quat = [pose.pose.orientation.x, pose.pose.orientation.y,
                pose.pose.orientation.z, pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(quat)
        pos = pose.pose.position
        state = np.array([pos.x, pos.y, yaw])
        return state

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

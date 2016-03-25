#!/usr/bin/env python

import rospy
import camproj
import math
import tf
import numpy as np
from tf.transformations import quaternion_matrix
from collections import defaultdict
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid


NODE_NAME = "info_planner"
POSE_TOPIC = "/mavros/setpoint_position/local"
POSE_SUB_TOPIC = "/mavros/local_position/pose"
MAP_TOPIC = "/map"
MAP_FRAME = "map"
QUAD_FRAME = "quad/base_link"
CAM_FRAME = "quad/back_camera_link"


class InfoPlanner(object):

    def __init__(self):
        fov_v = rospy.get_param("~fov_v", math.pi / 3)
        fov_h = rospy.get_param("~fov_h", math.pi / 3)
        self.map_frame = rospy.get_param("~map_frame", MAP_FRAME)
        self.quad_frame = rospy.get_param("~quad_frame", QUAD_FRAME)
        self.camera_frame = rospy.get_param("~camera_frame", CAM_FRAME)
        self.cam = camproj.CameraProjection(fov_v, fov_h)
        self.pose = None
        self.pub = None
        self.map_sub = None
        self.pose_sub = None
        self.time_grid = defaultdict(lambda: defaultdict(float))
        self.tfl = tf.TransformListener()
        self.rate = rospy.Rate(rospy.get_param("~frequency", 10))

    def start(self):
        self.pub = rospy.Publisher(POSE_TOPIC, PoseStamped, queue_size=1)
        self.map_sub = rospy.Subscriber(MAP_TOPIC, OccupancyGrid,
                                        self.map_callback)
        self.pose_sub = rospy.Subscriber(POSE_SUB_TOPIC, PoseStamped,
                                         self.pose_callback)
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            try:
                map_ori = PoseStamped()
                map_ori.header.frame_id = self.map_frame
                map_ori.pose.orientation.w = 1
                pose_qm = self.tfl.transformPose(self.quad_frame, map_ori)

                quad_ori = PoseStamped()
                quad_ori.header.frame_id = self.quad_frame
                quad_ori.pose.orientation.w = 1
                pose_cq = self.tfl.transformPose(self.camera_frame, quad_ori)

                rot_qm, trans_qm = self.pose_to_matrix(pose_qm)
                rot_cq, trans_cq = self.pose_to_matrix(pose_cq)
                projection = self.cam.get_projection(rot_qm, rot_cq,
                                                     trans_qm, trans_cq)
            except tf.Exception:
                print "TF ERROR"
            self.rate.sleep()

    def pose_to_matrix(self, ps):
        trans = np.matrix([ps.pose.position.x,
                           ps.pose.position.y,
                           ps.pose.position.z]).T
        rot = np.matrix(quaternion_matrix([ps.pose.orientation.x,
                                           ps.pose.orientation.y,
                                           ps.pose.orientation.z,
                                           ps.pose.orientation.w])[:3, :3])
        return rot, trans

    def map_callback(self, og):
        self.og = og

    def pose_callback(self, ps):
        self.ps = ps


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=False)
    infopl = InfoPlanner()
    infopl.start()

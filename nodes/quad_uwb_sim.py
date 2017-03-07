#!/usr/bin/env python

import math
import rospy
import tf
import roshelper
import random

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged

import numpy as np


NODE_NAME = "quad_sim"
n = roshelper.Node(NODE_NAME, anonymous=False)

SET_QUAD_TOPIC = "set_quad_pose"
RANGE_TOPIC = "uwb_range"
ODOM_TOPIC = "odom_topic"
BEBOP_ODOM_TOPIC = "/bebop/odom"
ALT_TOPIC ="/bebop/states/ardrone3/PilotingState/AltitudeChanged"


@n.entry_point()
class Sim(object):

    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.fixed_frame_id = rospy.get_param("~fixed_frame_id", "body")
        self.tag_names = rospy.get_param("tag_names")
        self.listener = tf.TransformListener()
        [self.tag_pos, self.tag_pub] = self.get_tag_transforms()
        #print self.tag_pos

    def get_tag_transforms(self):
        tag_pos = dict()
        tag_pub = dict()
        for tag_name in self.tag_names:
            try:
                self.listener.waitForTransform(self.fixed_frame_id, tag_name,
                                            rospy.Time(), rospy.Duration(1))
                trans, _ = self.listener.lookupTransform(
                    self.fixed_frame_id, tag_name, rospy.Time())
                tag_pos[tag_name] = np.array(trans[:3])
                topic_name = "/{}/{}".format(tag_name, RANGE_TOPIC)
                tag_pub[tag_name] = rospy.Publisher(topic_name, Range, queue_size=1)
            except tf.Exception:
                print "shit"
        return [tag_pos, tag_pub]

    @n.subscriber(SET_QUAD_TOPIC, Odometry)
    def quad_sub(self, odom):
        for tag_name in self.tag_pos.keys():
            r = Range()
            r.header.frame_id = tag_name
            dist = self.distance(self.tag_pos[tag_name],odom.pose.pose.position)
            dist = dist + random.gauss(0, 0.07)
            if dist < 0.1:
                dist = 0.1
            r.range = dist
            r.field_of_view = math.pi * 0.1
            r.min_range = 0
            r.max_range = 300
            self.tag_pub[tag_name].publish(r)
        self.alt_pub(odom)
        self.odom_pub(odom)

    @n.publisher(BEBOP_ODOM_TOPIC, Odometry)
    def odom_pub(self, odom):
        new_odom = Odometry()
        new_odom.header = odom.header
        new_odom.twist = odom.twist
        return new_odom

    @n.publisher(ALT_TOPIC, Ardrone3PilotingStateAltitudeChanged)
    def alt_pub(self, odom):
        alt = Ardrone3PilotingStateAltitudeChanged()
        alt.altitude = odom.pose.pose.position.z + random.gauss(0, 0.03)
        return alt

    def distance(self, uwb, pos):
        return math.sqrt((uwb[0] - pos.x)**2 + (uwb[1] - pos.y)**2 + (uwb[2] - pos.z)**2)

    @n.publisher(SET_QUAD_TOPIC, Odometry)
    def circle_pub(self):
        time = rospy.Time.now().to_sec()
        z = 0.2*math.cos(time) + 1.5 + random.gauss(0, 0.05)
        dz = -0.2*math.sin(time) + random.gauss(0, 0.03)
        x = math.cos(time) + 1 + random.gauss(0, 0.05)
        dx = -math.sin(time) + random.gauss(0, 0.03)
        y = math.sin(time) + random.gauss(0, 0.05)
        dy = math.cos(time) + random.gauss(0, 0.03)
        odom = Odometry()
        odom.header.frame_id = self.fixed_frame_id
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose.position.x = x 
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        odom.twist.twist.linear.x = dx
        odom.twist.twist.linear.y = dy
        odom.twist.twist.linear.z = dz
        return odom

    @n.publisher("/white_prius/odometry/filtered/odom", Odometry)
    def white_prius_odom_pub(self):
        odom = Odometry()
        odom.header.frame_id = self.fixed_frame_id
        return odom


    @n.main_loop(frequency=10)
    def run(self):
        self.circle_pub()
        self.white_prius_odom_pub()

if __name__ == "__main__":
    n.start(spin=True)
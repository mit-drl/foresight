#!/usr/bin/env python

import math
import rospy
import tf
import roshelper
import random
import time

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from std_msgs.msg import Empty

from shapely.geometry import Polygon
#from shapely.geometry import Point
from point import Point

import networkx as nx

NODE_NAME = "rrt_planner"
n = roshelper.Node(NODE_NAME, anonymous=False)

SETPOINT_TOPIC = "/move_base_simple/goal"
ODOM_TOPIC = "/odometry/filtered"
POLYGON_TOPIC = "/bounding_poly"
RRT_TOPIC = "/rrt_path"

@n.entry_point()
class RRT_Planner(object):

    def __init__(self):

        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.fixed_frame_id = rospy.get_param("~fixed_frame_id", "body")

        self.polygon = None
        self.pose = None
        self.setpoint = None
        self.path = None

        self.start_time = 0
        self.waiting_time = 3

        self.graph = nx.Graph()
        self.delta_q = 1
        self.step_size = 0.05

        self.odom_msg = None
        self.polygon_msg = None
        self.setpoint_msg = None

        self.listener = tf.TransformListener()


    @n.publisher(RRT_TOPIC, Path)
    def publish_rrt(self):

        polygon = self.polygon
        setpoint = self.setpoint
        pose = self.pose

        if polygon is not None and pose is not None and setpoint is not None:
            if self.path is not None:
                self.path.remove(0)
                self.path.insert(0, pose)
                self.path.pop()
                self.path.append(setpoint)
                print "repairing path"
                self.path = self.repair2(self.path,polygon,self.step_size, self.delta_q)
            if self.path is None:
                print "starting path afresh"
                self.graph = self.make_rrt(polygon,pose,setpoint,self.step_size,self.delta_q,1000)
                self.path = self.path_from_graph(self.graph,pose,setpoint)

            path = Path()
            path.header.frame_id = self.fixed_frame_id
            path.header.stamp = rospy.Time.now()

            for point in self.path:
                x = point.x
                y = point.y
                new_pose = PoseStamped()
                new_pose.header.frame_id = self.fixed_frame_id
                new_pose.pose.position.x = x
                new_pose.pose.position.y = y
                new_pose.pose.position.z = 1.5
                path.poses.append(new_pose)

            self.publish_setpoint_pose(path)
            return path

    @n.publisher("/setpoint_pose", PoseStamped)
    def publish_setpoint_pose(self, path):
            return path.poses[1]

    def path_from_graph(self,graph,start,end):
        try:
            path = nx.shortest_path(self.graph, source=start,target=end)
        except nx.NetworkXError:
            print "Error converting graph into path"
            path = []
        return path

    def repair2(self,path,polygon,step_size,delta_q):
        for each_point in path:
            point = Point(each_point)
            if polygon.contains(point) is False:
                return None
        i = 0
        while i < len(path)-1:
            start = Point(path[i])
            end = Point(path[i+1])
            if self.is_there_collision(polygon,end,start,step_size,start.distance(end)):
                return None
            i = i + 1
        return path

    def repair(self,path,polygon,step_size,delta_q):
        def repair_nodes(start, index):
            print "repair nodes"
            if index > len(path) - 1:
                return [index, None]
            end = Point(path[index])
            print "does polygon contain x: %f y: %f  ?" % (end.x, end.y)
            if polygon.contains(end):
                print "yes"
                if self.is_there_collision(polygon, end, start, step_size, start.distance(end)):
                    print "IT HAPPENED"
                    # need to repair link between start and next points
                    graph = self.make_rrt(polygon,start,end,step_size,delta_q/4.0,100)
                    print "RRT DONE"
                    new_path = self.path_from_graph(graph,start,end)
                    if len(new_path) == 0:
                        return repair_nodes(start, index+1)
                    else:
                        return [index, new_path]
                else:
                    #print "no collisions they say between x1: %f y1: %f and x2: %f and y2: %f" % (start.x,start.y,end.x,end.y)
                    new_path = [start, end]
                    return [index, []]
            else:
                print "no"
                return repair_nodes(start, index+1)

        repaired_path = path

        i = 1
        while i < len(path):
            print "start is %d" % i
            start_point = Point(path[i])

            [i, new_path] = repair_nodes(start_point,i)
            if new_path is None:
                print "none"
                return None
            else:
                #[i, new_path] = repair_info
                print new_path
                if len(new_path) > 0:
                    new_path.pop(0)
                    new_path.pop()
                    start_index = repaired_path.index(start_point) + 1
                    for new_point in new_path:
                        repaired_path.insert(start_index, new_point)
                        start_index = start_index + 1

            i = i + 1

        return repaired_path

    def make_rrt(self, polygon, start, target, step_size, delta_q, max_k):
        k = 0
        graph = nx.Graph()
        graph.add_node(start)
        unfinished = True
        while k < max_k and unfinished:
            (minx, miny, maxx, maxy) = polygon.bounds

            rand_x = random.uniform(minx, maxx)
            rand_y = random.uniform(miny, maxy)

            q_rand = Point(rand_x, rand_y)

            q_near = self.find_nearest(q_rand, graph)
            q_new = self.new_conf(q_near, q_rand, delta_q)

            if polygon.contains(q_new):
                print "it contains q_new"
                if self.is_there_collision(polygon, q_new, q_near, step_size, delta_q) is False:
                    #print "adding point x: %f y: %f" % (q_new.x, q_new.y)
                    graph.add_node(q_new)
                    graph.add_edge(q_near, q_new, weight=q_near.distance(q_new))

                    if self.attempt_to_complete(polygon, q_new, target, step_size):
                        graph.add_node(target)
                        graph.add_edge(q_new, target, weight = q_new.distance(target))
                        unfinished = False
                    k = k + 1

            #else:
                #print "point x: %f y: %f was not in polygon" % (q_new.x, q_new.y)

        return graph

    def attempt_to_complete(self, polygon, q_new, setpoint, step_size):
        if self.is_there_collision(polygon, setpoint, q_new, step_size, q_new.distance(setpoint)) is False:
            return True
        return False

    # return TRUE if there is a collision
    def is_there_collision(self, polygon, q_new, q_near, step_size, delta_q):
        step = step_size
        while step <= delta_q:
            q_check = self.new_conf(q_near, q_new,step)
            if polygon.contains(q_check) is False:
                return True
            step = step + step_size
        return False

    def new_conf(self, q_near, q_rand, delta_q):
        diff_x = q_rand.x - q_near.x
        diff_y = q_rand.y - q_near.y
        dist = q_rand.distance(q_near)
        new_x = q_near.x + delta_q*(diff_x/dist)
        new_y = q_near.y + delta_q*(diff_y/dist)

        return Point(new_x,new_y)

    def find_nearest(self, q_rand, graph):
        q_near = None
        for point in graph.nodes():
            if q_near is None:
                q_near = point
            elif q_rand.distance(point) < q_rand.distance(q_near):
                q_near = point
        return q_near

    @n.subscriber(POLYGON_TOPIC, PolygonStamped)
    def polygon_sub(self, poly):
        self.polygon_msg = poly
        raw_poly = poly.polygon.points
        points = []
        for point in raw_poly:
            x = point.x
            y = point.y
            points.append([x,y])
        self.polygon = Polygon(points)

    @n.subscriber(SETPOINT_TOPIC, PoseStamped)
    def setpoint_sub(self, ps):
        try:
            self.listener.waitForTransform(ps.header.frame_id, self.fixed_frame_id,
                                            rospy.Time(), rospy.Duration(1))
            ps_tf = self.listener.transformPose(self.fixed_frame_id, ps)

            self.setpoint_msg = ps_tf
            self.setpoint = Point(ps_tf.pose.position.x, ps_tf.pose.position.y)
        except tf.Exception:
            rospy.logerr("Setpoint sub tf error")


    @n.subscriber(ODOM_TOPIC, Odometry)
    def odom_sub(self, odom):
        try:
            ps = PoseStamped()
            ps.header.frame_id = odom.header.frame_id
            ps.pose = odom.pose.pose
            self.listener.waitForTransform(ps.header.frame_id, self.fixed_frame_id,
                                            rospy.Time(), rospy.Duration(1))
            ps_tf = self.listener.transformPose(self.fixed_frame_id, ps)


            self.odom_msg = ps_tf
            self.pose = Point(ps_tf.pose.position.x, ps_tf.pose.position.y)
        except tf.Exception:
            rospy.logerr("Odom sub tf error")

    @n.main_loop(frequency=30)
    def run(self):
        if self.setpoint is not None and self.pose is not None:
            self.publish_rrt()


if __name__ == "__main__":
    n.start(spin=True)

#!/usr/bin/env python

import roshelper
import tf
import math
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
import numpy as np
from foresight.msg import PoseArrayWithTimes
from foresight.msg import PoseStampedWithTime

n = roshelper.Node("waypoint_controller", anonymous=False)

@n.entry_point()
class WaypointController(object):

    def __init__(self):
        self.frame_id = rospy.get_param("frame_id", "base_link")
        self.listener = tf.TransformListener()
        self.x = 0
        self.y = 0
        self.z = 0
        self.yaw = 0
        self.current_pose = Pose()
        self.waypoints = None
        self.wait_times = []
        self.prev_traj = None
        self.traj = None
        self.index = 0
        self.min_dist = 0.15
        self.step_size = 10
        self.made_target = False
        self.start_time = 0.0
        self.fixed_frame_id = None
        # min yaw distance is from the sin value of 18 degrees
        self.min_yaw_dist = abs(math.sin(math.pi/10.0))

    @n.publisher("/setpoint_pose", PoseStamped)
    def pub_setpoint(self):
        goal = self.traj.pose_array.poses[self.index]
        time = self.traj.wait_times[self.index]

        dist = self.dist([self.x,self.y,self.z], [goal.position.x,goal.position.y,goal.position.z])
        yaw_dist = abs(math.sin(self.yaw - self.yaw_from_pose(goal)))
        #print "distance: %f" % dist
        #print "yaw dist: %f" % yaw_dist
        # if you are at the last point in the trajectory, do not
        # increase the index!
        if dist < self.min_dist and yaw_dist < self.min_yaw_dist and self.index < len(self.traj.pose_array.poses) - 1:
            if self.made_target == False:
                self.made_target = True
                self.start_time = rospy.Time.now()
        if self.made_target:
            if rospy.Time.now() - self.start_time > rospy.Duration(time):
                self.made_target = False
                if self.index < len(self.traj.pose_array.poses) - 1:
                    self.index = self.index + 1
                    goal = self.traj.pose_array.poses[self.index]

        ps = PoseStamped()
        ps.header.frame_id = self.fixed_frame_id
        ps.pose = goal
        return ps

    @n.publisher("/trajectory", PoseArray)
    def pub_traj(self):
        return self.traj.pose_array

    @n.publisher("/trajectory_with_times", PoseArrayWithTimes)
    def pub_traj_times(self):
        return self.traj

    def yaw_from_pose(self,pose):
        quat = self.quat_to_list(pose.orientation)
        euler = tf.transformations.euler_from_quaternion(quat)
        yaw = euler[2]
        return yaw

    def dist(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)

    @n.subscriber("/waypoints", PoseArrayWithTimes)
    def traj_sub(self, waypoints):
        if not self.waypoints == waypoints.pose_array.poses:
            self.waypoints = waypoints.pose_array.poses
            self.fixed_frame_id = waypoints.pose_array.header.frame_id
            self.wait_times = list(waypoints.wait_times)
            self.index = 0
            self.traj = self.process_waypoints(self.waypoints, self.wait_times)

    def process_waypoints(self, waypoints, wait_times):
        prev = None
        traj = []
        times = []
        new_waypoints = []
        wait_times = [0.1] + wait_times
        waypoints = [self.current_pose] + waypoints

        for k in range(len(waypoints)):
            waypoint = waypoints[k]
            if k == 0:
                traj.append(waypoint)
                times.append(wait_times[k])
                prev = waypoint
            else:
                wp1 = prev.position
                wp2 = waypoint.position
                p1 = np.array([wp1.x, wp1.y, wp1.z])
                p2 = np.array([wp2.x, wp2.y, wp2.z])
                dist = np.linalg.norm(p2 - p1)
                if dist >= self.step_size:
                    num_points = int(dist/self.step_size)
                    diff = p2 - p1
                    check_if_waypoint_is_reached = waypoint.position
                    for i in range(1,num_points):
                        new_point = Pose()
                        new_point.orientation = waypoint.orientation
                        #np.arange(0,dist,0.1).tolist()
                        newp = p1 + diff*float(i)/float(num_points)
                        new_point.position.x = newp[0]
                        new_point.position.y = newp[1]
                        new_point.position.z = newp[2]
                        check_if_waypoint_is_reached = new_point.position
                        traj.append(new_point)
                        if new_point.position == waypoint.position:
                            times.append(wait_times[k])
                        else:
                            times.append(0)
                    if not check_if_waypoint_is_reached == waypoint.position:
                        traj.append(waypoint)
                        times.append(wait_times[k])
                else:
                    traj.append(waypoint)
                    times.append(wait_times[k])
                prev = waypoint
        new_trajectory = PoseArrayWithTimes()
        new_trajectory.pose_array.header.frame_id = self.fixed_frame_id
        new_trajectory.pose_array.poses = traj
        new_trajectory.wait_times = times
        return new_trajectory

    @n.subscriber("/odometry/filtered", Odometry)
    def odom_sub(self, odom):
        ps = PoseStamped()
        ps.header = odom.header
        ps.pose = odom.pose.pose
        # self.listener.waitForTransform(ps.header.frame_id, self.fixed_frame_id,
        #                             rospy.Time(), rospy.Duration(1))
        # ps = self.listener.transformPose(self.fixed_frame_id, ps)
        self.x = ps.pose.position.x
        self.y = ps.pose.position.y
        self.z = ps.pose.position.z

        quat = ps.pose.orientation
        quat = self.quat_to_list(quat)
        euler = tf.transformations.euler_from_quaternion(quat)
        self.yaw = euler[2]

        self.current_pose = ps.pose

    def quat_to_list(self, quat):
        return [quat.x, quat.y, quat.z, quat.w]

    @n.main_loop(frequency=30)
    def run(self):
        if not self.traj == None:
            self.pub_setpoint()
            self.pub_traj()
            self.pub_traj_times()
            self.prev_traj = self.traj


if __name__ == "__main__":
    n.start(spin=True)

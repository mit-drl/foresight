#!/usr/bin/env python

import roshelper
import tf
import math
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray

n = roshelper.Node("waypoint_test_node", anonymous=False)

@n.entry_point()
class WaypointController(object):

    def __init__(self):
        self.frame_id = rospy.get_param("frame_id", "base_link")
        self.listener = tf.TransformListener()
        self.x = 0
        self.y = 0
        self.z = 0
        self.yaw = 0
        self.traj = None
        self.index = 0
        self.min_dist = 0.25
        # min yaw distance is from the sin value of 18 degrees
        self.min_yaw_dist = abs(math.sin(math.pi/10.0))

    @n.publisher("/setpoint_pose", PoseStamped)
    def pub_setpoint(self):
        goal = self.traj[self.index]
        
        dist = self.dist([self.x,self.y,self.z], [goal.position.x,goal.position.y,goal.position.z])
        yaw_dist = abs(math.sin(self.yaw - self.yaw_from_pose(goal)))
        print "distance: %f" % dist
        #print "yaw dist: %f" % yaw_dist
        # if you are at the last point in the trajectory, do not
        # increase the index!
        if dist < self.min_dist and yaw_dist < self.min_yaw_dist and self.index < len(self.traj) - 1:
            self.index = self.index + 1
            goal = self.traj[self.index]

        ps = PoseStamped()
        ps.header.frame_id = "odom"
        ps.pose = goal
        return ps

    def yaw_from_pose(self,pose):
        quat = self.quat_to_list(pose.orientation)
        euler = tf.transformations.euler_from_quaternion(quat)
        yaw = euler[2]
        return yaw

    def dist(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)

    @n.subscriber("/trajectory", PoseArray)
    def traj_sub(self, traj):
        if not self.traj == traj.poses:
            self.traj = traj.poses
            self.index = 0

    @n.subscriber("/bebop/odom", Odometry)
    def odom_sub(self, odom):
        try:
            ps = PoseStamped()
            ps.header = odom.header
            ps.pose = odom.pose.pose
            self.listener.waitForTransform(ps.header.frame_id, self.frame_id,
                                           rospy.Time(), rospy.Duration(1))
            ps_tf = self.listener.transformPose(self.frame_id, ps)
            self.x = ps.pose.position.x
            self.y = ps.pose.position.y
            self.z = ps.pose.position.z

            quat = ps.pose.orientation
            quat = self.quat_to_list(quat)
            euler = tf.transformations.euler_from_quaternion(quat)
            self.yaw = euler[2]

        except:
            print "tf error"

    def quat_to_list(self, quat):
        return [quat.x, quat.y, quat.z, quat.w]

    @n.main_loop(frequency=30)
    def run(self):
        if not self.traj == None:
            self.pub_setpoint()


if __name__ == "__main__":
    n.start(spin=True)

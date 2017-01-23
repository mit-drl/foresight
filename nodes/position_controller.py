#!/usr/bin/env python

import roshelper
import math
import rospy
import tf
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

NODE_NAME = "position_controller"
n = roshelper.Node(NODE_NAME, anonymous=False)


@n.entry_point()
class PositionController(object):

    def __init__(self):
        self.frame_id = rospy.get_param("frame_id", "base_link")
        self.listener = tf.TransformListener()
        self.efforts = [0, 0, 0]
        self.pose = Pose()
        self.setpoint = Pose()
        self.topics = {"/pid_x/control_effort": 0,
                       "/pid_y/control_effort": 1,
                       "/pid_z/control_effort": 2}

    @n.publisher("/bebop/cmd_vel", Twist)
    def publish_cmd_vel(self, vx, vy, vz):
        vel = Twist()
        # if vx > 0.03:
        #     vel.linear.x = vx
        # if vy > 0.03:
        #     vel.linear.y = vy
        # if vz > 0.03:
        #     vel.linear.z = vz
        if self.dist_to_goal() > 0.1:
            vel.linear.x = vx
            vel.linear.y = vy
            vel.linear.z = vz
        return vel

    @n.publisher(Float64)
    def float_pub(self, fl):
        return fl

    @n.subscriber("/pid_x/control_effort", Float64)
    @n.subscriber("/pid_y/control_effort", Float64)
    @n.subscriber("/pid_z/control_effort", Float64)
    def effort_sub(self, effort, topic_name):
        self.efforts[self.topics[topic_name]] = effort.data

    @n.subscriber("/bebop/odom", Odometry)
    def odom_sub(self, odom):
        self.float_pub(odom.pose.pose.position.x).publish("/pid_x/state")
        self.float_pub(odom.pose.pose.position.y).publish("/pid_y/state")
        self.float_pub(odom.pose.pose.position.z).publish("/pid_z/state")
        self.pose = odom.pose.pose

    @n.subscriber("/setpoint_pose", PoseStamped)
    def setpoint_sub(self, ps):
        # print ps
        self.setpoint = ps.pose
        ps_tf = self.listener.transformPose(self.frame_id, ps)
        print ps_tf
        self.float_pub(ps.pose.position.x).publish("/pid_x/setpoint")
        self.float_pub(ps.pose.position.y).publish("/pid_y/setpoint")
        self.float_pub(ps.pose.position.z).publish("/pid_z/setpoint")

    def dist_to_goal(self):
        pos = self.pose.position
        spos = self.setpoint.position
        x_dist = pow(pos.x - spos.x, 2)
        y_dist = pow(pos.y - spos.y, 2)
        z_dist = pow(pos.z - spos.z, 2)
        return math.sqrt(x_dist + y_dist + z_dist)

    @n.main_loop(frequency=30)
    def run(self):
        self.publish_cmd_vel(*self.efforts)


if __name__ == "__main__":
    n.start(spin=True)

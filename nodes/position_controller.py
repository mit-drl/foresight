#!/usr/bin/env python

import roshelper
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

NODE_NAME = "position_controller"
n = roshelper.Node(NODE_NAME, anonymous=False)


@n.entry_point()
class PositionController(object):

    def __init__(self):
        self.frame_id = rospy.get_param("frame_id", "odom")
        self.efforts = [0, 0, 0]
        self.topics = {"/pid_x/control_effort": 0,
                       "/pid_y/control_effort": 1,
                       "/pid_z/control_effort": 2}

    @n.publisher("/bebop/cmd_vel", Twist)
    def publish_cmd_vel(self, vx, vy, vz):
        vel = Twist()
        if vx > 0.05:
            vel.linear.x = vx
        if vy > 0.05:
            vel.linear.y = vy
        if vz > 0.05:
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

    @n.subscriber("/setpoint_pose", PoseStamped)
    def setpoint_sub(self, ps):
        self.float_pub(ps.pose.position.x).publish("/pid_x/setpoint")
        self.float_pub(ps.pose.position.y).publish("/pid_y/setpoint")
        self.float_pub(ps.pose.position.z).publish("/pid_z/setpoint")

    @n.main_loop(frequency=30)
    def run(self):
        self.publish_cmd_vel(*self.efforts)


if __name__ == "__main__":
    n.start(spin=True)

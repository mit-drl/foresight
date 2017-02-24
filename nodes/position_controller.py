#!/usr/bin/env python

import roshelper
import math
import rospy
import tf
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from foresight.msg import ForesightState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

NODE_NAME = "position_controller"
n = roshelper.Node(NODE_NAME, anonymous=False)


@n.entry_point()
class PositionController(object):

    def __init__(self):
        self.frame_id = rospy.get_param("frame_id", "base_link")
        self.listener = tf.TransformListener()
        self.enabled = False
        self.efforts = [0, 0, 0, 0]
        self.pose = Pose()
        self.setpoint = None
        self.setpoint_timeout = 0.3
        self.last_setpoint_time = None
        self.topics = {"/pid_x/control_effort": 0,
                       "/pid_y/control_effort": 1,
                       "/pid_z/control_effort": 2,
                       "/pid_yaw/control_effort": 3}
        self.land = False
        self.set_yaw = 0

    @n.publisher("/setpoint_vel", Twist)
    def publish_cmd_vel(self, vx, vy, vz, vyaw):
        vel = Twist()
        if self.dist_to_goal() > 0.15:
            vel.linear.x = vx
            vel.linear.y = vy
            vel.linear.z = vz
            self.made_target = True
            self.start_time = rospy.Time.now()

            if abs(self.yaw - self.set_yaw) > math.pi:
                vel.angular.z = -vyaw
            else:
                vel.angular.z = vyaw
        return vel

    @n.publisher(Float64)
    def float_pub(self, fl):
        return fl

    @n.subscriber("/pid_x/control_effort", Float64)
    @n.subscriber("/pid_y/control_effort", Float64)
    @n.subscriber("/pid_z/control_effort", Float64)
    @n.subscriber("/pid_yaw/control_effort", Float64)
    def effort_sub(self, effort, topic_name):
        self.efforts[self.topics[topic_name]] = effort.data

    @n.subscriber("/odometry/filtered", Odometry)
    def odom_sub(self, odom):
        ps = PoseStamped()
        ps.header.frame_id = odom.header.frame_id
        ps.pose = odom.pose.pose
        self.listener.waitForTransform(ps.header.frame_id, self.frame_id,
                                       rospy.Time(), rospy.Duration(1))
        ps_tf = self.listener.transformPose(self.frame_id, ps)
        self.float_pub(ps_tf.pose.position.x).publish("/pid_x/state")
        self.float_pub(ps_tf.pose.position.y).publish("/pid_y/state")
        self.float_pub(ps_tf.pose.position.z).publish("/pid_z/state")

        quat = ps.pose.orientation
        quat = self.quat_to_list(quat)
        euler = tf.transformations.euler_from_quaternion(quat)
        yaw = euler[2]
        self.yaw = yaw
        self.float_pub(yaw).publish("/pid_yaw/state")
        self.pose = odom.pose.pose

    def quat_to_list(self, quat):
        return [quat.x, quat.y, quat.z, quat.w]

    def yaw_from_tf(self, ps_tf):
        quat = self.quat_to_list(ps_tf.pose.orientation)
        euler = tf.transformations.euler_from_quaternion(quat)
        yaw = euler[2]
        return yaw

    @n.subscriber("/setpoint_pose", PoseStamped)
    def setpoint_sub(self, ps):
        self.last_setpoint_time = rospy.Time.now()
        self.setpoint = ps.pose
        ps_tf = self.listener.transformPose(self.frame_id, ps)
        self.float_pub(ps_tf.pose.position.x).publish("/pid_x/setpoint")
        self.float_pub(ps_tf.pose.position.y).publish("/pid_y/setpoint")
        self.float_pub(ps_tf.pose.position.z).publish("/pid_z/setpoint")
        yaw = self.yaw_from_tf(ps)
        self.set_yaw = yaw
        self.float_pub(yaw).publish("/pid_yaw/setpoint")

    def dist_to_goal(self):
        if not self.setpoint is None:
            pos = self.pose.position
            spos = self.setpoint.position
            x_dist = pow(pos.x - spos.x, 2)
            y_dist = pow(pos.y - spos.y, 2)
            z_dist = pow(pos.z - spos.z, 2)
            return math.sqrt(x_dist + y_dist + z_dist)
        else:
            return 0

    @n.subscriber("/bebop/land", Empty)
    def on_land(self, empty):
        self.land = True

    @n.subscriber("/state", ForesightState)
    def update_controller_enabled(self, fs):
        self.enabled = not (fs.state == ForesightState.JOYSTICK)

    @n.main_loop(frequency=100)
    def run(self):
        if not self.land:
            if not self.setpoint is None:
                dt = (rospy.Time.now() - self.last_setpoint_time).to_sec()
                if dt < self.setpoint_timeout and self.enabled:
                    self.publish_cmd_vel(*self.efforts)


if __name__ == "__main__":
    n.start(spin=True)

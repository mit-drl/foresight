#!/usr/bin/env python

import sys
import copy
import rospy
import roshelper
import moveit_commander
import tf
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from moveit_msgs.msg import RobotState
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray


NODE_NAME = "moveit_planner"
n = roshelper.Node(NODE_NAME, anonymous=False)

POSE_TARGET_TOPIC = "/setpoint_goal"
JOINT_STATE_TOPIC = "/joint_state"
ODOM_TOPIC = "/odometry_filtered"
PATH_TOPIC = "/moveit_path"
POSE_ARRAY_TOPIC = "/moveit_pose_array"

@n.entry_point()
class MoveItPlanner(object):

    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.fixed_frame_id = rospy.get_param("~fixed_frame_id", "body")

        self.robot = moveit_commander.RobotCommander("moveit/robot_description")
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = self.robot.quad_body
        #self.group = moveit_commander.MoveGroupCommander("quad_body")
        self.display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)

        print "============ Waiting for RVIZ..."
        rospy.sleep(7)
        print "============ Starting tutorial "

        self.listener = tf.TransformListener()

        self.pose_target = []

        self.plan = None

        self.ticker = 0

        self.current_state = None


        #rospy.on_shutdown(self.turn_off_moveit)

    @n.subscriber(ODOM_TOPIC, Odometry)
    def odom_sub(self, odom):
        position = odom.pose.pose.position

        transform = Transform()
        translation = Vector3()
        translation.x = position.x
        translation.y = position.y
        translation.z = position.z
        transform.translation = translation

        self.current_state = RobotState()
        self.current_state.multi_dof_joint_state.header.frame_id = odom.header.frame_id
        self.current_state.multi_dof_joint_state.header.stamp = rospy.Time.now()
        self.current_state.multi_dof_joint_state.joint_names.append("odom_to_base_link_joint")
        self.current_state.multi_dof_joint_state.transforms.append(transform)

        self.group.set_start_state(self.current_state)

    @n.publisher(POSE_TARGET_TOPIC, PoseStamped)
    def target_pub(self):
        pose_target = geometry_msgs.msg.PoseStamped()
        pose_target.header.frame_id = "body"
        pose_target.pose.orientation.w = 1.0
        pose_target.pose.position.x = 0.7
        pose_target.pose.position.y = -0.05
        pose_target.pose.position.z = 1.1
        return pose_target

    @n.subscriber(POSE_TARGET_TOPIC, PoseStamped)
    def pose_target_sub(self, ps):
        if self.current_state is not None:
            #self.group.set_start_state(RobotState())
            self.listener.waitForTransform(ps.header.frame_id, self.fixed_frame_id,
                                            rospy.Time(), rospy.Duration(1))
            ps_tf = self.listener.transformPose(self.fixed_frame_id, ps)
            # self.pose_target = ps_tf
            # self.group.set_pose_target(ps_tf.pose)
            self.pose_target = []
            self.pose_target.append(ps_tf.pose.position.x)
            self.pose_target.append(ps_tf.pose.position.y)
            self.pose_target.append(ps_tf.pose.position.z)
            self.pose_target.append(ps_tf.pose.orientation.x)
            self.pose_target.append(ps_tf.pose.orientation.y)
            self.pose_target.append(ps_tf.pose.orientation.z)
            self.pose_target.append(ps_tf.pose.orientation.w)
            self.plan = self.group.plan(self.pose_target)
            print self.plan

            self.plan_pub(self.plan)

    @n.publisher(PATH_TOPIC, Path)
    def plan_pub(self, plan):
        path = Path()
        path.header.frame_id = self.fixed_frame_id
        points = plan.multi_dof_joint_trajectory.points
        for point in points:
            transform = point.transforms[0]
            new_pose = PoseStamped()
            new_pose.header = path.header
            new_pose.pose.position.x = transform.translation.x
            new_pose.pose.position.y = transform.translation.y
            new_pose.pose.position.z = transform.translation.z
            path.poses.append(new_pose)
        self.pose_array_pub(path)
        return path

    @n.publisher(POSE_ARRAY_TOPIC, PoseArray)
    def pose_array_pub(self, path):
        pa = PoseArray()
        pa.header = path.header
        for pose in path.poses:
            pa.poses.append(pose.pose)
        return pa

    def turn_off_moveit(self):
        self.moveit_commander.roscpp_shutdown()

    @n.main_loop(frequency=30)
    def run(self):
        #self.joint_state_pub()
        self.ticker = self.ticker % 30
        if self.ticker == 1:
            self.target_pub()

            #print "============ Planning frame: %s" % self.group.get_planning_frame()
            #print "============ End effector frame: %s" % self.group.get_end_effector_link()
            #print "============ Robot Groups:"
            #print self.robot.get_group_names()

            #print "============"
            # plan to a random location
            #self.group.clear_pose_targets()
            #self.group.set_start_state(RobotState())
            #r = self.group.get_random_joint_values()
            #print "Planning to random joint position: "


        # print "============ Printing robot state"
        # print self.robot.get_current_state()

            # print joint_state

            # self.group.set_pose_target(pose_target, "base_link")
            # self.group.plan()

            #p = self.group.plan(pose_target)
            #print "Solution:"
            #print p
        self.ticker = self.ticker + 1



if __name__ == "__main__":
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)

    n.start(spin=True)

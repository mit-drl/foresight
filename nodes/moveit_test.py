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
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from moveit_msgs.msg import RobotState


NODE_NAME = "moveit_planner"
n = roshelper.Node(NODE_NAME, anonymous=False)

POSE_TARGET_TOPIC = "/pose_target"
JOINT_STATE_TOPIC = "/joint_state"

@n.entry_point()
class MoveItPlanner(object):

    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "odom")
        self.fixed_frame_id = rospy.get_param("~fixed_frame_id", "base_link")

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = self.robot.quad_body
        #self.group = moveit_commander.MoveGroupCommander("quad_body")
        self.display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)

        print "============ Waiting for RVIZ..."
        rospy.sleep(10)
        print "============ Starting tutorial "

        self.listener = tf.TransformListener()

        self.pose_target = None

        self.plan = None

        self.ticker = 0

        #rospy.on_shutdown(self.turn_off_moveit)

    @n.publisher(JOINT_STATE_TOPIC, JointState)
    def joint_state_pub(self):
        joint_state = JointState()
        joint_state.header.frame_id = "/odom"
        joint_state.position.append(1.0)
        joint_state.position.append(0.5)
        return joint_state

    @n.publisher(POSE_TARGET_TOPIC, PoseStamped)
    def target_pub(self):
        self.group.get_random_joint_values()
        return r
        # pose_target = geometry_msgs.msg.PoseStamped()
        # pose_target.header.frame_id = "base_link"
        # pose_target.pose.orientation.w = 1.0
        # pose_target.pose.position.x = 0.7
        # pose_target.pose.position.y = -0.05
        # pose_target.pose.position.z = 1.1
        # return pose_target

    @n.subscriber(POSE_TARGET_TOPIC, PoseStamped)
    def pose_target_sub(self,ps):
        self.listener.waitForTransform(ps.header.frame_id, self.fixed_frame_id,
                                        rospy.Time(), rospy.Duration(1))
        ps_tf = self.listener.transformPose(self.fixed_frame_id, ps)
        self.pose_target = ps_tf
        self.group.set_pose_target(ps_tf.pose)
        self.plan = self.group.plan()

    def turn_off_moveit(self):
        self.moveit_commander.roscpp_shutdown()

    @n.main_loop(frequency=30)
    def run(self):
        #self.joint_state_pub()
        self.ticker = self.ticker % 30
        if self.ticker == 1:
            #self.target_pub()

            print "============ Planning frame: %s" % self.group.get_planning_frame()
            #print "============ End effector frame: %s" % self.group.get_end_effector_link()
            print "============ Robot Groups:"
            print self.robot.get_group_names()
            #print "============ Printing robot state"
            #print self.robot.get_current_state()
            #print "============"
            # plan to a random location 
            self.group.clear_pose_targets()
            self.group.set_start_state(RobotState())
            r = self.group.get_random_joint_values()
            print "Planning to random joint position: "
            print r

            pose_target = geometry_msgs.msg.Pose()
            pose_target.orientation.w = 1.0
            pose_target.position.x = 0.7
            pose_target.position.y = -0.05
            pose_target.position.z = 1.1

            p = self.group.plan(r)
            print "Solution:"
            print p
        self.ticker = self.ticker + 1



if __name__ == "__main__":
    print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)

    n.start(spin=True)
#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

class MoveItPlanner(object):

	def __init__(self):
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()




if __name__ == "__main__":
	print "============ Starting tutorial setup"
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('moveit_planner',
					anonymous=False)
	planner = MoveItPlanner()
	rospy.spin()
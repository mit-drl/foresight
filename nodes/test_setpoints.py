#!/usr/bin/env python

import rospy
import tf
import math
from std_msgs.msg import Float64MultiArray


NODE_NAME = "test_setpoints"


class Test_Setpoints(object):

    def __init__(self, frequency, setpoint_topic, state_topic):
        self.frequency = frequency
        self.setpoint_topic = setpoint_topic
        self.state_topic = state_topic
        self.rate = rospy.Rate(frequency)
        self.pub = rospy.Publisher(setpoint_topic, Float64MultiArray,
                                                    queue_size=2)
        self.setpoint = Float64MultiArray()
        self.sub = None
        self.state = None
        self.t = 0

    def state_cb(self, data):
        self.state = data.data[0:2]

    def start(self):
        self.sub = rospy.Subscriber(self.state_topic, Float64MultiArray,
                                                    self.state_cb)
        while not rospy.is_shutdown():
            if not self.state is None:
                self.setpoint.data = self.getNextPoint(self.state)
                self.pub.publish(self.setpoint)
            self.rate.sleep()

    def getNextPoint(self, state):
        goal_x = 3*math.cos(math.pi*self.t/4.0)
        goal_y = 3*math.sin(math.pi*self.t/4.0)
        z = 1.2
        yaw = math.pi
        setpoint = [goal_x, goal_y, z, yaw, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0]*18
        if math.sqrt((goal_x - state[0])**2 + (goal_y - state[1])**2) < 0.7:
            self.t = self.t + 1.0/self.frequency
        return setpoint

def main():
    rospy.init_node(NODE_NAME, anonymous=False)
    setpoint_topic = rospy.get_param("~setpoint_topic", "/visualization/setpointsQuadrotor")
    state_topic = rospy.get_param("~state_topic", "/visualization/QuadrotorState")
    att = Test_Setpoints(5, setpoint_topic, state_topic)
    att.start()
    rospy.spin()

if __name__ == "__main__":
    main()
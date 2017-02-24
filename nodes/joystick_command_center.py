#!/usr/bin/env python

import roshelper
import math
import rospy
import tf
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

NODE_NAME = "joystick_command_center"
n = roshelper.Node(NODE_NAME, anonymous=False)

JOY_TOPIC = "/bebop/joy"
PLANNER_ENABLED_TOPIC = "/planner_enabled"


@n.entry_point()
class JoystickCommandCenter(object):

    def __init__(self):
        self.planner_enabled = False
        self.last_joy = Joy()

    @n.subscriber(JOY_TOPIC, Joy)
    def joy_sub(self, joy):
        if joy.buttons[4] > 0:
            update = abs(self.last_joy.buttons[3] - joy.buttons[3]) > 0
            if joy.buttons[3] > 0 and update:
                self.planner_enabled_pub()
        self.last_joy = joy

    @n.publisher(PLANNER_ENABLED_TOPIC, Bool)
    def planner_enabled_pub(self):
        self.planner_enabled = not self.planner_enabled
        return self.planner_enabled

    @n.main_loop(frequency=100)
    def run(self):
        pass

if __name__ == "__main__":
    n.start(spin=True)

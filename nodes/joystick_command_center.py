#!/usr/bin/env python

import roshelper
import math
import rospy
import tf
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from foresight.msg import ForesightState
from enum import Enum

NODE_NAME = "joystick_command_center"
n = roshelper.Node(NODE_NAME, anonymous=False)

JOY_TOPIC = "/bebop/joy"
STATE_TOPIC = "/state"


class Buttons(object):
    A = 0
    B = 1
    X = 2
    Y = 3
    LB = 4
    RB = 5
    BACK = 6
    START = 7
    CENTER = 8


@n.entry_point()
class JoystickCommandCenter(object):

    def __init__(self):
        self.planner_enabled = False
        self.last_joy = Joy()
        self.fs = ForesightState()

    def on_button_click(self, joy, btn, func):
        update = abs(self.last_joy.buttons[btn] - joy.buttons[btn]) > 0
        if joy.buttons[btn] > 0 and update:
            func()

    @n.subscriber(JOY_TOPIC, Joy)
    def joy_sub(self, joy):
        if joy.buttons[Buttons.LB] > 0:
            self.on_button_click(joy, Buttons.A, self.toggle_landing_state)
            self.on_button_click(joy, Buttons.Y, self.toggle_planner_state)
        self.last_joy = joy

    @n.publisher(STATE_TOPIC, ForesightState)
    def toggle_planner_state(self):
        if self.fs.state == ForesightState.PLANNER:
            self.fs.state = ForesightState.JOYSTICK
        else:
            self.fs.state = ForesightState.PLANNER
        return self.fs

    @n.publisher(STATE_TOPIC, ForesightState)
    def toggle_landing_state(self):
        if self.fs.state == ForesightState.LANDING:
            self.fs.state = ForesightState.JOYSTICK
        else:
            self.fs.state = ForesightState.LANDING
        return self.fs

    @n.main_loop(frequency=100)
    def run(self):
        pass

if __name__ == "__main__":
    n.start(spin=True)

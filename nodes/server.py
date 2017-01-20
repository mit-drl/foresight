#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from foresight.cfg import PIDConfig

def callback(config, level):
    return config

if __name__ == "__main__":
    rospy.init_node("pd_server", anonymous = False)

    srv = Server(PIDConfig, callback)
    rospy.spin()
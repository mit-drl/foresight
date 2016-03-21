#!/usr/bin/env python

import rospy
from mavros_msgs.msg import OpticalFlowRad


NODE_NAME = "range_filter"
OF_TOPIC = "/mavros/px4flow/raw/optical_flow_rad"
FILTER_TOPIC = "/foresight/optical_flow_rad/filtered"


class RangeFilter(object):

    def __init__(self, learning_rate):
        self.lr = learning_rate
        self.sub = None
        self.pub = None

        # default starting value
        self.ofr = None

    def start(self):
        self.sub = rospy.Subscriber(
            OF_TOPIC, OpticalFlowRad, self.optical_flow_cb)
        self.pub = rospy.Publisher(
            FILTER_TOPIC, OpticalFlowRad, queue_size=1)
        return self

    def optical_flow_cb(self, ofr):
        if self.ofr is None:
            self.ofr = ofr
        self.ofr.distance = (1 - self.lr) * self.ofr.distance \
            + self.lr * ofr.distance
        self.pub.publish(self.ofr)

if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=False)
    lr = rospy.get_param("learning_rate", 0.5)
    rf = RangeFilter(lr)
    rf.start()
    rospy.spin()

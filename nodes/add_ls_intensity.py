#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan


IN_SCAN_TOPIC = "in_scan"
OUT_SCAN_TOPIC = "out_scan"
NODE_NAME = "intensity_adder"
pub = None
sub = None


def laserscan_callback(scan):
    scan.intensities = scan.ranges
    pub.publish(scan)


def main():
    global pub, sub
    pub = rospy.Publisher(OUT_SCAN_TOPIC, LaserScan, queue_size=1)
    sub = rospy.Subscriber(IN_SCAN_TOPIC, LaserScan, laserscan_callback)
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node(NODE_NAME)
    main()

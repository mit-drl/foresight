#!/usr/bin/env python

import roshelper
from nav_msgs.msg import Odometry

NODE_NAME = "odom_covariance"
n = roshelper.Node(NODE_NAME, anonymous=False)


@n.entry_point()
class OdomCovariance(object):

    def __init__(self, px, py, pz, rx, ry, rz):
        self.cov = [0] * 36
        self.cov[0] = px
        self.cov[7] = py
        self.cov[14] = pz
        self.cov[21] = rx
        self.cov[28] = ry
        self.cov[35] = rz

    @n.publisher("odom_output", Odometry)
    def odom_pub(self, odom):
        odom.pose.covariance = self.cov
        odom.twist.covariance = self.cov
        return odom

    @n.subscriber("odom_input", Odometry)
    def odom_sub(self, odom):
        self.odom_pub(odom)

    @n.main_loop(frequency=30)
    def run(self):
        pass


if __name__ == "__main__":
    n.start(spin=True)

#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import Float64MultiArray


NODE_NAME = "tf_to_multiarray"


#### Multiarray format:
### x, y, z, vx, vy, yaw, xaccel, yaccel
### each quad has 8 variables
### total length of array is 4*8
### first 8 are for the first quad

class TFToMultiArray(object):

    def __init__(self, frequency, state_topic, fixed_frame, child_frame):
        self.state_topic = state_topic
        self.fixed_frame = fixed_frame
        self.child_frame = child_frame
        self.tfl = tf.TransformListener()
        self.rate = rospy.Rate(frequency)
        self.pub = rospy.Publisher(state_topic, Float64MultiArray,
                                   queue_size=2)
        self.sub = None
        self.lr = 0.1
        self.pose = Float64MultiArray()

    def quat_to_list(self, quat):
        return [quat.x, quat.y, quat.z, quat.w]

    def quat_to_yaw(self, q):
        R2 = 2*(q[0]*q[1]-q[2]*q[3])
        R1 = q[0]**2-q[1]**2-q[2]**2+q[3]**2
        yaw = math.atan2(R2,R1)
        return yaw

    def set_mov(self, obj, var, val):
        setattr(obj, var, (1 - self.lr) * getattr(obj, var) + self.lr * val)
        return self

    def start(self):
        while not rospy.is_shutdown():
            try:
                self.tfl.waitForTransform(
                    self.fixed_frame, self.child_frame,
                    rospy.Time(), rospy.Duration(0.1))
                tr, quat = self.tfl.lookupTransform(
                    self.fixed_frame, self.child_frame, rospy.Time())
                lin, ang = self.tfl.lookupTwist(
                    self.fixed_frame, self.child_frame, rospy.Time(),0.05)
                yaw = self.quat_to_list(quat)
                data = tr + [yaw]
                self.pose.header.seq += 1
                self.pub.publish(self.pose)
            except tf.Exception:
                print "TF ERROR"
            self.rate.sleep()


def main():
    rospy.init_node(NODE_NAME, anonymous=False)
    state_topic = rospy.get_param("~state_topic", "/visualization/QuadrotorState")
    fixed_frame = rospy.get_param("~fixed_frame", "map")
    child_frame = rospy.get_param("~child_frame", "base_link")
    att = TFToMultiArray(100, state_topic, fixed_frame, child_frame)
    att.start()
    rospy.spin()


if __name__ == "__main__":
    main()

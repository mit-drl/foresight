#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


NODE_NAME = "imu_tf"


class IMU_TF(object):

    def __init__(self, frequency, imu_topic, fixed_frame, child_frame):
        self.imu_topic = imu_topic
        self.fixed_frame = fixed_frame
        self.child_frame = child_frame

        self.br = tf.TransformBroadcaster()
        self.rate = rospy.Rate(frequency)
        self.sub = None
        self.quat = None

    def quat_to_list(self, quat):
        print quat
        return [quat.x, quat.y, quat.z, quat.w]

    def imu_cb(self,data):
        self.quat = data.orientation

    def start(self):
        self.sub = rospy.Subscriber(self.imu_topic, Imu,
                                    self.imu_cb)
        while not rospy.is_shutdown():
            if not self.quat is None:
                self.br.sendTransform(
                    [0, 0, 0],
                    self.quat_to_list(self.quat),
                    rospy.Time.now(),
                    self.child_frame, self.fixed_frame)
            self.rate.sleep()

def main():
    rospy.init_node(NODE_NAME, anonymous=False)
    imu_topic = rospy.get_param("~imu_topic", "/mavros/imu/data")
    fixed_frame = rospy.get_param("~fixed_frame", "map")
    child_frame = rospy.get_param("~child_frame", "car/base_link")
    att = IMU_TF(100, imu_topic, fixed_frame, child_frame)
    att.start()
    rospy.spin()

if __name__ == "__main__":
    main()
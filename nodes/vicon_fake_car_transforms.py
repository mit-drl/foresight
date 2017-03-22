#!/usr/bin/env python

import roshelper
import rospy
import tf
from vicon_bridge.msg import Markers

NODE_NAME = "vicon_fake_car_transforms"
n = roshelper.Node(NODE_NAME, anonymous=False)

VICON_MARKERS_TOPIC = "/vicon/markers"
frame_map = {
    "fake_car_uwb1": "tag_right_center",
    "fake_car_uwb2": "tag_left_back",
    "fake_car_uwb3": "tag_left_front",
    "fake_car_uwb4": "tag_left_bumper",
    "fake_car_uwb5": "tag_right_bumper",
    "fake_car_uwb6": "tag_right_front"}

offset_map = {
    "tag_right_center": [-0.02, -0.045, 0.03],
    "tag_left_back": [0.025, 0.04, 0.03],
    "tag_left_front": [0.05, -0.035, 0.02],
    "tag_left_bumper": [0, -0.015, 0.09],
    "tag_right_bumper": [0, -0.015, 0.09],
    "tag_right_front": [0.04, -0.025, 0.03]}


@n.entry_point()
class ViconFakeCarTransforms(object):

    def __init__(self):
        self.fixed_frame_id = rospy.get_param("~fixed_frame_id", "body")
        self.br = tf.TransformBroadcaster()

    @n.subscriber(VICON_MARKERS_TOPIC, Markers)
    def markers_sub(self, markers):
        for marker in markers.markers:
            if marker.marker_name in frame_map:
                frame_id = frame_map[marker.marker_name]
                offset = offset_map[frame_id]
                self.br.sendTransform([
                    marker.translation.x * 0.001 + offset[0],
                    marker.translation.y * 0.001 + offset[1],
                    marker.translation.z * 0.001 + offset[2]],
                    [0, 0, 0, 1], rospy.Time.now(),
                    frame_id, self.fixed_frame_id)


if __name__ == "__main__":
    n.start(spin=True)

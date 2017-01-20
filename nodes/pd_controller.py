#!/usr/bin/env python

PACKAGE = 'foresight'

import rospy, roslib
roslib.load_manifest(PACKAGE)
import tf
from geometry_msgs.msg import Twist
import dynamic_reconfigure.client

NODE_NAME = "pd_controller"
BEBOP_NODE_NAME = "bebop/bebop_driver"

class PID(object):

    def __init__(self, P=0.0, I=0.0, D=0.0):
        self.kp = P
        self.ki = I
        self.kd = D
        self.setpoint = 0.0
        self.old_error = 0.0
        self.prev_time = rospy.Time()
        self.integral = 0.0
        self.max = 0.0

    def step(self, state):
        error = self.setpoint - state

        dt = rospy.Time() - self.prev_time
        derror = error - self.old_error
        self.integral += error*dt

        output = self.kp*error + self.kd*(derror/dt) + self.ki*self.integral

        if output > self.max:
            output = self.max
        elif output < -self.max:
            output = -self.max
        output = output/self.max

        self.prev_time = rospy.Time()
        self.old_error = error

        self.output = output


class PD_Controller(object):

    def __init__(self, frequency, cmd_vel_topic, fixed_frame, child_frame):

        #rospy.wait_for_service("pd_server")
        self.cmd_vel_topic = cmd_vel_topic
        self.fixed_frame = fixed_frame
        self.child_frame = child_frame
        self.tfl = tf.TransformListener()
        self.rate = rospy.Rate(frequency)
        self.pub = rospy.Publisher(cmd_vel_topic, Twist,
                                   queue_size=2)

        server = 'pd_server'
        self.yaw_p = rospy.get_param(server + '/yaw_gains_P')
        self.yaw_i = rospy.get_param(server + '/yaw_gains_I')
        self.yaw_d = rospy.get_param(server + '/yaw_gains_D')
        self.yaw_setpoint = rospy.get_param(server + '/yaw_setpoint')
        self.yaw_vel_max = 1 #rospy.get_param(BEBOP_NODE_NAME + '/SpeedSettingsMaxRotationSpeedCurrent')

        self.yaw_control = PID(self.yaw_p,self.yaw_i,self.yaw_d)
        self.yaw_control.max = self.yaw_vel_max
        self.yaw_control.setpoint = self.yaw_setpoint
        
        self.z_p = rospy.get_param(server + '/z_gains_P')
        self.z_i = rospy.get_param(server + '/z_gains_I')
        self.z_d = rospy.get_param(server + '/z_gains_D')
        self.z_setpoint = rospy.get_param(server + '/z_setpoint')
        self.z_vel_max = 1 #rospy.get_param(BEBOP_NODE_NAME + '/SpeedSettingsMaxVerticalSpeedCurrent')

        self.z_control = PID(self.z_p,self.z_i,self.z_d)
        self.z_control.max = self.z_vel_max
        self.z_control.setpoint = self.z_setpoint

        self.client = dynamic_reconfigure.client.Client("pd_server", timeout=30, config_callback=self.callback)

    def start(self):
        while not rospy.is_shutdown():
            try:
                self.tfl.waitForTransform(
                    self.fixed_frame, self.child_frame,
                    rospy.Time(), rospy.Duration(0.1))
                tr, quat = self.tfl.lookupTransform(
                    self.fixed_frame, self.child_frame, rospy.Time())

                euler = tf.transformations.euler_from_quaternion(quaternion)
                roll = euler[0]
                pitch = euler[1]
                yaw = euler[2]

                z = tr[2]

                #lin, ang = self.tfl.lookupTwist(
                #    self.fixed_frame, self.child_frame, rospy.Time(),0.05)

                self.yaw_control.step(yaw)
                yaw_rate = self.yaw_control.output

                self.z_control.step(z)
                z_rate = self.z_control.output

                command = Twist()
                command.linear.x = 0.0
                command.linear.y = 0.0
                command.linear.z = z_rate
                command.angular.z = yaw_rate

                self.pub.publish(self.pose)
            except tf.Exception:
                print "TF ERROR"
            self.rate.sleep()

    def callback(self, config):
        if not (self.yaw_control == None or self.z_control == None):
            self.yaw_control.setpoint = config['yaw_setpoint']
            self.z_control.setpoint = config['z_setpoint']
            self.yaw_control.kp = config['yaw_gains_P']
            self.yaw_control.ki = config['yaw_gains_I']
            self.yaw_control.kd = config['yaw_gains_D']
            self.z_control.kp = config['z_gains_P']
            self.z_control.ki = config['z_gains_I']
            self.z_control.kd = config['z_gains_D']

def main():
    rospy.init_node(NODE_NAME, anonymous=False)

    cmd_vel_topic = rospy.get_param("~cmd_vel", "/cmd_vel")
    fixed_frame = rospy.get_param("~fixed_frame", "map")
    child_frame = rospy.get_param("~child_frame", "base_link")

#    rospy.set_param('~yaw/gains', "{'P': 0.05, 'I': 0.0, 'D': 0.01}")
#    rospy.set_param('~z/gains', "{'P': 0.05, 'I': 0.0, 'D': 0.01}")
#    rospy.set_param('~yaw/setpont','0.0')
#    rospy.set_param('~z/setpoint','1.0')
    att = PD_Controller(100, cmd_vel_topic, fixed_frame, child_frame)
    att.start()
    rospy.spin()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rospy
import threading
import time
import math
import atexit

from components import Servo

from geometry_msgs.msg import Twist

CHN_PWM_CAM_YAW = 1
CHN_PWM_CAM_PITCH = 2

ROT_MAX = 180
ROT_MIN = 0

NODE_NAME = 'camera_controller'

class CameraMotorNode(object):

    msg = Twist()
    is_running = True

    def __init__(self):
        atexit.register(self.exist)

        # Initialize the node and name it.
        rospy.init_node(NODE_NAME)

        self.rate = float(rospy.get_param('~rate', 60.0))
        self.cmd_topic = rospy.get_param('~cameramotor_cmd_topic', '/cam_vel')
        self.yaw_min = float(rospy.get_param('~yaw_min', ROT_MIN))
        self.yaw_max = float(rospy.get_param('~yaw_max', ROT_MAX))
        self.yaw_off = int(rospy.get_param('~yaw_offset', -20))
        self.pitch_min = float(rospy.get_param('~pitch_min', ROT_MIN)) # 70 for original cam (cam on front)
        self.pitch_max = float(rospy.get_param('~pitch_max', ROT_MAX))
        self.pitch_off = int(rospy.get_param('~pitch_offset', 0))

        # Initialize servo
        self.servo_yaw = Servo.Servo(CHN_PWM_CAM_YAW)
        self.servo_yaw.debug = False
        self.servo_yaw.min_degree_value = self.yaw_min
        self.servo_yaw.max_degree_value = self.yaw_max
        self.servo_yaw.offset = self.yaw_off

        self.servo_pitch = Servo.Servo(CHN_PWM_CAM_PITCH)
        self.servo_pitch.debug = False
        self.servo_pitch.min_degree_value = self.pitch_min
        self.servo_pitch.max_degree_value = self.pitch_max
        self.servo_pitch.offset = self.pitch_off

        # Test motor
        self.servo_yaw.default()
        self.servo_pitch.default()
        self.smooth_rotate(self.servo_yaw, 90+5, 3)
        self.smooth_rotate(self.servo_yaw, 90-5, 3)

        # Set origin
        self.servo_yaw.default()
        self.servo_pitch.default()

        # Create topics (publisher & subscriber).
        self.sub_cmd = rospy.Subscriber(self.cmd_topic, Twist, self.cmd_callback, queue_size=1)

        # loop for process.
        rospy.loginfo("Node '%s' started.\nListening to %s", NODE_NAME, self.cmd_topic)

    def smooth_rotate(self, servo, rot, duration):
        pres = 0.1
        delta = rot - servo.value
        step = duration / pres
        rospy.loginfo("d %d s %d", delta, step)

        if (delta > 0):
            while (servo.value < rot):
                servo.write(servo.value + (delta / step))
                rospy.loginfo("-state %d", servo.value)
                time.sleep(pres)
        else:
            while (servo.value > rot):
                servo.write(servo.value + (delta / step))
                rospy.loginfo("+state %d", servo.value)
                time.sleep(pres)

    def spin(self):
        r = rospy.Rate(self.rate)
        max_vel = 3


        while not rospy.is_shutdown():
            # Manage Yaw
            yaw = self.msg.angular.z
            servo_angle_yaw = int(90 - 20 * yaw)
            #rospy.loginfo("Yaw : %f \servo : %f", yaw, servo_angle_yaw)
            #rospy.loginfo("d %d s %d", self.servo_yaw.value, servo_angle_yaw)
            #if (servo_angle_yaw != self.servo_yaw.value):
            #    if (servo_angle_yaw > self.servo_yaw.value - max_vel):
            #        self.servo_yaw.write(self.servo_yaw.value + max_vel)
            #        #rospy.loginfo("A %f", self.servo_yaw.value)
            #    elif (servo_angle_yaw < self.servo_yaw.value + max_vel):
            #        self.servo_yaw.write(self.servo_yaw.value - max_vel)
            #        #rospy.loginfo("B %f", self.servo_yaw.value)
            #    else:
            #        self.servo_yaw.write(servo_angle_yaw)
            #        #rospy.loginfo("C %f", self.servo_yaw.value)
            self.servo_yaw.write(servo_angle_yaw)

            # Manage Pitch
            pitch = self.msg.angular.y
            servo_angle_pitch = 180 - int(90 - 20 * pitch)
            # if (servo_angle_pitch > self.servo_pitch.value):
            #     self.servo_pitch.write(self.servo_pitch.value + max_vel)
            # elif (servo_angle_pitch < self.servo_pitch.value):
            #     self.servo_pitch.write(self.servo_pitch.value - max_vel)
            #if (servo_angle_pitch != self.servo_pitch.value):
            #    if (servo_angle_pitch > self.servo_pitch.value - max_vel):
            #        self.servo_pitch.write(self.servo_pitch.value + max_vel)
            #        #rospy.loginfo("A %f", self.servo_pitch.value)
            #    elif (servo_angle_pitch < self.servo_pitch.value + max_vel):
            #        self.servo_pitch.write(self.servo_pitch.value - max_vel)
            #        #rospy.loginfo("B %f", self.servo_pitch.value)
            #    else:
            #        self.servo_pitch.write(servo_angle_pitch)
            #        #rospy.loginfo("C %f", self.servo_pitch.value)
            self.servo_pitch.write(servo_angle_pitch)

            ## Sleep
            r.sleep()

    def cmd_callback(self, msg):
        #rospy.loginfo("Receive twist %s", msg)
        self.msg = msg

    def exist(self):
        self.msg = Twist()
        self.servo_yaw.default()
        self.servo_pitch.default()
        self.is_running = False

# Main function.
if __name__ == '__main__':
    try:
        ne = CameraMotorNode()
        ne.spin()
    except rospy.ROSInterruptException:
        ne.emergency()

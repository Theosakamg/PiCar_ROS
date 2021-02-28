#!/usr/bin/env python3

import rospy
import threading
import time
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

        self.rate = float(rospy.get_param('~rate', 20.0))
        self.cmd_topic = rospy.get_param('~cameramotor_cmd_topic', '/cam_vel')

        # Initialize servo
        self.servo_yaw = Servo.Servo(CHN_PWM_CAM_YAW)
        self.servo_yaw.debug = False
        self.servo_yaw.min_degree_value = ROT_MIN
        self.servo_yaw.max_degree_value = ROT_MAX
        self.servo_yaw.offset = 0

        self.servo_pitch = Servo.Servo(CHN_PWM_CAM_PITCH)
        self.servo_pitch.debug = False
        self.servo_pitch.min_degree_value = 70
        self.servo_pitch.max_degree_value = ROT_MAX
        self.servo_pitch.offset = 0

        # Set origin
        self.servo_yaw.default()
        self.servo_pitch.default()

        # Create topics (publisher & subscriber).
        rospy.Subscriber(self.cmd_topic, Twist, self.cmd_callback, queue_size=1)

        # loop for process.
        rospy.loginfo("Node '%s' started.\nListening to %s", NODE_NAME, self.cmd_topic)

    def spin(self):
        r = rospy.Rate(self.rate)
        while(self.is_running):
            # Manage Yaw
            yaw = self.msg.angular.z
            if(yaw > 0.1 or yaw < -0.1):
                servo_angle_yaw = int(90 - 20 * yaw)
                rospy.loginfo("Yaw : %f \servo : %f", yaw, servo_angle_yaw)
                self.servo_yaw.write(servo_angle_yaw)
            else:
                self.servo_yaw.default()

            # Manage Pitch
            pitch = self.msg.angular.y
            if(pitch > 0.1 or pitch < -0.1):
                servo_angle_pitch = 180 - int(90 - 20 * pitch)
                rospy.loginfo("Pitch : %f \servo : %f", pitch, servo_angle_pitch)
                self.servo_pitch.write(servo_angle_pitch)
            else:
                self.servo_pitch.default()

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

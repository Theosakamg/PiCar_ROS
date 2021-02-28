#!/usr/bin/env python3

import rospy
import threading
import time
import math
import atexit

from drivers import PCA9685
from drivers import TB6612
from components import Servo, Throttle

from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive

from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from picar_bringup.cfg import PicarConfig

import odom

NODE_NAME = 'picar_controller'
TYPE_ACKERMANN = 'ackermann_drive'

GPIO_MOTOR_ROT_A = 17
GPIO_MOTOR_ROT_B = 27

CHN_PWM_DIR = 0
CHN_PWM_A = 4
CHN_PWM_B = 5

DIR_MIN = 30
DIR_MAX = 150

class PicarNode(object):

    msg = AckermannDriveStamped()
    freq = 50
    is_running = True

    def __init__(self):
        atexit.register(self.emergency)

        # Initialize Direction
        self.servo = Servo.Servo(CHN_PWM_DIR)
        self.servo.debug = False
        self.servo.min_degree_value = DIR_MIN
        self.servo.max_degree_value = DIR_MAX
        self.servo.offset = 15

        # Initialize engine
        self.throttle_a = Throttle.Throttle(CHN_PWM_A)
        self.throttle_a.debug = False
        self.throttle_b = Throttle.Throttle(CHN_PWM_B)
        self.throttle_b.debug = False

        self.motorA = TB6612.Motor(GPIO_MOTOR_ROT_A, pwm=self.throttle_a.write, offset=False)
        self.motorB = TB6612.Motor(GPIO_MOTOR_ROT_B, pwm=self.throttle_b.write, offset=False)

        # Set origin
        self.servo.default()
        self.motorA.speed = 0
        self.motorB.speed = 0

        # Initialize the node and name it.
        rospy.init_node(NODE_NAME)

        # Load Parameters.
        self.arg_wheel_diameter = float(rospy.get_param('~wheel_diameter', 0.067))
        self.arg_motor_speed_max = int(rospy.get_param('~motor_speed_max', 162)) # empty : 195 (mesured)

        if self.arg_wheel_diameter <= 0:
            rospy.logwarn("Weel diameter can be < 0 meter !")
            self.arg_wheel_diameter = 0.067
        
        if self.arg_motor_speed_max <= 0:
            rospy.logwarn("Motor max speed can be < 0 meter !")
            self.arg_motor_speed_max = 200

        # convert max RPM to RPS (x/60), apply RPS to perimeter, to ratio for per cent.
        self.motor_ratio = ((float(self.arg_motor_speed_max)/60)*(self.arg_wheel_diameter*math.pi))/100
        rospy.loginfo("Motor ratio : %f (wheel %f, motor speed %d)", self.motor_ratio, self.arg_wheel_diameter, self.arg_motor_speed_max)

        self.ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd')
        self.message_type = rospy.get_param('~message_type', 'ackermann_drive_stamped') # ackermann_drive or ackermann_drive_stamped

        self.srv = DynamicReconfigureServer(PicarConfig, self.dynrec_callback)

        # Create topics (publisher & subscriber).
        rospy.Subscriber(self.ackermann_cmd_topic, AckermannDriveStamped, self.cmd_callback, queue_size=1)

        self.odomNode = odom.PicarToOdom()

        # Start main loop
        self.thread = threading.Thread(target=self.__loop, args=())
        self.thread.start()

        # loop for process.
        rospy.loginfo("Node '%s' started.\nListening to %s", NODE_NAME, self.ackermann_cmd_topic)
        try:
            rospy.spin()
        except KeyboardInterrupt:
            self.emergency()

    def __del__(self):
        self.emergency()

    def __loop(self):
        while(self.is_running):
            self.msg.header.stamp = rospy.Time.now()

            # Manage Direction
            steering_angle = self.msg.drive.steering_angle
            if(steering_angle > 0.1 or steering_angle < -0.1):
                servo_angle = int(90 - 20 * steering_angle)
                rospy.loginfo("steering : %f \servo : %f", steering_angle, servo_angle)
                self.servo.write(servo_angle)
            else:
                self.servo.default()

            ## Manage Speed
            # Direction of operation 
            cmd_speed = self.msg.drive.speed
            if (cmd_speed > 0):
                self.motorA.forward()
                self.motorB.forward()
            elif (cmd_speed < 0):
                self.motorA.backward()
                self.motorB.backward()

            # Limit to speed to motor
            #TODO Add accel concept.
            motor_speed = math.fabs(cmd_speed)/self.motor_ratio
            if (motor_speed > 100):
                motor_speed = 100

            # Set Motor speed
            self.motorA.speed = motor_speed
            self.motorB.speed = motor_speed

            self.odomNode.stateCallback(self.msg)
            ## Sleep
            time.sleep(1/self.freq)

    def cmd_callback(self, msg):
        self.msg = msg

    def dynrec_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: {steering_offset}""".format(**config))
        self.servo.offset = config.steering_offset
        return config

    def emergency(self):
        self.vel = AckermannDriveStamped()
        self.motorA.speed = 0
        self.motorB.speed = 0
        self.is_running = False

# Main function.
if __name__ == '__main__':
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = PicarNode()
    except rospy.ROSInterruptException: pass

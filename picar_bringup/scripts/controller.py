#!/usr/bin/env python3

import rospy, math

from drivers import PCA9685
from components import Servo, Throttle

from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

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

        # Initialize Direction
        self.servo = Servo.Servo(CHN_PWM_DIR)
        self.servo.debug = True
        self.servo.min_degree_value = DIR_MIN
        self.servo.max_degree_value = DIR_MAX
        self.servo.offset = 15

        # Initialize engine
        self.throttle_a = Throttle.Throttle(CHN_PWM_A)
        self.throttle_a.debug = True
        self.throttle_b = Throttle.Throttle(CHN_PWM_B)
        self.throttle_b.debug = True

        self.motorA = TB6612.Motor(GPIO_MOTOR_ROT_A, pwm=self.throttle_a.write, offset=False)
        self.motorB = TB6612.Motor(GPIO_MOTOR_ROT_B, pwm=self.throttle_b.write, offset=False)

        # Set origin
        self.servo.default()
        self.motorA.speed = 0
        self.motorB.speed = 0

        # Initialize the node and name it.
        rospy.init_node(NODE_NAME)

        # Load Parameters.
        self.ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd')
        self.message_type = rospy.get_param('~message_type', 'ackermann_drive_stamped') # ackermann_drive or ackermann_drive_stamped

        # Create topics (publisher & subscriber).
        rospy.Subscriber(self.ackermann_cmd_topic, AckermannDriveStamped, self.cmd_callback, queue_size=1)

        # Start main loop
        self.thread = threading.Thread(target=self.__loop, args=())
        self.thread.start()

        # loop for process.
        rospy.loginfo("Node '%s' started.\nListening to %s", NODE_NAME, self.ackermann_cmd_topic)
        rospy.spin()

    def __loop(self):
        while(self.is_running):
            # Manage Direction
            if(self.msg.angular.z > 0.1 or self.msg.angular.z < -0.1):
                self.servo.write( int(90 - 20 * self.msg.angular.z) )
            else:
                self.servo.default()

            # Manage Engine
            cmd_speed = self.msg.drive.speed
            if (cmd_speed > 0):
                self.motorA.forward()
                self.motorB.forward()
            elif (cmd_speed < 0):
                self.motorA.backward()
                self.motorB.backward()
            else:
                pass

            speed = abs(cmd_speed)
            if (speed > 1.0):
                motor_speed = 100
            else:
                motor_speed = 100 * speed

            self.motorA.speed = motor_speed
            self.motorB.speed = motor_speed

            time.sleep(1/self.freq)

    def cmd_callback(self, msg):
        self.msg = msg

# Main function.
if __name__ == '__main__':
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = PicarNode()
    except rospy.ROSInterruptException: pass

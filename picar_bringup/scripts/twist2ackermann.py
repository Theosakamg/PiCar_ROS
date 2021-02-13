#!/usr/bin/env python3

import rospy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

NODE_NAME = 'cmdvel_to_cmdack'
TYPE_ACKERMANN = 'ackermann_drive'

class Twist2Ackermann(object):

    def __init__(self):
        # Initialize the node and name it.
        rospy.init_node(NODE_NAME)

        # Load Parameters.
        self.twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
        self.ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd')
        self.wheelbase = rospy.get_param('~wheelbase', 10.0)
        self.frame_id = rospy.get_param('~frame_id', 'odom')
        self.message_type = rospy.get_param('~message_type', 'ackermann_drive_stamped') # ackermann_drive or ackermann_drive_stamped

        # Create topics (publisher & subscriber).
        rospy.Subscriber(self.twist_cmd_topic, Twist, self.cmd_callback, queue_size=1)
        if TYPE_ACKERMANN == self.message_type:
            self.pub = rospy.Publisher(self.ackermann_cmd_topic, AckermannDrive, queue_size=1)
        else:
            self.pub = rospy.Publisher(self.ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)

        # loop for process.
        rospy.loginfo("Node '%s' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", NODE_NAME, self.twist_cmd_topic, self.ackermann_cmd_topic, self.frame_id, self.wheelbase)
        rospy.spin()

    def convert_trans_rot_vel_to_steering_angle(self, speed, omega):
        steering_angle = 0
        if not (omega == 0 or speed == 0):
            radius = speed / omega
            steering_angle = math.atan(self.wheelbase / radius)

        return steering_angle

    def cmd_callback(self, data):
        steering = self.convert_trans_rot_vel_to_steering_angle(data.linear.x,
                                                                data.angular.z)

        if TYPE_ACKERMANN == self.message_type:
            msg = AckermannDrive()
            msg.steering_angle = steering
            msg.speed = data.linear.x

        else:
            msg = AckermannDriveStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            msg.drive.steering_angle = steering
            msg.drive.speed = data.linear.x
            
        self.pub.publish(msg)


# Main function.
if __name__ == '__main__':
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = Twist2Ackermann()
    except rospy.ROSInterruptException: pass

#!/usr/bin/env python3

import rospy, math

from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
from dynamic_reconfigure.server import Server as DynamicReconfigureServer

NODE_NAME = 'picar_controller'
TYPE_ACKERMANN = 'ackermann_drive'

class PicarNode(object):

    def __init__(self):
        # Initialize the node and name it.
        rospy.init_node(NODE_NAME)

        # Load Parameters.
        self.ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd')
        self.message_type = rospy.get_param('~message_type', 'ackermann_drive_stamped') # ackermann_drive or ackermann_drive_stamped

        # Create topics (publisher & subscriber).
        rospy.Subscriber(self.ackermann_cmd_topic, AckermannDriveStamped, self.cmd_callback, queue_size=1)

        # loop for process.
        rospy.loginfo("Node '%s' started.\nListening to %s", NODE_NAME, self.ackermann_cmd_topic)
        rospy.spin()

    def cmd_callback(self, data):
        if(msg.linear.x > 0.1):
            bw.forward()
            bw.speed = int(100 * msg.linear.x)
        elif(msg.linear.x < -0.1):
            bw.backward()
            bw.speed = int(100 * -msg.linear.x)
        # else:
        #     bw.stop()

        if(msg.angular.z > 0.1 or msg.angular.z < -0.1):
            fw.turn( int(90 - 20 * msg.angular.z) )
        else:
            fw.turn_straight()


# Main function.
if __name__ == '__main__':
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        ne = PicarNode()
    except rospy.ROSInterruptException: pass

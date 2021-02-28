#!/usr/bin/env python3

import time
import rospy
import math

from geometry_msgs.msg import Twist

wheel_diameter          = 0.067                     # meter
motor_speed_max         = 162                       # RPM
motor_speed_max_per_sec = motor_speed_max / 60      # RPS
wheel_circumference     = wheel_diameter * math.pi  # meter

max_speed = ((float(motor_speed_max)/60)*wheel_circumference)

if __name__ == '__main__':
    rospy.init_node("counter_publisher")
    rate = rospy.Rate(20)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    counter = 0

    rospy.loginfo("Publisher has been started.")
    start_time = time.time()
    while not rospy.is_shutdown():
        counter += 1

        msg = Twist()
        msg.linear.x = max_speed
        msg.linear.y = 0.00
        msg.linear.z = 0.00
        msg.angular.x = 0.00
        msg.angular.y = 0.00
        msg.angular.z = 0.00
        pub.publish(msg)

        rate.sleep()
        elapsed_time = time.time() - start_time
        rospy.loginfo("Send... %f", elapsed_time)
        if (elapsed_time >= 1.75):
            break
    
    msg = Twist()
    msg.linear.x = 0.00
    msg.linear.y = 0.00
    msg.linear.z = 0.00
    msg.angular.x = 0.00
    msg.angular.y = 0.00
    msg.angular.z = 0.00
    pub.publish(msg)
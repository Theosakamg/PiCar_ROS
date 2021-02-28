#!/usr/bin/env python3

import rospy
import math

import tf2_ros

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped

NODE_NAME = 'picar_odom'

class PicarToOdom(object):

    _odom_frame_ : str
    _base_frame_ : str
    # State message does not report servo position, so use the command instead
    _use_servo_cmd : bool
    # conversion gain and offset
    # double speed_to_erpm_gain_, speed_to_erpm_offset_;
    # double steering_to_servo_gain_, steering_to_servo_offset_;
    # double wheelbase_;
    _publish_tf : bool

    # odometry state
    #double x_, y_, yaw_;
    #std_msgs::Float64::ConstPtr last_servo_cmd_; ///< Last servo position commanded value
    #vesc_msgs::VescStateStamped::ConstPtr last_state_; ///< Last received state message
    _last_state : AckermannDriveStamped

    # ROS services
    #ros::Publisher odom_pub_;
    #ros::Subscriber vesc_state_sub_;
    #ros::Subscriber servo_sub_;
    #boost::shared_ptr<tf::TransformBroadcaster> tf_pub_;

    def __init__(self):
        self._odom_frame = "odom"
        self._base_frame = "base_link"
        self._use_servo_cmd = True
        self._publish_tf = True
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        # create odom publisher
        self._odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)

        # create tf broadcaster
        if (self._publish_tf):
            self.broadcaster = tf2_ros.TransformBroadcaster()

        # subscribe to picar state.
        # vesc_state_sub_ = nh.subscribe("sensors/core", 10, &VescToOdom::vescStateCallback, this);
        # if (use_servo_cmd_) {
        #     servo_sub_ = nh.subscribe("sensors/servo_position_command", 10,
        #                             &VescToOdom::servoCmdCallback, this);
        # }
    
    def stateCallback(self, state):
        # check that we have a last servo command if we are depending on it for angular velocity
        #if (use_servo_cmd_ && !last_servo_cmd_)
        #    return;

        # convert to engineering units
        current_speed = state.drive.speed
        #( state->state.speed - speed_to_erpm_offset_ ) / speed_to_erpm_gain_;
        current_steering_angle = 0.0
        current_angular_velocity = 0.0
        if (self._use_servo_cmd):
            current_steering_angle = state.drive.steering_angle
            #( last_servo_cmd_->data - steering_to_servo_offset_ ) / steering_to_servo_gain_;
            current_angular_velocity = state.drive.steering_angle_velocity
            #current_speed * tan(current_steering_angle) / wheelbase_;

        # use current state as last state if this is our first time here
        #if (not self._last_state):
        if (not hasattr(self, '_last_state')):
            self._last_state = state;

        # calc elapsed time
        dt = state.header.stamp - self._last_state.header.stamp;

        # propigate odometry
        x_dot = current_speed * math.cos(self._yaw)
        y_dot = current_speed * math.sin(self._yaw)
        self._x += x_dot * dt.to_sec()
        self._y += y_dot * dt.to_sec()

        if (self._use_servo_cmd):
            self._yaw += current_angular_velocity * dt.to_sec()

        rospy.loginfo("ODOM/TF \tspeed:%f \tangle:%f \tangle_velocity:%f \tdX:%f \tdY:%f \tX:%f \tY:%f \tyaw:%f \tduration:%f",
            current_speed,
            current_steering_angle,
            current_angular_velocity,
            x_dot,
            y_dot,
            self._x,
            self._y,
            self._yaw,
            dt.to_sec()
            )

        # save state for next time
        self._last_state = state;

        # publish odometry message
        odom = Odometry()
        odom.header.frame_id = self._odom_frame
        odom.header.stamp = state.header.stamp
        odom.child_frame_id = self._base_frame

        # Position
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self._yaw/2.0)
        odom.pose.pose.orientation.w = math.cos(self._yaw/2.0)

        # Position uncertainty
        # @todo Think about position uncertainty, perhaps get from parameters? */
        odom.pose.covariance[0]  = 0.2 ##< x
        odom.pose.covariance[7]  = 0.2 ##< y
        odom.pose.covariance[35] = 0.4 ##< yaw

        # Velocity ("in the coordinate frame given by the child_frame_id")
        odom.twist.twist.linear.x = current_speed
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = current_angular_velocity

        # Velocity uncertainty
        # @todo Think about velocity uncertainty */

        if (self._publish_tf):
            tf = TransformStamped()
            tf.header.stamp = rospy.Time.now()
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = odom.pose.pose.orientation
            self.broadcaster.sendTransform(tf)

        self._odom_pub.publish(odom)

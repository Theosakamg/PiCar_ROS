#!/usr/bin/env python3

import rospy
import math

import tf2_ros

from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped, Quaternion

from components import Sensor

NODE_NAME = 'picar_odom'

class PicarToOdom(object):
    """
    https://medium.com/@waleedmansoor/how-i-built-ros-odometry-for-differential-drive-vehicle-without-encoder-c9f73fe63d87
    """


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

    def __init__(self, sensorA, sensorB, distance_pulse):
        self._odom_frame = "odom"
        self._base_frame = "base_link"
        self._publish_tf = True
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0

        self.sensorA = sensorA
        self.sensorB = sensorB
        self.distance_pulse = distance_pulse

        # create odom publisher
        self._odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)

        # create tf broadcaster
        if (self._publish_tf):
            self.broadcaster = tf2_ros.TransformBroadcaster()

        self._reset_service = rospy.Service('odom_reset', std_srvs.srv.Empty, self.odom_reset_cb)

        # subscribe to picar state.
        # vesc_state_sub_ = nh.subscribe("sensors/core", 10, &VescToOdom::vescStateCallbac_c, this);
        # if (use_servo_cmd_) {
        #     servo_sub_ = nh.subscribe("sensors/servo_position_command", 10,
        #                             &VescToOdom::servoCmdCallback, this);
        # }
    
    def odom_reset_cb(self, req):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.sensorA.reset()
        self.sensorB.reset()
        return std_srvs.srv.EmptyResponse()


    def state_callback(self, state):
        # convert to engineering units
        current_speed = state.drive.speed
        current_steering_angle = state.drive.steering_angle
        current_angular_velocity = state.drive.steering_angle_velocity

        # use current state as last state if this is our first time here
        if (not hasattr(self, '_last_state')):
            self._last_state = state;

        # calc elapsed time
        dt = state.header.stamp - self._last_state.header.stamp;

        # propigate odometry
        ## from theorical
        x_dot = current_speed * math.cos(self._yaw)
        y_dot = current_speed * math.sin(self._yaw)
        delta_x  = x_dot * dt.to_sec()
        delta_y  = y_dot * dt.to_sec()
        delta_th = current_angular_velocity * dt.to_sec()

        rospy.loginfo("THEORICAL \tspeed:%f \tangle:%f \tangle_velocity:%f \tdelta X:%f Y:%f TH:%f",
            current_speed,
            current_steering_angle,
            current_angular_velocity,
            delta_x,
            delta_y,
            delta_th
            )

        ## from wheel encoder
        tick_a = self.sensorA.count_high()
        tick_b = self.sensorB.count_high()
        delta_tick_a = tick_a - self._last_encoder_left
        delta_tick_b = tick_b - self._last_encoder_right
        v_left  = delta_tick_a * self.distance_pulse / dt.to_sec()
        v_right = delta_tick_b * self.distance_pulse / dt.to_sec()
        delta_distance = 0.5f * (double)(delta_tick_a + delta_tick_B) * self.distance_pulse
        delta_x = delta_distance * (double)math.cos(self._yaw)
        delta_y = delta_distance * (double)math.sin(self._yaw)
        delta_th = (double)(delta_tick_b - delta_tick_a) * self.distance_pulse / 0.36f  # Distance between the two wheels is 0.36m

        rospy.loginfo("WHEEL Speed \tA:%f \tB:%f \tdelta X:%f Y:%f TH:%f",
            v_left,
            v_right,
            delta_x,
            delta_y,
            delta_th
            )

        # Apply position
        self._x += delta_x
        self._y += delta_y
        self._yaw += delta_th

        rospy.loginfo("ODOM/TF \tX:%f \tY:%f \tyaw:%f \tduration:%f",
            self._x,
            self._y,
            self._yaw,
            dt.to_sec()
            )

        # save state for next time
        self._last_state = state;
        self._last_encoder_left = tick_a
        self._last_encoder_right = tick_b

        # Common
        self._current_time = rospy.Time.now()
        self._quaternon = tf_conversions.transformations.quaternion_from_euler(0, 0, self._yaw)

        # publish odometry message
        odom = Odometry()
        odom.header.stamp = self._current_time
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame

        # Position
        odom.pose.pose.position.x   = self._x
        odom.pose.pose.position.y   = self._y
        odom.pose.pose.position.z   = 0.0
        odom.pose.pose.orientation  = self._quaternon
        #odom.pose.pose.orientation.x = 0.0
        #odom.pose.pose.orientation.y = 0.0
        #odom.pose.pose.orientation.z = math.sin(self._yaw/2.0)
        #odom.pose.pose.orientation.w = math.cos(self._yaw/2.0)

        # Position uncertainty
        # @todo Think about position uncertainty, perhaps get from parameters? */
        odom.pose.covariance[0]  = 0.2 ##< x
        odom.pose.covariance[7]  = 0.2 ##< y
        odom.pose.covariance[35] = 0.4 ##< yaw

        # Velocity ("in the coordinate frame given by the child_frame_id")
        odom.twist.twist.linear.x   = current_speed
        odom.twist.twist.linear.y   = 0.0
        odom.twist.twist.angular.z  = current_angular_velocity

        # Velocity uncertainty
        # @todo Think about velocity uncertainty */

        if (self._publish_tf):
            tf = TransformStamped()
            tf.header.stamp     = self._current_time
            tf.header.frame_id  = self._odom_frame
            tf.child_frame_id   = self._base_frame
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = self._quaternon
            self.broadcaster.sendTransform(tf)

        self._odom_pub.publish(odom)

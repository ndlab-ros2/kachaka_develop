#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

# Imports for QoS settings
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class KachakaFeedbackControl(Node):
    def __init__(self):
        super().__init__('kachaka_feedback_control')

        # Define a QoS profile with BEST_EFFORT reliability
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/kachaka/manual_control/cmd_vel', 10)

        # Subscriber for odometry data with BEST_EFFORT QoS
        self.odom_sub = self.create_subscription(
            Odometry,
            '/kachaka/odometry/odometry',
            self.callback_odom,
            qos_profile
        )

        self.x = None
        self.y = None
        self.yaw = None

        # Wait for the first odometry message to be received
        while self.x is None or self.y is None or self.yaw is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def callback_odom(self, msg):
        """Callback function for receiving odometry data."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)

    def get_yaw_from_quaternion(self, q):
        """
        Convert geometry_msgs/Quaternion to yaw angle in radians.
        ROS2 標準方式で tf_transformations に依存しない。
        """
        # q = geometry_msgs.msg.Quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def go_straight(self, distance, velocity=0.3):
        vel = Twist()
        x0 = self.x
        y0 = self.y
        while np.sqrt((self.x - x0)**2 + (self.y - y0)**2) < distance:
            vel.linear.x = velocity
            vel.angular.z = 0.0
            self.cmd_vel_pub.publish(vel)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.stop()

    def turn_right(self, angle_degree, yawrate=-0.5):
        vel = Twist()
        yaw0 = self.yaw
        target_angle = math.radians(angle_degree)
        while abs(math.atan2(math.sin(self.yaw - yaw0), math.cos(self.yaw - yaw0))) < target_angle:
            vel.linear.x = 0.0
            vel.angular.z = yawrate
            self.cmd_vel_pub.publish(vel)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.stop()

    def turn_left(self, angle_degree, yawrate=0.5):
        vel = Twist()
        yaw0 = self.yaw
        target_angle = math.radians(angle_degree)
        while abs(math.atan2(math.sin(self.yaw - yaw0), math.cos(self.yaw - yaw0))) < target_angle:
            vel.linear.x = 0.0
            vel.angular.z = yawrate
            self.cmd_vel_pub.publish(vel)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.stop()

    def stop(self):
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.cmd_vel_pub.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    kachaka_feedback_control = KachakaFeedbackControl()

    try:
        # Example movement commands: go straight, turn left, then turn right
        kachaka_feedback_control.go_straight(1.0)
        kachaka_feedback_control.turn_left(90)
        kachaka_feedback_control.turn_right(90)
    finally:
        kachaka_feedback_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

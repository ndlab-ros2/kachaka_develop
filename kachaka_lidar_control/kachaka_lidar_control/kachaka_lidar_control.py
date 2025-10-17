#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
# Import for QoS settings
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

def normalize_angle(angle):
    """
    Normalize angle to the range [-pi, pi).
    """
    return (angle + np.pi) % (2.0 * np.pi) - np.pi

class KachakaLidarControl(Node):
    def __init__(self):
        super().__init__('kachaka_lidar_control')

        # Create QoS profile for LaserScan subscription
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/kachaka/manual_control/cmd_vel', 10)

        # Subscriber for LiDAR scan data
        self.scan_sub = self.create_subscription(LaserScan, '/kachaka/lidar/scan', self.callback_scan, qos_profile)

        # Initialize minimum distance variables for each sector
        self.front_min = None
        self.right_min = None
        self.left_min = None

        # Create a timer to call process function at 10Hz
        self.timer = self.create_timer(0.1, self.process)

    def callback_scan(self, data):
        """
          -  Front sector: -15° to +15°     ( -0.26 rad to +0.26 rad )
          -  Right sector: -180° to -15°    ( -3.14 rad to -0.26 rad )
          -  Left sector:  +15° to +180°    ( +0.26 rad to +3.14 rad )
        """

        # Sector boundaries in the shifted coordinate system
        front_min_angle = np.deg2rad(-15)   # -15°
        front_max_angle = np.deg2rad(15)    # +15°
        right_min_angle = -np.pi            # -180°
        right_max_angle = np.deg2rad(-15)   # -15°
        left_min_angle  = np.deg2rad(15)    # +15°
        left_max_angle  = np.pi             # +180°

        # Initialize minimum distances with sensor's maximum range
        front_min = data.range_max
        right_min = data.range_max
        left_min  = data.range_max

        # Iterate through each distance reading and calculate shifted angle
        angle = data.angle_min
        for r in data.ranges:
            # Skip invalid readings
            if r == 0.0 or r < data.range_min:
                angle += data.angle_increment
                continue

            # Shift angle by +pi/2 so that "front" is 0 in the code
            shifted_angle = angle + np.pi / 2
            shifted_angle = normalize_angle(shifted_angle)

            # Check which sector the shifted angle belongs to
            if front_min_angle <= shifted_angle <= front_max_angle:
                # Front sector
                if r < front_min:
                    front_min = r
            elif right_min_angle <= shifted_angle < right_max_angle:
                # Right sector
                if r < right_min:
                    right_min = r
            elif left_min_angle < shifted_angle <= left_max_angle:
                # Left sector
                if r < left_min:
                    left_min = r

            angle += data.angle_increment

        # Save results for use in process()
        self.front_min = front_min
        self.right_min = right_min
        self.left_min  = left_min

        # Debug log
        self.get_logger().info(
            f"Front: {front_min:.2f} m, Right: {right_min:.2f} m, Left: {left_min:.2f} m"
        )

    def process(self):
        """
        Movement logic:
          - If front sector is clear (> threshold), move forward.
          - Otherwise, if right sector is blocked, rotate left.
          - Otherwise, if left sector is blocked, rotate right.
          - Otherwise, stop.
        """
        vel = Twist()
        threshold = 1.0  # meters

        if self.front_min is not None and self.front_min > threshold:
            # Front is clear
            vel.linear.x = 0.2
            vel.angular.z = 0.0
        else:
            # Front is blocked or invalid
            if self.right_min is not None and self.right_min < threshold:
                # Obstacle on right => rotate left
                vel.linear.x = 0.0
                vel.angular.z = 0.5
            elif self.left_min is not None and self.left_min < threshold:
                # Obstacle on left => rotate right
                vel.linear.x = 0.0
                vel.angular.z = -0.5
            else:
                # No clear direction => stop
                vel.linear.x = 0.0
                vel.angular.z = 0.0

        self.cmd_vel_pub.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = KachakaLidarControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
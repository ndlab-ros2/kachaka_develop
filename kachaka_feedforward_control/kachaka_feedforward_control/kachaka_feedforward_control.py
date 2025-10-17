#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.duration import Duration

class KachakaFeedforwardControl(Node):
    def __init__(self):
        super().__init__('kachaka_feedforward_control')
        # Create a Publisher that sends Twist type messages to the /kachaka/manual_control/cmd_vel topic
        self.publisher = self.create_publisher(Twist, '/kachaka/manual_control/cmd_vel', 10)

    def time_control(self, velocity, yawrate, time_seconds):
        """
        Control the robot at specified speed and time
        
        Parameters:
            velocity: Forward speed [m/s]
            yawrate: Rotation speed [rad/s]
            time_seconds: Movement duration [seconds]
        """
        # Create velocity command message
        vel = Twist()
        vel.linear.x = velocity
        vel.angular.z = yawrate
        
        # Calculate end time
        end_time = self.get_clock().now() + Duration(seconds=time_seconds)
        
        # Continue publishing velocity commands until the specified time
        while self.get_clock().now() < end_time:
            self.publisher.publish(vel)
            rclpy.spin_once(self, timeout_sec=0.1)  # Wait for 0.1 seconds

def main(args=None):
    rclpy.init(args=args)
    kachaka_feedforward_control = KachakaFeedforwardControl()

    try:
        # Execute the following movement sequence
        kachaka_feedforward_control.time_control(0.0, 0.0, 0.5)  # Stop
        kachaka_feedforward_control.time_control(0.1, 0.0, 2.0)  # Forward
        kachaka_feedforward_control.time_control(0.0, 0.0, 0.5)  # Stop
        kachaka_feedforward_control.time_control(-0.1, 0.0, 2.0)  # Backward
        kachaka_feedforward_control.time_control(0.0, 0.0, 0.5)  # Stop
        kachaka_feedforward_control.time_control(0.0, 0.5, 2.0)  # Rotate left
        kachaka_feedforward_control.time_control(0.0, 0.0, 0.5)  # Stop
        kachaka_feedforward_control.time_control(0.0, -0.5, 2.0)  # Rotate right
    except KeyboardInterrupt:
        pass
    finally:
        kachaka_feedforward_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
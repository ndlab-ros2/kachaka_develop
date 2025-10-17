#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.duration import Duration

class KachakaSimpleControl(Node):
    def __init__(self):
        super().__init__('kachaka_simple_control')
        # Create a Publisher that sends Twist type messages to the /cmd_vel topic
        self.publisher = self.create_publisher(Twist, '/kachaka/manual_control/cmd_vel', 10)
        self.get_logger().info('Robot control node has started')

    def move_robot(self, linear_x, angular_z, duration):
        """
        Move the robot at specified speed and time
        
        Parameters:
            linear_x: Forward speed [m/s]
            angular_z: Rotation speed [rad/s]
            duration: Movement duration [seconds]
        """
        # Create velocity command message
        vel = Twist()
        vel.linear.x = linear_x
        vel.angular.z = angular_z
        
        # Calculate end time
        end_time = self.get_clock().now() + Duration(seconds=duration)
        
        # Continue publishing velocity commands until the specified time
        while self.get_clock().now() < end_time:
            self.publisher.publish(vel)
            rclpy.spin_once(self, timeout_sec=0.1)  # Wait for 0.1 seconds
        
        # Stop after the movement is complete
        self.stop_robot()
        self.get_logger().info(f'Movement completed: Forward={linear_x}, Rotation={angular_z}, Time={duration} seconds')
    
    def stop_robot(self):
        """Stop the robot"""
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.publisher.publish(vel)
        self.get_logger().info('Robot has stopped')

def main(args=None):
    rclpy.init(args=args)
    robot_control = KachakaSimpleControl()

    try:
        # Execute the following movement sequence
        robot_control.move_robot(0.0, 0.0, 1.0)  # Stop
        robot_control.move_robot(0.1, 0.0, 1.0)  # Forward
        robot_control.move_robot(0.0, 0.0, 1.0)  # Stop
        robot_control.move_robot(0.0, 0.5, 2.0)  # Rotate left
        robot_control.move_robot(0.0, 0.0, 1.0)  # Stop
        robot_control.move_robot(0.0, -0.5, 2.0)  # Rotate right
        robot_control.move_robot(0.0, 0.0, 1.0)  # Stop
    except KeyboardInterrupt:
        robot_control.get_logger().info('Program was interrupted')
    finally:
        robot_control.stop_robot()
        robot_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

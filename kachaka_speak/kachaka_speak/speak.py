#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from kachaka_interfaces.action import ExecKachakaCommand
from kachaka_interfaces.msg import KachakaCommand
from rclpy.action import ActionClient
from rclpy.node import Node


class Speak(Node):
    """
    Node for making Kachaka robot speak
    """
    def __init__(self) -> None:
        """
        Node initialization
        - Set up action client
        - Wait for server
        """
        super().__init__("speak")
        self._action_client = ActionClient(
            self, ExecKachakaCommand, "/kachaka/kachaka_command/execute"
        )
        self._action_client.wait_for_server()

    def send_goal(self):
        """
        Send speech command
        Returns:
            Future: Future object representing the action execution result
        """
        # Create command message
        command = KachakaCommand()
        command.command_type = KachakaCommand.SPEAK_COMMAND
        command.speak_command_text = "こんにちは、カチャカです"

        # Create goal message
        goal_msg = ExecKachakaCommand.Goal()
        goal_msg.kachaka_command = command

        # Send goal asynchronously
        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    """
    Main function
    - Initialize ROS2
    - Create and run node
    - Release resources
    """
    # Initialize ROS2
    rclpy.init(args=args)

    # Create node
    speak = Speak()
    
    # Send speech command
    future = speak.send_goal()
    
    # Wait for command completion
    rclpy.spin_until_future_complete(speak, future)

    # Destroy node
    speak.destroy_node()
    
    # Shutdown ROS2
    rclpy.shutdown()


if __name__ == "__main__":
    main()
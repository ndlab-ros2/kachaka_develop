#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from kachaka_interfaces.action import ExecKachakaCommand
from kachaka_interfaces.msg import KachakaCommand
from rclpy.action import ActionClient
import json
import time


class KachakaSpeakDetection(Node):
    """
    Node for speaking detected objects
    """
    def __init__(self):
        super().__init__('kachaka_speak_detection')
        
        # Set QoS profile for object detection data
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to object detection information from Lesson7
        self.subscription = self.create_subscription(
            String,
            '/image_yolo_detection/objects',  # Topic from Lesson7
            self.object_callback,
            qos
        )

        # Action client for speech output
        self._action_client = ActionClient(
            self, ExecKachakaCommand, "/kachaka/kachaka_command/execute"
        )
        self._action_client.wait_for_server()
        
        # Dictionary for converting English object names to Japanese
        self.japanese_names = {
            'person': '人',
            'car': '車',
            'chair': '椅子',
            'bottle': 'ボトル',
            'cup': 'カップ',
            'laptop': 'ノートパソコン',
            'mouse': 'マウス',
            'keyboard': 'キーボード',
            'cell phone': 'スマートフォン',
            'book': '本',
            # Add more as needed
        }
        # Store the last detected objects
        self.last_objects = []
        
        # Create a timer to periodically speak the detected objects
        self.timer = self.create_timer(5.0, self.timer_callback)
        
        self.get_logger().info("Kachaka speak detection node has started.")

    def object_callback(self, msg):
        """
        Process detected object information and store it
        """
        try:
            # Convert JSON string to Python object
            object_info = json.loads(msg.data)
            self.last_objects = object_info.get("objects", [])
            
        except Exception as e:
            self.get_logger().error(f"Error processing object information: {str(e)}")
            
    def timer_callback(self):
        """
        Periodically speak the detected objects
        """
        if not self.last_objects:
            return
            
        # Convert all detected objects to Japanese and create a summary message
        japanese_objects = []
        for obj in self.last_objects:
            japanese_name = self.japanese_names.get(obj, obj)
            japanese_objects.append(japanese_name)
        
        # Create a summary message
        if len(japanese_objects) == 1:
            text = f"{japanese_objects[0]}を検出しました"
        else:
            text = f"{', '.join(japanese_objects[:-1])}と{japanese_objects[-1]}を検出しました"
                
        # Execute speech output

        self.send_goal(text)
        
        self.get_logger().info(f"Speech output: {text}")

    def send_goal(self, text):
        command = KachakaCommand()
        command.command_type = KachakaCommand.SPEAK_COMMAND
        command.speak_command_text = text

        goal_msg = ExecKachakaCommand.Goal()
        goal_msg.kachaka_command = command

        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return
        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result}")


def main(args=None):
    rclpy.init(args=args)
    node = KachakaSpeakDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
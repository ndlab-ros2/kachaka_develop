#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import asyncio

import kachaka_api

class KachakaApiNode(Node):
    def __init__(self):
        super().__init__('kachaka_api_node')

        # 非同期イベントループ用
        self.loop = asyncio.get_event_loop()
        self.client = kachaka_api.aio.KachakaApiClient()

        # 起動時に非同期タスクを実行
        self.loop.create_task(self.init_robot_pose())

    async def init_robot_pose(self):
        # 現在の姿勢取得
        pose = await self.client.get_robot_pose()
        self.get_logger().info(f"現在の姿勢: x={pose.x:.2f}, y={pose.y:.2f}, theta={pose.theta:.2f}")

        # 姿勢を設定
        result = await self.client.set_robot_pose({"x": 0.0, "y": 1.0, "theta": 0.0})
        self.get_logger().info(f"set_robot_pose({0.0, 1.0, 0.0}) -> success={result.success}")

        result = await self.client.set_robot_pose({"x": 2.0, "y": 1.0, "theta": 0.0})
        self.get_logger().info(f"set_robot_pose({2.0, 1.0, 0.0}) -> success={result.success}")

        # 反映を待つ
        await asyncio.sleep(5)

        pose = await self.client.get_robot_pose()
        self.get_logger().info(f"最終姿勢: x={pose.x:.2f}, y={pose.y:.2f}, theta={pose.theta:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = KachakaApiNode()

    try:
        # rclpy.spin と asyncio を統合
        asyncio.get_event_loop().run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
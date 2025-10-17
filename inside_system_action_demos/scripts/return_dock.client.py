#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from kachaka_interfaces.msg import KachakaCommand
from kachaka_interfaces.action import ExecKachakaCommand
from rclpy.action import ActionClient


class KachakaSpeakAndDock(Node):
    def __init__(self):
        super().__init__('kachaka_speak_and_dockking')

        # アクションクライアント
        self._action_client = ActionClient(
            self, ExecKachakaCommand, "/kachaka/kachaka_command/execute"
        )
        self.get_logger().info("アクションクライアントを初期化中...")
        self._action_client.wait_for_server()
        self.get_logger().info("アクションサーバーに接続しました")

        # 実行ステージ管理
        self.stage = 0

    def send_command(self, command: KachakaCommand):
        # """
        # 共通関数: KachakaCommandをアクションとして送信し、完了まで待機
        # """
        goal_msg = ExecKachakaCommand.Goal()
        goal_msg.kachaka_command = command

        self.get_logger().info(f"Goalを送信: {command.command_type}")
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goalが拒否されました")
            return False

        # 完了待ち
        self.get_logger().info("Goalを受理しました。完了を待機中...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        self.get_logger().info(f"Goal完了: {command.command_type}")
        return True

    def run(self):
        # """
        # 発話 → 移動 → ドッキング の順で実行
        # """
        
        # 1. 発話
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "充電ドックに戻らせていただきます。よろしくお願いします"

        self.get_logger().info("✅ 発話完了")

        if not self.send_command(speak_command):
            return

        # 2. 移動
         # 移動 (アクション経由)　　シェルフの手前で停止するようにする！
        self.get_logger().info("📡 MOVE_TO_LOCATION_COMMAND を送信しました")
        self.get_logger().info("✅ 移動開始")

        command = KachakaCommand()
        command.command_type = KachakaCommand.MOVE_TO_LOCATION_COMMAND
        command.move_to_location_command_target_location_id = "home"

        goal_msg = ExecKachakaCommand.Goal()
        goal_msg.kachaka_command = command
        # future = self._action_client.send_goal_async(goal_msg)
        # return future

        self.get_logger().info("✅ 移動完了")

        if not self.send_command(command):
            return

        self.get_logger().info("すべてのステージが完了しました")


def main(args=None):
    rclpy.init(args=args)
    node = KachakaSpeakAndDock()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

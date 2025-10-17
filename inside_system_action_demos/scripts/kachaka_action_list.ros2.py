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

        #1.開始の発話
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND

        #これからどのような行動を取るのかを発話する
        speak_command.speak_command_text = "シェルフとのドッキングと移動を開始します" 

        self.get_logger().info("✅ 開始の発話中...")

        if not self.send_command(speak_command):
            return


        #2.指定位置までの移動
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "指定された位置まで移動します。ご注意ください"

        self.get_logger().info("✅ 指定位置への移動の発話中...")

        if not self.send_command(speak_command):
            return

        self.get_logger().info("📡 MOVE_TO_LOCATION_COMMAND を送信しました")
        self.get_logger().info("✅ 指定位置への移動開始")

        # 移動先座標を定義(シェルフのホームの手前の位置)
        pos_x = 0.749277
        pos_y = 1.25
        yaw   = 3.140458

        move_command_1 = KachakaCommand()
        move_command_1.command_type = KachakaCommand.MOVE_TO_POSE_COMMAND
        move_command_1.move_to_pose_command_x = pos_x
        move_command_1.move_to_pose_command_y = pos_y
        move_command_1.move_to_pose_command_yaw = yaw
        goal_msg = ExecKachakaCommand.Goal()
        goal_msg.kachaka_command = move_command_1
        # future = self._action_client.send_goal_async(goal_msg)
        # return future

        if not self.send_command(move_command_1):
            return
        
        self.get_logger().info("✅ 指定位置への移動完了")
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "移動完了。次の動作に移行します"

        self.get_logger().info("✅ 指定位置への移動終了の発話中...")

        if not self.send_command(speak_command):
            return


        #3.ドッキング
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "続いて、ドッキングを開始します"

        self.get_logger().info("✅ ドッキング開始の発話中...")

        if not self.send_command(speak_command):
            return  

        self.get_logger().info("📡 DOCK_SHELF_COMMAND を送信しました")
        self.get_logger().info("✅ ドッキング開始")

        dock_command = KachakaCommand()
        dock_command.command_type = KachakaCommand.DOCK_SHELF_COMMAND
        dock_command.undock_shelf_command_target_shelf_id = "S03"
        if not self.send_command(dock_command):
            return
        
        self.get_logger().info("✅ ドッキング完了")
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "ドッキング完了。次の動作に移行します"

        self.get_logger().info("✅ ドッキング終了の発話中...")

        if not self.send_command(speak_command):
            return
        

        #4.ホームまで移動
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "続いて、ホームまで移動します。ご注意ください"

        self.get_logger().info("✅ ホームへの移動開始の発話中...")

        if not self.send_command(speak_command):
            return  
        
        self.get_logger().info("📡 MOVE_TO_LOCATION_COMMAND を送信しました")
        self.get_logger().info("✅ ホームへの移動開始")

        #指定座標までの移動方法

        # # 移動先座標を定義(ホームの位置)
        # pos_x = 1.35811
        # pos_y = -0.034669
        # yaw   = -1.5707959999999999

        # move_command_2 = KachakaCommand()
        # move_command_2.command_type = KachakaCommand.MOVE_TO_POSE_COMMAND
        # move_command_2.move_to_pose_command_x = pos_x
        # move_command_2.move_to_pose_command_y = pos_y
        # move_command_2.move_to_pose_command_yaw = yaw
        # goal_msg = ExecKachakaCommand.Goal()
        # goal_msg.kachaka_command = move_command_2


        #目的地をターゲットにした移動方法

        move_command_2 = KachakaCommand()
        move_command_2.command_type = KachakaCommand.MOVE_TO_LOCATION_COMMAND
        move_command_2.move_to_location_command_target_location_id = "L01"

        goal_msg = ExecKachakaCommand.Goal()
        goal_msg.kachaka_command = move_command_2

        if not self.send_command(move_command_2):
            return
        
        self.get_logger().info("✅ ホームへの移動完了")
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "移動完了。次の動作に移行します"

        self.get_logger().info("✅ ホームへの移動終了の発話中...")

        if not self.send_command(speak_command):
            return


        #5.ホームでのドッキング解除
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "続いて、ドッキングの解除を行います"

        self.get_logger().info("✅ ドッキング開始の発話中...")

        if not self.send_command(speak_command):
            return  
        
        self.get_logger().info("📡 UNDOCK_SHELF_COMMAND を送信しました")
        undock_command = KachakaCommand()
        undock_command.command_type = KachakaCommand.UNDOCK_SHELF_COMMAND
        undock_command.undock_shelf_command_target_shelf_id = "S03"
        if not self.send_command(undock_command):
            return
        
        self.get_logger().info("✅ ドッキング解除完了")
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "ドッキング解除完了。次の動作に移行します"

        self.get_logger().info("✅ ドッキング解除終了の発話中...")

        if not self.send_command(speak_command):
            return
        

        #6.指定位置までの移動
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "指定された位置まで移動します。ご注意ください"

        self.get_logger().info("✅ 指定位置への移動の発話中...")

        if not self.send_command(speak_command):
            return

        self.get_logger().info("📡 MOVE_TO_LOCATION_COMMAND を送信しました")
        self.get_logger().info("✅ 指定位置への移動開始")

        # 移動先座標を定義(任意位置座標)
        pos_x = 0.849277
        pos_y = 1.25
        yaw   = 3.140458

        move_command_3 = KachakaCommand()
        move_command_3.command_type = KachakaCommand.MOVE_TO_POSE_COMMAND
        move_command_3.move_to_pose_command_x = pos_x
        move_command_3.move_to_pose_command_y = pos_y
        move_command_3.move_to_pose_command_yaw = yaw
        goal_msg = ExecKachakaCommand.Goal()
        goal_msg.kachaka_command = move_command_3
        # future = self._action_client.send_goal_async(goal_msg)
        # return future

        if not self.send_command(move_command_3):
            return
        
        self.get_logger().info("✅ 指定位置への移動完了")
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "移動完了。次の動作に移行します"

        self.get_logger().info("✅ 指定位置への移動終了の発話中...")

        if not self.send_command(speak_command):
            return
        

        #7.指定位置までの移動
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "指定された位置まで移動します。ご注意ください"

        self.get_logger().info("✅ 指定位置への移動の発話中...")

        if not self.send_command(speak_command):
            return

        self.get_logger().info("📡 MOVE_TO_LOCATION_COMMAND を送信しました")
        self.get_logger().info("✅ 指定位置への移動開始")

        # 移動先座標を定義(ホームの手前の位置)
        pos_x = 1.35811
        pos_y = 0.665331
        yaw   = -1.5707959999999999

        move_command_4 = KachakaCommand()
        move_command_4.command_type = KachakaCommand.MOVE_TO_POSE_COMMAND
        move_command_4.move_to_pose_command_x = pos_x
        move_command_4.move_to_pose_command_y = pos_y
        move_command_4.move_to_pose_command_yaw = yaw
        goal_msg = ExecKachakaCommand.Goal()
        goal_msg.kachaka_command = move_command_4
        # future = self._action_client.send_goal_async(goal_msg)
        # return future

        if not self.send_command(move_command_4):
            return
        
        self.get_logger().info("✅ 指定位置への移動完了")
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "移動完了。次の動作に移行します"

        self.get_logger().info("✅ 指定位置への移動終了の発話中...")

        if not self.send_command(speak_command):
            return
        

        #8.ホームでのドッキング
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "続いて、ドッキングを開始します"

        self.get_logger().info("✅ ドッキング開始の発話中...")

        if not self.send_command(speak_command):
            return  

        self.get_logger().info("📡 DOCK_SHELF_COMMAND を送信しました")
        self.get_logger().info("✅ ドッキング開始")

        dock_command = KachakaCommand()
        dock_command.command_type = KachakaCommand.DOCK_SHELF_COMMAND
        dock_command.undock_shelf_command_target_shelf_id = "S03"
        if not self.send_command(dock_command):
            return
        
        self.get_logger().info("✅ ドッキング完了")
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "ドッキング完了。次の動作に移行します"

        self.get_logger().info("✅ ドッキング終了の発話中...")

        if not self.send_command(speak_command):
            return
        

        #9.シェルフのホームまで移動
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "続いて、シェルフのホームまで移動します。ご注意ください"

        self.get_logger().info("✅ シェルフホームへの移動開始の発話中...")

        if not self.send_command(speak_command):
            return  
        
        self.get_logger().info("📡 MOVE_TO_LOCATION_COMMAND を送信しました")
        self.get_logger().info("✅ シェルフホームへの移動開始")

        #指定座標までの移動方法

        # # 移動先座標を定義(シェルフホームの位置)
        # pos_x = 0.049277
        # pos_y = 1.244201
        # yaw   = 3.140458

        # move_command_5 = KachakaCommand()
        # move_command_5.command_type = KachakaCommand.MOVE_TO_POSE_COMMAND
        # move_command_5.move_to_pose_command_x = pos_x
        # move_command_5.move_to_pose_command_y = pos_y
        # move_command_5.move_to_pose_command_yaw = yaw
        # goal_msg = ExecKachakaCommand.Goal()
        # goal_msg.kachaka_command = move_command_5

        #目的地をターゲットにした移動方法

        move_command_5 = KachakaCommand()
        move_command_5.command_type = KachakaCommand.MOVE_TO_LOCATION_COMMAND
        move_command_5.move_to_location_command_target_location_id = "S03_home"

        goal_msg = ExecKachakaCommand.Goal()
        goal_msg.kachaka_command = move_command_5

        if not self.send_command(move_command_5):
            return
        
        self.get_logger().info("✅ シェルフホームへの移動完了")
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "移動完了。次の動作に移行します"

        self.get_logger().info("✅ シェルフホームへの移動終了の発話中...")

        if not self.send_command(speak_command):
            return
        

        #10.半回転(シェルフを置く位置の前方に障害物がある時に有効)
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "半回転します"

        self.get_logger().info("✅ 半回転の合図の発話中...")

        if not self.send_command(speak_command):
            return

        self.get_logger().info("📡 MOVE_TO_LOCATION_COMMAND を送信しました")
        self.get_logger().info("✅ 半回転開始")

        # 移動先座標を定義(半回転の位置座標)
        pos_x = 0.129277
        pos_y = 1.294201
        yaw   = 3.140458

        move_command_6 = KachakaCommand()
        move_command_6.command_type = KachakaCommand.MOVE_TO_POSE_COMMAND
        move_command_6.move_to_pose_command_x = pos_x
        move_command_6.move_to_pose_command_y = pos_y
        move_command_6.move_to_pose_command_yaw = yaw
        goal_msg = ExecKachakaCommand.Goal()
        goal_msg.kachaka_command = move_command_6
        # future = self._action_client.send_goal_async(goal_msg)
        # return future

        if not self.send_command(move_command_6):
            return
        
        self.get_logger().info("✅ 半回転完了")
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "半回転完了。次の動作に移行します"

        self.get_logger().info("✅ 半回転終了の発話中...")

        if not self.send_command(speak_command):
            return
        

        #11.ドッキング解除
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "続いて、ドッキングの解除を行います"

        self.get_logger().info("✅ ドッキング解除開始の発話中...")

        if not self.send_command(speak_command):
            return  
        
        self.get_logger().info("📡 UNDOCK_SHELF_COMMAND を送信しました")
        undock_command = KachakaCommand()
        undock_command.command_type = KachakaCommand.UNDOCK_SHELF_COMMAND
        undock_command.undock_shelf_command_target_shelf_id = "S03"
        if not self.send_command(undock_command):
            return
        
        self.get_logger().info("✅ ドッキング解除完了")
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "ドッキング解除完了。次の動作に移行します"

        self.get_logger().info("✅ ドッキング解除終了の発話中...")

        if not self.send_command(speak_command):
            return


        #12.充電ドックに戻る
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "最後に、充電ドックへ移動します。ご注意ください"

        self.get_logger().info("✅ 充電ドックへの移動開始の発話中...")

        if not self.send_command(speak_command):
            return  
        
        self.get_logger().info("📡 MOVE_TO_LOCATION_COMMAND を送信しました")
        self.get_logger().info("✅ 充電ドックへの移動開始")

        move_command_7 = KachakaCommand()
        move_command_7.command_type = KachakaCommand.MOVE_TO_LOCATION_COMMAND
        move_command_7.move_to_location_command_target_location_id = "home"

        goal_msg = ExecKachakaCommand.Goal()
        goal_msg.kachaka_command = move_command_7
        # future = self._action_client.send_goal_async(goal_msg)
        # return future

        if not self.send_command(move_command_7):
            return
        
        self.get_logger().info("✅ 充電ドックへの移動完了")
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "移動完了。"

        self.get_logger().info("✅ 充電ドックへの移動終了の発話中...")

        if not self.send_command(speak_command):
            return


        #13.最後の発話
        self.get_logger().info("すべてのステージが完了しました")
        self.get_logger().info("🗣 SPEAK_COMMAND を送信しました")

        speak_command = KachakaCommand()
        speak_command.command_type = KachakaCommand.SPEAK_COMMAND
        speak_command.speak_command_text = "これにてすべてのステージが完了しました。お疲れ様でした。"

        self.get_logger().info("✅ 最後の発話中...")

        if not self.send_command(speak_command):
            return  


def main(args=None):
    rclpy.init(args=args)
    node = KachakaSpeakAndDock()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

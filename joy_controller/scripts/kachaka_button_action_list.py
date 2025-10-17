#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from kachaka_interfaces.msg import KachakaCommand
from kachaka_interfaces.action import ExecKachakaCommand
from action_msgs.msg import GoalStatus


class KachakaJoyControl(Node):
    def __init__(self):
        super().__init__('kachaka_joy_control')

        # 前回ボタン状態を保持
        self.prev_buttons = []

        # アクションクライアント
        self._action_client = ActionClient(
            self, ExecKachakaCommand, "/kachaka/kachaka_command/execute"
        )
        self.get_logger().info("アクションクライアントを初期化中...")
        self._action_client.wait_for_server()
        self.get_logger().info("アクションサーバーに接続しました")

        # ジョイスティック購読
        self.joy_sub = self.create_subscription(
            Joy, "/operation/joy_linux/joy", self.joy_callback, 10
        )

        # cmd_vel publisher（R1 + スティックで操作）
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.axis_linear_x = 1    # 左スティック上下
        self.axis_angular_z = 3   # 左スティック左右
        self.linear_scale = 0.5
        self.angular_scale = 0.75

        # ボタン番号設定
        self.move_shelf_home_button_idx = 0   # ☓ボタン
        self.move_charging_dock_button_idx = 1  # ◯ボタン
        self.move_home_button_idx = 2  # △ボタン
        self.localization_button_idx = 3  # □ボタン

        self.deadline_button_idx = 5  # R1ボタン
        self.dokking_button_idx = 6  # L2ボタン
        self.undokking_button_idx = 7  # R2ボタン
        self.half_turn_button_idx = 8  # SHAREボタン
        self.speak_button_idx = 9  # OPTIONSボタン
        self.Forced_termination_button_idx = 10  # ロゴボタン
        self.move_specified_position_1_button_idx = 11  # L3
        self.move_specified_position_2_button_idx = 12  # R3

        # 現在のゴールハンドル
        self.current_goal_handle = None

    def send_command(self, command: KachakaCommand):
        """共通関数: KachakaCommandを非同期で送信"""
        goal_msg = ExecKachakaCommand.Goal()
        goal_msg.kachaka_command = command

        self.get_logger().info(f"Goalを送信: {command.command_type}")
        send_future = self._action_client.send_goal_async(goal_msg)

        # goal送信後のコールバック
        def goal_response_callback(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().error("Goalが拒否されました")
                return
            self.get_logger().info("Goalを受理しました。完了を待機中...")
            self.current_goal_handle = goal_handle

            # 結果待ちも非同期で処理
            result_future = goal_handle.get_result_async()

            def result_callback(r):
                result_msg = r.result()
                status = result_msg.status
                # 実際の結果は result_msg.result で取得可能
                self.get_logger().info(
                    f"Goal完了: {command.command_type}, status={status}"
                )
                # 完了後は必ずクリア
                self.current_goal_handle = None

            result_future.add_done_callback(result_callback)

        send_future.add_done_callback(goal_response_callback)

    def joy_callback(self, joy_msg: Joy):
        buttons = joy_msg.buttons
        axes = joy_msg.axes

        # 初回のみ前回状態を初期化
        if not self.prev_buttons:
            self.prev_buttons = [0] * len(buttons)

        # R1ボタン（手動操作デッドマン）
        R1_pressed = self.deadline_button_idx < len(buttons) and buttons[self.deadline_button_idx]

        # 各種ボタンの立ち上がりエッジ検出
        cross_pressed = self._is_pressed(self.move_shelf_home_button_idx, buttons)
        circle_pressed = self._is_pressed(self.move_charging_dock_button_idx, buttons)
        triangle_pressed = self._is_pressed(self.move_home_button_idx, buttons)
        square_pressed = self._is_pressed(self.localization_button_idx, buttons)
        L2_pressed = self._is_pressed(self.dokking_button_idx, buttons)
        R2_pressed = self._is_pressed(self.undokking_button_idx, buttons)
        share_pressed = self._is_pressed(self.half_turn_button_idx, buttons)
        options_pressed = self._is_pressed(self.speak_button_idx, buttons)
        logo_pressed = self._is_pressed(self.Forced_termination_button_idx, buttons)
        leftpush_pressed = self._is_pressed(self.move_specified_position_1_button_idx, buttons)
        rightpush_pressed = self._is_pressed(self.move_specified_position_2_button_idx, buttons)

        # --- 手動操作処理 ---
        if R1_pressed:
            twist = Twist()
            twist.linear.x = self.linear_scale * axes[self.axis_linear_x]
            twist.angular.z = self.angular_scale * axes[self.axis_angular_z]
            self.cmd_vel_pub.publish(twist)
        else:
            # R1を離した瞬間に停止
            if self.prev_buttons[self.deadline_button_idx] == 1:
                self.cmd_vel_pub.publish(Twist())

        # --- R1押下時のコマンド処理 ---
        if R1_pressed:

            # ☓ボタンでホームへ移動
            """[[[最初に絶対このボタン押す!!!そうでないと、
            全ての移動系のコマンドがコード起動中は移動せず、
            コードの起動が終了した時に移動し始めるというエラーが生じる"""

            if cross_pressed:

                #目的地をターゲットにした移動方法
                cmd = KachakaCommand()
                cmd.command_type = KachakaCommand.MOVE_TO_LOCATION_COMMAND
                cmd.move_to_location_command_target_location_id = "L01"
                self.send_command(cmd)

                # # 指定座標までの移動方法
                # move_cmd = KachakaCommand()
                # move_cmd.command_type = KachakaCommand.MOVE_TO_POSE_COMMAND

                # # 移動先座標を定義(ホームの位置座標)
                # move_cmd.move_to_pose_command_x = 1.35811
                # move_cmd.move_to_pose_command_y = -0.034669
                # move_cmd.move_to_pose_command_yaw = -1.5707959999999999
                # self.send_command(move_cmd)

            # ◯ボタンで発話ver.1を行う
            #!!!◯ボタンに移動系のコマンドを入れるとコード起動中は移動せず、コードの起動が終了した時に移動し始めるため、移動系のコマンドは入れないようにする!!!
            if circle_pressed:
                cmd = KachakaCommand()
                cmd.command_type = KachakaCommand.SPEAK_COMMAND
                cmd.speak_command_text = "現在コントローラーでの操作中です。ご注意ください"
                self.send_command(cmd)

            # △ボタンでシェルフのホームへ移動
            if triangle_pressed:

                #目的地をターゲットにした移動方法
                cmd = KachakaCommand()
                cmd.command_type = KachakaCommand.MOVE_TO_LOCATION_COMMAND
                cmd.move_to_location_command_target_location_id = "S03_home"
                self.send_command(cmd)

                # # 指定座標までの移動方法
                # move_cmd = KachakaCommand()
                # move_cmd.command_type = KachakaCommand.MOVE_TO_POSE_COMMAND

                # # 移動先座標を定義(シェルフのホームの位置座標)
                # move_cmd.move_to_pose_command_x = 0.049277
                # move_cmd.move_to_pose_command_y = 1.244201
                # move_cmd.move_to_pose_command_yaw = -0.001135
                # self.send_command(move_cmd)
            
            # □ボタンで充電ドックへ移動
            if square_pressed:
                cmd = KachakaCommand()
                cmd.command_type = KachakaCommand.MOVE_TO_LOCATION_COMMAND
                cmd.move_to_location_command_target_location_id = "home"
                self.send_command(cmd)

            # L2ボタンでドッキング
            if L2_pressed:
                cmd = KachakaCommand()
                cmd.command_type = KachakaCommand.DOCK_SHELF_COMMAND
                cmd.undock_shelf_command_target_shelf_id = "S03"
                self.send_command(cmd)

            # R2ボタンでドッキング解除
            if R2_pressed:
                cmd = KachakaCommand()
                cmd.command_type = KachakaCommand.UNDOCK_SHELF_COMMAND
                cmd.undock_shelf_command_target_shelf_id = "S03"
                self.send_command(cmd)

            # SHAREボタンで半回転
            if share_pressed:
                # 発話
                speak_cmd = KachakaCommand()
                speak_cmd.command_type = KachakaCommand.SPEAK_COMMAND
                speak_cmd.speak_command_text = "半回転します。"
                self.send_command(speak_cmd)

                # その場で半回転
                twist = Twist()
                twist.angular.z = 1.51  # 正の値で左回り、負の値で右回り。rad/s
                duration = 2.0           # 秒数（速度に応じて調整してください）
                end_time = self.get_clock().now().nanoseconds / 1e9 + duration

                while self.get_clock().now().nanoseconds / 1e9 < end_time:
                    self.cmd_vel_pub.publish(twist)

                # 回転終了で停止
                self.cmd_vel_pub.publish(Twist())

                # 発話
                speak_cmd_done = KachakaCommand()
                speak_cmd_done.command_type = KachakaCommand.SPEAK_COMMAND
                speak_cmd_done.speak_command_text = "半回転完了。"
                self.send_command(speak_cmd_done)


            # OPTIONSボタンで発話ver.2を行う
            if options_pressed:
                cmd = KachakaCommand()
                cmd.command_type = KachakaCommand.SPEAK_COMMAND
                cmd.speak_command_text = "コントローラーボタンコマンド起動中"
                self.send_command(cmd)

            # 左スティック押し込みボタンで指定位置1への移動
            if leftpush_pressed:
                speak_command = KachakaCommand()
                speak_command.command_type = KachakaCommand.SPEAK_COMMAND
                speak_command.speak_command_text = "指定番号1番まで移動します。ご注意ください"
                self.send_command(speak_command)

                cmd = KachakaCommand()
                cmd.command_type = KachakaCommand.MOVE_TO_POSE_COMMAND

                # 移動先座標を定義(シェルフのホームの手前の位置)
                cmd.move_to_pose_command_x = 0.749277
                cmd.move_to_pose_command_y = 1.25
                cmd.move_to_pose_command_yaw = 3.140458
                self.send_command(cmd)

                speak_command = KachakaCommand()
                speak_command.command_type = KachakaCommand.SPEAK_COMMAND
                speak_command.speak_command_text = "指定番号1番への移動が完了しました。"
                self.send_command(speak_command)

            # 右スティック押し込みボタンで指定位置2への移動
            if rightpush_pressed:
                speak_command = KachakaCommand()
                speak_command.command_type = KachakaCommand.SPEAK_COMMAND
                speak_command.speak_command_text = "指定番号2番まで移動します。ご注意ください"
                self.send_command(speak_command)

                cmd = KachakaCommand()
                cmd.command_type = KachakaCommand.MOVE_TO_POSE_COMMAND

                # 移動先座標を定義(ホームの手前の位置)
                cmd.move_to_pose_command_x = 1.35811
                cmd.move_to_pose_command_y = 0.665331
                cmd.move_to_pose_command_yaw = -1.570796
                self.send_command(cmd)

                speak_command = KachakaCommand()
                speak_command.command_type = KachakaCommand.SPEAK_COMMAND
                speak_command.speak_command_text = "指定番号2番への移動が完了しました。"
                self.send_command(speak_command)

        if logo_pressed:
            self.get_logger().info("ロゴボタン押下: 現在の動作を強制停止します")

            # Cmd_vel を送って即停止
            self.cmd_vel_pub.publish(Twist())

            # Goal がまだ実行中ならキャンセル
            if self.current_goal_handle is not None and self.current_goal_handle.status == GoalStatus.STATUS_EXECUTING:
                cancel_future = self.current_goal_handle.cancel_goal_async()

                def cancel_done_callback(fut):
                    cancel_response = fut.result()
                    if cancel_response.goals_canceling:
                        self.get_logger().info("強制停止リクエスト送信済み")
                    else:
                        self.get_logger().warn("強制停止リクエストは無効でした")
                    self.current_goal_handle = None

                cancel_future.add_done_callback(cancel_done_callback)
            else:
                self.current_goal_handle = None
                self.get_logger().info("現在のゴールはキャンセルできませんでした")

        # 状態更新
        self.prev_buttons = list(buttons)

    def _is_pressed(self, idx, buttons):
        """立ち上がりエッジ検出"""
        return idx < len(buttons) and buttons[idx] == 1 and self.prev_buttons[idx] == 0


def main(args=None):
    rclpy.init(args=args)
    node = KachakaJoyControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
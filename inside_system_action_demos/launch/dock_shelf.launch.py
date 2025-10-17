# Copyright 2025
#
# Licensed under the Apache License, Version 2.0 (the "License");
# You may not use this file except in compliance with the License.
# You may obtain a copy at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # パッケージのshareディレクトリを取得（rviz設定ファイル用）
    rviz_file = get_package_share_directory("kachaka_description")

    # Launch引数の宣言
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config_file = os.path.join(
        rviz_file,
        'config',
        'kachaka.rviz')

    # 環境変数（ログバッファのflush設定など）
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    # declare_rviz_config_file_cmd = DeclareLaunchArgument(
    #     "rviz_config",
    #     default_value=os.path.join(pkg_share, "rviz", "default.rviz"),
    #     description="Full path to the RVIZ config file to use",
    # )

    # dock_shelf_client.py ノード
    dock_shelf_client_node = Node(
        package="inside_system_action_demos",   # あなたのパッケージ名
        executable="kachaka_action_list.ros2",         # setup.pyで登録した実行名
        name="kachaka_action_list_ros2",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # RViz2 ノード
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # LaunchDescription を構築
    ld = LaunchDescription()

    # 環境変数設定
    ld.add_action(stdout_linebuf_envvar)

    # 引数宣言
    ld.add_action(declare_use_sim_time_cmd)
    # ld.add_action(declare_rviz_config_file_cmd)

    # ノード起動
    ld.add_action(dock_shelf_client_node)
    ld.add_action(rviz2_node)

    return ld

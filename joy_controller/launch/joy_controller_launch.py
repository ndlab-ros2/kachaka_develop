# Copyright 2025
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # パッケージのshareディレクトリを取得（rviz設定ファイル用）
    rviz_file = get_package_share_directory("kachaka_description")

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config_file = os.path.join(
        rviz_file,
        'config',
        'kachaka.rviz')
    
    # joy_linux_node のパラメータ読み込み
    joy_config_dir = os.path.join(
        get_package_share_directory('joy_controller'),
        'config'
    )
    joy_param_file = os.path.join(joy_config_dir, 'joy_linux_param.yaml')
    with open(joy_param_file, 'r') as f:
        joy_params = yaml.safe_load(f)['joy_linux_node']['ros__parameters']

    # joy_controller_node のパラメータ読み込み
    controller_param_file = os.path.join(joy_config_dir, 'joy_controller_param.yaml')
    with open(controller_param_file, 'r') as f:
        controller_params = yaml.safe_load(f)['joy_controller_node']['ros__parameters']

    # 環境変数（ログバッファのflush設定など）
    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    # Node: joy_linux
    joy_linux_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_linux_node',
        namespace='/operation/joy_linux',
        parameters=[joy_params],
        output='screen'
    )

    # Node: joy_controller
    joy_controller_node = Node(
        package='joy_controller',
        executable='joy_controller_node',
        name='joy_controller_node',
        namespace='/operation/joy_controller',
        parameters=[controller_params],
        output='screen'
    )

    # kachaka_button_action_list.py ノード
    kachaka_button_node = Node(
        package="joy_controller",   # あなたのパッケージ名
        executable="kachaka_button_action_list",         # setup.pyで登録した実行名
        name="kachaka_button_action_list",
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
    ld.add_action(joy_linux_node)
    ld.add_action(joy_controller_node)
    ld.add_action(kachaka_button_node)
    ld.add_action(rviz2_node)

    return ld
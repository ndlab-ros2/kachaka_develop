#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

rviz_bringup = get_package_share_directory('kachaka_description')

rviz_config_path = os.path.join(rviz_bringup, 'config', 'display.rviz')

def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="kachaka",
        description="Namespace for the robot state publisher",
    )

    frame_prefix_arg = DeclareLaunchArgument(
        "frame_prefix",
        default_value=EnvironmentVariable("FRAME_PREFIX", default_value=""),
        description="Frame prefix for the robot state publisher",
    )

    namespace = LaunchConfiguration("namespace")
    frame_prefix = LaunchConfiguration("frame_prefix")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("kachaka_description"),
                    "robot",
                    "kachaka.urdf.xacro",
                ]
            ),
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_content,
                "frame_prefix": frame_prefix,
            }
        ],
    )

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['--display-config', rviz_config_path]
        )

    return LaunchDescription(
        [namespace_arg, frame_prefix_arg, robot_state_publisher_node, rviz_node]
    )

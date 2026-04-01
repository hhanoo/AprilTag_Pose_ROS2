#!/usr/bin/env python3
"""
AprilTag Pose Estimator Launch File (C++)
Launches AprilTag pose estimator node with configurable parameters and optional image viewer
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    """Setup launch nodes with evaluated launch configurations"""

    # =========================================================
    # Arguments
    # =========================================================
    # Launch Configuration Arguments
    config_file = LaunchConfiguration("config_file")
    show_service_result_window = LaunchConfiguration("show_service_result_window")
    camera_topic = LaunchConfiguration("camera_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    tag_family = LaunchConfiguration("tag_family")
    tag_size = LaunchConfiguration("tag_size")

    # Base Parameters
    parameters = [
        ParameterFile(config_file, allow_substs=True),
    ]

    # Optional Overrides
    if camera_topic.perform(context) != "":
        parameters.append(
            {
                "camera_topic": camera_topic,
            }
        )
    if camera_info_topic.perform(context) != "":
        parameters.append(
            {
                "camera_info_topic": camera_info_topic,
            }
        )
    if tag_family.perform(context) != "":
        parameters.append(
            {
                "tag_family": tag_family,
            }
        )
    if tag_size.perform(context) != "":
        parameters.append(
            {
                "tag_size": tag_size,
            }
        )
    if show_service_result_window.perform(context) != "":
        parameters.append(
            {
                "show_service_result_window": show_service_result_window,
            }
        )

    # =========================================================
    # Nodes
    # =========================================================
    # AprilTag pose estimator node
    pose_estimator_node = Node(
        package="apriltag_pose_estimator",
        executable="pose_estimator_node",
        name="pose_estimator_node",
        output="screen",
        parameters=parameters,
    )

    # =========================================================
    # Nodes to Start
    # =========================================================
    nodes_to_start = [
        pose_estimator_node,
    ]

    return nodes_to_start


def generate_launch_description():
    """Generate launch description with declared arguments"""

    # =========================================================
    # Declare Launch Arguments
    # =========================================================
    declared_arguments = []

    # Core Arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("apriltag_pose_estimator"),
                    "config",
                    "pose_estimator.yaml",
                ]
            ),
            description="Path to YAML configuration file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "show_service_result_window",
            default_value="",
            description="Show OpenCV result window on service call (disable in Docker/headless)",
        )
    )

    # Optional Overrides (empty = disabled)
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_topic",
            default_value="",
            description="Camera image topic (relative path for namespace compatibility)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="",
            description="Camera info topic (relative path for namespace compatibility)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tag_family",
            default_value="",
            description="AprilTag family (e.g., tagStandard41h12, tag36h11)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tag_size",
            default_value="",
            description="AprilTag size in meters (default: 0.02778 = (50mm / 9 * 5 (tagStandard41h12) / 1000.0 (to meters)))",
        )
    )

    # =========================================================
    # Launch Description
    # =========================================================
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

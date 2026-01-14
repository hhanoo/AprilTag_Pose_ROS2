#!/usr/bin/env python3
"""
AprilTag Pose Estimator Launch File
Launches AprilTag pose estimator node with configurable parameters and optional image viewer
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context):
    """Setup launch nodes with evaluated launch configurations"""

    # =========================================================
    # Initialize Arguments
    # =========================================================
    camera_topic = LaunchConfiguration("camera_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    tag_family = LaunchConfiguration("tag_family")
    tag_size = LaunchConfiguration("tag_size")
    show_detection = LaunchConfiguration("show_detection")

    # =========================================================
    # Configuration
    # =========================================================
    # Get YAML config file path
    config_file = PathJoinSubstitution(
        [
            FindPackageShare("apriltag_pose_estimator_py"),
            "config",
            "pose_estimator.yaml",
        ]
    )

    # =========================================================
    # Nodes
    # =========================================================
    # AprilTag pose estimator node
    pose_estimator_node = Node(
        package="apriltag_pose_estimator_py",
        executable="pose_estimator_node",
        name="pose_estimator_node",
        output="screen",
        parameters=[
            config_file,  # Load default parameters from YAML
            {  # Override with launch arguments
                "tag_size": tag_size,
                "tag_family": tag_family,
                "camera_topic": camera_topic,
                "camera_info_topic": camera_info_topic,
            },
        ],
    )

    # =========================================================
    # Visualization Tools
    # =========================================================
    # rqt_image_view for detection image
    detection_viewer = ExecuteProcess(
        condition=IfCondition(show_detection),
        cmd=[
            "ros2",
            "run",
            "rqt_image_view",
            "rqt_image_view",
            "/detection_image",
        ],
        output="screen",
        shell=False,
    )

    # =========================================================
    # Nodes to Start
    # =========================================================
    nodes_to_start = [
        pose_estimator_node,
        detection_viewer,
    ]

    return nodes_to_start


def generate_launch_description():
    """Generate launch description with declared arguments"""

    # =========================================================
    # Declare Launch Arguments
    # =========================================================
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_topic",
            default_value="/color/image_raw",
            description="Camera image topic",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/color/camera_info",
            description="Camera info topic",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tag_family",
            default_value="tagStandard41h12",
            description="AprilTag family (e.g., tagStandard41h12, tag36h11)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tag_size",
            default_value="0.02778",
            description="AprilTag size in meters (default: 0.02778 = (50mm / 9 * 5 (tagStandard41h12) / 1000.0 (to meters)))",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "show_detection",
            default_value="true",
            description="Show detection image in rqt_image_view",
        )
    )

    # =========================================================
    # Launch Description
    # =========================================================
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

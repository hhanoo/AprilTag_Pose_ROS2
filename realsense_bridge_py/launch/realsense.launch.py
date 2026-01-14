#!/usr/bin/env python3
"""
RealSense Camera Launch File
Launches RealSense node with configurable parameters and optional image viewer
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
    serial_number = LaunchConfiguration("serial_number")
    frame_rate = LaunchConfiguration("frame_rate")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    enable_depth = LaunchConfiguration("enable_depth")
    show_image = LaunchConfiguration("show_image")

    # =========================================================
    # Configuration
    # =========================================================
    # Get YAML config file path
    config_file = PathJoinSubstitution(
        [FindPackageShare("realsense_bridge_py"), "config", "realsense.yaml"]
    )

    # =========================================================
    # Nodes
    # =========================================================
    # RealSense node
    realsense_node = Node(
        package="realsense_bridge_py",
        executable="realsense_node",
        name="realsense_node",
        output="screen",
        parameters=[
            config_file,  # Load default parameters from YAML
            {  # Override with launch arguments
                "serial_number": serial_number,
                "frame_rate": frame_rate,
                "resolution.width": width,
                "resolution.height": height,
                "enable_depth": enable_depth,
            },
        ],
    )

    # =========================================================
    # Visualization Tools
    # =========================================================
    # rqt_image_view for color image
    image_viewer = ExecuteProcess(
        condition=IfCondition(show_image),
        cmd=[
            "ros2",
            "run",
            "rqt_image_view",
            "rqt_image_view",
            "/color/image_raw",
        ],
        output="screen",
        shell=False,
    )

    # =========================================================
    # Nodes to Start
    # =========================================================
    nodes_to_start = [
        realsense_node,
        image_viewer,
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
            "serial_number",
            default_value="",
            description="Specific camera serial number (empty for first available)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "frame_rate",
            default_value="30",
            description="Camera frame rate (FPS)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "width",
            default_value="1280",
            description="Image width in pixels",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "height",
            default_value="720",
            description="Image height in pixels",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "enable_depth",
            default_value="false",
            description="Enable depth stream",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "show_image",
            default_value="true",
            description="Show camera image in rqt_image_view",
        )
    )

    # =========================================================
    # Launch Description
    # =========================================================
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )

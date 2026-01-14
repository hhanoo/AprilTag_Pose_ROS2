import os
from glob import glob

from setuptools import find_packages, setup

package_name = "realsense_bridge_py"

setup(
    # =========================================================
    # Package metadata
    # =========================================================
    name=package_name,
    version="0.0.0",
    # =========================================================
    # Package discovery
    # Automatically find all Python packages in the directory
    # =========================================================
    packages=find_packages(exclude=["test"]),
    # =========================================================
    # Data files to install
    # These files are required for ROS2 package discovery
    # =========================================================
    data_files=[
        # Register package with ROS2 index
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # Install package.xml for dependency management
        ("share/" + package_name, ["package.xml"]),
        # Install launch files
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        # Install config files
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
    ],
    # =========================================================
    # Dependencies
    # =========================================================
    install_requires=["setuptools"],
    zip_safe=True,
    # =========================================================
    # Package information
    # =========================================================
    maintainer="hhanoo",
    maintainer_email="woo980711@gmail.com",
    description="RealSense to ROS2 bridge for camera data publishing",
    license="MIT",
    # =========================================================
    # Extra dependencies (for testing, etc.)
    # =========================================================
    extras_require={},
    # =========================================================
    # Entry points (Executable scripts)
    # Maps command names to Python functions
    # Format: 'command_name = package.module:function'
    # =========================================================
    entry_points={
        "console_scripts": [
            # ros2 run realsense_bridge_py realsense_node
            "realsense_node = realsense_bridge_py.realsense_node:main",
        ],
    },
)

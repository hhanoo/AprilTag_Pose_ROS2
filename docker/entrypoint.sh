#!/bin/bash
set -e

# Set timezone to Seoul
export TZ=Asia/Seoul
ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace (if built)
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

exec "$@"

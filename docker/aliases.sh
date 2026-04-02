#!/bin/bash

# ROS env
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash

# Launch
alias run_cam='ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=1280x720x30 enable_depth:=false'
alias run_est='ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py'
alias run_viz='ros2 run apriltag_pose_visualizer pose_visualizer_node'

# Monitoring
alias echo_pose='ros2 topic echo /pose_estimator_node/target_poses'
alias call_pose='ros2 service call /pose_estimator_node/target_point_pose apriltag_pose_estimator_msgs/srv/TargetPointPose "{request_time: {sec: 0, nanosec: 0}}"'

# Help
alias cmd_help='echo; echo "[apriltag-pose-ros2] Commands: run_cam, run_est, run_viz, echo_pose, call_pose, cmd_help"'

# Show help on interactive shell entry
case $- in
    *i*) cmd_help ;;
esac

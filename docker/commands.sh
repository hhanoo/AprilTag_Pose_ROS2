# ===== ROS env =====
[ -f /opt/ros/humble/setup.bash ]  && source /opt/ros/humble/setup.bash
[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash

# ===== Common helpers =====
source-ros-ws() {
    [ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash
}

# ===== Build =====
build() {
    cd /ros2_ws || return 1
    colcon build \
        --symlink-install \
        --cmake-args \
            -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
            -DCMAKE_BUILD_TYPE=Release "$@"
    source-ros-ws
}

# ===== Test =====
run-tests() {
    cd /ros2_ws || return 1
    colcon test --packages-select-regex 'apriltag' --event-handlers console_cohesion+ "$@"
    colcon test-result --verbose
}

# ===== Launchers =====
run-camera() {
    source-ros-ws
    ros2 launch realsense2_camera rs_launch.py \
        camera_namespace:=/ \
        rgb_camera.color_profile:=1280x720x30 \
        enable_depth:=false "$@"
}

run-estimator() {
    source-ros-ws
    ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py "$@"
}

run-visualizer() {
    source-ros-ws
    ros2 run apriltag_pose_visualizer pose_visualizer_node "$@"
}

# ===== Monitoring =====
echo-pose() {
    source-ros-ws
    ros2 topic echo /pose_estimator_node/target_poses "$@"
}

call-pose() {
    source-ros-ws
    ros2 service call /pose_estimator_node/target_point_pose \
        apriltag_pose_estimator_msgs/srv/TargetPointPose \
        "{request_time: {sec: 0, nanosec: 0}}" "$@"
}

# ===== Help =====
cmd-help() {
    printf "\n[apriltag-pose-ros2] Commands:\n\n"

    printf "  Build:\n"
    printf "    %-14s - %s\n" "build"      "colcon build --symlink-install + source overlay"
    printf "\n"

    printf "  Test:\n"
    printf "    %-14s - %s\n" "run-tests"  "colcon test (apriltag packages) + result summary"
    printf "\n"

    printf "  Launchers:\n"
    printf "    %-14s - %s\n" "run-camera"     "realsense2_camera rs_launch.py (1280x720x30, no depth)"
    printf "    %-14s - %s\n" "run-estimator"  "apriltag_pose_estimator launch"
    printf "    %-14s - %s\n" "run-visualizer" "apriltag_pose_visualizer node"
    printf "\n"

    printf "  Monitoring:\n"
    printf "    %-14s - %s\n" "echo-pose"  "Echo /pose_estimator_node/target_poses"
    printf "    %-14s - %s\n" "call-pose"  "Call /pose_estimator_node/target_point_pose"
    printf "\n"

    printf "  Help:\n"
    printf "    %-14s - %s\n" "cmd-help"   "Show this help"
    echo
}

# ===== Show help on interactive shell =====
case $- in
    *i*) cmd-help ;;
esac

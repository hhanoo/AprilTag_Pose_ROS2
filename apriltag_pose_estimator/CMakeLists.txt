cmake_minimum_required(VERSION 3.8)
project(apriltag_pose_estimator)

# =========================================================
# Compiler Configuration
# =========================================================
# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================================================
# Dependencies
# =========================================================
# Core ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(apriltag_pose_estimator_msgs REQUIRED)

# External libraries
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(APRILTAG REQUIRED apriltag)

# =========================================================
# Include Directories
# =========================================================
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${APRILTAG_INCLUDE_DIRS}
)

# =========================================================
# Library - AprilTag Detector and Estimator
# =========================================================
add_library(${PROJECT_NAME}_lib
    src/april_tag_detector.cpp
    src/multi_tag_pose_estimator.cpp
    src/slerp_pose_filter.cpp
)
target_link_libraries(${PROJECT_NAME}_lib
    ${OpenCV_LIBS}
    ${APRILTAG_LIBRARIES}
    Eigen3::Eigen
)

# =========================================================
# Executable - Pose Estimator Node
# =========================================================
add_executable(pose_estimator_node
    src/pose_estimator_node.cpp
    src/main.cpp
)
ament_target_dependencies(pose_estimator_node
    rclcpp
    sensor_msgs
    geometry_msgs
    std_msgs
    cv_bridge
    apriltag_pose_estimator_msgs
)
target_link_libraries(pose_estimator_node
    ${PROJECT_NAME}_lib
    ${OpenCV_LIBS}
    ${APRILTAG_LIBRARIES}
    Eigen3::Eigen
)

# =========================================================
# Installation
# =========================================================
# Install library
install(TARGETS
    ${PROJECT_NAME}_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install executable
install(TARGETS
    pose_estimator_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY
    include/
    DESTINATION include
)

# Install launch and config files
install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}
)

# =========================================================
# Testing
# =========================================================
if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_multi_tag_pose_estimator
        test/test_multi_tag_pose_estimator.cpp
    )
    target_link_libraries(test_multi_tag_pose_estimator
        ${PROJECT_NAME}_lib
    )
endif()

# =========================================================
# Package Export
# =========================================================
ament_package()

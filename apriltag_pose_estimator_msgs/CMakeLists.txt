cmake_minimum_required(VERSION 3.8)
project(apriltag_pose_estimator_msgs)

# =========================================================
# Compiler options
# =========================================================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================================================
# Dependencies
# =========================================================
# * Core ROS2 build system
find_package(ament_cmake REQUIRED)

# * Interface generation
find_package(rosidl_default_generators REQUIRED)

# * Message dependencies
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# =========================================================
# Interface Generation
# - Generate custom messages, services, and actions
# =========================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/TagDetection.msg"
  # Services
  "srv/TargetPointPose.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

# =========================================================
# Package declaration
# =========================================================
ament_package()

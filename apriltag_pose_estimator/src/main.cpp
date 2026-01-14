#include <rclcpp/rclcpp.hpp>

#include "apriltag_pose_estimator/pose_estimator_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<apriltag_pose_estimator::PoseEstimatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

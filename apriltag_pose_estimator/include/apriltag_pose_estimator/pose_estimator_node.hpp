#ifndef APRILTAG_POSE_ESTIMATOR__POSE_ESTIMATOR_NODE_HPP_
#define APRILTAG_POSE_ESTIMATOR__POSE_ESTIMATOR_NODE_HPP_

#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#include "apriltag_pose_estimator/april_tag_detector.hpp"
#include "apriltag_pose_estimator/multi_tag_pose_estimator.hpp"
#include "apriltag_pose_estimator/tag_config.hpp"

namespace apriltag_pose_estimator {

class PoseEstimatorNode : public rclcpp::Node {
   public:
    // ========================================================================
    // Constructor / Destructor
    // ========================================================================
    PoseEstimatorNode();
    ~PoseEstimatorNode() = default;

   private:
    // ========================================================================
    // Callbacks
    // ========================================================================
    void imageCallback(
        const sensor_msgs::msg::Image::SharedPtr msg  // Camera image message
    );
    void cameraInfoCallback(
        const sensor_msgs::msg::CameraInfo::SharedPtr msg  // Camera info message
    );

    // ========================================================================
    // Helper Functions
    // ========================================================================
    void publishPoses(
        const std::vector<Eigen::Matrix4f>& transforms,  // Target point transforms
        const rclcpp::Time&                 timestamp    // Message timestamp
    );
    void publishVisualization(
        const std::vector<Eigen::Matrix4f>& transforms,  // Target point transforms
        const rclcpp::Time&                 timestamp    // Message timestamp
    );
    geometry_msgs::msg::Pose matrixToPose(
        const Eigen::Matrix4f& matrix  // 4x4 transformation matrix
    );

    // ========================================================================
    // Detectors
    // ========================================================================
    std::unique_ptr<AprilTagDetector>      detector_;   // AprilTag detector
    std::unique_ptr<MultiTagPoseEstimator> estimator_;  // Multi-tag pose estimator

    // ========================================================================
    // ROS2 Communication
    // ========================================================================
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      image_sub_;        // Image subscriber
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;  // Camera info subscriber

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr        pose_pub_;       // Target poses publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;     // Visualization markers publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr              detection_pub_;  // Detection image publisher

    // ========================================================================
    // Camera Info
    // ========================================================================
    cv::Mat     camera_matrix_;         // Camera intrinsic matrix (3x3)
    cv::Mat     dist_coeffs_;           // Distortion coefficients
    bool        camera_info_received_;  // Camera info received flag
    std::string camera_frame_;          // Camera frame ID

    // ========================================================================
    // Parameters
    // ========================================================================
    std::vector<int64_t>     marker_ids_;          // Marker IDs to use
    int64_t                  base_marker_id_;      // Base marker ID
    double                   tag_size_;            // Tag size in meters
    std::string              tag_family_;          // AprilTag family
    std::vector<double>      marker_offsets_;      // [X, Y] offset between markers
    std::vector<double>      target_points_flat_;  // Target points (Nx6 flat list)
    std::vector<TargetPoint> target_points_;       // Target points

    // ========================================================================
    // Topic Names
    // ========================================================================
    std::string camera_topic_;           // Camera image topic
    std::string camera_info_topic_;      // Camera info topic
    std::string pose_topic_;             // Target poses topic
    std::string detection_image_topic_;  // Detection image topic
    std::string marker_topic_;           // Visualization markers topic

    // ========================================================================
    // Flags
    // ========================================================================
    bool publish_visualization_;    // Publish visualization markers flag
    bool publish_detection_image_;  // Publish detection image flag

    // ========================================================================
    // State
    // ========================================================================
    bool             detection_active_;   // Detection active flag
    std::vector<int> last_detected_ids_;  // Last detected tag IDs
};

}  // namespace apriltag_pose_estimator

#endif  // APRILTAG_POSE_ESTIMATOR__POSE_ESTIMATOR_NODE_HPP_

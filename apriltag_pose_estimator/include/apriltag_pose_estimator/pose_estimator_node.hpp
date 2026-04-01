#ifndef APRILTAG_POSE_ESTIMATOR__POSE_ESTIMATOR_NODE_HPP_
#define APRILTAG_POSE_ESTIMATOR__POSE_ESTIMATOR_NODE_HPP_

#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>

#include "apriltag_pose_estimator/april_tag_detector.hpp"
#include "apriltag_pose_estimator/multi_tag_pose_estimator.hpp"
#include "apriltag_pose_estimator/tag_config.hpp"
#include "apriltag_pose_estimator_msgs/msg/tag_detection.hpp"
#include "apriltag_pose_estimator_msgs/srv/target_point_pose.hpp"

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
    // * Subscribers
    void imageCallback(
        const sensor_msgs::msg::Image::SharedPtr msg  // Camera image message
    );
    void cameraInfoCallback(
        const sensor_msgs::msg::CameraInfo::SharedPtr msg  // Camera info message
    );

    // * Services Servers
    void targetPointPoseServiceCallback(
        std::shared_ptr<apriltag_pose_estimator_msgs::srv::TargetPointPose::Request>  request,
        std::shared_ptr<apriltag_pose_estimator_msgs::srv::TargetPointPose::Response> response);

    // ========================================================================
    // Publishers
    // ========================================================================
    void publishTagDetection();

    // ========================================================================
    // Helper Functions
    // ========================================================================
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
    rclcpp::Publisher<apriltag_pose_estimator_msgs::msg::TagDetection>::SharedPtr tag_detection_pub_;

    // Services
    rclcpp::Service<apriltag_pose_estimator_msgs::srv::TargetPointPose>::SharedPtr target_point_pose_server_;

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
    std::vector<int64_t>     marker_ids_;                  // Marker IDs to use
    int64_t                  base_marker_id_;              // Base marker ID
    double                   tag_size_;                    // Tag size in meters
    std::string              tag_family_;                  // AprilTag family
    std::vector<double>      marker_offsets_;              // [X, Y] offset between markers
    std::vector<double>      target_points_flat_;          // Target points (Nx6 flat list)
    std::vector<TargetPoint> target_points_;               // Target points
    bool                     show_service_result_window_;  // Show result window on service call
    int                      display_width_;               // Display window width (0 = original)
    int                      display_height_;              // Display window height (0 = original)

    // ========================================================================
    // Topic Names
    // ========================================================================
    std::string camera_topic_;         // Camera image topic
    std::string camera_info_topic_;    // Camera info topic
    std::string tag_detection_topic_;  // Tag detection info topic

    // ========================================================================
    // Service Names
    // ========================================================================
    std::string target_point_pose_service_;  // Target point pose service name

    // ========================================================================
    // State
    // ========================================================================
    bool             detection_active_;   // Detection active flag
    std::vector<int> last_detected_ids_;  // Last detected tag IDs

    // Latest detection results for service
    std::mutex                   detection_mutex_;      // Mutex for thread-safe access
    bool                         latest_tag_detected_;  // Latest tag detected flag
    cv::Mat                      latest_rvec_;          // Latest rotation vector
    cv::Mat                      latest_tvec_;          // Latest translation vector
    cv::Mat                      latest_vis_image_;     // Latest visualization image
    std::vector<Eigen::Matrix4f> latest_transforms_;    // Latest target point transforms
    rclcpp::Time                 latest_timestamp_;     // Latest detection timestamp
};

}  // namespace apriltag_pose_estimator

#endif  // APRILTAG_POSE_ESTIMATOR__POSE_ESTIMATOR_NODE_HPP_
#ifndef APRILTAG_POSE_ESTIMATOR__POSE_ESTIMATOR_NODE_HPP_
#define APRILTAG_POSE_ESTIMATOR__POSE_ESTIMATOR_NODE_HPP_

#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <string>
#include <vector>

#include "apriltag_pose_estimator/april_tag_detector.hpp"
#include "apriltag_pose_estimator/multi_tag_pose_estimator.hpp"
#include "apriltag_pose_estimator/tag_config.hpp"
#include "apriltag_pose_estimator_msgs/msg/tag_detection.hpp"
#include "apriltag_pose_estimator_msgs/srv/target_point_pose.hpp"

namespace apriltag_pose_estimator {

// Per-group config parsed from yaml (groups.<name>.{...})
// Each group outputs the base marker pose (camera frame). No per-target offsets.
struct GroupConfig {
    std::vector<int64_t> marker_ids;      // Marker IDs
    int64_t              base_marker_id;  // Base marker ID
    std::vector<double>  marker_offsets;  // 6N flat (T_base <- marker_i)
};

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
    // Group Parameter Parsing
    // ========================================================================
    // yaml must define non-empty group_names (multi-group only since v3.0.0).
    void parseGroupsFromParams();

    // ========================================================================
    // Publishers / Service Helpers
    // ========================================================================
    void                     publishGroupNamesLatched();                               // Once at startup (transient_local)
    void                     publishTagDetection(const std::string& reference_group);  // Use reference_group's rvec/tvec
    geometry_msgs::msg::Pose matrixToPose(const Eigen::Matrix4f& matrix);
    geometry_msgs::msg::Pose makeNanPose();  // Fill failed groups

    // ========================================================================
    // Detector / Estimator
    // ========================================================================
    std::unique_ptr<AprilTagDetector>                             detector_;    // Single, group-agnostic
    std::map<std::string, std::unique_ptr<MultiTagPoseEstimator>> estimators_;  // Per-group

    // ========================================================================
    // ROS2 Communication
    // ========================================================================
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      image_sub_;        // Image subscriber
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;  // Camera info subscriber

    // Publishers
    rclcpp::Publisher<apriltag_pose_estimator_msgs::msg::TagDetection>::SharedPtr tag_detection_pub_;    // Raw detection (debug)
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr                   target_poses_pub_;     // Per-group base marker pose
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                           group_names_pub_;      // Latched group names CSV
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr                  group_status_pub_;     // Per-group valid bits
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr                         detection_image_pub_;  // BGR8 overlay

    // Services
    rclcpp::Service<apriltag_pose_estimator_msgs::srv::TargetPointPose>::SharedPtr target_point_pose_server_;

    // ========================================================================
    // Camera Info
    // ========================================================================
    cv::Mat     camera_matrix_;                    // Camera intrinsic matrix (3x3)
    cv::Mat     dist_coeffs_;                      // Distortion coefficients
    bool        camera_info_received_;             // Camera info received flag
    std::string camera_frame_;                     // Camera frame ID
    bool        use_distortion_from_camera_info_;  // Use distortion coefficients from camera_info.d

    // ========================================================================
    // Common Parameters
    // ========================================================================
    double      tag_size_;                    // Tag size in meters
    std::string tag_family_;                  // AprilTag family
    bool        publish_detection_image_;     // Publish detection_image (skip if no subscriber)
    bool        show_service_result_window_;  // Show OpenCV window on service call
    int         display_width_;               // Display window width (0 = original)
    int         display_height_;              // Display window height (0 = original)

    // ========================================================================
    // Topic / Service Names
    // ========================================================================
    std::string camera_topic_;               // Input camera image topic
    std::string camera_info_topic_;          // Input camera info topic
    std::string tag_detection_topic_;        // Output: raw TagDetection (debug)
    std::string target_poses_topic_;         // Output: PoseArray (per-group base marker pose)
    std::string group_names_topic_;          // Output: latched String CSV
    std::string group_status_topic_;         // Output: per-group 0/1 valid bits
    std::string detection_image_topic_;      // Output: BGR8 overlay image
    std::string target_point_pose_service_;  // Service name

    // ========================================================================
    // Multi-Group State
    // ========================================================================
    std::vector<std::string>           group_names_;  // Publish/iteration order (yaml declaration order)
    std::map<std::string, GroupConfig> group_configs_;

    // For change-detection logging
    std::vector<int> last_detected_ids_;

    // ========================================================================
    // Latest Detection Results (Per Group, for service)
    // ========================================================================
    std::mutex                             detection_mutex_;
    std::map<std::string, bool>            latest_valid_per_group_;  // Last estimate success per group
    std::map<std::string, cv::Mat>         latest_rvec_per_group_;
    std::map<std::string, cv::Mat>         latest_tvec_per_group_;
    std::map<std::string, Eigen::Matrix4f> latest_base_transform_per_group_;  // T_camera <- base_marker
    cv::Mat                                latest_vis_image_;                 // Single image with all-group overlays
    rclcpp::Time                           latest_timestamp_;
};

}  // namespace apriltag_pose_estimator

#endif  // APRILTAG_POSE_ESTIMATOR__POSE_ESTIMATOR_NODE_HPP_

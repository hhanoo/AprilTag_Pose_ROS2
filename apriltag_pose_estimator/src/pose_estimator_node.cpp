#include "apriltag_pose_estimator/pose_estimator_node.hpp"

#include <Eigen/Geometry>

namespace apriltag_pose_estimator {

// ========================================================================
// Constructor
// ========================================================================
PoseEstimatorNode::PoseEstimatorNode()
    : Node("pose_estimator_node"),
      camera_info_received_(false),
      detection_active_(false) {
    // ---------- Declare Parameters ----------
    // * AprilTag configuration
    this->declare_parameter("marker_ids", std::vector<int64_t>{0, 1, 2});
    this->declare_parameter("base_marker_id", -1);
    this->declare_parameter("tag_size", 0.02778);
    this->declare_parameter("tag_family", "tagStandard41h12");
    this->declare_parameter("marker_offsets", std::vector<double>{0.065, 0.0});
    this->declare_parameter("target_points", std::vector<double>{
                                                 0.000, 0.000, 0.000, 0.000, 0.000, 0.000,  // Point 1 (x y z r p y)
                                                 0.000, 0.000, 0.000, 0.000, 0.000, 0.000   // Point 2 (x y z r p y)
                                             });
    // * Input topics
    this->declare_parameter("camera_topic", "realsense_node/color/image_raw");
    this->declare_parameter("camera_info_topic", "realsense_node/color/camera_info");
    this->declare_parameter("camera_frame", "camera_color_optical_frame");
    // * Output topics
    this->declare_parameter("pose_topic", "pose_estimator_node/target_poses");
    this->declare_parameter("detection_image_topic", "pose_estimator_node/detection_image");
    this->declare_parameter("marker_topic", "visualization_markers");
    // * Publishing flags
    this->declare_parameter("publish_visualization", true);
    this->declare_parameter("publish_detection_image", true);

    // ---------- Get Parameters ----------
    // * AprilTag configuration
    marker_ids_         = this->get_parameter("marker_ids").as_integer_array();
    base_marker_id_     = this->get_parameter("base_marker_id").as_int();
    tag_size_           = this->get_parameter("tag_size").as_double();
    tag_family_         = this->get_parameter("tag_family").as_string();
    marker_offsets_     = this->get_parameter("marker_offsets").as_double_array();
    target_points_flat_ = this->get_parameter("target_points").as_double_array();

    // * Input topics
    camera_topic_      = this->get_parameter("camera_topic").as_string();
    camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
    camera_frame_      = this->get_parameter("camera_frame").as_string();

    // * Output topics
    pose_topic_            = this->get_parameter("pose_topic").as_string();
    detection_image_topic_ = this->get_parameter("detection_image_topic").as_string();
    marker_topic_          = this->get_parameter("marker_topic").as_string();

    // * Publishing flags
    publish_visualization_   = this->get_parameter("publish_visualization").as_bool();
    publish_detection_image_ = this->get_parameter("publish_detection_image").as_bool();

    // ---------- Data Conversion ----------
    // * Convert marker_ids to int vector
    std::vector<int> marker_ids_int;
    for (auto id : marker_ids_) {
        marker_ids_int.push_back(static_cast<int>(id));
    }

    // * Convert target_points_flat_ to TargetPoint vector
    for (size_t i = 0; i < target_points_flat_.size() / 6; i++) {
        TargetPoint target_point;
        target_point.position(0)     = static_cast<float>(target_points_flat_[i * 6]);
        target_point.position(1)     = static_cast<float>(target_points_flat_[i * 6 + 1]);
        target_point.position(2)     = static_cast<float>(target_points_flat_[i * 6 + 2]);
        target_point.rotation_rpy(0) = static_cast<float>(target_points_flat_[i * 6 + 3]);
        target_point.rotation_rpy(1) = static_cast<float>(target_points_flat_[i * 6 + 4]);
        target_point.rotation_rpy(2) = static_cast<float>(target_points_flat_[i * 6 + 5]);
        target_points_.push_back(target_point);
    }

    // ---------- Initialize Components ----------
    // Create detector
    detector_ = std::make_unique<AprilTagDetector>(tag_family_, tag_size_);

    // ---------- Create subscribers ----------
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic_,
        10,
        std::bind(&PoseEstimatorNode::imageCallback, this, std::placeholders::_1));

    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_,
        10,
        std::bind(&PoseEstimatorNode::cameraInfoCallback, this, std::placeholders::_1));

    // ---------- Create publishers ----------
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(pose_topic_, 10);

    if (publish_visualization_) {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            marker_topic_, 10);
    }

    if (publish_detection_image_) {
        detection_pub_ = this->create_publisher<sensor_msgs::msg::Image>(detection_image_topic_, 10);
    }

    // ---------- Initialize camera distortion coefficients ----------
    dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);

    RCLCPP_INFO(this->get_logger(), "AprilTag Pose Estimator initialized");
    RCLCPP_INFO(this->get_logger(), "  Marker IDs: [%s]",
                [&]() {
                    std::string ids_str;
                    for (size_t i = 0; i < marker_ids_int.size(); i++) {
                        ids_str += std::to_string(marker_ids_int[i]);
                        if (i < marker_ids_int.size() - 1)
                            ids_str += ", ";
                    }
                    return ids_str;
                }()
                    .c_str());
    RCLCPP_INFO(this->get_logger(), "  Tag family: %s", tag_family_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Tag size: %.5f m", tag_size_);
    RCLCPP_INFO(this->get_logger(), "  Target points: %zu", target_points_.size());
}

// ========================================================================
// Callback Functions
// ========================================================================
void PoseEstimatorNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (camera_info_received_) {
        return;  // Already received
    }

    // Extract camera matrix
    camera_matrix_                  = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix_.at<double>(0, 0) = msg->k[0];  // fx
    camera_matrix_.at<double>(1, 1) = msg->k[4];  // fy
    camera_matrix_.at<double>(0, 2) = msg->k[2];  // cx
    camera_matrix_.at<double>(1, 2) = msg->k[5];  // cy

    // Extract distortion coefficients
    if (!msg->d.empty()) {
        dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F);
        for (size_t i = 0; i < msg->d.size(); i++) {
            dist_coeffs_.at<double>(i, 0) = msg->d[i];
        }
    }

    // Convert marker offsets
    std::vector<float> marker_offsets_float;
    for (auto offset : marker_offsets_) {
        marker_offsets_float.push_back(static_cast<float>(offset));
    }

    // Convert marker_ids to int vector
    std::vector<int> marker_ids_int;
    for (auto id : marker_ids_) {
        marker_ids_int.push_back(static_cast<int>(id));
    }

    // Create estimator now that we have camera info
    estimator_ = std::make_unique<MultiTagPoseEstimator>(
        marker_ids_int,
        marker_offsets_float,
        target_points_,
        tag_size_,
        camera_matrix_,
        dist_coeffs_,
        static_cast<int>(base_marker_id_));

    camera_info_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Camera info received");
    RCLCPP_INFO(this->get_logger(), "  fx: %.2f, fy: %.2f",
                camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(1, 1));
    RCLCPP_INFO(this->get_logger(), "  cx: %.2f, cy: %.2f",
                camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2));
}

void PoseEstimatorNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (!camera_info_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Camera info not received yet");
        return;
    }

    // Convert ROS image to OpenCV
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat cv_image  = cv_ptr->image;
    cv::Mat vis_image = cv_image.clone();

    // Detect AprilTags
    std::vector<TagDetection> tags = detector_->detect(cv_image, camera_matrix_, dist_coeffs_);

    if (tags.empty()) {
        if (detection_active_) {
            RCLCPP_WARN(this->get_logger(), "No AprilTags detected");
            detection_active_ = false;
            last_detected_ids_.clear();
        }
        return;
    }

    // Log detected tag IDs only when they change
    std::vector<int> detected_ids;
    for (const auto& tag : tags) {
        detected_ids.push_back(tag.id);
    }
    std::sort(detected_ids.begin(), detected_ids.end());

    if (detected_ids != last_detected_ids_) {
        std::string ids_str;
        for (size_t i = 0; i < detected_ids.size(); i++) {
            ids_str += std::to_string(detected_ids[i]);
            if (i < detected_ids.size() - 1)
                ids_str += ", ";
        }
        RCLCPP_INFO(this->get_logger(), "Detected %zu tags: [%s]", detected_ids.size(), ids_str.c_str());
        last_detected_ids_ = detected_ids;
    }

    // Estimate poses using multi-tag estimator
    std::vector<Eigen::Matrix4f> point_transforms;
    bool                         success = estimator_->estimate(tags, vis_image, point_transforms);

    if (!success) {
        if (detection_active_) {
            RCLCPP_WARN(this->get_logger(), "Pose estimation failed - not all required markers detected");
            detection_active_ = false;
        }
        return;
    }

    detection_active_ = true;

    // Publish poses
    publishPoses(point_transforms, msg->header.stamp);

    // Publish visualization
    if (publish_visualization_) {
        publishVisualization(point_transforms, msg->header.stamp);
    }

    // Publish detection image
    if (publish_detection_image_) {
        auto detection_msg = cv_bridge::CvImage(msg->header, "bgr8", vis_image).toImageMsg();
        detection_pub_->publish(*detection_msg);
    }
}

// ========================================================================
// Publishing Functions
// ========================================================================
void PoseEstimatorNode::publishPoses(
    const std::vector<Eigen::Matrix4f>& transforms,
    const rclcpp::Time&                 timestamp) {
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp    = timestamp;
    pose_array.header.frame_id = camera_frame_;

    for (const auto& transform : transforms) {
        pose_array.poses.push_back(matrixToPose(transform));
    }

    pose_pub_->publish(pose_array);
}

void PoseEstimatorNode::publishVisualization(
    const std::vector<Eigen::Matrix4f>& transforms,
    const rclcpp::Time&                 timestamp) {
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < transforms.size(); i++) {
        visualization_msgs::msg::Marker marker;
        marker.header.stamp    = timestamp;
        marker.header.frame_id = camera_frame_;
        marker.ns              = "target_points";
        marker.id              = i;
        marker.type            = visualization_msgs::msg::Marker::SPHERE;
        marker.action          = visualization_msgs::msg::Marker::ADD;

        auto pose   = matrixToPose(transforms[i]);
        marker.pose = pose;

        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;

        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker_array.markers.push_back(marker);
    }

    marker_pub_->publish(marker_array);
}

// ========================================================================
// Utility Functions
// ========================================================================
geometry_msgs::msg::Pose PoseEstimatorNode::matrixToPose(const Eigen::Matrix4f& matrix) {
    geometry_msgs::msg::Pose pose;

    // Extract translation
    pose.position.x = matrix(0, 3);
    pose.position.y = matrix(1, 3);
    pose.position.z = matrix(2, 3);

    // Extract rotation and convert to quaternion
    Eigen::Matrix3f    rotation = matrix.block<3, 3>(0, 0);
    Eigen::Quaternionf quat(rotation);

    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();

    return pose;
}

}  // namespace apriltag_pose_estimator

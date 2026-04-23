#include "apriltag_pose_estimator/pose_estimator_node.hpp"

#include <Eigen/Geometry>
#include <cstdlib>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

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
    this->declare_parameter("show_service_result_window", false);
    this->declare_parameter("display_width", 0);
    this->declare_parameter("display_height", 0);
    // * Input topics
    this->declare_parameter("camera_topic", "realsense_node/color/image_raw");
    this->declare_parameter("camera_info_topic", "realsense_node/color/camera_info");
    this->declare_parameter("camera_frame", "camera_color_optical_frame");
    // * Distortion handling parameter
    this->declare_parameter("use_distortion_from_camera_info", false);
    // * Output topics
    this->declare_parameter("tag_detection_topic", "pose_estimator_node/tag_detections");
    // * Service server
    this->declare_parameter("target_point_pose_service", "pose_estimator_node/target_point_pose");

    // ---------- Get Parameters ----------
    // * AprilTag configuration
    marker_ids_                 = this->get_parameter("marker_ids").as_integer_array();
    base_marker_id_             = this->get_parameter("base_marker_id").as_int();
    tag_size_                   = this->get_parameter("tag_size").as_double();
    tag_family_                 = this->get_parameter("tag_family").as_string();
    marker_offsets_             = this->get_parameter("marker_offsets").as_double_array();
    target_points_flat_         = this->get_parameter("target_points").as_double_array();
    show_service_result_window_ = this->get_parameter("show_service_result_window").as_bool();
    display_width_              = this->get_parameter("display_width").as_int();
    display_height_             = this->get_parameter("display_height").as_int();

    // * Input topics
    camera_topic_      = this->get_parameter("camera_topic").as_string();
    camera_info_topic_ = this->get_parameter("camera_info_topic").as_string();
    camera_frame_      = this->get_parameter("camera_frame").as_string();

    // * Output topics
    tag_detection_topic_ = this->get_parameter("tag_detection_topic").as_string();

    // * Service server
    target_point_pose_service_ = this->get_parameter("target_point_pose_service").as_string();

    // * Distortion handling
    use_distortion_from_camera_info_ =
        this->get_parameter("use_distortion_from_camera_info").as_bool();

    // ---------- OpenCV GUI Thread ----------
    cv::startWindowThread();

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
    tag_detection_pub_ = this->create_publisher<apriltag_pose_estimator_msgs::msg::TagDetection>(
        tag_detection_topic_, 10);

    // ---------- Service server ----------
    target_point_pose_server_ = this->create_service<apriltag_pose_estimator_msgs::srv::TargetPointPose>(
        target_point_pose_service_,
        std::bind(&PoseEstimatorNode::targetPointPoseServiceCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

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
    RCLCPP_INFO(this->get_logger(), "  Target point pose service: %s", target_point_pose_service_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Show service result window: %s",
                show_service_result_window_ ? "true" : "false");
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

    // Distortion coefficients for PnP.
    //   - false (default): zeros — 입력 이미지가 이미 rectified 인 경우.
    //                      RealSense D4xx color stream 은 하드웨어에서 undistort 된 상태로
    //                      전송되며 camera_info.d 도 factory 에서 [0,...,0] 으로 들어옴.
    //   - true           : camera_info.d 를 그대로 사용. Orbbec Femto Bolt 등 raw 이미지
    //                      (color/image_raw) 를 구독할 때 필요.
    cv::Mat dist_for_pnp;
    if (use_distortion_from_camera_info_) {
        dist_for_pnp = dist_coeffs_;
    } else {
        dist_for_pnp = cv::Mat::zeros(5, 1, CV_64F);
    }

    // Create estimator now that we have camera info
    estimator_ = std::make_unique<MultiTagPoseEstimator>(
        marker_ids_int,
        marker_offsets_float,
        target_points_,
        tag_size_,
        camera_matrix_,
        dist_for_pnp,
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
            {
                std::lock_guard<std::mutex> lock(detection_mutex_);
                latest_tag_detected_ = false;
                latest_rvec_         = cv::Mat::zeros(3, 1, CV_64F);
                latest_tvec_         = cv::Mat::zeros(3, 1, CV_64F);
                latest_vis_image_    = cv::Mat();
                latest_transforms_.clear();
            }
            publishTagDetection();
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
    std::vector<Eigen::Matrix4f> target_point_transforms;
    cv::Mat                      rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat                      tvec = cv::Mat::zeros(3, 1, CV_64F);

    bool success = estimator_->estimate(tags, vis_image, rvec, tvec, target_point_transforms);

    if (!success) {
        if (detection_active_) {
            RCLCPP_WARN(this->get_logger(), "Pose estimation failed - not all required markers detected");
            detection_active_ = false;
            last_detected_ids_.clear();
            {
                std::lock_guard<std::mutex> lock(detection_mutex_);
                latest_tag_detected_ = false;
                latest_rvec_         = cv::Mat::zeros(3, 1, CV_64F);
                latest_tvec_         = cv::Mat::zeros(3, 1, CV_64F);
                latest_vis_image_    = cv::Mat();
                latest_transforms_.clear();
            }
            publishTagDetection();
        }
        return;
    }

    detection_active_ = true;

    // Store latest detection results for service response
    {
        std::lock_guard<std::mutex> lock(detection_mutex_);
        latest_tag_detected_ = true;
        latest_rvec_         = rvec;
        latest_tvec_         = tvec;
        latest_vis_image_    = vis_image.clone();
        latest_transforms_   = target_point_transforms;
        latest_timestamp_    = msg->header.stamp;
    }

    // Publish tag detection info (dist_coeffs, rvec, tvec, tag_size)
    publishTagDetection();
}

// Service callback for getting target poses
void PoseEstimatorNode::targetPointPoseServiceCallback(
    std::shared_ptr<apriltag_pose_estimator_msgs::srv::TargetPointPose::Request>  request,
    std::shared_ptr<apriltag_pose_estimator_msgs::srv::TargetPointPose::Response> response) {
    std::lock_guard<std::mutex> lock(detection_mutex_);

    if (show_service_result_window_ && !latest_vis_image_.empty()) {
        const char* display_env         = std::getenv("DISPLAY");
        const char* wayland_display_env = std::getenv("WAYLAND_DISPLAY");
        const bool  has_gui_session =
            (display_env != nullptr && display_env[0] != '\0') ||
            (wayland_display_env != nullptr && wayland_display_env[0] != '\0');

        if (!has_gui_session) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "show_service_result_window=true but no GUI session found (DISPLAY/WAYLAND_DISPLAY). "
                "Skipping result window.");
        } else {
            try {
                cv::Mat display_image = latest_vis_image_.clone();

                // Draw tvec (tx, ty, tz)
                int                        txt_y    = 40;
                std::array<std::string, 3> t_labels = {"tx", "ty", "tz"};
                for (int i = 0; i < 3; i++) {
                    char buf[64];
                    std::snprintf(buf, sizeof(buf), "%s: %.4f m", t_labels[i].c_str(),
                                  latest_tvec_.at<double>(i));
                    cv::putText(display_image, buf, cv::Point(20, txt_y),
                                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2,
                                cv::LINE_AA);
                    txt_y += 30;
                }

                // Convert rvec to roll, pitch, yaw (degrees)
                cv::Mat rot_mat;
                cv::Rodrigues(latest_rvec_, rot_mat);
                double sy    = std::sqrt(rot_mat.at<double>(0, 0) * rot_mat.at<double>(0, 0) +
                                         rot_mat.at<double>(1, 0) * rot_mat.at<double>(1, 0));
                double roll  = std::atan2(rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2));
                double pitch = std::atan2(-rot_mat.at<double>(2, 0), sy);
                double yaw   = std::atan2(rot_mat.at<double>(1, 0), rot_mat.at<double>(0, 0));

                std::array<std::pair<std::string, double>, 3> r_labels = {
                    {{"roll", roll * 180.0 / CV_PI},
                     {"pitch", pitch * 180.0 / CV_PI},
                     {"yaw", yaw * 180.0 / CV_PI}}};
                for (const auto& [label, value] : r_labels) {
                    char buf[64];
                    std::snprintf(buf, sizeof(buf), "%s: %.4f deg", label.c_str(), value);
                    cv::putText(display_image, buf, cv::Point(20, txt_y),
                                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2,
                                cv::LINE_AA);
                    txt_y += 30;
                }

                if (display_width_ > 0 && display_height_ > 0) {
                    cv::Mat display_resized;
                    cv::resize(display_image, display_resized, cv::Size(display_width_, display_height_));
                    cv::imshow("AprilTag Pose Estimator Result", display_resized);
                } else {
                    cv::imshow("AprilTag Pose Estimator Result", display_image);
                }
                cv::waitKey(1);
            } catch (const cv::Exception& e) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    5000,
                    "Failed to show service result window: %s",
                    e.what());
            }
        }
    }

    // No poses available
    if (latest_transforms_.empty()) {
        response->success = false;
        response->message = "No recent valid target point poses available";
        RCLCPP_WARN(this->get_logger(), "Service called but no recent valid target poses are available");
        return;
    }

    // Reject if no detection newer than request_time
    if (latest_timestamp_ < request->request_time) {
        response->success = false;
        response->message = "No detection newer than request_time";
        RCLCPP_WARN(
            this->get_logger(),
            "Service rejected: request_time=%.9d, latest_detection=%.9f",
            request->request_time.sec,
            latest_timestamp_.seconds());
        return;
    }

    // Fill response with latest poses
    response->data_time             = latest_timestamp_;
    response->poses.header.stamp    = latest_timestamp_;
    response->poses.header.frame_id = camera_frame_;

    for (const auto& transform : latest_transforms_) {
        response->poses.poses.push_back(matrixToPose(transform));
    }

    response->success = true;
    response->message = "Target point pose retrieved successfully";

    RCLCPP_INFO(this->get_logger(), "✅ Service: Returned %zu target poses",
                latest_transforms_.size());
}

// ========================================================================
// Publishing Functions
// ========================================================================
void PoseEstimatorNode::publishTagDetection() {
    apriltag_pose_estimator_msgs::msg::TagDetection detection_msg;

    detection_msg.tag_detected = latest_tag_detected_;
    detection_msg.tag_size     = tag_size_;

    // -------- camera matrix (3x3) --------
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            detection_msg.camera_matrix[r * 3 + c] =
                camera_matrix_.at<double>(r, c);
        }
    }

    // -------- distortion coefficients --------
    detection_msg.dist_coeffs.resize(dist_coeffs_.rows);
    for (int i = 0; i < dist_coeffs_.rows; i++) {
        detection_msg.dist_coeffs[i] = dist_coeffs_.at<double>(i, 0);
    }

    // -------- rvec / tvec --------
    for (int i = 0; i < 3; i++) {
        detection_msg.rvec[i] = latest_rvec_.at<double>(i);
        detection_msg.tvec[i] = latest_tvec_.at<double>(i);
    }

    tag_detection_pub_->publish(detection_msg);
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
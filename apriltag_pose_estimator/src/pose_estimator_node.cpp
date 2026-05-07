#include "apriltag_pose_estimator/pose_estimator_node.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>

namespace apriltag_pose_estimator {

namespace {

// NaN pose for failed group detection (avoid zero-pose ambiguity)
geometry_msgs::msg::Pose nanPose() {
    geometry_msgs::msg::Pose p;
    const float              nan = std::numeric_limits<float>::quiet_NaN();
    p.position.x                 = nan;
    p.position.y                 = nan;
    p.position.z                 = nan;
    p.orientation.x              = nan;
    p.orientation.y              = nan;
    p.orientation.z              = nan;
    p.orientation.w              = nan;
    return p;
}

// Identity target point (zero translation/rotation) → estimator returns base marker pose
TargetPoint identityTargetPoint() {
    TargetPoint tp;
    tp.position     = Eigen::Vector3f::Zero();
    tp.rotation_rpy = Eigen::Vector3f::Zero();
    return tp;
}

}  // namespace

// ========================================================================
// Constructor
// ========================================================================
PoseEstimatorNode::PoseEstimatorNode()
    : Node("pose_estimator_node"),
      camera_info_received_(false) {
    // ---------- Common AprilTag Parameters ----------
    this->declare_parameter("tag_size", 0.02778);
    this->declare_parameter("tag_family", "tagStandard41h12");
    this->declare_parameter("publish_detection_image", false);
    this->declare_parameter("show_service_result_window", false);
    this->declare_parameter("display_width", 0);
    this->declare_parameter("display_height", 0);

    // ---------- Input Topics ----------
    this->declare_parameter("camera_topic", "/camera/color/image_raw");
    this->declare_parameter("camera_info_topic", "/camera/color/camera_info");
    this->declare_parameter("camera_frame", "camera_color_optical_frame");
    this->declare_parameter("use_distortion_from_camera_info", false);

    // ---------- Output Topics (yaml override or ~/<topic> default) ----------
    this->declare_parameter("tag_detection_topic", "~/tag_detections");
    this->declare_parameter("target_poses_topic", "~/target_poses");
    this->declare_parameter("group_names_topic", "~/group_names");
    this->declare_parameter("group_status_topic", "~/group_status");
    this->declare_parameter("detection_image_topic", "~/detection_image");

    // ---------- Service ----------
    this->declare_parameter("target_point_pose_service", "~/target_point_pose");

    // ---------- Get Common Parameters ----------
    tag_size_                        = this->get_parameter("tag_size").as_double();
    tag_family_                      = this->get_parameter("tag_family").as_string();
    publish_detection_image_         = this->get_parameter("publish_detection_image").as_bool();
    show_service_result_window_      = this->get_parameter("show_service_result_window").as_bool();
    display_width_                   = this->get_parameter("display_width").as_int();
    display_height_                  = this->get_parameter("display_height").as_int();
    camera_topic_                    = this->get_parameter("camera_topic").as_string();
    camera_info_topic_               = this->get_parameter("camera_info_topic").as_string();
    camera_frame_                    = this->get_parameter("camera_frame").as_string();
    use_distortion_from_camera_info_ = this->get_parameter("use_distortion_from_camera_info").as_bool();
    tag_detection_topic_             = this->get_parameter("tag_detection_topic").as_string();
    target_poses_topic_              = this->get_parameter("target_poses_topic").as_string();
    group_names_topic_               = this->get_parameter("group_names_topic").as_string();
    group_status_topic_              = this->get_parameter("group_status_topic").as_string();
    detection_image_topic_           = this->get_parameter("detection_image_topic").as_string();
    target_point_pose_service_       = this->get_parameter("target_point_pose_service").as_string();

    // ---------- Group Parameters ----------
    parseGroupsFromParams();

    // ---------- OpenCV GUI Thread ----------
    cv::startWindowThread();

    // ---------- Detector ----------
    detector_ = std::make_unique<AprilTagDetector>(tag_family_, tag_size_);

    // ---------- Subscribers ----------
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic_,
        10,
        std::bind(&PoseEstimatorNode::imageCallback, this, std::placeholders::_1));
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_,
        10,
        std::bind(&PoseEstimatorNode::cameraInfoCallback, this, std::placeholders::_1));

    // ---------- Publishers ----------
    tag_detection_pub_ = this->create_publisher<apriltag_pose_estimator_msgs::msg::TagDetection>(
        tag_detection_topic_, 10);
    target_poses_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        target_poses_topic_, 10);
    // group_names / group_status: latched (transient_local + KEEP_LAST(1) + reliable)
    auto latched_qos  = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    group_names_pub_  = this->create_publisher<std_msgs::msg::String>(group_names_topic_, latched_qos);
    group_status_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(group_status_topic_, 10);
    detection_image_pub_ =
        this->create_publisher<sensor_msgs::msg::Image>(detection_image_topic_, 10);

    // ---------- Service ----------
    target_point_pose_server_ = this->create_service<apriltag_pose_estimator_msgs::srv::TargetPointPose>(
        target_point_pose_service_,
        std::bind(&PoseEstimatorNode::targetPointPoseServiceCallback,
                  this,
                  std::placeholders::_1,
                  std::placeholders::_2));

    // ---------- Initial state ----------
    dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
    for (const auto& name : group_names_) {
        latest_valid_per_group_[name]      = false;
        latest_rvec_per_group_[name]       = cv::Mat::zeros(3, 1, CV_64F);
        latest_tvec_per_group_[name]       = cv::Mat::zeros(3, 1, CV_64F);
        latest_transforms_per_group_[name] = {};
    }

    // ---------- Latched group_names publish ----------
    publishGroupNamesLatched();

    // ---------- Logging ----------
    RCLCPP_INFO(this->get_logger(), "AprilTag Pose Estimator initialized");
    RCLCPP_INFO(this->get_logger(), "  Tag family: %s", tag_family_.c_str());
    RCLCPP_INFO(this->get_logger(), "  Tag size: %.5f m", tag_size_);
    RCLCPP_INFO(this->get_logger(), "  Groups (%zu):", group_names_.size());
    for (const auto& name : group_names_) {
        const auto&        g = group_configs_.at(name);
        std::ostringstream ids_ss;
        for (size_t i = 0; i < g.marker_ids.size(); i++) {
            if (i)
                ids_ss << ", ";
            ids_ss << g.marker_ids[i];
        }
        RCLCPP_INFO(this->get_logger(), "    [%s] marker_ids=[%s] base=%ld",
                    name.c_str(), ids_ss.str().c_str(), g.base_marker_id);
    }
    RCLCPP_INFO(this->get_logger(), "  Service: %s", target_point_pose_service_.c_str());
    RCLCPP_INFO(this->get_logger(), "  publish_detection_image: %s",
                publish_detection_image_ ? "true" : "false");
}

// ========================================================================
// Group Parameter Parsing
// ========================================================================
void PoseEstimatorNode::parseGroupsFromParams() {
    // group_names is required (multi-group only since v3.0.0)
    this->declare_parameter("group_names", std::vector<std::string>{});
    auto names = this->get_parameter("group_names").as_string_array();

    if (names.empty()) {
        RCLCPP_FATAL(this->get_logger(),
                     "yaml must define non-empty 'group_names'. "
                     "Single-group legacy mode was removed in v3.0.0.");
        rclcpp::shutdown();
        return;
    }

    group_names_ = names;
    for (const auto& name : names) {
        const std::string prefix = "groups." + name + ".";
        this->declare_parameter(prefix + "marker_ids", std::vector<int64_t>{});
        this->declare_parameter(prefix + "base_marker_id", -1);
        this->declare_parameter(prefix + "marker_offsets", std::vector<double>{});

        GroupConfig g;
        g.marker_ids     = this->get_parameter(prefix + "marker_ids").as_integer_array();
        g.base_marker_id = this->get_parameter(prefix + "base_marker_id").as_int();
        g.marker_offsets = this->get_parameter(prefix + "marker_offsets").as_double_array();
        if (g.marker_ids.empty()) {
            RCLCPP_FATAL(this->get_logger(),
                         "Group '%s' has empty marker_ids. Define %smarker_ids in yaml.",
                         name.c_str(), prefix.c_str());
            rclcpp::shutdown();
            return;
        }
        group_configs_[name] = std::move(g);
    }
}

// ========================================================================
// Latched Group Names Publish
// ========================================================================
void PoseEstimatorNode::publishGroupNamesLatched() {
    std_msgs::msg::String msg;
    std::ostringstream    ss;
    for (size_t i = 0; i < group_names_.size(); i++) {
        if (i)
            ss << ",";
        ss << group_names_[i];
    }
    msg.data = ss.str();
    group_names_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published latched group_names: '%s'", msg.data.c_str());
}

// ========================================================================
// Camera Info Callback
// ========================================================================
void PoseEstimatorNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (camera_info_received_) {
        return;
    }

    // Extract camera matrix
    camera_matrix_                  = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix_.at<double>(0, 0) = msg->k[0];
    camera_matrix_.at<double>(1, 1) = msg->k[4];
    camera_matrix_.at<double>(0, 2) = msg->k[2];
    camera_matrix_.at<double>(1, 2) = msg->k[5];

    // Extract distortion coefficients
    if (!msg->d.empty()) {
        dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F);
        for (size_t i = 0; i < msg->d.size(); i++) {
            dist_coeffs_.at<double>(i, 0) = msg->d[i];
        }
    }

    // Distortion for PnP — false: zeros (rectified image), true: camera_info.d
    cv::Mat dist_for_pnp;
    if (use_distortion_from_camera_info_) {
        dist_for_pnp = dist_coeffs_;
    } else {
        dist_for_pnp = cv::Mat::zeros(5, 1, CV_64F);
    }

    // ---------- Validate each group + create estimators ----------
    for (const auto& name : group_names_) {
        auto& g = group_configs_.at(name);

        const size_t N = g.marker_ids.size();
        if (g.marker_offsets.size() != N * 6) {
            RCLCPP_FATAL(this->get_logger(),
                         "[%s] marker_offsets size mismatch: expected %zu (= 6 * %zu), got %zu",
                         name.c_str(), N * 6, N, g.marker_offsets.size());
            rclcpp::shutdown();
            return;
        }

        // Locate base marker index
        int base_index = -1;
        for (size_t i = 0; i < N; i++) {
            if (static_cast<int>(g.marker_ids[i]) == static_cast<int>(g.base_marker_id)) {
                base_index = static_cast<int>(i);
                break;
            }
        }
        if (base_index < 0) {
            RCLCPP_FATAL(this->get_logger(),
                         "[%s] base_marker_id=%ld not found in marker_ids",
                         name.c_str(), g.base_marker_id);
            rclcpp::shutdown();
            return;
        }

        // Force base entry to identity (zero) with WARN if non-zero
        for (int k = 0; k < 6; k++) {
            double v = g.marker_offsets[base_index * 6 + k];
            if (std::abs(v) > 1e-9) {
                RCLCPP_WARN(this->get_logger(),
                            "[%s] base_marker marker_offsets[%d]=%.6f non-zero; forcing to 0",
                            name.c_str(), base_index * 6 + k, v);
                g.marker_offsets[base_index * 6 + k] = 0.0;
            }
        }

        // Build inputs for MultiTagPoseEstimator
        std::vector<int> marker_ids_int;
        marker_ids_int.reserve(g.marker_ids.size());
        for (auto id : g.marker_ids) {
            marker_ids_int.push_back(static_cast<int>(id));
        }
        std::vector<float> marker_offsets_float;
        marker_offsets_float.reserve(g.marker_offsets.size());
        for (auto v : g.marker_offsets) {
            marker_offsets_float.push_back(static_cast<float>(v));
        }

        // Inject a single identity target so estimator output equals the base marker pose
        std::vector<TargetPoint> identity_targets = {identityTargetPoint()};

        estimators_[name] = std::make_unique<MultiTagPoseEstimator>(
            marker_ids_int,
            marker_offsets_float,
            identity_targets,
            tag_size_,
            camera_matrix_,
            dist_for_pnp,
            static_cast<int>(g.base_marker_id));
    }

    camera_info_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Camera info received");
    RCLCPP_INFO(this->get_logger(), "  fx: %.2f, fy: %.2f",
                camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(1, 1));
    RCLCPP_INFO(this->get_logger(), "  cx: %.2f, cy: %.2f",
                camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2));
    RCLCPP_INFO(this->get_logger(), "  Estimators created: %zu", estimators_.size());
}

// ========================================================================
// Image Callback
// ========================================================================
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

    // ---------- Detect AprilTags (single pass, group-agnostic) ----------
    std::vector<TagDetection> all_tags = detector_->detect(cv_image, camera_matrix_, dist_coeffs_);

    // Log detected IDs only when changed
    std::vector<int> detected_ids;
    detected_ids.reserve(all_tags.size());
    for (const auto& tag : all_tags) {
        detected_ids.push_back(tag.id);
    }
    std::sort(detected_ids.begin(), detected_ids.end());
    if (detected_ids != last_detected_ids_) {
        std::ostringstream ids_ss;
        for (size_t i = 0; i < detected_ids.size(); i++) {
            if (i)
                ids_ss << ", ";
            ids_ss << detected_ids[i];
        }
        RCLCPP_INFO(this->get_logger(), "Detected %zu tags: [%s]",
                    detected_ids.size(), ids_ss.str().c_str());
        last_detected_ids_ = detected_ids;
    }

    // ---------- Per-group estimate ----------
    geometry_msgs::msg::PoseArray pose_array_msg;
    pose_array_msg.header.stamp    = msg->header.stamp;
    pose_array_msg.header.frame_id = camera_frame_;

    std_msgs::msg::UInt8MultiArray status_msg;
    status_msg.data.reserve(group_names_.size());

    {
        std::lock_guard<std::mutex> lock(detection_mutex_);

        for (const auto& name : group_names_) {
            const auto& cfg     = group_configs_.at(name);
            auto&       est     = estimators_.at(name);
            auto&       last_v  = latest_valid_per_group_[name];
            auto&       last_r  = latest_rvec_per_group_[name];
            auto&       last_t  = latest_tvec_per_group_[name];
            auto&       last_xf = latest_transforms_per_group_[name];

            // Filter detected tags by this group's marker_ids
            std::vector<TagDetection> group_tags;
            group_tags.reserve(cfg.marker_ids.size());
            for (const auto& tag : all_tags) {
                for (auto id : cfg.marker_ids) {
                    if (tag.id == static_cast<int>(id)) {
                        group_tags.push_back(tag);
                        break;
                    }
                }
            }

            // Estimate for this group
            std::vector<Eigen::Matrix4f> transforms;
            cv::Mat                      rvec = cv::Mat::zeros(3, 1, CV_64F);
            cv::Mat                      tvec = cv::Mat::zeros(3, 1, CV_64F);
            const bool                   ok   = !group_tags.empty()
                                                    ? est->estimate(group_tags, vis_image, rvec, tvec, transforms)
                                                    : false;

            if (ok) {
                last_v  = true;
                last_r  = rvec;
                last_t  = tvec;
                last_xf = transforms;
                for (const auto& xf : transforms) {
                    pose_array_msg.poses.push_back(matrixToPose(xf));
                }
                status_msg.data.push_back(1);
            } else {
                last_v = false;
                last_xf.clear();
                // Fill single NaN pose (one per group → base marker pose)
                pose_array_msg.poses.push_back(nanPose());
                status_msg.data.push_back(0);
            }
        }

        // Cache vis_image for service result window (after all-group overlays)
        latest_vis_image_ = vis_image.clone();
        latest_timestamp_ = msg->header.stamp;
    }

    // ---------- Publish ----------
    target_poses_pub_->publish(pose_array_msg);
    group_status_pub_->publish(status_msg);

    if (publish_detection_image_) {
        // Single vis_image with all-group overlays. Skip encoding when no subscriber.
        if (detection_image_pub_->get_subscription_count() > 0) {
            std_msgs::msg::Header header;
            header.stamp    = msg->header.stamp;
            header.frame_id = camera_frame_;
            auto img_msg    = cv_bridge::CvImage(header, "bgr8", vis_image).toImageMsg();
            detection_image_pub_->publish(*img_msg);
        }
    }

    // tag_detection uses first group's rvec/tvec (debug only in multi-group mode)
    publishTagDetection(group_names_.front());
}

// ========================================================================
// Service Callback
// ========================================================================
void PoseEstimatorNode::targetPointPoseServiceCallback(
    std::shared_ptr<apriltag_pose_estimator_msgs::srv::TargetPointPose::Request>  request,
    std::shared_ptr<apriltag_pose_estimator_msgs::srv::TargetPointPose::Response> response) {
    std::lock_guard<std::mutex> lock(detection_mutex_);

    // Group dispatch: empty group_name → first group in group_names
    std::string group = request->group_name;
    if (group.empty()) {
        group = group_names_.front();
    }
    auto cfg_it = group_configs_.find(group);
    if (cfg_it == group_configs_.end()) {
        response->success = false;
        response->message = "Unknown group_name: '" + group + "'";
        RCLCPP_WARN(this->get_logger(), "Service: unknown group '%s'", group.c_str());
        return;
    }

    if (show_service_result_window_ && !latest_vis_image_.empty()) {
        const char* display_env         = std::getenv("DISPLAY");
        const char* wayland_display_env = std::getenv("WAYLAND_DISPLAY");
        const bool  has_gui_session =
            (display_env != nullptr && display_env[0] != '\0') ||
            (wayland_display_env != nullptr && wayland_display_env[0] != '\0');

        if (!has_gui_session) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "show_service_result_window=true but no GUI session.");
        } else {
            try {
                cv::Mat        display_image = latest_vis_image_.clone();
                const cv::Mat& rvec_ref      = latest_rvec_per_group_[group];
                const cv::Mat& tvec_ref      = latest_tvec_per_group_[group];

                int                        txt_y    = 40;
                std::array<std::string, 3> t_labels = {"tx", "ty", "tz"};
                for (int i = 0; i < 3; i++) {
                    char buf[64];
                    std::snprintf(buf, sizeof(buf), "%s: %.4f m", t_labels[i].c_str(),
                                  tvec_ref.at<double>(i));
                    cv::putText(display_image, buf, cv::Point(20, txt_y),
                                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2,
                                cv::LINE_AA);
                    txt_y += 30;
                }

                cv::Mat rot_mat;
                cv::Rodrigues(rvec_ref, rot_mat);
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
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Failed to show service result window: %s", e.what());
            }
        }
    }

    const auto& transforms = latest_transforms_per_group_[group];
    if (!latest_valid_per_group_[group] || transforms.empty()) {
        response->success = false;
        response->message = "No recent valid target poses for group '" + group + "'";
        RCLCPP_WARN(this->get_logger(),
                    "Service: no recent valid poses for group '%s'", group.c_str());
        return;
    }

    // Reject if no detection newer than request_time
    if (latest_timestamp_ < request->request_time) {
        response->success = false;
        response->message = "No detection newer than request_time";
        return;
    }

    response->data_time             = latest_timestamp_;
    response->poses.header.stamp    = latest_timestamp_;
    response->poses.header.frame_id = camera_frame_;
    for (const auto& xf : transforms) {
        response->poses.poses.push_back(matrixToPose(xf));
    }
    response->success = true;
    response->message = "Target point pose retrieved successfully (group='" + group + "')";

    RCLCPP_INFO(this->get_logger(), "Service: returned %zu poses for group '%s'",
                transforms.size(), group.c_str());
}

// ========================================================================
// publishTagDetection (uses reference_group's rvec/tvec; debug only)
// ========================================================================
void PoseEstimatorNode::publishTagDetection(const std::string& reference_group) {
    apriltag_pose_estimator_msgs::msg::TagDetection detection_msg;

    const bool  valid    = latest_valid_per_group_[reference_group];
    const auto& rvec_ref = latest_rvec_per_group_[reference_group];
    const auto& tvec_ref = latest_tvec_per_group_[reference_group];

    detection_msg.tag_detected = valid;
    detection_msg.tag_size     = tag_size_;

    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            detection_msg.camera_matrix[r * 3 + c] = camera_matrix_.at<double>(r, c);
        }
    }
    detection_msg.dist_coeffs.resize(dist_coeffs_.rows);
    for (int i = 0; i < dist_coeffs_.rows; i++) {
        detection_msg.dist_coeffs[i] = dist_coeffs_.at<double>(i, 0);
    }
    for (int i = 0; i < 3; i++) {
        detection_msg.rvec[i] = rvec_ref.at<double>(i);
        detection_msg.tvec[i] = tvec_ref.at<double>(i);
    }

    tag_detection_pub_->publish(detection_msg);
}

// ========================================================================
// Utility
// ========================================================================
geometry_msgs::msg::Pose PoseEstimatorNode::matrixToPose(const Eigen::Matrix4f& matrix) {
    geometry_msgs::msg::Pose pose;
    pose.position.x             = matrix(0, 3);
    pose.position.y             = matrix(1, 3);
    pose.position.z             = matrix(2, 3);
    Eigen::Matrix3f    rotation = matrix.block<3, 3>(0, 0);
    Eigen::Quaternionf quat(rotation);
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    return pose;
}

geometry_msgs::msg::Pose PoseEstimatorNode::makeNanPose() {
    return nanPose();
}

}  // namespace apriltag_pose_estimator

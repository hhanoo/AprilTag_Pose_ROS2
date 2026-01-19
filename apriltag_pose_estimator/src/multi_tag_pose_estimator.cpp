#include "apriltag_pose_estimator/multi_tag_pose_estimator.hpp"

#include <algorithm>
#include <iostream>

namespace apriltag_pose_estimator {

// ========================================================================
// Constructor
// ========================================================================
MultiTagPoseEstimator::MultiTagPoseEstimator(
    const std::vector<int>&         marker_ids,
    const std::vector<float>&       marker_offsets,
    const std::vector<TargetPoint>& target_points,
    float                           tag_size,
    const cv::Mat&                  camera_matrix,
    const cv::Mat&                  dist_coeffs,
    int                             base_marker_id)
    : marker_ids_(marker_ids),
      target_points_(target_points),
      target_point_count_(target_points.size()),
      tag_size_(tag_size),
      camera_matrix_(camera_matrix.clone()),
      dist_coeffs_(dist_coeffs.clone()) {
    // Initialize marker offsets
    marker_offset_x_ = marker_offsets[0];
    marker_offset_y_ = marker_offsets[1];

    // Set base marker ID
    if (base_marker_id == -1) {
        base_marker_id_ = marker_ids[0];
    } else {
        base_marker_id_ = base_marker_id;
    }
}

// ========================================================================
// Pose Estimation Functions
// ========================================================================
bool MultiTagPoseEstimator::estimate(
    const std::vector<TagDetection>& tags,
    cv::Mat&                         img,
    std::vector<Eigen::Matrix4f>&    point_transforms) {
    // 1. Make a dictionary for fast lookup: tag_id → tag
    std::map<int, TagDetection> tag_dict;
    for (const auto& tag : tags) {
        tag_dict[tag.id] = tag;
    }

    // 2. Check if all required marker IDs are found in the detected tags
    for (int marker_id : marker_ids_) {
        if (tag_dict.find(marker_id) == tag_dict.end()) {
            return false;  // Not all required markers detected
        }
    }

    // 3. Prepare 3D (world) and 2D (image) correspondence arrays
    size_t                   num_markers = marker_ids_.size();
    std::vector<cv::Point3f> pt3D;
    std::vector<cv::Point2f> pt2D;
    pt3D.reserve(num_markers * 4);  // Pre-allocate memory (4 corners per marker)
    pt2D.reserve(num_markers * 4);

    float a  = tag_size_ / 2.0f;  // Half tag size
    float ox = marker_offset_x_;
    float oy = marker_offset_y_;

    // Find base marker index
    int base_index = 0;
    for (size_t i = 0; i < marker_ids_.size(); i++) {
        if (marker_ids_[i] == base_marker_id_) {
            base_index = i;
            break;
        }
    }

    // 4. Fill in pt3D and pt2D by following marker ID order
    for (size_t i = 0; i < marker_ids_.size(); i++) {
        int   marker_id = marker_ids_[i];
        float dx        = ox * (static_cast<int>(i) - base_index);
        float dy        = oy * (static_cast<int>(i) - base_index);

        // 4.1 Define world coordinates for the four corners of each marker
        pt3D.push_back(cv::Point3f(-a + dx, a + dy, 0.0f));   // top-left
        pt3D.push_back(cv::Point3f(a + dx, a + dy, 0.0f));    // top-right
        pt3D.push_back(cv::Point3f(a + dx, -a + dy, 0.0f));   // bottom-right
        pt3D.push_back(cv::Point3f(-a + dx, -a + dy, 0.0f));  // bottom-left

        // 4.2 Use the tag's detected corners (2D image points)
        const auto& corners = tag_dict[marker_id].corners;
        pt2D.insert(pt2D.end(), corners.begin(), corners.end());
    }

    // 5. Estimate camera pose using solvePnP
    cv::Mat rvec, tvec;
    bool    success = cv::solvePnP(pt3D, pt2D, camera_matrix_, dist_coeffs_, rvec, tvec);

    if (!success) {
        return false;  // Pose estimation failed
    }

    // 6. Convert to 4x4 transformation matrix
    Eigen::Matrix4f T_cam_2_marker = rvecTvecToMatrix(rvec, tvec);

    // 7. Compute point poses and project them onto the image
    point_transforms.clear();
    point_transforms.reserve(target_points_.size());  // Pre-allocate memory

    for (size_t i = 0; i < target_points_.size(); i++) {
        // 7.0 Target point pose in marker frame
        Eigen::Vector3f t_offset(
            target_points_[i].position(0),
            target_points_[i].position(1),
            target_points_[i].position(2));
        Eigen::AngleAxisf roll(
            target_points_[i].rotation_rpy(0),
            Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitch(
            target_points_[i].rotation_rpy(1),
            Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yaw(
            target_points_[i].rotation_rpy(2),
            Eigen::Vector3f::UnitZ());

        Eigen::Matrix3f R_offset = yaw.matrix() * pitch.matrix() * roll.matrix();

        Eigen::Matrix4f T_marker_2_target_point   = Eigen::Matrix4f::Identity();
        T_marker_2_target_point.block<3, 3>(0, 0) = R_offset;
        T_marker_2_target_point.block<3, 1>(0, 3) = t_offset;

        // 7.1 Target point pose in camera frame
        Eigen::Matrix4f T_camera_2_target_point = T_cam_2_marker * T_marker_2_target_point;
        Eigen::Vector3f pos_cam                 = T_camera_2_target_point.block<3, 1>(0, 3);

        // 7.2 Image projection
        Eigen::Vector3f img_point_eigen;
        img_point_eigen(0) = camera_matrix_.at<double>(0, 0) * pos_cam(0) +
                             camera_matrix_.at<double>(0, 2) * pos_cam(2);
        img_point_eigen(1) = camera_matrix_.at<double>(1, 1) * pos_cam(1) +
                             camera_matrix_.at<double>(1, 2) * pos_cam(2);
        img_point_eigen(2) = pos_cam(2);

        if (img_point_eigen(2) > 0) {
            img_point_eigen /= img_point_eigen(2);

            // 7.3 Draw white circle with black border
            cv::Point img_point(
                static_cast<int>(img_point_eigen(0)),
                static_cast<int>(img_point_eigen(1)));
            cv::circle(img, img_point, 5, cv::Scalar(255, 255, 255), -1);  // Fill white
            cv::circle(img, img_point, 5, cv::Scalar(0, 0, 0), 2);         // Black border
        }

        // 7.4 Store the transformation matrix for this point
        point_transforms.push_back(T_camera_2_target_point);
    }

    // 8. Visualize the base marker pose using axes
    cv::drawFrameAxes(img, camera_matrix_, dist_coeffs_, rvec, tvec, tag_size_);

    return true;
}

// ========================================================================
// Utility Functions
// ========================================================================
Eigen::Matrix4f MultiTagPoseEstimator::rvecTvecToMatrix(
    const cv::Mat& rvec,
    const cv::Mat& tvec) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
            T(row, col) = R.at<double>(row, col);
        }
        T(row, 3) = tvec.at<double>(row, 0);
    }

    return T;
}

}  // namespace apriltag_pose_estimator

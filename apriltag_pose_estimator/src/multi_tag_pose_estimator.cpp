#include "apriltag_pose_estimator/multi_tag_pose_estimator.hpp"

#include <algorithm>
#include <array>
#include <iostream>
#include <stdexcept>

namespace apriltag_pose_estimator {

// ========================================================================
// Constructor
// ========================================================================
MultiTagPoseEstimator::MultiTagPoseEstimator(
    const std::vector<int>&   marker_ids,
    const std::vector<float>& marker_offsets_6n,
    float                     tag_size,
    const cv::Mat&            camera_matrix,
    const cv::Mat&            dist_coeffs,
    int                       base_marker_id)
    : marker_ids_(marker_ids),
      tag_size_(tag_size),
      camera_matrix_(camera_matrix.clone()),
      dist_coeffs_(dist_coeffs.clone()) {
    // Set base marker ID
    if (base_marker_id == -1) {
        base_marker_id_ = marker_ids[0];
    } else {
        base_marker_id_ = base_marker_id;
    }

    // Find base marker index
    base_index_ = -1;
    for (size_t i = 0; i < marker_ids_.size(); i++) {
        if (marker_ids_[i] == base_marker_id_) {
            base_index_ = static_cast<int>(i);
            break;
        }
    }
    if (base_index_ < 0) {
        throw std::invalid_argument(
            "MultiTagPoseEstimator: base_marker_id not found in marker_ids");
    }

    // Validate flat list length: 6 values per marker
    const size_t N = marker_ids_.size();
    if (marker_offsets_6n.size() != N * 6) {
        throw std::invalid_argument(
            "MultiTagPoseEstimator: marker_offsets must have size 6 * marker_ids.size()");
    }

    // Compile per-marker T_base <- marker_i (rotation order: Rz * Ry * Rx)
    marker_T_base_to_marker_.clear();
    marker_T_base_to_marker_.reserve(N);
    for (size_t i = 0; i < N; i++) {
        float x  = marker_offsets_6n[i * 6 + 0];
        float y  = marker_offsets_6n[i * 6 + 1];
        float z  = marker_offsets_6n[i * 6 + 2];
        float r  = marker_offsets_6n[i * 6 + 3];
        float p  = marker_offsets_6n[i * 6 + 4];
        float yw = marker_offsets_6n[i * 6 + 5];

        // base_marker entry is identity by definition.
        if (static_cast<int>(i) == base_index_) {
            x = y = z = r = p = yw = 0.0f;
        }

        Eigen::AngleAxisf Rx(r, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf Ry(p, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf Rz(yw, Eigen::Vector3f::UnitZ());
        Eigen::Matrix3f   R = Rz.matrix() * Ry.matrix() * Rx.matrix();

        Eigen::Matrix4f T   = Eigen::Matrix4f::Identity();
        T.block<3, 3>(0, 0) = R;
        T.block<3, 1>(0, 3) = Eigen::Vector3f(x, y, z);
        marker_T_base_to_marker_.push_back(T);
    }
}

// ========================================================================
// Pose Estimation Functions
// ========================================================================
bool MultiTagPoseEstimator::estimate(
    const std::vector<TagDetection>& tags,
    cv::Mat&                         img,
    cv::Mat&                         rvec,
    cv::Mat&                         tvec,
    Eigen::Matrix4f&                 base_transform_out) {
    // 1. Make a dictionary for fast lookup: tag_id → tag
    std::map<int, TagDetection> tag_dict;
    for (const auto& tag : tags) {
        tag_dict[tag.id] = tag;
    }

    // 2. Check if all required marker IDs are found in the detected tags
    for (int marker_id : marker_ids_) {
        if (tag_dict.find(marker_id) == tag_dict.end()) {
            registerMiss();
            return false;  // Not all required markers detected
        }
    }

    // 3. Prepare 3D (world) and 2D (image) correspondence arrays
    size_t                   num_markers = marker_ids_.size();
    std::vector<cv::Point3f> pt3D;
    std::vector<cv::Point2f> pt2D;
    pt3D.reserve(num_markers * 4);  // Pre-allocate memory (4 corners per marker)
    pt2D.reserve(num_markers * 4);

    float a = tag_size_ / 2.0f;  // Half tag size

    // Local marker corners (each marker's own frame, z = 0).
    const std::array<Eigen::Vector4f, 4> local_corners = {
        Eigen::Vector4f(-a, +a, 0.0f, 1.0f),  // top-left
        Eigen::Vector4f(+a, +a, 0.0f, 1.0f),  // top-right
        Eigen::Vector4f(+a, -a, 0.0f, 1.0f),  // bottom-right
        Eigen::Vector4f(-a, -a, 0.0f, 1.0f),  // bottom-left
    };

    // 4. Fill in pt3D and pt2D by following marker ID order.
    //    pt3D is expressed in the base_marker frame so that solvePnP yields
    //    the camera-to-base_marker transform directly.
    for (size_t i = 0; i < marker_ids_.size(); i++) {
        const int              marker_id = marker_ids_[i];
        const Eigen::Matrix4f& T         = marker_T_base_to_marker_[i];

        // 4.1 Transform each local corner into base_marker frame
        for (const auto& c : local_corners) {
            Eigen::Vector4f p_base = T * c;
            pt3D.emplace_back(p_base.x(), p_base.y(), p_base.z());
        }

        // 4.2 Use the tag's detected corners (2D image points)
        const auto& corners = tag_dict[marker_id].corners;
        pt2D.insert(pt2D.end(), corners.begin(), corners.end());
    }

    // 5. Estimate camera pose using solvePnP (Using SOLVEPNP_ITERATIVE)
    bool success     = cv::solvePnP(pt3D, pt2D, camera_matrix_, dist_coeffs_, rvec, tvec);
    bool reused_pose = false;  // True if last pose is reused

    if (!success) {
        registerMiss();
        return false;  // Pose estimation failed
    }

    // 6. Reprojection Error-based Outlier Rejection
    // 추정된 pose로 3D 포인트를 이미지에 재투영하여 평균 거리를 계산하고,
    // 임계값(max_reproj_error_) 초과 시 비정상 프레임으로 판단하여 이전 유효 pose를 사용
    {
        // 6.1 Reproject 3D points to image plane
        std::vector<cv::Point2f> projected;
        cv::projectPoints(pt3D, rvec, tvec, camera_matrix_, dist_coeffs_, projected);

        // 6.2 Calculate mean reprojection error
        double totalErr = 0.0;
        for (size_t i = 0; i < pt2D.size(); ++i) {
            double dx = projected[i].x - pt2D[i].x;
            double dy = projected[i].y - pt2D[i].y;
            totalErr += std::sqrt(dx * dx + dy * dy);
        }
        double meanReprojErr = totalErr / pt2D.size();

        // 6.3 Outlier Rejection (reuse last pose up to max_pose_reuse_ frames)
        if (meanReprojErr > max_reproj_error_) {
            if (has_last_valid_pose_ && reuse_count_ < max_pose_reuse_) {
                rvec        = last_rvec_.clone();
                tvec        = last_tvec_.clone();
                reused_pose = true;
                reuse_count_++;
            } else {
                resetStaleState();
                return false;
            }
        } else {
            last_rvec_           = rvec.clone();
            last_tvec_           = tvec.clone();
            has_last_valid_pose_ = true;
            reuse_count_         = 0;
        }
    }

    // 7. Convert to 4x4 transformation matrix & SLERP 필터 적용
    Eigen::Matrix4f T_cam_2_marker          = rvecTvecToMatrix(rvec, tvec);
    Eigen::Matrix4f T_cam_2_marker_filtered = pose_filter_.filter(T_cam_2_marker);

    // 7.1 Update rvec/tvec to filtered values
    {
        cv::Mat R_f(3, 3, CV_64F);
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                R_f.at<double>(r, c) = static_cast<double>(T_cam_2_marker_filtered(r, c));
        cv::Rodrigues(R_f, rvec);
        tvec.at<double>(0) = T_cam_2_marker_filtered(0, 3);
        tvec.at<double>(1) = T_cam_2_marker_filtered(1, 3);
        tvec.at<double>(2) = T_cam_2_marker_filtered(2, 3);
    }

    // 8. Output: filtered base marker pose (camera frame)
    base_transform_out = T_cam_2_marker_filtered;

    // 9. Visualize the base marker pose using axes (skip reused poses)
    if (!reused_pose) {
        cv::drawFrameAxes(img, camera_matrix_, dist_coeffs_, rvec, tvec, tag_size_);
    }

    miss_count_ = 0;
    return true;
}

void MultiTagPoseEstimator::resetStaleState() {
    has_last_valid_pose_ = false;
    reuse_count_         = 0;
    pose_filter_.reset();
}

void MultiTagPoseEstimator::registerMiss() {
    miss_count_++;
    // Reset filter/cache after a long detection gap
    if (miss_count_ >= max_miss_reset_) {
        resetStaleState();
    }
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

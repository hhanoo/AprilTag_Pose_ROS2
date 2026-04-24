#ifndef APRILTAG_POSE_ESTIMATOR__MULTI_TAG_POSE_ESTIMATOR_HPP_
#define APRILTAG_POSE_ESTIMATOR__MULTI_TAG_POSE_ESTIMATOR_HPP_

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

#include "apriltag_pose_estimator/slerp_pose_filter.hpp"
#include "apriltag_pose_estimator/tag_config.hpp"

namespace apriltag_pose_estimator {

class MultiTagPoseEstimator {
   public:
    // ========================================================================
    // Constructor
    // ========================================================================
    MultiTagPoseEstimator(
        const std::vector<int>&         marker_ids,          // Marker IDs to use
        const std::vector<float>&       marker_offsets_6n,   // Per-marker 6-DoF flat (6*N)
        const std::vector<TargetPoint>& target_points,       // Target point offsets (N)
        float                           tag_size,            // Tag size in meters
        const cv::Mat&                  camera_matrix,       // Camera intrinsic matrix
        const cv::Mat&                  dist_coeffs,         // Distortion coefficients
        int                             base_marker_id = -1  // Base marker ID (default: first marker ID)
    );

    // ========================================================================
    // Pose Estimation Functions
    // ========================================================================
    bool estimate(
        const std::vector<TagDetection>& tags,                    // Detected AprilTags
        cv::Mat&                         img,                     // Image to draw on
        cv::Mat&                         rvec,                    // Rotation vector
        cv::Mat&                         tvec,                    // Translation vector
        std::vector<Eigen::Matrix4f>&    target_point_transforms  // Output: target point transforms
    );

   private:
    // ========================================================================
    // Utility Functions
    // ========================================================================
    Eigen::Matrix4f rvecTvecToMatrix(
        const cv::Mat& rvec,  // Rotation vector
        const cv::Mat& tvec   // Translation vector
    );

    // ========================================================================
    // Member Variables
    // ========================================================================
    std::vector<int>             marker_ids_;               // Marker IDs to use (default: [0, 1, 2])
    std::vector<Eigen::Matrix4f> marker_T_base_to_marker_;  // T_base <- marker_i, length N
    int                          base_index_;               // Index of base_marker_id in marker_ids_
    std::vector<TargetPoint>     target_points_;            // Target point offsets (N)
    int                          target_point_count_;       // Number of target points (default: 6)
    float                        tag_size_;                 // Tag size in meters (default: 0.02778)
    cv::Mat                      camera_matrix_;            // Camera intrinsic matrix (3x3)
    cv::Mat                      dist_coeffs_;              // Distortion coefficients (5x1)
    int                          base_marker_id_;           // Base marker ID (default: first marker ID)

    // Reprojection Error-based Outlier Rejection
    bool    has_last_valid_pose_ = false;  // Whether the previous valid pose exists
    cv::Mat last_rvec_;                    // Last valid rotation vector
    cv::Mat last_tvec_;                    // Last valid translation vector
    double  max_reproj_error_ = 15.0;      // Maximum reprojection error threshold (pixel units)

    // Quaternion SLERP EMA filter
    PoseFilter pose_filter_;
};

}  // namespace apriltag_pose_estimator

#endif  // APRILTAG_POSE_ESTIMATOR__MULTI_TAG_POSE_ESTIMATOR_HPP_

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
        const std::vector<int>&   marker_ids,          // Marker IDs to use
        const std::vector<float>& marker_offsets_6n,   // Per-marker 6-DoF flat (6*N)
        float                     tag_size,            // Tag size in meters
        const cv::Mat&            camera_matrix,       // Camera intrinsic matrix
        const cv::Mat&            dist_coeffs,         // Distortion coefficients
        int                       base_marker_id = -1  // Base marker ID (default: first marker ID)
    );

    // ========================================================================
    // Pose Estimation Functions
    // ========================================================================
    // base_transform_out: filtered T_camera <- base_marker
    bool estimate(
        const std::vector<TagDetection>& tags,               // Detected AprilTags
        cv::Mat&                         img,                // Image to draw on
        cv::Mat&                         rvec,               // Rotation vector
        cv::Mat&                         tvec,               // Translation vector
        Eigen::Matrix4f&                 base_transform_out  // Output: base marker pose (camera frame)
    );

   private:
    // ========================================================================
    // Utility Functions
    // ========================================================================
    Eigen::Matrix4f rvecTvecToMatrix(
        const cv::Mat& rvec,  // Rotation vector
        const cv::Mat& tvec   // Translation vector
    );

    // Reset stale state (last pose cache + filter)
    void resetStaleState();

    // Count a failure; reset stale state after too many
    void registerMiss();

    // ========================================================================
    // Member Variables
    // ========================================================================
    std::vector<int>             marker_ids_;               // Marker IDs to use (default: [0, 1, 2])
    std::vector<Eigen::Matrix4f> marker_T_base_to_marker_;  // T_base <- marker_i, length N
    int                          base_index_;               // Index of base_marker_id in marker_ids_
    float                        tag_size_;                 // Tag size in meters (default: 0.02778)
    cv::Mat                      camera_matrix_;            // Camera intrinsic matrix (3x3)
    cv::Mat                      dist_coeffs_;              // Distortion coefficients (5x1)
    int                          base_marker_id_;           // Base marker ID (default: first marker ID)

    // Reprojection Error-based Outlier Rejection
    bool    has_last_valid_pose_ = false;  // Whether the previous valid pose exists
    cv::Mat last_rvec_;                    // Last valid rotation vector
    cv::Mat last_tvec_;                    // Last valid translation vector
    double  max_reproj_error_ = 15.0;      // Maximum reprojection error threshold (pixel units)
    int     reuse_count_      = 0;         // Consecutive frames the last pose was reused
    int     max_pose_reuse_   = 5;         // Max consecutive reuses before failing
    int     miss_count_       = 0;         // Consecutive estimate failures
    int     max_miss_reset_   = 5;         // Consecutive failures that trigger a reset

    // Quaternion SLERP EMA filter
    PoseFilter pose_filter_;
};

}  // namespace apriltag_pose_estimator

#endif  // APRILTAG_POSE_ESTIMATOR__MULTI_TAG_POSE_ESTIMATOR_HPP_

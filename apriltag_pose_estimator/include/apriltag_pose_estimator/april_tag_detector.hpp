#ifndef APRILTAG_POSE_ESTIMATOR__APRIL_TAG_DETECTOR_HPP_
#define APRILTAG_POSE_ESTIMATOR__APRIL_TAG_DETECTOR_HPP_

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tagStandard41h12.h>

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "apriltag_pose_estimator/tag_config.hpp"

namespace apriltag_pose_estimator {

class AprilTagDetector {
   public:
    // ========================================================================
    // Constructor / Destructor
    // ========================================================================
    AprilTagDetector(
        const std::string& tag_family,  // Tag family name (tag36h11, tagStandard41h12, etc.)
        float              tag_size     // Tag size in meters
    );
    ~AprilTagDetector();

    // ========================================================================
    // Detection Functions
    // ========================================================================
    std::vector<TagDetection> detect(
        const cv::Mat& image,          // Input image (grayscale or BGR)
        const cv::Mat& camera_matrix,  // Camera intrinsic matrix (3x3)
        const cv::Mat& dist_coeffs     // Distortion coefficients
    );

    // ========================================================================
    // Visualization Functions
    // ========================================================================
    void drawDetections(
        cv::Mat&                         image,          // Image to draw on
        const std::vector<TagDetection>& detections,     // Detected tags
        const cv::Mat&                   camera_matrix,  // Camera intrinsic matrix
        const cv::Mat&                   dist_coeffs,    // Distortion coefficients
        float                            tag_size        // Tag size in meters
    );

   private:
    // ========================================================================
    // Member Variables
    // ========================================================================
    apriltag_detector_t* td_;
    apriltag_family_t*   tf_;
    std::string          tag_family_;
    float                tag_size_;
};

}  // namespace apriltag_pose_estimator

#endif  // APRILTAG_POSE_ESTIMATOR__APRIL_TAG_DETECTOR_HPP_

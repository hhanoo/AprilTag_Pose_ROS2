#include "apriltag_pose_estimator/april_tag_detector.hpp"

#include <iostream>

namespace apriltag_pose_estimator {

// ========================================================================
// Constructor / Destructor
// ========================================================================
AprilTagDetector::AprilTagDetector(const std::string& tag_family, float tag_size)
    : tag_family_(tag_family), tag_size_(tag_size) {
    // Create detector
    td_ = apriltag_detector_create();

    // Create tag family
    if (tag_family == "tag36h11") {
        tf_ = tag36h11_create();
    } else if (tag_family == "tagStandard41h12") {
        tf_ = tagStandard41h12_create();
    } else {
        std::cerr << "Unknown tag family: " << tag_family << ", using tag36h11" << std::endl;
        tf_ = tag36h11_create();
    }

    apriltag_detector_add_family(td_, tf_);

    // Configure detector
    td_->quad_decimate = 2.0;
    td_->quad_sigma    = 0.0;
    td_->nthreads      = 4;
    td_->debug         = 0;
    td_->refine_edges  = 1;
}

AprilTagDetector::~AprilTagDetector() {
    if (td_) {
        apriltag_detector_destroy(td_);
    }

    if (tf_) {
        if (tag_family_ == "tag36h11") {
            tag36h11_destroy(tf_);
        } else if (tag_family_ == "tagStandard41h12") {
            tagStandard41h12_destroy(tf_);
        }
    }
}

// ========================================================================
// Detection Functions
// ========================================================================
std::vector<TagDetection> AprilTagDetector::detect(
    const cv::Mat& image,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs) {
    std::vector<TagDetection> detections;

    // Convert to grayscale if needed
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image.clone();
    }

    // Create image_u8_t for apriltag library
    image_u8_t im = {
        gray.cols,  // width
        gray.rows,  // height
        gray.cols,  // stride
        gray.data   // buf
    };

    // Detect tags
    zarray_t* detections_zarray = apriltag_detector_detect(td_, &im);

    // Process each detection
    for (int i = 0; i < zarray_size(detections_zarray); i++) {
        apriltag_detection_t* det;
        zarray_get(detections_zarray, i, &det);

        // Get corner points
        std::vector<cv::Point2f> corners;
        for (int j = 0; j < 4; j++) {
            corners.push_back(cv::Point2f(det->p[j][0], det->p[j][1]));
        }

        // Estimate pose using solvePnP
        std::vector<cv::Point3f> object_points;
        float                    half_size = tag_size_ / 2.0f;
        object_points.push_back(cv::Point3f(-half_size, half_size, 0));
        object_points.push_back(cv::Point3f(half_size, half_size, 0));
        object_points.push_back(cv::Point3f(half_size, -half_size, 0));
        object_points.push_back(cv::Point3f(-half_size, -half_size, 0));

        cv::Mat rvec, tvec;
        cv::solvePnP(object_points, corners, camera_matrix, dist_coeffs, rvec, tvec);

        // Convert to transformation matrix
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);

        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                pose(row, col) = rotation_matrix.at<double>(row, col);
            }
            pose(row, 3) = tvec.at<double>(row, 0);
        }

        // Create detection
        TagDetection detection;
        detection.id      = det->id;
        detection.pose    = pose;
        detection.corners = corners;

        detections.push_back(detection);
    }

    // Clean up
    apriltag_detections_destroy(detections_zarray);

    return detections;
}

// ========================================================================
// Visualization Functions
// ========================================================================
void AprilTagDetector::drawDetections(
    cv::Mat&                         image,
    const std::vector<TagDetection>& detections,
    const cv::Mat&                   camera_matrix,
    const cv::Mat&                   dist_coeffs,
    float                            tag_size) {
    for (const auto& det : detections) {
        // Draw corners
        for (size_t i = 0; i < det.corners.size(); i++) {
            cv::line(
                image,
                det.corners[i],
                det.corners[(i + 1) % 4],
                cv::Scalar(0, 255, 0),
                2);
        }

        // Draw ID
        cv::Point2f center(0, 0);
        for (const auto& corner : det.corners) {
            center += corner;
        }
        center.x /= 4.0f;
        center.y /= 4.0f;

        cv::putText(
            image,
            std::to_string(det.id),
            center,
            cv::FONT_HERSHEY_SIMPLEX,
            1.0,
            cv::Scalar(0, 0, 255),
            2);

        // Draw axes
        Eigen::Matrix3f rotation    = det.pose.block<3, 3>(0, 0);
        Eigen::Vector3f translation = det.pose.block<3, 1>(0, 3);

        cv::Mat rvec(3, 1, CV_64F);
        cv::Mat R_cv(3, 3, CV_64F);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R_cv.at<double>(i, j) = rotation(i, j);
            }
        }
        cv::Rodrigues(R_cv, rvec);

        cv::Mat tvec(3, 1, CV_64F);
        for (int i = 0; i < 3; i++) {
            tvec.at<double>(i, 0) = translation(i);
        }

        cv::drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, tag_size);
    }
}

}  // namespace apriltag_pose_estimator

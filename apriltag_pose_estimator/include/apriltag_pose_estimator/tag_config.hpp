#ifndef APRILTAG_POSE_ESTIMATOR__TAG_CONFIG_HPP_
#define APRILTAG_POSE_ESTIMATOR__TAG_CONFIG_HPP_

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

namespace apriltag_pose_estimator {

struct TagDetection {
    int                      id;       // Tag ID
    Eigen::Matrix4f          pose;     // 4x4 transformation matrix (tag to camera)
    std::vector<cv::Point2f> corners;  // Corner points in image coordinates
};

}  // namespace apriltag_pose_estimator

#endif  // APRILTAG_POSE_ESTIMATOR__TAG_CONFIG_HPP_

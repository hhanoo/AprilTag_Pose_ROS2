#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

#include "apriltag_pose_estimator/multi_tag_pose_estimator.hpp"
#include "apriltag_pose_estimator/tag_config.hpp"

namespace apriltag_pose_estimator {

// ========================================================================
// Test Fixture
//
// 실제 카메라/ROS 없이 estimator만 단위 테스트하기 위한 공통 준비물.
// 가상 카메라와 2-마커 그룹(base=0, marker1은 base 기준 +x 0.2m)을 정의하고,
// 알려진 pose에서 3D 코너를 이미지로 투영해 "완벽한 검출 결과"를 합성한다.
// 각 TEST_F는 이 클래스를 상속받아 아래 멤버/헬퍼를 그대로 사용한다.
// ========================================================================
class MultiTagPoseEstimatorTest : public ::testing::Test {
   protected:
    // 각 테스트 시작 직전에 매번 호출됨(gtest 규약).
    // estimator를 새로 만들어 내부 상태(재사용 카운터, SLERP 필터)가
    // 테스트 간에 섞이지 않도록 한다.
    void SetUp() override {
        // 가상 카메라 내부 파라미터 (640x480 해상도 가정, 렌즈 왜곡 없음)
        camera_matrix_                  = cv::Mat::eye(3, 3, CV_64F);
        camera_matrix_.at<double>(0, 0) = 600.0;  // fx
        camera_matrix_.at<double>(1, 1) = 600.0;  // fy
        camera_matrix_.at<double>(0, 2) = 320.0;  // cx
        camera_matrix_.at<double>(1, 2) = 240.0;  // cy
        dist_coeffs_                    = cv::Mat::zeros(5, 1, CV_64F);

        // 그룹 구성: 마커당 6개 오프셋(x, y, z, roll, pitch, yaw)
        // marker0(base): identity, marker1: base 기준 +x 0.2m
        marker_ids_     = {0, 1};
        marker_offsets_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                           0.2f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

        estimator_ = std::make_unique<MultiTagPoseEstimator>(
            marker_ids_, marker_offsets_, tag_size_, camera_matrix_, dist_coeffs_);
    }

    // 주어진 base pose(rvec/tvec)에서 각 마커의 3D 코너를 이미지에 투영해
    // 검출 결과를 합성한다. marker_x_offsets가 마커의 "실제" 위치이므로,
    // 설정값(0.2m)과 다르게 주면 재투영 오차가 큰 outlier 프레임이 된다.
    std::vector<TagDetection> makeDetections(
        const cv::Mat& rvec, const cv::Mat& tvec,
        const std::vector<float>& marker_x_offsets) {
        const float a = tag_size_ / 2.0f;
        // multi_tag_pose_estimator.cpp의 local_corners와 동일한 순서
        const std::vector<cv::Point3f> local_corners = {
            {-a, +a, 0.0f}, {+a, +a, 0.0f}, {+a, -a, 0.0f}, {-a, -a, 0.0f}};

        std::vector<TagDetection> detections;
        for (size_t i = 0; i < marker_ids_.size(); i++) {
            std::vector<cv::Point3f> pts3d;
            for (const auto& c : local_corners) {
                pts3d.emplace_back(c.x + marker_x_offsets[i], c.y, c.z);
            }
            std::vector<cv::Point2f> pts2d;
            cv::projectPoints(pts3d, rvec, tvec, camera_matrix_, dist_coeffs_, pts2d);

            TagDetection det;
            det.id      = marker_ids_[i];
            det.pose    = Eigen::Matrix4f::Identity();
            det.corners = pts2d;
            detections.push_back(det);
        }
        return detections;
    }

    // 설정된 offset(0.2m)과 일치하는 정상 프레임
    std::vector<TagDetection> validDetections(const cv::Mat& rvec, const cv::Mat& tvec) {
        return makeDetections(rvec, tvec, {0.0f, 0.2f});
    }

    // marker1이 설정보다 +0.15m 어긋난 outlier 프레임 (재투영 오차 > 15px 유발)
    std::vector<TagDetection> outlierDetections(const cv::Mat& rvec, const cv::Mat& tvec) {
        return makeDetections(rvec, tvec, {0.0f, 0.35f});
    }

    // estimate() 호출 래퍼: rvec/tvec 출력 인자는 테스트에서 쓰지 않으므로 감춘다
    bool runEstimate(const std::vector<TagDetection>& dets, cv::Mat& img,
                     Eigen::Matrix4f& base_xf) {
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
        cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
        return estimator_->estimate(dets, img, rvec, tvec, base_xf);
    }

    // 새 검정 프레임 (노드의 vis_image 역할)
    cv::Mat blankImage() { return cv::Mat::zeros(480, 640, CV_8UC3); }

    // 이미지에 아무것도 그려지지 않았는지 확인 (모든 픽셀이 0인지 검사)
    static bool imageUntouched(const cv::Mat& img) {
        cv::Scalar s = cv::sum(img);
        return (s[0] + s[1] + s[2] + s[3]) == 0.0;
    }

    std::vector<int>                       marker_ids_;
    std::vector<float>                     marker_offsets_;
    float                                  tag_size_ = 0.1f;
    cv::Mat                                camera_matrix_;
    cv::Mat                                dist_coeffs_;
    std::unique_ptr<MultiTagPoseEstimator> estimator_;
};

// ========================================================================
// (A) Outlier pose 재사용은 연속 max_pose_reuse_frames(5)회로 제한되어야 한다
// ========================================================================
TEST_F(MultiTagPoseEstimatorTest, OutlierPoseReuseIsBounded) {
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.5);

    Eigen::Matrix4f base_xf;
    cv::Mat         img = blankImage();

    // 정상 프레임 → 유효 pose 저장
    ASSERT_TRUE(runEstimate(validDetections(rvec, tvec), img, base_xf));

    // 연속 outlier 5프레임까지는 이전 pose 재사용(성공 유지)
    for (int k = 0; k < 5; k++) {
        img = blankImage();
        EXPECT_TRUE(runEstimate(outlierDetections(rvec, tvec), img, base_xf))
            << "reuse frame " << k + 1 << " should still succeed";
    }

    // 6번째 연속 outlier부터는 실패로 보고해야 함 (무기한 재사용 금지)
    img = blankImage();
    EXPECT_FALSE(runEstimate(outlierDetections(rvec, tvec), img, base_xf))
        << "stale pose must not be reused beyond the reuse limit";

    // 이후 정상 프레임이 오면 다시 추정 가능해야 함
    img = blankImage();
    EXPECT_TRUE(runEstimate(validDetections(rvec, tvec), img, base_xf));
}

// ========================================================================
// (B) 재사용된(실측이 아닌) pose로는 축을 그리지 않아야 한다
// ========================================================================
TEST_F(MultiTagPoseEstimatorTest, NoAxesDrawnForReusedPose) {
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.5);

    Eigen::Matrix4f base_xf;

    // 정상 프레임: 축이 그려져야 함 (sanity check)
    cv::Mat img_valid = blankImage();
    ASSERT_TRUE(runEstimate(validDetections(rvec, tvec), img_valid, base_xf));
    EXPECT_FALSE(imageUntouched(img_valid)) << "axes should be drawn for a measured pose";

    // outlier 프레임(재사용 pose): 이미지에 아무것도 그리지 않아야 함
    cv::Mat img_reuse = blankImage();
    ASSERT_TRUE(runEstimate(outlierDetections(rvec, tvec), img_reuse, base_xf));
    EXPECT_TRUE(imageUntouched(img_reuse))
        << "stale (reused) pose must not be visualized on the latest image";
}

// ========================================================================
// (C) 검출 공백(연속 미검출) 후에는 필터가 리셋되어 옛 pose와 블렌딩되지 않아야 한다
// ========================================================================
TEST_F(MultiTagPoseEstimatorTest, FilterResetsAfterDetectionGap) {
    cv::Mat rvec  = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat tvec1 = (cv::Mat_<double>(3, 1) << 0.0, 0.0, 0.5);
    cv::Mat tvec2 = (cv::Mat_<double>(3, 1) << 0.3, 0.0, 0.5);  // +x 0.3m 이동

    Eigen::Matrix4f base_xf;
    cv::Mat         img = blankImage();

    // P1에서 정상 추정 → 필터 상태가 P1에 초기화됨
    ASSERT_TRUE(runEstimate(validDetections(rvec, tvec1), img, base_xf));

    // 5프레임 연속 미검출 (marker1 없음 → estimate 실패)
    for (int k = 0; k < 5; k++) {
        auto only_base = validDetections(rvec, tvec1);
        only_base.resize(1);  // marker0만 남김
        img = blankImage();
        ASSERT_FALSE(runEstimate(only_base, img, base_xf));
    }

    // 재등장한 P2 프레임: 옛 P1 상태와 블렌딩되지 않고 P2를 그대로 출력해야 함
    img = blankImage();
    ASSERT_TRUE(runEstimate(validDetections(rvec, tvec2), img, base_xf));
    EXPECT_NEAR(base_xf(0, 3), 0.3f, 1e-3f)
        << "pose after detection gap must not be blended with the stale filter state";
}

}  // namespace apriltag_pose_estimator

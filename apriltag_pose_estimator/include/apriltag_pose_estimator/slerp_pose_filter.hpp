#ifndef APRILTAG_POSE_ESTIMATOR__SLERP_POSE_FILTER_HPP_
#define APRILTAG_POSE_ESTIMATOR__SLERP_POSE_FILTER_HPP_

// Quaternion SLERP 기반 적응형 EMA Pose 필터
//
// 목적: solvePnP 결과의 프레임 간 지터(노이즈)를 제거합니다.
// 원리: 회전을 Quaternion으로 변환한 후 SLERP(Spherical Linear Interpolation)으로
//       시간적 평균화(EMA)를 적용합니다.
// 특징:
//   - Euler 각도 대신 Quaternion 사용 → 짐벌락/래핑 문제 방지
//   - 적응형 alpha → 정지 시 강한 스무딩, 이동 시 빠른 추종
//   - 회전과 병진을 독립적으로 필터링

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace apriltag_pose_estimator {
// 필터 설정 구조체
// 각 파라미터의 의미와 조정 방법이 주석으로 설명되어 있습니다.
struct PoseFilterConfig {
    float minAlpha               = 0.05f;  // 정지 시 최소 alpha (최대 스무딩)
    float maxAlpha               = 0.8f;   // 이동 시 최대 alpha (최소 스무딩, 빠른 추종)
    float stationaryThresholdDeg = 0.3f;   // rotation 변화 < 0.3° → 정지로 간주
    float movingThresholdDeg     = 2.0f;   // rotation 변화 > 2.0° → 이동으로 간주
    float stationaryThresholdMm  = 0.5f;   // translation 변화 < 0.5mm → 정지로 간주
    float movingThresholdMm      = 5.0f;   // translation 변화 > 5.0mm → 이동으로 간주
};

// Quaternion SLERP EMA 포즈 필터 클래스
class PoseFilter {
   public:
    explicit PoseFilter(const PoseFilterConfig& config = PoseFilterConfig{});

    // 메인 인터페이스: raw 4x4 pose → 필터링된 4x4 pose 반환
    Eigen::Matrix4f filter(const Eigen::Matrix4f& rawPose);

    // 필터 상태 초기화 (검출 실패 후 재시작 시 사용)
    void reset();

    // 필터가 초기화되었는지 확인
    bool isInitialized() const { return initialized_; }

   private:
    // Quaternion 정규화 (w >= 0 보장하여 sign-flip 방지)
    Eigen::Quaternionf normalizeQuaternion(const Eigen::Quaternionf& q) const;

    // 적응형 alpha 계산 (변화량에 따라 minAlpha ~ maxAlpha 보간)
    float computeAdaptiveAlpha(float changeMagnitude, float stationaryThresh, float movingThresh) const;

    PoseFilterConfig   config_;
    bool               initialized_ = false;
    Eigen::Quaternionf filteredRotation_;
    Eigen::Vector3f    filteredTranslation_;
};

}  // namespace apriltag_pose_estimator

#endif  // APRILTAG_POSE_ESTIMATOR__SLERP_POSE_FILTER_HPP_

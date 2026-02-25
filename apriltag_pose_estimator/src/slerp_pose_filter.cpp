#include "apriltag_pose_estimator/slerp_pose_filter.hpp"

#include <algorithm>
#include <cmath>

namespace apriltag_pose_estimator {

PoseFilter::PoseFilter(const PoseFilterConfig& config) : config_(config) {}

void PoseFilter::reset() { initialized_ = false; }

Eigen::Quaternionf PoseFilter::normalizeQuaternion(const Eigen::Quaternionf& q) const {
    auto n = q.normalized();
    // w < 0인 경우 부호 반전 → canonical form 유지 (sign-flip 방지)
    if (n.w() < 0) n.coeffs() = -n.coeffs();
    return n;
}

float PoseFilter::computeAdaptiveAlpha(float change, float stThresh, float mvThresh) const {
    if (change <= stThresh) return config_.minAlpha;
    if (change >= mvThresh) return config_.maxAlpha;
    float t = (change - stThresh) / (mvThresh - stThresh);
    return config_.minAlpha + t * (config_.maxAlpha - config_.minAlpha);
}

Eigen::Matrix4f PoseFilter::filter(const Eigen::Matrix4f& rawPose) {
    Eigen::Quaternionf rawQ(rawPose.block<3, 3>(0, 0));
    Eigen::Vector3f    rawT = rawPose.block<3, 1>(0, 3);
    rawQ                    = normalizeQuaternion(rawQ);

    // 첫 프레임: 초기화 후 그대로 반환
    if (!initialized_) {
        filteredRotation_    = rawQ;
        filteredTranslation_ = rawT;
        initialized_         = true;
        Eigen::Matrix4f r    = Eigen::Matrix4f::Identity();
        r.block<3, 3>(0, 0) = rawQ.toRotationMatrix();
        r.block<3, 1>(0, 3) = rawT;
        return r;
    }

    // sign-flip 방지: dot product가 음수이면 rawQ를 반전하여 최단 경로 SLERP 보장
    if (filteredRotation_.dot(rawQ) < 0) rawQ.coeffs() = -rawQ.coeffs();

    // 회전 변화량 (degrees) 및 병진 변화량 (mm) 계산
    auto  diffQ    = normalizeQuaternion(filteredRotation_.conjugate() * rawQ);
    float angleDeg = 2.0f * std::acos(std::min(1.0f, std::abs(diffQ.w()))) * 180.0f / M_PI;
    float transMm  = (rawT - filteredTranslation_).norm() * 1000.0f;

    // 적응형 alpha 계산
    float rotAlpha   = computeAdaptiveAlpha(angleDeg, config_.stationaryThresholdDeg,
                                             config_.movingThresholdDeg);
    float transAlpha = computeAdaptiveAlpha(transMm, config_.stationaryThresholdMm,
                                             config_.movingThresholdMm);

    // SLERP EMA (회전) + LERP EMA (병진)
    filteredRotation_    = normalizeQuaternion(filteredRotation_.slerp(rotAlpha, rawQ));
    filteredTranslation_ = (1.0f - transAlpha) * filteredTranslation_ + transAlpha * rawT;

    Eigen::Matrix4f result    = Eigen::Matrix4f::Identity();
    result.block<3, 3>(0, 0) = filteredRotation_.toRotationMatrix();
    result.block<3, 1>(0, 3) = filteredTranslation_;
    return result;
}

}  // namespace apriltag_pose_estimator

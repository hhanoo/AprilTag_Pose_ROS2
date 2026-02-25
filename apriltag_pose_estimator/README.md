# AprilTag Pose Estimator (C++)

AprilTag 기반 다중 타겟 포즈 추정 C++ 패키지입니다.

## 개요

`apriltag_pose_estimator`는 AprilTag C 라이브러리를 사용하여 카메라 이미지에서 AprilTag를 검출하고, 다중 마커 기반으로 정확한 타겟 포인트의 포즈를 계산합니다. Python 버전(`apriltag_pose_estimator_py`)보다 높은 성능과 낮은 지연시간을 제공합니다.

## 주요 기능

- **고성능 AprilTag 검출**: C 라이브러리 기반으로 빠른 처리 속도
- **다중 마커 포즈 추정**: 여러 마커의 위치 관계를 이용한 정확한 포즈 계산
- **타겟 포인트 계산**: 베이스 마커 기준 다중 타겟 포인트의 3D 위치 추정
- **시각화 지원**: RViz용 visualization markers 및 detection 이미지 퍼블리싱
- **실시간 처리**: 30fps 이상의 실시간 포즈 추정

## ROS2 API

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `pose_estimator_node/target_poses` | `geometry_msgs/PoseArray` | 계산된 타겟 포인트들의 포즈 배열 |
| `visualization_markers` | `visualization_msgs/MarkerArray` | RViz 시각화용 마커 배열 |
| `pose_estimator_node/detection_image` | `sensor_msgs/Image` | AprilTag 검출 결과가 그려진 이미지 |

**Note**: 토픽 이름은 노드의 네임스페이스에 따라 전체 경로가 결정됩니다. (예: `/vision/pose_estimator_node/target_poses`)

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `realsense_node/color/image_raw` | `sensor_msgs/Image` | 입력 카메라 이미지 (BGR8 인코딩) |
| `realsense_node/color/camera_info` | `sensor_msgs/CameraInfo` | 카메라 내부 파라미터 (intrinsics, distortion) |

### Parameters

#### AprilTag Configuration
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `marker_ids` | list[int] | `[0, 1, 2]` | 사용할 AprilTag 마커 ID 목록 |
| `base_marker_id` | int | `-1` | 베이스 마커 ID (-1: 첫 번째 marker_id 사용) |
| `tag_size` | double | `0.02778` | AprilTag 크기 (m, 27.78mm) |
| `tag_family` | string | `"tagStandard41h12"` | AprilTag family (tag36h11, tagStandard41h12 등) |
| `marker_offsets` | list[double] | `[0.065, 0.0]` | 마커 간 오프셋 [X, Y] (m) |
| `point_offsets` | list[double] | `[2개 포인트]` | 타겟 포인트 오프셋 (Nx6 flat list, m, radian) |

#### Topics
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `camera_topic` | string | `"realsense_node/color/image_raw"` | 카메라 이미지 토픽 |
| `camera_info_topic` | string | `"realsense_node/color/camera_info"` | 카메라 정보 토픽 |
| `camera_frame` | string | `"camera_color_optical_frame"` | 카메라 좌표계 frame ID |
| `pose_topic` | string | `"pose_estimator_node/target_poses"` | 포즈 출력 토픽 |
| `detection_image_topic` | string | `"pose_estimator_node/detection_image"` | 검출 이미지 출력 토픽 |
| `marker_topic` | string | `"visualization_markers"` | RViz 마커 토픽 |

#### Publishing Options
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `publish_visualization` | bool | `true` | RViz 마커 퍼블리시 여부 |
| `publish_detection_image` | bool | `true` | 검출 이미지 퍼블리시 여부 |

## 사용 방법

> ⚠️ **선행 조건**: `apriltag_estimator.launch.py`는 **포즈 추정 노드만** 실행합니다.
> 카메라 드라이버를 **먼저** 별도 터미널에서 실행해야 합니다.
>
> ```bash
> # 터미널 1: RealSense 카메라 먼저 실행
> ros2 launch realsense2_camera rs_launch.py
> ```
>
> **토픽 불일치 주의**: `realsense2_camera`의 기본 토픽은 `camera/camera/color/image_raw`이지만,
> `pose_estimator.yaml`의 기본값은 `realsense_node/color/image_raw`입니다. 그대로 실행하면 연결되지 않습니다.
>
> **방법 A (권장): launch argument로 토픽 오버라이드**
> ```bash
> # 터미널 2: realsense2_camera 기본 토픽에 맞춰 실행
> ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py \
>   camera_topic:=camera/camera/color/image_raw \
>   camera_info_topic:=camera/camera/color/camera_info
> ```
>
> **방법 B: ros2 run으로 토픽 리매핑**
> ```bash
> # 터미널 2: 토픽 리매핑으로 실행
> ros2 run apriltag_pose_estimator pose_estimator_node --ros-args \
>   -r realsense_node/color/image_raw:=camera/camera/color/image_raw \
>   -r realsense_node/color/camera_info:=camera/camera/color/camera_info
> ```
>
> **방법 C: YAML 직접 수정**
> `config/pose_estimator.yaml`의 `camera_topic`, `camera_info_topic`을 실제 드라이버 토픽으로 변경한 뒤 실행:
> ```bash
> # 터미널 2:
> ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py
> ```

### 1. 단독 실행

```bash
# 기본 설정으로 실행
ros2 run apriltag_pose_estimator pose_estimator_node

# 파라미터와 함께 실행
ros2 run apriltag_pose_estimator pose_estimator_node --ros-args \
  -p tag_size:=0.05 \
  -p tag_family:=tag36h11
```

### 2. Launch 파일 사용

```bash
# 기본 설정
ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py

# 자주 바꾸는 파라미터 오버라이드
ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py \
  tag_size:=0.05 \
  tag_family:=tag36h11 \
  show_detection:=false

# 커스텀 YAML 파일 사용
ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py \
  config_file:=/path/to/custom_config.yaml
```

**Launch Arguments:**
- `config_file`: YAML 설정 파일 경로 (default: 패키지 내부 `pose_estimator.yaml`)
- `camera_topic`: 카메라 이미지 토픽 (default: `color/image_raw`)
- `camera_info_topic`: 카메라 정보 토픽 (default: `color/camera_info`)
- `tag_family`: AprilTag family (default: `tagStandard41h12`)
- `tag_size`: AprilTag 크기 (default: `0.02778`)
- `show_detection`: rqt_image_view로 검출 이미지 표시 (default: `true`)

**Note**: 대부분의 파라미터는 YAML 파일에서 관리되며, launch argument로 자주 변경하는 파라미터만 오버라이드 가능합니다.

### 3. 네임스페이스와 함께 사용

```bash
# 네임스페이스 지정 (예: vision)
ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py

# 결과 토픽:
# - /vision/pose_estimator_node/target_poses
# - /vision/pose_estimator_node/detection_image
# - /vision/visualization_markers
```

## 의존성

### ROS2 패키지
- `rclcpp`: ROS2 C++ 클라이언트 라이브러리
- `sensor_msgs`: 이미지 및 카메라 정보 메시지 타입
- `geometry_msgs`: 포즈 메시지 타입
- `visualization_msgs`: RViz 시각화 마커 메시지 타입
- `cv_bridge`: OpenCV-ROS2 이미지 변환 브릿지

### 시스템 라이브러리
- `libapriltag-dev`: AprilTag C 라이브러리 (태그 검출)
- `libopencv-dev`: OpenCV (이미지 처리 및 PnP)
- `libeigen3-dev`: Eigen3 (행렬 연산 및 변환)

### 설치

```bash
# 모든 의존성 설치
sudo apt-get install -y \
  libapriltag-dev \
  libopencv-dev \
  libeigen3-dev
```

## 알고리즘

### Multi-Tag Pose Estimation

이 패키지는 다중 마커 기반 포즈 추정을 수행합니다:

1. **AprilTag 검출**: 카메라 이미지에서 모든 AprilTag 검출 및 개별 포즈 추정
2. **3D-2D 대응**: 각 마커의 3D 월드 좌표와 2D 이미지 좌표 매칭
3. **PnP 문제 해결**: `cv::solvePnP`를 사용하여 베이스 마커의 정확한 카메라 포즈 추정
4. **타겟 포인트 계산**: 베이스 마커 기준으로 각 타겟 포인트의 3D 위치 계산

```
Multiple AprilTags → Combined 3D-2D correspondences → Single PnP solution → Target points
```

이 방식은 단일 마커보다 훨씬 정확하고 안정적인 포즈 추정을 제공합니다:
- **정확도**: 단일 마커 대비 2-3배 향상
- **안정성**: 부분 오클루전에 강인함
- **정밀도**: Sub-millimeter 정밀도 달성 가능

### 지원하는 AprilTag Families

- `tagStandard41h12`: 41비트, Hamming distance 12 (기본값, 권장)
- `tag36h11`: 36비트, Hamming distance 11

**Note**: 추가 family 지원이 필요한 경우 `april_tag_detector.hpp`/`.cpp`에서 헤더 추가 및 생성 함수 추가 필요

## 성능

**Python 버전 대비 성능:**
- **처리 속도**: 약 3-5배 빠름
- **지연시간**: 약 50-70% 감소
- **CPU 사용률**: 약 60-70% 감소

**벤치마크** (1280x720, 3 tags, Intel i7-10750H):
- Python (`apriltag_pose_estimator_py`): ~15-25ms/frame
- C++ (`apriltag_pose_estimator`): ~5-8ms/frame

## 아키텍처

```
Camera Image
     │
     ▼
┌─────────────────────┐
│ AprilTagDetector    │
│ - Detect tags       │
│ - Estimate poses    │
└──────────┬──────────┘
           │ TagDetections
           ▼
┌─────────────────────┐
│ MultiTagPose        │
│ Estimator           │
│ - Combine markers   │
│ - PnP solve         │
│ - Calculate points  │
└──────────┬──────────┘
           │ Transforms
           ▼
┌─────────────────────┐
│ PoseEstimatorNode   │
│ - Publish poses     │
│ - Visualization     │
│ - Detection image   │
└──────────┬──────────┘
           │
           ▼
    ROS2 Topics
    /target_poses
    /visualization_markers
```

## 설정 가이드

### 마커 설정

[`config/pose_estimator.yaml`](config/pose_estimator.yaml) 파일을 수정하여 설정을 변경할 수 있습니다:

```yaml
pose_estimator_node:
  ros__parameters:
    # AprilTag Configuration
    marker_ids: [0, 1, 2]                # 사용할 마커 ID 목록
    base_marker_id: -1                   # 베이스 마커 (-1: 첫 번째 marker_id 사용)
    tag_size: 0.02778                    # 태그 크기 (m)
    tag_family: 'tagStandard41h12'       # AprilTag family
    marker_offsets: [0.065, 0.0]         # [X, Y] 마커 간 오프셋 (m)
    
    # Target Point Offsets (Nx3 flat list)
    point_offsets: [
      0.000, 0.000, 0.000,    # Point 1
      0.000, 0.000, 0.000,    # Point 2
    ]
    
    # Input Topics
    camera_topic: 'realsense_node/color/image_raw'
    camera_info_topic: 'realsense_node/color/camera_info'
    camera_frame: 'camera_color_optical_frame'
    
    # Output Topics
    pose_topic: 'pose_estimator_node/target_poses'
    detection_image_topic: 'pose_estimator_node/detection_image'
    marker_topic: 'visualization_markers'
    
    # Publishing Options
    publish_visualization: true
    publish_detection_image: true
```

## 트러블슈팅

### 카메라 데이터를 수신하지 못하는 경우

**증상**: 노드가 실행되지만 아무 로그도 출력되지 않거나 `Waiting for camera data...` 상태 지속

**원인**: 구독 토픽과 카메라 드라이버 퍼블리시 토픽이 불일치

**확인 방법**:
```bash
# 실제 퍼블리시 중인 토픽 확인
ros2 topic list | grep color

# 노드가 구독 중인 토픽 확인
ros2 node info /pose_estimator_node
```

**해결 방법**: 구독 토픽을 실제 드라이버 토픽에 맞춰 실행 (`realsense2_camera` 기본값 예시)
```bash
ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py \
  camera_topic:=camera/camera/color/image_raw \
  camera_info_topic:=camera/camera/color/camera_info
```

---

### AprilTag가 검출되지 않는 경우

**증상**: `No AprilTags detected` 로그 메시지

**해결 방법**:

1. **카메라 토픽 확인**:
   ```bash
   ros2 topic list | grep image
   ros2 topic hz /color/image_raw
   ```

2. **조명 조건 개선**: 밝고 균일한 조명 사용

3. **Tag 크기 확인**: `tag_size` 파라미터가 실제 태그 크기와 일치하는지 확인

4. **Tag family 확인**: 사용하는 태그와 `tag_family` 파라미터가 일치하는지 확인

5. **카메라 초점**: 이미지가 선명한지 확인

### 포즈 추정이 실패하는 경우

**증상**: `Pose estimation failed - not all required markers detected`

**해결 방법**:

1. **모든 마커 가시성 확인**: `marker_ids`에 지정된 모든 마커가 보이는지 확인
   ```bash
   ros2 topic echo /detection_image
   ```

2. **마커 ID 확인**: 실제 태그의 ID와 `marker_ids` 파라미터가 일치하는지 확인

3. **마커 배치**: 마커들이 `marker_offsets` 설정대로 배치되어 있는지 확인

### 포즈가 불안정한 경우

**증상**: 포즈가 지터링(떨림)하거나 튀는 현상

**해결 방법**:

1. **카메라 캘리브레이션**: 카메라 내부 파라미터가 정확한지 확인

2. **태그 크기**: `tag_size` 값이 정확한지 확인 (mm → m 변환 주의)

3. **마커 오프셋**: `marker_offsets` 값이 실제 거리와 일치하는지 확인

4. **이미지 품질**: 해상도를 높이거나 프레임 레이트를 낮춤

## Python 버전과의 비교

| 특징 | Python (`apriltag_pose_estimator_py`) | C++ (`apriltag_pose_estimator`) |
|------|---------------------------------------|----------------------------------|
| 성능 | ⭐⭐ | ⭐⭐⭐⭐⭐ |
| 개발 속도 | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |
| 메모리 사용 | 높음 | 낮음 |
| 실시간성 | 30fps | 60fps+ |
| 의존성 설치 | 쉬움 (`pip install`) | 중간 (시스템 라이브러리) |
| 권장 용도 | 프로토타이핑, 개발 | 배포, 실시간 시스템 |

## 관련 패키지

- [`realsense2_camera`](https://github.com/IntelRealSense/realsense-ros): RealSense 공식 ROS2 드라이버

## 라이선스

MIT

## 작성자

- **Maintainer**: hhanoo <woo980711@gmail.com>

# Apriltag Pose ROS2

**AprilTag 기반 실시간 6-DOF 포즈 추정 시스템**

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-blue)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C++-17-blue)](https://isocpp.org/)
[![License](https://img.shields.io/badge/License-MIT-orange)](LICENSE)
[![Docker](https://img.shields.io/badge/Docker-Supported-brightgreen)](docker/)

## 📋 목차

- [데모](#데모)
- [개요](#개요)
- [주요 기능](#주요-기능)
- [빠른 시작](#빠른-시작)
- [시스템 요구사항](#시스템-요구사항)
- [설치](#설치)
- [빌드](#빌드)
- [실행](#실행)
- [사용법](#사용법)
- [설정](#설정)
- [ROS2 인터페이스](#ros2-인터페이스)
- [문제 해결](#문제-해결)
- [자주 묻는 질문](#자주-묻는-질문)
- [라이선스](#라이선스)

---

## 데모

### 시스템 구조

```
┌──────────────────────────────────────────────────────────────────┐
│                    Apriltag Pose ROS2 System                     │
└──────────────────────────────────────────────────────────────────┘

    Intel RealSense D435/D455
              │
              │ USB 3.0
              ▼
    ┌──────────────────────┐
    │ realsense2_camera    │ (C++)
    │ - Camera streaming   │
    │ - Intrinsics pub     │
    └──────────┬───────────┘
               │ /color/image_raw
               │ /color/camera_info
               ▼
    ┌──────────────────────┐
    │apriltag_pose_        │ (C++)
    │estimator             │
    │ - Tag detection      │
    │ - Multi-tag fusion   │
    │ - 6-DOF pose calc    │
    └──────────┬───────────┘
               │ /tag_detections
               │ /target_poses
               ▼
    ┌──────────────────────┐
    │apriltag_pose_        │ (Python)
    │visualizer            │
    │ - Overlay drawing    │
    │ - Roll/Pitch/Yaw     │
    └──────────────────────┘
```

### 프로젝트 구조

```
Apriltag_Pose_ROS2/
├── realsense-ros/                 # RealSense ROS2 driver (realsense2_camera)
│   └── realsense2_camera/
│       └── launch/
│           └── rs_launch.py       # Camera launch file
│
├── apriltag_pose_estimator/       # Pose estimator (C++)
│   ├── src/
│   │   ├── pose_estimator_node.cpp
│   │   ├── april_tag_detector.cpp
│   │   └── multi_tag_pose_estimator.cpp
│   ├── launch/
│   │   └── apriltag_estimator.launch.py
│   └── config/
│       └── pose_estimator.yaml
│
├── apriltag_pose_estimator_msgs/  # Custom interfaces
│   ├── msg/
│   │   └── TagDetection.msg       # AprilTag pose data
│   └── srv/
│       └── TargetPointPose.srv    # Pose query service
│
├── apriltag_pose_visualizer/      # Visualization (Python)
│   └── apriltag_pose_visualizer/
│       └── pose_visualizer_node.py
│
└── docker/                        # Docker support
    ├── Dockerfile
    ├── build.sh
    └── run.sh
```

---

## 개요

### 이게 뭔가요?

Apriltag Pose ROS2는 Intel RealSense 카메라와 AprilTag를 활용한 실시간 6자유도(6-DOF) 포즈 추정 시스템입니다. 다중 마커 융합 알고리즘을 통해 단일 마커 대비 2-3배 향상된 정확도로 타겟 객체의 위치와 자세를 추정합니다.

### 주요 구성요소

- **realsense2_camera** (C++): Intel RealSense D435/D455 공식 ROS2 드라이버 (`realsense-ros`)
- **apriltag_pose_estimator** (C++): 고성능 AprilTag 검출 및 다중 마커 포즈 융합
- **apriltag_pose_visualizer** (Python): 실시간 검출 결과 시각화 및 Roll/Pitch/Yaw 계산
- **apriltag_pose_estimator_msgs**: ROS2 커스텀 메시지 및 서비스 인터페이스

### 사용 사례

- 로봇 비전 시스템의 객체 위치 추적
- 자율 네비게이션을 위한 공간 인식
- AR/VR 마커 기반 위치 추정
- 산업 자동화 시스템의 정밀 포지셔닝
- 연구 개발 및 교육

---

## 주요 기능

- 🎯 **다중 마커 융합** - PnP 기반 multi-tag 포즈 추정으로 단일 마커 대비 2-3배 정확도 향상
- ⚡ **고성능 C++ 구현** - Python 대비 3-5배 빠른 처리 속도 (5-8ms/frame @ 1280x720)
- 📷 **Intel RealSense 통합** - D435/D455 카메라 네이티브 지원 및 자동 intrinsic 캘리브레이션
- 🔧 **YAML 기반 설정** - 마커 ID, 크기, 오프셋을 YAML로 유연하게 구성
- 📊 **실시간 시각화** - RViz 마커 및 OpenCV 기반 detection image 오버레이
- 🐳 **Docker 지원** - 빌드 및 실행 스크립트 포함, 원클릭 컨테이너 환경
- 🔌 **서비스 인터페이스** - 온디맨드 포즈 쿼리를 위한 ROS2 서비스 제공
- 🎨 **혼합 언어 아키텍처** - Python(카메라/시각화) + C++(검출/추정)의 최적 조합

---

## 빠른 시작

### Option 1: Docker (권장)

```bash
# 1. 저장소 클론
git clone <repository-url> ~/Apriltag_Pose_ROS2
cd ~/Apriltag_Pose_ROS2

# 2. Docker 이미지 빌드
cd docker
./build.sh

# 3. Docker 컨테이너 실행
./run.sh

# 4. 컨테이너 내에서 빌드 및 실행
cd /ros2_ws
colcon build
source install/setup.bash

# 터미널 1: RealSense 카메라 실행
ros2 launch realsense2_camera rs_launch.py

# 터미널 2 (새 터미널): 포즈 추정기 실행
ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py
```

### Option 2: Native Installation

```bash
# 1. 저장소 클론 및 의존성 설치
git clone <repository-url> ~/vision_ws/src/Apriltag_Pose_ROS2
cd ~/vision_ws
rosdep install --from-paths src --ignore-src -r -y

# 2. 빌드
colcon build
source install/setup.bash

# 3. 실행
ros2 launch realsense2_camera rs_launch.py
ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py
```

---

## 시스템 요구사항

### 필수

- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **Python**: 3.10+
- **C++**: C++17 compiler (GCC 9+, Clang 10+)
- **CMake**: 3.16+

### 하드웨어

- **Camera**: Intel RealSense D435 or D455 (USB 3.0 required)
- **CPU**: Multi-core processor (Intel i5+ or equivalent recommended)
- **RAM**: 4GB minimum, 8GB recommended
- **USB**: USB 3.0 port for RealSense camera

### 소프트웨어 의존성

**System Libraries:**

- libapriltag-dev (AprilTag C library)
- libopencv-dev (OpenCV 4.x)
- libeigen3-dev (Eigen3)
- ros-humble-realsense2-\* (RealSense SDK)

**Python Packages:**

- pyrealsense2==2.56.5.9235
- dt-apriltags==3.1.7
- opencv-python==4.10.0.84
- numpy<2.0 (for cv_bridge compatibility)

---

## 설치

### Method 1: Docker (권장)

Docker를 사용하면 모든 의존성이 자동으로 설치됩니다:

```bash
cd ~/Apriltag_Pose_ROS2/docker
./build.sh
```

빌드 완료 후 `./run.sh`로 컨테이너를 실행하면 모든 환경이 준비됩니다.

### Method 2: Native Installation

#### 1. ROS2 Humble 설치

공식 설치 가이드를 따르세요:

- [ROS2 Humble (Ubuntu 22.04)](https://docs.ros.org/en/humble/Installation.html)

#### 2. 시스템 의존성 설치

```bash
# 기본 빌드 도구
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep

# Vision 라이브러리
sudo apt install -y \
  libapriltag-dev \
  libopencv-dev \
  libeigen3-dev \
  ros-humble-realsense2-*
```

#### 3. Python 패키지 설치

```bash
# ROS2 호환 setuptools 및 packaging
pip3 install \
  setuptools==58.2.0 \
  packaging==21.3 \
  wheel

# Vision 관련 패키지 (NumPy는 반드시 <2.0)
pip3 install \
  'numpy>=1.21.0,<2.0' \
  pyrealsense2==2.56.5.9235 \
  dt-apriltags==3.1.7 \
  opencv-python==4.10.0.84
```

#### 4. 워크스페이스 생성 및 저장소 클론

```bash
# 워크스페이스 생성
mkdir -p ~/vision_ws/src
cd ~/vision_ws/src

# 저장소 클론
git clone <repository-url> Apriltag_Pose_ROS2
```

#### 5. ROS 의존성 설치

```bash
cd ~/vision_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## 빌드

### 전체 워크스페이스 빌드

```bash
cd ~/vision_ws
colcon build
source install/setup.bash
```

### 특정 패키지만 빌드

```bash
colcon build --packages-select apriltag_pose_estimator
source install/setup.bash
```

### 빌드 타입

```bash
# 릴리스 최적화 빌드 (성능 중요 시)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 디버그 빌드
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# 자세한 출력과 함께
colcon build --event-handlers console_direct+
```

### 클린 빌드

```bash
# 빌드 결과물 삭제
cd ~/vision_ws
rm -rf build install log

# 처음부터 재빌드
colcon build
source install/setup.bash
```

**Expected build output:**

```
Starting >>> apriltag_pose_estimator_msgs
Starting >>> realsense2_camera
Finished <<< apriltag_pose_estimator_msgs [5.2s]
Starting >>> apriltag_pose_estimator
Starting >>> apriltag_pose_visualizer
Finished <<< realsense2_camera [8.1s]
Finished <<< apriltag_pose_visualizer [8.3s]
Finished <<< apriltag_pose_estimator [12.5s]

Summary: 4 packages finished
```

---

## 실행

### Complete System Launch

> ⚠️ **실행 순서 필수 준수**: `apriltag_estimator.launch.py`는 카메라를 포함하지 않습니다.
> **반드시 터미널 1(카메라)을 먼저 실행**한 후 터미널 2(추정기)를 실행하세요.

```bash
# 터미널 1: RealSense 카메라 실행 (먼저 실행)
source ~/vision_ws/install/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=1280x720x30 \
  enable_depth:=false

# 터미널 2: AprilTag 포즈 추정기 실행 (카메라 실행 후)
source ~/vision_ws/install/setup.bash
ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py \
  camera_topic:=camera/camera/color/image_raw \
  camera_info_topic:=camera/camera/color/camera_info

# 터미널 3 (선택): 시각화 노드 실행
source ~/vision_ws/install/setup.bash
ros2 run apriltag_pose_visualizer pose_visualizer_node
```

- Launch system ROS2 Graph
  ![Launch_ROS_Graph](docs/launch_ros_graph.png)

### Launch with Custom Configuration

```bash
# 카메라 해상도 변경
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=640x480x60

# AprilTag 크기 및 family 변경
ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py \
  tag_size:=0.05 \
  tag_family:=tag36h11

# 커스텀 YAML 설정 파일 사용
ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py \
  config_file:=/path/to/custom_config.yaml
```

### Docker Execution

```bash
# Docker 컨테이너 실행
cd ~/Apriltag_Pose_ROS2/docker
./run.sh

# 컨테이너 내부에서
colcon build && source install/setup.bash
ros2 launch realsense2_camera rs_launch.py
```

---

## 사용법

### Basic Workflow

1. **카메라 연결 확인**

   ```bash
   # RealSense 디바이스 확인
   rs-enumerate-devices
   ```

2. **시스템 실행**

   ```bash
   # 카메라 스트리밍 시작
   ros2 launch realsense2_camera rs_launch.py

   # 포즈 추정 시작
   ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py
   ```

3. **토픽 모니터링**

   ```bash
   # 활성 토픽 확인
   ros2 topic list

   # 검출된 포즈 확인
   ros2 topic echo /pose_estimator_node/target_poses

   # 카메라 이미지 확인
   ros2 run rqt_image_view rqt_image_view
   ```

4. **서비스 호출**
   ```bash
   # 현재 포즈 쿼리
   ros2 service call /pose_estimator_node/target_point_pose \
     apriltag_pose_estimator_msgs/srv/TargetPointPose \
     "{request_time: {sec: 0, nanosec: 0}}"
   ```

### Visualization in RViz

```bash
# RViz2 실행
rviz2

# Add → By topic → /visualization_markers → MarkerArray
# Fixed Frame: camera_color_optical_frame
```

---

## 설정

### Camera Configuration

`realsense2_camera`는 launch 인수로 직접 파라미터를 전달합니다:

```bash
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=1280x720x30 \
  enable_depth:=false \
  rgb_camera.enable_auto_exposure:=true
```

### Pose Estimator Configuration

[apriltag_pose_estimator/config/pose_estimator.yaml](apriltag_pose_estimator/config/pose_estimator.yaml) 편집:

```yaml
pose_estimator_node:
  ros__parameters:
    # AprilTag Detection
    marker_ids: [0, 1, 2] # Tag IDs to detect
    base_marker_id: 1 # Reference marker (-1: use first)
    tag_size: 0.02778 # Tag size in meters (27.78mm)
    tag_family: "tagStandard41h12" # tag36h11, tagStandard41h12
    marker_offsets: [0.065, 0.0] # [X, Y] offset between tags (m)

    # Target Points (position + rotation)
    # Format: [x, y, z, roll, pitch, yaw, ...]
    target_points: [
        0.000,
        0.000,
        0.000,
        0.000,
        0.000,
        0.0000, # Point 0
        0.000,
        0.000,
        0.000,
        0.000,
        0.000,
        1.5708, # Point 1 (90° yaw)
      ]

    # Input Topics
    camera_topic: "camera/camera/color/image_raw"
    camera_info_topic: "camera/camera/color/camera_info"

    # Publishing Options
    publish_visualization: true # RViz markers
    publish_detection_image: true # Annotated image
```

### 주요 파라미터 설명

- **tag_size**: 실제 AprilTag의 검은색 사각형 크기를 미터 단위로 측정 (정확도에 매우 중요!)
- **marker_offsets**: 마커 중심 간의 실제 물리적 거리 [X, Y] (미터)
- **target_points**: 베이스 마커를 기준으로 한 타겟 포인트의 상대적 위치 및 회전

---

## ROS2 인터페이스

### Nodes

| Node Name              | Type   | Package                  | Description                  |
| ---------------------- | ------ | ------------------------ | ---------------------------- |
| `realsense_node`       | C++    | realsense2_camera        | RealSense 카메라 스트리밍    |
| `pose_estimator_node`  | C++    | apriltag_pose_estimator  | AprilTag 검출 및 포즈 추정   |
| `pose_visualizer_node` | Python | apriltag_pose_visualizer | 검출 결과 시각화 및 RPY 계산 |

### Topics

#### Published Topics

| Topic                                  | Type                                        | Publisher           | Description                            |
| -------------------------------------- | ------------------------------------------- | ------------------- | -------------------------------------- |
| `/color/image_raw`                     | `sensor_msgs/Image`                         | realsense_node      | RGB 카메라 이미지 (BGR8)               |
| `/color/camera_info`                   | `sensor_msgs/CameraInfo`                    | realsense_node      | 카메라 내부 파라미터                   |
| `/pose_estimator_node/tag_detections`  | `apriltag_pose_estimator_msgs/TagDetection` | pose_estimator_node | AprilTag 검출 데이터 (visualization용) |
| `/pose_estimator_node/target_poses`    | `geometry_msgs/PoseArray`                   | pose_estimator_node | 타겟 포인트 포즈 배열                  |
| `/visualization_markers`               | `visualization_msgs/MarkerArray`            | pose_estimator_node | RViz 시각화 마커                       |
| `/pose_estimator_node/detection_image` | `sensor_msgs/Image`                         | pose_estimator_node | 검출 결과 오버레이 이미지              |

#### Subscribed Topics

| Topic                              | Type                     | Subscriber          | Description              |
| ---------------------------------- | ------------------------ | ------------------- | ------------------------ |
| `/camera/camera/color/image_raw`   | `sensor_msgs/Image`      | pose_estimator_node | 입력 카메라 이미지       |
| `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | pose_estimator_node | 카메라 캘리브레이션 정보 |

### Services

| Service                                  | Type                                               | Provider            | Description               |
| ---------------------------------------- | -------------------------------------------------- | ------------------- | ------------------------- |
| `/pose_estimator_node/target_point_pose` | `apriltag_pose_estimator_msgs/srv/TargetPointPose` | pose_estimator_node | 최신 검출 포즈 쿼리       |
| `/realsense_node/get_device_info`        | `std_srvs/srv/Trigger`                             | realsense_node      | 카메라 디바이스 정보 조회 |

### Parameters

| Parameter               | Type      | Default              | Description                  |
| ----------------------- | --------- | -------------------- | ---------------------------- |
| `marker_ids`            | list[int] | `[0, 1, 2]`          | 사용할 AprilTag 마커 ID 목록 |
| `base_marker_id`        | int       | `1`                  | 베이스 마커 ID               |
| `tag_size`              | double    | `0.02778`            | AprilTag 크기 (m)            |
| `tag_family`            | string    | `"tagStandard41h12"` | AprilTag family              |
| `frame_rate`            | int       | `30`                 | 카메라 프레임 레이트 (Hz)    |
| `publish_visualization` | bool      | `true`               | RViz 마커 퍼블리시 여부      |

### Custom Messages

**TagDetection.msg:**

```msg
bool       tag_detected    # Tag detected flag
float64[9] camera_matrix   # 3x3 intrinsic matrix (row-major)
float64[]  dist_coeffs     # OpenCV distortion coefficients
float64[3] rvec            # Rodrigues rotation vector
float64[3] tvec            # Translation vector (meters)
float64    tag_size        # Tag size (meters)
```

**TargetPointPose.srv:**

```srv
# Request
builtin_interfaces/Time request_time
---
# Response
bool success                        # Success flag
string message                      # Status message
builtin_interfaces/Time data_time   # Data timestamp
geometry_msgs/PoseArray poses       # Target poses
```

---

## 문제 해결

### 1. Camera Issues

#### 1-1. Problem: "No RealSense devices detected"

**Solution:**

```bash
# Check USB connection
lsusb | grep Intel

# Check RealSense kernel module
sudo modprobe uvcvideo

# Test camera directly
realsense-viewer

# Check permissions
sudo usermod -aG video $USER
sudo usermod -aG plugdev $USER
# Log out and log back in
```

#### 1-2. Problem: "Failed to set USB power"

**Solution:**

- USB 3.0 포트 사용 (파란색 포트)
- 카메라 재연결
- 다른 USB 케이블 시도 (반드시 USB 3.0 케이블)

### 2. AprilTag Detection Issues

#### 2-1. Problem: "No AprilTags detected"

**Solution:**

1. **조명 확인**: 밝고 균일한 조명 필요 (그림자 최소화)
2. **초점 확인**: 카메라 초점이 태그에 맞춰져야 함
3. **태그 크기 확인**:
   ```yaml
   # config/pose_estimator.yaml에서 실제 크기와 일치하는지 확인
   tag_size: 0.02778 # 27.78mm in meters
   ```
4. **태그 family 확인**: 실제 인쇄한 태그와 설정이 일치해야 함
5. **거리**: 카메라에서 태그까지 0.3m ~ 3m 권장
6. **각도**: 태그가 카메라를 향하도록 (각도 < 45°)

#### 2-2. Problem: "Pose estimation failed - not all markers detected"

**Solution:**

- `marker_ids: [0, 1, 2]`에 지정된 모든 태그가 카메라에 보여야 함
- 부분 가림(occlusion) 최소화
- marker_ids를 단일 마커로 줄여서 테스트: `marker_ids: [1]`

#### 2-3. Problem: "Pose가 불안정하거나 지터링(떨림)"

**Solution:**

1. **카메라 캘리브레이션**: 카메라 내부 파라미터가 정확한지 확인
2. **태그 크기**: `tag_size` 값이 정확한지 확인 (mm → m 변환 주의)
3. **마커 오프셋**: `marker_offsets` 값이 실제 거리와 일치하는지 확인
4. **이미지 품질**: 해상도를 높이거나 프레임 레이트를 낮춤

### 3. Build Issues

#### 3-1. Problem: "Could not find libapriltag-dev"

```bash
sudo apt update
sudo apt install -y libapriltag-dev

# If still not found:
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install -y libapriltag-dev
```

#### 3-2. Problem: "cv_bridge NumPy version conflict"

```bash
# NumPy must be <2.0 for cv_bridge compatibility
pip3 install 'numpy>=1.21.0,<2.0'
```

---

## 자주 묻는 질문

---

## 라이선스

이 프로젝트는 MIT 라이선스로 배포됩니다 - 자세한 내용은 [LICENSE](LICENSE) 파일을 참조하세요.

---

**Maintainer**: hhanoo (woo980711@gmail.com)

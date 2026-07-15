# AprilTag Pose ROS2

**AprilTag 기반 실시간 6-DOF 포즈 추정 시스템**

[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![C++](https://img.shields.io/badge/C++-17-00599C?logo=cplusplus&logoColor=white)](https://isocpp.org/)
[![Python](https://img.shields.io/badge/Python-3.10+-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?logo=opencv&logoColor=white)](https://opencv.org/)
[![Docker](https://img.shields.io/badge/Docker-Supported-2496ED?logo=docker&logoColor=white)](docker/)
[![License](https://img.shields.io/badge/License-MIT-orange?logo=opensourceinitiative&logoColor=white)](LICENSE)

---

## 목차

- [데모](#데모)
- [개요](#개요)
- [주요 기능](#주요-기능)
- [시스템 구조](#시스템-구조)
- [프로젝트 구조](#프로젝트-구조)
- [빠른 시작](#빠른-시작)
- [시스템 요구사항](#시스템-요구사항)
- [설치](#설치)
- [빌드](#빌드)
- [실행](#실행)
- [사용법](#사용법)
- [설정](#설정)
- [API / ROS 인터페이스](#api--ros-인터페이스)
- [문제 해결](#문제-해결)
- [라이선스](#라이선스)
- [Maintainer](#maintainer)

---

## 데모

<details>
<summary>ROS2 Graph</summary>

![Launch_ROS_Graph](docs/launch_ros_graph.png)

</details>

---

## 개요

### 프로젝트 목적

AprilTag Pose ROS2는 ROS2 호환 RGB 카메라와 AprilTag 마커를 활용하여 타겟 객체의 6자유도(6-DOF) 위치와 자세를 실시간으로 추정하는 시스템입니다. 다중 마커 융합 알고리즘(Multi-tag PnP)을 적용하여 단일 마커 대비 높은 정확도를 달성하며, SLERP 기반 쿼터니언 필터로 안정적인 포즈 출력을 제공합니다.

### 주요 구성요소

- **apriltag_pose_estimator** (C++): AprilTag 검출, 다중 마커 포즈 융합, SLERP 필터링
- **apriltag_pose_visualizer** (Python): 실시간 검출 결과 시각화, Roll/Pitch/Yaw 계산, 3D 산점도
- **apriltag_pose_estimator_msgs** (C++): ROS2 커스텀 메시지 및 서비스 인터페이스

> 카메라 드라이버는 본 패키지에 포함되어 있지 않습니다. `sensor_msgs/Image` + `CameraInfo`를 퍼블리시하는 ROS2 카메라 드라이버를 사용 환경에 맞게 별도로 실행하세요.

### 적용 가능 영역

- 로봇 비전 시스템의 객체 위치 추적
- 자율 네비게이션을 위한 공간 인식
- 산업 자동화 시스템의 정밀 포지셔닝
- AR/VR 마커 기반 위치 추정
- 연구 개발 및 교육

---

## 주요 기능

- **다중 마커 PnP 융합**: 복수의 AprilTag 코너를 동시에 활용하여 단일 마커 대비 향상된 6-DOF 포즈 추정
- **SLERP 쿼터니언 필터**: Gimbal lock 없는 회전 보간과 적응형 EMA로 프레임 간 지터 억제 (연속 5프레임 미검출 시 자동 리셋되어 재등장 시 옛 pose와 블렌딩 방지)
- **Reprojection Error 기반 이상치 제거**: 15픽셀 임계값으로 불량 검출 자동 필터링 (이전 유효 pose 재사용은 연속 5프레임으로 제한, 재사용 pose는 축 시각화 제외)
- **고성능 C++ 파이프라인**: 실시간 처리에 적합한 네이티브 구현 (5-8ms/frame @ 1280x720)
- **Multi-Group 동시 추정**: 한 카메라 프레임에서 여러 마커 그룹의 base pose를 동시에 publish (그룹별 독립 SLERP 필터)
- **YAML 기반 유연한 설정**: 마커 ID, 크기, 그룹별 오프셋을 설정 파일로 관리
- **실시간 3D 시각화**: 위치(tvec) 및 회전(RPY) 히스토리의 3D 산점도와 통계 오버레이
- **서비스 인터페이스**: 온디맨드 포즈 쿼리를 위한 ROS2 서비스 제공
- **Docker 지원**: 빌드 및 실행 스크립트 포함, 원클릭 컨테이너 환경

---

## 시스템 구조

```
┌──────────────────────────────────────────────────────────────────────┐
│                      AprilTag Pose ROS2 System                       │
└──────────────────────────────────────────────────────────────────────┘

        ROS2 RGB Camera
              │
              │ USB 3.0
              ▼
    ┌───────────────────────┐
    │  Camera Driver Node   │
    │  - Camera streaming   │
    │  - Intrinsics publish │
    └───────────┬───────────┘
                │  sensor_msgs/Image
                │  sensor_msgs/CameraInfo
                │
                ├─────────────────────────────────────┐
                │                                     │ sensor_msgs/Image
                ▼                                     ▼
    ┌───────────────────────┐                ┌───────────────────────┐
    │ apriltag_pose_        │  TagDetection  │ apriltag_pose_        │
    │ estimator             │───────────────▶│ visualizer            │  (Python)
    │  - Tag detection      │                │  - Overlay drawing    │
    │  - Multi-tag PnP      │                │  - RPY calculation    │
    │  - SLERP filtering    │                │  - 3D scatter plots   │
    │  - Outlier rejection  │                └───────────────────────┘
    └───────────┬───────────┘
                │  /target_poses        (PoseArray, group_names order)
                │  /group_names         (latched CSV)
                │  /group_status        (per-group valid bits)
                │  /detection_image     (BGR8 overlay)
                │  /tag_detections      (raw, debug — visualizer 입력)
                ▼
         Service Client
    (target_point_pose query, group_name dispatch)
```

---

## 프로젝트 구조

```
AprilTag_Pose_ROS2/
├── apriltag_pose_estimator/                    # 포즈 추정 패키지 (C++)
│   ├── src/
│   │   ├── pose_estimator_node.cpp             # ROS2 노드 구현
│   │   ├── april_tag_detector.cpp              # AprilTag 검출 래퍼
│   │   ├── multi_tag_pose_estimator.cpp        # 다중 마커 PnP 융합
│   │   ├── slerp_pose_filter.cpp               # 쿼터니언 SLERP 필터
│   │   └── main.cpp                            # 엔트리 포인트
│   ├── include/apriltag_pose_estimator/
│   │   ├── pose_estimator_node.hpp
│   │   ├── april_tag_detector.hpp
│   │   ├── multi_tag_pose_estimator.hpp
│   │   ├── slerp_pose_filter.hpp
│   │   └── tag_config.hpp                      # 데이터 구조 정의
│   ├── config/
│   │   └── pose_estimator.yaml                 # 설정 파일
│   ├── launch/
│   │   └── apriltag_estimator.launch.py        # 런치 파일
│   ├── scripts/
│   │   └── jitter_probe.py                     # 서비스 응답 지터 측정 스크립트
│   ├── test/
│   │   └── test_multi_tag_pose_estimator.cpp   # estimator 단위 테스트 (gtest)
│   ├── CMakeLists.txt
│   └── package.xml
├── apriltag_pose_estimator_msgs/               # 커스텀 인터페이스
│   ├── msg/
│   │   └── TagDetection.msg
│   ├── srv/
│   │   └── TargetPointPose.srv
│   ├── CMakeLists.txt
│   └── package.xml
├── apriltag_pose_visualizer/                   # 시각화 패키지 (Python)
│   ├── apriltag_pose_visualizer/
│   │   └── pose_visualizer_node.py
│   ├── setup.py
│   └── package.xml
├── realsense-ros/                              # RealSense ROS2 드라이버 (외부 저장소, git 미추적)
├── docker/                                     # Docker 환경
│   ├── Dockerfile
│   ├── build.sh
│   ├── run.sh
│   ├── entrypoint.sh
│   ├── commands.sh
│   └── config.sh.example
└── docs/
    └── launch_ros_graph.png
```

---

## 빠른 시작

### Option 1: Docker (권장)

```bash
# 1. Docker 이미지 가져오기
docker pull hhanoo/project:apriltag-pose-ros2-humble

# 2. 컨테이너 실행 (X11 포워딩 포함)
cd AprilTag_Pose_ROS2/docker
./run.sh

# 3. 컨테이너 내부에서 빌드 (RealSense 드라이버는 워크스페이스에 포함되어 있음)
cd /ros2_ws
colcon build

# 4. 실행 (터미널 2개)
ros2 launch <your_camera_driver> ...   # 사용 카메라 ROS2 드라이버 (RealSense는 미리 정의된 run-camera 사용)
run-estimator                          # ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py
```

### Option 2: Native

```bash
# 1. 의존성 설치 및 빌드 (사용 카메라 ROS2 드라이버는 사전 설치 필요)
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash

# 2. 실행 (터미널 2개)
ros2 launch <your_camera_driver> ...
ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py
```

---

## 시스템 요구사항

### 필수

| 항목   | 요구사항                  |
| ------ | ------------------------- |
| OS     | Ubuntu 22.04 LTS          |
| ROS2   | Humble                    |
| Python | 3.10+                     |
| C++    | C++17 (GCC 9+, Clang 10+) |

### 하드웨어

| 항목   | 사양                                                               |
| ------ | ------------------------------------------------------------------ |
| Camera | ROS2 호환 RGB 카메라 (`sensor_msgs/Image` + `CameraInfo` 퍼블리시) |
| USB    | USB 3.0 포트 (필수)                                                |
| CPU    | 멀티코어 프로세서 (Intel i5+ 권장)                                 |
| RAM    | 4GB 이상 (8GB 권장)                                                |

### 소프트웨어 의존성

**시스템 라이브러리:**

- `libapriltag-dev` - AprilTag C 라이브러리
- `libopencv-dev` - OpenCV 4.x
- `libeigen3-dev` - Eigen3 선형대수 라이브러리
- 사용 카메라에 맞는 ROS2 드라이버 (별도 설치)

**Python 패키지:**

- `numpy` (<2.0, cv_bridge 호환)
- `dt-apriltags` (==3.1.7)
- `opencv-python` (==4.10.0.84)
- `matplotlib` (3D 시각화)

---

## 설치

### Method 1: Docker (권장)

Docker를 사용하면 모든 의존성이 자동으로 설치됩니다.

```bash
# 1. Docker 이미지 가져오기
docker pull hhanoo/project:apriltag-pose-ros2-humble

# 2. 컨테이너 실행
cd AprilTag_Pose_ROS2/docker
./run.sh
```

<details>
<summary>직접 빌드 (개발자용)</summary>

```bash
# 0. 프로젝트 루트로 이동
cd AprilTag_Pose_ROS2/docker

# 1. 설정 파일 생성 후 IMAGE_NAME을 로컬 이름으로 변경
cp config.sh.example config.sh
# config.sh에서 IMAGE_NAME="apriltag-pose-ros2-humble" 로 수정

# 2. Docker 이미지 빌드
./build.sh

# 3. 컨테이너 실행
./run.sh
```

</details>

### Method 2: Native

#### 1. ROS2 Humble 설치

[ROS2 Humble 공식 설치 가이드](https://docs.ros.org/en/humble/Installation.html)를 참고하세요.

#### 2. 시스템 의존성 설치

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  python3-pip python3-colcon-common-extensions python3-rosdep \
  libapriltag-dev libopencv-dev libeigen3-dev
```

> 사용 카메라에 맞는 ROS2 드라이버는 별도로 설치하세요.

#### 3. Python 패키지 설치

```bash
pip3 install \
  setuptools==58.2.0 packaging==21.3 wheel \
  'numpy>=1.21.0,<2.0' \
  dt-apriltags==3.1.7 \
  opencv-python==4.10.0.84
```

#### 4. ROS 의존성 설치

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## 빌드

### 전체 빌드

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 특정 패키지 빌드

```bash
colcon build --symlink-install --packages-select apriltag_pose_estimator
source install/setup.bash
```

### 릴리스 빌드

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 클린 빌드

```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

---

## 실행

실행 순서를 반드시 준수하세요. `apriltag_estimator.launch.py`에는 카메라가 포함되어 있지 않습니다.

### 전체 시스템 실행

```bash
# 터미널 1: 카메라 ROS2 드라이버 실행 (먼저)
source ~/ros2_ws/install/setup.bash
ros2 launch <your_camera_driver> ...

# 터미널 2: AprilTag 포즈 추정기 실행
source ~/ros2_ws/install/setup.bash
ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py

# 터미널 3 (선택): 시각화 노드 실행
source ~/ros2_ws/install/setup.bash
ros2 run apriltag_pose_visualizer pose_visualizer_node
```

### 개별 노드 실행

```bash
# 커스텀 카메라 토픽 지정
ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py \
  camera_topic:=camera/camera/color/image_raw \
  camera_info_topic:=camera/camera/color/camera_info

# 태그 파라미터 변경
ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py \
  tag_size:=0.05 \
  tag_family:=tag36h11

# 커스텀 설정 파일 사용
ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py \
  config_file:=/path/to/custom_config.yaml

# GUI 윈도우 비활성화 (Docker / headless 환경)
ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py \
  show_service_result_window:=false
```

### Docker 실행

> **권장**: 직접 `docker exec`로 컨테이너에 진입하지 말고 항상 [run.sh](docker/run.sh)를 사용하세요.  
> `run.sh`는 도커 이미지 확인 · X11 권한 · 마운트 · 호스트 권한 복원(`HOST_UID`/`HOST_GID`) · 기존 컨테이너 재사용을 한 번에 처리합니다.

```bash
docker pull hhanoo/project:apriltag-pose-ros2-humble
cd docker
./run.sh

# 컨테이너 내부
cd /ros2_ws
colcon build && source install/setup.bash
ros2 launch <your_camera_driver> ...   # 사용 카메라 ROS2 드라이버 (RealSense는 run-camera 사용)
```

전체 command 정의는 [commands.sh](docker/commands.sh)를 참고하세요.

| Command          | 설명                     | 참고                                                                                                 |
| ---------------- | ------------------------ | ---------------------------------------------------------------------------------------------------- |
| `build`          | 워크스페이스 빌드        | colcon build --symlink-install + overlay 자동 source                                                 |
| `run-tests`      | 단위 테스트 실행         | colcon test (apriltag 패키지) + 결과 요약                                                            |
| `run-camera`     | RealSense 카메라 실행    | [rs_launch.py](realsense-ros/realsense2_camera/launch/rs_launch.py) (1280x720x30, depth 비활성)      |
| `run-estimator`  | 포즈 추정기 실행         | [apriltag_estimator.launch.py](apriltag_pose_estimator/launch/apriltag_estimator.launch.py)          |
| `run-visualizer` | 시각화 노드 실행         | [pose_visualizer_node.py](apriltag_pose_visualizer/apriltag_pose_visualizer/pose_visualizer_node.py) |
| `echo-pose`      | 포즈 토픽 모니터링       | --                                                                                                   |
| `call-pose`      | 포즈 서비스 호출         | [TargetPointPose.srv](apriltag_pose_estimator_msgs/srv/TargetPointPose.srv)                          |
| `cmd-help`       | 사용 가능한 command 목록 | 컨테이너 접속 시 자동 출력                                                                           |

> `run-camera`는 워크스페이스에 포함된 realsense-ros 기준입니다. 다른 카메라를 쓸 경우 해당 ROS2 드라이버를 직접 실행하세요  
> (estimator 구독 토픽: `camera/color/image_raw`, `camera/color/camera_info`).

---

## 사용법

### 워크플로우

```
카메라 연결 확인 ──────▶ 시스템 실행 ───────▶ 토픽 모니터링 ──────▶ 서비스 호출
       │                  │                  │                  │
  카메라 드라이버        launch x2          topic echo        service call
    진단 도구          (cam + est)       /target_poses    /target_point_pose
```

#### 1. 카메라 연결 확인

사용 카메라 ROS2 드라이버의 진단 도구로 연결 상태를 확인하세요.

#### 2. 시스템 실행

```bash
# 1) 사용 카메라 ROS2 드라이버 실행
ros2 launch <your_camera_driver> ...

# 2) 포즈 추정기 실행
ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py
```

#### 3. 토픽 모니터링

```bash
# 활성 토픽 확인
ros2 topic list

# 검출된 포즈 확인
ros2 topic echo /pose_estimator_node/target_poses

# 이미지 확인
ros2 run rqt_image_view rqt_image_view
```

#### 4. 서비스 호출

```bash
ros2 service call /pose_estimator_node/target_point_pose \
  apriltag_pose_estimator_msgs/srv/TargetPointPose \
  "{request_time: {sec: 0, nanosec: 0}, group_name: ''}"
```

#### 5. 서비스 응답 지터 측정 (선택)

`scripts/jitter_probe.py`는 `target_point_pose` 서비스를 N회 반복 호출하여 응답 포즈의 흔들림(지터)을 위치 (mm) / 회전 (deg) 통계로 보여주는 진단 스크립트입니다. 노드가 실행 중일 때 별도 터미널에서 실행하세요. 빌드 불필요(순수 Python).

**기본 실행**

```bash
# 100회 호출, 0.1초 간격, default 그룹 추적
python3 ros2_ws/src/apriltag_pose_estimator/scripts/jitter_probe.py
```

실행 시작 시 자동으로 timestamp 기반 로그/CSV 파일을 생성하고 경로를 출력합니다 (예: `jitter_probe_20260424_175126.log` / `.csv`). CSV에는 매 호출의 raw 값(`idx, ok, x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg, message`)이 기록되어 외부 분석/그래프 도구로 활용할 수 있습니다.

**옵션**

| 옵션             | 기본값                                   | 설명                                           |
| ---------------- | ---------------------------------------- | ---------------------------------------------- |
| `-n, --count`    | 100                                      | 서비스 호출 횟수                               |
| `-i, --interval` | 0.1                                      | 호출 간격 (초)                                 |
| `--service`      | `/pose_estimator_node/target_point_pose` | 서비스 이름                                    |
| `--group`        | `""`                                     | 조회할 그룹 이름 (빈 문자열 = 첫/default 그룹) |
| `--log <path>`   | 자동 timestamp                           | 로그 파일 경로 (원할 경우 직접 지정 가능)      |
| `--csv <path>`   | 로그와 동일 위치 `.csv`                  | CSV 파일 경로 (원할 경우 직접 지정 가능)       |
| `--no-log`       | off                                      | 파일 저장을 끄고 콘솔만 사용                   |

**사용 예시**

```bash
# 200회 호출, 50ms 간격
python3 .../jitter_probe.py -n 200 -i 0.05

# 특정 그룹 추적 (multi-group yaml 환경)
python3 .../jitter_probe.py --group secondary

# 로그/CSV 저장 경로 직접 지정
python3 .../jitter_probe.py --log /tmp/run1.log --csv /tmp/run1.csv

# 파일 저장 끄고 콘솔 출력만
python3 .../jitter_probe.py --no-log
```

**출력 예시**

```
position (mm):
  x      mean=  -14.5532 mm  std=  0.2340  min=  -14.9100  max=  -14.1200  p2p=  0.7900
  y      mean=  -47.2991 mm  std=  ...
  z      mean= +270.3540 mm  std=  ...

orientation (deg):
  roll   mean=  ...
  pitch  mean=  ...
  yaw    mean=  ...
```

- `mean`: 평균 (정적 위치 / 반복 가능성 검증)
- `std`: 1-σ 표준편차 (낮을수록 안정)
- `p2p`: peak-to-peak (max−min, 실제 떨림 폭)

---

## 설정

### Docker 설정

[config.sh](docker/config.sh.example)

```bash
IMAGE_NAME="hhanoo/project:apriltag-pose-ros2-humble"  # Docker Hub 이미지 (기본값)
CONTAINER_NAME="apriltag-pose-ros2-humble"             # Docker 컨테이너 이름
ROS_DOMAIN_ID=                                         # ROS2 도메인 ID
XAUTHORITY_PATH="$HOME/.Xauthority"                    # X11 .Xauthority 경로 (GUI/RViz 표시용)
```

> `run.sh` 실행 전 `docker pull hhanoo/project:apriltag-pose-ros2-humble`로 이미지를 가져오세요.
>
> 직접 빌드하려면 `IMAGE_NAME`을 `"apriltag-pose-ros2-humble"` 등으로 변경 후 `./build.sh`를 실행하세요.

### pose_estimator.yaml (Multi-Group, 권장)

[apriltag_pose_estimator/config/pose_estimator.yaml](apriltag_pose_estimator/config/pose_estimator.yaml)

신규 사용자는 multi-group 형식을 기본으로 사용합니다. 단일 그룹 운영도 group_names에 그룹 하나만 두면 동일한 인터페이스로 동작합니다.

<!-- prettier-ignore -->
```yaml
pose_estimator_node:
  ros__parameters:
    # 공통 AprilTag 설정
    tag_size: 0.02778
    tag_family: 'tagStandard41h12'

    # 입력 / 출력 토픽 (yaml에 적으면 그 값, 없으면 ~/<topic> 자동 사용)
    camera_topic: 'camera/camera/color/image_raw'
    camera_info_topic: 'camera/camera/color/camera_info'
    camera_frame: 'camera_color_optical_frame'
    use_distortion_from_camera_info: false

    target_poses_topic:    'pose_estimator_node/target_poses'
    group_names_topic:     'pose_estimator_node/group_names'      # latched
    group_status_topic:    'pose_estimator_node/group_status'
    detection_image_topic: 'pose_estimator_node/detection_image'

    publish_detection_image: true

    # ---- Multi-group 정의 ----
    # 각 그룹은 base marker의 pose 1개를 출력. group_names 순서가 PoseArray / group_status 순서.
    group_names:
      - default
      - secondary

    groups:
      default:
        marker_ids: [0, 1, 2]
        base_marker_id: 1
        # 마커마다 6 doubles [x, y, z, roll, pitch, yaw] (m, rad), T_base <- marker_i
        # base_marker entry는 정의상 모두 0 (자동 강제)
        marker_offsets: [
          -0.065,  0.000,  0.000,   0.0000,  0.0000,  0.0000,   # marker 0
           0.000,  0.000,  0.000,   0.0000,  0.0000,  0.0000,   # marker 1 (base)
           0.065,  0.000,  0.000,   0.0000,  0.0000,  0.0000,   # marker 2
        ]

      secondary:
        marker_ids: [10, 11, 12]
        base_marker_id: 11
        marker_offsets: [
          -0.065,  0.000,  0.000,   0.0000,  0.0000,  0.0000,
           0.000,  0.000,  0.000,   0.0000,  0.0000,  0.0000,
           0.065,  0.000,  0.000,   0.0000,  0.0000,  0.0000,
        ]
```

> **v3.0.0 ABI break**
>
> - yaml: `group_names` + `groups.<name>.{...}` 스키마가 필수. 단일 그룹만 사용해도 `group_names: [<your_group>]` + `groups.<your_group>.{...}` 형태로 명시해야 함. 이전 단일 그룹 형식(`marker_ids` / `marker_offsets` / `target_points` 최상위)은 더 이상 지원하지 않음.
> - srv: `TargetPointPose.srv` request에 `string group_name` 필드 추가. 빈 문자열은 group_names의 첫 그룹을 가리킴.
> - 출력: `target_points` 개념 제거. 각 그룹은 base marker pose 1개만 출력하므로 `PoseArray.poses[i]`가 `group_names[i]`와 1:1 매핑.

### 주요 파라미터 설명

- **tag_size**: AprilTag 검은색 사각형의 한 변 길이(m). 정확도에 직접적으로 영향
- **marker_offsets**: 마커별 base_marker 기준 6-DoF 변환 (T_base ← marker_i). `marker_ids` 순서대로 마커마다 6개 값 `[x, y, z, roll, pitch, yaw]`(m, rad)을 평면 리스트로 나열. `base_marker_id`에 해당하는 항목은 정의상 identity (모두 0)여야 하며, 0이 아니면 WARN 후 강제 0 처리됨. 마커 수가 `N`이면 총 `6N`개. 회전 합성은 `R = Rz(yaw) * Ry(pitch) * Rx(roll)` (intrinsic Z-Y-X)
- **use_distortion_from_camera_info**: PnP 시 사용할 왜곡 계수 소스 선택
  - `false` (기본): 왜곡 계수를 0 으로 간주. 이미 rectified 된 이미지를 퍼블리시하는 카메라에 사용
  - `true`: `camera_info.d` 의 값을 그대로 사용. raw 이미지를 퍼블리시하는 카메라에 사용
- **display_width / display_height**: 서비스 결과 윈도우 크기. `0`이면 원본 해상도 그대로 표시

### Launch 인수

| 인수                         | 기본값                | 설명                          |
| ---------------------------- | --------------------- | ----------------------------- |
| `config_file`                | `pose_estimator.yaml` | YAML 설정 파일 경로           |
| `show_service_result_window` | `""` (YAML 값 사용)   | 서비스 결과 윈도우 표시 여부  |
| `camera_topic`               | `""` (YAML 값 사용)   | 카메라 이미지 토픽 오버라이드 |
| `camera_info_topic`          | `""` (YAML 값 사용)   | 카메라 정보 토픽 오버라이드   |
| `tag_family`                 | `""` (YAML 값 사용)   | 태그 패밀리 오버라이드        |
| `tag_size`                   | `""` (YAML 값 사용)   | 태그 크기 오버라이드          |

---

## API / ROS 인터페이스

### 노드

| 노드                   | 언어   | 패키지                   | 설명                         |
| ---------------------- | ------ | ------------------------ | ---------------------------- |
| `pose_estimator_node`  | C++    | apriltag_pose_estimator  | AprilTag 검출 및 포즈 추정   |
| `pose_visualizer_node` | Python | apriltag_pose_visualizer | 검출 결과 시각화 및 RPY 계산 |

> 카메라 드라이버 노드는 본 패키지 외부 의존성으로, 사용 환경에 맞게 별도로 실행해야 합니다.

### Published 토픽

| 토픽                                   | 타입                                        | QoS / 비고                                   | 설명                                                           |
| -------------------------------------- | ------------------------------------------- | -------------------------------------------- | -------------------------------------------------------------- |
| `/pose_estimator_node/tag_detections`  | `apriltag_pose_estimator_msgs/TagDetection` | KEEP_LAST(10)                                | Raw 검출 데이터 (debug. multi-group에선 첫 그룹 rvec/tvec)     |
| `/pose_estimator_node/target_poses`    | `geometry_msgs/PoseArray`                   | KEEP_LAST(10)                                | 각 그룹의 base marker pose (group_names 순서, 실패 그룹은 NaN) |
| `/pose_estimator_node/group_names`     | `std_msgs/String`                           | **transient_local + KEEP_LAST(1)** (latched) | 그룹 이름 CSV. boot 동기화용                                   |
| `/pose_estimator_node/group_status`    | `std_msgs/UInt8MultiArray`                  | KEEP_LAST(10)                                | group_names 순서대로 0(invalid) / 1(valid)                     |
| `/pose_estimator_node/detection_image` | `sensor_msgs/Image` (BGR8)                  | KEEP_LAST(10), subscriber 0이면 publish skip | 모든 그룹 검출 결과를 한 이미지에 누적 오버레이                |

### Subscribed 토픽

| 토픽                        | 타입                     | 서브스크라이버      | 설명                     |
| --------------------------- | ------------------------ | ------------------- | ------------------------ |
| `/camera/color/image_raw`   | `sensor_msgs/Image`      | pose_estimator_node | 입력 카메라 이미지       |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | pose_estimator_node | 카메라 캘리브레이션 정보 |

### 서비스

| 서비스                                   | 타입              | 제공자              | 설명                |
| ---------------------------------------- | ----------------- | ------------------- | ------------------- |
| `/pose_estimator_node/target_point_pose` | `TargetPointPose` | pose_estimator_node | 최신 검출 포즈 쿼리 |

### 커스텀 메시지

**TagDetection.msg:**

```msg
bool       tag_detected    # 태그 검출 플래그
float64[9] camera_matrix   # 3x3 내부 파라미터 행렬 (row-major)
float64[]  dist_coeffs     # 왜곡 계수 (OpenCV 순서)
float64[3] rvec            # Rodrigues 회전 벡터
float64[3] tvec            # 병진 벡터 (m)
float64    tag_size        # 태그 크기 (m)
```

**TargetPointPose.srv:**

```srv
# Request
builtin_interfaces/Time request_time    # 요청 시각
string group_name                       # 조회 그룹 이름 ("" = 첫/default 그룹)
---
# Response
bool success                            # 성공 여부
string message                          # 상태 메시지
builtin_interfaces/Time data_time       # 데이터 타임스탬프
geometry_msgs/PoseArray poses           # 해당 그룹의 base marker pose (요소 1개)
```

> `group_name`이 빈 문자열이면 `group_names`의 첫 그룹을 가리킴. 알 수 없는 그룹 이름은 service error.

### 네트워크 구성 (Docker)

| 항목          | 값               | 설명                  |
| ------------- | ---------------- | --------------------- |
| Network mode  | `host`           | 호스트 네트워크 공유  |
| ROS_DOMAIN_ID | `99`             | ROS2 도메인 ID        |
| IPC           | `host`           | IPC 네임스페이스 공유 |
| X11           | `/tmp/.X11-unix` | GUI 포워딩            |

---

## 문제 해결

### 1. 카메라가 인식되지 않을 때

카메라 드라이버가 디바이스를 찾지 못하면 일반적으로 USB 연결·권한·커널 모듈 문제입니다.

```bash
# USB 연결 확인
lsusb

# 커널 모듈 로드 (UVC 카메라)
sudo modprobe uvcvideo

# 권한 설정
sudo usermod -aG video $USER
sudo usermod -aG plugdev $USER
# 로그아웃 후 재로그인
```

> 카메라 모델별 추가 진단 절차는 사용 ROS2 드라이버 문서를 참고하세요.

### 2. "No AprilTags detected"

```
[WARN] No tags detected in current frame
```

```bash
# 확인 사항:
# - 조명: 밝고 균일한 조명 (그림자 최소화)
# - 초점: 카메라 초점이 태그에 맞춰져 있는지 확인
# - 거리: 카메라-태그 간 0.3m ~ 3m 권장
# - 각도: 태그가 카메라를 향하도록 (< 45도)

# config에서 태그 크기와 패밀리가 실제와 일치하는지 확인
# tag_size: 0.02778  (실측값을 미터 단위로)
# tag_family: 'tagStandard41h12'  (인쇄한 태그와 동일해야 함)
```

### 3. 포즈 불안정/지터링

```
# 포즈 추정값이 크게 흔들리는 경우
```

```bash
# 1. tag_size 값이 실측치와 정확히 일치하는지 확인 (mm -> m 변환 주의)
# 2. marker_offsets 값이 마커 간 실제 거리/회전과 일치하는지 확인 (마커별 6-DoF)
# 3. 해상도를 높이거나 프레임 레이트를 낮춰서 이미지 품질 개선
#    (사용 카메라 ROS2 드라이버 launch에서 해상도/FPS 옵션을 조정)

# 4. scripts/jitter_probe.py 로 정량 측정 (사용법 섹션 5번 참고)
python3 ros2_ws/src/apriltag_pose_estimator/scripts/jitter_probe.py
```

### 4. "Could not find libapriltag-dev"

```
CMake Error: Could not find libapriltag
```

```bash
sudo apt update
sudo apt install -y libapriltag-dev

# 여전히 실패 시
sudo add-apt-repository universe
sudo apt update
sudo apt install -y libapriltag-dev
```

### 5. cv_bridge NumPy 버전 충돌

```
ImportError: numpy.core.multiarray failed to import
```

```bash
pip3 install 'numpy>=1.21.0,<2.0'
```

---

## 라이선스

이 프로젝트는 MIT 라이선스로 배포됩니다. 자세한 내용은 [LICENSE](LICENSE) 파일을 참조하세요.

---

## Maintainer

**hhanoo** (woo980711@gmail.com)

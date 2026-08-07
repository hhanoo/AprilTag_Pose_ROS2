# AprilTag Pose ROS2<!-- omit from toc -->

**AprilTag 마커 그룹을 융합해 6-DOF 포즈를 실시간 추정하는 ROS 2 패키지 모음**

[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![C++](https://img.shields.io/badge/C++-17-00599C?logo=cplusplus&logoColor=white)](https://isocpp.org/)
[![Python](https://img.shields.io/badge/Python-3.10+-3776AB?logo=python&logoColor=white)](https://www.python.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?logo=opencv&logoColor=white)](https://opencv.org/)
[![Docker](https://img.shields.io/badge/Docker-Supported-2496ED?logo=docker&logoColor=white)](docker/)
[![License](https://img.shields.io/badge/License-MIT-orange?logo=opensourceinitiative&logoColor=white)](LICENSE)

---

## 목차<!-- omit from toc -->

- [데모](#데모)
- [개요](#개요)
  - [프로젝트 목적](#프로젝트-목적)
  - [주요 구성요소](#주요-구성요소)
  - [적용 가능 영역](#적용-가능-영역)
- [주요 기능](#주요-기능)
- [시스템 구조](#시스템-구조)
- [프로젝트 구조](#프로젝트-구조)
- [빠른 시작](#빠른-시작)
  - [Option 1: Docker (권장)](#option-1-docker-권장)
  - [Option 2: Native](#option-2-native)
- [시스템 요구사항](#시스템-요구사항)
  - [필수](#필수)
  - [하드웨어](#하드웨어)
  - [소프트웨어 의존성](#소프트웨어-의존성)
  - [외부 패키지](#외부-패키지)
- [설치](#설치)
  - [Method 1: Docker (권장)](#method-1-docker-권장)
  - [Method 2: Native](#method-2-native)
- [빌드](#빌드)
  - [전체 빌드](#전체-빌드)
  - [특정 패키지 빌드](#특정-패키지-빌드)
  - [클린 빌드](#클린-빌드)
  - [단위 테스트](#단위-테스트)
- [실행](#실행)
  - [전체 시스템 실행 (권장)](#전체-시스템-실행-권장)
  - [개별 실행](#개별-실행)
  - [Docker Commands](#docker-commands)
- [사용법](#사용법)
  - [워크플로우](#워크플로우)
  - [1. 카메라 스트림 확인](#1-카메라-스트림-확인)
  - [2. 포즈 추정기 실행](#2-포즈-추정기-실행)
  - [3. 토픽 모니터링](#3-토픽-모니터링)
  - [4. 서비스 호출](#4-서비스-호출)
  - [5. 서비스 응답 지터 측정](#5-서비스-응답-지터-측정)
- [설정](#설정)
  - [Docker 설정](#docker-설정)
  - [포즈 추정기 설정](#포즈-추정기-설정)
  - [Launch 인자](#launch-인자)
- [API / 인터페이스](#api--인터페이스)
- [문제 해결](#문제-해결)
  - [1. 카메라가 인식되지 않을 때](#1-카메라가-인식되지-않을-때)
  - [2. 태그가 검출되지 않을 때](#2-태그가-검출되지-않을-때)
  - [3. 포즈가 흔들릴 때](#3-포즈가-흔들릴-때)
  - [4. libapriltag를 찾지 못할 때](#4-libapriltag를-찾지-못할-때)
  - [5. cv_bridge NumPy 버전 충돌](#5-cv_bridge-numpy-버전-충돌)
  - [6. 시각화 노드에 영상이 뜨지 않을 때](#6-시각화-노드에-영상이-뜨지-않을-때)
  - [7. 서비스 결과 창이 뜨지 않을 때](#7-서비스-결과-창이-뜨지-않을-때)
- [라이선스](#라이선스)
- [Maintainer](#maintainer)

---

## 데모

<details>
<summary>ROS 2 Graph</summary>

![ROS 2 Graph](docs/launch_ros_graph.png)

</details>

---

## 개요

### 프로젝트 목적

ROS 2 호환 RGB 카메라와 AprilTag 마커만으로 대상 물체의 6자유도 위치·자세를 실시간 추정하는 비전 스택. 한 물체에 부착한 여러 마커의 코너를 한 번의 `solvePnP`에 모두 투입해 단일 마커 대비 자세 정확도를 끌어올리며, 재투영 오차 기반 이상치 제거와 쿼터니언 SLERP EMA 필터를 거쳐 프레임 간 지터를 억제한 pose를 내보내는 구조. 마커 묶음을 그룹 단위로 정의하므로 한 카메라 프레임에서 여러 대상의 pose를 동시에 추정할 수 있으며, 각 그룹은 기준 마커(base marker) 한 개의 pose로 표현.

카메라 드라이버는 본 저장소에 포함되지 않음. `sensor_msgs/Image`와 `sensor_msgs/CameraInfo`를 퍼블리시하는 드라이버라면 제조사를 가리지 않고 연동 가능하며, Docker 환경에는 RealSense 실행 command만 미리 정의.

### 주요 구성요소

- **apriltag_pose_estimator** (C++): AprilTag 검출, 그룹별 다중 마커 PnP 융합, 이상치 제거, SLERP 필터링, 토픽/서비스 인터페이스
- **apriltag_pose_estimator_msgs** (IDL): 검출 결과 메시지와 온디맨드 포즈 쿼리 서비스 정의
- **apriltag_pose_visualizer** (Python): 검출 오버레이, RPY 환산, 위치·회전 히스토리 3D 산점도

### 적용 가능 영역

- 로봇 비전 시스템의 대상 물체 위치 추적
- 매니퓰레이터 픽/플레이스의 마커 기반 정렬
- 산업 자동화 설비의 정밀 포지셔닝
- AR/VR 마커 기반 위치 추정
- 연구·교육용 포즈 추정 실습

---

## 주요 기능

**다중 마커 PnP 융합**: 그룹에 속한 모든 마커의 코너를 base marker 좌표계로 변환해 한 번의 `solvePnP`에 투입. 단일 마커보다 관측 코너 수가 늘어 자세 추정이 안정.

**Multi-Group 동시 추정**: `group_names` 순서대로 그룹별 estimator를 두어 한 프레임에서 여러 대상의 base pose를 동시 publish. 그룹마다 독립적인 SLERP 필터와 이상치 상태를 유지.

**재투영 오차 기반 이상치 제거**: 평균 재투영 오차가 15픽셀을 넘으면 불량 프레임으로 보고 직전 유효 pose를 최대 5프레임까지 재사용. 한도를 넘으면 캐시와 필터를 리셋하고 실패 처리하며, 재사용된 pose는 축 시각화에서 제외.

**적응형 SLERP EMA 필터**: 회전은 쿼터니언 SLERP, 병진은 LERP로 지수이동평균. 변화량에 따라 alpha를 0.05~0.8 사이에서 보간해 정지 시 강한 스무딩, 이동 시 빠른 추종을 동시에 확보하며, 짐벌락과 부호 반전 문제에서 자유로운 구조.

**검출 공백 자동 복구**: 연속 5프레임 추정 실패 시 pose 캐시와 필터를 초기화. 마커가 재등장했을 때 옛 pose와 블렌딩되는 현상을 방지.

**YAML 기반 그룹 정의**: 마커 ID, base marker, 마커별 6-DoF 오프셋을 설정 파일 하나로 관리. 코드 수정 없이 대상 형상 변경 가능.

**진단 도구 내장**: `jitter_probe.py`가 서비스를 반복 호출해 위치(mm)·회전(deg) 통계를 콘솔·로그·CSV로 기록.

**Docker 지원**: 빌드/실행 스크립트와 컨테이너 셸 command를 포함해 의존성 설치 없이 바로 사용 가능.

---

## 시스템 구조

```
                        ROS 2 RGB Camera Driver (external)
                                       │
                                       │ sensor_msgs/Image + sensor_msgs/CameraInfo
                                       ▼
  ┌──────────────────────────────────────────────────────────────────────┐
  │ pose_estimator_node (C++)                                            │
  │                                                                      │
  │   ┌─────────────────────────────┐    ┌─────────────────────────────┐ │
  │   │ AprilTagDetector            │    │ MultiTagPoseEstimator       │ │
  │   │  tag36h11                   │───▶│  multi-marker solvePnP      │ │
  │   │  tagStandard41h12           │    │  reprojection rejection     │ │
  │   │  quad_decimate 2.0          │    │  SLERP EMA pose filter      │ │
  │   └─────────────────────────────┘    └─────────────────────────────┘ │
  │                                                                      │
  │             one base marker pose per group (camera frame)            │
  └────────────────────────┬──────────────────────────┬──────────────────┘
                           │ topics                   │ service
                           ▼                          ▼
  ┌─────────────────────────────────────┐  ┌─────────────────────────────┐
  │ pose_visualizer_node (Python)       │  │ Service Client              │
  │  overlay / RPY / 3D scatter         │  │  target_point_pose          │
  └─────────────────────────────────────┘  └─────────────────────────────┘
```

**데이터 흐름**

[검출] Camera Image → AprilTagDetector → group marker filter → MultiTagPoseEstimator  
[추정] corners in base frame → solvePnP → reprojection check → SLERP EMA filter  
[출력] base pose → /target_poses (PoseArray) + /group_status (valid bits)  
[서비스] Client (group_name) → /target_point_pose → latest base pose (1 pose)  
[시각화] /tag_detections → pose_visualizer_node → OpenCV overlay + 3D scatter

---

## 프로젝트 구조

```
AprilTag_Pose_ROS2/
├── apriltag_pose_estimator/                       # 포즈 추정 패키지 (C++)
│   ├── include/apriltag_pose_estimator/
│   │   ├── pose_estimator_node.hpp                # 노드 선언 / 그룹 상태
│   │   ├── april_tag_detector.hpp                 # 검출기 래퍼 선언
│   │   ├── multi_tag_pose_estimator.hpp           # 다중 마커 PnP 선언
│   │   ├── slerp_pose_filter.hpp                  # SLERP EMA 필터 선언
│   │   └── tag_config.hpp                         # TagDetection 구조체
│   ├── src/
│   │   ├── main.cpp                               # 엔트리 포인트
│   │   ├── pose_estimator_node.cpp                # ROS 2 노드 구현
│   │   ├── april_tag_detector.cpp                 # AprilTag 검출 래퍼
│   │   ├── multi_tag_pose_estimator.cpp           # 다중 마커 PnP 융합
│   │   └── slerp_pose_filter.cpp                  # 쿼터니언 SLERP 필터
│   ├── config/pose_estimator.yaml                 # 노드 파라미터 정본
│   ├── launch/apriltag_estimator.launch.py        # 런치 파일
│   ├── scripts/jitter_probe.py                    # 서비스 응답 지터 측정
│   ├── test/test_multi_tag_pose_estimator.cpp     # estimator 단위 테스트
│   ├── CMakeLists.txt
│   └── package.xml
│
├── apriltag_pose_estimator_msgs/                  # 커스텀 인터페이스
│   ├── msg/TagDetection.msg                       # 검출 결과 (debug)
│   ├── srv/TargetPointPose.srv                    # 온디맨드 포즈 쿼리
│   ├── CMakeLists.txt
│   └── package.xml
│
├── apriltag_pose_visualizer/                      # 시각화 패키지 (Python)
│   ├── apriltag_pose_visualizer/
│   │   └── pose_visualizer_node.py                # 오버레이 / RPY / 3D 산점도
│   ├── setup.py
│   └── package.xml
│
├── docker/                                        # 컨테이너 개발 환경
│   ├── Dockerfile                                 # humble-desktop 기반 이미지
│   ├── build.sh                                   # 이미지 빌드
│   ├── run.sh                                     # 컨테이너 실행 / 재접속
│   ├── entrypoint.sh                              # 호스트 소유권 복원
│   ├── commands.sh                                # 컨테이너 셸 command 정의
│   └── config.sh.example                          # 이미지·컨테이너 이름 설정 원본
│
├── realsense-ros/                                 # RealSense 드라이버 (외부, git 미추적)
├── docs/launch_ros_graph.png                      # ROS 2 그래프 캡처
├── LICENSE
└── README.md
```

> 패키지가 저장소 루트에 바로 놓인 구조로, 저장소 루트 자체가 colcon 워크스페이스. Docker는 이 루트를 컨테이너의 `/ros2_ws`에 마운트.

---

## 빠른 시작

### Option 1: Docker (권장)

```bash
# 1. 저장소 클론
git clone https://github.com/hhanoo/AprilTag_Pose_ROS2.git
cd AprilTag_Pose_ROS2

# 2. 이미지 가져오기 (직접 빌드는 ./docker/build.sh)
docker pull hhanoo/project:apriltag-pose-ros2-humble

# 3. 컨테이너 실행
./docker/run.sh

# 4. 컨테이너 내부에서 빌드
build

# 5. 실행 (호스트에서 ./docker/run.sh 를 한 번 더 실행해 셸 2개 확보)
run-camera        # 카메라 드라이버 (RealSense 기준)
run-estimator     # 포즈 추정기
```

### Option 2: Native

```bash
# 1. 저장소 클론
git clone https://github.com/hhanoo/AprilTag_Pose_ROS2.git
cd AprilTag_Pose_ROS2

# 2. 의존성 설치
rosdep install --from-paths . --ignore-src -r -y

# 3. 빌드
colcon build --symlink-install
source install/setup.bash

# 4. 실행 (터미널 2개)
ros2 launch <your_camera_driver> ...
ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py
```

---

## 시스템 요구사항

### 필수

| 항목      | 요구사항                        |
| --------- | ------------------------------- |
| OS        | Ubuntu 22.04 LTS                |
| ROS 2     | Humble Hawksbill                |
| C++       | C++17 이상 (GCC 9+ / Clang 10+) |
| Python    | 3.10 이상                       |
| 빌드 도구 | colcon, CMake 3.8 이상, rosdep  |

### 하드웨어

| 항목     | 사양                               | 비고                                             |
| -------- | ---------------------------------- | ------------------------------------------------ |
| Camera   | ROS 2 호환 RGB 카메라              | `sensor_msgs/Image` + `CameraInfo` 퍼블리시 필수 |
| AprilTag | `tag36h11` 또는 `tagStandard41h12` | 그룹의 마커 전량이 한 프레임에 보여야 추정 성공  |
| USB      | USB 3.0                            | 1280x720x30 스트리밍 기준                        |
| CPU      | 멀티코어 (Intel i5 이상 권장)      | 검출기 스레드 4개 사용                           |
| RAM      | 4GB 이상 (8GB 권장)                | —                                                |

### 소프트웨어 의존성

**시스템 라이브러리:**

- `libapriltag-dev` (AprilTag C 라이브러리)
- `libopencv-dev` (OpenCV 4.x)
- `libeigen3-dev` (Eigen3)

**ROS 2 패키지:**

- `rclcpp`, `rclpy`
- `sensor_msgs`, `geometry_msgs`, `std_msgs`, `std_srvs`, `builtin_interfaces`
- `cv_bridge`
- `rosidl_default_generators`, `rosidl_default_runtime`
- `ament_cmake_gtest` (테스트)

**Python 패키지:**

- `numpy` (<2.0, cv_bridge 호환)
- `opencv-python` (4.10.0.84)
- `matplotlib` (시각화 노드의 3D 산점도)

> Docker 이미지는 위 목록에 더해 `dt-apriltags`, `PyQt5`, `setuptools==58.2.0`, `packaging==21.3`을 함께 고정 설치.

### 외부 패키지

| 패키지        | 출처                                                                            | 용도                                                                                                     |
| ------------- | ------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------- |
| realsense-ros | [IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros) | `run-camera`가 호출하는 RealSense ROS 2 드라이버. git 미추적 외부 저장소로 워크스페이스 루트에 직접 클론 |
| hhanoo/ros    | [Docker Hub](https://hub.docker.com/r/hhanoo/ros/tags)                          | Docker 베이스 이미지 (`humble-desktop`)                                                                  |

---

## 설치

### Method 1: Docker (권장)

의존성이 이미지에 모두 포함되어 별도 설치 불필요.

```bash
docker pull hhanoo/project:apriltag-pose-ros2-humble
./docker/run.sh
```

<details>
<summary>직접 빌드 (개발자용)</summary>

```bash
# 1. 설정 파일 생성 후 IMAGE_NAME을 로컬 이름으로 변경
cp docker/config.sh.example docker/config.sh
# docker/config.sh 에서 IMAGE_NAME="apriltag-pose-ros2-humble" 로 수정

# 2. 이미지 빌드
./docker/build.sh

# 3. 컨테이너 실행
./docker/run.sh
```

</details>

### Method 2: Native

#### 0. 저장소 클론

```bash
git clone https://github.com/hhanoo/AprilTag_Pose_ROS2.git
cd AprilTag_Pose_ROS2
```

#### 1. ROS 2 Humble 설치

[ROS 2 Humble 공식 설치 가이드](https://docs.ros.org/en/humble/Installation.html)를 참고.

#### 2. 시스템 의존성 설치

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config git \
  python3-pip python3-colcon-common-extensions python3-rosdep \
  libapriltag-dev libopencv-dev libeigen3-dev
```

#### 3. Python 패키지 설치

```bash
pip3 install \
  setuptools==58.2.0 packaging==21.3 wheel \
  'numpy>=1.21.0,<2.0' \
  opencv-python==4.10.0.84 \
  matplotlib
```

#### 4. ROS 의존성 설치

```bash
rosdep update
rosdep install --from-paths . --ignore-src -r -y
```

#### 5. 카메라 드라이버 준비

사용 카메라의 ROS 2 드라이버를 별도로 설치. RealSense를 쓰는 경우 워크스페이스 루트에 드라이버 소스를 클론한 뒤 함께 빌드.

```bash
git clone -b ros2-development https://github.com/IntelRealSense/realsense-ros.git
```

---

## 빌드

### 전체 빌드

```bash
colcon build --symlink-install
source install/setup.bash
```

### 특정 패키지 빌드

```bash
colcon build --symlink-install --packages-select apriltag_pose_estimator
source install/setup.bash
```

### 클린 빌드

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### 단위 테스트

`MultiTagPoseEstimator`는 가상 카메라와 합성 코너로 ROS·카메라 없이 검증.

```bash
colcon test --packages-select-regex 'apriltag' --event-handlers console_cohesion+
colcon test-result --verbose
```

---

## 실행

`apriltag_estimator.launch.py`에는 카메라가 포함되지 않으므로 카메라 드라이버를 먼저 띄울 것.

### 전체 시스템 실행 (권장)

```bash
# 터미널 1: 카메라 드라이버
source install/setup.bash
ros2 launch <your_camera_driver> ...

# 터미널 2: 포즈 추정기
source install/setup.bash
ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py

# 터미널 3 (선택): 시각화 노드
source install/setup.bash
ros2 run apriltag_pose_visualizer pose_visualizer_node
```

### 개별 실행

디버깅 목적의 파라미터 오버라이드 예시.

```bash
# 카메라 토픽 지정
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

# 서비스 결과 창 비활성화 (headless 환경)
ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py \
  show_service_result_window:=false
```

### Docker Commands

전체 command 정의는 [commands.sh](docker/commands.sh)를 참고하세요.

| Command          | 설명                                              | 참고                                                                                                 |
| ---------------- | ------------------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| `build`          | 워크스페이스 Release 빌드 후 overlay 자동 source  | `colcon build --symlink-install`                                                                     |
| `run-tests`      | apriltag 패키지 테스트 실행 후 결과 요약          | [test_multi_tag_pose_estimator.cpp](apriltag_pose_estimator/test/test_multi_tag_pose_estimator.cpp)  |
| `run-camera`     | RealSense 카메라 실행 (1280x720x30, depth 비활성) | realsense-ros (외부 저장소)                                                                          |
| `run-estimator`  | 포즈 추정기 실행                                  | [apriltag_estimator.launch.py](apriltag_pose_estimator/launch/apriltag_estimator.launch.py)          |
| `run-visualizer` | 시각화 노드 실행                                  | [pose_visualizer_node.py](apriltag_pose_visualizer/apriltag_pose_visualizer/pose_visualizer_node.py) |
| `echo-pose`      | `/pose_estimator_node/target_poses` 모니터링      | —                                                                                                    |
| `call-pose`      | `/pose_estimator_node/target_point_pose` 호출     | [TargetPointPose.srv](apriltag_pose_estimator_msgs/srv/TargetPointPose.srv)                          |
| `cmd-help`       | command 목록 출력 (컨테이너 접속 시 자동 실행)    | —                                                                                                    |

> `docker exec`로 직접 진입하지 말고 [run.sh](docker/run.sh)를 사용할 것.  
> 이미지 존재 확인, X11 권한, `/dev`·워크스페이스 마운트, `HOST_UID`/`HOST_GID` 기반 소유권 복원, 기존 컨테이너 재사용을 한 번에 처리.

---

## 사용법

### 워크플로우

```
Camera Driver ───▶ Pose Estimator ───▶ Topic Monitor ───▶ Service Call
      │                   │                  │                 │
  run-camera        run-estimator        echo-pose         call-pose
```

### 1. 카메라 스트림 확인

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic echo /camera/color/camera_info --once
```

`CameraInfo`를 한 번이라도 받아야 estimator가 생성되므로, 이 토픽이 비어 있으면 추정이 시작되지 않음.

### 2. 포즈 추정기 실행

```bash
ros2 launch apriltag_pose_estimator apriltag_estimator.launch.py
```

기동 로그에 태그 패밀리·크기와 그룹별 `marker_ids` / `base_marker_id`가 출력되므로 설정이 의도대로 반영됐는지 확인.

### 3. 토픽 모니터링

```bash
# 활성 토픽 목록
ros2 topic list

# 그룹 순서 (latched, 늦게 접속해도 수신)
ros2 topic echo /pose_estimator_node/group_names --once

# 그룹별 base marker pose
ros2 topic echo /pose_estimator_node/target_poses

# 그룹별 유효 여부 (0 / 1)
ros2 topic echo /pose_estimator_node/group_status

# 검출 오버레이 영상
ros2 run rqt_image_view rqt_image_view
```

`PoseArray.poses[i]`가 `group_names[i]`와 1:1 대응이며, 추정에 실패한 그룹은 NaN pose로 채워짐.

### 4. 서비스 호출

```bash
ros2 service call /pose_estimator_node/target_point_pose \
  apriltag_pose_estimator_msgs/srv/TargetPointPose \
  "{request_time: {sec: 0, nanosec: 0}, group_name: ''}"
```

`group_name`이 빈 문자열이면 `group_names`의 첫 그룹을 조회. `request_time`보다 최신인 검출이 없으면 실패 응답이 돌아오므로, 최신 여부를 따지지 않으려면 `0`을 그대로 사용.

### 5. 서비스 응답 지터 측정

[jitter_probe.py](apriltag_pose_estimator/scripts/jitter_probe.py)는 서비스를 N회 반복 호출해 응답 pose의 흔들림을 위치(mm)·회전(deg) 통계로 요약하는 진단 스크립트. 빌드가 필요 없는 순수 Python이며, 노드가 실행 중인 상태에서 별도 터미널로 실행.

```bash
# 100회 호출, 0.1초 간격, 첫 그룹 조회
python3 apriltag_pose_estimator/scripts/jitter_probe.py
```

시작 시 노드 파라미터(`group_names`, `tag_size`, `tag_family`, 그룹별 `marker_ids` / `base_marker_id` / `marker_offsets`)를 조회해 출력하고, timestamp가 붙은 로그·CSV 파일을 자동 생성. CSV에는 호출별 raw 값(`idx, ok, x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg, message`)이 기록되어 외부 분석 도구로 넘기기 좋은 형태.

| 옵션             | 기본값                                   | 설명                                   |
| ---------------- | ---------------------------------------- | -------------------------------------- |
| `-n, --count`    | `100`                                    | 서비스 호출 횟수                       |
| `-i, --interval` | `0.1`                                    | 호출 간격 (초)                         |
| `--service`      | `/pose_estimator_node/target_point_pose` | 서비스 이름                            |
| `--group`        | `""`                                     | 조회할 그룹 이름 (빈 문자열 = 첫 그룹) |
| `--node`         | `--service` 경로에서 추출                | 파라미터를 조회할 노드 이름            |
| `--log`          | `/ros2_ws/jitter_probe_<timestamp>.log`  | 로그 파일 경로                         |
| `--csv`          | 로그와 같은 위치의 `.csv`                | CSV 파일 경로                          |
| `--no-log`       | off                                      | 파일 저장을 끄고 콘솔만 사용           |
| `--no-params`    | off                                      | 기동 시 노드 파라미터 조회 생략        |

```bash
# 200회 호출, 50ms 간격
python3 .../jitter_probe.py -n 200 -i 0.05

# 특정 그룹 조회
python3 .../jitter_probe.py --group secondary

# 저장 경로 직접 지정
python3 .../jitter_probe.py --log /tmp/run1.log --csv /tmp/run1.csv
```

출력 예시:

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

- `mean`: 평균 (정적 위치와 반복 재현성 확인용)
- `std`: 1-σ 표준편차 (낮을수록 안정)
- `p2p`: peak-to-peak (max−min, 실제 떨림 폭)

---

## 설정

### Docker 설정

**[docker/config.sh](docker/config.sh.example)**

```bash
IMAGE_NAME="hhanoo/project:apriltag-pose-ros2-humble"  # Docker 이미지 이름
CONTAINER_NAME="apriltag-pose-ros2-humble"             # 컨테이너 이름
ROS_DOMAIN_ID=""                                       # ROS 2 도메인 ID
XAUTHORITY_PATH="$HOME/.Xauthority"                    # X11 .Xauthority 경로
```

> `config.sh`는 git 미추적 파일이며, `build.sh` / `run.sh`가 없을 때 `config.sh.example`에서 자동 복사.  
> 로컬 빌드로 전환하려면 `IMAGE_NAME`을 `apriltag-pose-ros2-humble` 등으로 바꾼 뒤 `./docker/build.sh` 실행.

### 포즈 추정기 설정

**[apriltag_pose_estimator/config/pose_estimator.yaml](apriltag_pose_estimator/config/pose_estimator.yaml)**

<!-- prettier-ignore -->
```yaml
pose_estimator_node:
  ros__parameters:
    # 공통 AprilTag 설정
    tag_size: 0.02778                   # 태그 검은 사각형 한 변 길이 (m)
    tag_family: 'tagStandard41h12'      # tag36h11 또는 tagStandard41h12

    # 입력 토픽
    camera_topic: 'camera/color/image_raw'
    camera_info_topic: 'camera/color/camera_info'
    camera_frame: 'camera_color_optical_frame'
    use_distortion_from_camera_info: false

    # 출력 토픽 (미지정 시 ~/<topic> 기본값 사용)
    tag_detection_topic:   'pose_estimator_node/tag_detections'
    target_poses_topic:    'pose_estimator_node/target_poses'
    group_names_topic:     'pose_estimator_node/group_names'      # latched
    group_status_topic:    'pose_estimator_node/group_status'
    detection_image_topic: 'pose_estimator_node/detection_image'

    publish_detection_image: true       # 구독자가 없으면 인코딩 자체를 건너뜀

    # 서비스 결과 창 (headless 환경에서는 false)
    show_service_result_window: false
    display_width: 0                    # 0 = 원본 해상도
    display_height: 0

    # Multi-group 정의: group_names 순서가 PoseArray / group_status 순서
    group_names:
      - default
      - secondary

    groups:
      default:
        marker_ids: [0, 1, 2]
        base_marker_id: 1
        # 마커마다 6개 값 [x, y, z, roll, pitch, yaw] (m, rad), T_base <- marker_i
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

주요 파라미터:

- **tag_size**: AprilTag 검은 사각형 한 변의 실측 길이(m). 병진 스케일에 직결되므로 mm↔m 환산에 주의
- **group_names**: v3.0.0부터 필수. 비어 있으면 FATAL 로그와 함께 노드 종료. 단일 그룹만 쓰더라도 그룹 하나를 명시
- **marker_ids**: 그룹을 구성하는 마커 ID. 그중 하나라도 프레임에서 빠지면 해당 그룹은 추정 실패로 처리
- **base_marker_id**: 그룹 출력 pose의 기준 마커. `marker_ids`에 없으면 FATAL 후 종료
- **marker_offsets**: `marker_ids` 순서대로 마커마다 6개 값을 나열한 평면 리스트(총 `6N`개). base marker 항목은 정의상 identity이며, 0이 아니면 WARN 후 0으로 강제. 회전 합성은 `R = Rz(yaw) * Ry(pitch) * Rx(roll)` (intrinsic Z-Y-X)
- **use_distortion_from_camera_info**: `false`면 왜곡 계수를 0으로 두어 rectified 이미지에 대응, `true`면 `camera_info.d`를 그대로 사용해 raw 이미지에 대응
- **display_width / display_height**: 서비스 결과 창 크기. `0`이면 원본 해상도 그대로 표시

> **v3.0.0 ABI break**
>
> - yaml: `group_names` + `groups.<name>.{...}` 스키마가 필수. 최상위 `marker_ids` / `marker_offsets` / `target_points`를 쓰던 이전 단일 그룹 형식은 지원 종료
> - srv: `TargetPointPose.srv` request에 `string group_name` 필드 추가. 빈 문자열은 `group_names`의 첫 그룹을 지칭
> - 출력: `target_points` 개념 제거. 그룹당 base marker pose 1개만 내보내므로 `PoseArray.poses[i]`가 `group_names[i]`와 1:1 매핑

### Launch 인자

| 인자                         | 기본값                               | 설명                          |
| ---------------------------- | ------------------------------------ | ----------------------------- |
| `config_file`                | `<share>/config/pose_estimator.yaml` | YAML 설정 파일 경로           |
| `show_service_result_window` | `""` (YAML 값 사용)                  | 서비스 결과 창 표시 여부      |
| `camera_topic`               | `""` (YAML 값 사용)                  | 카메라 이미지 토픽 오버라이드 |
| `camera_info_topic`          | `""` (YAML 값 사용)                  | 카메라 정보 토픽 오버라이드   |
| `tag_family`                 | `""` (YAML 값 사용)                  | 태그 패밀리 오버라이드        |
| `tag_size`                   | `""` (YAML 값 사용)                  | 태그 크기 오버라이드          |

---

## API / 인터페이스

**노드**

| 이름                   | 언어   | 패키지                   | 설명                                   |
| ---------------------- | ------ | ------------------------ | -------------------------------------- |
| `pose_estimator_node`  | C++    | apriltag_pose_estimator  | AprilTag 검출과 그룹별 6-DOF 포즈 추정 |
| `pose_visualizer_node` | Python | apriltag_pose_visualizer | 검출 오버레이, RPY 환산, 3D 산점도     |

**Published 토픽**

| 이름                                   | 타입                                        | QoS                            | 설명                                                              |
| -------------------------------------- | ------------------------------------------- | ------------------------------ | ----------------------------------------------------------------- |
| `/pose_estimator_node/target_poses`    | `geometry_msgs/PoseArray`                   | KEEP_LAST(10)                  | 그룹별 base marker pose. `group_names` 순서이며 실패 그룹은 NaN   |
| `/pose_estimator_node/group_names`     | `std_msgs/String`                           | transient_local + KEEP_LAST(1) | 그룹 이름 CSV. 기동 직후 1회 publish되는 latched 토픽             |
| `/pose_estimator_node/group_status`    | `std_msgs/UInt8MultiArray`                  | KEEP_LAST(10)                  | `group_names` 순서대로 0(invalid) / 1(valid)                      |
| `/pose_estimator_node/detection_image` | `sensor_msgs/Image` (BGR8)                  | KEEP_LAST(10)                  | 전 그룹 검출 결과를 누적한 오버레이. 구독자가 없으면 publish 생략 |
| `/pose_estimator_node/tag_detections`  | `apriltag_pose_estimator_msgs/TagDetection` | KEEP_LAST(10)                  | 디버그용 raw 검출. multi-group에서는 첫 그룹의 rvec/tvec          |

**Subscribed 토픽**

| 이름                                  | 타입                                        | 구독 노드            | 설명                                  |
| ------------------------------------- | ------------------------------------------- | -------------------- | ------------------------------------- |
| `/camera/color/image_raw`             | `sensor_msgs/Image`                         | pose_estimator_node  | 입력 카메라 영상                      |
| `/camera/color/camera_info`           | `sensor_msgs/CameraInfo`                    | pose_estimator_node  | 카메라 내부 파라미터. 최초 1회만 반영 |
| `/camera/camera/color/image_raw`      | `sensor_msgs/Image`                         | pose_visualizer_node | 시각화용 입력 영상 (노드에 하드코딩)  |
| `/pose_estimator_node/tag_detections` | `apriltag_pose_estimator_msgs/TagDetection` | pose_visualizer_node | 오버레이·산점도 입력                  |

**서비스**

| 이름                                     | 타입                                           | 제공 노드           | 설명                                   |
| ---------------------------------------- | ---------------------------------------------- | ------------------- | -------------------------------------- |
| `/pose_estimator_node/target_point_pose` | `apriltag_pose_estimator_msgs/TargetPointPose` | pose_estimator_node | 지정 그룹의 최신 base marker pose 조회 |

**커스텀 메시지**

`TagDetection.msg`

```msg
bool       tag_detected    # 태그 검출 플래그
float64[9] camera_matrix   # 3x3 내부 파라미터 행렬 (row-major)
float64[]  dist_coeffs     # 왜곡 계수 (OpenCV 순서)
float64[3] rvec            # Rodrigues 회전 벡터
float64[3] tvec            # 병진 벡터 (m)
float64    tag_size        # 태그 크기 (m)
```

`TargetPointPose.srv`

```srv
# Request
builtin_interfaces/Time request_time    # 클라이언트 기준 시각
string group_name                       # 조회 그룹 ("" = 첫 그룹)
---
# Response
bool success                            # 성공 여부
string message                          # 상태 메시지
builtin_interfaces/Time data_time       # 응답 pose의 타임스탬프
geometry_msgs/PoseArray poses           # 해당 그룹의 base marker pose (요소 1개)
```

> 알 수 없는 그룹 이름, 유효한 최신 pose 부재, `request_time`보다 오래된 검출은 모두 `success: false`와 사유 메시지로 응답.

**네트워크 구성 (Docker)**

| 항목          | 값                                    | 비고                                      |
| ------------- | ------------------------------------- | ----------------------------------------- |
| Network mode  | `host`                                | 호스트 네트워크 공유                      |
| IPC           | `host`                                | 공유 메모리 전송 활성화                   |
| Privileged    | `true`                                | `/dev` 전체 마운트와 함께 USB 카메라 접근 |
| ROS_DOMAIN_ID | `config.sh`의 `ROS_DOMAIN_ID`         | 기본값은 빈 문자열                        |
| X11           | `/tmp/.X11-unix`, `$HOME/.Xauthority` | GUI·RViz 포워딩                           |

---

## 문제 해결

### 1. 카메라가 인식되지 않을 때

증상:

```
[WARN] Camera info not received yet
```

해결:

```bash
# 카메라 토픽이 실제로 뜨는지 확인
ros2 topic list | grep camera

# USB 연결 확인
lsusb

# 권한 부여 후 재로그인
sudo usermod -aG video,plugdev $USER
```

> estimator는 `CameraInfo` 수신 시점에 그룹별 estimator를 생성하므로, 이미지만 오고 `CameraInfo`가 없으면 계속 대기 상태. 카메라 모델별 진단 절차는 해당 드라이버 문서를 참고.

### 2. 태그가 검출되지 않을 때

증상:

```
Detected 0 tags: []
```

해결:

- `tag_family`가 실제 인쇄한 패밀리와 같은지 확인 (`tag36h11` / `tagStandard41h12` 외에는 `tag36h11`로 대체됨)
- `tag_size`를 검은 사각형 한 변의 실측값(m)으로 지정
- 조명을 균일하게 하고 그림자·반사를 줄일 것
- 카메라와 태그 거리 0.3~3 m, 입사각 45도 이내 권장
- 그룹의 `marker_ids` 전량이 한 프레임에 보여야 추정 성공. 일부만 보이면 그룹 전체가 실패 처리

### 3. 포즈가 흔들릴 때

증상:

```
group_status가 1과 0을 반복하거나 pose 값이 튐
```

해결:

```bash
# 1. tag_size 실측값 재확인 (mm -> m 환산 주의)
# 2. marker_offsets가 마커 간 실제 거리·회전과 일치하는지 확인
# 3. 카메라 해상도를 올리거나 프레임레이트를 낮춰 이미지 품질 확보
# 4. 정량 측정
python3 apriltag_pose_estimator/scripts/jitter_probe.py -n 200
```

> `marker_offsets`가 실제 배치와 어긋나면 재투영 오차가 임계값(15픽셀)을 넘어 직전 pose 재사용과 실패가 번갈아 발생. 이 상태에서는 축 시각화도 함께 끊김.

### 4. libapriltag를 찾지 못할 때

증상:

```
CMake Error: Could not find libapriltag
```

해결:

```bash
sudo apt update
sudo apt install -y libapriltag-dev

# 여전히 실패하면 universe 저장소 활성화
sudo add-apt-repository universe
sudo apt update
sudo apt install -y libapriltag-dev
```

### 5. cv_bridge NumPy 버전 충돌

증상:

```
ImportError: numpy.core.multiarray failed to import
```

해결:

```bash
pip3 install 'numpy>=1.21.0,<2.0'
```

### 6. 시각화 노드에 영상이 뜨지 않을 때

증상:

```
AprilTag Pose Visualizer 창이 열리지 않거나 검게 유지됨
```

해결:

- `pose_visualizer_node`의 이미지 구독 토픽은 `/camera/camera/color/image_raw`로 하드코딩되어 있어, estimator 기본값(`/camera/color/image_raw`)과 네임스페이스가 다름
- 카메라를 `camera` 네임스페이스 아래에서 띄우거나, `ros2 run ... --ros-args -r` 로 리매핑해 토픽을 맞출 것

```bash
ros2 run apriltag_pose_visualizer pose_visualizer_node --ros-args \
  -r /camera/camera/color/image_raw:=/camera/color/image_raw
```

### 7. 서비스 결과 창이 뜨지 않을 때

증상:

```
[WARN] show_service_result_window=true but no GUI session.
```

해결:

- `DISPLAY` 또는 `WAYLAND_DISPLAY`가 비어 있으면 창을 열지 않고 경고만 출력
- Docker에서는 [run.sh](docker/run.sh)로 진입해 X11 마운트와 `DISPLAY` 전달을 받을 것
- headless 운용이면 `show_service_result_window: false`로 두고 `detection_image` 토픽을 사용

---

## 라이선스

이 프로젝트는 MIT 라이선스로 배포됩니다. 자세한 내용은 [LICENSE](LICENSE) 파일을 참조하세요.

> 워크스페이스에 함께 두는 realsense-ros는 본 저장소에 포함되지 않는 외부 프로젝트로, Apache-2.0 등 해당 저장소의 라이선스를 따릅니다.

---

## Maintainer

**hhanoo** (woo980711@gmail.com)

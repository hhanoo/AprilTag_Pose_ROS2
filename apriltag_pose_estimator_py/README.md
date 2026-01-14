# AprilTag Pose Estimator

AprilTag 기반 다중 타겟 포즈 추정 패키지입니다.

## 개요

`apriltag_pose_estimator_py`는 `dt_apriltags` 라이브러리를 사용하여 카메라 이미지에서 AprilTag를 검출하고, 다중 마커 기반으로 정확한 타겟 포인트의 포즈를 계산합니다. 카메라 이미지 구독부터 AprilTag 검출, 포즈 추정까지 모든 기능을 하나의 노드에서 통합 처리합니다.

## 주요 기능

- **AprilTag 검출**: `dt_apriltags`를 사용한 실시간 AprilTag 검출
- **다중 마커 포즈 추정**: 여러 마커의 위치 관계를 이용한 정확한 포즈 계산
- **타겟 포인트 계산**: 베이스 마커 기준 다중 타겟 포인트의 3D 위치 추정
- **시각화 지원**: RViz용 visualization markers 및 detection 이미지 퍼블리싱
- **실시간 카메라 통합**: 카메라 이미지 및 camera_info 직접 구독

## ROS2 API

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `target_poses` | `geometry_msgs/PoseArray` | 계산된 타겟 포인트들의 포즈 배열 |
| `visualization_markers` | `visualization_msgs/MarkerArray` | RViz 시각화용 마커 배열 |
| `detection_image` | `sensor_msgs/Image` | AprilTag 검출 결과가 그려진 이미지 |

**Note**: 토픽 이름은 상대 경로로 정의되어 있어, 노드의 네임스페이스에 따라 실제 경로가 결정됩니다. 기본적으로는 `/target_poses` 형태로 퍼블리시됩니다.

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `camera_topic` | `sensor_msgs/Image` | 입력 카메라 이미지 (BGR8 인코딩) |
| `camera_info_topic` | `sensor_msgs/CameraInfo` | 카메라 내부 파라미터 (intrinsics, distortion) |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `marker_ids` | list[int] | `[0, 1, 2]` | 사용할 AprilTag 마커 ID 목록 |
| `base_marker_id` | int | `1` | 베이스 마커 ID (좌표계 기준점) |
| `tag_size` | float | `0.02778` | AprilTag 크기 (미터 단위) |
| `tag_family` | string | `'tagStandard41h12'` | AprilTag family (예: tag36h11, tagStandard41h12) |
| `marker_offsets` | list[float] | `[0.065, 0.0]` | 마커 간 X, Y 오프셋 (미터) |
| `camera_topic` | string | `'color/image_raw'` | 카메라 이미지 토픽 이름 (상대 경로) |
| `camera_info_topic` | string | `'color/camera_info'` | 카메라 정보 토픽 이름 (상대 경로) |
| `camera_frame` | string | `'camera_color_optical_frame'` | 카메라 좌표계 프레임 ID |
| `publish_visualization` | bool | `true` | Visualization markers 퍼블리시 여부 |
| `publish_detection_image` | bool | `true` | Detection 이미지 퍼블리시 여부 |
| `point_offsets` | list[float] | `[0.001, 0.001, -0.150, ...]` | 타겟 포인트 오프셋 (flat list, Nx3로 자동 변환) |

## Configuration

### YAML 설정 파일

패키지는 `config/pose_estimator.yaml` 파일을 통해 기본 설정을 관리합니다. 이 파일에는 모든 노드 파라미터가 정의되어 있습니다:

- **marker_ids**: 감지할 AprilTag ID 목록
- **base_marker_id**: 좌표계 기준이 되는 베이스 마커 ID
- **tag_size**: AprilTag의 실제 크기 (미터 단위)
- **tag_family**: 사용할 AprilTag family (예: tagStandard41h12, tag36h11)
- **marker_offsets**: 마커 간 X, Y 오프셋
- **point_offsets**: 타겟 포인트 오프셋 (flat list 형태로 저장, 자동으로 Nx3 배열로 변환)
- **camera_topic**: 카메라 이미지 토픽
- **camera_info_topic**: 카메라 정보 토픽
- **camera_frame**: 카메라 좌표계 프레임 ID
- **publish_visualization**: 시각화 마커 퍼블리시 여부
- **publish_detection_image**: 검출 이미지 퍼블리시 여부

### 설정 커스터마이징

설정을 변경하는 방법은 3가지입니다:

#### 1. YAML 파일 직접 수정

`config/pose_estimator.yaml` 파일을 편집하여 기본값을 변경:

```yaml
pose_estimator_node:
  ros__parameters:
    tag_size: 0.03  # 태그 크기 변경
    marker_ids: [0, 1, 2, 3]  # 마커 ID 추가
    point_offsets: [
      0.0, 0.0, -0.2,
      0.1, 0.1, -0.2,
      # ... 추가 포인트
    ]
```

#### 2. Launch argument로 오버라이드

Launch 파일 실행 시 특정 파라미터만 변경:

```bash
ros2 launch apriltag_pose_estimator_py apriltag_estimator.launch.py \
  tag_size:=0.03 \
  tag_family:=tag36h11
```

#### 3. 런타임에 파라미터 변경

노드 실행 중 파라미터 동적 변경:

```bash
# 파라미터 확인
ros2 param list /pose_estimator_node
ros2 param get /pose_estimator_node tag_size

# 파라미터 변경
ros2 param set /pose_estimator_node tag_size 0.03
```

**Note**: `point_offsets`는 flat list 형태로 저장되며 (예: `[x0, y0, z0, x1, y1, z1, ...]`), 노드 초기화 시 자동으로 Nx3 배열로 변환됩니다.

## 사용 방법

### 1. 단독 노드 실행

기본 설정으로 실행:

```bash
ros2 run apriltag_pose_estimator_py pose_estimator_node
```

파라미터와 함께 실행:

```bash
ros2 run apriltag_pose_estimator_py pose_estimator_node --ros-args \
  -p tag_family:=tag36h11 \
  -p tag_size:=0.05 \
  -p camera_topic:=camera/image_raw
```

### 2. Launch 파일 사용

기본 실행 (detection 이미지 뷰어 포함):

```bash
ros2 launch apriltag_pose_estimator_py apriltag_estimator.launch.py
```

파라미터 커스터마이징:

```bash
ros2 launch apriltag_pose_estimator_py apriltag_estimator.launch.py \
  tag_family:=tag36h11 \
  tag_size:=0.05 \
  camera_topic:=/my_camera/image_raw \
  show_detection:=false
```

**Launch Arguments:**

- `camera_topic`: 카메라 이미지 토픽 (default: `/color/image_raw`)
- `camera_info_topic`: 카메라 정보 토픽 (default: `/color/camera_info`)
- `tag_family`: AprilTag family (default: `tagStandard41h12`)
- `tag_size`: 태그 크기 미터 단위 (default: `0.02778`)
- `show_detection`: detection 이미지 뷰어 표시 여부 (default: `true`)

### 3. 다른 Launch 파일에 통합

```python
from launch_ros.actions import Node

pose_estimator_node = Node(
    package='apriltag_pose_estimator_py',
    executable='pose_estimator_node',
    name='pose_estimator_node',
    parameters=[{
        'marker_ids': [0, 1, 2],
        'base_marker_id': 1,
        'tag_size': 0.02778,
        'tag_family': 'tagStandard41h12',
        'camera_topic': 'color/image_raw',
        'camera_info_topic': 'color/camera_info'
    }]
)
```

## 의존성

### ROS2 패키지
- `rclpy`: ROS2 Python 클라이언트 라이브러리
- `geometry_msgs`: 포즈 및 geometry 메시지
- `sensor_msgs`: 이미지 및 카메라 정보 메시지
- `visualization_msgs`: RViz 시각화 마커
- `cv_bridge`: OpenCV-ROS2 이미지 변환
- `tf2_ros`, `tf2_geometry_msgs`: TF2 좌표 변환

### Python 라이브러리
- `dt-apriltags`: AprilTag 검출 라이브러리
- `numpy>=1.21,<2.0`: NumPy (ROS2 Humble cv_bridge 호환성을 위해 1.x 버전 필요)
- `opencv-python`: OpenCV

## 알고리즘

### Multi-Tag Pose Estimation

이 패키지는 `MultiTagPoseEstimator` 클래스를 사용하여 다중 마커 기반 포즈 추정을 수행합니다:

1. **AprilTag 검출**: 카메라 이미지에서 모든 AprilTag 검출
2. **3D-2D 대응**: 각 마커의 3D 월드 좌표와 2D 이미지 좌표 매칭
3. **PnP 문제 해결**: `cv2.solvePnP`를 사용하여 카메라 포즈 추정
4. **타겟 포인트 계산**: 베이스 마커 기준으로 각 타겟 포인트의 3D 위치 계산

이 방식은 단일 마커보다 훨씬 정확하고 안정적인 포즈 추정을 제공합니다.

## 트러블슈팅

### AprilTag가 검출되지 않는 경우

**증상**: `No AprilTags detected` 로그 메시지

**해결 방법**:

1. **카메라 토픽 확인**:
   ```bash
   ros2 topic list | grep image
   ros2 topic echo /color/image_raw --once
   ```

2. **Tag family 확인**: 사용 중인 AprilTag의 family와 파라미터가 일치하는지 확인
   - `tag36h11`: 표준 36h11 family
   - `tagStandard41h12`: 41h12 family (주의: `tag41h12`가 아님!)

3. **조명 및 거리**: 적절한 조명과 카메라-태그 거리 유지

### Camera info not received

**증상**: `Camera info not received yet` 경고 메시지

**해결 방법**:

1. **Camera info 토픽 확인**:
   ```bash
   ros2 topic list | grep camera_info
   ros2 topic echo /color/camera_info --once
   ```

2. **토픽 이름 수정**: 파라미터로 올바른 camera_info 토픽 지정

### Detection 이미지가 보이지 않는 경우

**증상**: `rqt_image_view`에서 이미지가 표시되지 않음

**해결 방법**:

1. **토픽 확인**:
   ```bash
   ros2 topic list | grep detection_image
   ros2 topic hz /detection_image
   ```

2. **파라미터 확인**: `publish_detection_image` 파라미터가 `true`인지 확인

3. **수동 실행**:
   ```bash
   ros2 run rqt_image_view rqt_image_view /detection_image
   ```

### Pose estimation failed

**증상**: `Pose estimation failed. Required IDs: [0, 1, 2], Detected IDs: [0, 1]`

**원인**: 필요한 모든 마커가 검출되지 않음

**해결 방법**:
- 모든 필수 마커가 카메라 시야에 들어오도록 조정
- `marker_ids` 파라미터를 실제 사용 가능한 마커 ID로 수정

## 개발 정보

### 패키지 구조

```
apriltag_pose_estimator_py/
├── config/
│   └── pose_estimator.yaml           # YAML 설정 파일
├── launch/
│   └── apriltag_estimator.launch.py  # Launch 파일
├── apriltag_pose_estimator_py/
│   ├── __init__.py
│   ├── pose_estimator_node.py        # 메인 노드 (통합 검출 + 추정)
│   └── multi_tag_pose_estimator.py   # 다중 마커 포즈 추정 알고리즘
├── resource/
│   └── apriltag_pose_estimator_py
├── package.xml                        # 패키지 메타데이터
├── setup.py                           # Python 패키지 설정
├── setup.cfg                          # 설정 파일
└── README.md                          # 이 문서
```

### 빌드 방법

```bash
cd /ros2_ws
colcon build --packages-select apriltag_pose_estimator_py
source install/setup.bash
```

### 테스트

카메라 및 AprilTag 준비 후:

```bash
# 노드 실행
ros2 run apriltag_pose_estimator_py pose_estimator_node

# 다른 터미널에서 토픽 확인
ros2 topic list
ros2 topic hz /target_poses
ros2 topic echo /target_poses --once

# RViz에서 시각화
rviz2
# Add > MarkerArray > Topic: /visualization_markers
```

## 라이선스

MIT License

## 관련 패키지

- [`realsense_bridge_py`](../realsense_bridge_py/): Intel RealSense 카메라 ROS2 브릿지

## 참고 자료

- [AprilTag Library](https://github.com/AprilRobotics/apriltag)
- [dt-apriltags Python Wrapper](https://github.com/duckietown/lib-dt-apriltags)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)


# RealSense Bridge

Intel RealSense D400 시리즈 카메라를 ROS2 Humble과 연결하는 브릿지 패키지입니다.

## 개요

`realsense_bridge_py`는 `pyrealsense2` 라이브러리를 사용하여 Intel RealSense 카메라의 데이터를 ROS2 토픽으로 퍼블리시합니다. Color 및 Depth 이미지 스트림을 지원하며, 카메라 내부 파라미터(intrinsics)도 함께 제공합니다.

## 주요 기능

- **Color 이미지 스트림**: BGR8 포맷의 컬러 이미지 퍼블리싱
- **Depth 이미지 스트림**: 16-bit unsigned integer 포맷의 깊이 이미지 (선택적)
- **카메라 정보**: 카메라 내부 파라미터 (K, P, D 행렬) 퍼블리싱
- **디바이스 관리**: 여러 카메라 중 시리얼 번호로 특정 디바이스 선택 가능
- **설정 가능**: 해상도, 프레임레이트, 자동 노출 등 다양한 파라미터 지원

## ROS2 API

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `color/image_raw` | `sensor_msgs/Image` | Color 이미지 (BGR8) |
| `color/camera_info` | `sensor_msgs/CameraInfo` | Color 카메라 내부 파라미터 |
| `depth/image_raw` | `sensor_msgs/Image` | Depth 이미지 (16UC1, enable_depth=true일 때만) |
| `depth/camera_info` | `sensor_msgs/CameraInfo` | Depth 카메라 내부 파라미터 (enable_depth=true일 때만) |

**Note**: 토픽 이름은 상대 경로로 정의되어 있어, 노드의 네임스페이스에 따라 실제 경로가 결정됩니다. 기본적으로는 `/color/image_raw` 형태로 퍼블리시됩니다.

### Services

| Service | Type | Description |
|---------|------|-------------|
| `get_device_info` | `std_srvs/Trigger` | 연결된 카메라의 정보 반환 (시리얼 번호, 해상도, FPS) |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_number` | string | `''` | 특정 카메라 시리얼 번호 (비어있으면 첫 번째 디바이스 사용) |
| `frame_rate` | int | `30` | 카메라 프레임 레이트 (FPS) |
| `resolution.width` | int | `1280` | 이미지 너비 (픽셀) |
| `resolution.height` | int | `720` | 이미지 높이 (픽셀) |
| `frame_ids.color` | string | `'camera_color_optical_frame'` | Color 이미지의 TF 프레임 ID |
| `frame_ids.depth` | string | `'camera_depth_optical_frame'` | Depth 이미지의 TF 프레임 ID |
| `enable_depth` | bool | `false` | Depth 스트림 활성화 여부 |
| `auto_exposure` | bool | `true` | 자동 노출 설정 |

## Configuration

### YAML 설정 파일

패키지는 `config/realsense.yaml` 파일을 통해 기본 설정을 관리합니다. 이 파일에는 모든 노드 파라미터가 정의되어 있습니다:

**참고**: 모든 토픽은 **상대 경로**로 퍼블리시됩니다 (예: `color/image_raw`). 실제 전체 경로는 노드의 namespace에 따라 결정됩니다.

- **serial_number**: 특정 카메라 시리얼 번호 (비어있으면 첫 번째 디바이스 사용)
- **frame_rate**: 카메라 프레임 레이트 (FPS)
- **resolution**: 이미지 해상도 (width, height)
- **frame_ids**: Color 및 Depth 이미지의 TF 프레임 ID
- **enable_depth**: Depth 스트림 활성화 여부
- **auto_exposure**: 자동 노출 설정

### 설정 커스터마이징

설정을 변경하는 방법은 3가지입니다:

#### 1. YAML 파일 직접 수정

`config/realsense.yaml` 파일을 편집하여 기본값을 변경:

```yaml
realsense_node:
  ros__parameters:
    frame_rate: 30
    resolution:
      width: 1280
      height: 720
    enable_depth: true
```

#### 2. Launch argument로 오버라이드

Launch 파일 실행 시 특정 파라미터만 변경:

```bash
ros2 launch realsense_bridge_py realsense.launch.py \
  frame_rate:=30 \
  width:=1280 \
  height:=720 \
  enable_depth:=true
```

#### 3. 런타임에 파라미터 변경

노드 실행 중 파라미터 동적 변경:

```bash
# 파라미터 확인
ros2 param list /realsense_node
ros2 param get /realsense_node frame_rate

# 파라미터 변경 (일부 파라미터는 재시작 필요)
ros2 param set /realsense_node frame_rate 30
```

**Note**: 해상도나 프레임레이트 같은 카메라 설정은 노드 초기화 시에만 적용되므로, 변경 후 노드를 재시작해야 합니다.

## 사용 방법

### 1. 단독 노드 실행

기본 설정으로 실행:

```bash
ros2 run realsense_bridge_py realsense_node
```

파라미터와 함께 실행:

```bash
ros2 run realsense_bridge_py realsense_node --ros-args \
  -p frame_rate:=60 \
  -p resolution.width:=640 \
  -p resolution.height:=480 \
  -p enable_depth:=true
```

### 2. Launch 파일 사용

기본 실행 (이미지 뷰어 포함):

```bash
ros2 launch realsense_bridge_py realsense.launch.py
```

파라미터 커스터마이징:

```bash
ros2 launch realsense_bridge_py realsense.launch.py \
  frame_rate:=60 \
  width:=640 \
  height:=480 \
  enable_depth:=true \
  show_image:=false
```

**Launch Arguments:**

- `enable_depth`: Depth 스트림 활성화 여부 (default: `false`)
- `frame_rate`: 프레임 레이트 (default: `30`)
- `width`: 이미지 너비 (default: `1280`)
- `height`: 이미지 높이 (default: `720`)
- `show_image`: rqt_image_view 자동 실행 여부 (default: `true`)
- `serial_number`: 특정 카메라 시리얼 번호 (default: `''` - 첫 번째 디바이스)

### 3. 다른 Launch 파일에 통합

```python
from launch_ros.actions import Node

realsense_node = Node(
    package='realsense_bridge_py',
    executable='realsense_node',
    name='realsense_node',
    parameters=[{
        'frame_rate': 30,
        'resolution.width': 1280,
        'resolution.height': 720,
        'enable_depth': False
    }]
)
```

## 의존성

### ROS2 패키지
- `rclpy`: ROS2 Python 클라이언트 라이브러리
- `sensor_msgs`: 이미지 및 카메라 정보 메시지
- `std_srvs`: 표준 서비스 정의
- `cv_bridge`: OpenCV-ROS2 이미지 변환

### Python 라이브러리
- `pyrealsense2`: Intel RealSense SDK Python wrapper
- `numpy>=1.21,<2.0`: NumPy (ROS2 Humble cv_bridge 호환성을 위해 1.x 버전 필요)
- `opencv-python`: OpenCV

## 트러블슈팅

### 카메라를 찾을 수 없는 경우

**증상**: `No RealSense devices found!` 에러 메시지

**해결 방법**:

1. **USB 연결 확인**:
   ```bash
   lsusb | grep Intel
   ```
   Intel Corp. RealSense 디바이스가 보여야 합니다.

2. **Docker 환경**: USB 디바이스 권한 필요
   - `--privileged` 모드로 실행하거나
   - 특정 디바이스 마운트: `--device=/dev/bus/usb`

3. **udev 규칙 확인**:
   ```bash
   ls /etc/udev/rules.d/99-realsense-libusb.rules
   ```
   파일이 없다면 [Intel RealSense GitHub](https://github.com/IntelRealSense/librealsense/blob/master/config/99-realsense-libusb.rules)에서 다운로드하여 설치:
   ```bash
   sudo wget -O /etc/udev/rules.d/99-realsense-libusb.rules \
     https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

### "Device or resource busy" 에러

**증상**: `xioctl(VIDIOC_S_FMT) failed, errno=16 Last Error: Device or resource busy`

**원인**: 다른 프로세스가 이미 카메라를 사용 중입니다.

**해결 방법**:

1. 실행 중인 RealSense 관련 프로세스 확인:
   ```bash
   ps aux | grep realsense
   ```

2. 기존 프로세스 종료:
   ```bash
   pkill -f realsense_node
   ```

3. 노드 재시작

### NumPy 버전 호환성 문제

**증상**: `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x`

**원인**: ROS2 Humble의 `cv_bridge`는 NumPy 1.x ABI로 컴파일되어 있습니다.

**해결 방법**:

```bash
pip install "numpy>=1.21,<2.0"
```

Docker 환경에서는 Dockerfile에 명시적으로 NumPy 버전을 고정하세요.

### 이미지가 보이지 않는 경우

1. **토픽 확인**:
   ```bash
   ros2 topic list
   ros2 topic echo /color/image_raw --once
   ```

2. **rqt_image_view로 수동 확인**:
   ```bash
   ros2 run rqt_image_view rqt_image_view /color/image_raw
   ```

3. **X11 forwarding 확인** (Docker 환경):
   ```bash
   echo $DISPLAY  # :0 또는 유사한 값이 출력되어야 함
   xhost +local:docker  # 호스트에서 실행
   ```

## 개발 정보

### 패키지 구조

```
realsense_bridge_py/
├── config/
│   └── realsense.yaml           # YAML 설정 파일
├── launch/
│   └── realsense.launch.py      # Launch 파일
├── realsense_bridge_py/
│   ├── __init__.py
│   └── realsense_node.py        # 메인 노드 구현
├── resource/
│   └── realsense_bridge_py
├── package.xml                   # 패키지 메타데이터
├── setup.py                      # Python 패키지 설정
├── setup.cfg                     # 설정 파일
└── README.md                     # 이 문서
```

### 빌드 방법

```bash
cd /ros2_ws
colcon build --packages-select realsense_bridge_py
source install/setup.bash
```

### 테스트

카메라 연결 후:

```bash
# 노드 실행
ros2 run realsense_bridge_py realsense_node

# 다른 터미널에서 토픽 확인
ros2 topic list
ros2 topic hz /color/image_raw

# 서비스 호출
ros2 service call /get_device_info std_srvs/srv/Trigger
```

## 라이선스

MIT License

## 참고 자료

- [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense)
- [pyrealsense2 Documentation](https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.html)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)


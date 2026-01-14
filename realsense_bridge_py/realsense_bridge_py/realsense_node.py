#!/usr/bin/env python3
"""
RealSense Bridge Node
Bridges Intel RealSense camera with ROS2
"""

import numpy as np
import pyrealsense2 as rs
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger


class RealsenseNode(Node):
    """RealSense to ROS2 Bridge Node"""

    # ========================================================
    # Initialization
    # ========================================================
    def __init__(self):
        super().__init__("realsense_node")

        # Declare parameters
        self.declare_parameter("serial_number", "")
        self.declare_parameter("frame_rate", 30)
        self.declare_parameter("resolution.width", 1280)
        self.declare_parameter("resolution.height", 720)
        self.declare_parameter("frame_ids.color", "camera_color_optical_frame")
        self.declare_parameter("frame_ids.depth", "camera_depth_optical_frame")
        self.declare_parameter("enable_depth", False)
        self.declare_parameter("auto_exposure", True)

        # Get parameters
        self.serial_number = self.get_parameter("serial_number").value
        self.frame_rate = self.get_parameter("frame_rate").value
        self.width = self.get_parameter("resolution.width").value
        self.height = self.get_parameter("resolution.height").value
        self.color_frame_id = self.get_parameter("frame_ids.color").value
        self.depth_frame_id = self.get_parameter("frame_ids.depth").value
        self.enable_depth = self.get_parameter("enable_depth").value
        self.auto_exposure = self.get_parameter("auto_exposure").value

        # Initialize RealSense
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.intr_params = None
        self.is_running = False
        self.bridge = CvBridge()

        # Publishers
        self.color_pub = self.create_publisher(Image, "color/image_raw", 10)
        self.color_info_pub = self.create_publisher(CameraInfo, "color/camera_info", 10)

        if self.enable_depth:
            self.depth_pub = self.create_publisher(Image, "depth/image_raw", 10)
            self.depth_info_pub = self.create_publisher(
                CameraInfo, "depth/camera_info", 10
            )

        # Service
        self.device_info_srv = self.create_service(
            Trigger, "get_device_info", self.get_device_info_callback
        )

        # Initialize camera
        if self.initialize_camera():
            # Timer for publishing
            self.timer = self.create_timer(1.0 / self.frame_rate, self.publish_frames)
            self.get_logger().info("RealSense bridge initialized successfully")
        else:
            self.get_logger().error("Failed to initialize RealSense camera")

    # ========================================================
    # Camera Initialization and Device Management
    # ========================================================
    def get_connected_devices(self):
        """Get list of connected RealSense devices"""
        context = rs.context()
        devices = context.query_devices()
        return [dev.get_info(rs.camera_info.serial_number) for dev in devices]

    def initialize_camera(self):
        """Initialize RealSense camera"""
        device_list = self.get_connected_devices()

        if not device_list:
            self.get_logger().error("No RealSense devices found!")
            return False

        # Use specified serial number or first device
        if self.serial_number:
            if self.serial_number not in device_list:
                self.get_logger().error(f"Device {self.serial_number} not found!")
                return False
            target_sn = self.serial_number
        else:
            target_sn = device_list[0]
            self.get_logger().info(f"Using first available device: {target_sn}")

        try:
            # Configure device
            self.config.enable_device(target_sn)

            # Enable color stream
            self.config.enable_stream(
                rs.stream.color,
                self.width,
                self.height,
                rs.format.bgr8,
                self.frame_rate,
            )

            # Enable depth stream if requested
            if self.enable_depth:
                self.config.enable_stream(
                    rs.stream.depth,
                    self.width,
                    self.height,
                    rs.format.z16,
                    self.frame_rate,
                )

            # Start pipeline
            profile = self.pipeline.start(self.config)

            # Get intrinsics
            color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
            self.intr_params = color_stream.get_intrinsics()

            self.is_running = True
            self.serial_number = target_sn
            self.get_logger().info(f"Camera {target_sn} started successfully")

            return True

        except Exception as e:
            self.get_logger().error(f"Failed to initialize camera: {e}")
            return False

    # ========================================================
    # Camera Info and Message Creation
    # ========================================================
    def create_camera_info(self, intrinsics, frame_id):
        """Create CameraInfo message from intrinsics"""
        camera_info = CameraInfo()
        camera_info.header.frame_id = frame_id
        camera_info.width = intrinsics.width
        camera_info.height = intrinsics.height

        # Intrinsic matrix K
        # fmt: off
        camera_info.k = [
            intrinsics.fx, 0.0, intrinsics.ppx,
            0.0, intrinsics.fy, intrinsics.ppy,
            0.0, 0.0, 1.0
        ]

        # Projection matrix P
        camera_info.p = [
            intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
            0.0, intrinsics.fy, intrinsics.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        # fmt: on

        # Distortion coefficients (assuming minimal distortion)
        camera_info.distortion_model = "plumb_bob"
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Rectification matrix R (identity)
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        return camera_info

    # ========================================================
    # Frame Publishing
    # ========================================================
    def publish_frames(self):
        """Publish camera frames"""
        if not self.is_running:
            return

        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            timestamp = self.get_clock().now().to_msg()

            # Publish color frame
            color_frame = frames.get_color_frame()
            if color_frame:
                color_image = np.asanyarray(color_frame.get_data())
                color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
                color_msg.header.stamp = timestamp
                color_msg.header.frame_id = self.color_frame_id
                self.color_pub.publish(color_msg)

                # Publish camera info
                if self.intr_params:
                    camera_info = self.create_camera_info(
                        self.intr_params, self.color_frame_id
                    )
                    camera_info.header.stamp = timestamp
                    self.color_info_pub.publish(camera_info)

            # Publish depth frame if enabled
            if self.enable_depth:
                depth_frame = frames.get_depth_frame()
                if depth_frame:
                    depth_image = np.asanyarray(depth_frame.get_data())
                    depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
                    depth_msg.header.stamp = timestamp
                    depth_msg.header.frame_id = self.depth_frame_id
                    self.depth_pub.publish(depth_msg)

                    # Publish depth camera info
                    depth_intr = (
                        depth_frame.profile.as_video_stream_profile().intrinsics
                    )
                    depth_info = self.create_camera_info(
                        depth_intr, self.depth_frame_id
                    )
                    depth_info.header.stamp = timestamp
                    self.depth_info_pub.publish(depth_info)

        except Exception as e:
            self.get_logger().error(f"Error publishing frames: {e}")

    # ========================================================
    # Service Callbacks
    # ========================================================
    def get_device_info_callback(self, request, response):
        """Service callback for device info"""
        if self.is_running:
            response.success = True
            response.message = (
                f"Device SN: {self.serial_number}, "
                f"Resolution: {self.width}x{self.height}, "
                f"FPS: {self.frame_rate}"
            )
        else:
            response.success = False
            response.message = "Camera is not running"
        return response

    # ========================================================
    # Cleanup
    # ========================================================
    def destroy_node(self):
        """Clean shutdown"""
        if self.is_running:
            try:
                self.pipeline.stop()
                self.get_logger().info("Camera stopped")
            except Exception as e:
                self.get_logger().error(f"Error stopping camera: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealsenseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

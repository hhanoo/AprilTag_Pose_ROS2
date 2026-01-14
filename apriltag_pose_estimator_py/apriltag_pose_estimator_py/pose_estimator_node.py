#!/usr/bin/env python3
"""
AprilTag Pose Estimator Node
Subscribes to camera images, detects AprilTags, and estimates multi-target poses
"""

import cv2
import dt_apriltags
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker, MarkerArray

from .multi_tag_pose_estimator import MultiTagPoseEstimator


class PoseEstimatorNode(Node):
    """AprilTag detection and pose estimation from camera images"""

    # ========================================================
    # Initialization
    # ========================================================
    def __init__(self):
        super().__init__("pose_estimator_node")

        # Declare parameters
        self.declare_parameter("marker_ids", [0, 1, 2])
        self.declare_parameter("base_marker_id", 1)
        self.declare_parameter("tag_size", 0.02778)
        self.declare_parameter("tag_family", "tagStandard41h12")
        self.declare_parameter("marker_offsets", [0.065, 0.0])
        # Point offsets as flat list (will be reshaped to Nx3)
        # fmt: off
        self.declare_parameter('point_offsets', [
            0.001, 0.001, -0.150, # Point 0
            0.0, 0.0, -0.300, # Point 1
            0.200, 0.200, -0.300, # Point 2
            -0.200, 0.200, -0.300, # Point 3
            -0.200, -0.200, -0.300, # Point 4
            0.200, -0.200, -0.300 # Point 5
        ])
        # fmt: on
        self.declare_parameter("camera_topic", "color/image_raw")
        self.declare_parameter("camera_info_topic", "color/camera_info")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("publish_visualization", True)
        self.declare_parameter("publish_detection_image", True)

        # Get parameters
        self.marker_ids = self.get_parameter("marker_ids").value
        self.base_marker_id = self.get_parameter("base_marker_id").value
        self.tag_size = self.get_parameter("tag_size").value
        self.tag_family = self.get_parameter("tag_family").value
        self.marker_offsets = self.get_parameter("marker_offsets").value
        # Reshape point_offsets from flat list to Nx3 array
        point_offsets_flat = self.get_parameter("point_offsets").value
        self.point_offsets = np.array(point_offsets_flat).reshape(-1, 3).tolist()
        self.camera_topic = self.get_parameter("camera_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.publish_viz = self.get_parameter("publish_visualization").value
        self.publish_det_img = self.get_parameter("publish_detection_image").value

        # CV Bridge
        self.bridge = CvBridge()

        # AprilTag detector
        self.detector = dt_apriltags.Detector(
            searchpath=["apriltags"],
            families=self.tag_family,
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

        # Pose estimator (initialized after camera info received)
        self.estimator = None
        self.camera_matrix = None
        self.dist_coeffs = np.zeros((5, 1))
        self.camera_info_received = False

        # Detection state tracking (for logging)
        self.last_detected_ids = []
        self.detection_active = False

        # Publishers
        self.target_poses_pub = self.create_publisher(PoseArray, "target_poses", 10)

        if self.publish_viz:
            self.viz_pub = self.create_publisher(
                MarkerArray, "visualization_markers", 10
            )

        if self.publish_det_img:
            self.detection_img_pub = self.create_publisher(Image, "detection_image", 10)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, self.camera_topic, self.image_callback, 10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, 10
        )

        self.get_logger().info("AprilTag pose estimator initialized")
        self.get_logger().info(f"Tag family: {self.tag_family}")
        self.get_logger().info(f"Marker IDs: {self.marker_ids}")
        self.get_logger().info(f"Base marker ID: {self.base_marker_id}")
        self.get_logger().info(f"Camera topic: {self.camera_topic}")

    # ========================================================
    # ROS2 Callbacks
    # ========================================================
    def camera_info_callback(self, msg):
        """Initialize camera parameters from camera_info"""
        if self.camera_info_received:
            return

        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d).reshape(-1, 1)

        # Initialize pose estimator
        self.estimator = MultiTagPoseEstimator(
            list_marker_ids=self.marker_ids,
            list_marker_offsets=self.marker_offsets,
            list_point_offsets=self.point_offsets,
            tag_size=self.tag_size,
            camera_matrix=self.camera_matrix,
            dist_coeffs=self.dist_coeffs,
            base_marker_id=self.base_marker_id,
        )

        self.camera_info_received = True
        self.get_logger().info(
            "Camera info received and AprilTag pose estimator initialized"
        )
        self.get_logger().info(f"Camera matrix:\n{self.camera_matrix}")

    def image_callback(self, msg):
        """Process camera image and detect AprilTags"""
        if not self.camera_info_received:
            self.get_logger().warn(
                "Camera info not received yet", throttle_duration_sec=2.0
            )
            return

        # Convert ROS image to OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Convert to grayscale for AprilTag detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        tags = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=[
                self.camera_matrix[0, 0],  # fx
                self.camera_matrix[1, 1],  # fy
                self.camera_matrix[0, 2],  # cx
                self.camera_matrix[1, 2],  # cy
            ],
            tag_size=self.tag_size,
        )

        if len(tags) == 0:
            # Log only when detection state changes
            if self.detection_active:
                self.get_logger().warn("No AprilTags detected")
                self.detection_active = False
                self.last_detected_ids = []
            return

        # Log detected tag IDs only when they change
        detected_ids = sorted([tag.tag_id for tag in tags])
        if detected_ids != self.last_detected_ids:
            self.get_logger().info(f"Detected {len(tags)} tags: {detected_ids}")
            self.last_detected_ids = detected_ids

        # Estimate poses using multi-tag estimator
        success, point_transforms, vis_image = self.estimator.detect(tags, cv_image)

        if not success:
            # Log only when detection state changes
            if self.detection_active:
                self.get_logger().warn(
                    f"Pose estimation failed. Required IDs: {self.marker_ids}, Detected IDs: {detected_ids}"
                )
                self.detection_active = False
            return

        # Publish target poses
        self._publish_target_poses(point_transforms, msg.header.stamp)

        # Publish visualization if enabled
        if self.publish_viz:
            self._publish_visualization(point_transforms, msg.header.stamp)

        # Publish detection image if enabled
        if self.publish_det_img and vis_image is not None:
            self._publish_detection_image(vis_image, msg.header.stamp)

    # ========================================================
    # Publishing Methods
    # ========================================================
    def _publish_target_poses(self, transforms, timestamp):
        """Publish target poses as PoseArray"""
        pose_array = PoseArray()
        pose_array.header.stamp = timestamp
        pose_array.header.frame_id = self.camera_frame

        for T in transforms:
            pose = Pose()
            pose.position.x = float(T[0, 3])
            pose.position.y = float(T[1, 3])
            pose.position.z = float(T[2, 3])

            # Convert rotation matrix to quaternion
            quat = self._rotation_matrix_to_quaternion(T[:3, :3])
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            pose_array.poses.append(pose)

        self.target_poses_pub.publish(pose_array)
        self.get_logger().debug(f"Published {len(transforms)} target poses")

    def _publish_visualization(self, transforms, timestamp):
        """Publish visualization markers"""
        marker_array = MarkerArray()

        for i, T in enumerate(transforms):
            marker = Marker()
            marker.header.stamp = timestamp
            marker.header.frame_id = self.camera_frame
            marker.ns = "target_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(T[0, 3])
            marker.pose.position.y = float(T[1, 3])
            marker.pose.position.z = float(T[2, 3])
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.scale.z = 0.02

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        self.viz_pub.publish(marker_array)

    def _publish_detection_image(self, image, timestamp):
        """Publish image with AprilTag detections drawn"""
        try:
            img_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            img_msg.header.stamp = timestamp
            img_msg.header.frame_id = self.camera_frame
            self.detection_img_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish detection image: {e}")

    # ========================================================
    # Utility Methods
    # ========================================================
    def _rotation_matrix_to_quaternion(self, R):
        """Convert 3x3 rotation matrix to quaternion [x, y, z, w]"""
        trace = np.trace(R)

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        return np.array([x, y, z, w])


def main(args=None):
    rclpy.init(args=args)
    node = PoseEstimatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

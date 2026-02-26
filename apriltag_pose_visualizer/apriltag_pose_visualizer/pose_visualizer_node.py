#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from apriltag_pose_estimator_msgs.msg import TagDetection


class PoseVisualizerNode(Node):
    def __init__(self):
        super().__init__("pose_visualizer_node")

        # --------------------------------------------------
        # Subscribers
        # --------------------------------------------------
        self.image_sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.image_callback,
            10,
        )

        self.pose_sub = self.create_subscription(
            TagDetection,
            "/pose_estimator_node/tag_detections",
            self.pose_callback,
            10,
        )

        # --------------------------------------------------
        # Internal state
        # --------------------------------------------------
        self.bridge = CvBridge()

        self.latest_image = None
        self.latest_pose = None

        # Timer for visualization loop
        self.timer = self.create_timer(0.03, self.visualize)  # ~30 Hz

        self.get_logger().info("AprilTag Pose Visualizer Node started")

    # ==================================================
    # Callbacks
    # ==================================================
    def image_callback(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed: {e}")

    def pose_callback(self, msg: TagDetection):
        self.latest_pose = msg

    # ==================================================
    # Visualization
    # ==================================================
    def visualize(self):
        # Need both image and pose
        if self.latest_image is None or self.latest_pose is None:
            return

        img = self.latest_image.copy()
        pose = self.latest_pose

        # --------------------------------------------------
        # Draw detection status
        # --------------------------------------------------
        status_text = "TAG DETECTED" if pose.tag_detected else "NO TAG"
        status_color = (0, 255, 0) if pose.tag_detected else (0, 0, 255)

        cv2.putText(
            img,
            status_text,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            status_color,
            2,
            cv2.LINE_AA,
        )

        if not pose.tag_detected:
            cv2.imshow("AprilTag Pose Visualizer", img)
            cv2.waitKey(1)
            return

        # --------------------------------------------------
        # Camera parameters
        # --------------------------------------------------
        camera_matrix = np.array(pose.camera_matrix, dtype=np.float64).reshape(3, 3)
        dist_coeffs = np.array(pose.dist_coeffs, dtype=np.float64).reshape(-1, 1)

        rvec = np.array(pose.rvec, dtype=np.float64).reshape(3, 1)
        tvec = np.array(pose.tvec, dtype=np.float64).reshape(3, 1)

        # ---------------------------------------------
        # Convert rvec to roll, pitch, yaw (degrees)
        # ---------------------------------------------
        R, _ = cv2.Rodrigues(rvec)  # rotation matrix

        sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0.0

        # Convert to degrees
        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)
        yaw_deg = np.degrees(yaw)

        # --------------------------------------------------
        # Draw coordinate frame
        # --------------------------------------------------
        try:
            cv2.drawFrameAxes(
                img,
                camera_matrix,
                dist_coeffs,
                rvec,
                tvec,
                pose.tag_size * 1.5,
            )
        except Exception as e:
            self.get_logger().warn(f"drawFrameAxes failed: {e}")

        # --------------------------------------------------
        # Draw pose text
        # --------------------------------------------------
        txt_y = 80
        for i, label in enumerate(["x", "y", "z"]):
            cv2.putText(
                img,
                f"t{label}: {tvec[i,0]:.4f} m",
                (20, txt_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            txt_y += 30

        for label, value in [
            ("roll", roll_deg),
            ("pitch", pitch_deg),
            ("yaw", yaw_deg),
        ]:
            cv2.putText(
                img,
                f"{label}: {value:.4f} deg",
                (20, txt_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            txt_y += 30

        # --------------------------------------------------
        # Show image
        # --------------------------------------------------
        cv2.imshow("AprilTag Pose Visualizer", img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = PoseVisualizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

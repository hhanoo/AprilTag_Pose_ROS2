#!/usr/bin/env python3
import cv2
import matplotlib.pyplot as plt
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

        # --------------------------------------------------
        # Two separate plots: tvec (position) and RPY (rotation)
        # --------------------------------------------------
        self.scatter_pts = []  # (x, y, z) in mm for tvec plot
        self.rpy_pts = []  # (roll, pitch, yaw) in deg for RPY plot
        self.max_pts = 500

        self.fig_tvec = plt.figure("tvec (position)")
        self.ax_tvec = self.fig_tvec.add_subplot(111, projection="3d")
        self.ax_tvec.set_xlabel("X (mm)")
        self.ax_tvec.set_ylabel("Y (mm)")
        self.ax_tvec.set_zlabel("Z (mm)")
        self.ax_tvec.set_title("tvec (position) - filtered")
        self.scatter_artist_tvec = None
        self.tvec_lims = (
            None  # (xmin, xmax, ymin, ymax, zmin, zmax) expand-only to avoid view jump
        )

        self.fig_rpy = plt.figure("RPY (rotation)")
        self.ax_rpy = self.fig_rpy.add_subplot(111, projection="3d")
        self.ax_rpy.set_xlabel("roll (deg)")
        self.ax_rpy.set_ylabel("pitch (deg)")
        self.ax_rpy.set_zlabel("yaw (deg)")
        self.ax_rpy.set_title("RPY (rotation) - filtered")
        self.scatter_artist_rpy = None
        self.rpy_lims = None  # expand-only axis limits

        self.stats_text_tvec = None  # text2D artist handle for tvec stats
        self.stats_text_rpy = None  # text2D artist handle for RPY stats

        self.timer_plot = self.create_timer(0.03, self.plot_pose_plots)

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

    def plot_pose_plots(self):
        # 1. Check if latest pose is available and tag is detected
        if self.latest_pose is None or not self.latest_pose.tag_detected:
            return
        pose = self.latest_pose

        # 2. Convert tvec to mm
        scale = 1000.0  # tvec [m] -> mm
        x = pose.tvec[0] * scale
        y = pose.tvec[1] * scale
        z = pose.tvec[2] * scale

        # 3. Convert rvec to roll/pitch/yaw (degrees)
        rvec = np.array(pose.rvec, dtype=np.float64).reshape(3, 1)
        R, _ = cv2.Rodrigues(rvec)
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        if sy >= 1e-6:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0.0
        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)
        yaw_deg = np.degrees(yaw)

        # 4. Add to rolling windows
        self.scatter_pts.append((x, y, z))
        self.rpy_pts.append((roll_deg, pitch_deg, yaw_deg))

        # 5. Keep only last N points
        if len(self.scatter_pts) > self.max_pts:
            self.scatter_pts.pop(0)
            self.rpy_pts.pop(0)

        # 6. Update tvec plot (expand-only limits)
        # 6.1 Remove existing scatter artist
        if self.scatter_artist_tvec is not None:
            self.scatter_artist_tvec.remove()
            self.scatter_artist_tvec = None
        # 6.2 Create new scatter artist
        if self.scatter_pts:
            xs, ys, zs = zip(*self.scatter_pts)
            self.scatter_artist_tvec = self.ax_tvec.scatter(
                xs, ys, zs, s=4, c="steelblue", alpha=0.8
            )
            # 6.3 Update limits expand-only: never shrink so removing old points doesn't move the plot
            # 6.3.1 Calculate limits
            xmin, xmax = min(xs), max(xs)
            ymin, ymax = min(ys), max(ys)
            zmin, zmax = min(zs), max(zs)
            # 6.3.2 Calculate margin
            margin = 0.005
            dx = max((xmax - xmin) * margin, 0.05)
            dy = max((ymax - ymin) * margin, 0.05)
            dz = max((zmax - zmin) * margin, 0.05)
            # 6.3.3 Check if limits are None
            if self.tvec_lims is None:
                self.tvec_lims = (
                    xmin - dx,
                    xmax + dx,
                    ymin - dy,
                    ymax + dy,
                    zmin - dz,
                    zmax + dz,
                )
            else:
                self.tvec_lims = (
                    min(self.tvec_lims[0], xmin - dx),
                    max(self.tvec_lims[1], xmax + dx),
                    min(self.tvec_lims[2], ymin - dy),
                    max(self.tvec_lims[3], ymax + dy),
                    min(self.tvec_lims[4], zmin - dz),
                    max(self.tvec_lims[5], zmax + dz),
                )
            # 6.3.4 Apply limits
            self.ax_tvec.set_xlim(self.tvec_lims[0], self.tvec_lims[1])
            self.ax_tvec.set_ylim(self.tvec_lims[2], self.tvec_lims[3])
            self.ax_tvec.set_zlim(self.tvec_lims[4], self.tvec_lims[5])
            # 6.4 Update statistics
            # 6.4.1 Convert scatter points to numpy array
            pts_arr = np.array(self.scatter_pts)
            # 6.4.2 Calculate mean and std
            mean = pts_arr.mean(axis=0)
            std = pts_arr.std(axis=0)
            stats_str = (
                f"μ x={mean[0]:.2f} y={mean[1]:.2f} z={mean[2]:.2f}\n"
                f"σ x={std[0]:.2f}  y={std[1]:.2f}  z={std[2]:.2f}"
            )
            # 6.4.3 Remove existing stats text
            if self.stats_text_tvec is not None:
                self.stats_text_tvec.remove()
            # 6.4.4 Create new stats text
            self.stats_text_tvec = self.ax_tvec.text2D(
                0.02,
                0.97,
                stats_str,
                transform=self.ax_tvec.transAxes,
                verticalalignment="top",
                fontsize=8,
                fontfamily="monospace",
                color="white",
                bbox=dict(boxstyle="round,pad=0.3", facecolor="black", alpha=0.5),
            )
        # 6.5 Refresh plot
        self.fig_tvec.canvas.draw()

        # 7. Update RPY plot (expand-only limits)
        # 7.1 Remove existing scatter artist
        if self.scatter_artist_rpy is not None:
            self.scatter_artist_rpy.remove()
            self.scatter_artist_rpy = None
        # 7.2 Create new scatter artist
        if self.rpy_pts:
            rolls, pitches, yaws = zip(*self.rpy_pts)
            self.scatter_artist_rpy = self.ax_rpy.scatter(
                rolls, pitches, yaws, s=4, c="coral", alpha=0.8
            )
            # 7.3 Update limits expand-only: never shrink so removing old points doesn't move the plot
            # 7.3.1 Calculate limits
            rxmin, rxmax = min(rolls), max(rolls)
            rymin, rymax = min(pitches), max(pitches)
            rzmin, rzmax = min(yaws), max(yaws)
            # 7.3.2 Calculate margin
            dr = 0.005
            rdx = max((rxmax - rxmin) * dr, 0.05)
            rdy = max((rymax - rymin) * dr, 0.05)
            rdz = max((rzmax - rzmin) * dr, 0.05)
            # 7.3.3 Check if limits are None
            if self.rpy_lims is None:
                self.rpy_lims = (
                    rxmin - rdx,
                    rxmax + rdx,
                    rymin - rdy,
                    rymax + rdy,
                    rzmin - rdz,
                    rzmax + rdz,
                )
            else:
                self.rpy_lims = (
                    min(self.rpy_lims[0], rxmin - rdx),
                    max(self.rpy_lims[1], rxmax + rdx),
                    min(self.rpy_lims[2], rymin - rdy),
                    max(self.rpy_lims[3], rymax + rdy),
                    min(self.rpy_lims[4], rzmin - rdz),
                    max(self.rpy_lims[5], rzmax + rdz),
                )
            # 7.3.4 Apply limits
            self.ax_rpy.set_xlim(self.rpy_lims[0], self.rpy_lims[1])
            self.ax_rpy.set_ylim(self.rpy_lims[2], self.rpy_lims[3])
            self.ax_rpy.set_zlim(self.rpy_lims[4], self.rpy_lims[5])
            # 7.4 Update statistics
            # 7.4.1 Convert rpy points to numpy array
            rpy_arr = np.array(self.rpy_pts)
            # 7.4.2 Calculate mean and std
            mean_r = rpy_arr.mean(axis=0)
            std_r = rpy_arr.std(axis=0)
            rpy_stats_str = (
                f"μ r={mean_r[0]:.2f} p={mean_r[1]:.2f} y={mean_r[2]:.2f}\n"
                f"σ r={std_r[0]:.2f}  p={std_r[1]:.2f}  y={std_r[2]:.2f}"
            )
            # 7.4.3 Remove existing stats text
            if self.stats_text_rpy is not None:
                self.stats_text_rpy.remove()
            # 7.4.4 Create new stats text
            self.stats_text_rpy = self.ax_rpy.text2D(
                0.02,
                0.97,
                rpy_stats_str,
                transform=self.ax_rpy.transAxes,
                verticalalignment="top",
                fontsize=8,
                fontfamily="monospace",
                color="white",
                bbox=dict(boxstyle="round,pad=0.3", facecolor="black", alpha=0.5),
            )
        # 7.5 Refresh plot
        self.fig_rpy.canvas.draw()

        # 8. Refresh plots
        plt.pause(0.01)


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

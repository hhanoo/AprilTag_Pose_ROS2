import copy

import cv2
import numpy as np


class MultiTagPoseEstimator:
    """
    Multi-AprilTag pose estimator for calculating target point poses
    Uses multiple AprilTags to improve pose estimation accuracy
    """

    # ========================================================
    # Initialization
    # ========================================================
    def __init__(
        self,
        list_marker_ids,
        list_marker_offsets,
        list_point_offsets,
        tag_size,
        camera_matrix,
        dist_coeffs,
        base_marker_id=None,
    ):
        """
        Initialize the MultiTagObjectPoseEstimator point.

        Parameters:
            list_marker_ids (list): List of tag IDs to be used
            list_marker_offsets (list): X, Y distances between markers
            list_point_offsets (list): List of point position offsets in Nx3 array format [[x1, y1, z1], [x2, y2, z2], ...]
            tag_size (float): Actual size of the tag (unit: m)
            camera_matrix (np.ndarray): Camera intrinsic parameter matrix
            dist_coeffs (np.ndarray): Camera distortion coefficients
            base_marker_id (int, optional): ID of the base marker
        """
        self.marker_ids = copy.deepcopy(list_marker_ids)
        self.marker_offset_x, self.marker_offset_y = list_marker_offsets
        self.point_offsets = copy.deepcopy(list_point_offsets)
        self.point_count = len(list_point_offsets)
        self.tag_size = tag_size
        self.camera_matrix = copy.deepcopy(camera_matrix)
        self.dist_coeffs = copy.deepcopy(dist_coeffs)
        self.base_marker_id = (
            base_marker_id if base_marker_id is not None else self.marker_ids[0]
        )

    # ========================================================
    # Coordinate Transformation Utilities
    # ========================================================
    def rvec_tvec_to_matrix(self, r, t):
        """
        Convert rotation vector and translation vector to a 4x4 homogeneous transformation matrix.

        Parameters:
            r (np.ndarray): Rotation vector of shape (3, 1), in Rodrigues format
            t (np.ndarray): Translation vector of shape (3, 1)

        Returns:
            T (np.ndarray): 4x4 homogeneous transformation matrix (rotation + translation)
        """
        R = cv2.Rodrigues(r)[0]
        T = np.array(
            [
                [R[0, 0], R[0, 1], R[0, 2], t[0][0]],  # Row 1
                [R[1, 0], R[1, 1], R[1, 2], t[1][0]],  # Row 2
                [R[2, 0], R[2, 1], R[2, 2], t[2][0]],  # Row 3
                [0.0, 0.0, 0.0, 1.0],  # Row 4 (homogeneous bottom row)
            ],
            dtype=np.float32,
        )
        return T

    # ========================================================
    # AprilTag Detection and Pose Estimation
    # ========================================================
    def detect(self, tags, img):
        """
        Detect markers in the image and estimate point poses relative to the base marker.

        Parameters:
            tags (list): List of detected AprilTag objects
            img (np.ndarray): Current image (for drawing purposes)

        Returns:
            success (bool): True if all required markers are detected and pose estimated
            point_T (list of np.ndarray): List of 4x4 transformation matrices for each point
            img (np.ndarray): Image with visualization (axes and projected points)
        """
        # 1. Make a dictionary for fast lookup: tag_id → tag
        tag_dict = {tag.tag_id: tag for tag in tags}

        # 2. Check if all required marker IDs are found in the detected tags
        if not all(marker_id in tag_dict for marker_id in self.marker_ids):
            return False, [], img  # Not all required markers detected

        # 3. Prepare 3D (world) and 2D (image) correspondence arrays
        num_markers = len(self.marker_ids)
        pt3D = np.zeros([num_markers * 4, 3])  # 4 corners per tag
        pt2D = np.zeros([num_markers * 4, 2])
        a = self.tag_size / 2.0  # Half tag size
        ox, oy = self.marker_offset_x, self.marker_offset_y
        base_index = self.marker_ids.index(
            self.base_marker_id
        )  # base marker offset index

        # 4. Fill in pt3D and pt2D by following marker ID order
        for i, marker_id in enumerate(self.marker_ids):
            dx = ox * (i - base_index)
            dy = oy * (i - base_index)

            # 4.1 Define world coordinates for the four corners of each marker
            pt3D[i * 4 + 0] = [-a + dx, a + dy, 0.0]  # top-left
            pt3D[i * 4 + 1] = [a + dx, a + dy, 0.0]  # top-right
            pt3D[i * 4 + 2] = [a + dx, -a + dy, 0.0]  # bottom-right
            pt3D[i * 4 + 3] = [-a + dx, -a + dy, 0.0]  # bottom-left

            # 4.2 Use the tag's detected corners (2D image points)
            corners = tag_dict[marker_id].corners
            pt2D[i * 4 : (i + 1) * 4] = corners

        # 5. Estimate camera pose using solvePnP
        success, rvec, tvec = cv2.solvePnP(
            pt3D, pt2D, self.camera_matrix, self.dist_coeffs
        )
        if not success:
            return False, [], img  # pose estimation failed

        # 6. Convert to 4x4 transformation matrix
        T = self.rvec_tvec_to_matrix(rvec, tvec)
        marker_rotation = T[:3, :3]
        marker_position = T[:3, 3]

        # 7. Compute point poses and project them onto the image
        point_T = []
        for i, offset in enumerate(self.point_offsets):
            offset_vec = np.array(offset, dtype=np.float32)  # shape (3,)
            obj_pos = marker_position + marker_rotation @ offset_vec

            # 7.1 Project the 3D point position into image plane
            pos_cam = obj_pos.astype(np.float32)
            img_point = self.camera_matrix @ pos_cam
            img_point /= img_point[2]

            # 7.2 Draw white circle with black border
            img = cv2.circle(
                img, (int(img_point[0]), int(img_point[1])), 5, (255, 255, 255), -1
            )  # Fill white circle
            img = cv2.circle(
                img, (int(img_point[0]), int(img_point[1])), 5, (0, 0, 0), 2
            )  # Draw black border

            # 7.3 Store the transformation matrix for this point
            obj_T = T.copy()
            obj_T[:3, 3] = obj_pos
            point_T.append(obj_T)

        # 8. Visualize the base marker pose using axes
        img = cv2.drawFrameAxes(
            img,
            self.camera_matrix,
            self.dist_coeffs,
            marker_rotation.astype(np.float64),
            marker_position.reshape(3, 1).astype(np.float64),
            self.tag_size,
        )

        return True, point_T, img

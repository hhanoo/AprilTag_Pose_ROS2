# AprilTag pose data for OpenCV drawFrameAxes

float64[9] camera_matrix   # Row-major 3x3 intrinsic matrix
float64[]  dist_coeffs     # Distortion coefficients (OpenCV order)
float64[3] rvec            # Rodrigues rotation vector
float64[3] tvec            # Translation vector (meters)
float64    tag_size        # Tag size (meters)
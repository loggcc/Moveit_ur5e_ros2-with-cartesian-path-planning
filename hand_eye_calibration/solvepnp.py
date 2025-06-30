import numpy as np
import cv2
import pyrealsense2 as rs

CHECKERBOARD = (9, 6)
SQUARE_SIZE = 0.025  # meters


#camera intirnsic matix for realsense d455i
camera_matrix = np.array([
    [387.3082, 0, 323.28192],
    [0, 386.88937, 237.11705],
    [0, 0, 1]
], dtype=np.float32)

dist_coeffs = np.array([-0.05379387, 0.06378421, -0.00086663, 0.00019662, -0.02069664], dtype=np.float32)




# Prepare 3D object points
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

# RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

for _ in range(30):
    frames = pipeline.wait_for_frames()

frames = pipeline.wait_for_frames()
color_frame = frames.get_color_frame()
image = np.asanyarray(color_frame.get_data())
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

pipeline.stop()

ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)


if ret:

    # Refine corners
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    # Estimate pose
    success, rvec, tvec = cv2.solvePnP(objp, corners_refined, camera_matrix, dist_coeffs)
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()

    print("4x4 transformation matrix (camera_T_checkerboard):")
    print(T)

    # Visualize detected corners
    cv2.drawChessboardCorners(image, CHECKERBOARD, corners_refined, ret)

    # Draw coordinate axes (0.05m long)
    axis = np.float32([[0.05,0,0], [0,0.05,0], [0,0,-0.05]]).reshape(-1,3)
    imgpts, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)
    corner = tuple(corners_refined[0].ravel().astype(int))
    image = cv2.line(image, corner, tuple(imgpts[0].ravel().astype(int)), (255,0,0), 2)
    image = cv2.line(image, corner, tuple(imgpts[1].ravel().astype(int)), (0,255,0), 2)
    image = cv2.line(image, corner, tuple(imgpts[2].ravel().astype(int)), (0,0,255), 2)
else:
    print("Checkerboard not found.")

cv2.imshow("Checkerboard with Pose (or Raw Frame)", image)
cv2.waitKey(0)
cv2.destroyAllWindows()

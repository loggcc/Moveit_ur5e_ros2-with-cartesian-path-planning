import pyrealsense2 as rs
import numpy as np

# Configure pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Wait for frames and get intrinsics
for i in range(30):  # warm up
    frames = pipeline.wait_for_frames()

color_frame = frames.get_color_frame()
intr = color_frame.profile.as_video_stream_profile().get_intrinsics()

# Camera matrix
camera_matrix = np.array([
    [intr.fx, 0, intr.ppx],
    [0, intr.fy, intr.ppy],
    [0,      0,       1]
], dtype=np.float32)

# Distortion coefficients
dist_coeffs = np.array(intr.coeffs, dtype=np.float32)

print("Camera matrix (intrinsics):\n", camera_matrix)
print("Distortion coefficients:\n", dist_coeffs)

# Save for later use
np.savez('calib.npz', cmx=camera_matrix, dist=dist_coeffs)

# Stop streaming
pipeline.stop()


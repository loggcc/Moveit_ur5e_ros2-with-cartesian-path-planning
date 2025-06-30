#!/usr/bin/env python3
import pyrealsense2 as rs
import cv2
import numpy as np

# Configure and start RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

print("Press 's' to save frame and exit. Press 'ESC' to exit without saving.")

try:
    while True:
        # Wait for a coherent color frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Show the image
        cv2.imshow('RealSense Color Stream', color_image)
        key = cv2.waitKey(1)

        if key == ord('s'):  # exit
            break
        elif key == 27:  # ESC key to exit
            print("Exited without saving.")
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()


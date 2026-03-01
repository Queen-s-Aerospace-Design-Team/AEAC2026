#!/usr/bin/env python3
"""
record_svo.py - Record SVO2 files from ZED camera
Usage: python3 record_svo.py [output_name]
  - output_name: optional, defaults to timestamp-based name
  - Press 'q' to stop recording
"""

import pyzed.sl as sl
import cv2
import sys
import os
from datetime import datetime


def main():
    # Output filename
    if len(sys.argv) > 1:
        output_name = sys.argv[1]
    else:
        output_name = datetime.now().strftime("recording_%Y%m%d_%H%M%S")

    if not output_name.endswith(".svo2"):
        output_name += ".svo2"

    output_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings")
    os.makedirs(output_dir, exist_ok=True)
    output_path = os.path.join(output_dir, output_name)

    # Initialize camera
    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1200  # good balance of quality/performance on Jetson
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.NONE  # best depth quality; use ULTRA if too slow

    print("Opening camera...")
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open camera: {err}")
        return 1

    # Start recording
    recording_params = sl.RecordingParameters(output_path, sl.SVO_COMPRESSION_MODE.LOSSLESS)    
    err = zed.enable_recording(recording_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to start recording: {err}")
        zed.close()
        return 1

    print(f"Recording to: {output_path}")
    print("Press 'q' in the preview window to stop recording.")

    image = sl.Mat()
    frame_count = 0

    try:
        while True:
            if zed.grab() == sl.ERROR_CODE.SUCCESS:
                frame_count += 1

                # Show preview (left eye)
                zed.retrieve_image(image, sl.VIEW.LEFT)
                frame = image.get_data()

                # Add recording indicator
                cv2.circle(frame, (30, 30), 10, (0, 0, 255), -1)  # red dot
                cv2.putText(frame, f"REC  Frame: {frame_count}", (50, 38),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                cv2.imshow("ZED Recording (press q to stop)", frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        print("\nStopping via keyboard interrupt...")

    # Cleanup
    zed.disable_recording()
    zed.close()
    cv2.destroyAllWindows()

    print(f"\nDone! Recorded {frame_count} frames.")
    print(f"Saved to: {output_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

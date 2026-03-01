#!/usr/bin/env python3
"""
extract_training_frames.py — Run on Jetson after recording an SVO
Extracts every Nth frame as lossless PNG for Roboflow upload.

Usage:
    python extract_training_frames.py --svo flight_01.svo2
    python extract_training_frames.py --svo flight_01.svo2 --out ./training_frames --every_n 15

Defaults:
    --every_n 15  → 2 FPS from 30 FPS source (good default for diverse viewpoints)
    --every_n 30  → 1 FPS
    --every_n  5  → 6 FPS
    --every_n  1  → every frame
"""

import pyzed.sl as sl
import cv2
import os
import argparse


def main():
    parser = argparse.ArgumentParser(
        description="Extract training frames from SVO as lossless PNG"
    )
    parser.add_argument("--svo", required=True, help="Path to .svo2 file")
    parser.add_argument("--out", default="./training_frames", help="Output directory")
    parser.add_argument(
        "--every_n",
        type=int,
        default=15,
        help="Extract every Nth frame (default: 15 = 2 FPS from 30 FPS)",
    )
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)

    # Open SVO
    zed = sl.Camera()
    params = sl.InitParameters()
    params.set_from_svo_file(args.svo)
    params.svo_real_time_mode = False  # Process as fast as possible

    status = zed.open(params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Error opening SVO: {status}")
        exit(1)

    total_frames = zed.get_svo_number_of_frames()
    expected = total_frames // args.every_n
    print(f"SVO: {total_frames} frames. Extracting every {args.every_n}th → ~{expected} images")

    image = sl.Mat()
    frame_num = 0
    saved = 0

    while zed.grab() == sl.ERROR_CODE.SUCCESS:
        if frame_num % args.every_n == 0:
            zed.retrieve_image(image, sl.VIEW.LEFT)  # Left camera only
            img_np = image.get_data()[:, :, :3]  # BGRA → BGR
            cv2.imwrite(os.path.join(args.out, f"frame_{saved:05d}.png"), img_np)
            saved += 1

            if saved % 50 == 0:
                print(f"  Saved {saved} frames...")

        frame_num += 1

    zed.close()
    print(f"\nDone. {saved} frames saved to {args.out}/")


if __name__ == "__main__":
    main()

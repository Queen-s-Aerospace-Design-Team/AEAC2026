#!/usr/bin/env python3
"""
validate_svo.py — Visual validation of YOLO detections on SVO recordings
Run on Jetson after exporting TRT engine, BEFORE integrating into flight code.

Replays an SVO, runs YOLO on each frame, draws bounding boxes with class name,
confidence, and optionally depth. Can show live, save to MP4, log to CSV, or
compare two engines side-by-side.

Usage examples:
    # Quick visual check (with monitor):
    python validate_svo.py --svo test_flight.svo2 --engine yolo11n_640.engine --show

    # Save MP4 for laptop review (headless):
    python validate_svo.py --svo test_flight.svo2 --engine yolo11n_640.engine --save results.mp4

    # With depth overlay:
    python validate_svo.py --svo test_flight.svo2 --engine yolo11n_640.engine --save results.mp4 --depth

    # Compare 640 vs 1280 engines side-by-side:
    python validate_svo.py --svo test_flight.svo2 --engine yolo11n_640.engine --engine2 yolo11n_1280.engine --save compare.mp4

    # Log detections to CSV for analysis:
    python validate_svo.py --svo test_flight.svo2 --engine yolo11n_640.engine --log detections.csv

    # Full combo (save video + CSV + depth):
    python validate_svo.py --svo test_flight.svo2 --engine yolo11n_640.engine --save results.mp4 --log detections.csv --depth

What to look for:
    ✓ Are circles detected with > 70% confidence?
    ✓ Are corners detected consistently across frames?
    ✗ Any false positives on blank walls / sky / ground?
    ✗ Any missed targets (especially at distance or in shadow)?
    ✗ Does the 640 engine miss things the 1280 catches?
    → If yes: consider using 1280 for scoring thread or retraining with more data.
"""

import pyzed.sl as sl
import cv2
import numpy as np
import argparse
import csv
import time
from ultralytics import YOLO


# --- Configuration ---
# Update these to match your Roboflow project class names/order
CLASS_NAMES = {0: "circle", 1: "corner"}
CLASS_COLORS = {0: (0, 255, 255), 1: (0, 255, 0)}  # Yellow for circles, green for corners


def parse_args():
    parser = argparse.ArgumentParser(
        description="Visual validation of YOLO detections on SVO recordings"
    )
    parser.add_argument("--svo", required=True, help="Path to .svo2 file")
    parser.add_argument("--engine", required=True, help="Path to .engine file (TRT)")
    parser.add_argument(
        "--engine2", default=None, help="Optional 2nd engine for side-by-side comparison"
    )
    parser.add_argument("--imgsz", type=int, default=640, help="Inference resolution (640 or 1280)")
    parser.add_argument(
        "--imgsz2", type=int, default=1280, help="Inference resolution for 2nd engine"
    )
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold")
    parser.add_argument("--show", action="store_true", help="Show live preview window")
    parser.add_argument("--save", default=None, help="Save output to MP4 file")
    parser.add_argument("--log", default=None, help="Save detections to CSV file")
    parser.add_argument(
        "--depth", action="store_true", help="Enable depth and show distance in bbox label"
    )
    return parser.parse_args()


def draw_detections(img, results, depth_map=None, frame_num=0, csv_writer=None):
    """Draw bounding boxes with class, confidence, and optional depth."""
    for box in results[0].boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cls = int(box.cls[0])
        conf = float(box.conf[0])
        color = CLASS_COLORS.get(cls, (255, 255, 255))
        name = CLASS_NAMES.get(cls, f"cls{cls}")

        # Get depth at bbox center
        depth_str = ""
        depth_val = None
        if depth_map is not None:
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            # Clamp to image bounds
            cy = min(cy, depth_map.shape[0] - 1)
            cx = min(cx, depth_map.shape[1] - 1)
            depth_val = depth_map[cy, cx]
            if np.isfinite(depth_val):
                depth_str = f" {depth_val:.1f}m"

        label = f"{name} {conf:.0%}{depth_str}"

        # Draw box
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

        # Draw label background + text
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
        cv2.rectangle(img, (x1, y1 - th - 6), (x1 + tw, y1), color, -1)
        cv2.putText(img, label, (x1, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)

        # CSV logging
        if csv_writer:
            csv_writer.writerow([
                frame_num,
                name,
                f"{conf:.3f}",
                x1, y1, x2, y2,
                f"{depth_val:.2f}" if depth_val is not None and np.isfinite(depth_val) else "",
            ])

    return img


def main():
    args = parse_args()

    if not args.show and not args.save and not args.log:
        print("Warning: no output specified. Use --show, --save, or --log.")
        print("Running anyway (will print progress)...")

    # --- Open SVO ---
    zed = sl.Camera()
    params = sl.InitParameters()
    params.set_from_svo_file(args.svo)
    params.svo_real_time_mode = False
    if args.depth:
        params.depth_mode = sl.DEPTH_MODE.NEURAL_LIGHT

    status = zed.open(params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Error opening SVO: {status}")
        exit(1)

    total_frames = zed.get_svo_number_of_frames()
    res = zed.get_camera_information().camera_configuration.resolution
    W, H = res.width, res.height
    print(f"SVO: {total_frames} frames, {W}x{H}")

    # --- Load model(s) ---
    print(f"Loading engine: {args.engine}")
    model = YOLO(args.engine, task="detect")
    model2 = None
    if args.engine2:
        print(f"Loading engine 2: {args.engine2}")
        model2 = YOLO(args.engine2, task="detect")

    # --- Output setup ---
    writer = None
    out_w = W * 2 if model2 else W  # Side-by-side doubles width
    if args.save:
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(args.save, fourcc, 30, (out_w, H))
        print(f"Saving video to {args.save}")

    csv_file = None
    csv_writer = None
    if args.log:
        csv_file = open(args.log, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["frame", "class", "confidence", "x1", "y1", "x2", "y2", "depth_m"])
        print(f"Logging detections to {args.log}")

    # --- Processing loop ---
    image = sl.Mat()
    depth = sl.Mat() if args.depth else None
    frame_num = 0

    print("Processing...")
    start = time.time()

    while zed.grab() == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image, sl.VIEW.LEFT)
        frame = image.get_data()[:, :, :3].copy()  # BGRA → BGR

        depth_map = None
        if args.depth and depth is not None:
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            depth_map = depth.get_data()

        # Run model 1
        results1 = model(frame, imgsz=args.imgsz, conf=args.conf, verbose=False)
        vis1 = draw_detections(frame.copy(), results1, depth_map, frame_num, csv_writer)

        # Optional: run model 2 for side-by-side
        if model2:
            results2 = model2(frame, imgsz=args.imgsz2, conf=args.conf, verbose=False)
            vis2 = draw_detections(frame.copy(), results2, depth_map, frame_num)

            # Add engine labels
            cv2.putText(
                vis1, f"Engine 1 ({args.imgsz})",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2,
            )
            cv2.putText(
                vis2, f"Engine 2 ({args.imgsz2})",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2,
            )
            vis = np.hstack([vis1, vis2])
        else:
            vis = vis1

        # Frame counter overlay
        cv2.putText(
            vis, f"Frame {frame_num}/{total_frames}",
            (10, H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1,
        )

        # Show live preview
        if args.show:
            cv2.imshow("SVO Validation", vis)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord(" "):
                cv2.waitKey(0)  # Space = pause, any key = resume

        # Write to video
        if writer:
            writer.write(vis)

        frame_num += 1
        if frame_num % 100 == 0:
            elapsed = time.time() - start
            fps = frame_num / elapsed
            pct = frame_num * 100 // total_frames
            print(f"  Frame {frame_num}/{total_frames} ({fps:.1f} FPS, {pct}%)")

    # --- Cleanup ---
    zed.close()
    if writer:
        writer.release()
    if csv_file:
        csv_file.close()
    if args.show:
        cv2.destroyAllWindows()

    elapsed = time.time() - start
    print(f"\nDone. {frame_num} frames in {elapsed:.1f}s ({frame_num / elapsed:.1f} FPS)")
    if args.save:
        print(f"Video saved to {args.save}")
    if args.log:
        print(f"Detections logged to {args.log}")


if __name__ == "__main__":
    main()

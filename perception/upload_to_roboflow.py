#!/usr/bin/env python3
"""
upload_to_roboflow.py — Run on PC after transferring PNGs from Jetson
Uploads curated training frames to a Roboflow project for labeling.

Prerequisites:
    pip install roboflow

Usage:
    python upload_to_roboflow.py --folder ./training_frames --api_key YOUR_KEY --workspace your-ws --project circle-corner-detection

Alternative: drag & drop the folder in Roboflow web UI (fine for < 1000 images)
"""

import os
import argparse
from roboflow import Roboflow


def main():
    parser = argparse.ArgumentParser(
        description="Upload training frames to Roboflow"
    )
    parser.add_argument("--folder", default="./training_frames", help="Folder with PNG images")
    parser.add_argument("--api_key", required=True, help="Roboflow API key (Settings → API Key)")
    parser.add_argument("--workspace", required=True, help="Roboflow workspace name")
    parser.add_argument("--project", default="circle-corner-detection", help="Roboflow project name")
    args = parser.parse_args()

    rf = Roboflow(api_key=args.api_key)
    project = rf.workspace(args.workspace).project(args.project)

    images = sorted([f for f in os.listdir(args.folder) if f.endswith(".png")])
    print(f"Uploading {len(images)} images from {args.folder}...")

    for i, img in enumerate(images):
        project.upload(os.path.join(args.folder, img))
        if (i + 1) % 50 == 0:
            print(f"  {i + 1}/{len(images)}")

    print(f"\nDone. {len(images)} images uploaded.")
    print("Open Roboflow Annotate to start labeling.")


if __name__ == "__main__":
    main()

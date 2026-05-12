#!/usr/bin/env python3
"""
detector_worker.py — Run on Jetson. Watches segments/ for new SVO files and
emits one JSONL per segment matching the contract in
zed_positional_measurement/models.py:PaperDetection.from_dict.

Uses RF-DETR via the Roboflow Inference SDK. The Inference container handles
letterbox + TensorRT compilation transparently; we just hand it BGR frames
and convert (cx, cy, w, h) → xyxy before writing the JSONL contract.

Requires:
    pip install inference

    export ROBOFLOW_API_KEY="rf_xxxxxxxxx"   # your private API key

Usage:
    python3 detector_worker.py \\
        --segments-dir runs/<session_id>/segments \\
        --out-dir      runs/<session_id>/detections \\
        --model-id     aeac2026/2            # default; override if needed
"""

import argparse
import json
import os
import time
from pathlib import Path

import pyzed.sl as sl
from inference import get_model


# class_id (Roboflow class index) → (label, color) for the SDK contract.
# TODO: confirm these IDs match your Roboflow project. Check Roboflow → Classes.
CLASS_MAP: dict[int, tuple[str, str]] = {
    0: ("paper", "red"),
    1: ("paper", "blue"),
    2: ("paper", "yellow"),
}

# Fallback: map by class string name if `class_id` is missing on a prediction.
# Keys must match the class names you set in Roboflow Annotate.
CLASS_NAME_MAP: dict[str, tuple[str, str]] = {
    "paper_red":    ("paper", "red"),
    "paper_blue":   ("paper", "blue"),
    "paper_yellow": ("paper", "yellow"),
}


def _resolve_class(pred) -> tuple[str, str] | None:
    """Map a Roboflow prediction → (label, color) for the SDK. Returns None if
    the prediction's class isn't in either map (caller should skip it)."""
    cls_id = getattr(pred, "class_id", None)
    if cls_id is not None and int(cls_id) in CLASS_MAP:
        return CLASS_MAP[int(cls_id)]
    cls_name = getattr(pred, "class_name", None) or getattr(pred, "class", None)
    if cls_name in CLASS_NAME_MAP:
        return CLASS_NAME_MAP[cls_name]
    return None


def process_segment(svo_path: Path, out_path: Path, model, conf: float) -> int:
    """Process one SVO segment → one JSONL file. Returns frames processed."""
    zed = sl.Camera()
    params = sl.InitParameters()
    params.set_from_svo_file(str(svo_path))
    params.svo_real_time_mode = False
    if zed.open(params) != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"failed to open {svo_path}")

    image = sl.Mat()
    local_frame_idx = 0

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w") as handle:
        while zed.grab() == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT)
            img_bgr = image.get_data()[:, :, :3]  # BGRA → BGR

            # Roboflow Inference letterboxes to 640 internally and returns
            # predictions in the input image's native pixel coordinates.
            result = model.infer(img_bgr, confidence=conf)[0]

            for pred in result.predictions:
                resolved = _resolve_class(pred)
                if resolved is None:
                    continue
                label, color = resolved

                # Roboflow gives (cx, cy, w, h) in native coords. → xyxy.
                cx, cy, w, h = pred.x, pred.y, pred.width, pred.height
                x1 = int(round(cx - w / 2))
                y1 = int(round(cy - h / 2))
                x2 = int(round(cx + w / 2))
                y2 = int(round(cy + h / 2))

                handle.write(json.dumps({
                    "frame_idx": local_frame_idx,
                    "bbox_xyxy": [x1, y1, x2, y2],
                    "label": label,
                    "color": color,
                    "confidence": float(pred.confidence),
                }) + "\n")
            local_frame_idx += 1

    zed.close()
    return local_frame_idx


def _next_segment_id(seg_id: str) -> str:
    """segment-00001 → segment-00002.svo2 (the file to check for existence)."""
    return f"segment-{int(seg_id.split('-')[1]) + 1:05d}.svo2"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--segments-dir", required=True, type=Path)
    ap.add_argument("--out-dir",      required=True, type=Path)
    ap.add_argument("--model-id", default="aeac2026/2",
                    help="Roboflow model id, e.g. workspace/project/version "
                         "(default: aeac2026/2)")
    ap.add_argument("--api-key", default=os.environ.get("ROBOFLOW_API_KEY"),
                    help="Roboflow API key (default: $ROBOFLOW_API_KEY)")
    ap.add_argument("--conf", type=float, default=0.25,
                    help="Roboflow confidence threshold (default: 0.25)")
    ap.add_argument("--poll-seconds", type=float, default=2.0,
                    help="how often to scan for new finished segments")
    args = ap.parse_args()

    if not args.api_key:
        ap.error("api key required: --api-key or export ROBOFLOW_API_KEY=...")

    print("[detector_worker] loading model "
          "(may pull weights + compile TRT engine on first run)…")
    model = get_model(model_id=args.model_id, api_key=args.api_key)
    print(f"[detector_worker] model ready: {args.model_id}")

    seen: set[str] = set()
    while True:
        for svo in sorted(args.segments_dir.glob("segment-*.svo2")):
            seg_id = svo.stem  # "segment-00001"
            out_path = args.out_dir / f"{seg_id}.jsonl"
            if seg_id in seen or out_path.exists():
                seen.add(seg_id)
                continue
            # Recorder writes segments sequentially. A segment-NNNNN is
            # "finished" once segment-N+1 exists on disk.
            if not (args.segments_dir / _next_segment_id(seg_id)).exists():
                continue
            print(f"[detector_worker] processing {seg_id}")
            n = process_segment(svo, out_path, model, conf=args.conf)
            print(f"[detector_worker]   wrote {n} frames → {out_path}")
            seen.add(seg_id)

        time.sleep(args.poll_seconds)


if __name__ == "__main__":
    main()

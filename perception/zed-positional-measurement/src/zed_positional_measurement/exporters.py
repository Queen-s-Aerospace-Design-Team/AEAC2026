from __future__ import annotations

import csv
from pathlib import Path
from typing import Iterable

from .models import MeasurementRecord
from .storage import write_jsonl


CSV_FIELDS = [
    "session_id",
    "segment_id",
    "frame_idx",
    "timestamp_ns",
    "entity_type",
    "label",
    "color_or_corner_id",
    "detector_confidence",
    "pixel_u",
    "pixel_v",
    "bbox_xyxy",
    "pose_world_xyz",
    "pose_world_xyzw",
    "tracking_state",
    "tracking_confidence",
    "point_world_xyz",
    "point_camera_xyz",
    "depth_m",
    "plane_id",
    "plane_status",
    "plane_center_xyz",
    "plane_normal_xyz",
    "plane_equation_abcd",
    "plane_type",
    "quality_flags",
]


def export_measurements_jsonl(path: Path, records: Iterable[MeasurementRecord]) -> None:
    write_jsonl(path, (record.to_dict() for record in records))


def export_measurements_csv(path: Path, records: Iterable[MeasurementRecord]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for record in records:
            if not record.is_clean:
                continue
            row = record.to_dict()
            for key in ("bbox_xyxy", "pose_world_xyz", "pose_world_xyzw", "point_world_xyz", "point_camera_xyz"):
                if row[key] is not None:
                    row[key] = ",".join(str(value) for value in row[key])
            for key in ("plane_center_xyz", "plane_normal_xyz", "plane_equation_abcd", "quality_flags"):
                if row[key] is not None:
                    row[key] = ",".join(str(value) for value in row[key])
            writer.writerow(row)

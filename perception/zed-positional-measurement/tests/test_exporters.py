from __future__ import annotations

import unittest
import sys
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
TMP_ROOT = ROOT / "test-output"
TMP_ROOT.mkdir(exist_ok=True)
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from zed_positional_measurement.exporters import export_measurements_csv
from zed_positional_measurement.models import MeasurementRecord


def _record(*, clean: bool) -> MeasurementRecord:
    return MeasurementRecord(
        session_id="s1",
        segment_id="segment-00001",
        frame_idx=0,
        timestamp_ns=1,
        entity_type="paper",
        label="paper",
        color_or_corner_id="red",
        detector_confidence=0.9,
        pixel_u=10,
        pixel_v=12,
        bbox_xyxy=(1, 2, 3, 4),
        pose_world_xyz=(0.0, 0.0, 0.0),
        pose_world_xyzw=(0.0, 0.0, 0.0, 1.0),
        tracking_state="OK",
        tracking_confidence=100,
        point_world_xyz=(1.0, 2.0, 3.0),
        point_camera_xyz=(1.0, 2.0, 3.0),
        depth_m=2.0,
        plane_status="SUCCESS",
        plane_center_xyz=(0.0, 0.0, 0.0),
        plane_normal_xyz=(0.0, 0.0, 1.0),
        plane_equation_abcd=(0.0, 0.0, 1.0, 0.0),
        plane_type="VERTICAL",
        quality_flags=("clean",) if clean else ("point_invalid",),
    )


class ExporterTests(unittest.TestCase):
    def test_csv_export_includes_only_clean_rows(self) -> None:
        output_dir = TMP_ROOT / "exporters"
        shutil.rmtree(output_dir, ignore_errors=True)
        output_dir.mkdir(parents=True, exist_ok=True)
        output = output_dir / "measurements.csv"
        export_measurements_csv(output, [_record(clean=True), _record(clean=False)])

        lines = output.read_text(encoding="utf-8").strip().splitlines()
        self.assertEqual(len(lines), 2)
        self.assertIn("plane_id", lines[0])
        self.assertIn("red", lines[1])


if __name__ == "__main__":
    unittest.main()

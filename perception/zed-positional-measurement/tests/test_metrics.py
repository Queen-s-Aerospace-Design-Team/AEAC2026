from __future__ import annotations

import unittest
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from zed_positional_measurement.metrics import compute_measurement_metrics, compute_tracking_metrics
from zed_positional_measurement.models import MeasurementRecord, PoseRecord


def _pose(frame_idx: int, xyz: tuple[float, float, float], state: str = "OK") -> PoseRecord:
    return PoseRecord(
        session_id="session-a",
        segment_id="segment-00001",
        frame_idx=frame_idx,
        timestamp_ns=frame_idx,
        tracking_state=state,
        tracking_confidence=99,
        pose_world_xyz=xyz if state == "OK" else None,
        pose_world_xyzw=(0.0, 0.0, 0.0, 1.0) if state == "OK" else None,
    )


def _paper(
    *,
    color: str,
    point_world_xyz: tuple[float, float, float] | None,
    plane_status: str,
    plane_id: str | None,
    quality_flags: tuple[str, ...],
) -> MeasurementRecord:
    return MeasurementRecord(
        session_id="session-a",
        segment_id="segment-00001",
        frame_idx=0,
        timestamp_ns=1,
        entity_type="paper",
        label="paper",
        color_or_corner_id=color,
        detector_confidence=0.9,
        pixel_u=10,
        pixel_v=12,
        bbox_xyxy=(1, 2, 3, 4),
        pose_world_xyz=(0.0, 0.0, 0.0),
        pose_world_xyzw=(0.0, 0.0, 0.0, 1.0),
        tracking_state="OK",
        tracking_confidence=100,
        point_world_xyz=point_world_xyz,
        point_camera_xyz=point_world_xyz,
        depth_m=2.0,
        plane_status=plane_status,
        plane_center_xyz=(0.0, 0.0, 0.0) if plane_status == "SUCCESS" else None,
        plane_normal_xyz=(0.0, 0.0, 1.0) if plane_status == "SUCCESS" else None,
        plane_equation_abcd=(0.0, 0.0, 1.0, 0.0) if plane_status == "SUCCESS" else None,
        plane_type="VERTICAL" if plane_status == "SUCCESS" else None,
        plane_id=plane_id,
        quality_flags=quality_flags,
    )


class MetricTests(unittest.TestCase):
    def test_compute_tracking_metrics_matches_current_fields(self) -> None:
        metrics = compute_tracking_metrics(
            [
                _pose(0, (0.0, 0.0, 0.0)),
                _pose(1, (3.0, 0.0, 0.0)),
                _pose(2, (3.0, 4.0, 0.0)),
            ],
            export_written_at_ns=5_000_000_000,
            session_closed_at_ns=2_000_000_000,
        )

        self.assertEqual(metrics.ok_tracking_ratio, 1.0)
        self.assertEqual(metrics.loop_return_error_m, 5.0)
        self.assertEqual(metrics.max_pose_jump_m, 4.0)
        self.assertEqual(metrics.p95_pose_jump_m, 3.95)
        self.assertEqual(metrics.final_export_latency_s, 3.0)

    def test_compute_measurement_metrics_uses_papers_and_plane_success(self) -> None:
        metrics = compute_measurement_metrics(
            [
                _paper(
                    color="red",
                    point_world_xyz=(1.0, 0.0, 0.0),
                    plane_status="SUCCESS",
                    plane_id="plane-0001",
                    quality_flags=("clean",),
                ),
                _paper(
                    color="blue",
                    point_world_xyz=None,
                    plane_status="NOT_FOUND",
                    plane_id=None,
                    quality_flags=("point_invalid", "plane_unavailable"),
                ),
            ],
            {
                ("paper", "red"): (0.0, 0.0, 0.0),
                ("paper", "blue"): (1.0, 1.0, 1.0),
            },
        )

        self.assertEqual(metrics.paper_position_mae_m, 1.0)
        self.assertEqual(metrics.paper_position_p95_m, 1.0)
        self.assertEqual(metrics.plane_success_rate, 0.5)
        self.assertEqual(metrics.valid_measurement_rate, 0.5)


if __name__ == "__main__":
    unittest.main()

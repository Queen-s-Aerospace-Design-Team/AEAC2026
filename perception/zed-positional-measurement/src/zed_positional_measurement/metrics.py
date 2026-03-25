from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from .geometry import euclidean_distance
from .models import MeasurementRecord, PoseRecord


@dataclass(frozen=True)
class TrackingMetrics:
    ok_tracking_ratio: float
    loop_return_error_m: float | None
    max_pose_jump_m: float | None
    p95_pose_jump_m: float | None
    final_export_latency_s: float | None


@dataclass(frozen=True)
class MeasurementMetrics:
    paper_position_mae_m: float | None
    paper_position_p95_m: float | None
    plane_success_rate: float
    valid_measurement_rate: float


def _percentile(sorted_values: list[float], percentile: float) -> float | None:
    if not sorted_values:
        return None
    if len(sorted_values) == 1:
        return sorted_values[0]
    index = (len(sorted_values) - 1) * percentile
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return sorted_values[lower]
    return sorted_values[lower] + (sorted_values[upper] - sorted_values[lower]) * (index - lower)


def compute_tracking_metrics(
    poses: Iterable[PoseRecord],
    *,
    export_written_at_ns: int | None = None,
    session_closed_at_ns: int | None = None,
) -> TrackingMetrics:
    pose_list = list(poses)
    valid = [pose for pose in pose_list if pose.tracking_state == "OK" and pose.pose_world_xyz is not None]
    ok_ratio = (len(valid) / len(pose_list)) if pose_list else 0.0

    loop_return_error = None
    if len(valid) >= 2:
        loop_return_error = euclidean_distance(valid[0].pose_world_xyz or (0, 0, 0), valid[-1].pose_world_xyz or (0, 0, 0))

    jumps: list[float] = []
    for previous, current in zip(valid, valid[1:], strict=False):
        jumps.append(
            euclidean_distance(previous.pose_world_xyz or (0, 0, 0), current.pose_world_xyz or (0, 0, 0))
        )
    sorted_jumps = sorted(jumps)
    latency = None
    if export_written_at_ns is not None and session_closed_at_ns is not None:
        latency = max(0.0, (export_written_at_ns - session_closed_at_ns) / 1_000_000_000)

    return TrackingMetrics(
        ok_tracking_ratio=ok_ratio,
        loop_return_error_m=loop_return_error,
        max_pose_jump_m=max(jumps) if jumps else None,
        p95_pose_jump_m=_percentile(sorted_jumps, 0.95),
        final_export_latency_s=latency,
    )


def compute_measurement_metrics(
    records: Iterable[MeasurementRecord],
    ground_truth: dict[tuple[str, str], tuple[float, float, float]],
) -> MeasurementMetrics:
    all_records = list(records)
    paper_records = [record for record in all_records if record.entity_type == "paper"]
    record_list = [record for record in paper_records if record.is_clean and record.point_world_xyz is not None]
    paper_errors: list[float] = []

    for record in record_list:
        truth = ground_truth.get((record.entity_type, record.color_or_corner_id))
        if truth is None:
            continue
        error = euclidean_distance(record.point_world_xyz or (0, 0, 0), truth)
        paper_errors.append(error)

    paper_errors.sort()
    plane_success_count = sum(
        1 for record in paper_records if record.plane_status == "SUCCESS" and record.plane_id is not None
    )
    return MeasurementMetrics(
        paper_position_mae_m=(sum(paper_errors) / len(paper_errors)) if paper_errors else None,
        paper_position_p95_m=_percentile(paper_errors, 0.95),
        plane_success_rate=(plane_success_count / len(paper_records)) if paper_records else 0.0,
        valid_measurement_rate=(len(record_list) / len(paper_records)) if paper_records else 0.0,
    )

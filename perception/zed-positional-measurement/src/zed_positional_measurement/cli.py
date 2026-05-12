from __future__ import annotations

import argparse
import json
from pathlib import Path

from .config import PipelineConfig
from .metrics import compute_measurement_metrics, compute_tracking_metrics
from .models import MeasurementRecord, PoseRecord
from .pipeline import MeasurementPipeline
from .storage import SessionStore, read_json


def _load_config(path: str | None) -> PipelineConfig:
    if path is None:
        return PipelineConfig()
    return PipelineConfig.from_json_file(path)


def _cmd_run(args: argparse.Namespace) -> int:
    pipeline = MeasurementPipeline(_load_config(args.config))
    session_root = pipeline.run_live_session()
    print(session_root)
    return 0


def _cmd_record(args: argparse.Namespace) -> int:
    pipeline = MeasurementPipeline(_load_config(args.config))
    session_root = pipeline.record_session(
        args.session_root,
        process_in_background=False,
        finalize_at_end=False,
    )
    print(session_root)
    return 0


def _cmd_process(args: argparse.Namespace) -> int:
    pipeline = MeasurementPipeline(_load_config(args.config))
    pipeline.process_session(args.session_root, finalized=False)
    return 0


def _cmd_finalize(args: argparse.Namespace) -> int:
    pipeline = MeasurementPipeline(_load_config(args.config))
    pipeline.finalize_session(args.session_root)
    return 0


def _cmd_eval_tracking(args: argparse.Namespace) -> int:
    store = SessionStore.open(args.session_root)
    poses: list[PoseRecord] = []
    for entry in store.list_segments():
        poses.extend(store.read_pose_cache(entry.segment_id, finalized=True))
    metadata = store.session_metadata()
    metrics = compute_tracking_metrics(
        poses,
        export_written_at_ns=store.paths.measurements_jsonl_path.stat().st_mtime_ns if store.paths.measurements_jsonl_path.exists() else None,
        session_closed_at_ns=max((entry.closed_at_ns for entry in store.list_segments()), default=None),
    )
    print(json.dumps(metrics.__dict__, indent=2))
    return 0


def _cmd_eval_measurements(args: argparse.Namespace) -> int:
    store = SessionStore.open(args.session_root)
    records: list[MeasurementRecord] = []
    for entry in store.list_segments():
        records.extend(store.read_measurement_cache(entry.segment_id, finalized=True))
    ground_truth_raw = read_json(Path(args.ground_truth))
    ground_truth = {
        (str(key.split(":", 1)[0]), str(key.split(":", 1)[1])): tuple(float(v) for v in value)
        for key, value in ground_truth_raw.items()
    }
    metrics = compute_measurement_metrics(records, ground_truth)
    print(json.dumps(metrics.__dict__, indent=2))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="zed-measure")
    subparsers = parser.add_subparsers(dest="command", required=True)

    run_parser = subparsers.add_parser("run", help="Record a live session, process segments, and finalize exports.")
    run_parser.add_argument("--config")
    run_parser.set_defaults(func=_cmd_run)

    record_parser = subparsers.add_parser("record", help="Record a live session without processing or finalization.")
    record_parser.add_argument("--config")
    record_parser.add_argument("--session-root")
    record_parser.set_defaults(func=_cmd_record)

    process_parser = subparsers.add_parser("process", help="Process recorded segments into provisional caches.")
    process_parser.add_argument("session_root")
    process_parser.add_argument("--config")
    process_parser.set_defaults(func=_cmd_process)

    finalize_parser = subparsers.add_parser("finalize", help="Replay processed segments and emit final exports.")
    finalize_parser.add_argument("session_root")
    finalize_parser.add_argument("--config")
    finalize_parser.set_defaults(func=_cmd_finalize)

    tracking_parser = subparsers.add_parser("eval-tracking", help="Compute tracking metrics from finalized pose caches.")
    tracking_parser.add_argument("session_root")
    tracking_parser.set_defaults(func=_cmd_eval_tracking)

    measurements_parser = subparsers.add_parser(
        "eval-measurements",
        help="Compute paper/plane measurement metrics from finalized caches and a ground truth JSON file.",
    )
    measurements_parser.add_argument("session_root")
    measurements_parser.add_argument("ground_truth")
    measurements_parser.set_defaults(func=_cmd_eval_measurements)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return int(args.func(args))

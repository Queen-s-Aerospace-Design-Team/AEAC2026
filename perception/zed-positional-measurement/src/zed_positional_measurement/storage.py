from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable, TypeVar

from .config import PipelineConfig
from .models import FrameRecord, MeasurementRecord, PoseRecord, SegmentManifestEntry

T = TypeVar("T")


def utc_time_ns() -> int:
    return time.time_ns()


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def write_jsonl(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        for row in rows:
            handle.write(json.dumps(row, sort_keys=True))
            handle.write("\n")


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def read_typed_jsonl(path: Path, factory: Callable[[dict[str, Any]], T]) -> list[T]:
    return [factory(row) for row in read_jsonl(path)]


@dataclass(frozen=True)
class SessionPaths:
    root: Path

    @property
    def session_id(self) -> str:
        return self.root.name

    @property
    def segments_dir(self) -> Path:
        return self.root / "segments"

    @property
    def maps_dir(self) -> Path:
        return self.root / "maps"

    @property
    def cache_dir(self) -> Path:
        return self.root / "cache"

    @property
    def exports_dir(self) -> Path:
        return self.root / "exports"

    @property
    def stream_dir(self) -> Path:
        return self.root / "stream"

    @property
    def session_metadata_path(self) -> Path:
        return self.root / "session.json"

    @property
    def area_map_path(self) -> Path:
        return self.maps_dir / f"{self.session_id}.area"

    @property
    def measurements_jsonl_path(self) -> Path:
        return self.exports_dir / "measurements.jsonl"

    @property
    def measurements_csv_path(self) -> Path:
        return self.exports_dir / "measurements.csv"

    @property
    def frames_jsonl_path(self) -> Path:
        return self.stream_dir / "frames.jsonl"

    def segment_svo_path(self, segment_id: str) -> Path:
        return self.segments_dir / f"{segment_id}.svo2"

    def segment_dir(self, segment_id: str) -> Path:
        return self.cache_dir / segment_id

    def segment_manifest_path(self, segment_id: str) -> Path:
        return self.segment_dir(segment_id) / "segment.json"

    def segment_pose_cache_path(self, segment_id: str, finalized: bool) -> Path:
        suffix = "final" if finalized else "provisional"
        return self.segment_dir(segment_id) / f"poses.{suffix}.jsonl"

    def segment_measurement_cache_path(self, segment_id: str, finalized: bool) -> Path:
        suffix = "final" if finalized else "provisional"
        return self.segment_dir(segment_id) / f"measurements.{suffix}.jsonl"

    def segment_frame_cache_path(self, segment_id: str, finalized: bool) -> Path:
        suffix = "final" if finalized else "provisional"
        return self.segment_dir(segment_id) / f"frames.{suffix}.jsonl"


def create_session_paths(root_dir: Path | str, session_id: str) -> SessionPaths:
    root = Path(root_dir) / session_id
    paths = SessionPaths(root=root)
    for directory in (
        paths.root,
        paths.segments_dir,
        paths.maps_dir,
        paths.cache_dir,
        paths.exports_dir,
        paths.stream_dir,
    ):
        directory.mkdir(parents=True, exist_ok=True)
    return paths


class SessionStore:
    def __init__(self, paths: SessionPaths):
        self.paths = paths

    @classmethod
    def create(cls, config: PipelineConfig) -> "SessionStore":
        session_id = config.runtime.session_id or time.strftime("session-%Y%m%d-%H%M%S")
        paths = create_session_paths(config.runtime.root_dir, session_id)
        store = cls(paths)
        write_json(
            paths.session_metadata_path,
            {
                "session_id": session_id,
                "created_at_ns": utc_time_ns(),
                "config": config.to_dict(),
            },
        )
        return store

    @classmethod
    def open(cls, session_root: Path | str) -> "SessionStore":
        return cls(SessionPaths(root=Path(session_root)))

    def session_metadata(self) -> dict[str, Any]:
        return read_json(self.paths.session_metadata_path)

    def write_segment_manifest(self, entry: SegmentManifestEntry) -> None:
        write_json(self.paths.segment_manifest_path(entry.segment_id), entry.to_dict())

    def read_segment_manifest(self, segment_id: str) -> SegmentManifestEntry:
        return SegmentManifestEntry.from_dict(read_json(self.paths.segment_manifest_path(segment_id)))

    def list_segments(self) -> list[SegmentManifestEntry]:
        segment_files = sorted(self.paths.cache_dir.glob("*/segment.json"))
        return [SegmentManifestEntry.from_dict(read_json(path)) for path in segment_files]

    def next_segment_id(self) -> str:
        return f"segment-{len(self.list_segments()) + 1:05d}"

    def set_segment_status(
        self,
        segment_id: str,
        status: str,
        *,
        error_message: str | None = None,
        finalized: bool | None = None,
    ) -> SegmentManifestEntry:
        entry = self.read_segment_manifest(segment_id)
        payload = entry.to_dict()
        payload["status"] = status
        payload["updated_at_ns"] = utc_time_ns()
        if error_message is not None:
            payload["error_message"] = error_message
        if finalized is not None:
            if finalized:
                payload["final_pose_path"] = str(self.paths.segment_pose_cache_path(segment_id, finalized=True))
                payload["final_measurement_path"] = str(
                    self.paths.segment_measurement_cache_path(segment_id, finalized=True)
                )
                payload["final_frame_path"] = str(self.paths.segment_frame_cache_path(segment_id, finalized=True))
            else:
                payload["provisional_pose_path"] = str(self.paths.segment_pose_cache_path(segment_id, finalized=False))
                payload["provisional_measurement_path"] = str(
                    self.paths.segment_measurement_cache_path(segment_id, finalized=False)
                )
                payload["provisional_frame_path"] = str(self.paths.segment_frame_cache_path(segment_id, finalized=False))
        updated = SegmentManifestEntry.from_dict(payload)
        self.write_segment_manifest(updated)
        return updated

    def pending_segments(self, *, finalized: bool) -> list[SegmentManifestEntry]:
        statuses = {"recorded", "failed"} if not finalized else {"processed"}
        return [entry for entry in self.list_segments() if entry.status in statuses]

    def write_pose_cache(self, segment_id: str, finalized: bool, rows: Iterable[PoseRecord]) -> Path:
        path = self.paths.segment_pose_cache_path(segment_id, finalized=finalized)
        write_jsonl(path, (row.to_dict() for row in rows))
        return path

    def write_frame_cache(
        self,
        segment_id: str,
        finalized: bool,
        rows: Iterable[FrameRecord],
    ) -> Path:
        path = self.paths.segment_frame_cache_path(segment_id, finalized=finalized)
        write_jsonl(path, (row.to_dict() for row in rows))
        return path

    def write_measurement_cache(
        self,
        segment_id: str,
        finalized: bool,
        rows: Iterable[MeasurementRecord],
    ) -> Path:
        path = self.paths.segment_measurement_cache_path(segment_id, finalized=finalized)
        write_jsonl(path, (row.to_dict() for row in rows))
        return path

    def read_pose_cache(self, segment_id: str, finalized: bool) -> list[PoseRecord]:
        return read_typed_jsonl(self.paths.segment_pose_cache_path(segment_id, finalized=finalized), PoseRecord.from_dict)

    def read_measurement_cache(self, segment_id: str, finalized: bool) -> list[MeasurementRecord]:
        return read_typed_jsonl(
            self.paths.segment_measurement_cache_path(segment_id, finalized=finalized),
            MeasurementRecord.from_dict,
        )

    def read_frame_cache(self, segment_id: str, finalized: bool) -> list[FrameRecord]:
        return read_typed_jsonl(
            self.paths.segment_frame_cache_path(segment_id, finalized=finalized),
            FrameRecord.from_dict,
        )

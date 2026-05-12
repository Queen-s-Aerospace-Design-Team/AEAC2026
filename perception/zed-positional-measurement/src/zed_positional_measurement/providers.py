from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

from .models import CornerDetection, PaperDetection
from .storage import read_jsonl


@dataclass
class SegmentJsonlPaperProvider:
    root_dir: Path | None

    def get(self, segment_id: str, frame_idx: int) -> list[PaperDetection]:
        if self.root_dir is None:
            return []
        path = Path(self.root_dir) / f"{segment_id}.jsonl"
        by_frame: dict[int, list[PaperDetection]] = defaultdict(list)
        for row in read_jsonl(path):
            detection = PaperDetection.from_dict(row)
            by_frame[detection.frame_idx].append(detection)
        return by_frame.get(frame_idx, [])


@dataclass
class SegmentJsonlCornerProvider:
    root_dir: Path | None

    def get(self, segment_id: str, frame_idx: int) -> list[CornerDetection]:
        if self.root_dir is None:
            return []
        path = Path(self.root_dir) / f"{segment_id}.jsonl"
        by_frame: dict[int, list[CornerDetection]] = defaultdict(list)
        for row in read_jsonl(path):
            detection = CornerDetection.from_dict(row)
            by_frame[detection.frame_idx].append(detection)
        return by_frame.get(frame_idx, [])

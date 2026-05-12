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

from zed_positional_measurement.config import PipelineConfig
from zed_positional_measurement.models import SegmentManifestEntry
from zed_positional_measurement.storage import SessionStore


class StorageTests(unittest.TestCase):
    def test_pending_segments_only_returns_unfinished_segment_manifests(self) -> None:
        output_dir = TMP_ROOT / "storage"
        shutil.rmtree(output_dir, ignore_errors=True)
        output_dir.mkdir(parents=True, exist_ok=True)
        config = PipelineConfig.from_dict(
            {
                "runtime": {
                    "root_dir": str(output_dir),
                    "session_id": "session-a",
                }
            }
        )
        store = SessionStore.create(config)
        store.write_segment_manifest(
            SegmentManifestEntry(
                session_id=store.paths.session_id,
                segment_id="segment-00001",
                svo_path=str(store.paths.segment_svo_path("segment-00001")),
                status="recorded",
                started_at_ns=1,
                closed_at_ns=2,
                frame_count=10,
            )
        )
        store.write_segment_manifest(
            SegmentManifestEntry(
                session_id=store.paths.session_id,
                segment_id="segment-00002",
                svo_path=str(store.paths.segment_svo_path("segment-00002")),
                status="processed",
                started_at_ns=3,
                closed_at_ns=4,
                frame_count=12,
            )
        )

        pending = store.pending_segments(finalized=False)

        self.assertEqual([entry.segment_id for entry in pending], ["segment-00001"])


if __name__ == "__main__":
    unittest.main()

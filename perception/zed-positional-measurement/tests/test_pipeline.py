from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any
import tempfile
import unittest
import sys
import shutil

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
TMP_ROOT = ROOT / "test-output"
TMP_ROOT.mkdir(exist_ok=True)
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from zed_positional_measurement.config import PipelineConfig
from zed_positional_measurement.models import CornerDetection, PaperDetection, SegmentManifestEntry
from zed_positional_measurement.pipeline import MeasurementPipeline
from zed_positional_measurement.sdk import DetectionContext, FrameBuffers, PaperObservation, PlaneSnapshot, PoseSnapshot
from zed_positional_measurement.storage import SessionStore, read_jsonl


@dataclass
class _Frame:
    pose: PoseSnapshot
    paper_observations: list[PaperObservation]
    points: dict[tuple[int, int], tuple[float, float, float]]
    depths: dict[tuple[int, int], float]
    planes_by_pixel: dict[tuple[int, int], PlaneSnapshot]
    image_size: tuple[int, int]
    floor_plane: PlaneSnapshot | None = None


class FakePaperProvider:
    def __init__(self, frames: dict[int, list[PaperDetection]]):
        self.frames = frames

    def get(self, segment_id: str, frame_idx: int) -> list[PaperDetection]:
        return list(self.frames.get(frame_idx, []))


class FakeCornerProvider:
    def __init__(self, frames: dict[int, list[CornerDetection]]):
        self.frames = frames

    def get(self, segment_id: str, frame_idx: int) -> list[CornerDetection]:
        return list(self.frames.get(frame_idx, []))


class FakeAdapter:
    def __init__(self, frames_by_segment: dict[str, list[_Frame]]):
        self.frames_by_segment = frames_by_segment
        self.opened_segments: list[str] = []
        self.disabled_tracking: list[str | None] = []
        self.detect_plane_calls: list[tuple[int, int]] = []
        self.detect_floor_plane_calls = 0

    def open_live_camera(self, recording: Any) -> Any:
        raise NotImplementedError

    def open_svo_camera(self, recording: Any, svo_path: Path) -> dict[str, Any]:
        segment_id = svo_path.stem
        self.opened_segments.append(segment_id)
        return {"segment_id": segment_id, "index": -1, "current": None}

    def close_camera(self, camera: dict[str, Any]) -> None:
        return None

    def build_runtime_parameters(self) -> object:
        return object()

    def enable_tracking(self, camera: Any, tracking: Any, *, mode: str, area_path: Path | None) -> None:
        return None

    def disable_tracking(self, camera: Any, *, area_path: Path | None) -> None:
        self.disabled_tracking.append(str(area_path) if area_path else None)

    def configure_paper_detection(self, camera: Any, config: Any) -> DetectionContext:
        return DetectionContext(enabled=True, mode=config.mode, runtime_params=None)

    def disable_paper_detection(self, camera: Any) -> None:
        return None

    def create_frame_buffers(self) -> FrameBuffers:
        return FrameBuffers(point_cloud_world={}, depth_map={}, image_size=(0, 0))

    def grab(self, camera: dict[str, Any], runtime: object) -> str:
        frames = self.frames_by_segment[camera["segment_id"]]
        camera["index"] += 1
        if camera["index"] >= len(frames):
            return "END"
        camera["current"] = frames[camera["index"]]
        return "SUCCESS"

    def success_code(self) -> str:
        return "SUCCESS"

    def end_of_svo_code(self) -> str:
        return "END"

    def pose_snapshot(self, camera: dict[str, Any]) -> PoseSnapshot:
        return camera["current"].pose

    def paper_observations(
        self,
        camera: dict[str, Any],
        context: DetectionContext,
        config: Any,
        external_detections: list[PaperDetection],
    ) -> list[PaperObservation]:
        if external_detections:
            detection = external_detections[0]
            x1, y1, x2, y2 = detection.bbox_xyxy
            return [
                PaperObservation(
                    bbox_xyxy=detection.bbox_xyxy,
                    center_pixel=((x1 + x2) // 2, (y1 + y2) // 2),
                    confidence=detection.confidence,
                    label=detection.label,
                    color=detection.color,
                    mask_pixels=tuple(),
                )
            ]
        return list(camera["current"].paper_observations)

    def sample_world_point(self, buffers: FrameBuffers, u: int, v: int):
        return buffers.__dict__.get("current_points", {}).get((u, v))

    def sample_depth(self, buffers: FrameBuffers, u: int, v: int):
        return buffers.__dict__.get("current_depths", {}).get((u, v))

    def detect_plane(self, camera: dict[str, Any], u: int, v: int) -> PlaneSnapshot:
        self.detect_plane_calls.append((u, v))
        return camera["current"].planes_by_pixel.get(
            (u, v),
            PlaneSnapshot(
                status="NOT_FOUND",
                center_xyz=None,
                normal_xyz=None,
                equation_abcd=None,
                plane_type=None,
            ),
        )

    def detect_floor_plane(self, camera: dict[str, Any]) -> PlaneSnapshot:
        self.detect_floor_plane_calls += 1
        return camera["current"].floor_plane or PlaneSnapshot(
            status="NOT_FOUND",
            center_xyz=None,
            normal_xyz=None,
            equation_abcd=None,
            plane_type=None,
        )

    def populate_frame_buffers(self, camera: dict[str, Any], buffers: FrameBuffers) -> None:
        frame = camera["current"]
        buffers.image_size = frame.image_size
        buffers.__dict__["current_points"] = frame.points
        buffers.__dict__["current_depths"] = frame.depths


class FakeLiveRecordAdapter:
    def __init__(self) -> None:
        self.enable_recording_calls = 0
        self.disable_recording_calls = 0
        self.close_camera_calls = 0
        self.grab_calls = 0

    def open_live_camera(self, recording: Any) -> dict[str, Any]:
        return {"live": True}

    def build_runtime_parameters(self) -> object:
        return object()

    def enable_recording(self, camera: Any, output_path: Path, recording: Any) -> None:
        self.enable_recording_calls += 1

    def disable_recording(self, camera: Any) -> None:
        self.disable_recording_calls += 1

    def grab(self, camera: Any, runtime: object) -> str:
        self.grab_calls += 1
        if self.grab_calls == 1:
            return "SUCCESS"
        raise KeyboardInterrupt

    def success_code(self) -> str:
        return "SUCCESS"

    def close_camera(self, camera: Any) -> None:
        self.close_camera_calls += 1


def _make_store(tmp_path, config: PipelineConfig) -> SessionStore:
    store = SessionStore.create(config)
    segment_id = "segment-00001"
    store.paths.segment_svo_path(segment_id).write_text("placeholder", encoding="utf-8")
    store.write_segment_manifest(
        SegmentManifestEntry(
            session_id=store.paths.session_id,
            segment_id=segment_id,
            svo_path=str(store.paths.segment_svo_path(segment_id)),
            status="recorded",
            started_at_ns=1,
            closed_at_ns=2,
            frame_count=1,
        )
    )
    return store


class PipelineTests(unittest.TestCase):
    def test_record_session_skips_svo_recording_when_disabled(self) -> None:
        output_dir = TMP_ROOT / "pipeline-no-svo"
        shutil.rmtree(output_dir, ignore_errors=True)
        output_dir.mkdir(parents=True, exist_ok=True)
        config = PipelineConfig.from_dict(
            {
                "recording": {
                    "enable_svo_recording": False,
                },
                "paper": {"mode": "external_boxes"},
                "runtime": {
                    "root_dir": str(output_dir),
                    "session_id": "session-no-svo",
                },
            }
        )
        adapter = FakeLiveRecordAdapter()
        pipeline = MeasurementPipeline(
            config,
            adapter=adapter,
            paper_provider=FakePaperProvider({}),
            corner_provider=FakeCornerProvider({}),
        )

        session_root = pipeline.record_session(
            process_in_background=False,
            finalize_at_end=False,
        )

        store = SessionStore.open(session_root)
        self.assertEqual(adapter.enable_recording_calls, 0)
        self.assertEqual(adapter.disable_recording_calls, 0)
        self.assertEqual(adapter.close_camera_calls, 1)
        self.assertEqual(store.list_segments(), [])

    def test_run_live_session_requires_svo_when_replay_processing_is_enabled(self) -> None:
        config = PipelineConfig.from_dict(
            {
                "recording": {
                    "enable_svo_recording": False,
                },
                "paper": {"mode": "external_boxes"},
                "runtime": {
                    "root_dir": str(TMP_ROOT / "pipeline-requires-svo"),
                    "session_id": "session-requires-svo",
                },
            }
        )
        pipeline = MeasurementPipeline(
            config,
            adapter=FakeLiveRecordAdapter(),
            paper_provider=FakePaperProvider({}),
            corner_provider=FakeCornerProvider({}),
        )

        with self.assertRaisesRegex(ValueError, "enable_svo_recording=true"):
            pipeline.run_live_session()

    def test_process_segment_rescales_inputs_and_writes_caches(self) -> None:
        output_dir = TMP_ROOT / "pipeline-segment"
        shutil.rmtree(output_dir, ignore_errors=True)
        output_dir.mkdir(parents=True, exist_ok=True)
        config = PipelineConfig.from_dict(
            {
                "paper": {"mode": "external_boxes"},
                "runtime": {
                    "root_dir": str(output_dir),
                    "session_id": "session-b",
                    "detector_image_size": [50, 25],
                },
            }
        )
        pose = PoseSnapshot(
            timestamp_ns=100,
            tracking_state="OK",
            tracking_confidence=99,
            pose_world_xyz=(1.0, 0.0, 0.0),
            pose_world_xyzw=(0.0, 0.0, 0.0, 1.0),
        )
        plane = PlaneSnapshot(
            status="SUCCESS",
            center_xyz=(0.0, 0.0, 0.0),
            normal_xyz=(0.0, 0.0, 1.0),
            equation_abcd=(0.0, 0.0, 1.0, 0.0),
            plane_type="VERTICAL",
        )
        frames = {
            "segment-00001": [
                _Frame(
                    pose=pose,
                    paper_observations=[],
                    points={
                        (29, 19): (1.0, 2.0, 3.0),
                        (30, 20): (2.0, 4.0, 6.0),
                        (31, 21): (3.0, 6.0, 9.0),
                        (9, 10): (10.0, 0.0, 0.0),
                        (10, 10): (2.0, 0.0, 0.0),
                        (11, 10): (5.0, 0.0, 0.0),
                    },
                    depths={
                        (30, 20): 6.0,
                        (10, 10): 2.0,
                    },
                    planes_by_pixel={
                        (30, 20): plane,
                        (16, 8): PlaneSnapshot(
                            status="SUCCESS",
                            center_xyz=(0.2, 0.0, 0.0),
                            normal_xyz=(0.0, 0.0, 1.0),
                            equation_abcd=(0.0, 0.0, 1.0, 0.0),
                            plane_type="VERTICAL",
                        ),
                        (50, 24): PlaneSnapshot(
                            status="SUCCESS",
                            center_xyz=(2.0, 0.0, 0.0),
                            normal_xyz=(1.0, 0.0, 0.0),
                            equation_abcd=(1.0, 0.0, 0.0, -2.0),
                            plane_type="VERTICAL",
                        ),
                        (10, 10): plane,
                    },
                    image_size=(100, 50),
                )
            ]
        }
        store = _make_store(output_dir, config)
        adapter = FakeAdapter(frames)
        pipeline = MeasurementPipeline(
            config,
            adapter=adapter,
            paper_provider=FakePaperProvider(
                {
                    0: [
                        PaperDetection(
                            frame_idx=0,
                            bbox_xyxy=(10, 5, 20, 15),
                            label="paper",
                            color="red",
                            confidence=0.9,
                        )
                    ]
                }
            ),
            corner_provider=FakeCornerProvider(
                {0: [CornerDetection(frame_idx=0, corner_id="c1", u=5, v=5, confidence=0.8)]}
            ),
        )

        pipeline.process_segment(store.paths.root, "segment-00001", finalized=False)

        measurements = store.read_measurement_cache("segment-00001", finalized=False)
        frames_out = store.read_frame_cache("segment-00001", finalized=False)
        self.assertEqual(len(measurements), 2)
        self.assertEqual(len(frames_out), 1)
        paper_record = next(record for record in measurements if record.entity_type == "paper")
        corner_record = next(record for record in measurements if record.entity_type == "corner")
        self.assertEqual(paper_record.bbox_xyxy, (20, 10, 40, 30))
        self.assertEqual(paper_record.point_world_xyz, (2.0, 4.0, 6.0))
        self.assertEqual(paper_record.plane_id, "plane-0001")
        self.assertTrue(paper_record.is_clean)
        self.assertEqual(corner_record.pixel_u, 10)
        self.assertEqual(corner_record.pixel_v, 10)
        self.assertEqual(corner_record.point_world_xyz, (2.0, 0.0, 0.0))
        self.assertEqual(len(frames_out[0].planes), 2)
        self.assertEqual(frames_out[0].papers[0].plane_id, "plane-0001")
        self.assertEqual(frames_out[0].planes[0].source, "paper_center")
        self.assertEqual(frames_out[0].planes[1].source, "scene_probe")
        self.assertEqual(store.read_segment_manifest("segment-00001").status, "processed")
        self.assertEqual(adapter.detect_floor_plane_calls, 0)

    def test_process_session_skips_already_processed_segments(self) -> None:
        output_dir = TMP_ROOT / "pipeline-resume"
        shutil.rmtree(output_dir, ignore_errors=True)
        output_dir.mkdir(parents=True, exist_ok=True)
        config = PipelineConfig.from_dict(
            {
                "paper": {"mode": "external_boxes"},
                "runtime": {
                    "root_dir": str(output_dir),
                    "session_id": "session-c",
                },
            }
        )
        store = SessionStore.create(config)
        for segment_id, status in (("segment-00001", "recorded"), ("segment-00002", "processed")):
            store.paths.segment_svo_path(segment_id).write_text("placeholder", encoding="utf-8")
            store.write_segment_manifest(
                SegmentManifestEntry(
                    session_id=store.paths.session_id,
                    segment_id=segment_id,
                    svo_path=str(store.paths.segment_svo_path(segment_id)),
                    status=status,
                    started_at_ns=1,
                    closed_at_ns=2,
                    frame_count=1,
                )
            )
        pose = PoseSnapshot(
            timestamp_ns=100,
            tracking_state="OK",
            tracking_confidence=99,
            pose_world_xyz=(0.0, 0.0, 0.0),
            pose_world_xyzw=(0.0, 0.0, 0.0, 1.0),
        )
        plane = PlaneSnapshot(
            status="SUCCESS",
            center_xyz=(0.0, 0.0, 0.0),
            normal_xyz=(0.0, 0.0, 1.0),
            equation_abcd=(0.0, 0.0, 1.0, 0.0),
            plane_type="VERTICAL",
        )
        frames = {
            "segment-00001": [
                _Frame(
                    pose=pose,
                    paper_observations=[],
                    points={},
                    depths={},
                    planes_by_pixel={(2, 2): plane},
                    image_size=(10, 10),
                )
            ]
        }
        adapter = FakeAdapter(frames)
        pipeline = MeasurementPipeline(
            config,
            adapter=adapter,
            paper_provider=FakePaperProvider({}),
            corner_provider=FakeCornerProvider({}),
        )

        pipeline.process_session(store.paths.root, finalized=False)

        self.assertEqual(adapter.opened_segments, ["segment-00001"])

    def test_export_session_writes_combined_frame_stream(self) -> None:
        output_dir = TMP_ROOT / "pipeline-export-stream"
        shutil.rmtree(output_dir, ignore_errors=True)
        output_dir.mkdir(parents=True, exist_ok=True)
        config = PipelineConfig.from_dict(
            {
                "paper": {"mode": "external_boxes"},
                "runtime": {
                    "root_dir": str(output_dir),
                    "session_id": "session-stream",
                    "detector_image_size": [50, 25],
                },
            }
        )
        pose = PoseSnapshot(
            timestamp_ns=100,
            tracking_state="OK",
            tracking_confidence=99,
            pose_world_xyz=(1.0, 0.0, 0.0),
            pose_world_xyzw=(0.0, 0.0, 0.0, 1.0),
        )
        plane = PlaneSnapshot(
            status="SUCCESS",
            center_xyz=(0.0, 0.0, 0.0),
            normal_xyz=(0.0, 0.0, 1.0),
            equation_abcd=(0.0, 0.0, 1.0, 0.0),
            plane_type="VERTICAL",
        )
        frames = {
            "segment-00001": [
                _Frame(
                    pose=pose,
                    paper_observations=[],
                    points={(30, 20): (2.0, 4.0, 6.0)},
                    depths={(30, 20): 6.0},
                    planes_by_pixel={(30, 20): plane},
                    image_size=(100, 50),
                )
            ]
        }
        store = _make_store(output_dir, config)
        pipeline = MeasurementPipeline(
            config,
            adapter=FakeAdapter(frames),
            paper_provider=FakePaperProvider(
                {
                    0: [
                        PaperDetection(
                            frame_idx=0,
                            bbox_xyxy=(10, 5, 20, 15),
                            label="paper",
                            color="red",
                            confidence=0.9,
                        )
                    ]
                }
            ),
            corner_provider=FakeCornerProvider({}),
        )

        pipeline.process_segment(store.paths.root, "segment-00001", finalized=True)
        pipeline.export_session(store.paths.root)

        rows = read_jsonl(store.paths.frames_jsonl_path)
        self.assertEqual(len(rows), 1)
        self.assertIn("planes", rows[0])
        self.assertIn("papers", rows[0])
        self.assertEqual(rows[0]["papers"][0]["plane_id"], "plane-0001")
        self.assertEqual(rows[0]["planes"][0]["source"], "paper_center")

    def test_process_segment_uses_nine_scene_probe_points_by_default(self) -> None:
        output_dir = TMP_ROOT / "pipeline-scene-probes"
        shutil.rmtree(output_dir, ignore_errors=True)
        output_dir.mkdir(parents=True, exist_ok=True)
        config = PipelineConfig.from_dict(
            {
                "paper": {"mode": "external_boxes"},
                "runtime": {
                    "root_dir": str(output_dir),
                    "session_id": "session-probes",
                    "detector_image_size": [50, 25],
                },
            }
        )
        pose = PoseSnapshot(
            timestamp_ns=100,
            tracking_state="OK",
            tracking_confidence=99,
            pose_world_xyz=(0.0, 0.0, 0.0),
            pose_world_xyzw=(0.0, 0.0, 0.0, 1.0),
        )
        plane = PlaneSnapshot(
            status="SUCCESS",
            center_xyz=(0.0, 0.0, 0.0),
            normal_xyz=(0.0, 0.0, 1.0),
            equation_abcd=(0.0, 0.0, 1.0, 0.0),
            plane_type="VERTICAL",
        )
        frames = {
            "segment-00001": [
                _Frame(
                    pose=pose,
                    paper_observations=[],
                    points={(30, 20): (2.0, 4.0, 6.0)},
                    depths={(30, 20): 6.0},
                    planes_by_pixel={
                        (30, 20): plane,
                    },
                    image_size=(100, 50),
                )
            ]
        }
        store = _make_store(output_dir, config)
        adapter = FakeAdapter(frames)
        pipeline = MeasurementPipeline(
            config,
            adapter=adapter,
            paper_provider=FakePaperProvider(
                {
                    0: [
                        PaperDetection(
                            frame_idx=0,
                            bbox_xyxy=(10, 5, 20, 15),
                            label="paper",
                            color="red",
                            confidence=0.9,
                        )
                    ]
                }
            ),
            corner_provider=FakeCornerProvider({}),
        )

        pipeline.process_segment(store.paths.root, "segment-00001", finalized=False)

        self.assertEqual(len(adapter.detect_plane_calls), 10)
        self.assertEqual(
            set(adapter.detect_plane_calls[1:]),
            {(16, 8), (50, 8), (82, 8), (16, 24), (50, 24), (82, 24), (16, 41), (50, 41), (82, 41)},
        )
        self.assertEqual(adapter.detect_floor_plane_calls, 0)


if __name__ == "__main__":
    unittest.main()

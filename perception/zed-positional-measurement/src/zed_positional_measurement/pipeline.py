from __future__ import annotations

import multiprocessing as mp
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Iterable, Sequence

from .config import PipelineConfig
from .exporters import export_measurements_csv, export_measurements_jsonl
from .geometry import (
    clamp_point,
    euclidean_distance,
    nearest_point,
    scale_bbox_xyxy,
    scale_point,
    vector_alignment_angle_deg,
    world_to_camera_point,
)
from .models import (
    CornerDetection,
    FramePaperRecord,
    FramePlaneRecord,
    FrameRecord,
    MeasurementRecord,
    PaperDetection,
    PlaneSource,
    PoseRecord,
    SegmentManifestEntry,
    TrackingMode,
)
from .providers import SegmentJsonlCornerProvider, SegmentJsonlPaperProvider
from .sdk import DetectionContext, FrameBuffers, PaperObservation, PlaneSnapshot, PoseSnapshot, ZedSdkAdapter
from .storage import SessionStore, utc_time_ns, write_jsonl


def _worker_main(config_dict: dict[str, Any], session_root: str, segment_queue: Any) -> None:
    pipeline = MeasurementPipeline(PipelineConfig.from_dict(config_dict))
    while True:
        try:
            segment_id = segment_queue.get()
        except (EOFError, OSError):
            break
        if segment_id is None:
            break
        pipeline.process_segment(session_root, str(segment_id), finalized=False)


@dataclass(frozen=True)
class _PlaneCandidate:
    source: PlaneSource
    seed_uv: tuple[int, int] | None
    plane: PlaneSnapshot
    paper_index: int | None = None


class MeasurementPipeline:
    def __init__(
        self,
        config: PipelineConfig,
        *,
        adapter: ZedSdkAdapter | None = None,
        paper_provider: SegmentJsonlPaperProvider | None = None,
        corner_provider: SegmentJsonlCornerProvider | None = None,
    ) -> None:
        self.config = config
        self.adapter = adapter or ZedSdkAdapter()
        self.paper_provider = paper_provider or SegmentJsonlPaperProvider(
            Path(config.paper.external_detections_dir) if config.paper.external_detections_dir else None
        )
        self.corner_provider = corner_provider or SegmentJsonlCornerProvider(
            Path(config.corners.detections_dir) if config.corners.detections_dir else None
        )

    def run_live_session(self) -> Path:
        self._validate_for_processing(process_in_background=True, finalize_at_end=True)
        store = SessionStore.create(self.config)
        self.record_session(store.paths.root, process_in_background=True, finalize_at_end=True)
        return store.paths.root

    def record_session(
        self,
        session_root: Path | str | None = None,
        *,
        process_in_background: bool,
        finalize_at_end: bool,
    ) -> Path:
        store = SessionStore.open(session_root) if session_root is not None else SessionStore.create(self.config)
        self._validate_for_processing(process_in_background=process_in_background, finalize_at_end=finalize_at_end)
        camera = self.adapter.open_live_camera(self.config.recording)
        runtime = self.adapter.build_runtime_parameters()
        recording_enabled = self.config.recording.enable_svo_recording

        ctx: Any | None = None
        worker: Any | None = None
        segment_queue: Any | None = None
        if process_in_background:
            ctx = mp.get_context("spawn") if self.config.runtime.process_spawn else mp.get_context()
            segment_queue = ctx.Queue()
            worker = ctx.Process(target=_worker_main, args=(self.config.to_dict(), str(store.paths.root), segment_queue))
            worker.start()

        current_segment_id: str | None = None
        current_started_ns = 0
        current_frame_count = 0
        segment_duration_ns = self.config.recording.segment_duration_s * 1_000_000_000

        def close_segment() -> None:
            nonlocal current_segment_id, current_started_ns, current_frame_count
            if not recording_enabled or current_segment_id is None:
                return
            self.adapter.disable_recording(camera)
            entry = SegmentManifestEntry(
                session_id=store.paths.session_id,
                segment_id=current_segment_id,
                svo_path=str(store.paths.segment_svo_path(current_segment_id)),
                status="recorded",
                started_at_ns=current_started_ns,
                closed_at_ns=utc_time_ns(),
                frame_count=current_frame_count,
            )
            store.write_segment_manifest(entry)
            if segment_queue is not None:
                segment_queue.put(current_segment_id)
            current_segment_id = None
            current_started_ns = 0
            current_frame_count = 0

        try:
            while True:
                if recording_enabled and current_segment_id is None:
                    current_segment_id = store.next_segment_id()
                    current_started_ns = utc_time_ns()
                    self.adapter.enable_recording(
                        camera,
                        store.paths.segment_svo_path(current_segment_id),
                        self.config.recording,
                    )
                status = self.adapter.grab(camera, runtime)
                if status == self.adapter.success_code():
                    if recording_enabled:
                        current_frame_count += 1
                    if recording_enabled and utc_time_ns() - current_started_ns >= segment_duration_ns:
                        close_segment()
                    continue
                raise RuntimeError(f"Camera grab failed while recording: {status}")
        except KeyboardInterrupt:
            if recording_enabled:
                close_segment()
        finally:
            if recording_enabled and current_segment_id is not None:
                close_segment()
            self.adapter.close_camera(camera)
            if segment_queue is not None:
                segment_queue.put(None)
            if worker is not None:
                worker.join()
        if finalize_at_end:
            self.finalize_session(store.paths.root)
        return store.paths.root

    def process_session(self, session_root: Path | str, *, finalized: bool = False) -> None:
        self._validate_for_processing(process_in_background=False, finalize_at_end=True)
        store = SessionStore.open(session_root)
        if finalized:
            segments = store.list_segments()
        else:
            segments = store.pending_segments(finalized=False)
        for entry in segments:
            self.process_segment(store.paths.root, entry.segment_id, finalized=finalized)

    def finalize_session(self, session_root: Path | str) -> None:
        self._validate_for_processing(process_in_background=False, finalize_at_end=True)
        store = SessionStore.open(session_root)
        for entry in store.list_segments():
            self.process_segment(store.paths.root, entry.segment_id, finalized=True)
        self.export_session(store.paths.root)

    def process_segment(self, session_root: Path | str, segment_id: str, *, finalized: bool) -> None:
        store = SessionStore.open(session_root)
        entry = store.read_segment_manifest(segment_id)
        tracking_mode = self._tracking_mode_for_phase(store, finalized)
        area_path = self._area_path_for_mode(store, tracking_mode)
        frame_offset = self._segment_frame_offset(store, segment_id)
        store.set_segment_status(segment_id, "processing")

        camera = self.adapter.open_svo_camera(self.config.recording, entry.svo_file)
        runtime = self.adapter.build_runtime_parameters()
        detection_context = DetectionContext(enabled=False, mode="none")
        try:
            self.adapter.enable_tracking(camera, self.config.tracking, mode=tracking_mode, area_path=area_path)
            detection_context = self.adapter.configure_paper_detection(camera, self.config.paper)
            pose_rows: list[PoseRecord] = []
            measurement_rows: list[MeasurementRecord] = []
            frame_rows: list[FrameRecord] = []
            buffers = self.adapter.create_frame_buffers()
            local_frame_idx = 0
            while True:
                status = self.adapter.grab(camera, runtime)
                if status == self.adapter.success_code():
                    self.adapter.populate_frame_buffers(camera, buffers)
                    pose = self.adapter.pose_snapshot(camera)
                    frame_idx = frame_offset + local_frame_idx
                    pose_rows.append(
                        PoseRecord(
                            session_id=store.paths.session_id,
                            segment_id=segment_id,
                            frame_idx=frame_idx,
                            timestamp_ns=pose.timestamp_ns,
                            tracking_state=pose.tracking_state,
                            tracking_confidence=pose.tracking_confidence,
                            pose_world_xyz=pose.pose_world_xyz,
                            pose_world_xyzw=pose.pose_world_xyzw,
                        )
                    )
                    observations = self._paper_observations(
                        camera,
                        buffers,
                        detection_context,
                        segment_id,
                        local_frame_idx,
                    )
                    frame_record, paper_measurements = self._frame_record_and_paper_measurements(
                        camera=camera,
                        buffers=buffers,
                        pose=pose,
                        session_id=store.paths.session_id,
                        segment_id=segment_id,
                        frame_idx=frame_idx,
                        observations=observations,
                    )
                    frame_rows.append(frame_record)
                    measurement_rows.extend(paper_measurements)
                    measurement_rows.extend(
                        self._corner_measurements(
                            camera,
                            buffers,
                            pose,
                            store.paths.session_id,
                            segment_id,
                            local_frame_idx,
                            frame_idx,
                        )
                    )
                    local_frame_idx += 1
                    continue
                if status == self.adapter.end_of_svo_code():
                    break
                raise RuntimeError(f"SVO processing failed for {segment_id}: {status}")

            store.write_pose_cache(segment_id, finalized=finalized, rows=pose_rows)
            store.write_measurement_cache(segment_id, finalized=finalized, rows=measurement_rows)
            store.write_frame_cache(segment_id, finalized=finalized, rows=frame_rows)
            if detection_context.enabled:
                self.adapter.disable_paper_detection(camera)
            save_area = store.paths.area_map_path if tracking_mode == "vslam_map" else None
            self.adapter.disable_tracking(camera, area_path=save_area)
            store.set_segment_status(segment_id, "finalized" if finalized else "processed", finalized=finalized)
        except Exception as exc:
            if detection_context.enabled:
                try:
                    self.adapter.disable_paper_detection(camera)
                except Exception:
                    pass
            try:
                self.adapter.disable_tracking(camera, area_path=None)
            except Exception:
                pass
            store.set_segment_status(segment_id, "failed", error_message=str(exc))
            raise
        finally:
            self.adapter.close_camera(camera)

    def export_session(self, session_root: Path | str) -> None:
        store = SessionStore.open(session_root)
        measurements: list[MeasurementRecord] = []
        frames: list[FrameRecord] = []
        for entry in store.list_segments():
            if entry.final_measurement_path:
                measurements.extend(store.read_measurement_cache(entry.segment_id, finalized=True))
            if entry.final_frame_path:
                frames.extend(store.read_frame_cache(entry.segment_id, finalized=True))
        write_jsonl(store.paths.frames_jsonl_path, (frame.to_dict() for frame in frames))
        export_measurements_jsonl(store.paths.measurements_jsonl_path, measurements)
        export_measurements_csv(store.paths.measurements_csv_path, measurements)

    def _tracking_mode_for_phase(self, store: SessionStore, finalized: bool) -> TrackingMode:
        if finalized:
            if self.config.tracking.mode in {"vslam_map", "vslam_localize"} and store.paths.area_map_path.exists():
                return "vslam_localize"
            if self.config.tracking.mode == "vslam_localize":
                return "vio"
        if self.config.tracking.mode == "vslam_localize" and not store.paths.area_map_path.exists():
            return "vslam_map"
        return self.config.tracking.mode

    def _segment_frame_offset(self, store: SessionStore, segment_id: str) -> int:
        offset = 0
        for entry in store.list_segments():
            if entry.segment_id == segment_id:
                break
            offset += entry.frame_count
        return offset

    def _validate_for_processing(self, *, process_in_background: bool, finalize_at_end: bool) -> None:
        if self.config.paper.mode == "native_yolo" and not self.config.paper.native_yolo_onnx_path:
            raise ValueError("paper.native_yolo_onnx_path must be set when paper.mode='native_yolo'")
        if (process_in_background or finalize_at_end) and not self.config.recording.enable_svo_recording:
            raise ValueError(
                "The current replay-based pipeline requires recording.enable_svo_recording=true "
                "when processing or finalization is enabled."
            )

    def _area_path_for_mode(self, store: SessionStore, mode: TrackingMode) -> Path | None:
        if mode == "vio":
            return None
        return store.paths.area_map_path

    def _paper_observations(
        self,
        camera: Any,
        buffers: FrameBuffers,
        context: DetectionContext,
        segment_id: str,
        local_frame_idx: int,
    ) -> list[PaperObservation]:
        if self.config.paper.mode == "external_boxes":
            raw_detections = self.paper_provider.get(segment_id, local_frame_idx)
            detections = [self._scale_paper_detection(detection, buffers.image_size) for detection in raw_detections]
        else:
            detections = []
        return self.adapter.paper_observations(camera, context, self.config.paper, detections)

    def _frame_record_and_paper_measurements(
        self,
        *,
        camera: Any,
        buffers: FrameBuffers,
        pose: PoseSnapshot,
        session_id: str,
        segment_id: str,
        frame_idx: int,
        observations: Sequence[PaperObservation],
    ) -> tuple[FrameRecord, list[MeasurementRecord]]:
        paper_rows: list[tuple[PaperObservation, tuple[float, float, float] | None, float | None, PlaneSnapshot]] = []
        plane_candidates: list[_PlaneCandidate] = []
        for paper_index, observation in enumerate(observations):
            center_u, center_v = observation.center_pixel
            point_world = self.adapter.sample_world_point(buffers, center_u, center_v)
            depth_m = self.adapter.sample_depth(buffers, center_u, center_v)
            plane = self._plane_for_pose(camera, pose, center_u, center_v)
            paper_rows.append((observation, point_world, depth_m, plane))
            if plane.status == "SUCCESS":
                plane_candidates.append(
                    _PlaneCandidate(
                        source="paper_center",
                        seed_uv=(center_u, center_v),
                        plane=plane,
                        paper_index=paper_index,
                    )
                )

        for probe_pixel in self._scene_probe_pixels(buffers.image_size):
            plane = self._plane_for_pose(camera, pose, probe_pixel[0], probe_pixel[1])
            if plane.status == "SUCCESS":
                plane_candidates.append(
                    _PlaneCandidate(
                        source="scene_probe",
                        seed_uv=probe_pixel,
                        plane=plane,
                    )
                )

        floor_plane = self._floor_plane_for_pose(camera, pose)
        if floor_plane is not None and floor_plane.status == "SUCCESS":
            plane_candidates.append(
                _PlaneCandidate(
                    source="floor",
                    seed_uv=None,
                    plane=floor_plane,
                )
            )

        planes, paper_plane_ids = self._frame_planes_from_candidates(plane_candidates)
        frame_papers: list[FramePaperRecord] = []
        measurement_rows: list[MeasurementRecord] = []
        for paper_index, (observation, point_world, depth_m, plane) in enumerate(paper_rows):
            center_u, center_v = observation.center_pixel
            measurement = self._measurement_record(
                pose=pose,
                session_id=session_id,
                segment_id=segment_id,
                frame_idx=frame_idx,
                entity_type="paper",
                label=observation.label,
                color_or_corner_id=observation.color,
                detector_confidence=observation.confidence,
                pixel_u=center_u,
                pixel_v=center_v,
                bbox_xyxy=observation.bbox_xyxy,
                point_world=point_world,
                depth_m=depth_m,
                plane=plane,
                plane_id=paper_plane_ids.get(paper_index),
            )
            measurement_rows.append(measurement)
            frame_papers.append(self._frame_paper_record(measurement))

        frame_record = FrameRecord(
            session_id=session_id,
            frame_idx=frame_idx,
            timestamp_ns=pose.timestamp_ns,
            tracking_state=pose.tracking_state,
            tracking_confidence=pose.tracking_confidence,
            pose_world_xyz=pose.pose_world_xyz,
            pose_world_xyzw=pose.pose_world_xyzw,
            planes=tuple(planes),
            papers=tuple(frame_papers),
        )
        return frame_record, measurement_rows

    def _scene_probe_pixels(self, image_size: tuple[int, int]) -> tuple[tuple[int, int], ...]:
        if not self.config.plane_detection.enable_scene_probes:
            return tuple()
        width, height = image_size
        if width <= 0 or height <= 0:
            return tuple()
        pixels: list[tuple[int, int]] = []
        seen: set[tuple[int, int]] = set()
        for u_norm, v_norm in self.config.plane_detection.scene_probe_points_normalized:
            u = int(round(min(max(float(u_norm), 0.0), 1.0) * (width - 1)))
            v = int(round(min(max(float(v_norm), 0.0), 1.0) * (height - 1)))
            pixel = clamp_point((u, v), image_size)
            if pixel not in seen:
                seen.add(pixel)
                pixels.append(pixel)
        return tuple(pixels)

    def _floor_plane_for_pose(self, camera: Any, pose: PoseSnapshot) -> PlaneSnapshot | None:
        if not self.config.plane_detection.include_floor_plane:
            return None
        if not pose.tracking_state.endswith("OK"):
            return PlaneSnapshot(
                status="skipped_tracking_not_ok",
                center_xyz=None,
                normal_xyz=None,
                equation_abcd=None,
                plane_type=None,
            )
        return self.adapter.detect_floor_plane(camera)

    def _frame_planes_from_candidates(
        self,
        candidates: Sequence[_PlaneCandidate],
    ) -> tuple[list[FramePlaneRecord], dict[int, str]]:
        planes: list[FramePlaneRecord] = []
        paper_plane_ids: dict[int, str] = {}
        for candidate in candidates:
            plane_id = None
            for plane in planes:
                if self._planes_match(plane, candidate.plane):
                    plane_id = plane.plane_id
                    break
            if plane_id is None:
                plane_id = f"plane-{len(planes) + 1:04d}"
                planes.append(
                    FramePlaneRecord(
                        plane_id=plane_id,
                        source=candidate.source,
                        seed_uv=candidate.seed_uv,
                        center_world_xyz=candidate.plane.center_xyz,
                        normal_world_xyz=candidate.plane.normal_xyz,
                        equation_abcd=candidate.plane.equation_abcd,
                        plane_type=candidate.plane.plane_type,
                    )
                )
            if candidate.paper_index is not None:
                paper_plane_ids[candidate.paper_index] = plane_id
        return planes, paper_plane_ids

    def _planes_match(self, existing: FramePlaneRecord, candidate: PlaneSnapshot) -> bool:
        angle_deg = vector_alignment_angle_deg(existing.normal_world_xyz, candidate.normal_xyz)
        if angle_deg is None or angle_deg > self.config.plane_detection.dedupe_normal_angle_deg:
            return False
        if existing.center_world_xyz is None or candidate.center_xyz is None:
            return False
        return (
            euclidean_distance(existing.center_world_xyz, candidate.center_xyz)
            <= self.config.plane_detection.dedupe_center_distance_m
        )

    def _corner_measurements(
        self,
        camera: Any,
        buffers: FrameBuffers,
        pose: PoseSnapshot,
        session_id: str,
        segment_id: str,
        local_frame_idx: int,
        frame_idx: int,
    ) -> list[MeasurementRecord]:
        raw_corners = self.corner_provider.get(segment_id, local_frame_idx)
        corners = [self._scale_corner_detection(detection, buffers.image_size) for detection in raw_corners]
        records: list[MeasurementRecord] = []
        for corner in corners:
            camera_position = pose.pose_world_xyz if pose.pose_world_xyz is not None else (0.0, 0.0, 0.0)
            point_world = self._nearest_patch_point(buffers, (corner.u, corner.v), camera_position)
            records.append(
                self._measurement_record(
                    pose=pose,
                    session_id=session_id,
                    segment_id=segment_id,
                    frame_idx=frame_idx,
                    entity_type="corner",
                    label="corner",
                    color_or_corner_id=corner.corner_id,
                    detector_confidence=corner.confidence,
                    pixel_u=corner.u,
                    pixel_v=corner.v,
                    bbox_xyxy=None,
                    point_world=point_world,
                    depth_m=self.adapter.sample_depth(buffers, corner.u, corner.v),
                    plane=self._plane_for_pose(camera, pose, corner.u, corner.v),
                    plane_id=None,
                )
            )
        return records

    def _nearest_patch_point(
        self,
        buffers: FrameBuffers,
        point_xy: tuple[int, int],
        camera_position: Sequence[float],
    ) -> tuple[float, float, float] | None:
        return nearest_point(
            (point for point in self._patch_points(buffers, point_xy) if point is not None),
            camera_position,
        )

    def _patch_points(self, buffers: FrameBuffers, point_xy: tuple[int, int]) -> list[tuple[float, float, float] | None]:
        radius = self.config.corners.patch_radius_px
        width, height = buffers.image_size
        clamped = clamp_point(point_xy, buffers.image_size)
        points: list[tuple[float, float, float] | None] = []
        for v in range(max(0, clamped[1] - radius), min(height, clamped[1] + radius + 1)):
            for u in range(max(0, clamped[0] - radius), min(width, clamped[0] + radius + 1)):
                points.append(self.adapter.sample_world_point(buffers, u, v))
        return points

    def _plane_for_pose(self, camera: Any, pose: PoseSnapshot, u: int, v: int) -> PlaneSnapshot:
        if pose.tracking_state.endswith("OK"):
            return self.adapter.detect_plane(camera, u, v)
        return PlaneSnapshot(status="skipped_tracking_not_ok", center_xyz=None, normal_xyz=None, equation_abcd=None, plane_type=None)

    def _measurement_record(
        self,
        *,
        pose: PoseSnapshot,
        session_id: str,
        segment_id: str,
        frame_idx: int,
        entity_type: str,
        label: str,
        color_or_corner_id: str,
        detector_confidence: float,
        pixel_u: int,
        pixel_v: int,
        bbox_xyxy: tuple[int, int, int, int] | None,
        point_world: tuple[float, float, float] | None,
        depth_m: float | None,
        plane: PlaneSnapshot,
        plane_id: str | None,
    ) -> MeasurementRecord:
        quality_flags: list[str] = []
        point_camera = None
        pose_is_valid = pose.pose_world_xyz is not None and pose.pose_world_xyzw is not None and pose.tracking_state.endswith("OK")
        if not pose_is_valid:
            quality_flags.append("pose_invalid")
        if point_world is None:
            quality_flags.append("point_invalid")
        if plane.status != "SUCCESS":
            quality_flags.append("plane_unavailable")
        if depth_m is None:
            quality_flags.append("depth_invalid")
        if pose_is_valid and point_world is not None:
            point_camera = world_to_camera_point(point_world, pose.pose_world_xyz or (0, 0, 0), pose.pose_world_xyzw or (0, 0, 0, 1))
            if depth_m is not None:
                quality_flags.append("clean")
        return MeasurementRecord(
            session_id=session_id,
            segment_id=segment_id,
            frame_idx=frame_idx,
            timestamp_ns=pose.timestamp_ns,
            entity_type=entity_type,  # type: ignore[arg-type]
            label=label,
            color_or_corner_id=color_or_corner_id,
            detector_confidence=detector_confidence,
            pixel_u=pixel_u,
            pixel_v=pixel_v,
            bbox_xyxy=bbox_xyxy,
            pose_world_xyz=pose.pose_world_xyz,
            pose_world_xyzw=pose.pose_world_xyzw,
            tracking_state=pose.tracking_state,
            tracking_confidence=pose.tracking_confidence,
            point_world_xyz=point_world,
            point_camera_xyz=point_camera,
            depth_m=depth_m,
            plane_id=plane_id,
            plane_status=plane.status,
            plane_center_xyz=plane.center_xyz,
            plane_normal_xyz=plane.normal_xyz,
            plane_equation_abcd=plane.equation_abcd,
            plane_type=plane.plane_type,
            quality_flags=tuple(quality_flags),
        )

    def _frame_paper_record(self, measurement: MeasurementRecord) -> FramePaperRecord:
        return FramePaperRecord(
            label=measurement.label,
            color=measurement.color_or_corner_id,
            detector_confidence=measurement.detector_confidence,
            center_uv=(measurement.pixel_u, measurement.pixel_v),
            depth_m=measurement.depth_m,
            point_world_xyz=measurement.point_world_xyz,
            point_camera_xyz=measurement.point_camera_xyz,
            plane_id=measurement.plane_id,
            plane_query_status=measurement.plane_status,
            quality_flags=measurement.quality_flags,
        )

    def _scale_paper_detection(self, detection: PaperDetection, image_size: tuple[int, int]) -> PaperDetection:
        detector_size = self.config.runtime.detector_image_size
        if detector_size is None or detector_size == image_size:
            return detection
        return replace(detection, bbox_xyxy=scale_bbox_xyxy(detection.bbox_xyxy, detector_size, image_size))

    def _scale_corner_detection(self, detection: CornerDetection, image_size: tuple[int, int]) -> CornerDetection:
        detector_size = self.config.runtime.detector_image_size
        if detector_size is None or detector_size == image_size:
            return detection
        u, v = scale_point((detection.u, detection.v), detector_size, image_size)
        return replace(detection, u=u, v=v)

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal

TrackingMode = Literal["vio", "vslam_map", "vslam_localize", "gnss_fusion"]
SegmentStatus = Literal["recorded", "processing", "processed", "finalized", "failed"]
EntityType = Literal["paper", "corner"]
PlaneSource = Literal["paper_center", "scene_probe", "floor"]


def _to_float_tuple(values: Any, length: int) -> tuple[float, ...] | None:
    if values is None:
        return None
    result = tuple(float(v) for v in values)
    if len(result) != length:
        raise ValueError(f"Expected {length} values, got {len(result)}")
    return result


def _to_int_tuple(values: Any, length: int) -> tuple[int, ...] | None:
    if values is None:
        return None
    result = tuple(int(v) for v in values)
    if len(result) != length:
        raise ValueError(f"Expected {length} values, got {len(result)}")
    return result


@dataclass(frozen=True)
class PaperDetection:
    frame_idx: int
    bbox_xyxy: tuple[int, int, int, int]
    label: str
    color: str
    confidence: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "frame_idx": self.frame_idx,
            "bbox_xyxy": list(self.bbox_xyxy),
            "label": self.label,
            "color": self.color,
            "confidence": self.confidence,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "PaperDetection":
        return cls(
            frame_idx=int(data["frame_idx"]),
            bbox_xyxy=_to_int_tuple(data["bbox_xyxy"], 4),  # type: ignore[arg-type]
            label=str(data["label"]),
            color=str(data["color"]),
            confidence=float(data["confidence"]),
        )


@dataclass(frozen=True)
class CornerDetection:
    frame_idx: int
    corner_id: str
    u: int
    v: int
    confidence: float

    def to_dict(self) -> dict[str, Any]:
        return {
            "frame_idx": self.frame_idx,
            "corner_id": self.corner_id,
            "u": self.u,
            "v": self.v,
            "confidence": self.confidence,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "CornerDetection":
        return cls(
            frame_idx=int(data["frame_idx"]),
            corner_id=str(data["corner_id"]),
            u=int(data["u"]),
            v=int(data["v"]),
            confidence=float(data["confidence"]),
        )


@dataclass(frozen=True)
class PoseRecord:
    session_id: str
    segment_id: str
    frame_idx: int
    timestamp_ns: int
    tracking_state: str
    tracking_confidence: int
    pose_world_xyz: tuple[float, float, float] | None
    pose_world_xyzw: tuple[float, float, float, float] | None

    def to_dict(self) -> dict[str, Any]:
        return {
            "session_id": self.session_id,
            "segment_id": self.segment_id,
            "frame_idx": self.frame_idx,
            "timestamp_ns": self.timestamp_ns,
            "tracking_state": self.tracking_state,
            "tracking_confidence": self.tracking_confidence,
            "pose_world_xyz": list(self.pose_world_xyz) if self.pose_world_xyz else None,
            "pose_world_xyzw": list(self.pose_world_xyzw) if self.pose_world_xyzw else None,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "PoseRecord":
        return cls(
            session_id=str(data["session_id"]),
            segment_id=str(data["segment_id"]),
            frame_idx=int(data["frame_idx"]),
            timestamp_ns=int(data["timestamp_ns"]),
            tracking_state=str(data["tracking_state"]),
            tracking_confidence=int(data["tracking_confidence"]),
            pose_world_xyz=_to_float_tuple(data.get("pose_world_xyz"), 3),  # type: ignore[arg-type]
            pose_world_xyzw=_to_float_tuple(data.get("pose_world_xyzw"), 4),  # type: ignore[arg-type]
        )


@dataclass(frozen=True)
class MeasurementRecord:
    session_id: str
    segment_id: str
    frame_idx: int
    timestamp_ns: int
    entity_type: EntityType
    label: str
    color_or_corner_id: str
    detector_confidence: float
    pixel_u: int
    pixel_v: int
    bbox_xyxy: tuple[int, int, int, int] | None
    pose_world_xyz: tuple[float, float, float] | None
    pose_world_xyzw: tuple[float, float, float, float] | None
    tracking_state: str
    tracking_confidence: int
    point_world_xyz: tuple[float, float, float] | None
    point_camera_xyz: tuple[float, float, float] | None
    depth_m: float | None
    plane_status: str
    plane_center_xyz: tuple[float, float, float] | None = None
    plane_normal_xyz: tuple[float, float, float] | None = None
    plane_equation_abcd: tuple[float, float, float, float] | None = None
    plane_type: str | None = None
    plane_id: str | None = None
    quality_flags: tuple[str, ...] = field(default_factory=tuple)

    @property
    def is_clean(self) -> bool:
        return "clean" in self.quality_flags

    def to_dict(self) -> dict[str, Any]:
        return {
            "session_id": self.session_id,
            "segment_id": self.segment_id,
            "frame_idx": self.frame_idx,
            "timestamp_ns": self.timestamp_ns,
            "entity_type": self.entity_type,
            "label": self.label,
            "color_or_corner_id": self.color_or_corner_id,
            "detector_confidence": self.detector_confidence,
            "pixel_u": self.pixel_u,
            "pixel_v": self.pixel_v,
            "bbox_xyxy": list(self.bbox_xyxy) if self.bbox_xyxy else None,
            "pose_world_xyz": list(self.pose_world_xyz) if self.pose_world_xyz else None,
            "pose_world_xyzw": list(self.pose_world_xyzw) if self.pose_world_xyzw else None,
            "tracking_state": self.tracking_state,
            "tracking_confidence": self.tracking_confidence,
            "point_world_xyz": list(self.point_world_xyz) if self.point_world_xyz else None,
            "point_camera_xyz": list(self.point_camera_xyz) if self.point_camera_xyz else None,
            "depth_m": self.depth_m,
            "plane_id": self.plane_id,
            "plane_status": self.plane_status,
            "plane_center_xyz": list(self.plane_center_xyz) if self.plane_center_xyz else None,
            "plane_normal_xyz": list(self.plane_normal_xyz) if self.plane_normal_xyz else None,
            "plane_equation_abcd": list(self.plane_equation_abcd) if self.plane_equation_abcd else None,
            "plane_type": self.plane_type,
            "quality_flags": list(self.quality_flags),
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "MeasurementRecord":
        return cls(
            session_id=str(data["session_id"]),
            segment_id=str(data["segment_id"]),
            frame_idx=int(data["frame_idx"]),
            timestamp_ns=int(data["timestamp_ns"]),
            entity_type=str(data["entity_type"]),  # type: ignore[arg-type]
            label=str(data["label"]),
            color_or_corner_id=str(data["color_or_corner_id"]),
            detector_confidence=float(data["detector_confidence"]),
            pixel_u=int(data["pixel_u"]),
            pixel_v=int(data["pixel_v"]),
            bbox_xyxy=_to_int_tuple(data.get("bbox_xyxy"), 4),  # type: ignore[arg-type]
            pose_world_xyz=_to_float_tuple(data.get("pose_world_xyz"), 3),  # type: ignore[arg-type]
            pose_world_xyzw=_to_float_tuple(data.get("pose_world_xyzw"), 4),  # type: ignore[arg-type]
            tracking_state=str(data["tracking_state"]),
            tracking_confidence=int(data["tracking_confidence"]),
            point_world_xyz=_to_float_tuple(data.get("point_world_xyz"), 3),  # type: ignore[arg-type]
            point_camera_xyz=_to_float_tuple(data.get("point_camera_xyz"), 3),  # type: ignore[arg-type]
            depth_m=float(data["depth_m"]) if data.get("depth_m") is not None else None,
            plane_id=str(data["plane_id"]) if data.get("plane_id") is not None else None,
            plane_status=str(data["plane_status"]),
            plane_center_xyz=_to_float_tuple(data.get("plane_center_xyz"), 3),  # type: ignore[arg-type]
            plane_normal_xyz=_to_float_tuple(data.get("plane_normal_xyz"), 3),  # type: ignore[arg-type]
            plane_equation_abcd=_to_float_tuple(data.get("plane_equation_abcd"), 4),  # type: ignore[arg-type]
            plane_type=str(data["plane_type"]) if data.get("plane_type") is not None else None,
            quality_flags=tuple(str(flag) for flag in data.get("quality_flags", [])),
        )


@dataclass(frozen=True)
class FramePlaneRecord:
    plane_id: str
    source: PlaneSource
    seed_uv: tuple[int, int] | None
    center_world_xyz: tuple[float, float, float] | None
    normal_world_xyz: tuple[float, float, float] | None
    equation_abcd: tuple[float, float, float, float] | None
    plane_type: str | None

    def to_dict(self) -> dict[str, Any]:
        return {
            "plane_id": self.plane_id,
            "source": self.source,
            "seed_uv": list(self.seed_uv) if self.seed_uv else None,
            "center_world_xyz": list(self.center_world_xyz) if self.center_world_xyz else None,
            "normal_world_xyz": list(self.normal_world_xyz) if self.normal_world_xyz else None,
            "equation_abcd": list(self.equation_abcd) if self.equation_abcd else None,
            "type": self.plane_type,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "FramePlaneRecord":
        return cls(
            plane_id=str(data["plane_id"]),
            source=str(data["source"]),  # type: ignore[arg-type]
            seed_uv=_to_int_tuple(data.get("seed_uv"), 2),  # type: ignore[arg-type]
            center_world_xyz=_to_float_tuple(data.get("center_world_xyz"), 3),  # type: ignore[arg-type]
            normal_world_xyz=_to_float_tuple(data.get("normal_world_xyz"), 3),  # type: ignore[arg-type]
            equation_abcd=_to_float_tuple(data.get("equation_abcd"), 4),  # type: ignore[arg-type]
            plane_type=str(data["type"]) if data.get("type") is not None else None,
        )


@dataclass(frozen=True)
class FramePaperRecord:
    label: str
    color: str
    detector_confidence: float
    center_uv: tuple[int, int]
    depth_m: float | None
    point_world_xyz: tuple[float, float, float] | None
    point_camera_xyz: tuple[float, float, float] | None
    plane_id: str | None
    plane_query_status: str
    quality_flags: tuple[str, ...] = field(default_factory=tuple)

    @property
    def is_clean(self) -> bool:
        return "clean" in self.quality_flags

    def to_dict(self) -> dict[str, Any]:
        return {
            "label": self.label,
            "color": self.color,
            "detector_confidence": self.detector_confidence,
            "center_uv": list(self.center_uv),
            "depth_m": self.depth_m,
            "point_world_xyz": list(self.point_world_xyz) if self.point_world_xyz else None,
            "point_camera_xyz": list(self.point_camera_xyz) if self.point_camera_xyz else None,
            "plane_id": self.plane_id,
            "plane_query_status": self.plane_query_status,
            "quality_flags": list(self.quality_flags),
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "FramePaperRecord":
        return cls(
            label=str(data["label"]),
            color=str(data["color"]),
            detector_confidence=float(data["detector_confidence"]),
            center_uv=_to_int_tuple(data["center_uv"], 2),  # type: ignore[arg-type]
            depth_m=float(data["depth_m"]) if data.get("depth_m") is not None else None,
            point_world_xyz=_to_float_tuple(data.get("point_world_xyz"), 3),  # type: ignore[arg-type]
            point_camera_xyz=_to_float_tuple(data.get("point_camera_xyz"), 3),  # type: ignore[arg-type]
            plane_id=str(data["plane_id"]) if data.get("plane_id") is not None else None,
            plane_query_status=str(data["plane_query_status"]),
            quality_flags=tuple(str(flag) for flag in data.get("quality_flags", [])),
        )


@dataclass(frozen=True)
class FrameRecord:
    session_id: str
    frame_idx: int
    timestamp_ns: int
    tracking_state: str
    tracking_confidence: int
    pose_world_xyz: tuple[float, float, float] | None
    pose_world_xyzw: tuple[float, float, float, float] | None
    planes: tuple[FramePlaneRecord, ...] = field(default_factory=tuple)
    papers: tuple[FramePaperRecord, ...] = field(default_factory=tuple)

    def to_dict(self) -> dict[str, Any]:
        return {
            "session_id": self.session_id,
            "frame_idx": self.frame_idx,
            "timestamp_ns": self.timestamp_ns,
            "tracking_state": self.tracking_state,
            "tracking_confidence": self.tracking_confidence,
            "pose_world_xyz": list(self.pose_world_xyz) if self.pose_world_xyz else None,
            "pose_world_xyzw": list(self.pose_world_xyzw) if self.pose_world_xyzw else None,
            "planes": [plane.to_dict() for plane in self.planes],
            "papers": [paper.to_dict() for paper in self.papers],
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "FrameRecord":
        return cls(
            session_id=str(data["session_id"]),
            frame_idx=int(data["frame_idx"]),
            timestamp_ns=int(data["timestamp_ns"]),
            tracking_state=str(data["tracking_state"]),
            tracking_confidence=int(data["tracking_confidence"]),
            pose_world_xyz=_to_float_tuple(data.get("pose_world_xyz"), 3),  # type: ignore[arg-type]
            pose_world_xyzw=_to_float_tuple(data.get("pose_world_xyzw"), 4),  # type: ignore[arg-type]
            planes=tuple(FramePlaneRecord.from_dict(row) for row in data.get("planes", [])),
            papers=tuple(FramePaperRecord.from_dict(row) for row in data.get("papers", [])),
        )


@dataclass(frozen=True)
class SegmentManifestEntry:
    session_id: str
    segment_id: str
    svo_path: str
    status: SegmentStatus
    started_at_ns: int
    closed_at_ns: int
    frame_count: int
    provisional_pose_path: str | None = None
    provisional_measurement_path: str | None = None
    provisional_frame_path: str | None = None
    final_pose_path: str | None = None
    final_measurement_path: str | None = None
    final_frame_path: str | None = None
    error_message: str | None = None
    updated_at_ns: int | None = None

    @property
    def svo_file(self) -> Path:
        return Path(self.svo_path)

    def to_dict(self) -> dict[str, Any]:
        return {
            "session_id": self.session_id,
            "segment_id": self.segment_id,
            "svo_path": self.svo_path,
            "status": self.status,
            "started_at_ns": self.started_at_ns,
            "closed_at_ns": self.closed_at_ns,
            "frame_count": self.frame_count,
            "provisional_pose_path": self.provisional_pose_path,
            "provisional_measurement_path": self.provisional_measurement_path,
            "provisional_frame_path": self.provisional_frame_path,
            "final_pose_path": self.final_pose_path,
            "final_measurement_path": self.final_measurement_path,
            "final_frame_path": self.final_frame_path,
            "error_message": self.error_message,
            "updated_at_ns": self.updated_at_ns,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "SegmentManifestEntry":
        return cls(
            session_id=str(data["session_id"]),
            segment_id=str(data["segment_id"]),
            svo_path=str(data["svo_path"]),
            status=str(data["status"]),  # type: ignore[arg-type]
            started_at_ns=int(data["started_at_ns"]),
            closed_at_ns=int(data["closed_at_ns"]),
            frame_count=int(data.get("frame_count", 0)),
            provisional_pose_path=data.get("provisional_pose_path"),
            provisional_measurement_path=data.get("provisional_measurement_path"),
            provisional_frame_path=data.get("provisional_frame_path"),
            final_pose_path=data.get("final_pose_path"),
            final_measurement_path=data.get("final_measurement_path"),
            final_frame_path=data.get("final_frame_path"),
            error_message=data.get("error_message"),
            updated_at_ns=int(data["updated_at_ns"]) if data.get("updated_at_ns") is not None else None,
        )

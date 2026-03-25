from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from .models import TrackingMode


def _default_scene_probe_points_normalized() -> tuple[tuple[float, float], ...]:
    one_sixth = 1.0 / 6.0
    one_half = 0.5
    five_sixths = 5.0 / 6.0
    return (
        (one_sixth, one_sixth),
        (one_half, one_sixth),
        (five_sixths, one_sixth),
        (one_sixth, one_half),
        (one_half, one_half),
        (five_sixths, one_half),
        (one_sixth, five_sixths),
        (one_half, five_sixths),
        (five_sixths, five_sixths),
    )


@dataclass(frozen=True)
class RecordingConfig:
    enable_svo_recording: bool = True
    resolution: str = "HD1080"
    fps: int = 30
    depth_mode: str = "NEURAL"
    coordinate_system: str = "RIGHT_HANDED_Z_UP_X_FORWARD"
    coordinate_units: str = "METER"
    compression_mode: str = "H264"
    segment_duration_s: int = 20


@dataclass(frozen=True)
class TrackingConfig:
    mode: TrackingMode = "vslam_map"
    positional_tracking_mode: str = "GEN_3"
    enable_imu_fusion: bool = True
    set_gravity_as_origin: bool = True
    set_floor_as_origin: bool = False
    enable_pose_smoothing: bool = False
    enable_2d_ground_mode: bool = False


@dataclass(frozen=True)
class PaperClassConfig:
    label: str
    color: str


@dataclass(frozen=True)
class PaperDetectionConfig:
    mode: str = "native_yolo"
    confidence_threshold: int = 25
    native_yolo_onnx_path: str | None = None
    native_yolo_input_size: tuple[int, int] | None = None
    native_yolo_class_count: int | None = None
    classes: dict[str, PaperClassConfig] = field(default_factory=dict)
    external_detections_dir: str | None = None


@dataclass(frozen=True)
class CornerConfig:
    detections_dir: str | None = None
    patch_radius_px: int = 2


@dataclass(frozen=True)
class PlaneDetectionConfig:
    enable_scene_probes: bool = True
    scene_probe_points_normalized: tuple[tuple[float, float], ...] = field(
        default_factory=_default_scene_probe_points_normalized
    )
    include_floor_plane: bool = False
    dedupe_normal_angle_deg: float = 10.0
    dedupe_center_distance_m: float = 0.4


@dataclass(frozen=True)
class RuntimeConfig:
    root_dir: str = "runs"
    session_id: str | None = None
    process_spawn: bool = True
    detector_image_size: tuple[int, int] | None = None


@dataclass(frozen=True)
class PipelineConfig:
    recording: RecordingConfig = field(default_factory=RecordingConfig)
    tracking: TrackingConfig = field(default_factory=TrackingConfig)
    paper: PaperDetectionConfig = field(default_factory=PaperDetectionConfig)
    corners: CornerConfig = field(default_factory=CornerConfig)
    plane_detection: PlaneDetectionConfig = field(default_factory=PlaneDetectionConfig)
    runtime: RuntimeConfig = field(default_factory=RuntimeConfig)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> "PipelineConfig":
        recording = RecordingConfig(**raw.get("recording", {}))
        tracking = TrackingConfig(**raw.get("tracking", {}))

        raw_paper = dict(raw.get("paper", {}))
        classes = {
            str(key): PaperClassConfig(**value)
            for key, value in raw_paper.pop("classes", {}).items()
        }
        paper = PaperDetectionConfig(classes=classes, **raw_paper)

        corners = CornerConfig(**raw.get("corners", {}))
        raw_plane_detection = dict(raw.get("plane_detection", {}))
        probe_points = tuple(
            tuple(float(value) for value in point)
            for point in raw_plane_detection.pop("scene_probe_points_normalized", _default_scene_probe_points_normalized())
        )
        plane_detection = PlaneDetectionConfig(
            scene_probe_points_normalized=probe_points,
            **raw_plane_detection,
        )
        runtime = RuntimeConfig(**raw.get("runtime", {}))
        return cls(
            recording=recording,
            tracking=tracking,
            paper=paper,
            corners=corners,
            plane_detection=plane_detection,
            runtime=runtime,
        )

    @classmethod
    def from_json_file(cls, path: Path | str) -> "PipelineConfig":
        file_path = Path(path)
        return cls.from_dict(json.loads(file_path.read_text(encoding="utf-8")))

    def write_json(self, path: Path | str) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=2), encoding="utf-8")

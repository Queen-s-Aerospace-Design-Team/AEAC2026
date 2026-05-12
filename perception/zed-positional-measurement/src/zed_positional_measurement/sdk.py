from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

from .config import PaperClassConfig, PaperDetectionConfig, RecordingConfig, TrackingConfig
from .geometry import bbox_center, iou_xyxy
from .models import PaperDetection, TrackingMode


class ZedSdkUnavailableError(RuntimeError):
    pass


@dataclass(frozen=True)
class PoseSnapshot:
    timestamp_ns: int
    tracking_state: str
    tracking_confidence: int
    pose_world_xyz: tuple[float, float, float] | None
    pose_world_xyzw: tuple[float, float, float, float] | None


@dataclass(frozen=True)
class PlaneSnapshot:
    status: str
    center_xyz: tuple[float, float, float] | None
    normal_xyz: tuple[float, float, float] | None
    equation_abcd: tuple[float, float, float, float] | None
    plane_type: str | None


@dataclass(frozen=True)
class PaperObservation:
    bbox_xyxy: tuple[int, int, int, int]
    center_pixel: tuple[int, int]
    confidence: float
    label: str
    color: str
    mask_pixels: tuple[tuple[int, int], ...]


@dataclass
class DetectionContext:
    enabled: bool
    mode: str
    runtime_params: Any | None = None


@dataclass
class FrameBuffers:
    point_cloud_world: Any
    depth_map: Any
    image_size: tuple[int, int]


class ZedSdkAdapter:
    def __init__(self) -> None:
        self._sl = None

    @property
    def sl(self) -> Any:
        if self._sl is None:
            try:
                import pyzed.sl as sl  # type: ignore[import-not-found]
            except ImportError as exc:
                raise ZedSdkUnavailableError(
                    "pyzed.sl is not available. Install the ZED SDK Python bindings to use this pipeline."
                ) from exc
            self._sl = sl
        return self._sl

    def _resolve_enum(self, namespace: Any, *names: str) -> Any:
        for name in names:
            if hasattr(namespace, name):
                return getattr(namespace, name)
        raise AttributeError(f"None of {names!r} exists on {namespace!r}")

    def _set_if_present(self, obj: Any, attr_names: Iterable[str], value: Any) -> None:
        for attr_name in attr_names:
            if hasattr(obj, attr_name):
                setattr(obj, attr_name, value)
                return

    def _enum_from_name(self, namespace: Any, name: str) -> Any:
        aliases = {
            "GEN_3": ("GEN_3", "GEN3"),
            "RIGHT_HANDED_Z_UP_X_FORWARD": (
                "RIGHT_HANDED_Z_UP_X_FORWARD",
                "RIGHT_HANDED_Z_UP_X_FWD",
            ),
            "METER": ("METER", "METERS"),
            "H264": ("H264", "H264_BASED"),
            "H265": ("H265", "H265_BASED"),
            "NEURAL": ("NEURAL",),
        }
        return self._resolve_enum(namespace, *(aliases.get(name, (name,))))

    def _vector_to_tuple(self, value: Any, length: int) -> tuple[float, ...] | None:
        if value is None:
            return None
        if isinstance(value, (list, tuple)):
            seq = value
        elif hasattr(value, "__getitem__"):
            seq = [value[i] for i in range(length)]
        elif hasattr(value, "tolist"):
            seq = value.tolist()
        else:
            return None
        if len(seq) < length:
            return None
        return tuple(float(seq[i]) for i in range(length))

    def _bbox_from_sdk_object(self, sdk_object: Any) -> tuple[int, int, int, int]:
        corners = getattr(sdk_object, "bounding_box_2d", None)
        if corners is None:
            raise ValueError("SDK object does not expose bounding_box_2d")
        xs = [int(round(float(corner[0]))) for corner in corners]
        ys = [int(round(float(corner[1]))) for corner in corners]
        return min(xs), min(ys), max(xs), max(ys)

    def _mat_dimensions(self, mat: Any) -> tuple[int, int]:
        for width_name, height_name in (("get_width", "get_height"), ("width", "height")):
            width = getattr(mat, width_name, None)
            height = getattr(mat, height_name, None)
            if callable(width) and callable(height):
                return int(width()), int(height())
            if width is not None and height is not None and not callable(width) and not callable(height):
                return int(width), int(height)
        data = mat.get_data()
        height = len(data)
        width = len(data[0]) if height else 0
        return width, height

    def _mat_get_value(self, mat: Any, x: int, y: int) -> Any:
        value = mat.get_value(x, y)
        if isinstance(value, tuple) and len(value) == 2:
            return value[1]
        return value

    def _mask_pixels(self, sdk_object: Any, bbox_xyxy: tuple[int, int, int, int]) -> tuple[tuple[int, int], ...]:
        mask = getattr(sdk_object, "mask", None)
        if mask is None:
            return tuple()
        width, height = self._mat_dimensions(mask)
        origin_x, origin_y, _, _ = bbox_xyxy
        pixels: list[tuple[int, int]] = []
        for y in range(height):
            for x in range(width):
                value = self._mat_get_value(mask, x, y)
                is_set = bool(value[0] if isinstance(value, (list, tuple)) else value)
                if is_set:
                    pixels.append((origin_x + x, origin_y + y))
        return tuple(pixels)

    def _label_and_color(
        self,
        raw_label: Any,
        classes: dict[str, PaperClassConfig],
        fallback: PaperDetection | None,
    ) -> tuple[str, str]:
        if fallback is not None:
            return fallback.label, fallback.color
        if raw_label is None:
            return "paper", ""
        key = str(raw_label)
        if key in classes:
            config = classes[key]
            return config.label, config.color
        return key, key

    def _match_external_detection(
        self,
        bbox_xyxy: tuple[int, int, int, int],
        detections: Sequence[PaperDetection],
    ) -> PaperDetection | None:
        if not detections:
            return None
        ranked = sorted(detections, key=lambda detection: iou_xyxy(bbox_xyxy, detection.bbox_xyxy), reverse=True)
        best = ranked[0]
        return best if iou_xyxy(bbox_xyxy, best.bbox_xyxy) > 0.1 else None

    def open_live_camera(self, recording: RecordingConfig) -> Any:
        sl = self.sl
        init = sl.InitParameters()
        init.camera_resolution = self._enum_from_name(sl.RESOLUTION, recording.resolution)
        init.camera_fps = recording.fps
        init.depth_mode = self._enum_from_name(sl.DEPTH_MODE, recording.depth_mode)
        init.coordinate_system = self._enum_from_name(sl.COORDINATE_SYSTEM, recording.coordinate_system)
        init.coordinate_units = self._enum_from_name(sl.UNIT, recording.coordinate_units)
        camera = sl.Camera()
        error = camera.open(init)
        if error != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to open ZED camera: {error}")
        return camera

    def open_svo_camera(self, recording: RecordingConfig, svo_path: Path) -> Any:
        sl = self.sl
        init = sl.InitParameters()
        init.depth_mode = self._enum_from_name(sl.DEPTH_MODE, recording.depth_mode)
        init.coordinate_system = self._enum_from_name(sl.COORDINATE_SYSTEM, recording.coordinate_system)
        init.coordinate_units = self._enum_from_name(sl.UNIT, recording.coordinate_units)
        if hasattr(init, "set_from_svo_file"):
            init.set_from_svo_file(str(svo_path))
        else:
            init.input = sl.InputType()
            init.input.set_from_svo_file(str(svo_path))
        camera = sl.Camera()
        error = camera.open(init)
        if error != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to open SVO {svo_path}: {error}")
        return camera

    def close_camera(self, camera: Any) -> None:
        camera.close()

    def enable_recording(self, camera: Any, output_path: Path, recording: RecordingConfig) -> None:
        sl = self.sl
        params = sl.RecordingParameters()
        if hasattr(params, "video_filename"):
            params.video_filename = str(output_path)
        else:
            params.video_filename = str(output_path)
        params.compression_mode = self._enum_from_name(sl.SVO_COMPRESSION_MODE, recording.compression_mode)
        error = camera.enable_recording(params)
        if error != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to enable SVO recording to {output_path}: {error}")

    def disable_recording(self, camera: Any) -> None:
        camera.disable_recording()

    def build_runtime_parameters(self) -> Any:
        sl = self.sl
        params = sl.RuntimeParameters()
        params.measure3D_reference_frame = self._resolve_enum(sl.REFERENCE_FRAME, "WORLD")
        return params

    def enable_tracking(
        self,
        camera: Any,
        tracking: TrackingConfig,
        *,
        mode: TrackingMode,
        area_path: Path | None,
    ) -> None:
        sl = self.sl
        params = sl.PositionalTrackingParameters()
        self._set_if_present(params, ("mode",), self._enum_from_name(sl.POSITIONAL_TRACKING_MODE, tracking.positional_tracking_mode))
        self._set_if_present(params, ("enable_imu_fusion",), tracking.enable_imu_fusion)
        self._set_if_present(params, ("set_gravity_as_origin",), tracking.set_gravity_as_origin)
        self._set_if_present(params, ("set_floor_as_origin",), tracking.set_floor_as_origin)
        self._set_if_present(params, ("enable_pose_smoothing",), tracking.enable_pose_smoothing)
        self._set_if_present(params, ("enable_2d_ground_mode",), tracking.enable_2d_ground_mode)
        enable_area_memory = mode in {"vslam_map", "vslam_localize"}
        self._set_if_present(params, ("enable_area_memory",), enable_area_memory)
        self._set_if_present(params, ("enable_localization_only",), mode == "vslam_localize")
        if area_path is not None and enable_area_memory:
            self._set_if_present(params, ("area_file_path",), str(area_path))
        error = camera.enable_positional_tracking(params)
        if error != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to enable positional tracking: {error}")

    def disable_tracking(self, camera: Any, *, area_path: Path | None) -> None:
        if area_path is not None:
            camera.disable_positional_tracking(str(area_path))
        else:
            camera.disable_positional_tracking()

    def success_code(self) -> Any:
        return self.sl.ERROR_CODE.SUCCESS

    def end_of_svo_code(self) -> Any:
        return self._resolve_enum(self.sl.ERROR_CODE, "END_OF_SVOFILE_REACHED")

    def grab(self, camera: Any, runtime_parameters: Any) -> Any:
        return camera.grab(runtime_parameters)

    def create_frame_buffers(self) -> FrameBuffers:
        sl = self.sl
        return FrameBuffers(
            point_cloud_world=sl.Mat(),
            depth_map=sl.Mat(),
            image_size=(0, 0),
        )

    def populate_frame_buffers(self, camera: Any, buffers: FrameBuffers) -> None:
        sl = self.sl
        camera.retrieve_measure(buffers.point_cloud_world, self._resolve_enum(sl.MEASURE, "XYZRGBA"))
        camera.retrieve_measure(buffers.depth_map, self._resolve_enum(sl.MEASURE, "DEPTH"))
        buffers.image_size = self._mat_dimensions(buffers.point_cloud_world)

    def sample_world_point(self, buffers: FrameBuffers, u: int, v: int) -> tuple[float, float, float] | None:
        value = self._mat_get_value(buffers.point_cloud_world, u, v)
        if not isinstance(value, (list, tuple)) or len(value) < 3:
            return None
        coords = tuple(float(value[index]) for index in range(3))
        if any(not (coord == coord and abs(coord) != float("inf")) for coord in coords):
            return None
        return coords

    def sample_depth(self, buffers: FrameBuffers, u: int, v: int) -> float | None:
        value = self._mat_get_value(buffers.depth_map, u, v)
        scalar = float(value[0] if isinstance(value, (list, tuple)) else value)
        if scalar != scalar or abs(scalar) == float("inf"):
            return None
        return scalar

    def pose_snapshot(self, camera: Any) -> PoseSnapshot:
        sl = self.sl
        pose = sl.Pose()
        try:
            tracking_state = camera.get_position(pose, self._resolve_enum(sl.REFERENCE_FRAME, "WORLD"))
        except TypeError:
            tracking_state = camera.get_position(pose)
        pose_xyz = None
        pose_xyzw = None
        if hasattr(pose, "get_translation"):
            pose_xyz = self._vector_to_tuple(pose.get_translation(), 3)
        if hasattr(pose, "get_orientation"):
            pose_xyzw = self._vector_to_tuple(pose.get_orientation(), 4)
        timestamp_ns = 0
        timestamp = getattr(pose, "timestamp", None)
        if timestamp is not None:
            if hasattr(timestamp, "get_nanoseconds"):
                timestamp_ns = int(timestamp.get_nanoseconds())
            elif hasattr(timestamp, "data_ns"):
                timestamp_ns = int(timestamp.data_ns)
        tracking_confidence = 0
        for attr_name in ("pose_confidence", "confidence"):
            value = getattr(pose, attr_name, None)
            if value is not None:
                tracking_confidence = int(value)
                break
        state_name = getattr(tracking_state, "name", None)
        state_text = state_name if state_name is not None else str(tracking_state)
        return PoseSnapshot(
            timestamp_ns=timestamp_ns,
            tracking_state=state_text,
            tracking_confidence=tracking_confidence,
            pose_world_xyz=pose_xyz,  # type: ignore[arg-type]
            pose_world_xyzw=pose_xyzw,  # type: ignore[arg-type]
        )

    def _plane_snapshot(self, plane: Any, status: Any) -> PlaneSnapshot:
        sl = self.sl
        status_name = getattr(status, "name", None)
        status_text = status_name if status_name is not None else str(status)
        if status != sl.ERROR_CODE.SUCCESS:
            return PlaneSnapshot(status=status_text, center_xyz=None, normal_xyz=None, equation_abcd=None, plane_type=None)
        center = None
        for attr_name in ("get_center",):
            method = getattr(plane, attr_name, None)
            if callable(method):
                center = self._vector_to_tuple(method(), 3)
                break
        if center is None:
            transform = None
            for attr_name in ("get_transform", "get_pose"):
                method = getattr(plane, attr_name, None)
                if callable(method):
                    transform = method()
                    break
            if transform is not None:
                for attr_name in ("get_translation", "get_translation_vector"):
                    method = getattr(transform, attr_name, None)
                    if callable(method):
                        center = self._vector_to_tuple(method(), 3)
                        break
        normal = self._vector_to_tuple(plane.get_normal(), 3) if hasattr(plane, "get_normal") else None
        equation = self._vector_to_tuple(plane.get_plane_equation(), 4) if hasattr(plane, "get_plane_equation") else None
        plane_type = None
        if hasattr(plane, "get_type"):
            raw = plane.get_type()
            plane_type = getattr(raw, "name", None) or str(raw)
        return PlaneSnapshot(
            status=status_text,
            center_xyz=center,  # type: ignore[arg-type]
            normal_xyz=normal,  # type: ignore[arg-type]
            equation_abcd=equation,  # type: ignore[arg-type]
            plane_type=plane_type,
        )

    def detect_plane(self, camera: Any, u: int, v: int) -> PlaneSnapshot:
        sl = self.sl
        plane = sl.Plane()
        coord = sl.uint2()
        try:
            coord[0] = int(u)
            coord[1] = int(v)
        except Exception:
            if hasattr(coord, "x"):
                coord.x = int(u)
                coord.y = int(v)
        status = camera.find_plane_at_hit(coord, plane)
        return self._plane_snapshot(plane, status)

    def detect_floor_plane(self, camera: Any) -> PlaneSnapshot:
        sl = self.sl
        plane = sl.Plane()
        reset_tracking_floor_frame = sl.Transform()
        try:
            status = camera.find_floor_plane(plane, reset_tracking_floor_frame)
        except TypeError:
            try:
                status = camera.find_floor_plane(plane)
            except TypeError as exc:
                raise RuntimeError("ZED SDK floor plane API signature is unsupported by this adapter") from exc
        return self._plane_snapshot(plane, status)

    def configure_paper_detection(self, camera: Any, config: PaperDetectionConfig) -> DetectionContext:
        sl = self.sl
        if config.mode == "none":
            return DetectionContext(enabled=False, mode="none")
        params = sl.ObjectDetectionParameters()
        self._set_if_present(params, ("enable_tracking",), True)
        self._set_if_present(params, ("enable_segmentation", "enable_mask_output"), True)
        if config.mode == "native_yolo":
            if not config.native_yolo_onnx_path:
                raise ValueError("paper.native_yolo_onnx_path is required when paper.mode='native_yolo'")
            model = self._resolve_enum(
                sl.OBJECT_DETECTION_MODEL,
                "CUSTOM_YOLOLIKE_BOX_OBJECTS",
                "CUSTOM_BOX_OBJECTS",
            )
            self._set_if_present(params, ("detection_model",), model)
            if config.native_yolo_onnx_path:
                self._set_if_present(params, ("custom_onnx_file",), str(config.native_yolo_onnx_path))
            if config.native_yolo_input_size:
                self._set_if_present(params, ("custom_onnx_dynamic_input_shape",), False)
                self._set_if_present(params, ("custom_onnx_input_size",), config.native_yolo_input_size)
            if config.native_yolo_class_count is not None:
                self._set_if_present(params, ("custom_yolo_onnx_class_count", "custom_class_count"), config.native_yolo_class_count)
        else:
            model = self._resolve_enum(sl.OBJECT_DETECTION_MODEL, "CUSTOM_BOX_OBJECTS")
            self._set_if_present(params, ("detection_model",), model)
        error = camera.enable_object_detection(params)
        if error != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to enable object detection: {error}")
        runtime = sl.ObjectDetectionRuntimeParameters()
        self._set_if_present(runtime, ("detection_confidence_threshold",), config.confidence_threshold)
        return DetectionContext(enabled=True, mode=config.mode, runtime_params=runtime)

    def disable_paper_detection(self, camera: Any) -> None:
        camera.disable_object_detection()

    def _sdk_custom_box(self, detection: PaperDetection) -> Any:
        sl = self.sl
        obj = sl.CustomBoxObjectData()
        obj.unique_object_id = detection.frame_idx
        obj.probability = float(detection.confidence)
        obj.label = int(detection.label) if str(detection.label).isdigit() else 0
        obj.bounding_box_2d = [
            [detection.bbox_xyxy[0], detection.bbox_xyxy[1]],
            [detection.bbox_xyxy[2], detection.bbox_xyxy[1]],
            [detection.bbox_xyxy[2], detection.bbox_xyxy[3]],
            [detection.bbox_xyxy[0], detection.bbox_xyxy[3]],
        ]
        obj.is_grounded = False
        return obj

    def paper_observations(
        self,
        camera: Any,
        context: DetectionContext,
        config: PaperDetectionConfig,
        external_detections: Sequence[PaperDetection],
    ) -> list[PaperObservation]:
        if not context.enabled:
            return []
        sl = self.sl
        if context.mode == "external_boxes":
            if not external_detections:
                return []
            camera.ingest_custom_box_objects([self._sdk_custom_box(detection) for detection in external_detections])
        objects = sl.Objects()
        camera.retrieve_objects(objects, context.runtime_params)
        observations: list[PaperObservation] = []
        for sdk_object in getattr(objects, "object_list", []):
            bbox_xyxy = self._bbox_from_sdk_object(sdk_object)
            matched = self._match_external_detection(bbox_xyxy, external_detections)
            raw_label = getattr(sdk_object, "raw_label", getattr(sdk_object, "label", None))
            label, color = self._label_and_color(raw_label, config.classes, matched)
            confidence = float(getattr(sdk_object, "confidence", getattr(sdk_object, "probability", 0.0)))
            observations.append(
                PaperObservation(
                    bbox_xyxy=bbox_xyxy,
                    center_pixel=bbox_center(bbox_xyxy),
                    confidence=matched.confidence if matched is not None else confidence,
                    label=label,
                    color=color,
                    mask_pixels=self._mask_pixels(sdk_object, bbox_xyxy),
                )
            )
        return observations

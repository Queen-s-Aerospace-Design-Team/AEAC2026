from __future__ import annotations

import math
import statistics
from typing import Iterable, Sequence


def is_finite_point(point: Sequence[float] | None, *, expected_len: int = 3) -> bool:
    if point is None or len(point) < expected_len:
        return False
    return all(math.isfinite(float(point[index])) for index in range(expected_len))


def scale_bbox_xyxy(
    bbox_xyxy: tuple[int, int, int, int],
    source_size: tuple[int, int],
    target_size: tuple[int, int],
) -> tuple[int, int, int, int]:
    src_w, src_h = source_size
    dst_w, dst_h = target_size
    if src_w <= 0 or src_h <= 0:
        raise ValueError("Source size must be positive")
    scale_x = dst_w / src_w
    scale_y = dst_h / src_h
    x1, y1, x2, y2 = bbox_xyxy
    return (
        int(round(x1 * scale_x)),
        int(round(y1 * scale_y)),
        int(round(x2 * scale_x)),
        int(round(y2 * scale_y)),
    )


def scale_point(
    point_xy: tuple[int, int],
    source_size: tuple[int, int],
    target_size: tuple[int, int],
) -> tuple[int, int]:
    scaled = scale_bbox_xyxy((point_xy[0], point_xy[1], point_xy[0], point_xy[1]), source_size, target_size)
    return scaled[0], scaled[1]


def bbox_center(bbox_xyxy: tuple[int, int, int, int]) -> tuple[int, int]:
    x1, y1, x2, y2 = bbox_xyxy
    return int(round((x1 + x2) / 2)), int(round((y1 + y2) / 2))


def clamp_point(point_xy: tuple[int, int], image_size: tuple[int, int]) -> tuple[int, int]:
    x, y = point_xy
    width, height = image_size
    return min(max(0, x), width - 1), min(max(0, y), height - 1)


def median_point(points: Iterable[Sequence[float]]) -> tuple[float, float, float] | None:
    valid = [tuple(float(point[index]) for index in range(3)) for point in points if is_finite_point(point)]
    if not valid:
        return None
    return tuple(statistics.median(values) for values in zip(*valid))


def nearest_point(points: Iterable[Sequence[float]], camera_position: Sequence[float]) -> tuple[float, float, float] | None:
    valid = [tuple(float(point[index]) for index in range(3)) for point in points if is_finite_point(point)]
    if not valid:
        return None
    return min(valid, key=lambda point: euclidean_distance(point, camera_position))


def euclidean_distance(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a[:3], b[:3], strict=True)))


def vector_alignment_angle_deg(a: Sequence[float], b: Sequence[float]) -> float | None:
    if not is_finite_point(a) or not is_finite_point(b):
        return None
    ax, ay, az = (float(value) for value in a[:3])
    bx, by, bz = (float(value) for value in b[:3])
    norm_a = math.sqrt(ax * ax + ay * ay + az * az)
    norm_b = math.sqrt(bx * bx + by * by + bz * bz)
    if norm_a == 0.0 or norm_b == 0.0:
        return None
    cosine = abs((ax * bx + ay * by + az * bz) / (norm_a * norm_b))
    cosine = min(1.0, max(-1.0, cosine))
    return math.degrees(math.acos(cosine))


def invert_quaternion(xyzw: Sequence[float]) -> tuple[float, float, float, float]:
    x, y, z, w = (float(value) for value in xyzw[:4])
    norm_sq = x * x + y * y + z * z + w * w
    if norm_sq == 0:
        raise ValueError("Quaternion norm must be non-zero")
    return (-x / norm_sq, -y / norm_sq, -z / norm_sq, w / norm_sq)


def quaternion_rotate_vector(xyzw: Sequence[float], vector_xyz: Sequence[float]) -> tuple[float, float, float]:
    x, y, z, w = (float(value) for value in xyzw[:4])
    vx, vy, vz = (float(value) for value in vector_xyz[:3])

    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)

    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def world_to_camera_point(
    point_world_xyz: Sequence[float],
    pose_world_xyz: Sequence[float],
    pose_world_xyzw: Sequence[float],
) -> tuple[float, float, float]:
    translated = tuple(float(point_world_xyz[i]) - float(pose_world_xyz[i]) for i in range(3))
    return quaternion_rotate_vector(invert_quaternion(pose_world_xyzw), translated)


def iou_xyxy(a: Sequence[int], b: Sequence[int]) -> float:
    ax1, ay1, ax2, ay2 = [int(value) for value in a[:4]]
    bx1, by1, bx2, by2 = [int(value) for value in b[:4]]
    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)
    inter_w = max(0, inter_x2 - inter_x1)
    inter_h = max(0, inter_y2 - inter_y1)
    intersection = inter_w * inter_h
    if intersection == 0:
        return 0.0
    a_area = max(0, ax2 - ax1) * max(0, ay2 - ay1)
    b_area = max(0, bx2 - bx1) * max(0, by2 - by1)
    union = a_area + b_area - intersection
    return intersection / union if union > 0 else 0.0

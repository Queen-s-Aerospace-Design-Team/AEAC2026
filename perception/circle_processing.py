import json
import time
import numpy as np
from sklearn.cluster import DBSCAN
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

# Constants to tune: -------------------------------------------------------------

CONFIDENCE_THRESHOLD = 0.75  # paper detection confidence threshold
parallel_tolerance = 0.9     # corner inference parallelism tolerance
eps_pt = 0.3                    # DBSCAN clustering radius in metres
min_samples_pt = 5              # DBSCAN minimum samples per point cluster
eps_pl = 0.2                    # DBSCAN clustering radius, unitless
min_samples_pl = 3              # DBSCAN minimum samples per plane cluster
eps_gr = 0.15                   # DBSCAN clustering radius for ground plane in metres
NORMAL_DISTANCE_WEIGHT = 0.3    # scales signed-distance-from-origin (m) into the
                                # same magnitude range as unit-normal components
                                # when clustering planes in 4D (n, d) space

# Data Structures: -------------------------------------------------------

@dataclass
class PaperDetection:  # One instance of a paper in a frame
    # Identity
    colour:             str
    # Position
    position:           np.ndarray      # point_world_xyz [x, y, z]
    depth_m:            float           # depth in metres
    center_uv:          tuple[int, int] # pixel coords [u, v]
    # Plane (resolved from plane_id lookup at parse time)
    normal:             Optional[np.ndarray]  # normal_world_xyz
    equation:           Optional[list[float]] # equation_abcd [a, b, c, d]
    plane_type:         Optional[str]         # e.g. "VERTICAL"
    plane_query_status: str                   # "SUCCESS", "FAILED", etc.
    # Quality
    confidence:         float                 # 0-1
    quality_flags:      list[str]             # e.g. ["clean"]


@dataclass
class DetectedPlane:
    plane_id:          str
    normal:            np.ndarray       # normal_world_xyz [x, y, z]
    equation:          list[float]      # [a, b, c, d]
    plane_type:        str              # "VERTICAL", "HORIZONTAL", etc.
    source:            str              # source (floor_query, paper_center, scene_probe)
    center_world_xyz:  np.ndarray = field(default_factory=lambda: np.array([0., 0., 0.]))

@dataclass
class Frame:
    frame_index:    int
    timestamp_ns:   int
    tracking_state: str
    papers:         list[PaperDetection] = field(default_factory=list)
    planes:         list[DetectedPlane]  = field(default_factory=list)

@dataclass
class InferredCorner:
    position:     np.ndarray    # 3D point from plane intersection
    wall_plane_1: DetectedPlane
    wall_plane_2: DetectedPlane
    ground_plane: DetectedPlane
    label:        str           # e.g. "northwest" — filled in once north is known


# Parsers: --------------------------------------------------------------

def parse_paper(p: dict, plane_lookup: dict) -> Optional[PaperDetection]:
    """Parse a single paper detection from a frame, resolving its plane via lookup."""

    # Cheapest checks up front
    if "point_world_xyz" not in p:
        return None
    if "clean" not in p.get("quality_flags", []):
        return None
    if p.get("detector_confidence", 0) < CONFIDENCE_THRESHOLD:
        return None

    # Resolve plane via plane_id lookup
    plane_id = p.get("plane_id")
    plane = plane_lookup.get(plane_id) if plane_id else None

    # Papers without a resolved plane still get clustered, can't be snapped to a plane
    if plane:
        normal     = np.array(plane["normal_world_xyz"])
        equation   = plane["equation_abcd"]
        plane_type = plane.get("type")
    else:
        normal     = None
        equation   = None
        plane_type = None

    center_uv = p.get("center_uv", [None, None])

    return PaperDetection(
        colour             = p["color"].lower().strip(),
        position           = np.array(p["point_world_xyz"]),
        depth_m            = p.get("depth_m", 0.0),
        center_uv          = (center_uv[0], center_uv[1]),
        normal             = normal,
        equation           = equation,
        plane_type         = plane_type,
        plane_query_status = p.get("plane_query_status", "UNKNOWN"),
        confidence         = p["detector_confidence"],
        quality_flags      = p.get("quality_flags", []),
    )


def parse_frame(raw: dict) -> Optional[Frame]:
    """Parse a single frame from the JSONL stream."""
    # endswith("OK") matches the producer's own check (sdk/pipeline both use
    # endswith), so we don't silently disagree on OK_RELOCALIZED-style variants.
    if not raw.get("tracking_state", "").endswith("OK"):
        return None

    # Build raw plane lookup table for paper parsing (raw dicts keyed by plane_id)
    raw_plane_lookup = {
        p["plane_id"]: p
        for p in raw.get("planes", [])
        if "normal_world_xyz" in p
    }

    # Parse papers using the shared parse_paper function
    papers = [
        p for p in (
            parse_paper(raw_p, raw_plane_lookup)
            for raw_p in raw.get("papers", [])
        )
        if p is not None
    ]

    # Parse planes into DetectedPlane objects
    planes = []
    for raw_p in raw.get("planes", []):
        if "equation_abcd" not in raw_p or "normal_world_xyz" not in raw_p:
            continue
        center = raw_p.get("center_world_xyz", [0.0, 0.0, 0.0])
        planes.append(DetectedPlane(
            plane_id         = raw_p["plane_id"],
            normal           = np.array(raw_p["normal_world_xyz"]),
            equation         = raw_p["equation_abcd"],
            plane_type       = raw_p.get("type", "UNKNOWN"),
            source           = raw_p.get("source", "UNKNOWN"),
            center_world_xyz = np.array(center),
        ))

    return Frame(
        frame_index    = raw["frame_idx"],
        timestamp_ns   = raw["timestamp_ns"],
        tracking_state = raw["tracking_state"],
        papers         = papers,
        planes         = planes,
    )


# Corner Inference: ------------------------------------------------------------------

def infer_corners(wall_planes: list[DetectedPlane],
                  ground_plane: DetectedPlane) -> list[InferredCorner]:
    """
    For every pair of non-parallel wall planes, intersect them with the
    ground plane to produce a corner point.
    """
    corners = []

    for i, wall_1 in enumerate(wall_planes):
        for wall_2 in wall_planes[i+1:]:

            # Skip walls that are roughly parallel — they don't form a corner
            normal_dot = abs(np.dot(wall_1.normal, wall_2.normal))
            if normal_dot > parallel_tolerance:  
                continue

            # Build the 3x3 system from the three plane equations
            A = np.array([
                wall_1.equation[:3],
                wall_2.equation[:3],
                ground_plane.equation[:3],
            ])
            b = np.array([
                -wall_1.equation[3],
                -wall_2.equation[3],
                -ground_plane.equation[3],
            ])

            # Check the system is actually solvable before trying
            if abs(np.linalg.det(A)) < 1e-6:
                continue  # planes don't form a clean intersection

            position = np.linalg.solve(A, b)

            corners.append(InferredCorner(
                position     = position,
                wall_plane_1 = wall_1,
                wall_plane_2 = wall_2,
                ground_plane = ground_plane,
                label        = "",  # filled in later once north is known
            ))

    return corners


# The Accumulator: -------------------------------------------------------

@dataclass
class DetectionAccumulator:
    paper_positions:   list = field(default_factory=list)  # [n, 3]
    paper_normals:     list = field(default_factory=list)  # [n, 3] or None
    paper_colours:     list = field(default_factory=list)  # [n]
    paper_confidences: list = field(default_factory=list)  # [n]
    paper_equations:   list = field(default_factory=list)  # [n, 4] or None
    paper_plane_types: list = field(default_factory=list)  # [n]
    all_planes:        list = field(default_factory=list)  # all DetectedPlane objects

    def ingest_frame(self, frame: Frame):
        for p in frame.papers:
            self.paper_positions.append(p.position)
            self.paper_normals.append(p.normal)
            self.paper_colours.append(p.colour)
            self.paper_confidences.append(p.confidence)
            self.paper_equations.append(p.equation)
            self.paper_plane_types.append(p.plane_type)

        self.all_planes.extend(frame.planes)

    def to_numpy(self):
        return {
            "paper_positions":   np.array(self.paper_positions) if self.paper_positions else np.array([]),
            "paper_colours":     np.array(self.paper_colours),
            "paper_confidences": np.array(self.paper_confidences),
            "paper_normals":     self.paper_normals,       # list — may contain None
            "paper_equations":   self.paper_equations,     # list — may contain None
            "paper_plane_types": np.array(self.paper_plane_types),
            "all_planes":        self.all_planes,
        }


# Entry Points: -------------------------------------------------------
#   tail_frames  — for live reading during flight
#   ingest_json_file — for offline testing

def _infer_session_root(filepath: str) -> Optional[Path]:
    """If filepath looks like <root>/stream/frames.jsonl, return <root>."""
    path = Path(filepath)
    if path.parent.name == "stream":
        return path.parent.parent
    return None


def verify_coordinate_system(session_root: Optional[Path]) -> None:
    """
    Assert the producer recorded with Z-up coordinates. Every downstream
    geometry call here (UP_VECTOR, ground_z from xyz[2], surface classifier,
    wall projections) assumes Z-up; if the producer ever switches coord
    systems, the answers go silently wrong. Better to fail loudly at parse.
    """
    if session_root is None:
        return  # caller didn't give us a session dir; skip the check
    meta_path = session_root / "session.json"
    if not meta_path.exists():
        return  # not a full session directory; nothing to verify
    with open(meta_path, "r") as f:
        meta = json.load(f)
    cs = meta.get("config", {}).get("recording", {}).get("coordinate_system")
    if cs is not None and cs != "RIGHT_HANDED_Z_UP_X_FORWARD":
        raise ValueError(
            f"circle_processing assumes Z-up coordinates but the session was "
            f"recorded with coordinate_system={cs!r}. Re-record with "
            f"recording.coordinate_system='RIGHT_HANDED_Z_UP_X_FORWARD' or "
            f"transform the JSONL stream before passing it in."
        )


def tail_frames(filepath: str, accumulator: DetectionAccumulator) -> DetectionAccumulator:
    """Read frames live as they are appended during flight.
    Note: runs an infinite loop — intended for use in a background thread."""
    verify_coordinate_system(_infer_session_root(filepath))

    frames_parsed  = 0
    frames_skipped = 0

    with open(filepath, "r") as f:
        while True:
            line = f.readline()
            if not line:
                time.sleep(0.01)
                continue

            try:
                raw = json.loads(line.strip())
            except json.JSONDecodeError:
                frames_skipped += 1
                continue

            frame = parse_frame(raw)
            if frame is None:
                frames_skipped += 1
                continue

            accumulator.ingest_frame(frame)
            frames_parsed += 1

    return accumulator  # technically unreachable during live flight


def ingest_json_file(filepath: str) -> DetectionAccumulator:
    """Read a completed JSONL file — useful for offline testing."""
    verify_coordinate_system(_infer_session_root(filepath))

    accumulator    = DetectionAccumulator()
    frames_parsed  = 0
    frames_skipped = 0

    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            try:
                raw = json.loads(line)
            except json.JSONDecodeError:
                frames_skipped += 1
                continue

            frame = parse_frame(raw)
            if frame is None:
                frames_skipped += 1
                continue

            accumulator.ingest_frame(frame)
            frames_parsed += 1

    print(f"Ingested {frames_parsed} frames, skipped {frames_skipped}.")
    print(f"  → {len(accumulator.paper_positions)} paper detections")
    print(f"  → {len(accumulator.all_planes)} plane detections")

    return accumulator


# DBSCAN Clustering: -------------------------------------------------------

def cluster_papers(data: dict) -> list[dict]:
    """
    Cluster paper detections and return one representative per cluster,
    preserving colour, normal, and equation from the highest-confidence
    detection in each cluster.
    """
    positions = data["paper_positions"]
    if len(positions) == 0:
        return []

    labels = DBSCAN(eps_pt, min_samples=min_samples_pt).fit_predict(positions)  # **PARAMETERS TO ADJUST**

    clusters = []
    for label in set(labels):
        if label == -1:
            continue

        mask = labels == label

        # Position — median for robustness
        centroid = np.median(positions[mask], axis=0)

        # Colour — majority vote across cluster
        colours = data["paper_colours"][mask]
        colour  = max(set(colours), key=list(colours).count)

        # Normal and equation — take from highest confidence detection
        confidences = data["paper_confidences"][mask]
        best_idx    = np.argmax(confidences)
        indices     = np.where(mask)[0]
        best        = indices[best_idx]

        clusters.append({
            "position":   centroid,
            "colour":     colour,
            "normal":     data["paper_normals"][best],
            "equation":   data["paper_equations"][best],
            "plane_type": data["paper_plane_types"][best],
        })

    return clusters


def canonicalize_plane(plane: DetectedPlane) -> DetectedPlane:
    """
    Flip the plane normal into a canonical hemisphere so two observations of the
    same wall — one with ZED's normal pointing toward the camera, the other with
    the normal flipped after the camera moved past it — don't get treated as two
    different walls in clustering. The equation coefficients are flipped to
    match so plane math stays correct.
    """
    n_raw = np.asarray(plane.normal, dtype=float)
    n_norm = float(np.linalg.norm(n_raw))
    if n_norm < 1e-9:
        return plane
    n = n_raw / n_norm
    if n[int(np.argmax(np.abs(n)))] < 0:
        n = -n
        eq = [-plane.equation[0], -plane.equation[1], -plane.equation[2], -plane.equation[3]]
    else:
        eq = list(plane.equation)
    return DetectedPlane(
        plane_id         = plane.plane_id,
        normal           = n,
        equation         = eq,
        plane_type       = plane.plane_type,
        source           = plane.source,
        center_world_xyz = plane.center_world_xyz,
    )


def cluster_planes(planes: list[DetectedPlane]) -> list[DetectedPlane]:
    """
    Cluster plane detections by normal direction AND signed distance, so two
    parallel walls (e.g. opposite walls of a room) don't collapse into a single
    cluster. Returns one representative plane per unique wall.
    """
    if not planes:
        return []

    # Sign-canonicalize first; otherwise a single wall observed from both sides
    # becomes two clusters with opposite normals.
    canon = [canonicalize_plane(p) for p in planes]

    # 4D feature: [nx, ny, nz, d_scaled] where d is the canonical signed distance
    # from origin (d = -n·c for a point c on the plane). Scaling brings meters
    # into the same magnitude band as unit-normal components.
    features = np.array([
        [
            p.normal[0],
            p.normal[1],
            p.normal[2],
            (-float(np.dot(p.normal, p.center_world_xyz))) * NORMAL_DISTANCE_WEIGHT,
        ]
        for p in canon
    ])

    labels = DBSCAN(eps_pl, min_samples=min_samples_pl).fit_predict(features)

    clustered = []
    for label in set(labels):
        if label == -1:
            continue

        mask = labels == label
        indices = np.where(mask)[0]

        cluster_features = features[mask]
        mean_feature     = cluster_features.mean(axis=0)
        dists            = np.linalg.norm(cluster_features - mean_feature, axis=1)
        best             = canon[indices[int(np.argmin(dists))]]

        clustered.append(best)

    return clustered


# Find Ground Plane: ----------------------------------------------------------------

def find_ground_plane(all_planes: list[DetectedPlane]) -> Optional[DetectedPlane]:
    """
    Find the ground plane from accumulated flight planes.

    Strategy:
      1. Prefer planes tagged source=="floor" — ZED's find_floor_plane() result.
         (The producer's PlaneSource literal is "floor"; see models.py.)
      2. Fall back to any HORIZONTAL planes, pick the lowest cluster.
      3. Return None if no horizontal planes found.

    The returned plane's equation uses the measured normal, not a hardcoded
    [0,0,1] — that way corner intersections stay accurate even if gravity
    isn't perfectly aligned with +Z in the session frame.
    """

    # Step 1: try floor-query planes first
    floor_queries = [
        p for p in all_planes
        if p.source == "floor" and p.plane_type == "HORIZONTAL"
    ]

    if len(floor_queries) >= 3:  # need at least 3 for a reliable consensus
        normal = np.median(
            np.array([p.normal for p in floor_queries]), axis=0
        )
        normal = normal / np.linalg.norm(normal)
        centers    = np.array([p.center_world_xyz for p in floor_queries])
        center_xyz = np.median(centers, axis=0)
        d          = -float(np.dot(normal, center_xyz))

        return DetectedPlane(
            plane_id         = "ground_consensus",
            normal           = normal,
            equation         = [float(normal[0]), float(normal[1]), float(normal[2]), d],
            plane_type       = "HORIZONTAL",
            source           = "floor_consensus",
            center_world_xyz = center_xyz,
        )

    # Step 2: fall back to all horizontal planes
    horizontal = [
        p for p in all_planes
        if p.plane_type == "HORIZONTAL"
    ]

    if not horizontal:
        return None  # no horizontal data at all — cannot determine ground

    heights = np.array([p.center_world_xyz[2] for p in horizontal])

    # Step 3: cluster by height to separate ground/roof/platforms
    if len(horizontal) >= 2:
        labels = DBSCAN(
            eps_gr, min_samples=2
        ).fit_predict(heights.reshape(-1, 1))
    else:
        labels = np.array([0])  # single detection, treat as its own cluster

    # Find the lowest valid cluster
    valid_labels = [l for l in set(labels) if l != -1]
    lowest_label = None

    if not valid_labels:
        # All detections were noise — just take the single lowest point
        ground_z = float(np.min(heights))
    else:
        cluster_medians = {
            l: float(np.median(heights[labels == l]))
            for l in valid_labels
        }
        lowest_label = min(cluster_medians, key=lambda k: cluster_medians[k])
        ground_z     = cluster_medians[lowest_label]

        # Sanity check — if the lowest cluster is suspiciously high,
        # something is wrong with the plane data
        if ground_z > 1.0:
            print(f"Warning: ground plane detected at z={ground_z:.2f}m — "
                  f"expected near 0. Check plane data.")

    # Use the normal and centroid from the lowest cluster
    if lowest_label is not None:
        lowest_planes = [
            horizontal[i] for i, l in enumerate(labels)
            if l == lowest_label
        ]
        normal = np.median(
            np.array([p.normal for p in lowest_planes]), axis=0
        )
        normal = normal / np.linalg.norm(normal)
        center_xyz = np.median(
            np.array([p.center_world_xyz for p in lowest_planes]), axis=0
        )
    else:
        # No clean cluster — use a pure vertical up vector as best guess
        normal     = np.array([0.0, 0.0, 1.0])
        center_xyz = np.array([0.0, 0.0, ground_z])

    d = -float(np.dot(normal, center_xyz))
    return DetectedPlane(
        plane_id         = "ground_fallback",
        normal           = normal,
        equation         = [float(normal[0]), float(normal[1]), float(normal[2]), d],
        plane_type       = "HORIZONTAL",
        source           = "height_cluster_fallback",
        center_world_xyz = center_xyz,
    )


# Gravity Vector: -------------------------------------------------------
""" ZED SDK gravity-aligns its world frame by default via IMU,
so z-up is a safe assumption. Confirm with teammate that
this hasn't been disabled in the ZED config. """
 
DOWN_VECTOR = np.array([0.0, 0.0, -1.0])
UP_VECTOR   = np.array([0.0, 0.0,  1.0])
 
 
# Wall Snapping: -------------------------------------------------------
 
def snap_to_plane(point: np.ndarray, plane_normal: np.ndarray,
                  plane_point: np.ndarray) -> np.ndarray:
    """Project a point onto a plane defined by a normal and a point on the plane."""
    n = plane_normal / np.linalg.norm(plane_normal)
    v = point - plane_point
    distance = np.dot(v, n)
    return point - distance * n
 
 
def point_to_plane_on_equation(point: np.ndarray,
                               equation: list[float]) -> np.ndarray:
    """Project a point onto a plane defined by equation ax+by+cz+d=0."""
    n = np.array(equation[:3])
    d = equation[3]
    n_norm = np.linalg.norm(n)
    n_unit = n / n_norm
    signed_dist = (np.dot(n, point) + d) / n_norm
    return point - signed_dist * n_unit
 
 
def find_best_wall(circle_position: np.ndarray,
                   circle_normal: Optional[np.ndarray],
                   wall_planes: list[DetectedPlane]) -> Optional[DetectedPlane]:
    """
    Find the wall plane a circle belongs to.
    Uses normal agreement if the circle has a normal, falls back to distance.
    """
    if not wall_planes:
        return None
 
    best_plane = None
    best_score = -np.inf
 
    for wall in wall_planes:
        # Distance from circle to this wall plane
        eq = wall.equation
        dist = abs(eq[0]*circle_position[0] + eq[1]*circle_position[1] +
                    eq[2]*circle_position[2] + eq[3])
        dist /= np.linalg.norm(eq[:3])
 
        if circle_normal is not None:
            # Score = how well normals agree, penalized by distance
            normal_agreement = abs(np.dot(
                circle_normal / np.linalg.norm(circle_normal),
                wall.normal / np.linalg.norm(wall.normal)
            ))
            score = normal_agreement - 0.5 * dist  # **WEIGHTS TO ADJUST**
        else:
            score = -dist
 
        if score > best_score:
            best_score = score
            best_plane = wall
 
    return best_plane
 
 
def snap_circles_to_walls(paper_clusters: list[dict],
                          wall_planes: list[DetectedPlane]) -> list[dict]:
    """
    For each clustered circle, find its best matching wall and
    snap its position onto that wall plane.
    """
    for circle in paper_clusters:
        wall = find_best_wall(circle["position"], circle["normal"], wall_planes)
        circle["wall"] = wall
 
        if wall is not None:
            circle["position_snapped"] = point_to_plane_on_equation(
                circle["position"], wall.equation
            )
        else:
            circle["position_snapped"] = circle["position"]
 
    return paper_clusters
 
 
# Surface Classifier: -------------------------------------------------------
 
def classify_surface(circle_normal: Optional[np.ndarray],
                     plane_type: Optional[str],
                     circle_z: float,
                     ground_z: float,
                     angle_threshold_deg: float = 20.0) -> str:
    """
    Classify what surface a circle is on: wall, ground, roof, or inclined.
    Uses the ZED plane_type first, falls back to normal-based classification.
    """
    if plane_type == "VERTICAL":
        return "wall"
    if plane_type == "HORIZONTAL":
        if circle_z < ground_z + 0.5:
            return "ground"
        else:
            return "roof"
 
    # For unlabeled or unusual plane types, classify using the normal
    if circle_normal is None:
        return "unknown"
 
    angle_to_up = np.degrees(np.arccos(
        np.clip(abs(np.dot(circle_normal / np.linalg.norm(circle_normal),
                           UP_VECTOR)), 0, 1)
    ))
 
    if angle_to_up > (90 - angle_threshold_deg):
        return "wall"
    elif angle_to_up < angle_threshold_deg:
        if circle_z < ground_z + 0.5:
            return "ground"
        else:
            return "roof"
    else:
        return "inclined"
 
 
# Wall-Local Projection: -------------------------------------------------------
 
def project_onto_wall(circle_pos: np.ndarray, corner_pos: np.ndarray,
                      wall_normal: np.ndarray) -> tuple[float, float]:
    """
    Project the circle-to-corner offset onto the wall's local 2D axes.
    Returns (meters_down, meters_right) where positive = down/right.
    """
    down  = DOWN_VECTOR
    right = np.cross(wall_normal, down)
    right_norm = np.linalg.norm(right)
 
    if right_norm < 1e-6:
        # Wall normal is parallel to gravity — shouldn't happen for a wall
        return 0.0, 0.0
 
    right = right / right_norm
 
    offset = circle_pos - corner_pos
 
    meters_down  = float(np.dot(offset, down))
    meters_right = float(np.dot(offset, right))
 
    return meters_down, meters_right
 
 
# Corner Labeling: -------------------------------------------------------
# Needs north reference vector to map corners to compass labels.
# Once north is known, pass it in. Until then, uses fallback labels.
 
def label_corners(corners: list[InferredCorner],
                  north_vector: Optional[np.ndarray] = None):
    """
    Assign compass labels (e.g. 'northwest') to each corner.
    Falls back to numbered labels if north is unknown.
    """
    if north_vector is None:
        for i, corner in enumerate(corners):
            corner.label = f"corner_{i+1}"
        return
 
    # Project each corner onto horizontal plane relative to building centroid
    centroid = np.mean([c.position for c in corners], axis=0)
 
    east = np.cross(UP_VECTOR, north_vector)
    east = east / np.linalg.norm(east)
 
    for corner in corners:
        offset = corner.position - centroid
        ns = float(np.dot(offset, north_vector))
        ew = float(np.dot(offset, east))
 
        ns_label = "north" if ns >= 0 else "south"
        ew_label = "east"  if ew >= 0 else "west"
        corner.label = f"{ns_label}{ew_label}"
 
 
# Nearest Corner: -------------------------------------------------------
 
def get_nearest_corner(circle_pos: np.ndarray,
                       corners: list[InferredCorner]) -> Optional[InferredCorner]:
    """Find the corner closest to a circle position."""
    if not corners:
        return None
    dists = [np.linalg.norm(circle_pos - c.position) for c in corners]
    return corners[int(np.argmin(dists))]
 
 
# Compass Direction for Walls: -------------------------------------------------------
# Needs north reference vector. Returns raw normal as fallback.
 
def get_compass_label(wall_normal: np.ndarray,
                      north_vector: Optional[np.ndarray] = None) -> str:
    """
    Map a wall normal to a compass direction string.
    Supports 8 directions: N, NE, E, SE, S, SW, W, NW.
    """
    if north_vector is None:
        return f"[normal: {wall_normal[0]:.2f}, {wall_normal[1]:.2f}]"

    horiz = wall_normal.copy()
    horiz[2] = 0.0
    norm = np.linalg.norm(horiz)
    if norm < 1e-6:
        return "horizontal"
    horiz = horiz / norm

    east = np.cross(UP_VECTOR, north_vector)
    east = east / np.linalg.norm(east)

    ns = float(np.dot(horiz, north_vector))
    ew = float(np.dot(horiz, east))

    # Use atan2 to get angle, then map to 8 compass sectors
    angle_deg = np.degrees(np.arctan2(ew, ns))  # 0=N, 90=E, -90=W, 180=S
    if angle_deg < 0:
        angle_deg += 360

    # 8 sectors of 45 degrees each, centered on each direction
    sectors = [
        (337.5, 360.0, "north"), (0.0, 22.5, "north"),
        (22.5,   67.5, "northeast"),
        (67.5,  112.5, "east"),
        (112.5, 157.5, "southeast"),
        (157.5, 202.5, "south"),
        (202.5, 247.5, "southwest"),
        (247.5, 292.5, "west"),
        (292.5, 337.5, "northwest"),
    ]

    for low, high, label in sectors:
        if low <= angle_deg < high:
            return label

    return "north"  # fallback
 
 
# Output Formatter: -------------------------------------------------------
 
def describe_circle(circle: dict, ground_z: float,
                    corners: list[InferredCorner],
                    north_vector: Optional[np.ndarray] = None) -> str:
    """Generate a plain-English description for one clustered circle."""
 
    colour  = circle["colour"].capitalize()
    pos     = circle["position_snapped"]
    normal  = circle["normal"]
    wall    = circle.get("wall")
 
    surface = classify_surface(
        normal, circle.get("plane_type"), pos[2], ground_z
    )
 
    nearest = get_nearest_corner(pos, corners)
 
    if surface == "wall" and wall is not None and nearest is not None:
        wall_dir = get_compass_label(wall.normal, north_vector)
        down, right = project_onto_wall(pos, nearest.position, wall.normal)
        height_agl = pos[2] - ground_z
 
        down_str  = f"{abs(down):.1f}m {'below' if down > 0 else 'above'}"
        right_str = f"{abs(right):.1f}m {'right' if right > 0 else 'left'}"
 
        return (
            f"{colour} circle on the {wall_dir}-facing wall, "
            f"{height_agl:.1f}m above ground, "
            f"{down_str} and {right_str} of the {nearest.label} corner."
        )
 
    elif surface == "ground" and nearest is not None:
        if wall is not None:
            wall_dir = get_compass_label(wall.normal, north_vector)
            eq = wall.equation
            dist_from_wall = abs(eq[0]*pos[0] + eq[1]*pos[1] +
                                  eq[2]*pos[2] + eq[3])
            dist_from_wall /= np.linalg.norm(eq[:3])
            return (
                f"{colour} circle on the ground, "
                f"{dist_from_wall:.1f}m from the {wall_dir} face, "
                f"near the {nearest.label} corner."
            )
        else:
            return (
                f"{colour} circle on the ground, "
                f"near the {nearest.label} corner."
            )
 
    elif surface == "roof" and nearest is not None:
        height_agl = pos[2] - ground_z
        return (
            f"{colour} circle on the roof, "
            f"{height_agl:.1f}m above ground, "
            f"near the {nearest.label} corner."
        )
 
    elif surface == "inclined" and nearest is not None:
        height_agl = pos[2] - ground_z
        return (
            f"{colour} circle on an inclined surface, "
            f"{height_agl:.1f}m above ground, "
            f"near the {nearest.label} corner."
        )
 
    else:
        height_agl = pos[2] - ground_z
        return (
            f"{colour} circle, {height_agl:.1f}m above ground, "
            f"position: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]."
        )
 
 
def format_all_circles(paper_clusters: list[dict], ground_z: float,
                       corners: list[InferredCorner],
                       north_vector: Optional[np.ndarray] = None) -> str:
    """Main entry point for output — returns the full output text."""
    lines = []
    for i, circle in enumerate(paper_clusters, 1):
        desc = describe_circle(circle, ground_z, corners, north_vector)
        lines.append(f"Target {i}: {desc}")
    return "\n".join(lines)



# ── Main Pipeline ───────────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    # Ingest
    acc  = ingest_json_file("flight_data.jsonl")
    data = acc.to_numpy()

    # Cluster papers
    paper_clusters = cluster_papers(data)

    # Cluster wall planes and find ground
    wall_planes  = cluster_planes([p for p in acc.all_planes if p.plane_type == "VERTICAL"])
    ground_plane = find_ground_plane(acc.all_planes)  # function filters internally

    # Infer corners from wall intersections + ground
    if ground_plane is not None:
        corners = infer_corners(wall_planes, ground_plane)
    else:
        print("Warning: no ground plane detected — cannot infer corners.")
        corners = []

    # Summary
    print(f"\nPipeline results:")
    print(f"  → {len(paper_clusters)} unique circles")
    print(f"  → {len(wall_planes)} unique wall planes")
    print(f"  → {len(corners)} inferred corners")
    print(f"  → Ground plane: {'found' if ground_plane else 'NOT FOUND'}")

    for i, c in enumerate(paper_clusters, 1):
        print(f"  Circle {i}: {c['colour']} at {c['position']}")
    for i, corner in enumerate(corners, 1):
        print(f"  Corner {i}: {corner.position}")
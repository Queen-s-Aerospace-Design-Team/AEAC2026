# Schema

## Status

This document describes the target live handoff schema.

The current runtime in the repo is still replay-first, but it now also writes frame-oriented outputs during replay processing/finalization:

- per-segment frame caches under `cache/<segment_id>/frames.<suffix>.jsonl`
- finalized combined frame output at `stream/frames.jsonl`

Live append-only writing during recording still remains to be implemented.

## Primary Handoff Artifact

Primary contract to Lucas:

```text
runs/<session_id>/stream/frames.jsonl
```

Rules:

- append-only
- one JSON object per frame
- read live on the Jetson

## Top-Level Frame Payload

Target shape:

```json
{
  "session_id": "session-001",
  "frame_idx": 42,
  "timestamp_ns": 1710000000,
  "tracking_state": "OK",
  "tracking_confidence": 99,
  "pose_world_xyz": [1.2, 0.4, 1.8],
  "pose_world_xyzw": [0.0, 0.0, 0.0, 1.0],
  "planes": [],
  "papers": []
}
```

## Frame Fields

### `session_id`

What it is:

- the unique id for the current measurement session

How it is determined:

- generated when the session starts
- typically comes from the configured session id or a generated session name

How it is used downstream:

- groups all frames from the same flight/run together
- lets Lucas avoid mixing measurements from different sessions
- helps with log/debug correlation

### `frame_idx`

What it is:

- the monotonically increasing frame number within the session

How it is determined:

- incremented once for each ZED frame the pipeline packages into the live stream

How it is used downstream:

- preserves ordering
- lets Lucas detect dropped/skipped frames
- allows frame-level grouping and replay/debug analysis

### `timestamp_ns`

What it is:

- the frame timestamp in nanoseconds

How it is determined:

- taken from the current ZED frame / pose timestamp
- intended to reflect the actual live measurement time, not file write time

How it is used downstream:

- time-aligns detections and clustering state
- helps correlate the frame with detector output and flight events
- supports latency/debug analysis

### `tracking_state`

What it is:

- the current ZED positional tracking state for that frame

How it is determined:

- read directly from the ZED positional tracking API during the live frame loop
- expected values include states such as `OK` or non-OK tracking states

How it is used downstream:

- lets Lucas reject or down-weight measurements collected when pose was unstable
- helps separate real geometry issues from tracking failures
- provides an immediate quality gate at the frame level

### `tracking_confidence`

What it is:

- ZED's confidence value for the current tracked pose

How it is determined:

- read from the ZED pose/tracking result for that frame

How it is used downstream:

- provides a softer quality signal than `tracking_state`
- allows Lucas or later logic to threshold low-confidence frames
- helps debugging pose drift and weak localization periods

### `pose_world_xyz`

What it is:

- the camera position in the world frame as `[x, y, z]`

How it is determined:

- read from the ZED live pose estimate for the current frame
- expressed in the configured world coordinate system

How it is used downstream:

- anchors every paper measurement to the drone/camera pose for that frame
- allows Lucas to reason about repeated observations across time
- helps debug pose drift and spatial consistency

### `pose_world_xyzw`

What it is:

- the camera orientation in the world frame as a quaternion `[x, y, z, w]`

How it is determined:

- read from the ZED live pose estimate for the current frame

How it is used downstream:

- preserves the camera orientation for any downstream geometric reasoning
- helps interpret the camera-frame vs world-frame relationship
- supports debugging when world points look wrong but pose position alone seems reasonable

### `papers`

What it is:

- the list of all paper measurements produced for that frame

How it is determined:

- the external detector provides current-frame paper centers, classes/colors, and confidence
- this repo uses those detections with ZED depth, pose, and plane queries to build one per-paper measurement object
- if no papers are detected, this is an empty list

How it is used downstream:

- this is the main payload Lucas clusters
- each paper entry contains the world point, depth, and a `plane_id` link to the supporting plane for that paper
- repeated paper entries across frames let Lucas infer stable paper locations while the linked planes support downstream corner inference

### `planes`

What it is:

- the list of unique planes discovered for that frame

How it is determined:

- the ZED SDK does not provide an "all planes in frame" call
- instead, this repo issues multiple plane queries each frame:
  - one `find_plane_at_hit` query at each paper center
  - additional `find_plane_at_hit` queries at a small fixed set of scene probe pixels so walls without papers can still be discovered
  - an optional `find_floor_plane` query if floor context is needed
- successful plane results are deduplicated into unique frame-level plane objects

How it is used downstream:

- gives Lucas multiple wall or floor planes in a single frame instead of only the plane under each paper
- allows Lucas to intersect multiple planes across time to infer corners
- serves as the canonical plane list that `papers[].plane_id` points into

Optional future fields:

- camera metadata
- runtime warnings
- detector timing metadata

## Per-Paper Measurement

Each item in `papers` should contain one live paper measurement.

Target shape:

```json
{
  "label": "paper",
  "color": "red",
  "detector_confidence": 0.93,
  "center_uv": [320, 180],
  "depth_m": 2.2,
  "point_world_xyz": [3.1, 1.7, 1.2],
  "point_camera_xyz": [1.9, 1.3, -0.6],
  "plane_id": "plane-0001",
  "plane_query_status": "SUCCESS",
  "quality_flags": ["clean"]
}
```

## Per-Paper Fields

### `label`

What it is:

- the semantic label for the detected object

How it is determined:

- provided by the external detector for the current frame
- for this pipeline it will usually be a stable value such as `paper`

How it is used downstream:

- confirms the observation type
- leaves room for future expansion if other object types are added
- helps validate that Lucas is only clustering the intended measurement class

### `color`

What it is:

- the paper color or class identifier associated with the detection

How it is determined:

- provided by the external detector
- carried through unchanged into the frame payload

How it is used downstream:

- this is one of Lucas's primary grouping keys
- allows clustering by fire-marker type or paper color
- helps prevent mixing separate target classes into the same cluster

### `detector_confidence`

What it is:

- the detector's confidence score for the paper observation

How it is determined:

- provided directly by the external YOLO + Roboflow detector for the center-point detection

How it is used downstream:

- allows low-confidence detections to be filtered or down-weighted
- helps separate detector noise from ZED depth or pose issues
- supports debugging detector quality during flight review

### `center_uv`

What it is:

- the paper center pixel in image coordinates as `[u, v]`

How it is determined:

- provided by the external detector
- this repo uses that exact center pixel as the measurement location

How it is used downstream:

- identifies the image-space source of the measurement
- allows the 3D world point and plane query to be traced back to the exact pixel
- helps with debugging if a 3D point looks wrong and the team needs to inspect the original image location

### `depth_m`

What it is:

- the depth value in meters at the detected paper center pixel

How it is determined:

- read from the live ZED depth data at `center_uv`
- expressed in meters using the configured ZED unit system

How it is used downstream:

- gives Lucas the direct range measurement associated with the paper observation
- helps sanity-check world-point geometry
- supports filtering observations where the center-pixel depth is missing or clearly bad

### `point_world_xyz`

What it is:

- the measured paper point in the world frame as `[x, y, z]`

How it is determined:

- derived from the ZED point cloud or depth information at the center pixel
- transformed into the world frame using the current live ZED pose for that frame

How it is used downstream:

- this is the main spatial measurement Lucas clusters over time
- repeated world-frame observations of the same paper should converge to a stable location
- it is one of the most important fields for downstream geometry and final text-file generation

### `point_camera_xyz`

What it is:

- the measured paper point in the camera frame as `[x, y, z]`

How it is determined:

- read or derived from the ZED live 3D point at the paper center pixel before world-frame transformation

How it is used downstream:

- helps debug whether an error came from bad local depth or bad world-frame pose
- preserves the raw camera-relative geometry for validation
- can be used to compare local measurements across tracking modes

### `plane_id`

What it is:

- the identifier of the frame-level plane this paper is associated with

How it is determined:

- this repo runs `find_plane_at_hit` at the paper center pixel
- if the query succeeds, the resulting plane is matched to one item in the top-level `planes` list and that plane's id is stored here
- if no usable plane is found, this field is empty or `null`

How it is used downstream:

- tells Lucas exactly which plane supports the paper observation
- lets Lucas join paper measurements to the canonical per-frame plane list without duplicating full plane geometry inside every paper object
- makes it possible to reason about which papers share the same wall or surface

### `plane_query_status`

What it is:

- the status of the plane query performed at the paper center pixel

How it is determined:

- returned by the ZED SDK plane query for that paper center
- `SUCCESS` means a plane was found and linked to a `plane_id`

How it is used downstream:

- tells Lucas whether the paper-to-plane link should be trusted
- separates "paper detected but no plane found" from other failure modes
- helps with debugging and filtering

### `quality_flags`

What it is:

- a list of simple pipeline quality markers attached to the paper observation

How it is determined:

- assigned by this repo after checking the validity of tracking, depth, 3D point, and plane query results
- intended values are listed in the `Quality Flags` section below

How it is used downstream:

- gives Lucas a simple way to reject or down-weight bad observations
- makes the payload easier to work with than forcing downstream code to infer every failure condition from raw fields
- supports metrics, debugging, and audit review

## Per-Plane Measurement

Each item in `planes` should contain one unique plane discovered during that frame.

Target shape:

```json
{
  "plane_id": "plane-0001",
  "source": "paper_center",
  "seed_uv": [320, 180],
  "center_world_xyz": [3.0, 1.6, 1.1],
  "normal_world_xyz": [0.0, 1.0, 0.0],
  "equation_abcd": [0.0, 1.0, 0.0, -1.6],
  "type": "VERTICAL"
}
```

## Per-Plane Fields

### `plane_id`

What it is:

- the stable identifier for a plane within a single frame payload

How it is determined:

- assigned by this repo after successful plane queries are deduplicated into unique frame-level planes

How it is used downstream:

- lets Lucas link `papers[].plane_id` to the correct plane object
- allows one plane to be shared by multiple papers in the same frame
- gives a simple join key for frame-local processing

### `source`

What it is:

- the source that produced the plane candidate

How it is determined:

- assigned by this repo based on which query created the plane
- intended values include `paper_center`, `scene_probe`, and optionally `floor`

How it is used downstream:

- shows whether the plane came from a paper hit or a non-paper scene probe
- helps debug why a plane is present in a frame
- makes it easier to tune the non-paper wall discovery strategy later

### `seed_uv`

What it is:

- the image pixel used to query the plane for hit-based detections

How it is determined:

- copied from the pixel passed into `find_plane_at_hit`
- may be empty for floor-plane detections that do not use a hit pixel

How it is used downstream:

- helps trace a plane back to the exact image location that produced it
- supports debugging and visual overlay tooling
- helps explain why two frame planes are considered different or the same

### `center_world_xyz`

What it is:

- the plane center in world coordinates as reported by the ZED plane result

How it is determined:

- taken from the `sl.Plane` result for the successful plane query
- expressed in the session world frame

How it is used downstream:

- gives Lucas a representative location for the plane in the scene
- helps with plane-plane spatial reasoning and visualization
- supports debugging of plane placement

### `normal_world_xyz`

What it is:

- the plane normal vector in world coordinates as `[x, y, z]`

How it is determined:

- taken from the ZED plane result
- normalized by the SDK plane representation

How it is used downstream:

- this is one of Lucas's key inputs for inferring corners
- allows plane orientation comparisons across papers and frames
- helps separate different walls even when centers are spatially close

### `equation_abcd`

What it is:

- the plane equation coefficients `[a, b, c, d]`

How it is determined:

- taken from the ZED plane result
- represents the plane in the form `ax + by + cz + d = 0`

How it is used downstream:

- gives Lucas the full mathematical plane representation for intersections and distance checks
- supports more precise downstream geometry than the normal alone
- allows comparison and deduplication across repeated observations

### `type`

What it is:

- the ZED plane classification

How it is determined:

- returned by the ZED plane result when available
- expected values include categories such as `VERTICAL` or `HORIZONTAL`

How it is used downstream:

- helps Lucas quickly filter for wall-like planes when looking for room corners
- improves debugging readability
- supports simple rule-based validation

## Quality Flags

Current intended values:

- `pose_invalid`
- `point_invalid`
- `plane_unavailable`
- `depth_invalid`
- `clean`

## Fields Lucas Should Treat As Primary

- top-level:
  - `frame_idx`
  - `timestamp_ns`
  - `tracking_state`
  - `pose_world_xyz`
  - `planes`
- per-paper:
  - `color`
  - `point_world_xyz`
  - `plane_id`
  - `plane_query_status`
  - `quality_flags`
- per-plane:
  - `normal_world_xyz`
  - `equation_abcd`
  - `type`

## What This Repo No Longer Owns

This repo does not emit explicit corner measurements in the target design.

Instead:

- this repo emits papers plus per-frame plane information
- Lucas infers corners downstream

## Session Files

Expected session files:

```text
runs/<session_id>/
  session.json
  capture/session.svo2
  stream/frames.jsonl
  logs/events.jsonl
```

`capture/session.svo2` is optional and only exists when `recording.enable_svo_recording` is `true`.

## Detector Input Contract

The detector is external. The exact input contract into this repo is still not final, but the working assumptions are:

- center pixel
- color/class
- confidence
- frame association

See [Uncertainties.md](Uncertainties.md).

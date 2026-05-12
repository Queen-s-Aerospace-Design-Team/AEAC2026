# Technical Design

## Status

This document describes the target live-first architecture.

The current runtime in the repo is still replay-first, but replay processing/finalization now implements the frame-level plane model described here:

- top-level `planes[]`
- per-paper `plane_id`
- fixed 3x3 scene probes
- configurable floor-plane inclusion, off by default

## Architecture

The system is organized around a thin orchestration layer over the ZED SDK.

Core principle:

- ZED owns camera I/O, depth, live pose estimation, and plane queries.
- The external detector owns paper center inference.
- This repo owns live session orchestration, frame-level plane discovery, paper-to-plane association, append-only handoff to Lucas, and optional SVO recording.
- Lucas owns clustering, inferred corners, and final text-file generation.

## Runtime Topology

All major components run on the Jetson onboard the drone:

- ZED live capture + tracking
- this measurement pipeline
- Lucas's clustering code
- external YOLO + Roboflow detector

## Main Modules

- `src/zed_positional_measurement/sdk.py`
  - ZED adapter
  - live camera open/close
  - tracking enable/disable
  - SVO recording enable/disable
  - pose, point, depth, and plane extraction
- `src/zed_positional_measurement/pipeline.py`
  - live session orchestration
  - start/stop session lifecycle
  - frame packaging
  - append-only JSONL handoff
- `src/zed_positional_measurement/storage.py`
  - session directory creation
  - append-only stream/log persistence
  - metadata writes
- detector adapter boundary
  - placeholder interface only
  - receives external paper detections for the current frame
- `src/zed_positional_measurement/models.py`
  - frame payload and per-paper measurement types
- `src/zed_positional_measurement/metrics.py`
  - offline tracking and measurement metrics

## Session Layout

```text
runs/<session_id>/
  session.json
  capture/
    session.svo2
  stream/
    frames.jsonl
  logs/
    events.jsonl
```

Design intent:

- `capture/` stores the parallel SVO2 recording when `recording.enable_svo_recording` is enabled.
- `stream/frames.jsonl` is the live append-only handoff contract for Lucas.
- `logs/` stores operational events and errors for debugging.

## Live Processing Flow

### 1. Start Session

The session controller:

- opens the ZED camera with:
  - `RIGHT_HANDED_Z_UP_X_FORWARD`
  - meter units
  - configured resolution/fps/depth mode
- enables positional tracking
- enables SVO2 recording in parallel only when `recording.enable_svo_recording` is on
- opens the append-only frame stream output
- begins accepting detector results for live frames

### 2. For Each Frame

Per grabbed ZED frame:

1. grab frame from the ZED camera
2. get current pose/tracking state from ZED
3. receive paper center detections from the external detector boundary
4. for each paper detection:
   - use the detector center pixel
   - sample center-pixel depth
   - sample the world point at that center pixel
   - query `find_plane_at_hit` at the same pixel
5. run additional `find_plane_at_hit` queries at a small fixed set of scene probe pixels so walls without papers can still produce planes
6. optionally run `find_floor_plane` if floor-plane context is needed
7. deduplicate successful plane results into one frame-level `planes` list
8. assign each paper a `plane_id` pointing at the matching plane from its paper-center query when available
9. package one frame object containing:
   - frame metadata
   - ZED pose/tracking data
   - zero or more frame-level planes
   - zero or more paper measurements
10. append that frame object to `stream/frames.jsonl`

### 3. Stop Session

The session controller:

- stops live measurement emission
- closes SVO recording if it was enabled
- closes the camera
- flushes the append-only stream

## Tracking Modes

Recommended live default:

- `vslam_map`

Reason:

- it uses live visual SLAM with area memory and best matches the need for minimizing drift over the flight
- `GEN_3` is the recommended ZED tracking mode for modern use

Other modes:

- `vio`
  - fallback if mapping behavior is not wanted
- `gnss_fusion`
  - reserved for future RTK/GNSS integration

Recommended settings:

- positional tracking mode: `GEN_3`
- `enable_imu_fusion = true`
- `set_gravity_as_origin = true`
- `set_floor_as_origin = false`
- `enable_pose_smoothing = false`
- `enable_2d_ground_mode = false`

## Detector Boundary

The detector is intentionally outside this repo.

This repo should expose a small adapter interface that accepts current-frame paper detections, not own the YOLO + Roboflow implementation.

Assumed detector outputs:

- frame id or timestamp
- paper center pixel
- paper color/class
- confidence

The exact integration method is still tracked in [Uncertainties.md](Uncertainties.md).

## Measurement Flow

### Plane Discovery

The official ZED plane-detection API is hit-based, not full-scene enumeration.

That means:

- `find_plane_at_hit` returns the support plane for a specific target pixel
- `find_floor_plane` is a separate floor-specific query
- the SDK does not expose a native "return all visible planes in this frame" call

So the feasible design for multiple planes per frame is:

1. query each paper center with `find_plane_at_hit`
2. query a small fixed set of additional scene probe pixels each frame
3. optionally query the floor plane
4. deduplicate successful results into a frame-level `planes` array

Implemented default:

- 9 extra scene probes
- fixed 3x3 grid using normalized image-space probe centers
- `include_floor_plane = false`
- simple dedupe using plane-normal angle and plane-center distance thresholds

Consequence:

- if no probe lands on a visible wall, that wall will not appear in the frame payload
- getting multiple walls per frame is feasible, but it depends on the probe policy this repo chooses

### Paper Measurements

For each detected paper center:

1. read the detector center pixel
2. sample center-pixel depth from ZED
3. sample center-pixel world point from ZED
4. run `find_plane_at_hit` at that pixel
5. if the plane query succeeds, associate the result with one item in the frame-level `planes` array
6. store that plane link as `plane_id` in the paper measurement
7. emit the per-paper measurement inside the frame payload

### Corners

This repo does not calculate corners.

Instead:

- this repo emits paper measurements, frame-level planes, and paper-to-plane links
- Lucas infers corners downstream from repeated measurements and plane normals

## Output Contract

Primary live contract:

```text
runs/<session_id>/stream/frames.jsonl
```

Rules:

- append-only
- one JSON object per frame
- Lucas reads this file live on the same Jetson

## Quality Model

Each per-paper measurement carries quality flags.

Current intended flags:

- `pose_invalid`
- `point_invalid`
- `plane_unavailable`
- `depth_invalid`
- `clean`

`clean` means:

- pose is valid
- a world point exists
- depth exists

## Failure Strategy

- if tracking is not `OK`, still emit the frame with explicit invalid/weak quality flags
- if a paper-center plane query fails, still emit the paper measurement with `plane_unavailable` and no `plane_id`
- if the extra scene probes fail to find other walls, still emit the frame with whatever planes were found
- if the detector returns no papers for a frame, still emit the frame object with an empty papers list
- if the detector boundary stalls, the live ZED loop should not block indefinitely

## Known Limitations

- the ZED SDK plane API does not natively enumerate every visible plane in a frame
- the current 3x3 probe grid and dedupe thresholds are configurable but still need hardware tuning
- the exact detector integration mechanism is still unresolved
- the exact live JSON payload may be refined after Matt clarifies the minimum fields Lucas needs
- `gnss_fusion` is not yet wired

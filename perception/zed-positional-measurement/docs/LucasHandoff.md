# Lucas Handoff

## Status

This document describes the intended live handoff contract.

The current runtime in the repo is still replay-first.

What exists now:

- replay processing/finalization writes frame-oriented outputs with `planes[]` and `papers[].plane_id`

What still needs the live refactor:

- writing those frame objects live during recording for Lucas to tail in flight

## Purpose

This document explains exactly what Lucas should read from this pipeline, where it lives on disk, and how it connects to his clustering code.

Lucas's role:

- read the live frame stream from this repo
- cluster repeated paper measurements
- use multiple frame-level planes, plane normals, and paper-to-plane associations
- infer corners downstream
- emit the final text file expected by the client layer

## Runtime Relationship

Lucas's code runs on the same Jetson on the drone.

This means the simplest and recommended boundary is a local append-only file handoff, not replay-first processing and not a remote network dependency.

## Canonical Live Input

Lucas should read:

```text
runs/<session_id>/stream/frames.jsonl
```

Properties:

- append-only
- one JSON object per frame
- written live while the drone is flying
- durable on disk for later review

## What Lucas Gets Per Frame

Each line in `frames.jsonl` represents one frame object containing:

- frame metadata
- current ZED pose and tracking state
- zero or more frame-level planes
- zero or more paper measurements for that frame

This is not one JSON line per observation.

It is one JSON line per frame.

## Frame Object Shape

Target top-level structure:

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

Each item in `papers` should contain:

- paper label/color
- detector confidence
- detector center pixel
- center-pixel depth
- world point
- camera-frame point
- `plane_id` linking the paper to one item in `planes`
- plane query status
- quality flags

Each item in `planes` should contain:

- a frame-local `plane_id`
- plane center in world coordinates
- plane normal
- plane equation
- plane type
- source metadata showing whether the plane came from a paper center, scene probe, or floor query

## Fields Lucas Should Use

Primary top-level fields:

- `frame_idx`
- `timestamp_ns`
- `tracking_state`
- `tracking_confidence`
- `pose_world_xyz`
- `pose_world_xyzw`

Primary per-paper fields:

- `color`
- `detector_confidence`
- `center_uv`
- `depth_m`
- `point_world_xyz`
- `plane_id`
- `plane_query_status`
- `quality_flags`

Primary per-plane fields:

- `plane_id`
- `normal_world_xyz`
- `equation_abcd`
- `type`

## What This Repo Does Not Send

In the target design, this repo does **not** calculate or emit corners.

Instead:

- this repo emits paper measurements plus frame-level plane data
- Lucas infers corners from repeated measurements and plane normals

## Recommended First-Pass Filter For Lucas

For the first integration, Lucas should keep paper measurements where:

- `tracking_state == "OK"` at the frame level
- `point_world_xyz` is present
- `clean` is present in `quality_flags`

If a step specifically needs plane data, also require:

- `plane_id` is present on the paper
- the referenced plane object exists in `planes`
- `normal_world_xyz` is present on that plane

## Connection Model

### Intended v1 Connection

1. the ZED pipeline runs live
2. the external detector supplies current-frame paper centers
3. this repo runs multiple ZED plane queries for the frame
4. this repo packages one frame JSON object with `planes[]` and `papers[]`
5. this repo appends that frame object to `stream/frames.jsonl`
6. Lucas tails the same file locally and clusters measurements live

There is no replay dependency in the intended runtime path.

## Example Paper Payload

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

## Example Plane Payload

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

## Why There Are Multiple Planes Per Frame

Lucas needs multiple planes over time to infer corners from plane intersections, and papers may exist on only some walls.

Because of that, this repo should not only query the plane at each paper center. It should also run extra scene-plane queries each frame so walls without papers can still appear in the payload.

Important ZED SDK constraint:

- `find_plane_at_hit` gives the plane for a specific target pixel
- `find_floor_plane` is a separate floor-only query
- the SDK does not automatically return every visible plane in the image

So multiple planes per frame are feasible, but only by running multiple plane queries that this repo selects.

## SVO Recording

SVO2 is only recorded in parallel when `recording.enable_svo_recording` is `true`, but Lucas should not depend on replay to get measurements.

The SVO file exists for:

- audit/debugging
- future offline analysis
- neural-net workflows

## Summary

If Lucas wants the simplest correct integration, he should:

1. tail `runs/<session_id>/stream/frames.jsonl`
2. read one JSON object per frame
3. cluster on per-paper `point_world_xyz`
4. use each paper's `plane_id` to look up the supporting plane in `planes[]`
5. use `normal_world_xyz`, `equation_abcd`, and `type` from the referenced planes
6. infer corners downstream from those live measurements

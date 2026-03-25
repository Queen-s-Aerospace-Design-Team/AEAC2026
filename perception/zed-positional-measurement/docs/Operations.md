# Operations

## Status

This document describes the intended live-first operating model.

The current code in the repo still exposes the older replay-first CLI.

What is implemented now in that replay path:

- per-frame plane probing
- top-level `planes[]`
- per-paper `plane_id`
- fixed 3x3 scene probes

Treat the rest of this document as the target runtime contract for the upcoming live-first refactor.

## Purpose

This document is the runbook for the live ZED measurement service.

## Runtime Model

The intended runtime model is:

- explicit `start_session()`
- live measurement loop while the drone is flying
- explicit `stop_session()`
- optional parallel SVO2 recording during the session
- append-only frame JSON written throughout the flight

Lucas reads the live frame stream on the same Jetson.

## Start / Stop Control Contract

Target control boundary:

- `start_session(config)`
- `stop_session()`

Recommended immediate shape:

- a service class for the real runtime boundary
- a thin CLI wrapper only for local testing/debugging

The actual button/UI process that calls start/stop is still tracked in [Uncertainties.md](Uncertainties.md).

## Session Outputs

Expected session layout:

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

### `session.json`

Stores:

- session id
- created time
- config snapshot
- runtime metadata

### `capture/session.svo2`

Stores:

- full ZED SVO recording for the session when `recording.enable_svo_recording` is `true`

Purpose:

- audit/debugging
- future neural-net workflows
- offline replay if needed later

### `stream/frames.jsonl`

Stores:

- one JSON object per frame containing:
  - frame metadata
  - frame-level `planes`
  - frame-level `papers`

Purpose:

- live handoff to Lucas
- durable append-only session record

## Config Reference

### `recording`

- `resolution`
- `fps`
- `depth_mode`
- `coordinate_system`
- `coordinate_units`
- `compression_mode`
- `enable_svo_recording`

Recommended defaults:

- `depth_mode = NEURAL`
- `coordinate_system = RIGHT_HANDED_Z_UP_X_FORWARD`
- `coordinate_units = METER`

Important:

- if `enable_svo_recording` is `false`, the target live-first system should still run measurement and stream output
- in the current replay-first code, `run`, `process`, and `finalize` still depend on SVO recording being enabled

### `tracking`

- `mode`
- `positional_tracking_mode`
- `enable_imu_fusion`
- `set_gravity_as_origin`
- `set_floor_as_origin`
- `enable_pose_smoothing`
- `enable_2d_ground_mode`

Recommended live default:

- `mode = vslam_map`
- `positional_tracking_mode = GEN_3`

### `detector`

This repo should eventually expose config for the detector adapter boundary, but the exact integration mechanism is still unresolved.

Known expectations:

- detector is external to this repo
- detector returns paper center estimates
- this repo consumes those estimates live

### `runtime`

Expected live-runtime settings:

- `root_dir`
- `session_id`
- optional stream flush policy
- optional detector timeout policy

### `plane_detection`

- `enable_scene_probes`
- `scene_probe_points_normalized`
- `include_floor_plane`
- `dedupe_normal_angle_deg`
- `dedupe_center_distance_m`

Implemented default:

- `enable_scene_probes = true`
- `scene_probe_points_normalized =` fixed 3x3 normalized grid
- `include_floor_plane = false`
- `dedupe_normal_angle_deg = 10.0`
- `dedupe_center_distance_m = 0.4`

## Detector Input Contract

Target contract for each detector result:

- current ZED frame association
- paper center pixel
- paper color/class
- confidence

The exact transport is still tracked in [Uncertainties.md](Uncertainties.md).

## Recommended Operator Flow

1. Prepare `config.json`.
2. Confirm the external detector process is available.
3. Start the measurement session from the button/UI or service boundary.
4. Let the drone fly while the system:
   - tracks live pose
   - runs per-frame plane queries
   - records SVO2 if enabled
   - appends `stream/frames.jsonl`
5. Stop the session from the same control boundary.
6. Inspect:
   - `runs/<session_id>/stream/frames.jsonl`
   - `runs/<session_id>/capture/session.svo2` if SVO recording is enabled
   - `runs/<session_id>/logs/`

## Failure Handling

- If the detector returns no papers for a frame, still emit the frame with an empty `papers` list.
- If the plane query at a paper center fails, still emit that paper with no `plane_id`, explicit `plane_query_status`, and quality flags.
- If additional scene-plane probes do not find extra walls, still emit the frame with whatever planes were found.
- If tracking is weak or invalid, still emit the frame and mark the measurement quality accordingly.
- If the live detector connection fails, the ZED session should keep running, and SVO recording should continue if enabled.

## What To Inspect First When Results Look Wrong

- `tracking_state`
- `tracking_confidence`
- `planes`
- `papers[].plane_id`
- `point_world_xyz`
- `depth_m`
- `plane_query_status`
- `planes[].normal_world_xyz`
- `quality_flags`
- frame-to-detector alignment issues

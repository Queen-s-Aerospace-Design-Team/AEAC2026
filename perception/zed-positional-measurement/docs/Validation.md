# Validation

## Status

This document describes validation for the target live-first system.

The automated repo tests are now aligned with the current implemented frame-level plane design in the replay pipeline:

- frame caches and combined frame exports include top-level `planes[]`
- papers include `plane_id`
- the default 3x3 scene-probe grid is exercised in tests
- the current exported metrics are covered directly

The remaining gap is the live runtime path during recording, which still needs separate hardware validation.

## Purpose

This document describes how to validate the live ZED measurement pipeline without inflating the metric set.

## Automated Repo Tests

The repo includes deterministic non-hardware tests for the current codebase.

Run:

```bash
python -m unittest discover -s tests -v
```

These tests remain useful for:

- config/model logic
- geometry helpers
- append/write behavior
- per-frame plane probing
- paper-to-plane association
- finalized `stream/frames.jsonl` export
- metric computation

What the current automated tests do not cover:

- live append-only writing during recording
- real detector integration
- real ZED hardware behavior
- GNSS/RTK

## Current Test Coverage

### [tests/test_config.py](../tests/test_config.py)

`test_pipeline_config_round_trip_preserves_nested_classes`

- What it checks:
  - config parsing and serialization preserve nested class mappings and the new plane-detection settings
- Why it matters:
  - ensures the JSON config contract is stable and the repo reads back what it writes

`test_scale_helpers_convert_detector_coordinates_to_native_image_size`

- What it checks:
  - detector-space bounding boxes and points rescale correctly into the native ZED image size
- Why it matters:
  - bad scaling would corrupt every downstream depth, point, and plane lookup

### [tests/test_exporters.py](../tests/test_exporters.py)

`test_csv_export_includes_only_clean_rows`

- What it checks:
  - only clean measurements are emitted to the flattened CSV export
  - the export schema includes `plane_id`
- Why it matters:
  - keeps downstream CSV consumers focused on usable paper rows instead of invalid observations

### [tests/test_metrics.py](../tests/test_metrics.py)

`test_compute_tracking_metrics_matches_current_fields`

- What it checks:
  - tracking metrics are computed from pose rows using the same formulas the repo exposes through the CLI
- Why it matters:
  - protects the meaning of the reported tracking numbers from accidental regression

`test_compute_measurement_metrics_uses_papers_and_plane_success`

- What it checks:
  - paper-only measurement metrics are computed correctly
  - `plane_success_rate` and `valid_measurement_rate` use the intended denominators
- Why it matters:
  - makes sure the measurement summary matches the current paper-plus-plane design instead of the old corner-based model

### [tests/test_pipeline.py](../tests/test_pipeline.py)

`test_record_session_skips_svo_recording_when_disabled`

- What it checks:
  - the recorder does not enable SVO writing when `recording.enable_svo_recording` is `false`
- Why it matters:
  - verifies optional recording is actually optional

`test_run_live_session_requires_svo_when_replay_processing_is_enabled`

- What it checks:
  - the current replay-driven `run` path still rejects configurations that try to process/finalize without SVO recording
- Why it matters:
  - reflects the current runtime limitation explicitly instead of failing later in a less clear place

`test_process_segment_rescales_inputs_and_writes_caches`

- What it checks:
  - a replayed segment produces:
    - scaled detector inputs
    - paper/corner measurement caches
    - frame caches with top-level `planes[]`
    - `papers[].plane_id`
    - deduped planes
- Why it matters:
  - this is the core end-to-end test for the implemented frame-plane packaging logic

`test_process_session_skips_already_processed_segments`

- What it checks:
  - resume logic only processes unfinished segments
- Why it matters:
  - protects restart behavior and avoids duplicate work

`test_export_session_writes_combined_frame_stream`

- What it checks:
  - finalized replay exports the combined `stream/frames.jsonl` artifact with `planes[]` and `papers[]`
- Why it matters:
  - validates the canonical frame-stream output Lucas will consume in the current replay workflow

`test_process_segment_uses_nine_scene_probe_points_by_default`

- What it checks:
  - the default plane probing policy uses exactly 9 extra scene probes in the fixed 3x3 grid
  - floor-plane probing stays off by default
- Why it matters:
  - locks in the plane-discovery behavior you chose so it does not silently drift later

### [tests/test_storage.py](../tests/test_storage.py)

`test_pending_segments_only_returns_unfinished_segment_manifests`

- What it checks:
  - storage resume logic returns only unfinished segments for further processing
- Why it matters:
  - protects durability and resume behavior for longer sessions

## Current Repo Metrics

### Tracking

- `% tracking_state == OK`
- `loop_return_error_m`
- `max_pose_jump_m`
- `p95_pose_jump_m`
- `final_export_latency_s`

#### `% tracking_state == OK`

- What it is:
  - the fraction of pose rows where tracking is valid
- How it is calculated:
  - `valid_pose_count / total_pose_count`
  - a pose is counted as valid when `tracking_state == "OK"` and `pose_world_xyz` exists
- What it is for:
  - a simple health measure for ZED tracking stability over the run

#### `loop_return_error_m`

- What it is:
  - the distance between the first valid pose and the last valid pose
- How it is calculated:
  - Euclidean distance between the first and last `pose_world_xyz` among valid poses
- What it is for:
  - a simple drift proxy on routes that end near where they started

#### `max_pose_jump_m`

- What it is:
  - the largest frame-to-frame translation jump between consecutive valid poses
- How it is calculated:
  - compute Euclidean distance between each consecutive valid pose pair and take the maximum
- What it is for:
  - catches sudden pose discontinuities and obvious tracking instability

#### `p95_pose_jump_m`

- What it is:
  - the 95th percentile of frame-to-frame pose jumps
- How it is calculated:
  - compute all consecutive valid-pose jumps, sort them, then take the 95th percentile
- What it is for:
  - gives a more stable view of pose smoothness than the single worst jump

#### `final_export_latency_s`

- What it is:
  - the elapsed time from session close to finalized export write
- How it is calculated:
  - `(export_written_at_ns - session_closed_at_ns) / 1e9`
- What it is for:
  - tracks replay/finalization turnaround in the current implementation
  - this is the current replay metric, not the future live emit latency metric

### Measurement

- `paper_position_mae_m`
- `paper_position_p95_m`
- `plane_success_rate`
- `valid_measurement_rate`

#### `paper_position_mae_m`

- What it is:
  - mean absolute paper position error against ground truth
- How it is calculated:
  - take clean paper measurements with `point_world_xyz`
  - match each one to ground truth using `(entity_type, color_or_corner_id)`
  - compute Euclidean error for each matched paper
  - return the arithmetic mean
- What it is for:
  - the primary accuracy metric for paper localization

#### `paper_position_p95_m`

- What it is:
  - the 95th percentile paper position error against ground truth
- How it is calculated:
  - use the same matched paper error list as above and take the 95th percentile
- What it is for:
  - shows tail error, which is often more useful than the mean in this kind of system

#### `plane_success_rate`

- What it is:
  - the fraction of paper measurements that successfully linked to a plane
- How it is calculated:
  - `paper_records_with_plane_status_SUCCESS_and_plane_id / total_paper_records`
- What it is for:
  - measures how often the system can provide Lucas with the supporting plane information he needs for corner inference

#### `valid_measurement_rate`

- What it is:
  - the fraction of paper measurements that are clean enough for downstream use
- How it is calculated:
  - `clean_paper_records_with_point_world_xyz / total_paper_records`
- What it is for:
  - summarizes how much of the raw paper stream is actually usable downstream

These are the metrics currently computed by the repo.

## Metrics Still Pending With The Live Runtime Refactor

- `live_frame_emit_latency_ms`
- dropped frame rate
- detector timeout rate

These depend on the true live append-only runtime path and are not computed yet.

## Recommended Hardware Validation Flow

### 1. Prepare A Controlled Indoor Test

Use a room or hallway with:

- known paper target placements
- a route that returns near the start point
- enough wall structure for ZED tracking and plane detection

### 2. Start The Live Session

Run the live measurement service and detector together on the Jetson.

Expected live behavior:

- ZED pose updates continuously
- SVO2 is recorded in parallel if `recording.enable_svo_recording` is enabled
- `stream/frames.jsonl` grows during the session
- Lucas can read the same append-only stream live

### 3. Inspect Live Outputs

Focus on:

- `runs/<session_id>/stream/frames.jsonl`
- `runs/<session_id>/capture/session.svo2` if `recording.enable_svo_recording` is enabled
- operational logs

### 4. Check Frame Payload Quality

Look for:

- valid `tracking_state`
- non-empty `papers` on detector-positive frames
- valid `point_world_xyz`
- expected `planes[].normal_world_xyz` orientation
- valid `papers[].plane_id` links into `planes[]`
- reasonable `quality_flags`

### 5. Compute Offline Metrics

After the run, evaluate:

- pose stability
- paper position error
- plane detection success rate
- final export latency

## What Counts As A Good First Smoke Test

- the live session starts and stops cleanly
- the frame stream contains both `planes[]` and `papers[]`
- Lucas can consume the stream on the same Jetson
- SVO2 is recorded for the same session when `recording.enable_svo_recording` is enabled
- most frames with valid papers also have valid world points
- most frames with valid papers also have usable `plane_id` links
- pose remains stable enough for repeated paper observations to cluster tightly

## Known Validation Gaps

- no real camera validation was possible in this environment
- detector integration is still not wired
- no GNSS/RTK path validation yet
- the current tests validate the finalized frame JSON contract, but not live in-flight emission

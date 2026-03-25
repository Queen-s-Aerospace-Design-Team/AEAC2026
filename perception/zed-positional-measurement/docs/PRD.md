# PRD

## Summary

Build a live ZED-first spatial measurement pipeline that runs onboard the drone while recording, continuously measures paper targets in the world frame, attaches ZED pose/depth/plane information, and appends one JSON object per frame for Lucas's clustering pipeline.

The system should optionally record an SVO2 file in parallel for audit, debugging, and neural net workflows, but the primary runtime path is live, not replay-first.

The implementation should stay thin over the ZED SDK. The repo should not reimplement SLAM, depth estimation, or plane fitting when the SDK already provides those capabilities.

## Problem

The system must estimate relative locations of colored paper targets inside a building and provide usable live measurement data during the flight. The main failure modes are:

- inaccurate paper center depth measurement
- pose drift during flight
- unstable tracking state
- incorrect or delayed detector output
- missing or incorrect plane normals
- failing to observe enough distinct planes for downstream corner inference

If pose or plane quality is poor, Lucas's downstream clustering and inferred-corner logic degrade. The pipeline therefore needs to emit stable frame-aligned measurement objects and make measurement quality explicit.

## Users

- perception engineers building and debugging the live measurement stack
- operators who start and stop the onboard measurement session
- Lucas's clustering pipeline running on the same Jetson

## Goals

- Use the ZED SDK live while the drone is recording.
- Support optional SVO2 recording, controlled by configuration, without making replay the main data path.
- Emit one append-only JSON object per frame for Lucas.
- Include everything Lucas needs from this repo:
  - ZED pose/tracking state
  - per-paper world measurements
  - center-pixel depth
  - multiple frame-level planes when visible
  - per-paper links to the corresponding plane
- Keep the detector outside this repo behind a clean adapter boundary.
- Keep tracking modes modular so `vio`, `vslam_map`, and later `gnss_fusion` can be swapped cleanly.

## Non-Goals For v1

- real GNSS/RTK fusion wiring
- corner detection or corner calculation in this repo
- custom SLAM or custom plane fitting outside the ZED SDK
- ROS integration
- final text-file generation for the client

Lucas owns clustering, inferred corners, and final text output.

## v1 Scope

- Python implementation using `pyzed.sl`
- live ZED capture + live positional tracking
- optional parallel SVO2 recording
- live detector adapter placeholder for external YOLO + Roboflow
- center-pixel paper measurement
- live plane detection at each paper center
- additional live plane queries each frame so non-paper walls can still be represented
- frame-level `planes[]` output plus per-paper `plane_id` association
- append-only per-frame JSONL handoff for Lucas on the same Jetson
- explicit `start_session()` / `stop_session()` control boundary
- deterministic non-hardware tests for non-camera logic

## Success Criteria

- A live session can be started and stopped explicitly.
- The system records SVO2 in parallel only when `recording.enable_svo_recording` is enabled.
- The system appends one valid frame object at a time to the Lucas handoff file during flight.
- Each frame object includes ZED pose/tracking data, zero or more frame-level planes, and zero or more measured papers linked to those planes.
- Lucas can tail the append-only file on the same Jetson without needing replay/finalization first.
- Offline review is possible when SVO recording is enabled.

## Primary Metrics

- `% tracking_state == OK`
- `max_pose_jump_m`
- `p95_pose_jump_m`
- `live_frame_emit_latency_ms`
- `paper_position_mae_m`
- `paper_position_p95_m`
- `plane_success_rate`
- `valid_measurement_rate`

## Implementation Status

This document describes the intended live-first architecture.

The current runtime in the repo is still replay-first, but replay processing/finalization now implements the frame-level plane model:

- top-level `planes[]`
- per-paper `plane_id`
- fixed 3x3 scene probes
- floor-plane probing off by default

The remaining gap is the live runtime path during recording.

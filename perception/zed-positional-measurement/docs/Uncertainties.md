# Uncertainties

## Purpose

This document captures the remaining design questions for the live ZED pipeline so they can be clarified with Matt before the live-first refactor is implemented.

## Already Decided

- The system should use the ZED SDK live while recording.
- SVO recording should be available behind `recording.enable_svo_recording` for audit, debugging, and neural net usage.
- Lucas's code will run on the same Jetson on the drone.
- The external detector is a YOLO + Roboflow process that is not part of this repo.
- This repo should not calculate corners itself.
- This repo should pass plane information and plane normals to Lucas.
- Use the detector center pixel for measurement.
- The live handoff should be append-only.
- The live handoff should be one JSON object per frame.
- Frame-level plane discovery should use:
  - `find_plane_at_hit` at each paper center
  - 9 extra fixed scene probes in a 3x3 normalized image grid
  - simple plane dedupe
  - `include_floor_plane = false` by default

## Open Questions For Matt

### 1. Detector Integration Boundary

Current uncertainty:

- We do not yet know how the external YOLO + Roboflow detector will connect to this repo.

Options to clarify:

- direct Python callback/function call
- local in-memory queue
- local HTTP endpoint
- websocket
- watched file / append-only sidecar

Current implementation assumption for later refactor:

- create a clear placeholder detector adapter boundary and keep the detector out of this repo

### 2. Detector Output Contract

Current uncertainty:

- We are assuming the detector returns all of the following, but this still needs confirmation:
  - frame id or timestamp
  - center pixel
  - paper color/class
  - confidence

Need to confirm:

- exact field names
- whether frame id and timestamp are both available
- whether the detector output is synchronized to the exact ZED frame grab

### 3. Live Handoff File Contract

Current resolved assumption:

- one append-only per-session `frames.jsonl`
- one JSON object per frame

Remaining optional question:

- whether a rolling `latest_frame.json` snapshot is also useful later

### 4. Exact Payload Lucas Needs

Current uncertainty:

- The instruction is that Lucas needs "all the info outlined earlier," but the exact minimum required payload is not locked.

Need to confirm whether Lucas needs:

- ZED pose for every frame
- paper center pixel for every detection
- per-paper depth
- per-paper world point
- per-paper plane normal
- full plane data beyond just normals
- tracking confidence / status
- camera timestamp

Important note:

- Full frame depth should not be passed as JSON unless Matt explicitly wants that. It is too heavy for the live append-only contract.

### 5. Ownership Of Revised World State

Current uncertainty:

- The system can revise estimates over time, but it is not yet locked whether this repo should maintain a live best-estimate world state or whether Lucas should own all revision/clustering state.

Recommended assumption:

- this repo emits raw per-frame observations
- Lucas owns clustering, revision, and final text-file generation

### 6. Start / Stop Control Interface

Current uncertainty:

- The user confirmed a start/stop button model, but the immediate software interface is not defined.

Need to confirm:

- simple service API with `start_session()` / `stop_session()`
- CLI wrapper for testing only
- actual UI/process that will invoke those controls on the Jetson

Recommended assumption:

- implement a service class with explicit `start_session()` and `stop_session()`
- keep a thin CLI wrapper for local testing

## What This Means For The Refactor

The next implementation should move from:

- record first, replay later

to:

- live ZED capture
- live positional tracking
- live per-frame measurement packaging
- append-only frame JSON handoff to Lucas
- parallel SVO recording

The detector boundary and exact minimum payload still need confirmation, but the high-level live contract is now decided.

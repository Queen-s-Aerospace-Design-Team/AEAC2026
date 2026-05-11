# Known issues / things to watch

Tracking items we've intentionally deferred. None are blockers for the first hardware smoke test on the Jetson, but each is worth watching during/after the test.

Updated: 2026-05-11.

## Performance: ZED plane queries per frame

`MeasurementPipeline._frame_record_and_paper_measurements` runs `find_plane_at_hit` once per detected paper plus 9 fixed scene-probe pixels every frame, plus `find_floor_plane` when `plane_detection.include_floor_plane` is true (now the default). At HD1080 + NEURAL depth + 30 fps on Jetson, this may cap effective frame rate.

What to watch on the smoke test:

- frame loop wall time during the live session
- whether grab failures or dropped frames correlate with high plane-query counts

Cut levers if needed:

- shrink the 3x3 probe grid
- run scene probes every Nth frame instead of every frame
- disable floor probing if we're getting horizontals from the wall probes anyway

## Performance: `_mask_pixels` walks every pixel of every detected object in pure Python

In `sdk.ZedSdkAdapter._mask_pixels` we set `enable_segmentation=True` on the ZED object detector, then iterate the resulting per-pixel mask in Python and pack the active pixel coordinates into `PaperObservation.mask_pixels`. That field is never read anywhere downstream — the rest of the pipeline only uses `bbox_xyxy` and the bbox center pixel.

This is a pure-Python hot loop over thousands of pixels per paper per frame, populating a field nothing consumes.

When we get to perf work:

- set `enable_segmentation=False` in `configure_paper_detection`
- delete or stub out `_mask_pixels`
- leave `PaperObservation.mask_pixels = ()` (or remove the field if no future plan needs it)

## Clustering: `min_samples_pt = 5` requires multi-frame paper coverage

`circle_processing.cluster_papers` uses DBSCAN with `min_samples=5`. A paper observed in fewer than 5 frames is discarded as noise.

Why we kept it: averaging across ≥5 observations gives a more robust centroid, especially with noisy depth at range.

Implication: we cannot operate "first detection wins." We need enough frames per paper for the cluster to fire, which biases us toward batch / segment-streamed processing rather than truly per-frame live output.

What to watch:

- per-target frame counts after the smoke test
- targets near the edges of the route that may get short coverage
- whether segment-streamed `cluster_papers` re-runs produce stable centroids by the time the flight ends

## Drift signal: `max_pose_jump_m` is currently informational only

The producer reports `max_pose_jump_m` and `p95_pose_jump_m` (see `metrics.py`) but the pipeline does not gate measurements on either. If tracking briefly degrades mid-flight, samples from that window are still ingested by `circle_processing.cluster_papers` and can pull a centroid.

If we see noisy clusters after the smoke test:

- threshold on `tracking_confidence` per frame in the consumer
- detect frame-to-frame pose jumps inline and drop a window around any large jump
- consider whether to filter producer-side (cleaner data) or consumer-side (Lucas can tune)

## Config: `paper.mode = 'native_yolo'` requires an ONNX path

`pipeline._validate_for_processing` errors at session start if `paper.native_yolo_onnx_path` is unset while `paper.mode == 'native_yolo'` (the default). Two options before the Jetson run:

1. set `paper.native_yolo_onnx_path` in the config to a real ONNX file
2. switch to `paper.mode = 'external_boxes'` and populate `external_detections_dir` with the detector's per-segment JSONL output

## Config: SVO recording is currently required for processing/finalization

`_validate_for_processing` rejects configs that try to process or finalize without `recording.enable_svo_recording = true`. Fine for now (we want the audit SVO anyway), but it means there is no path today to run measurement without keeping the SVO around.

## Ops: Jetson camera daemons

ZED X cameras on Jetson rely on two daemons. See `perception/docs/learnings.md` for the full notes.

- `nvargus-daemon` — NVIDIA's capture service; if it crashes you get Argus timeouts.
- `zed_x_daemon` — StereoLabs' ZED X service; handles HW encoder and sensor sync.

Rules of operation:

- only one process can hold the camera at a time. Always quit ZED Explorer before running our scripts (and vice versa).
- if the camera fails to open: `sudo systemctl restart nvargus-daemon`, retry, then `sudo systemctl restart zed_x_daemon` if still failing.

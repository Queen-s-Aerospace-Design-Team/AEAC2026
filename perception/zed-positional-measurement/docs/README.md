# Docs

This folder captures the intent, architecture, runtime behavior, and data contracts for the intended live-first ZED spatial measurement pipeline.

Status:

- the docs now describe the intended live-first architecture
- the current runtime in the repo is still replay-first
- replay processing/finalization now emits frame-level `planes[]` and `papers[].plane_id`
- live append-only emission during recording still needs the larger runtime refactor

## Files

- `PRD.md`: Product requirements and live-first v1 scope.
- `Technical.md`: Live system architecture, module responsibilities, and frame-processing flow.
- `Operations.md`: Start/stop runtime model, expected outputs, and operator runbook.
- `Schema.md`: Live per-frame handoff schema and session file layout.
- `Validation.md`: Primary metrics and recommended live hardware validation flow.
- `LucasHandoff.md`: Exact live JSONL handoff contract for Lucas's clustering step.
- `Uncertainties.md`: Open design questions to confirm before the live-first refactor.

## Read Order

If someone is new to the repo, the fastest useful order is:

1. `PRD.md`
2. `Technical.md`
3. `Operations.md`
4. `Schema.md`
5. `LucasHandoff.md`
6. `Uncertainties.md`
7. `Validation.md`

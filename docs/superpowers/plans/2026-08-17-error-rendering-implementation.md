# Error Stages and Cloud Rendering Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add non-mutating multi-method matching-error evaluation, persistent ICP-stage metrics, and independent configurable/runtime cloud rendering while preserving current profile colors.

**Architecture:** Keep metric algorithms in `frame_alignment/core/registration.py`, stage snapshots in the controller/window quality state, and serialization in the existing exporter. Add a rendering configuration model/helper consumed by `Scene3DView`; GUI controls override config for the current session only.

**Tech Stack:** Python, NumPy, SciPy cKDTree, Open3D normals/ICP, PySide6, PyQtGraph, PyYAML, unittest.

## Global Constraints

- Default error ROI is `25.0 m`; default match threshold is `0.2 m`.
- Supported error methods are `nearest_neighbor`, `symmetric_chamfer`, and `point_to_plane`.
- Error evaluation must not mutate `PoseModel`.
- The first successful ICP snapshot is retained as `quality.initial_icp`; later manual edits cannot overwrite it.
- `quality.icp_error` is the post-ICP/pre-manual geometric error; `quality.manual_error` is calculated by the new button.
- Global and source clouds have independent `uniform`, `native`, and `cmap` rendering settings.
- Profile colors and legends remain unchanged by 3D rendering settings.
- Do not modify `src/frame_register_manual.py` or add runtime dependencies.

---

### Task 1: Implement pure error metrics

**Files:**
- Modify: `frame_alignment/core/registration.py`
- Test: `tests/test_registration_metrics.py`

**Interfaces:**
- Produce `matching_error(source_points, target_points, method, threshold, radius/normals)` returning `median_m`, `mean_m`, `rmse_m`, `p95_m`, `match_ratio` without mutating inputs.

- [ ] Add failing tests for one-way nearest-neighbor statistics, threshold ratio `0.2`, symmetric method, point-to-plane method, and empty inputs.
- [ ] Run `python -m unittest tests.test_registration_metrics`; confirm the new import/behavior fails.
- [ ] Implement finite `N×3` conversion, nearest-neighbor distances through `cKDTree`, symmetric aggregation, and target-normal point-to-plane distance using Open3D-compatible normals.
- [ ] Run the same test module and confirm it passes.
- [ ] Commit `feat: add matching error metrics`.

### Task 2: Add error controls and ICP stage snapshots

**Files:**
- Modify: `frame_alignment/app/controller.py`, `frame_alignment/ui/main_window.py`, `frame_alignment/io/exporter.py`
- Test: `tests/test_quality_stages.py`, `tests/test_main_window_integration.py`

**Interfaces:**
- Controller quality state contains `initial_icp`, `icp_error`, and `manual_error`.
- Main window exposes a non-mutating `compute_error()` callback and an adjacent `error_button`.

- [ ] Add failing tests proving compute-error leaves the pose transform unchanged, quality stages reset on new frame, first ICP snapshot survives manual mutation, and exported YAML preserves all stages.
- [ ] Run focused tests and confirm failures.
- [ ] Add `error` config parsing with defaults `method=nearest_neighbor`, `evaluation_radius_m=25.0`, `match_threshold_m=0.2`; implement the button and status/error reporting.
- [ ] On first successful ICP, save its native RMSE/fitness and correction magnitudes under `initial_icp`; immediately calculate and save `icp_error`; later manual edits only invalidate transient display state, not snapshots.
- [ ] Export nested stage data under `quality` without changing existing output paths or input fields.
- [ ] Run focused tests and commit `feat: preserve ICP and manual error stages`.

### Task 3: Implement independent cloud color mapping

**Files:**
- Create: `frame_alignment/ui/cloud_rendering.py`
- Modify: `frame_alignment/ui/scene_3d.py`, `frame_alignment/ui/main_window.py`
- Test: `tests/test_cloud_rendering.py`, `tests/test_views.py`

**Interfaces:**
- `render_cloud_colors(points, native_colors, settings, origin)` returns an RGBA array or uniform fallback.
- Settings support `uniform`, `native`, `cmap`; scalar values support `z` and `distance`; range supports `auto` or `[min, max]`.

- [ ] Add failing tests for separate global/source uniform colors, native fallback, deterministic cmap normalization, fixed range, and invalid settings fallback.
- [ ] Run focused tests and confirm failures.
- [ ] Implement colormap conversion using existing PyQtGraph facilities; do not add matplotlib or another dependency.
- [ ] Pass independent color arrays to the two 3D scatter items while leaving profile widgets and profile overlays unchanged.
- [ ] Run focused tests and commit `feat: add configurable cloud rendering`.

### Task 4: Add config defaults and GUI session overrides

**Files:**
- Modify: `config.yaml`, `frame_alignment/ui/main_window.py`, `frame_alignment/ui/data_io_panel.py`
- Test: `tests/test_cloud_rendering_config.py`, `tests/test_main_window_layout.py`

**Interfaces:**
- Config nests `display.clouds.global` and `display.clouds.source` with `mode`, `color`, `cmap`, `scalar`, `range`.
- GUI rendering controls apply only to the current session and never rewrite YAML automatically.

- [ ] Add failing tests for config parsing, default uniform mode, independent selectors, and profile color invariance.
- [ ] Implement the collapsible rendering panel with separate global/source controls and refresh-only color updates.
- [ ] Run focused tests and commit `feat: add rendering config and controls`.

### Task 5: Regression verification and documentation

**Files:**
- Modify: `README.md`, `USER_GUIDE.md`
- Test: existing focused test modules plus new metric/rendering tests

- [ ] Document error defaults/methods/stage fields and independent cloud rendering configuration.
- [ ] Run `python -m unittest` over new tests plus existing CLI, ICP, profile, exporter, and GUI integration tests.
- [ ] Run `src/frame_align_6dof.py --self-test`, compileall, and `git diff --check`.
- [ ] Confirm `src/frame_register_manual.py` is unchanged and commit `docs: document error and rendering controls`.

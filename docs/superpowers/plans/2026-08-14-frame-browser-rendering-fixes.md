# Frame Browser and Rendering Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make high-coordinate point clouds visible in the embedded 3D view, make all four profiles render reliably, and add manual/scanned Frame ID browsing with YAML annotation status in a right-side control column.

**Architecture:** Add a UI-independent `FrameCatalog` for directory scanning, ordering, navigation, and annotation counts. Keep point-cloud loading explicit behind the existing load button. Add explicit camera focus and atomic profile updates, then reorganize `MainWindow` into a left visualization area and a right control sidebar.

**Tech Stack:** Python 3.8, NumPy, PySide6 6.6, pyqtgraph 0.13, PyOpenGL, Open3D, pytest/unittest.

## Global Constraints

- Do not modify `src/frame_register_manual.py`.
- Source PCD points are already in map coordinates and must not be multiplied by `T0`.
- Frame navigation changes the Frame ID only; loading remains explicit.
- Frame discovery is non-recursive and accepts `.pcd` case-insensitively.
- Annotation truth is based only on `<yaml_output_dir>/<frame_id>.yaml` being a file.
- Manual 6DOF changes must not reset the user camera; a successful new-frame load must focus once.
- Keep `.venv`, `config.yaml`, caches, and generated output untracked.

---

### Task 1: Frame Catalog Domain Model

**Files:**
- Create: `frame_alignment/app/frame_catalog.py`
- Modify: `frame_alignment/app/__init__.py`
- Test: `tests/test_frame_catalog.py`

**Interfaces:**
- Produces: `natural_frame_key(value: str) -> tuple`
- Produces: `FrameCatalog.scan(frame_directory) -> tuple[str, ...]`
- Produces: `FrameCatalog.refresh_annotations(yaml_directory) -> frozenset[str]`
- Produces: `FrameCatalog.is_annotated(frame_id: str) -> bool`
- Produces: `FrameCatalog.previous(frame_id: str) -> Optional[str]`
- Produces: `FrameCatalog.next(frame_id: str) -> Optional[str]`
- Produces properties: `frame_ids`, `annotated_count`, `total_count`

- [ ] **Step 1: Write failing catalog tests**

Create tests using temporary directories that assert:

```python
catalog = FrameCatalog()
(root / "10.pcd").touch()
(root / "2.PCD").touch()
(root / "frame9.pcd").touch()
(root / "frame10.pcd").touch()
(root / "ignored.txt").touch()
(root / "nested").mkdir()
(root / "nested" / "1.pcd").touch()

assert catalog.scan(root) == ("2", "10", "frame9", "frame10")
assert catalog.previous("2") == "2"
assert catalog.next("2") == "10"
assert catalog.next("frame10") == "frame10"
```

Add annotation assertions:

```python
(yaml_dir / "2.yaml").touch()
(yaml_dir / "unrelated.yaml").touch()
catalog.refresh_annotations(yaml_dir)
assert catalog.is_annotated("2")
assert not catalog.is_annotated("10")
assert catalog.annotated_count == 1
assert catalog.total_count == 4
```

Assert an invalid directory raises `NotADirectoryError` without replacing a prior successful `frame_ids` value. Assert duplicate stems differing only by case appear once.

- [ ] **Step 2: Run the catalog tests and verify RED**

Run:

```powershell
D:\0_code\frame_register_manual\.venv\Scripts\python.exe -m pytest tests\test_frame_catalog.py -q
```

Expected: collection or import failure because `frame_alignment.app.frame_catalog` does not exist.

- [ ] **Step 3: Implement the minimal catalog**

Use a natural sort key based on digit/non-digit tokens:

```python
def natural_frame_key(value):
    return tuple(
        (0, int(token)) if token.isdigit() else (1, token.casefold())
        for token in re.split(r"(\d+)", value)
        if token
    )
```

In `scan`, resolve and validate the directory, iterate only `directory.iterdir()`, accept files whose `suffix.casefold() == ".pcd"`, deduplicate with `stem.casefold()`, sort, and assign `_frame_ids` only after scanning succeeds. `previous` and `next` return the boundary item at the ends and return `None` for an empty catalog or an ID not in the catalog.

In `refresh_annotations`, treat a missing/invalid/empty YAML directory as an empty annotation set. For a valid directory, compute annotated IDs by checking only `yaml_dir / (frame_id + ".yaml")` for each scanned ID.

- [ ] **Step 4: Run catalog tests and full domain regression**

Run:

```powershell
D:\0_code\frame_register_manual\.venv\Scripts\python.exe -m pytest tests\test_frame_catalog.py tests\test_frame_requests.py tests\test_frame_loader.py -q
```

Expected: all pass.

- [ ] **Step 5: Commit Task 1**

```powershell
git add frame_alignment/app/frame_catalog.py frame_alignment/app/__init__.py tests/test_frame_catalog.py
git commit -m "feat: add frame catalog and annotation index"
```

---

### Task 2: Editable Frame Browser and Annotation Status UI

**Files:**
- Modify: `frame_alignment/ui/data_io_panel.py`
- Modify: `tests/test_data_io_panel.py`
- Create: `tests/test_frame_browser_panel.py`

**Interfaces:**
- Consumes: `FrameCatalog`
- Preserves: `DataIOPanel.frame_id_edit` as the editable combo box line edit
- Produces: `DataIOPanel.scan_frames() -> bool`
- Produces: `DataIOPanel.refresh_annotation_status() -> None`
- Produces signal: `scan_failed = Signal(str)`
- Produces controls: `frame_combo`, `previous_button`, `next_button`, `scan_button`, `annotation_status_label`, `annotation_count_label`

- [ ] **Step 1: Write failing panel tests**

Create a real temporary frame directory and YAML directory. Assert:

```python
panel = DataIOPanel()
panel.frame_dir_edit.setText(str(frame_dir))
assert panel.scan_frames()
assert [panel.frame_combo.itemText(i) for i in range(panel.frame_combo.count())] == ["2", "10"]

panel.frame_combo.setCurrentText("2")
panel.yaml_dir_edit.setText(str(yaml_dir))
panel.refresh_annotation_status()
assert panel.annotation_status_label.text() == "已标注"
assert panel.annotation_count_label.text() == "1/2（已标注/总量）"
```

Connect `load_requested` to a counter, click previous/next, change the combo selection, process Qt events, and assert the counter remains zero. Click `load_button` and assert it becomes one. Assert manual text such as `manual-99` remains available and `get_load_request().frame_id == "manual-99"`. Also call `apply_config({"frame_cloud_map_path": str(frame_dir)})` on a fresh panel and assert the valid configured directory is scanned at startup.

For an invalid new directory, seed a valid list first, call `scan_frames`, assert it returns `False`, emits the path-bearing error, and preserves the prior combo items.

- [ ] **Step 2: Run panel tests and verify RED**

Run:

```powershell
$env:QT_QPA_PLATFORM='offscreen'
D:\0_code\frame_register_manual\.venv\Scripts\python.exe -m pytest tests\test_frame_browser_panel.py tests\test_data_io_panel.py -q
```

Expected: failures for missing combo, navigation, scan, and annotation controls.

- [ ] **Step 3: Implement the frame browser controls**

Construct `self.catalog = FrameCatalog()`, an editable `QComboBox`, and set:

```python
self.frame_combo.setEditable(True)
self.frame_id_edit = self.frame_combo.lineEdit()
```

Add previous, next, refresh, status, and count widgets to the existing collapsible panel. `scan_frames` must save the current text, call the catalog, update combo items with signals blocked, restore a matching/manual value, refresh annotations, and return `True`. Catch exceptions, emit `scan_failed(str(exc))`, leave old items unchanged, and return `False`.

When the frame directory is selected through the browse dialog, call `scan_frames`. At the end of `apply_config`, call `scan_frames` only when the configured frame path is an existing directory; legacy configurations containing a source file must not emit a scan error. Manual text edits only call `refresh_annotation_status`; they must not scan or load. YAML directory text changes call `refresh_annotation_status`. Navigation buttons call catalog boundary methods and update combo text only.

- [ ] **Step 4: Run panel tests and UI regression**

Run:

```powershell
$env:QT_QPA_PLATFORM='offscreen'
D:\0_code\frame_register_manual\.venv\Scripts\python.exe -m pytest tests\test_frame_browser_panel.py tests\test_data_io_panel.py tests\test_ui_review_fixes.py -q
```

Expected: all pass.

- [ ] **Step 5: Commit Task 2**

```powershell
git add frame_alignment/ui/data_io_panel.py tests/test_data_io_panel.py tests/test_frame_browser_panel.py
git commit -m "feat: add manual and scanned frame browsing"
```

---

### Task 3: Focus the 3D Camera on High-Coordinate Frames

**Files:**
- Modify: `frame_alignment/ui/scene_3d.py`
- Modify: `frame_alignment/ui/main_window.py`
- Modify: `tests/test_views.py`
- Modify: `tests/test_main_window_integration.py`

**Interfaces:**
- Produces: `Scene3DView.focus_on(center, roi_radius) -> None`
- Consumes in `MainWindow._on_frame_loaded`: `self.scene.focus_on(model.c0, self.roi_radius)`

- [ ] **Step 1: Write failing camera-focus tests**

In the scene test, instantiate a real `Scene3DView`, call:

```python
center = np.array([8628.86, 9650.01, 106.995])
view.focus_on(center, 35.0)
actual = view.opts["center"]
np.testing.assert_allclose([actual.x(), actual.y(), actual.z()], center)
assert view.opts["distance"] >= 70.0
```

Extend `FakeScene` with `focus_calls`. Load a frame, assert one focus call with `C0`, then nudge/undo/change slice settings and assert the count remains one. Load a second successful frame and assert it becomes two.

- [ ] **Step 2: Run focus tests and verify RED**

Run:

```powershell
$env:QT_QPA_PLATFORM='offscreen'
D:\0_code\frame_register_manual\.venv\Scripts\python.exe -m pytest tests\test_views.py tests\test_main_window_integration.py -q
```

Expected: failures because `focus_on` and fake focus tracking do not exist.

- [ ] **Step 3: Implement explicit focus**

Use `PySide6.QtGui.QVector3D` and set the camera only when the center is a finite `(3,)` vector and radius is positive:

```python
self.setCameraPosition(
    pos=QVector3D(float(center[0]), float(center[1]), float(center[2])),
    distance=max(5.0, float(roi_radius) * 2.0),
)
```

Do not call focus from `set_reference`, `set_adjusted`, `update_origin`, or any manual refresh path. Call it once in `_on_frame_loaded` immediately after setting Reference.

- [ ] **Step 4: Run camera tests and integration regression**

Run:

```powershell
$env:QT_QPA_PLATFORM='offscreen'
D:\0_code\frame_register_manual\.venv\Scripts\python.exe -m pytest tests\test_views.py tests\test_main_window_integration.py tests\test_end_to_end_workflow.py -q
```

Expected: all pass.

- [ ] **Step 5: Commit Task 3**

```powershell
git add frame_alignment/ui/scene_3d.py frame_alignment/ui/main_window.py tests/test_views.py tests/test_main_window_integration.py
git commit -m "fix: focus 3d camera on loaded lidar origin"
```

---

### Task 4: Atomic and Explicit Profile Rendering

**Files:**
- Modify: `frame_alignment/ui/profile_view.py`
- Modify: `frame_alignment/ui/main_window.py`
- Modify: `tests/test_views.py`
- Create: `tests/test_profile_rendering.py`

**Interfaces:**
- Produces: `ProfileView.set_profile_data(reference_points, adjusted_points, half_length) -> None`
- Preserves: independent `reference_item`, `adjusted_item`, and stable two-entry legend

- [ ] **Step 1: Write failing profile rendering tests**

Test finite filtering and union range:

```python
view.set_profile_data(
    np.array([[-2.0, -1.0], [0.0, np.nan], [2.0, 4.0]]),
    np.array([[-1.0, -3.0], [1.0, np.inf]]),
    20.0,
)
assert len(view.reference_item.data["x"]) == 2
assert len(view.adjusted_item.data["x"]) == 1
assert view.plotItem.viewRange()[0] == [-20.0, 20.0]
assert view.plotItem.viewRange()[1][0] < -3.0
assert view.plotItem.viewRange()[1][1] > 4.0
assert not view.empty_item.isVisible()
```

Then pass two empty arrays and assert both scatter items are empty and `empty_item.isVisible()` is true. Pass data again and assert it becomes false. Assert legend item count remains exactly two through all updates.

- [ ] **Step 2: Run profile tests and verify RED**

Run:

```powershell
$env:QT_QPA_PLATFORM='offscreen'
D:\0_code\frame_register_manual\.venv\Scripts\python.exe -m pytest tests\test_profile_rendering.py tests\test_views.py -q
```

Expected: failures for missing `set_profile_data` and `empty_item`.

- [ ] **Step 3: Implement one-pass profile refresh**

Add a finite-point helper that returns shape `(N, 2)`. Add one non-legend `TextItem` with text `当前剖面无点`, anchor `(0.5, 0.5)`, position `(0, 0)`, and hidden initial state.

`set_profile_data` must update both scatter items, set X range without padding, combine finite Y values, and set Y range with `max(0.05, span * 0.05)` padding. For a constant Y value, use a minimum half-range of `0.05`. If both sets are empty, set Y range to `[-1, 1]` and show the empty item.

Change `MainWindow.refresh_views` to calculate both slices first and call `profile.set_profile_data(reference, adjusted, self.slice_half_length)` once per direction.

- [ ] **Step 4: Run profile and main-window regression**

Run:

```powershell
$env:QT_QPA_PLATFORM='offscreen'
D:\0_code\frame_register_manual\.venv\Scripts\python.exe -m pytest tests\test_profile_rendering.py tests\test_views.py tests\test_main_window_integration.py -q
```

Expected: all pass and no pyqtgraph warnings.

- [ ] **Step 5: Commit Task 4**

```powershell
git add frame_alignment/ui/profile_view.py frame_alignment/ui/main_window.py tests/test_views.py tests/test_profile_rendering.py tests/test_main_window_integration.py
git commit -m "fix: render all profiles with explicit finite ranges"
```

---

### Task 5: Right-Side Control Layout and Export Status Refresh

**Files:**
- Modify: `frame_alignment/ui/main_window.py`
- Modify: `frame_alignment/ui/data_io_panel.py`
- Modify: `tests/test_main_window_integration.py`
- Create: `tests/test_main_window_layout.py`

**Interfaces:**
- Produces attributes: `visualization_widget`, `right_sidebar`, `pose_panel`, `matrix_panel`
- Consumes: `DataIOPanel.refresh_annotation_status()` after successful YAML export

- [ ] **Step 1: Write failing layout and export-refresh tests**

Show a `MainWindow` offscreen at `1600x980`, process events, and assert:

```python
scene_pos = window.scene.mapTo(window, QPoint(0, 0))
profile_pos = window.profiles[0].mapTo(window, QPoint(0, 0))
sidebar_pos = window.right_sidebar.mapTo(window, QPoint(0, 0))
pose_pos = window.pose_panel.mapTo(window, QPoint(0, 0))
matrix_pos = window.matrix_panel.mapTo(window, QPoint(0, 0))
data_pos = window.data_panel.mapTo(window, QPoint(0, 0))
assert scene_pos.x() < sidebar_pos.x()
assert profile_pos.x() < sidebar_pos.x()
assert pose_pos.y() < matrix_pos.y() < data_pos.y()
```

For annotation refresh, load a temporary frame, scan its directory, set a valid YAML output directory, export YAML, and assert the label changes from `未标注` to `已标注` and the count becomes `1/1（已标注/总量）`.

- [ ] **Step 2: Run layout tests and verify RED**

Run:

```powershell
$env:QT_QPA_PLATFORM='offscreen'
D:\0_code\frame_register_manual\.venv\Scripts\python.exe -m pytest tests\test_main_window_layout.py tests\test_main_window_integration.py -q
```

Expected: missing layout attributes/order and stale annotation status after export.

- [ ] **Step 3: Rebuild the main layout without changing behavior**

Create a horizontal root layout. Build `visualization_widget` with a grid containing Scene3DView across the top and the four ProfileViews in two rows below. Build `right_sidebar` with a vertical layout containing:

1. `pose_panel` from `_build_pose_panel`, without `matrix_text`;
2. `matrix_panel` from a new `_build_matrix_panel` containing `matrix_text`;
3. the existing collapsible `data_panel`;
4. a final stretch.

Place the right sidebar inside a widget-resizable `QScrollArea`, set a practical minimum width around 420 pixels, and give the left visualization area the stretch priority. Keep all existing signals, shortcuts, controller behavior, and widget attributes.

After `export_result` returns an outcome whose YAML exists, call `self.data_panel.refresh_annotation_status()` before showing the success message. PCD failure must not prevent YAML-derived status refresh.

- [ ] **Step 4: Run layout, export, and UI regression**

Run:

```powershell
$env:QT_QPA_PLATFORM='offscreen'
D:\0_code\frame_register_manual\.venv\Scripts\python.exe -m pytest tests\test_main_window_layout.py tests\test_main_window_integration.py tests\test_end_to_end_workflow.py tests\test_export_colors.py -q
```

Expected: all pass.

- [ ] **Step 5: Commit Task 5**

```powershell
git add frame_alignment/ui/main_window.py frame_alignment/ui/data_io_panel.py tests/test_main_window_layout.py tests/test_main_window_integration.py
git commit -m "feat: move data and pose controls into right sidebar"
```

---

### Task 6: Real-Sample Regression and Documentation

**Files:**
- Create: `tests/test_real_sample_rendering.py`
- Modify: `README.md`
- Modify: `config.example.yaml`

**Interfaces:**
- Consumes real sample paths only when both files exist
- Verifies `read_cloud`, ROI, `extract_slice`, `Scene3DView.focus_on`, and FrameCatalog against production data

- [ ] **Step 1: Write the real-sample test**

Use module-level constants for the two supplied sample paths and `unittest.skipUnless` if either is absent. Load the map and source once for the test class. Construct the supplied 4x4 pose and assert:

```python
assert len(global_map.points) > 6_000_000
assert len(source.points) > 90_000
assert len(global_map.roi(c0, 35.0).points) > 800_000
for angle in (0.0, 90.0, 45.0, -45.0):
    assert len(extract_slice(map_display.points, c0, angle, 20.0, 0.2)) > 0
    assert len(extract_slice(source_display.points, c0, angle, 20.0, 0.2)) > 0
```

For X-Z specifically, assert Reference and Adjusted counts are nonzero and pass the arrays into `ProfileView.set_profile_data`; assert both scatter items contain points. Call `Scene3DView.focus_on(c0, 35.0)` and assert its camera center equals `c0`.

- [ ] **Step 2: Run the real-sample acceptance test**

Run:

```powershell
$env:QT_QPA_PLATFORM='offscreen'
D:\0_code\frame_register_manual\.venv\Scripts\python.exe -m pytest tests\test_real_sample_rendering.py -q
```

Expected: pass. This task adds environment-backed acceptance coverage after the camera and profile behaviors have already completed their RED/GREEN cycles in Tasks 3 and 4; it does not introduce new production behavior.

- [ ] **Step 3: Update user documentation and example configuration**

Document the right-side frame browser, explicit load behavior, status meaning, annotation count, refresh button, and camera behavior. Keep example directories distinct:

```yaml
frame_cloud_map_path: ./data/frames
initial_pose_path: ./data/poses
output_path_yaml: ./alignment_output/yaml
output_path_pcd: ./alignment_output/pcd
```

State that these output directories must exist before export.

- [ ] **Step 4: Run complete verification**

Run:

```powershell
$env:QT_QPA_PLATFORM='offscreen'
D:\0_code\frame_register_manual\.venv\Scripts\python.exe -m pytest -q
D:\0_code\frame_register_manual\.venv\Scripts\python.exe .\src\frame_align_6dof.py --self-test
D:\0_code\frame_register_manual\.venv\Scripts\python.exe -m compileall -q frame_alignment src\frame_align_6dof.py tests
git diff --check
git status --short
git diff -- src\frame_register_manual.py
```

Expected: all tests pass; self-test prints `Self-test passed`; compile and diff checks produce no errors; the original script diff is empty.

- [ ] **Step 5: Commit Task 6**

```powershell
git add tests/test_real_sample_rendering.py README.md config.example.yaml
git commit -m "test: verify rendering with real high-coordinate sample"
```

---

### Task 7: Final Code Review and Branch Verification

**Files:**
- Review all files changed since commit `7b2b25dd`

**Interfaces:**
- Consumes: approved design at `docs/superpowers/specs/2026-08-14-frame-browser-rendering-fixes-design.md`
- Produces: a reviewed, green feature branch with no Critical or Important findings

- [ ] **Step 1: Request a read-only code review**

Give the reviewer the approved design, the commit range beginning at `7b2b25dd`, and require checks for camera focus frequency, no automatic frame load, catalog state safety, annotation truth, right-side layout, stable legends, finite profile ranges, real-sample coverage, and repository hygiene.

- [ ] **Step 2: Resolve findings one at a time with TDD**

For every Critical or Important finding, add a focused failing test, run it to confirm the reported defect, implement only the root-cause fix, and rerun the focused plus full suites. Record reasoned pushback when a finding conflicts with the approved design.

- [ ] **Step 3: Run final verification on committed HEAD**

Run:

```powershell
$env:QT_QPA_PLATFORM='offscreen'
D:\0_code\frame_register_manual\.venv\Scripts\python.exe -m pytest -q
D:\0_code\frame_register_manual\.venv\Scripts\python.exe .\src\frame_align_6dof.py --self-test
git diff --check HEAD~1..HEAD
git status --short
```

Expected: all checks pass and the worktree is clean.

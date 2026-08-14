# Dynamic LiDAR Yaw Profiles Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make all profiles follow the corrected LiDAR yaw while retaining map Z as height, and support four to six session-only editable profiles in adaptive 2×2/2×3 layouts.

**Architecture:** Move profile definitions and yaw-relative geometry into `core/profiles.py`, so 2D extraction and 3D overlays consume the same computed basis. Add a focused profile-management widget that owns session-only definitions and emits complete immutable snapshots; `MainWindow` creates/removes profile widgets, rebuilds the grid, and refreshes the scene from those snapshots.

**Tech Stack:** Python 3.8, NumPy, SciPy rotation matrices already produced by `PoseModel`, PySide6, pyqtgraph/OpenGL, unittest/pytest-compatible tests.

## Global Constraints

- Work only in `D:/w/fr` on `feature/new-frame-features`.
- Do not modify `src/frame_register_manual.py`.
- Profile heading follows corrected LiDAR yaw only; height remains fixed to map Z and does not follow pitch/roll.
- Default profiles are XZ `0°`, YZ `90°`, diagonal `+45°`, and diagonal `-45°` in logical cells `(0,0)`, `(1,0)`, `(0,1)`, `(1,1)`.
- XZ and YZ cannot be edited or deleted; default diagonals can be edited but not deleted.
- At most two extra profiles exist, initially `+90°` at `(0,2)` and `-90°` at `(1,2)`; only extras can be deleted.
- Four profiles use 2×2; five or six use 2×3. No profile state persists after the GUI closes.
- Angle mode accepts `-180°..180°`; parallel mode references XZ or YZ with signed finite offset.
- Profile half-length defaults to `20.0m` and is constrained to inclusive `10.0m..35.0m`.
- Do not change ICP, pose, YAML, PCD, or registration coordinate semantics.

---

### Task 1: Yaw-relative profile model and extraction

**Files:**
- Modify: `frame_alignment/core/profiles.py`
- Create: `tests/test_dynamic_profile_geometry.py`

**Interfaces:**
- Produces: `ProfileSpec`, `ProfileGeometry`, `default_profile_specs()`, `extra_profile_spec(slot)`, `profile_geometry(spec, origin, corrected_rotation)`, and `extract_slice(points, geometry, half_length, thickness)`.
- `ProfileSpec` is immutable and updated with `dataclasses.replace`; it contains `profile_id`, `name`, `color`, `grid_row`, `grid_column`, `mode`, `angle_deg`, `reference`, `offset_m`, `editable`, and `deletable`.
- `ProfileGeometry` contains the resolved `center`, `along_axis`, `across_axis`, and fixed `height_axis` plus display identity fields.

- [ ] **Step 1: Write failing default-definition and yaw-basis tests**

```python
def test_default_specs_and_extra_slots_are_stable():
    specs = default_profile_specs()
    assert [(s.grid_row, s.grid_column, s.angle_deg) for s in specs] == [
        (0, 0, 0.0), (1, 0, 90.0), (0, 1, 45.0), (1, 1, -45.0)]
    assert [s.deletable for s in specs] == [False, False, False, False]
    assert (extra_profile_spec(0).grid_row, extra_profile_spec(0).grid_column) == (0, 2)
    assert (extra_profile_spec(1).grid_row, extra_profile_spec(1).grid_column) == (1, 2)

def test_geometry_follows_yaw_but_keeps_map_z():
    rotation = Rotation.from_euler("ZYX", [30.0, 20.0, 15.0], degrees=True).as_matrix()
    geometry = profile_geometry(default_profile_specs()[0], np.zeros(3), rotation)
    np.testing.assert_allclose(geometry.along_axis, [np.cos(np.deg2rad(30)), np.sin(np.deg2rad(30)), 0])
    np.testing.assert_allclose(geometry.height_axis, [0, 0, 1])
```

- [ ] **Step 2: Run the geometry tests and verify RED**

Run:

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_dynamic_profile_geometry -v
```

Expected: FAIL because the new types and factories do not exist.

- [ ] **Step 3: Implement immutable profile definitions and resolved geometry**

Implement the effective angle and center rules:

```python
heading = np.arctan2(rotation[1, 0], rotation[0, 0])
x_heading = np.array((np.cos(heading), np.sin(heading), 0.0))
y_heading = np.array((-np.sin(heading), np.cos(heading), 0.0))
theta = np.deg2rad(spec.angle_deg if spec.mode == "angle" else (0.0 if spec.reference == "XZ" else 90.0))
along = np.cos(theta) * x_heading + np.sin(theta) * y_heading
across = -np.sin(theta) * x_heading + np.cos(theta) * y_heading
center = origin if spec.mode == "angle" else origin + spec.offset_m * across
height = np.array((0.0, 0.0, 1.0))
```

Validate shapes, finite inputs, mode/reference values, angle range, and extra slot `0/1`.

- [ ] **Step 4: Write failing projection and signed-offset tests**

```python
def test_parallel_xz_positive_offset_moves_along_heading_y():
    spec = replace(default_profile_specs()[2], mode="parallel", reference="XZ", offset_m=2.0)
    geometry = profile_geometry(spec, np.array([10.0, 20.0, 3.0]), np.eye(3))
    np.testing.assert_allclose(geometry.center, [10.0, 22.0, 3.0])

def test_extract_slice_projects_along_and_map_height():
    geometry = profile_geometry(default_profile_specs()[0], np.zeros(3), np.eye(3))
    result = extract_slice(np.array([[2.0, 0.05, 3.0], [2.0, 0.2, 4.0]]), geometry, 10.0, 0.2)
    np.testing.assert_allclose(result, [[2.0, 3.0]])
```

- [ ] **Step 5: Implement axis-based extraction and run GREEN**

Run the Task 1 test module and `tests.test_real_sample_rendering`; update real-sample calls to resolve default geometries before extraction.

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_dynamic_profile_geometry tests.test_real_sample_rendering -v
```

Expected: PASS, with real-data tests skipped only when their documented sample paths are absent.

- [ ] **Step 6: Commit Task 1**

```powershell
git add frame_alignment/core/profiles.py tests/test_dynamic_profile_geometry.py tests/test_real_sample_rendering.py
git commit -m "feat: add lidar-yaw profile geometry"
```

---

### Task 2: Dynamic 3D profile overlays

**Files:**
- Modify: `frame_alignment/ui/scene_3d.py`
- Modify: `tests/test_views.py`

**Interfaces:**
- Consumes: `ProfileGeometry` snapshots from Task 1.
- Produces: `slice_rectangle_vertices(geometry, half_length, vertical_half_length=None)` and `Scene3DView.update_slice_overlays(geometries, half_length)`.
- Overlay dictionaries are keyed by `profile_id`, not angle, so duplicate angles and offset parallel profiles remain distinct.

- [ ] **Step 1: Replace the static-overlay tests with failing geometry-driven tests**

```python
def test_vertical_overlay_uses_profile_axis_and_map_z():
    geometry = profile_geometry(default_profile_specs()[0], np.array([10.0, 20.0, 3.0]),
                                Rotation.from_euler("z", 90, degrees=True).as_matrix())
    vertices = slice_rectangle_vertices(geometry, half_length=2.0, vertical_half_length=1.0)
    np.testing.assert_allclose(vertices[0], [10.0, 18.0, 2.0], atol=1e-12)
    np.testing.assert_allclose(vertices[2], [10.0, 22.0, 4.0], atol=1e-12)
```

Add a `Scene3DView` test that calls `update_slice_overlays` with four, six, then four geometries and asserts overlay/label keys match the active profile IDs.

- [ ] **Step 2: Run the view tests and verify RED**

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_views -v
```

Expected: FAIL because the scene still owns four static angle-keyed overlays.

- [ ] **Step 3: Implement geometry-driven overlay lifecycle**

Remove `SliceSpec` ownership from `scene_3d.py`. Add missing line/label items on demand, update active items, and call `removeItem()` for IDs absent from the new snapshot. Position labels at:

```python
label_position = geometry.center + geometry.along_axis * half_length + geometry.height_axis * 0.25
```

Use a vertical rectangle spanning `along_axis` and fixed map-Z `height_axis`; default its vertical half extent to the profile half-length.

- [ ] **Step 4: Run the focused view tests and verify GREEN**

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_views -v
```

Expected: PASS.

- [ ] **Step 5: Commit Task 2**

```powershell
git add frame_alignment/ui/scene_3d.py tests/test_views.py
git commit -m "feat: render dynamic yaw profile overlays"
```

---

### Task 3: Session-only profile management controls

**Files:**
- Create: `frame_alignment/ui/profile_controls.py`
- Create: `tests/test_profile_controls.py`

**Interfaces:**
- Consumes: Task 1 `ProfileSpec`, `default_profile_specs()`, and `extra_profile_spec(slot)`.
- Produces: `ProfileControls(QtWidgets.QGroupBox)` with `profiles_changed = Signal(object)`, `profile_specs` tuple property, `add_profile()`, `delete_selected_profile()`, and `select_profile(profile_id)`.
- Every accepted edit emits the entire immutable tuple in logical grid order.

- [ ] **Step 1: Write failing default-state, editability, and add/delete tests**

```python
def test_controls_start_with_four_defaults_and_limit_extras():
    controls = ProfileControls()
    assert len(controls.profile_specs) == 4
    controls.add_profile()
    controls.add_profile()
    controls.add_profile()
    assert [(s.grid_row, s.grid_column) for s in controls.profile_specs[-2:]] == [(0, 2), (1, 2)]
    assert not controls.add_button.isEnabled()

def test_only_extra_profiles_can_be_deleted():
    controls = ProfileControls()
    controls.select_profile("diag_plus")
    assert not controls.delete_button.isEnabled()
    controls.add_profile()
    controls.select_profile("extra_1")
    assert controls.delete_button.isEnabled()
    controls.delete_selected_profile()
    assert len(controls.profile_specs) == 4
```

- [ ] **Step 2: Run tests and verify RED**

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_profile_controls -v
```

Expected: FAIL because the control module does not exist.

- [ ] **Step 3: Implement the control panel and signal-safe editors**

The editor contains a profile selector, mode selector, angle spin box `[-180, 180]`, reference selector `{XZ,YZ}`, signed finite offset spin box, add button, and delete button. Block signals while switching selection. XZ/YZ disable all position editors; default diagonals enable editors but not deletion.

- [ ] **Step 4: Add failing mode and parameter tests, then implement them**

```python
def test_diagonal_can_switch_to_parallel_signed_offset():
    controls = ProfileControls()
    controls.select_profile("diag_plus")
    controls.mode_combo.setCurrentData("parallel")
    controls.reference_combo.setCurrentText("YZ")
    controls.offset_edit.setValue(-2.5)
    spec = next(s for s in controls.profile_specs if s.profile_id == "diag_plus")
    assert (spec.mode, spec.reference, spec.offset_m) == ("parallel", "YZ", -2.5)
```

Run:

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_profile_controls -v
```

Expected: PASS.

- [ ] **Step 5: Commit Task 3**

```powershell
git add frame_alignment/ui/profile_controls.py tests/test_profile_controls.py
git commit -m "feat: add session profile controls"
```

---

### Task 4: Main-window integration, adaptive layout, and half-length bounds

**Files:**
- Modify: `frame_alignment/ui/main_window.py`
- Modify: `frame_alignment/ui/profile_view.py`
- Modify: `tests/test_main_window_integration.py`
- Modify: `tests/test_main_window_layout.py`
- Create: `tests/test_dynamic_profile_layout.py`

**Interfaces:**
- Consumes: Task 1 geometry functions, Task 2 scene API, and Task 3 profile snapshots.
- Produces: `MainWindow.profile_controls`, a compatibility-preserving ordered `MainWindow.profiles` list, `_apply_profile_specs(specs)`, and `_rebuild_profile_layout()`.
- `ProfileView.set_title(title)` updates an existing plot title without recreating legend items.

- [ ] **Step 1: Write failing default-layout and half-length tests**

```python
def test_default_layout_and_half_length_bounds(qapp):
    window = MainWindow(scene=FakeScene(), profile_factory=FakeProfile,
                        message_sink=lambda level, text: None)
    assert len(window.profiles) == 4
    assert window.length_edit.minimum() == 10.0
    assert window.length_edit.maximum() == 35.0
    assert window.length_edit.value() == 20.0
    assert window.profile_grid_positions() == {
        "xz": (0, 0), "yz": (1, 0), "diag_plus": (0, 1), "diag_minus": (1, 1)}
```

Use a local `QApplication` helper instead of requiring a pytest fixture so the test is runnable with unittest.

- [ ] **Step 2: Run integration/layout tests and verify RED**

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_dynamic_profile_layout tests.test_main_window_integration -v
```

Expected: FAIL because the control panel and adaptive layout are not integrated.

- [ ] **Step 3: Integrate profiles and rebuild the grid**

Store the visualization layout on `self.visualization_layout`. On every snapshot:

1. Reuse widgets for unchanged IDs.
2. Create widgets for new IDs and call `set_title()` for changed names.
3. Remove and delete widgets for deleted extra IDs.
4. Re-add the scene spanning 2 or 3 columns.
5. Place widgets at logical rows plus one scene row.
6. Keep `self.profiles` ordered by `(grid_column, grid_row)` compatibility only where existing tests require iteration.

- [ ] **Step 4: Integrate geometry refresh and update fakes**

Resolve geometries once per refresh:

```python
geometries = [profile_geometry(spec, model.current_origin, model.corrected_pose[:3, :3])
              for spec in self.profile_specs]
self.scene.update_slice_overlays(geometries, self.slice_half_length)
for profile, geometry in zip(self.profiles, geometries):
    reference = extract_slice(self.map_display.points, geometry, self.slice_half_length, self.slice_thickness)
    adjusted_profile = extract_slice(adjusted, geometry, self.slice_half_length, self.slice_thickness)
    profile.set_profile_data(reference, adjusted_profile, self.slice_half_length)
```

Update `FakeScene` and related integration assertions to capture geometry snapshots.

- [ ] **Step 5: Implement half-length initialization and bounds**

Normalize configuration before building widgets:

```python
requested = float(display.get("slice_half_length_m", 20.0))
self.slice_half_length = requested if np.isfinite(requested) and 10.0 <= requested <= 35.0 else 20.0
self.length_edit.setRange(10.0, 35.0)
```

The spin box enforces user bounds; changing it refreshes all 2D profiles and the 3D overlays without reloading or refocusing.

- [ ] **Step 6: Write and pass 5/6/delete/session-reset tests**

```python
def test_extra_profiles_expand_to_three_columns_and_delete_back_to_two():
    window = make_window()
    window.profile_controls.add_profile()
    assert window.profile_grid_positions()["extra_1"] == (0, 2)
    assert window.profile_column_count == 3
    window.profile_controls.add_profile()
    assert window.profile_grid_positions()["extra_2"] == (1, 2)
    window.profile_controls.select_profile("extra_1")
    window.profile_controls.delete_selected_profile()
    window.profile_controls.select_profile("extra_2")
    window.profile_controls.delete_selected_profile()
    assert window.profile_column_count == 2
```

Create a second `MainWindow` and assert it starts with exactly the four defaults even after the first window was edited.

- [ ] **Step 7: Run all directly related UI/profile tests**

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_dynamic_profile_geometry tests.test_views tests.test_profile_controls tests.test_dynamic_profile_layout tests.test_main_window_integration tests.test_profile_rendering -v
```

Expected: PASS.

- [ ] **Step 8: Commit Task 4**

```powershell
git add frame_alignment/ui/main_window.py frame_alignment/ui/profile_view.py tests/test_main_window_integration.py tests/test_main_window_layout.py tests/test_dynamic_profile_layout.py
git commit -m "feat: integrate adaptive profile layouts"
```

---

### Task 5: Focused regression and user documentation

**Files:**
- Modify: `README.md`
- Modify if present: `USER_GUIDE.md`
- Verify only: `src/frame_register_manual.py`

**Interfaces:**
- Documents the GUI-visible profile behavior implemented by Tasks 1-4.
- No production interfaces are introduced.

- [ ] **Step 1: Update documented behavior**

Document that XZ/YZ and diagonal profiles follow corrected LiDAR yaw while retaining map Z, that two session-only profiles may be added/removed, and that profile half-length is `10~35m` with a `20m` default.

- [ ] **Step 2: Run focused regression and static checks**

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_dynamic_profile_geometry tests.test_views tests.test_profile_controls tests.test_dynamic_profile_layout tests.test_main_window_integration tests.test_profile_rendering tests.test_core_behavior -v
.\.venv\Scripts\python.exe -m compileall -q frame_alignment src\frame_align_6dof.py tests
.\.venv\Scripts\python.exe src\frame_align_6dof.py --self-test
git diff --check
git diff --exit-code 9362f69a -- src\frame_register_manual.py
```

Expected: focused tests PASS, compileall is silent with exit code 0, self-test prints `Self-test passed`, diff check is clean, and the preserved script has no diff from the requested baseline.

- [ ] **Step 3: Commit Task 5**

```powershell
git add README.md USER_GUIDE.md
git commit -m "docs: explain dynamic lidar yaw profiles"
```

If `USER_GUIDE.md` is absent in this worktree, omit it from `git add` and record that fact in the final report.

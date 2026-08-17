# Map Anchor Compatibility Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Correct calib pose translations into the already-shifted PCD coordinate range by consuming sibling `map_anchor.yaml` metadata.

**Architecture:** Add one focused I/O helper that returns a corrected pose copy, then call it from directory-based `FrameLoader` and legacy file startup. Point clouds, rotations, pose adjustment math, profiles, ICP, and exports remain unchanged.

**Tech Stack:** Python 3.8, NumPy, PyYAML, unittest, existing Open3D point-cloud adapter.

## Global Constraints

- Work only on `feature/new-frame-features` in `D:/w/fr`.
- Do not modify `src/frame_register_manual.py`.
- Subtract sibling `map_anchor.yaml` field `map_translation_offset_xyz` from pose translation.
- Missing anchor preserves existing behavior; malformed anchor fails explicitly.
- Run only directly related unit tests and the three supplied real samples.

---

### Task 1: Anchor parser and loading integration

**Files:**
- Create: `frame_alignment/io/map_anchor.py`
- Create: `tests/test_map_anchor.py`
- Modify: `frame_alignment/io/frame_loader.py`
- Modify: `src/frame_align_6dof.py`
- Modify: `tests/test_frame_loader.py`
- Modify: `tests/test_cli_entry.py`

**Interfaces:**
- Produces: `pose_in_map_coordinates(pose, global_map_path) -> numpy.ndarray`.
- Consumes: a validated 4x4 pose and the selected global-map path.
- Missing `<map parent>/map_anchor.yaml` returns a pose copy unchanged.

- [ ] **Step 1: Write failing anchor behavior tests**

Test literal outcomes: `[10,20,30] - [-10000,-10000,0]` becomes
`[10010,10020,30]`; no anchor remains `[10,20,30]`; malformed offset raises
`ValueError` containing the anchor path. Assert the input matrix is not mutated.

- [ ] **Step 2: Run RED**

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_map_anchor -v
```

Expected: import failure because `frame_alignment.io.map_anchor` does not exist.

- [ ] **Step 3: Implement the minimal helper**

Use `yaml.safe_load`, require a mapping and a finite shape-`(3,)` offset, copy the
input pose, subtract the offset from `[:3,3]`, and return the copy.

- [ ] **Step 4: Write failing loader/CLI integration tests**

For `FrameLoader`, create map/frame/pose placeholders plus a sibling anchor and
assert `FrameData.initial_pose[:3,3]` is corrected. For `build_initial_frame`, use
the existing injected cloud reader and assert the same correction.

- [ ] **Step 5: Run integration RED**

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_frame_loader tests.test_cli_entry -v
```

Expected: literal pose translations remain uncorrected.

- [ ] **Step 6: Connect both loading paths and run GREEN**

Call `pose_in_map_coordinates` immediately after pose parsing in `FrameLoader`,
and after pose validation in `build_initial_frame`.

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_map_anchor tests.test_frame_loader tests.test_cli_entry -v
```

Expected: PASS.

- [ ] **Step 7: Commit**

```powershell
git add frame_alignment/io/map_anchor.py frame_alignment/io/frame_loader.py src/frame_align_6dof.py tests/test_map_anchor.py tests/test_frame_loader.py tests/test_cli_entry.py
git commit -m "fix: align poses with shifted map coordinates"
```

---

### Task 2: Three-map real-sample regression

**Files:**
- Modify: `tests/test_real_sample_rendering.py`

**Interfaces:**
- Consumes: real Jinhua, Yinxiu, and Xiangxue map/frame/calib/anchor files.
- Verifies: corrected 35m ROI and all four default map/frame profiles are non-empty.

- [ ] **Step 1: Generalize the real-sample test table**

Process one dataset at a time to cap memory. For each supplied path, load the map
and frame, parse/correct the pose, build the 35m ROI, resolve four default profile
geometries, and assert both profile point counts are positive.

- [ ] **Step 2: Run the real-sample regression**

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_real_sample_rendering -v
```

Expected: Jinhua, Yinxiu, and Xiangxue all PASS; an individual dataset is skipped
only if one of its documented files is absent.

- [ ] **Step 3: Run focused verification**

```powershell
.\.venv\Scripts\python.exe -m unittest tests.test_map_anchor tests.test_frame_loader tests.test_cli_entry tests.test_real_sample_rendering -v
.\.venv\Scripts\python.exe -m compileall -q frame_alignment src\frame_align_6dof.py tests
.\.venv\Scripts\python.exe src\frame_align_6dof.py --self-test
git diff --check
git diff --exit-code 15a55f0b -- src\frame_register_manual.py
```

- [ ] **Step 4: Commit**

```powershell
git add tests/test_real_sample_rendering.py
git commit -m "test: cover shifted real-map anchors"
```

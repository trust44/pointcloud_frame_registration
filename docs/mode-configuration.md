# Mode configuration

The application accepts `mode: register` (the default) and `mode: review`.
All relative paths are resolved from the configuration YAML's directory.

## Register mode

```yaml
mode: register
global_map_path: ./data/global_map.pcd
frame_cloud_map_path: ./data/frames
initial_pose_path: ./data/poses
frame_id: "1001"
output_path_yaml: ./alignment_output/yaml
output_path_pcd: ./alignment_output/pcd
interaction:
  allow_manual_adjustment: false
```

Set `interaction.allow_manual_adjustment` to `true` to enable 6DoF edits,
undo/redo, reset, and ICP.  When it is false, the register window remains a
read-only viewer but frame navigation and profile controls stay available.

## Review mode

```yaml
mode: review
global_map_path: ./data/global_map.pcd
registered_cloud_path: ./data/registered_frames
registered_pose_path: ./alignment_output/yaml
frame_id: "1001"
```

For frame `1001`, review mode loads `registered_cloud_path/1001.pcd` and,
when available, `registered_pose_path/1001.yaml`.  A supplied YAML must be an
exported alignment result containing `corrected_T_map_lidar`; that pose defines
the ROI, LiDAR origin, and profile centres.  If the pose directory is omitted,
unavailable, or lacks this frame's YAML, the coordinate-wise median of the
registered cloud (XY and Z) is used as the centre with identity orientation.
Review mode has no adjustment or export action.

## Profiles and navigation

Both modes start with a 2x3 grid: fixed XZ/YZ views; editable XZ/YZ parallel
views at `+10 m`; and editable `+30 deg`/`-60 deg` views.  The final two are
the existing optional profiles, so either may be deleted and re-added.

`Z` loads the current frame. `Left`/`Right` select the previous/next frame and
`Shift+Left`/`Shift+Right` move by ten frames. Review mode loads the selected
frame immediately; register mode keeps explicit loading. `V` runs ICP only
when manual adjustment is enabled in register mode.

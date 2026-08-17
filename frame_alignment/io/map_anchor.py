"""Convert world-coordinate poses into a shifted global-map coordinate range."""
from pathlib import Path

import numpy as np
import yaml


def pose_in_map_coordinates(pose, global_map_path):
    """Return a pose copy translated by a sibling map anchor when present."""
    corrected = np.asarray(pose, dtype=np.float64).copy()
    anchor_path = Path(global_map_path).expanduser().resolve().parent / "map_anchor.yaml"
    if not anchor_path.is_file():
        return corrected

    try:
        with anchor_path.open("r", encoding="utf-8-sig") as handle:
            data = yaml.safe_load(handle)
        if not isinstance(data, dict):
            raise ValueError("root must be a mapping")
        offset = np.asarray(data["map_translation_offset_xyz"], dtype=np.float64)
        if offset.shape != (3,) or not np.all(np.isfinite(offset)):
            raise ValueError("map_translation_offset_xyz must be a finite 3-vector")
    except (KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise ValueError("{}: invalid map anchor: {}".format(anchor_path, exc)) from exc

    corrected[:3, 3] -= offset
    return corrected

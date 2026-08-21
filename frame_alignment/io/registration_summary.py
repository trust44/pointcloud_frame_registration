"""Readers and display helpers for frame registration JSONL summaries."""
from pathlib import Path
import json

import numpy as np
import yaml


DOF_ROWS = (("tx", "m"), ("ty", "m"), ("tz", "m"),
            ("yaw", "deg"), ("pitch", "deg"), ("roll", "deg"))
STAT_ROWS = (("count", "count"), ("max", "m"), ("mean", "m"),
             ("p50", "m"), ("p95", "m"), ("rmse", "m"))


def read_summary_jsonl(path):
    """Return ``frame_id -> entry`` for non-empty JSONL records."""
    path = Path(path).expanduser().resolve()
    records = {}
    with path.open("r", encoding="utf-8-sig") as handle:
        for line_number, line in enumerate(handle, 1):
            text = line.strip()
            if not text:
                continue
            try:
                item = json.loads(text)
            except json.JSONDecodeError as exc:
                raise ValueError("Invalid registration JSONL at line {}: {}".format(line_number, path)) from exc
            if not isinstance(item, dict) or not str(item.get("frame_id", "")).strip():
                continue
            records[str(item["frame_id"]).strip()] = item
    return records


def read_gt_yaml(path):
    if path is None or not str(path).strip():
        return None
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        return None
    with path.open("r", encoding="utf-8-sig") as handle:
        data = yaml.safe_load(handle) or {}
    return data if isinstance(data, dict) else None


def _number(value):
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return value if np.isfinite(value) else None


def extract_dof(data):
    """Extract yaw/pitch/roll/tz from common summary or exported-GT layouts."""
    if not isinstance(data, dict):
        return {}
    result = {}
    local = data.get("manual_delta_lidar")
    if isinstance(local, dict):
        for key in ("tx", "ty", "tz", "yaw", "pitch", "roll"):
            aliases = {"tx": "dx_m", "ty": "dy_m", "tz": "dz_m",
                       "yaw": "yaw_deg", "pitch": "pitch_deg", "roll": "roll_deg"}
            value = _number(local.get(aliases[key]))
            if value is not None:
                result[key] = value
        return result
    initial = data.get("initial_T_map_lidar")
    corrected = data.get("corrected_T_map_lidar")
    if isinstance(initial, list) and isinstance(corrected, list):
        try:
            from scipy.spatial.transform import Rotation
            local_matrix = np.linalg.inv(np.asarray(initial, dtype=float)) @ np.asarray(corrected, dtype=float)
            yaw, pitch, roll = Rotation.from_matrix(local_matrix[:3, :3]).as_euler("ZYX", degrees=True)
            result.update(tx=float(local_matrix[0, 3]), ty=float(local_matrix[1, 3]), tz=float(local_matrix[2, 3]),
                          yaw=float(yaw), pitch=float(pitch), roll=float(roll))
            return result
        except (ValueError, TypeError, np.linalg.LinAlgError):
            pass
    angles = data.get("correction_yaw_pitch_roll_deg")
    if isinstance(angles, (list, tuple)) and len(angles) >= 3:
        result.update(yaw=_number(angles[0]), pitch=_number(angles[1]), roll=_number(angles[2]),
                     tx=0.0, ty=0.0)
    nested = data.get("manual_delta_about_lidar_origin")
    if isinstance(nested, dict):
        data = {**data, **nested}
    aliases = {
        "tx": ("x_offset_m", "dx_m", "tx_m", "tx"),
        "ty": ("y_offset_m", "dy_m", "ty_m", "ty"),
        "yaw": ("yaw_deg", "yaw"), "pitch": ("pitch_deg", "pitch"),
        "roll": ("roll_deg", "roll"), "tz": ("z_offset_m", "dz_m", "tz_m", "tz"),
    }
    for key, names in aliases.items():
        if result.get(key) is None:
            for name in names:
                value = _number(data.get(name))
                if value is not None:
                    result[key] = value
                    break
    matrix = data.get("correction_matrix_local")
    if isinstance(matrix, (list, tuple)) and len(matrix) >= 3:
        try:
            for key, row in (("tz", 2),):
                if result.get(key) is None:
                    result[key] = _number(matrix[row][3])
        except (IndexError, TypeError):
            pass
    return result


def extract_stats(data, name):
    section = data.get(name, {}) if isinstance(data, dict) else {}
    if not isinstance(section, dict):
        return {}
    result = {}
    for key, _ in STAT_ROWS:
        value = section.get(key)
        if value is None and key != "count":
            value = section.get(key + "_m")
        result[key] = _number(value)
    return result


def format_value(value, integer=False):
    if value is None:
        return "-"
    if float(value) == 0.0:
        return "0"
    if integer:
        return "{}".format(int(round(float(value))))
    return "{:.4f}".format(float(value))


def format_registration_report(entry, gt=None):
    """Create the matrix-panel text for one current-frame summary entry."""
    if not entry:
        return "未找到当前帧的配准统计结果"
    lines = [None, None]
    corrected_dof = extract_dof(entry)
    lines[1] = "{:^10} {:^12} {:^16} {:^12}".format("dof", "corrected", "ground truth", "Δ")
    gt_dof = extract_dof(gt)
    lines = ["配准自由度", "dof       corrected       ground truth       Δ"]
    for key, unit in DOF_ROWS:
        corrected = corrected_dof.get(key)
        ground = gt_dof.get(key)
        delta = None if corrected is None or ground is None else corrected - ground
        lines.append("{:^10} {:^12} {:^16} {:^12}".format(
            key + "/" + unit, format_value(corrected), format_value(ground), format_value(delta)))

    lines.extend(("", "配准前后统计", "stat      corrected       identity       Δ"))
    lines[-1] = "{:^10} {:^12} {:^16} {:^12}".format("stat", "corrected", "identity", "Δ")
    corrected_stats = extract_stats(entry, "corrected")
    identity_stats = extract_stats(entry, "identity")
    for key, unit in STAT_ROWS:
        corrected = corrected_stats.get(key)
        identity = identity_stats.get(key)
        delta = None if corrected is None or identity is None else corrected - identity
        is_count = key == "count"
        lines.append("{:^10} {:^12} {:^16} {:^12}".format(
            key + ("/" + unit if unit != "count" else ""),
            format_value(corrected, integer=is_count),
            format_value(identity, integer=is_count),
            format_value(delta, integer=is_count)))
    return "\n".join(lines)


# Redefine the renderer with fixed-width ASCII columns.  The matrix widget uses
# a monospaced font, so every row (including headers) has identical geometry.
def format_registration_report(entry, gt=None):
    if not entry:
        return "No registration summary for the current frame."

    layout = "{:^9}|{:^10}|{:^12}|{:^10}"
    corrected_dof = extract_dof(entry)
    gt_dof = extract_dof(gt)
    lines = ["Registration DOF", layout.format("dof", "corr", "GT", "Δ")]
    for key, unit in DOF_ROWS:
        corrected = corrected_dof.get(key)
        ground = gt_dof.get(key)
        delta = None if corrected is None or ground is None else corrected - ground
        lines.append(layout.format(key + "/" + unit, format_value(corrected),
                                   format_value(ground), format_value(delta)))

    lines.extend(("", "Before / after statistics",
                  layout.format("stat", "corr", "identity", "Δ")))
    corrected_stats = extract_stats(entry, "corrected")
    identity_stats = extract_stats(entry, "identity")
    for key, unit in STAT_ROWS:
        corrected = corrected_stats.get(key)
        identity = identity_stats.get(key)
        delta = None if corrected is None or identity is None else corrected - identity
        integer = key == "count"
        label = key if unit == "count" else key + "/" + unit
        lines.append(layout.format(label, format_value(corrected, integer),
                                   format_value(identity, integer), format_value(delta, integer)))
    return "\n".join(lines)

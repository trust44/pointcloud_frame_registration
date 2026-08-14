"""Parser for the sole map pose field accepted from per-frame text files."""
from pathlib import Path

import numpy as np


class PoseParseError(ValueError):
    """Raised when Tr_velo_to_map is missing, ambiguous, or not rigid."""


def validate_rigid_transform(matrix, tolerance=1e-5):
    matrix = np.asarray(matrix, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError("transform must be a 4x4 matrix")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("transform contains non-finite values")
    if not np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0), atol=1e-12):
        raise ValueError("transform last row must be [0, 0, 0, 1]")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=tolerance):
        raise ValueError("rotation is not orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=tolerance):
        raise ValueError("rotation determinant is not +1")
    return matrix.copy()


def parse_tr_velo_to_map(path):
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise PoseParseError("{}: pose file does not exist".format(path))
    matches = []
    for line in path.read_text(encoding="utf-8-sig").splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        if key.strip() == "Tr_velo_to_map":
            matches.append(value.split())
    if not matches:
        raise PoseParseError("{}: missing Tr_velo_to_map".format(path))
    if len(matches) != 1:
        raise PoseParseError("{}: duplicate Tr_velo_to_map entries ({})".format(path, len(matches)))
    if len(matches[0]) != 12:
        raise PoseParseError("{}: wrong count for Tr_velo_to_map; expected 12, got {}".format(
            path, len(matches[0])))
    try:
        values = np.asarray([float(value) for value in matches[0]], dtype=np.float64)
    except ValueError as exc:
        raise PoseParseError("{}: Tr_velo_to_map contains a non-numeric value".format(path)) from exc
    if not np.all(np.isfinite(values)):
        raise PoseParseError("{}: Tr_velo_to_map contains non-finite values".format(path))
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :] = values.reshape(3, 4)
    try:
        return validate_rigid_transform(matrix)
    except ValueError as exc:
        raise PoseParseError("{}: invalid Tr_velo_to_map: {}".format(path, exc)) from exc

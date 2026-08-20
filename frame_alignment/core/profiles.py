"""Yaw-relative thin-profile definitions and extraction in map coordinates."""
from dataclasses import dataclass

import numpy as np


ANGLE_MODE = "angle"
PARALLEL_MODE = "parallel"
REFERENCE_XZ = "XZ"
REFERENCE_YZ = "YZ"


@dataclass(frozen=True)
class ProfileSpec:
    profile_id: str
    name: str
    color: tuple
    grid_row: int
    grid_column: int
    angle_deg: float
    editable: bool
    deletable: bool
    mode: str = ANGLE_MODE
    reference: str = REFERENCE_XZ
    offset_m: float = 0.0


@dataclass(frozen=True)
class ProfileGeometry:
    profile_id: str
    name: str
    color: tuple
    center: np.ndarray
    along_axis: np.ndarray
    across_axis: np.ndarray
    height_axis: np.ndarray


_DEFAULT_PROFILE_SPECS = (
    ProfileSpec("xz", "X-Z / 0\u00b0", (1.0, 0.0, 0.0, 1.0), 0, 0, 0.0, False, False),
    ProfileSpec("yz", "Y-Z / 90\u00b0", (0.0, 1.0, 0.0, 1.0), 1, 0, 90.0, False, False),
    ProfileSpec("xz_offset", "X-Z parallel / +10 m", (0.0, 0.45, 1.0, 1.0),
                0, 1, 0.0, True, False, PARALLEL_MODE, REFERENCE_XZ, 10.0),
    ProfileSpec("yz_offset", "Y-Z parallel / +10 m", (0.65, 0.2, 0.8, 1.0),
                1, 1, 90.0, True, False, PARALLEL_MODE, REFERENCE_YZ, 10.0),
)


def default_profile_specs():
    """Return an immutable tuple containing the four required profiles."""
    return tuple(_DEFAULT_PROFILE_SPECS)


def extra_profile_spec(slot):
    """Return one of the two session-only profile definitions."""
    try:
        slot = int(slot)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("extra profile slot must be 0 or 1") from exc
    if slot == 0:
        return ProfileSpec(
            "extra_1", "Angle / +30\u00b0", (1.0, 0.55, 0.0, 1.0),
            0, 2, 30.0, True, True)
    if slot == 1:
        return ProfileSpec(
            "extra_2", "Angle / -60\u00b0", (0.0, 0.8, 0.8, 1.0),
            1, 2, -60.0, True, True)
    raise ValueError("extra profile slot must be 0 or 1")


def _finite_vector(value, name):
    vector = np.asarray(value, dtype=np.float64)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError("{} must be a finite 3-vector".format(name))
    return vector


def _finite_rotation(value):
    rotation = np.asarray(value, dtype=np.float64)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError("corrected_rotation must be a finite 3x3 matrix")
    return rotation


def profile_geometry(spec, origin, corrected_rotation):
    """Resolve one profile in map coordinates from corrected LiDAR yaw only."""
    if not isinstance(spec, ProfileSpec):
        raise TypeError("spec must be a ProfileSpec")
    origin = _finite_vector(origin, "origin")
    rotation = _finite_rotation(corrected_rotation)
    if spec.mode not in (ANGLE_MODE, PARALLEL_MODE):
        raise ValueError("profile mode must be 'angle' or 'parallel'")
    angle_deg = float(spec.angle_deg)
    if not np.isfinite(angle_deg) or angle_deg < -180.0 or angle_deg > 180.0:
        raise ValueError("profile angle must be within [-180, 180] degrees")
    offset_m = float(spec.offset_m)
    if not np.isfinite(offset_m):
        raise ValueError("profile offset must be finite")
    if spec.reference not in (REFERENCE_XZ, REFERENCE_YZ):
        raise ValueError("profile reference must be XZ or YZ")

    heading = np.arctan2(rotation[1, 0], rotation[0, 0])
    x_heading = np.array((np.cos(heading), np.sin(heading), 0.0), dtype=np.float64)
    y_heading = np.array((-np.sin(heading), np.cos(heading), 0.0), dtype=np.float64)
    effective_angle = angle_deg if spec.mode == ANGLE_MODE else (
        0.0 if spec.reference == REFERENCE_XZ else 90.0)
    theta = np.deg2rad(effective_angle)
    along = np.cos(theta) * x_heading + np.sin(theta) * y_heading
    across = -np.sin(theta) * x_heading + np.cos(theta) * y_heading
    center = origin.copy() if spec.mode == ANGLE_MODE else origin + offset_m * across
    return ProfileGeometry(
        spec.profile_id,
        spec.name,
        tuple(spec.color),
        center,
        along,
        across,
        np.array((0.0, 0.0, 1.0), dtype=np.float64),
    )


def extract_slice(points, geometry, half_length, thickness):
    """Return (distance along profile, relative map Z) for points inside a thin slice."""
    points = np.asarray(points, dtype=np.float64)
    if points.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points)):
        raise ValueError("points must have shape (N, 3) with finite values")
    if not isinstance(geometry, ProfileGeometry):
        raise TypeError("geometry must be a ProfileGeometry")
    half_length = float(half_length)
    thickness = float(thickness)
    if not np.isfinite(half_length) or half_length <= 0.0:
        raise ValueError("half_length must be positive and finite")
    if not np.isfinite(thickness) or thickness <= 0.0:
        raise ValueError("thickness must be positive and finite")
    relative = points - geometry.center
    along = relative @ geometry.along_axis
    across = relative @ geometry.across_axis
    height = relative @ geometry.height_axis
    keep = (np.abs(along) <= half_length) & (np.abs(across) <= thickness / 2.0)
    return np.column_stack((along[keep], height[keep]))


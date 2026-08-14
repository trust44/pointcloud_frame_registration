"""Map-frame 6DoF correction around the initial LiDAR origin."""
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from ..io.pose_parser import validate_rigid_transform


@dataclass(frozen=True)
class Delta:
    dx_m: float = 0.0
    dy_m: float = 0.0
    dz_m: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0

    def translation(self):
        return np.asarray((self.dx_m, self.dy_m, self.dz_m), dtype=np.float64)


def manual_transform(initial_pose, delta):
    initial_pose = validate_rigid_transform(initial_pose)
    pivot = initial_pose[:3, 3]
    rotation = Rotation.from_euler(
        "ZYX", (delta.yaw_deg, delta.pitch_deg, delta.roll_deg), degrees=True).as_matrix()
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = pivot - rotation @ pivot + delta.translation()
    return transform


class PoseModel:
    def __init__(self, initial_pose):
        self._initial_pose = None
        self._delta = Delta()
        self._undo = []
        self._redo = []
        self.set_initial_pose(initial_pose)

    @property
    def initial_pose(self):
        return self._initial_pose.copy()

    @property
    def c0(self):
        return self._initial_pose[:3, 3].copy()

    @property
    def delta(self):
        return self._delta

    @property
    def current_origin(self):
        return self.c0 + self._delta.translation()

    @property
    def transform(self):
        return manual_transform(self._initial_pose, self._delta)

    @property
    def corrected_pose(self):
        return self.transform @ self._initial_pose

    def set_initial_pose(self, initial_pose):
        self._initial_pose = validate_rigid_transform(initial_pose)
        self._delta = Delta()
        self._undo.clear()
        self._redo.clear()

    def set_delta(self, delta, record=True):
        if not np.all(np.isfinite(tuple(delta.__dict__.values()))):
            raise ValueError("6DoF delta contains non-finite values")
        if record:
            self._undo.append(self._delta)
            self._redo.clear()
        self._delta = delta

    def adjust(self, field, increment):
        if field not in Delta.__dataclass_fields__:
            raise ValueError("Unknown pose field: {}".format(field))
        values = dict(self._delta.__dict__)
        values[field] += float(increment)
        self.set_delta(Delta(**values))

    def reset(self):
        self.set_delta(Delta())

    def undo(self):
        if not self._undo:
            return False
        self._redo.append(self._delta)
        self._delta = self._undo.pop()
        return True

    def redo(self):
        if not self._redo:
            return False
        self._undo.append(self._delta)
        self._delta = self._redo.pop()
        return True

    def set_transform(self, transform):
        transform = validate_rigid_transform(transform)
        rotation = transform[:3, :3]
        displacement = rotation @ self.c0 + transform[:3, 3] - self.c0
        yaw, pitch, roll = Rotation.from_matrix(rotation).as_euler("ZYX", degrees=True)
        self.set_delta(Delta(*displacement, roll, pitch, yaw))

    def transform_points(self, points):
        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("Points must have shape (N, 3)")
        transform = self.transform
        return points @ transform[:3, :3].T + transform[:3, 3]


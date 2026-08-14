"""Oriented thin-slice extraction in map coordinates."""
import numpy as np


def extract_slice(points, center, angle_deg, half_length, thickness):
    theta = np.deg2rad(float(angle_deg))
    axis = np.array((np.cos(theta), np.sin(theta), 0.0))
    normal = np.array((-np.sin(theta), np.cos(theta), 0.0))
    relative = np.asarray(points, dtype=np.float64) - np.asarray(center, dtype=np.float64)
    along = relative @ axis
    across = relative @ normal
    keep = (np.abs(along) <= float(half_length)) & (np.abs(across) <= float(thickness) / 2.0)
    return np.column_stack((along[keep], relative[keep, 2]))


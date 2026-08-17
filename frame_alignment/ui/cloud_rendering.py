"""Color mapping helpers for independently rendered point-cloud layers."""

import numpy as np
import pyqtgraph as pg


_DEFAULT_COLOR = (0.7, 0.7, 0.7, 1.0)


def _uniform_color(value):
    if isinstance(value, str):
        text = value.strip().lstrip("#")
        if len(text) == 6:
            try:
                return np.array(tuple(int(text[index:index + 2], 16) / 255.0
                                     for index in (0, 2, 4)) + (1.0,))
            except ValueError:
                pass
    try:
        values = np.asarray(value, dtype=np.float64).reshape(-1)
        if values.size == 3:
            values = np.concatenate((values, [1.0]))
        if values.size == 4 and np.all(np.isfinite(values)):
            if np.max(values) > 1.0:
                values = values / 255.0
            if np.all((values >= 0.0) & (values <= 1.0)):
                return values
    except (TypeError, ValueError):
        pass
    return np.asarray(_DEFAULT_COLOR, dtype=np.float64)


def _scalar_values(points, scalar, origin):
    if scalar == "distance":
        return np.linalg.norm(points - np.asarray(origin, dtype=np.float64), axis=1)
    return points[:, 2]


def _get_colormap(name):
    name = str(name).lower()
    try:
        return pg.colormap.get(name)
    except (FileNotFoundError, KeyError):
        if name in ("gray", "grey"):
            return pg.ColorMap([0.0, 1.0], [[0, 0, 0, 255], [255, 255, 255, 255]])
        if name == "turbo":
            return pg.ColorMap([0.0, 0.5, 1.0], [[48, 18, 59, 255], [35, 139, 226, 255], [180, 4, 38, 255]])
        raise


def render_cloud_colors(points, native_colors, settings=None, origin=None):
    """Return an ``(N, 4)`` float RGBA array for one cloud layer."""
    settings = dict(settings or {})
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    fallback = _uniform_color(settings.get("color", _DEFAULT_COLOR))
    mode = str(settings.get("mode", "uniform")).strip().lower()
    if mode == "native":
        if native_colors is not None:
            colors = np.asarray(native_colors, dtype=np.float64)
            if colors.shape == (len(points), 3) and np.all(np.isfinite(colors)):
                colors = np.clip(colors, 0.0, 1.0)
                return np.column_stack((colors, np.ones(len(colors))))
        return np.tile(fallback, (len(points), 1))
    if mode != "cmap" or len(points) == 0:
        return np.tile(fallback, (len(points), 1))

    try:
        scalar = _scalar_values(points, str(settings.get("scalar", "z")),
                                np.zeros(3) if origin is None else origin)
        configured_range = settings.get("range", "auto")
        if isinstance(configured_range, str) and configured_range.lower() == "auto":
            low, high = float(np.min(scalar)), float(np.max(scalar))
        else:
            low, high = (float(value) for value in configured_range)
        if not np.isfinite(low) or not np.isfinite(high) or high <= low:
            return np.tile(fallback, (len(points), 1))
        normalized = np.clip((scalar - low) / (high - low), 0.0, 1.0)
        cmap = _get_colormap(settings.get("cmap", "viridis"))
        return np.asarray(cmap.map(normalized, mode="float"), dtype=np.float64)
    except (TypeError, ValueError, KeyError, IndexError):
        return np.tile(fallback, (len(points), 1))

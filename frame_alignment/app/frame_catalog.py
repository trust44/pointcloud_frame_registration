"""UI-independent discovery and annotation index for point-cloud frames."""

import re
from pathlib import Path
from typing import Optional, Tuple


def natural_frame_key(value: str) -> tuple:
    return tuple(
        (0, int(token)) if token.isdigit() else (1, token.casefold())
        for token in re.split(r"(\d+)", value)
        if token
    )


class FrameCatalog:
    def __init__(self) -> None:
        self._frame_ids = ()
        self._annotated_ids = frozenset()

    def scan(self, frame_directory) -> Tuple[str, ...]:
        if frame_directory is None or not str(frame_directory).strip():
            raise NotADirectoryError("Frame cloud directory is required")
        directory = Path(frame_directory).expanduser().resolve()
        if not directory.is_dir():
            raise NotADirectoryError(str(directory))
        frame_ids = []
        seen = set()
        for path in directory.iterdir():
            if not path.is_file() or path.suffix.casefold() != ".pcd":
                continue
            stem = path.stem
            folded = stem.casefold()
            if folded not in seen:
                seen.add(folded)
                frame_ids.append(stem)
        frame_ids.sort(key=natural_frame_key)
        self._frame_ids = tuple(frame_ids)
        self._annotated_ids = frozenset()
        return self._frame_ids

    def refresh_annotations(self, yaml_directory) -> frozenset:
        if yaml_directory is None or not str(yaml_directory).strip():
            self._annotated_ids = frozenset()
            return self._annotated_ids
        directory = Path(yaml_directory).expanduser().resolve()
        if not directory.is_dir():
            self._annotated_ids = frozenset()
            return self._annotated_ids
        self._annotated_ids = frozenset(
            frame_id for frame_id in self._frame_ids
            if (directory / (frame_id + ".yaml")).is_file()
        )
        return self._annotated_ids

    def is_annotated(self, frame_id: str) -> bool:
        return frame_id in self._annotated_ids

    def previous(self, frame_id: str) -> Optional[str]:
        if not self._frame_ids or frame_id not in self._frame_ids:
            return None
        index = self._frame_ids.index(frame_id)
        return self._frame_ids[max(0, index - 1)]

    def next(self, frame_id: str) -> Optional[str]:
        if not self._frame_ids or frame_id not in self._frame_ids:
            return None
        index = self._frame_ids.index(frame_id)
        return self._frame_ids[min(len(self._frame_ids) - 1, index + 1)]

    def offset(self, frame_id: str, amount: int) -> Optional[str]:
        """Return the frame ``amount`` places away, clamped at either end."""
        if not self._frame_ids or frame_id not in self._frame_ids:
            return None
        index = self._frame_ids.index(frame_id)
        destination = max(0, min(len(self._frame_ids) - 1, index + int(amount)))
        return self._frame_ids[destination]

    @property
    def frame_ids(self) -> Tuple[str, ...]:
        return self._frame_ids

    @property
    def annotated_count(self) -> int:
        return len(self._annotated_ids)

    @property
    def total_count(self) -> int:
        return len(self._frame_ids)

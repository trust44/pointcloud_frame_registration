from pathlib import Path

import pytest

from frame_alignment.app.frame_catalog import FrameCatalog, natural_frame_key


def test_scan_naturally_sorts_top_level_pcd_files(tmp_path):
    root = tmp_path / "frames"
    root.mkdir()
    for name in ("10.pcd", "2.PCD", "frame9.pcd", "frame10.pcd", "ignored.txt"):
        (root / name).touch()
    (root / "nested").mkdir()
    (root / "nested" / "1.pcd").touch()

    catalog = FrameCatalog()
    assert catalog.scan(root) == ("2", "10", "frame9", "frame10")
    assert catalog.previous("2") == "2"
    assert catalog.next("2") == "10"
    assert catalog.next("frame10") == "frame10"
    assert catalog.offset("2", 10) == "frame10"
    assert catalog.offset("frame10", -10) == "2"


def test_annotations_only_match_scanned_ids(tmp_path):
    root = tmp_path / "frames"
    root.mkdir()
    for name in ("2.pcd", "10.pcd", "frame9.pcd", "frame10.pcd"):
        (root / name).touch()
    yaml_dir = tmp_path / "yaml"
    yaml_dir.mkdir()
    (yaml_dir / "2.yaml").touch()
    (yaml_dir / "unrelated.yaml").touch()

    catalog = FrameCatalog()
    catalog.scan(root)
    catalog.refresh_annotations(yaml_dir)
    assert catalog.is_annotated("2")
    assert not catalog.is_annotated("10")
    assert catalog.annotated_count == 1
    assert catalog.total_count == 4


def test_invalid_scan_does_not_replace_previous_ids(tmp_path):
    root = tmp_path / "frames"
    root.mkdir()
    (root / "1.pcd").touch()
    catalog = FrameCatalog()
    catalog.scan(root)

    with pytest.raises(NotADirectoryError):
        catalog.scan(tmp_path / "missing")
    assert catalog.frame_ids == ("1",)


def test_duplicate_stems_differing_only_by_case_are_single_entry(tmp_path):
    root = tmp_path / "frames"
    root.mkdir()
    (root / "Foo.pcd").touch()
    (root / "foo.PCD").touch()
    catalog = FrameCatalog()
    assert catalog.scan(root) == ("Foo",)


def test_navigation_empty_or_unknown_and_invalid_annotations(tmp_path):
    catalog = FrameCatalog()
    assert catalog.previous("missing") is None
    assert catalog.next("missing") is None
    assert catalog.refresh_annotations(tmp_path / "missing") == frozenset()
    assert catalog.annotated_count == 0
    assert natural_frame_key("frame10") < natural_frame_key("frame100")


def test_blank_directories_do_not_resolve_to_working_directory(tmp_path, monkeypatch):
    """Blank paths must not scan or index files from the process working directory."""
    frame_dir = tmp_path / "frames"
    frame_dir.mkdir()
    (frame_dir / "1.pcd").touch()
    (tmp_path / "1.yaml").touch()
    monkeypatch.chdir(tmp_path)

    catalog = FrameCatalog()
    assert catalog.scan(frame_dir) == ("1",)
    catalog.refresh_annotations(tmp_path)
    assert catalog.annotated_count == 1

    with pytest.raises(NotADirectoryError):
        catalog.scan("")
    assert catalog.frame_ids == ("1",)
    assert catalog.refresh_annotations("") == frozenset()


def test_catalog_expands_user_directories(tmp_path, monkeypatch):
    """User-relative frame and YAML paths resolve before directory validation."""
    frame_dir = tmp_path / "frames"
    yaml_dir = tmp_path / "yaml"
    frame_dir.mkdir()
    yaml_dir.mkdir()
    (frame_dir / "2.pcd").touch()
    (yaml_dir / "2.yaml").touch()
    monkeypatch.setenv("USERPROFILE", str(tmp_path))

    catalog = FrameCatalog()
    assert catalog.scan("~/frames") == ("2",)
    assert catalog.refresh_annotations("~/yaml") == frozenset({"2"})

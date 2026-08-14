import tempfile
from pathlib import Path
import unittest


class FrameRequestTests(unittest.TestCase):
    def test_normalize_frame_id_strips_supported_extension(self):
        from frame_alignment.contracts import normalize_frame_id

        self.assertEqual(normalize_frame_id("1781158324500077000.pcd"), "1781158324500077000")
        self.assertEqual(normalize_frame_id("frame-A_01.TXT"), "frame-A_01")

    def test_normalize_frame_id_rejects_path_traversal_and_empty_names(self):
        from frame_alignment.contracts import normalize_frame_id

        for value in ("", "../frame", "folder/frame", r"folder\frame", ".", "..", "frame id"):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    normalize_frame_id(value)

    def test_load_request_resolves_exact_matching_files(self):
        from frame_alignment.contracts import LoadRequest

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            frame_dir = root / "frames"
            pose_dir = root / "poses"
            frame_dir.mkdir()
            pose_dir.mkdir()
            (root / "map.pcd").write_bytes(b"map")
            (frame_dir / "frame.001.pcd").write_bytes(b"frame")
            (pose_dir / "frame.001.txt").write_text("pose", encoding="utf-8")

            request = LoadRequest(root / "map.pcd", frame_dir, pose_dir, "frame.001.txt")
            paths = request.resolve_existing_paths()

            self.assertEqual(paths.frame_id, "frame.001")
            self.assertEqual(paths.global_map_file, (root / "map.pcd").resolve())
            self.assertEqual(paths.frame_cloud_file, (frame_dir / "frame.001.pcd").resolve())
            self.assertEqual(paths.initial_pose_file, (pose_dir / "frame.001.txt").resolve())

    def test_load_request_reports_missing_exact_frame_file(self):
        from frame_alignment.contracts import LoadRequest

        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "map.pcd").write_bytes(b"map")
            (root / "frames").mkdir()
            (root / "poses").mkdir()
            (root / "poses" / "wanted.txt").write_text("pose", encoding="utf-8")
            request = LoadRequest(root / "map.pcd", root / "frames", root / "poses", "wanted")

            with self.assertRaisesRegex(FileNotFoundError, r"wanted\.pcd"):
                request.resolve_existing_paths()

    def test_export_request_requires_yaml_and_optional_pcd_directory(self):
        from frame_alignment.contracts import ExportRequest

        with self.assertRaisesRegex(ValueError, "YAML"):
            ExportRequest("", False, "").validate()
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            with self.assertRaisesRegex(ValueError, "PCD"):
                ExportRequest(root, True, "").validate()
            request = ExportRequest(root, False, "")
            self.assertEqual(request.validate().yaml_output_dir, root.resolve())


if __name__ == "__main__":
    unittest.main()

import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np


ENTRY = Path(__file__).parents[1] / "src" / "frame_align_6dof.py"


def load_entry():
    spec = importlib.util.spec_from_file_location("frame_align_6dof_entry", ENTRY)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class CliEntryTests(unittest.TestCase):
    def test_new_config_resolves_relative_paths_without_requiring_pose_matrix(self):
        module = load_entry()
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            config_path = root / "settings.yaml"
            config_path.write_text(
                "global_map_path: data/map.pcd\n"
                "frame_cloud_map_path: frames\n"
                "initial_pose_path: poses\n"
                "frame_id: '000123'\n"
                "output_path_yaml: output\n",
                encoding="utf-8",
            )
            config = module.load_config(config_path)
            self.assertEqual(config["global_map_path"], str((root / "data/map.pcd").resolve()))
            self.assertEqual(config["frame_cloud_map_path"], str((root / "frames").resolve()))
            self.assertEqual(config["frame_id"], "000123")

    def test_legacy_files_build_an_initial_frame_without_transforming_source(self):
        module = load_entry()
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            map_path = root / "map.pcd"
            source_path = root / "frame.pcd"
            pose_path = root / "frame.txt"
            map_path.touch()
            source_path.touch()
            pose_path.write_text(
                "P0: 1 2 3\n"
                "Tr_velo_to_map: 1 0 0 10 0 1 0 20 0 0 1 30\n",
                encoding="utf-8",
            )
            points = {
                map_path.resolve(): np.array([[10.0, 20.0, 30.0]]),
                source_path.resolve(): np.array([[1.0, 2.0, 3.0]]),
            }

            def reader(path):
                return module.Cloud(points[Path(path).resolve()].copy())

            frame = module.build_initial_frame(
                {"global_map_path": str(map_path), "frame_cloud_map_path": str(source_path)},
                pose_path=pose_path,
                cloud_reader=reader,
            )
            np.testing.assert_array_equal(frame.source_cloud.points, [[1.0, 2.0, 3.0]])
            np.testing.assert_array_equal(frame.initial_pose[:3, 3], [10.0, 20.0, 30.0])
            self.assertEqual(frame.frame_id, "frame")

    def test_no_arguments_are_valid_for_empty_gui_startup(self):
        module = load_entry()
        args = module.parse_args([])
        self.assertIsNone(args.config)
        self.assertIsNone(args.map_path)


if __name__ == "__main__":
    unittest.main()

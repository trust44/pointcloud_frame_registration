import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock


ENTRY = Path(__file__).parents[1] / "src" / "frame_align_6dof.py"


def load_entry():
    spec = importlib.util.spec_from_file_location("frame_align_6dof_legacy_entry", ENTRY)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class LegacyOutputCompatibilityTests(unittest.TestCase):
    def test_exact_legacy_command_creates_and_prefills_default_output(self):
        module = load_entry()
        with tempfile.TemporaryDirectory() as temp:
            previous = Path.cwd()
            os.chdir(temp)
            try:
                sentinel = object()
                captured = {}

                def fake_gui(**kwargs):
                    captured.update(kwargs)
                    return 17

                with mock.patch.object(module, "build_initial_frame", return_value=sentinel), \
                        mock.patch.object(module, "run_gui", side_effect=fake_gui):
                    result = module.main([
                        "--map", "map.pcd", "--source", "frame.pcd", "--pose", "pose.txt"
                    ])

                expected = (Path(temp) / "alignment_output").resolve()
                self.assertEqual(result, 17)
                self.assertTrue(expected.is_dir())
                self.assertEqual(captured["config"]["output_path_yaml"], str(expected))
                self.assertEqual(captured["config"]["output_path_pcd"], str(expected))
                self.assertIs(captured["initial_frame"], sentinel)
            finally:
                os.chdir(str(previous))


if __name__ == "__main__":
    unittest.main()

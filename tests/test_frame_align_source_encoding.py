from pathlib import Path
import unittest


ROOT = Path(__file__).parents[1]


class FrameAlignSourceEncodingTests(unittest.TestCase):
    def test_all_alignment_sources_are_valid_utf8_and_compile(self):
        paths = [ROOT / "src" / "frame_align_6dof.py"]
        paths.extend(sorted((ROOT / "frame_alignment").rglob("*.py")))
        for path in paths:
            text = path.read_text(encoding="utf-8")
            compile(text, str(path), "exec")
            self.assertNotIn("锛", text, str(path))
            self.assertNotIn("鍗", text, str(path))


if __name__ == "__main__":
    unittest.main()

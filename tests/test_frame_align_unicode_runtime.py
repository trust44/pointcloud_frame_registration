import unittest


class FrameAlignUnicodeRuntimeTests(unittest.TestCase):
    def test_gui_runtime_labels_contain_real_delta_degree_and_chinese(self):
        from frame_alignment.ui.main_window import MainWindow

        titles = [row[1] for row in MainWindow.pose_fields]
        self.assertIn("ΔX map (m)", titles)
        self.assertIn("横滚 map (°)", titles)
        self.assertIn("俯仰 map (°)", titles)
        self.assertIn("偏航 map (°)", titles)
        self.assertNotIn("Adjucted", " ".join(titles))


if __name__ == "__main__":
    unittest.main()

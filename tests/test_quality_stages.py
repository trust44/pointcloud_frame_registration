import unittest


class QualityStageTests(unittest.TestCase):
    def test_manual_invalidation_preserves_icp_snapshots_and_clears_manual_error(self):
        from frame_alignment.app.controller import AlignmentController

        controller = AlignmentController(None, None)
        controller.quality["initial_icp"] = {"rmse_m": 0.1}
        controller.quality["icp_error"] = {"median_m": 0.2}
        controller.quality["manual_error"] = {"median_m": 0.3}
        controller.quality["icp_rmse_m"] = 0.1

        controller.invalidate_quality()

        self.assertEqual(controller.quality["initial_icp"], {"rmse_m": 0.1})
        self.assertEqual(controller.quality["icp_error"], {"median_m": 0.2})
        self.assertIsNone(controller.quality["manual_error"])
        self.assertIsNone(controller.quality["icp_rmse_m"])

    def test_recording_icp_keeps_first_snapshot_and_updates_current_error(self):
        from frame_alignment.app.controller import AlignmentController

        controller = AlignmentController(None, None)
        controller.record_icp({"icp_rmse_m": 0.1, "icp_fitness": 0.8}, {"median_m": 0.2})
        controller.record_icp({"icp_rmse_m": 0.05, "icp_fitness": 0.9}, {"median_m": 0.1})

        self.assertEqual(controller.quality["initial_icp"]["icp_rmse_m"], 0.1)
        self.assertEqual(controller.quality["icp_error"], {"median_m": 0.1})


if __name__ == "__main__":
    unittest.main()

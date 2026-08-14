from dataclasses import dataclass
import unittest

import numpy as np


@dataclass
class DummyFrame:
    frame_id: str
    initial_pose: object


class DummyModel:
    def __init__(self, pose):
        self.pose = pose
        self.delta = "zero"


class ControllerTests(unittest.TestCase):
    def test_failed_load_keeps_previous_valid_frame_and_model(self):
        from frame_alignment.app.controller import AlignmentController

        first = DummyFrame("first", np.eye(4))
        outcomes = [first, ValueError("bad pose: D:/poses/second.txt")]
        class Loader:
            def load_frame(self, request):
                result = outcomes.pop(0)
                if isinstance(result, Exception):
                    raise result
                return result

        rendered, errors = [], []
        controller = AlignmentController(Loader(), DummyModel, rendered.append, errors.append)
        self.assertTrue(controller.load_current_frame(object()))
        previous_model = controller.pose_model
        self.assertFalse(controller.load_current_frame(object()))

        self.assertIs(controller.current_frame, first)
        self.assertIs(controller.pose_model, previous_model)
        self.assertEqual(rendered, [first])
        self.assertIn("second.txt", errors[0])

    def test_successful_new_frame_resets_model_and_quality(self):
        from frame_alignment.app.controller import AlignmentController

        frames = [DummyFrame("first", np.eye(4)), DummyFrame("second", np.eye(4) * 2)]
        class Loader:
            def load_frame(self, request):
                return frames.pop(0)

        controller = AlignmentController(Loader(), DummyModel)
        controller.load_current_frame(object())
        first_model = controller.pose_model
        controller.quality["icp_rmse_m"] = 0.1
        controller.load_current_frame(object())

        self.assertEqual(controller.current_frame.frame_id, "second")
        self.assertIsNot(controller.pose_model, first_model)
        self.assertEqual(controller.pose_model.delta, "zero")
        self.assertIsNone(controller.quality["icp_rmse_m"])


if __name__ == "__main__":
    unittest.main()

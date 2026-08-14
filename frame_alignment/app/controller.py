"""State-safe application controller for loading and refreshing one frame."""


def empty_quality():
    return {
        "nn_residual_median_m": None,
        "nn_residual_p95_m": None,
        "icp_rmse_m": None,
        "icp_fitness": None,
    }


class AlignmentController:
    def __init__(self, loader, model_factory, on_loaded=None, on_error=None):
        self.loader = loader
        self.model_factory = model_factory
        self.on_loaded = on_loaded or (lambda frame: None)
        self.on_error = on_error or (lambda message: None)
        self.current_frame = None
        self.pose_model = None
        self.quality = empty_quality()

    def load_current_frame(self, request):
        try:
            candidate = self.loader.load_frame(request)
            candidate_model = self.model_factory(candidate.initial_pose)
        except Exception as exc:
            self.on_error(str(exc))
            return False
        self.current_frame = candidate
        self.pose_model = candidate_model
        self.quality = empty_quality()
        self.on_loaded(candidate)
        return True

    def invalidate_quality(self):
        self.quality = empty_quality()

    def refresh_views(self):
        if self.current_frame is not None:
            self.on_loaded(self.current_frame)

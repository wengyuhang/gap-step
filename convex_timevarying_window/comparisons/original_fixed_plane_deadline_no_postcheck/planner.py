"""Only remove dense post-solve validation; leave optimization/events intact."""
from convex_timevarying_window.wang_progress_mpc.planner import Planner as DeadlinePlanner, Config


class Planner(DeadlinePlanner):
    def validate(self, plan):
        # None means NOT CHECKED, never positive clearance or a safety claim.
        plan.post_trajectory_validation_skipped = True
        return None

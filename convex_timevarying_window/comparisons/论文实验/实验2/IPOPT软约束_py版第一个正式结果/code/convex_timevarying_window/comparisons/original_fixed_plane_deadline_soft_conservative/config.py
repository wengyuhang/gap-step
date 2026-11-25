"""Objective-only conservative variant of the no-postcheck deadline baseline."""
from dataclasses import dataclass
from convex_timevarying_window.comparisons.original_fixed_plane_deadline_no_postcheck.planner import Config as BaseConfig


@dataclass(frozen=True)
class Config(BaseConfig):
    soft_clearance_buffer: float = .25
    soft_clearance_weight: float = 10.

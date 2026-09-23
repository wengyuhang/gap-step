from dataclasses import dataclass


@dataclass(frozen=True)
class Config:
    dt: float = .020
    integration_dt: float = .005
    geometry_dt: float = .010
    horizon: float = 1.0
    samples: int = 16384
    temperature: float = 1.
    radius: float = .3859447197013662
    error_margin: float = .35
    feedback_geometry_reserve: float = 0.  # Candidate tightening; NEVER changes gate event slab.
    staged_entry: bool = False  # Explicit developmental ablation, not default.
    terminal_velocity_cost: bool = False
    frame_offset: float = .065
    frame_thickness: float = .085
    speed: float = 12.
    acceleration: float = 8.
    tilt: float = 42.
    max_rotor_thrust: float = 8.54858
    min_rotor_thrust: float = 8.54858e-6*150**2
    planning_budget: float = .100
    anchor_delay: float = .0
    tracking_position_threshold: float = .5
    tracking_velocity_threshold: float = 2.
    buffer_time: float = .080
    computation_clearance: float = .05  # Same threshold as danger detection; no extra reserve.
    consecutive_windows: int = 1
    request_interval: float = .0
    lower: tuple = (-28., -30., .65)
    upper: tuple = (28., 22., 12.)

    def __post_init__(self):
        if self.samples<24:raise ValueError('at least 24 samples required for proposal groups')

    @property
    def computation_duration(self):
        return self.planning_budget

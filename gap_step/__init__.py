"""GAP-Step minimal research project."""

__all__ = ["__version__"]

__version__ = "0.1.0"
from gap_step.env import ContinuousMazeEnv
from gap_step.model import TeacherActorCritic

__all__ = ["ContinuousMazeEnv", "TeacherActorCritic"]

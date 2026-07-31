"""Optimistic point-mass timing model used by the front end."""

from __future__ import annotations

import math


def lower_bound_time(distance: float, v_max: float) -> float:
    return max(0.0, float(distance)) / float(v_max)


def point_mass_time(distance: float, v_max: float, a_max: float) -> float:
    value = max(0.0, float(distance))
    switching_distance = v_max * v_max / a_max
    if value <= switching_distance:
        return 2.0 * math.sqrt(value / a_max)
    return 2.0 * v_max / a_max + (value - switching_distance) / v_max


__all__ = ["lower_bound_time", "point_mass_time"]


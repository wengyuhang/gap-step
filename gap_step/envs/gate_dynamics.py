from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class GateSpec:
    y: float
    d_min: float = 0.0
    d_max: float = 1.2
    omega: float = 1.0
    phi: float = 0.0
    fixed_width: float | None = None

    def width(self, t: float) -> float:
        if self.fixed_width is not None:
            return float(self.fixed_width)
        amp = 0.5 * (self.d_max - self.d_min)
        return float(self.d_min + amp * (1.0 + np.sin(self.omega * t + self.phi)))


class DynamicGates:
    def __init__(self, gates: Iterable[GateSpec], robot_radius: float, safe_margin: float):
        self.gates = list(gates)
        if not self.gates:
            raise ValueError("At least one gate is required.")
        self.robot_radius = float(robot_radius)
        self.safe_margin = float(safe_margin)

    @classmethod
    def from_config(cls, config: dict) -> "DynamicGates":
        gates = [
            GateSpec(
                y=float(g["y"]),
                d_min=float(g.get("d_min", 0.0)),
                d_max=float(g.get("d_max", 1.2)),
                omega=float(g.get("omega", 1.0)),
                phi=float(g.get("phi", 0.0)),
                fixed_width=None if g.get("fixed_width") is None else float(g["fixed_width"]),
            )
            for g in config["gates"]
        ]
        return cls(
            gates=gates,
            robot_radius=float(config.get("robot_radius", 0.18)),
            safe_margin=float(config.get("safe_margin", 0.10)),
        )

    @property
    def num_gates(self) -> int:
        return len(self.gates)

    @property
    def centers(self) -> np.ndarray:
        return np.array([g.y for g in self.gates], dtype=np.float32)

    def widths(self, t: float) -> np.ndarray:
        return np.array([g.width(t) for g in self.gates], dtype=np.float32)

    def safe_flags(self, t: float) -> np.ndarray:
        threshold = 2.0 * self.robot_radius + self.safe_margin
        return (self.widths(t) > threshold).astype(np.float32)

    def opening_bounds(self, t: float) -> list[tuple[float, float]]:
        widths = self.widths(t)
        return [
            (float(g.y - 0.5 * w), float(g.y + 0.5 * w))
            for g, w in zip(self.gates, widths)
        ]

    def gate_at_y(self, y: float, t: float, radius: float | None = None) -> int | None:
        radius = self.robot_radius if radius is None else radius
        for idx, (lo, hi) in enumerate(self.opening_bounds(t)):
            if lo + radius <= y <= hi - radius:
                return idx
        return None

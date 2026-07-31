"""Shared serializable data structures."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
from scipy.interpolate import PchipInterpolator


@dataclass(frozen=True)
class Disc:
    center: np.ndarray
    radius: float

    def __post_init__(self) -> None:
        center = np.asarray(self.center, dtype=float)
        if center.shape != (2,) or not np.all(np.isfinite(center)):
            raise ValueError("disc center must be a finite two-vector")
        if not np.isfinite(self.radius) or self.radius <= 0.0:
            raise ValueError("disc radius must be positive")
        object.__setattr__(self, "center", center)

    def to_dict(self) -> dict[str, Any]:
        return {"center": self.center.tolist(), "radius": self.radius}


@dataclass
class DiscTrack:
    gate_id: int
    track_id: int
    times: np.ndarray
    centers_local: np.ndarray
    radii: np.ndarray
    active_intervals: list[tuple[float, float]]
    _center_curve: PchipInterpolator = field(init=False, repr=False)
    _radius_curve: PchipInterpolator = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self.times = np.asarray(self.times, dtype=float)
        self.centers_local = np.asarray(self.centers_local, dtype=float)
        self.radii = np.asarray(self.radii, dtype=float)
        if (
            self.times.ndim != 1
            or self.centers_local.shape != (len(self.times), 2)
            or self.radii.shape != self.times.shape
            or len(self.times) < 2
        ):
            raise ValueError("invalid DiscTrack samples")
        if np.any(np.diff(self.times) <= 0.0) or np.any(self.radii <= 0.0):
            raise ValueError("track times/radii must be strictly positive and ordered")
        self.active_intervals = [
            (float(start), float(stop)) for start, stop in self.active_intervals
        ]
        self._center_curve = PchipInterpolator(
            self.times, self.centers_local, axis=0, extrapolate=False
        )
        self._radius_curve = PchipInterpolator(
            self.times, self.radii, extrapolate=False
        )

    def active_at(self, time: float, tolerance: float = 1.0e-9) -> bool:
        return any(
            start - tolerance <= time <= stop + tolerance
            for start, stop in self.active_intervals
        )

    def interval_containing(self, time: float) -> tuple[float, float] | None:
        for interval in self.active_intervals:
            if interval[0] - 1.0e-9 <= time <= interval[1] + 1.0e-9:
                return interval
        return None

    def evaluate(
        self, time: float
    ) -> tuple[np.ndarray, float, np.ndarray, float]:
        if not self.active_at(time):
            raise ValueError(f"track {self.track_id} is inactive at t={time}")
        center = np.asarray(self._center_curve(time), dtype=float)
        radius = float(self._radius_curve(time))
        center_dot = np.asarray(self._center_curve.derivative()(time), dtype=float)
        radius_dot = float(self._radius_curve.derivative()(time))
        return center, radius, center_dot, radius_dot

    def to_dict(self) -> dict[str, Any]:
        return {
            "gate_id": self.gate_id,
            "track_id": self.track_id,
            "times": self.times.tolist(),
            "centers_local": self.centers_local.tolist(),
            "radii": self.radii.tolist(),
            "active_intervals": [list(value) for value in self.active_intervals],
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "DiscTrack":
        return cls(
            gate_id=int(data["gate_id"]),
            track_id=int(data["track_id"]),
            times=np.asarray(data["times"], dtype=float),
            centers_local=np.asarray(data["centers_local"], dtype=float),
            radii=np.asarray(data["radii"], dtype=float),
            active_intervals=[tuple(x) for x in data["active_intervals"]],
        )


@dataclass(frozen=True)
class GraphNode:
    node_id: int
    gate_index: int
    track_id: int
    time: float
    center_local: np.ndarray
    center_world: np.ndarray
    radius: float
    kind: str = "gate"

    def to_dict(self) -> dict[str, Any]:
        return {
            "node_id": self.node_id,
            "gate_index": self.gate_index,
            "track_id": self.track_id,
            "time": self.time,
            "center_local": np.asarray(self.center_local).tolist(),
            "center_world": np.asarray(self.center_world).tolist(),
            "radius": self.radius,
            "kind": self.kind,
        }


@dataclass(frozen=True)
class GraphEdge:
    source: int
    target: int
    point_mass_time: float
    distance: float

    def to_dict(self) -> dict[str, Any]:
        return self.__dict__.copy()


@dataclass
class GraphSolution:
    nodes: dict[int, GraphNode]
    edges: list[GraphEdge]
    selected_node_ids: list[int]
    objective: tuple[float, float]
    blocked_edges: set[tuple[int, int]] = field(default_factory=set)

    @property
    def selected_nodes(self) -> list[GraphNode]:
        return [self.nodes[value] for value in self.selected_node_ids]

    def to_dict(self) -> dict[str, Any]:
        return {
            "nodes": [self.nodes[key].to_dict() for key in sorted(self.nodes)],
            "edges": [edge.to_dict() for edge in self.edges],
            "selected_node_ids": self.selected_node_ids,
            "objective": list(self.objective),
            "blocked_edges": [list(item) for item in sorted(self.blocked_edges)],
        }


@dataclass
class PlanResult:
    success: bool
    method: str
    scenario_name: str
    total_flight_time: float
    graph_coarse: GraphSolution | None
    graph_fine: GraphSolution | None
    backend: Any = None
    validation: Any = None
    disc_tracks: dict[int, list[DiscTrack]] = field(default_factory=dict)
    lazy_attempts: list[dict[str, Any]] = field(default_factory=list)
    failure_reason: str = ""

    def metrics_dict(self) -> dict[str, Any]:
        validation = (
            {} if self.validation is None else self.validation.to_dict()
        )
        return {
            "success": self.success,
            "method": self.method,
            "scenario": self.scenario_name,
            "total_flight_time": self.total_flight_time,
            "num_graph_nodes": 0
            if self.graph_fine is None
            else len(self.graph_fine.nodes),
            "num_graph_edges": 0
            if self.graph_fine is None
            else len(self.graph_fine.edges),
            "num_lazy_repairs": max(0, len(self.lazy_attempts) - 1),
            "failure_reason": self.failure_reason,
            **validation,
        }


__all__ = [
    "Disc",
    "DiscTrack",
    "GraphEdge",
    "GraphNode",
    "GraphSolution",
    "PlanResult",
]

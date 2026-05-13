from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np

from gap_step.utils import circle_intersects_rect, wrap_angle


Orientation = Literal["vertical", "horizontal"]
SplitName = Literal["train", "id_test", "ood_size_test", "ood_dynamics_test"]
Cell = tuple[int, int]
CellEdge = tuple[Cell, Cell]


@dataclass(frozen=True)
class WallSegment:
    id: int
    orientation: Orientation
    coord: float
    span: tuple[float, float]


@dataclass(frozen=True)
class Gate:
    id: int
    wall_id: int
    cell_edge: CellEdge
    orientation: Orientation
    center: np.ndarray
    slot_width: float
    wall_thickness: float
    d_min: float
    d_max: float
    omega_d: float
    phi_d: float
    theta0: float
    theta_amp: float
    omega_theta: float
    phi_theta: float
    theta_ref: float = 0.0
    theta_safe: float = 0.25

    def width(self, t: float) -> float:
        raw = self.d_min + 0.5 * (self.d_max - self.d_min) * (1.0 + np.sin(self.omega_d * t + self.phi_d))
        return float(min(raw, self.slot_width))

    def theta(self, t: float) -> float:
        return float(self.theta0 + self.theta_amp * np.sin(self.omega_theta * t + self.phi_theta))

    def is_safe(self, t: float, robot_radius: float, safe_margin: float) -> bool:
        # 窗口必须同时满足“开度足够”和“旋转角度接近参考方向”。
        safe_width = self.width(t) >= 2.0 * robot_radius + safe_margin
        safe_angle = abs(wrap_angle(self.theta(t) - self.theta_ref)) <= self.theta_safe
        return bool(safe_width and safe_angle)

    @property
    def axis_value(self) -> float:
        return float(self.center[1] if self.orientation == "vertical" else self.center[0])

    @property
    def slot_axis_bounds(self) -> tuple[float, float]:
        half = 0.5 * self.slot_width
        return float(self.axis_value - half), float(self.axis_value + half)

    @property
    def slot_rect(self) -> dict[str, float]:
        half = 0.5 * self.wall_thickness
        lo, hi = self.slot_axis_bounds
        if self.orientation == "vertical":
            return {"xmin": float(self.center[0] - half), "xmax": float(self.center[0] + half), "ymin": lo, "ymax": hi}
        return {"xmin": lo, "xmax": hi, "ymin": float(self.center[1] - half), "ymax": float(self.center[1] + half)}

    def safe_axis_bounds(self, t: float, robot_radius: float) -> tuple[float, float]:
        half = 0.5 * self.width(t)
        return float(self.axis_value - half + robot_radius), float(self.axis_value + half - robot_radius)


@dataclass(frozen=True)
class Maze:
    S: float
    start: np.ndarray
    goal: np.ndarray
    walls: list[dict[str, float]]
    gates: list[Gate]
    wall_segments: list[WallSegment]
    wall_thickness: float
    rows: int
    cols: int
    open_edges: set[CellEdge]
    gate_edges: set[CellEdge]
    start_cell: Cell
    goal_cell: Cell


STAGE_ORDER = ["C1", "C1_5", "C2A", "C2B", "C3", "C4", "C5"]


def canonical_stage_name(stage_name: str) -> str:
    if stage_name == "C2":
        return "C2B"
    return stage_name


def stage_from_step(step: int, steps_per_stage: int) -> str:
    idx = min(len(STAGE_ORDER) - 1, max(0, int(step) // max(1, int(steps_per_stage))))
    return STAGE_ORDER[idx]


def sample_maze(stage_name: str = "C5", split: SplitName = "train", seed: int | None = None) -> Maze:
    rng = np.random.default_rng(seed)
    stage_name = canonical_stage_name(stage_name)
    S = _sample_size(stage_name, split, rng)
    rows, cols, gate_range, extra_openings = _stage_layout(stage_name, rng)
    wall_thickness = 0.12

    # 先生成离散迷宫拓扑，再映射到连续坐标；这样形态更接近普通迷宫。
    open_edges = _randomized_dfs_maze(rows, cols, rng)
    open_edges |= _extra_open_edges(rows, cols, open_edges, extra_openings, rng)
    start_cell = (rows - 1, 0)
    goal_cell = (0, cols - 1)
    path_edges = _path_edges(rows, cols, open_edges, start_cell, goal_cell)
    gate_edges = _choose_gate_edges(path_edges, open_edges, gate_range, rng)

    geom = _grid_geometry(S, rows, cols)
    wall_segments, gates = _segments_and_gates(stage_name, split, rows, cols, open_edges, gate_edges, geom, wall_thickness, rng)
    walls = _build_wall_rects(wall_segments, gates, wall_thickness)
    start = _cell_center(start_cell, geom)
    goal = _cell_center(goal_cell, geom)
    start, goal = _nudge_if_blocked(start, goal, walls)
    return Maze(S, start, goal, walls, gates, wall_segments, wall_thickness, rows, cols, open_edges, gate_edges, start_cell, goal_cell)


def _sample_size(stage_name: str, split: SplitName, rng: np.random.Generator) -> float:
    if split == "id_test":
        return float(rng.choice([15.0, 19.0, 23.0]))
    if split == "ood_size_test":
        return float(rng.choice([17.0, 21.0, 25.0, 31.0]))
    if split == "ood_dynamics_test":
        return float(rng.choice([17.0, 25.0, 31.0]))
    stage_name = canonical_stage_name(stage_name)
    if stage_name == "C1":
        return 15.0
    if stage_name in {"C1_5", "C2A", "C2B", "C3"}:
        return float(rng.choice([15.0, 19.0]))
    return float(rng.choice([15.0, 19.0, 23.0]))


def _stage_layout(stage_name: str, rng: np.random.Generator) -> tuple[int, int, tuple[int, int], int]:
    stage_name = canonical_stage_name(stage_name)
    if stage_name == "C1":
        return 3, 4, (1, 1), 0
    if stage_name == "C1_5":
        return 3, 4, (1, 1), 0
    if stage_name == "C2A":
        return 3, 4, (1, 1), 0
    if stage_name == "C2B":
        return 4, 5, (1, 2), 1
    if stage_name == "C3":
        return (4, 5, (2, 3), 2) if rng.random() < 0.5 else (5, 6, (2, 3), 3)
    if stage_name == "C4":
        return 5, 6, (3, 5), 4
    return 6, 7, (6, 10), 8


def _edge(a: Cell, b: Cell) -> CellEdge:
    return (a, b) if a <= b else (b, a)


def _neighbors(cell: Cell, rows: int, cols: int) -> list[Cell]:
    r, c = cell
    out = []
    if r > 0:
        out.append((r - 1, c))
    if r + 1 < rows:
        out.append((r + 1, c))
    if c > 0:
        out.append((r, c - 1))
    if c + 1 < cols:
        out.append((r, c + 1))
    return out


def _randomized_dfs_maze(rows: int, cols: int, rng: np.random.Generator) -> set[CellEdge]:
    open_edges: set[CellEdge] = set()
    stack = [(0, 0)]
    visited = {(0, 0)}
    while stack:
        cell = stack[-1]
        choices = [n for n in _neighbors(cell, rows, cols) if n not in visited]
        if not choices:
            stack.pop()
            continue
        nxt = choices[int(rng.integers(0, len(choices)))]
        open_edges.add(_edge(cell, nxt))
        visited.add(nxt)
        stack.append(nxt)
    return open_edges


def _all_internal_edges(rows: int, cols: int) -> set[CellEdge]:
    edges = set()
    for r in range(rows):
        for c in range(cols):
            if r + 1 < rows:
                edges.add(_edge((r, c), (r + 1, c)))
            if c + 1 < cols:
                edges.add(_edge((r, c), (r, c + 1)))
    return edges


def _extra_open_edges(rows: int, cols: int, open_edges: set[CellEdge], count: int, rng: np.random.Generator) -> set[CellEdge]:
    closed = sorted(_all_internal_edges(rows, cols) - open_edges)
    rng.shuffle(closed)
    return set(closed[: min(count, len(closed))])


def _path_edges(rows: int, cols: int, open_edges: set[CellEdge], start: Cell, goal: Cell) -> list[CellEdge]:
    parent: dict[Cell, Cell | None] = {start: None}
    queue = [start]
    for cell in queue:
        if cell == goal:
            break
        for nxt in _neighbors(cell, rows, cols):
            if nxt not in parent and _edge(cell, nxt) in open_edges:
                parent[nxt] = cell
                queue.append(nxt)
    path: list[CellEdge] = []
    cur = goal
    while parent[cur] is not None:
        prev = parent[cur]
        assert prev is not None
        path.append(_edge(prev, cur))
        cur = prev
    path.reverse()
    return path


def _choose_gate_edges(path_edges: list[CellEdge], open_edges: set[CellEdge], gate_range: tuple[int, int], rng: np.random.Generator) -> set[CellEdge]:
    gate_count = int(rng.integers(gate_range[0], gate_range[1] + 1))
    ordered = list(path_edges)
    rng.shuffle(ordered)
    if len(ordered) < gate_count:
        rest = sorted(open_edges - set(ordered))
        rng.shuffle(rest)
        ordered.extend(rest)
    return set(ordered[:gate_count])


def _grid_geometry(S: float, rows: int, cols: int) -> dict[str, float]:
    margin = max(0.70, 0.055 * S)
    return {
        "S": S,
        "margin": margin,
        "cell_w": (S - 2.0 * margin) / cols,
        "cell_h": (S - 2.0 * margin) / rows,
    }


def _cell_center(cell: Cell, geom: dict[str, float]) -> np.ndarray:
    r, c = cell
    x = geom["margin"] + (c + 0.5) * geom["cell_w"]
    y = geom["margin"] + (r + 0.5) * geom["cell_h"]
    return np.array([x, y], dtype=np.float32)


def _edge_segment(edge: CellEdge, geom: dict[str, float]) -> tuple[Orientation, float, tuple[float, float], np.ndarray]:
    (r1, c1), (r2, c2) = edge
    m, cw, ch = geom["margin"], geom["cell_w"], geom["cell_h"]
    if r1 == r2:
        x = m + max(c1, c2) * cw
        span = (m + r1 * ch, m + (r1 + 1) * ch)
        center = np.array([x, 0.5 * (span[0] + span[1])], dtype=np.float32)
        return "vertical", float(x), span, center
    y = m + max(r1, r2) * ch
    span = (m + c1 * cw, m + (c1 + 1) * cw)
    center = np.array([0.5 * (span[0] + span[1]), y], dtype=np.float32)
    return "horizontal", float(y), span, center


def _segments_and_gates(
    stage_name: str,
    split: SplitName,
    rows: int,
    cols: int,
    open_edges: set[CellEdge],
    gate_edges: set[CellEdge],
    geom: dict[str, float],
    wall_thickness: float,
    rng: np.random.Generator,
) -> tuple[list[WallSegment], list[Gate]]:
    closed_edges = _all_internal_edges(rows, cols) - open_edges
    raw_segments = [_edge_segment(edge, geom)[:3] for edge in closed_edges]
    raw_segments.extend(_boundary_segments(rows, cols, geom))

    wall_segments = [
        WallSegment(i, orientation, coord, span)
        for i, (orientation, coord, span) in enumerate(_merge_segments(raw_segments))
    ]

    gates: list[Gate] = []
    for edge in sorted(gate_edges):
        orientation, coord, span, center = _edge_segment(edge, geom)
        wall_id = len(wall_segments)
        wall_segments.append(WallSegment(wall_id, orientation, coord, span))
        slot_width = min(1.8, max(0.9, 0.72 * (span[1] - span[0])))
        gates.append(
            Gate(
                id=len(gates),
                wall_id=wall_id,
                cell_edge=edge,
                orientation=orientation,
                center=center,
                slot_width=float(slot_width),
                wall_thickness=wall_thickness,
                **_gate_dynamics(stage_name, split, rng),
            )
        )
    return wall_segments, gates


def _boundary_segments(rows: int, cols: int, geom: dict[str, float]) -> list[tuple[Orientation, float, tuple[float, float]]]:
    m, cw, ch = geom["margin"], geom["cell_w"], geom["cell_h"]
    x0, x1 = m, m + cols * cw
    y0, y1 = m, m + rows * ch
    return [
        ("vertical", x0, (y0, y1)),
        ("vertical", x1, (y0, y1)),
        ("horizontal", y0, (x0, x1)),
        ("horizontal", y1, (x0, x1)),
    ]


def _merge_segments(segments: list[tuple[Orientation, float, tuple[float, float]]]) -> list[tuple[Orientation, float, tuple[float, float]]]:
    # 只合并没有窗口的共线墙段，窗口所在墙段单独保留，方便碰撞判定。
    grouped: dict[tuple[Orientation, float], list[tuple[float, float]]] = {}
    for orientation, coord, span in segments:
        key = (orientation, round(coord, 6))
        grouped.setdefault(key, []).append(span)

    merged = []
    for (orientation, coord), spans in grouped.items():
        spans.sort()
        cur_lo, cur_hi = spans[0]
        for lo, hi in spans[1:]:
            if lo <= cur_hi + 1e-6:
                cur_hi = max(cur_hi, hi)
            else:
                merged.append((orientation, coord, (cur_lo, cur_hi)))
                cur_lo, cur_hi = lo, hi
        merged.append((orientation, coord, (cur_lo, cur_hi)))
    return merged


def _gate_dynamics(stage_name: str, split: SplitName, rng: np.random.Generator) -> dict:
    stage_name = canonical_stage_name(stage_name)
    if stage_name == "C1":
        return {
            "d_min": 1.4,
            "d_max": 1.4,
            "omega_d": 0.0,
            "phi_d": 0.0,
            "theta0": 0.0,
            "theta_amp": 0.0,
            "omega_theta": 0.0,
            "phi_theta": 0.0,
        }
    if stage_name == "C1_5":
        return {
            "d_min": 0.75,
            "d_max": float(rng.uniform(1.45, 1.70)),
            "omega_d": float(rng.uniform(0.25, 0.45)),
            "phi_d": float(rng.uniform(0.0, 2.0 * np.pi)),
            "theta0": 0.0,
            "theta_amp": 0.0,
            "omega_theta": 0.0,
            "phi_theta": 0.0,
        }
    if stage_name == "C2A":
        return {
            "d_min": float(rng.uniform(0.30, 0.45)),
            "d_max": float(rng.uniform(1.35, 1.65)),
            "omega_d": float(rng.uniform(0.35, 0.65)),
            "phi_d": float(rng.uniform(0.0, 2.0 * np.pi)),
            "theta0": 0.0,
            "theta_amp": 0.0,
            "omega_theta": 0.0,
            "phi_theta": 0.0,
        }

    ood = split == "ood_dynamics_test"
    theta_amp = 0.0
    omega_theta = 0.0
    phi_theta = 0.0
    if stage_name in {"C3", "C4", "C5"}:
        theta_amp = float(rng.uniform(0.40, 0.70) if ood else rng.uniform(0.15, 0.40))
        omega_theta = float(rng.uniform(0.80, 1.30) if ood else rng.uniform(0.35, 0.75))
        phi_theta = float(rng.uniform(0.0, 2.0 * np.pi))

    return {
        "d_min": float(rng.uniform(0.30, 0.50)),
        "d_max": float(rng.uniform(1.00, 1.35) if ood else rng.uniform(1.20, 1.60)),
        "omega_d": float(rng.uniform(0.90, 1.40) if ood else rng.uniform(0.45, 0.85)),
        "phi_d": float(rng.uniform(0.0, 2.0 * np.pi)),
        "theta0": 0.0,
        "theta_amp": theta_amp,
        "omega_theta": omega_theta,
        "phi_theta": phi_theta,
    }


def _build_wall_rects(wall_segments: list[WallSegment], gates: list[Gate], wall_thickness: float) -> list[dict[str, float]]:
    gates_by_wall = {gate.wall_id: gate for gate in gates}
    walls: list[dict[str, float]] = []
    for segment in wall_segments:
        gate = gates_by_wall.get(segment.id)
        if gate is None:
            walls.append(_segment_rect(segment, segment.span[0], segment.span[1], wall_thickness))
            continue
        lo, hi = gate.slot_axis_bounds
        if segment.span[0] < lo:
            walls.append(_segment_rect(segment, segment.span[0], lo, wall_thickness))
        if hi < segment.span[1]:
            walls.append(_segment_rect(segment, hi, segment.span[1], wall_thickness))
    return walls


def _segment_rect(segment: WallSegment, lo: float, hi: float, wall_thickness: float) -> dict[str, float]:
    half = 0.5 * wall_thickness
    if segment.orientation == "vertical":
        return {"xmin": segment.coord - half, "xmax": segment.coord + half, "ymin": lo, "ymax": hi}
    return {"xmin": lo, "xmax": hi, "ymin": segment.coord - half, "ymax": segment.coord + half}


def _nudge_if_blocked(start: np.ndarray, goal: np.ndarray, walls: list[dict[str, float]]) -> tuple[np.ndarray, np.ndarray]:
    radius = 0.25
    if any(circle_intersects_rect(start, radius, wall) for wall in walls):
        start = start + np.array([radius, -radius], dtype=np.float32)
    if any(circle_intersects_rect(goal, radius, wall) for wall in walls):
        goal = goal + np.array([-radius, radius], dtype=np.float32)
    return start.astype(np.float32), goal.astype(np.float32)

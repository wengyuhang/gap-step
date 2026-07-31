"""End-to-end MDG planner with coarse-to-fine DP and lazy edge repair."""

from __future__ import annotations

import time

from nonconvex_timevarying_window.sc_dynatogt.dynamics import DynamicLimits

from .backend_adapter import (
    optimize_selected_path,
    selected_constraints,
)
from .baselines import FREE_POINT_METHODS, normalize_method
from .config import MDGConfig
from .disk_tracking import build_scenario_tracks
from .dynamic_gate import Scenario
from .dynamic_programming import solve_layered_graph
from .models import PlanResult
from .time_graph import (
    build_layered_graph,
    competing_tracks,
)
from .trajectory_validation import validate_trajectory


class MDGPlanner:
    def __init__(self, config: MDGConfig | None = None) -> None:
        self.config = MDGConfig() if config is None else config

    def plan(
        self,
        scenario: Scenario,
        *,
        method: str = "mdg_free",
        disc_tracks=None,
        progress_callback=None,
    ) -> PlanResult:
        normalized = normalize_method(method)
        frontend_started = time.perf_counter()
        tracks = (
            build_scenario_tracks(scenario, self.config, method=normalized)
            if disc_tracks is None
            else disc_tracks
        )
        if any(not tracks.get(index) for index in scenario.order):
            return PlanResult(
                False,
                normalized,
                scenario.name,
                float("inf"),
                None,
                None,
                disc_tracks=tracks,
                failure_reason="a gate has no valid disc track",
            )
        v_max = DynamicLimits().max_velocity
        coarse_dt = 0.01 if normalized == "dense_oracle" else self.config.graph.dt_coarse
        coarse_graph = build_layered_graph(
            scenario,
            tracks,
            self.config,
            dt=coarse_dt,
            v_max=v_max,
            max_transition_lookback=0.5
            if normalized == "dense_oracle"
            else None,
        )
        coarse = solve_layered_graph(coarse_graph)
        if coarse is None:
            return PlanResult(
                False,
                normalized,
                scenario.name,
                float("inf"),
                None,
                None,
                disc_tracks=tracks,
                failure_reason="coarse graph has no path",
            )
        if normalized == "dense_oracle" or not self.config.graph.enable_refine:
            fine_graph = coarse_graph
            fine = coarse
        else:
            refinements = competing_tracks(
                scenario,
                tracks,
                coarse.selected_nodes,
                self.config.graph.refine_competing_tracks,
            )
            fine_graph = build_layered_graph(
                scenario,
                tracks,
                self.config,
                dt=self.config.graph.dt_fine,
                v_max=v_max,
                refinements=refinements,
                terminal_center=coarse.selected_nodes[-1].time,
            )
            fine = solve_layered_graph(fine_graph)
        if fine is None:
            return PlanResult(
                False,
                normalized,
                scenario.name,
                float("inf"),
                coarse,
                None,
                disc_tracks=tracks,
                failure_reason="fine graph has no path",
            )
        frontend_seconds = time.perf_counter() - frontend_started
        blocked: set[tuple[int, int]] = set()
        attempts = []
        free_points = normalized in FREE_POINT_METHODS
        latest_backend = None
        latest_validation = None
        for attempt in range(self.config.backend.max_lazy_repairs + 1):
            backend_started = time.perf_counter()
            latest_backend = optimize_selected_path(
                scenario,
                tracks,
                fine,
                self.config,
                free_points=free_points,
            )
            constraints = selected_constraints(
                scenario, tracks, fine, free_points=free_points
            )
            latest_validation = validate_trajectory(
                scenario, constraints, latest_backend, self.config
            )
            attempts.append(
                {
                    "attempt": attempt,
                    "backend_seconds": time.perf_counter() - backend_started,
                    "backend_success": latest_backend.success,
                    "validation_success": latest_validation.success,
                    "failure_reasons": latest_validation.failure_reasons,
                    "worst_segment": latest_validation.worst_segment,
                    "selected_node_ids": list(fine.selected_node_ids),
                    "frontend_seconds": frontend_seconds if attempt == 0 else 0.0,
                }
            )
            if progress_callback is not None:
                progress_callback(attempts[-1])
            if latest_validation.success:
                return PlanResult(
                    True,
                    normalized,
                    scenario.name,
                    latest_backend.total_time,
                    coarse,
                    fine,
                    backend=latest_backend,
                    validation=latest_validation,
                    disc_tracks=tracks,
                    lazy_attempts=attempts,
                )
            if attempt >= self.config.backend.max_lazy_repairs:
                break
            segment = min(
                latest_validation.worst_segment,
                len(fine.selected_node_ids) - 2,
            )
            edge = (
                fine.selected_node_ids[segment],
                fine.selected_node_ids[segment + 1],
            )
            source_node = fine.nodes[edge[0]]
            target_node = fine.nodes[edge[1]]
            if source_node.kind == "gate" and target_node.kind == "gate":
                # A single time-grid edge usually has near-identical temporal
                # neighbors.  Blocking the complete selected track pair makes
                # each repair try a genuinely different spatial corridor.
                for source_id in fine_graph.layers[segment]:
                    source = fine_graph.nodes[source_id]
                    if source.track_id != source_node.track_id:
                        continue
                    for target_id in fine_graph.layers[segment + 1]:
                        target = fine_graph.nodes[target_id]
                        if target.track_id == target_node.track_id:
                            blocked.add((source_id, target_id))
            else:
                blocked.add(edge)
            next_solution = solve_layered_graph(
                fine_graph, blocked_edges=blocked
            )
            if next_solution is None:
                break
            fine = next_solution
        failure = (
            "backend/validation failed after lazy repair"
            if latest_validation is None
            else "; ".join(latest_validation.failure_reasons)
        )
        return PlanResult(
            False,
            normalized,
            scenario.name,
            float("inf") if latest_backend is None else latest_backend.total_time,
            coarse,
            fine,
            backend=latest_backend,
            validation=latest_validation,
            disc_tracks=tracks,
            lazy_attempts=attempts,
            failure_reason=failure,
        )


__all__ = ["MDGPlanner"]

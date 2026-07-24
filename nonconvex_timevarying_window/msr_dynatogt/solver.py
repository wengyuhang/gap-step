"""MSR-DynaTOGT orchestration and A0--A3 ablations."""

from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Iterable

import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.environment import SCWindowTrack
from nonconvex_timevarying_window.sc_dynatogt.optimizer import optimize_track

from .candidate_pool import Candidate, CandidatePool
from .config import MSRConfig
from .feasibility_repair import check_feasibility, repair_candidate
from .initializations import InitialGuess, generate_initial_guesses


METHODS = {
    "A0": "SC-DynaTOGT",
    "A1": "SC + 多初值",
    "A2": "SC + 可行性修复",
    "A3": "MSR-DynaTOGT",
}


@dataclass(frozen=True)
class ProtocolSelection:
    method: str
    protocol: str
    candidate: Candidate
    candidate_count: int
    wall_clock_seconds: float
    iterations: int
    evaluations: int
    time_budget_seconds: float | None = None

    def to_run_row(self, *, scene: str, seed: int) -> dict[str, object]:
        row = self.candidate.to_run_row(
            method=self.method,
            scene=scene,
            seed=seed,
            candidate_count=self.candidate_count,
            comparison_protocol=self.protocol,
        )
        row["method_name"] = METHODS[self.method]
        row["wall_clock_seconds"] = self.wall_clock_seconds
        row["iterations"] = self.iterations
        row["evaluations"] = self.evaluations
        row["time_budget_seconds"] = (
            "" if self.time_budget_seconds is None else self.time_budget_seconds
        )
        return row


@dataclass(frozen=True)
class MSRSolution:
    track_name: str
    seed: int
    initializations: tuple[InitialGuess, ...]
    raw_candidates: tuple[Candidate, ...]
    repaired_candidates: tuple[Candidate, ...]
    selections: tuple[ProtocolSelection, ...]
    raw_duplicates_removed: int
    repaired_duplicates_removed: int
    wall_clock_seconds: float

    @property
    def best(self) -> Candidate:
        for selection in self.selections:
            if selection.method == "A3" and selection.protocol == "native":
                return selection.candidate
        raise RuntimeError("native A3 selection is missing")

    @property
    def native(self) -> dict[str, Candidate]:
        return {
            selection.method: selection.candidate
            for selection in self.selections
            if selection.protocol == "native"
        }

    def run_rows(self, *, scene: str | None = None) -> list[dict[str, object]]:
        label = self.track_name if scene is None else scene
        return [selection.to_run_row(scene=label, seed=self.seed) for selection in self.selections]


def _sc_choice(candidates: Iterable[Candidate]) -> Candidate:
    """Emulate SC selection: optimizer status/objective, without hard feasibility rank."""

    values = tuple(candidates)
    if not values:
        raise RuntimeError("cannot choose from an empty candidate set")
    return min(
        values,
        key=lambda candidate: (
            0 if candidate.result.success else 1,
            candidate.result.objective,
            candidate.result.total_time,
        ),
    )


def _pool_choice(candidates: Iterable[Candidate], config: MSRConfig) -> tuple[Candidate, int, int]:
    pool = CandidatePool(config.candidate_pool)
    for candidate in candidates:
        pool.add(candidate)
    return pool.best, len(pool), pool.duplicates_removed


def _totals(candidates: Iterable[Candidate]) -> tuple[float, int, int]:
    values = tuple(candidates)
    return (
        float(sum(candidate.wall_clock_seconds for candidate in values)),
        int(sum(candidate.iterations for candidate in values)),
        int(sum(candidate.evaluations for candidate in values)),
    )


def _selection(
    method: str,
    protocol: str,
    candidate: Candidate,
    attempted: Iterable[Candidate],
    *,
    candidate_count: int,
    time_budget: float | None = None,
) -> ProtocolSelection:
    elapsed, iterations, evaluations = _totals(attempted)
    return ProtocolSelection(
        method=method,
        protocol=protocol,
        candidate=candidate,
        candidate_count=candidate_count,
        wall_clock_seconds=elapsed,
        iterations=iterations,
        evaluations=evaluations,
        time_budget_seconds=time_budget,
    )


def _within_budget(candidates: tuple[Candidate, ...], budget: float) -> tuple[Candidate, ...]:
    selected: list[Candidate] = []
    elapsed = 0.0
    for candidate in candidates:
        next_elapsed = elapsed + candidate.wall_clock_seconds
        if selected and next_elapsed > budget:
            break
        selected.append(candidate)
        elapsed = next_elapsed
    return tuple(selected)


def solve(
    track: SCWindowTrack,
    *,
    config: MSRConfig | None = None,
    seed: int = 0,
) -> MSRSolution:
    """Run shared local solves once, then derive A0--A3 and fairness views."""

    settings = MSRConfig() if config is None else config
    started = time.perf_counter()
    guesses = tuple(
        generate_initial_guesses(
            track,
            settings.optimization,
            settings.initialization,
            seed=seed,
        )
    )

    raw: list[Candidate] = []
    for guess in guesses:
        local_started = time.perf_counter()
        result = optimize_track(
            track,
            config=settings.optimization,
            initial_x=guess.x,
        )
        elapsed = time.perf_counter() - local_started
        report = check_feasibility(track, result, settings)
        raw.append(
            Candidate(
                initialization=guess,
                raw_result=result,
                raw_feasibility=report,
                result=result,
                feasibility=report,
                optimization_seconds=elapsed,
            )
        )

    repaired: list[Candidate] = []
    for candidate in raw:
        repair_started = time.perf_counter()
        outcome = repair_candidate(
            track,
            candidate.result,
            settings,
            original_report=candidate.feasibility,
        )
        repair_elapsed = time.perf_counter() - repair_started
        repaired.append(
            Candidate(
                initialization=candidate.initialization,
                raw_result=candidate.raw_result,
                raw_feasibility=candidate.raw_feasibility,
                result=outcome.result,
                feasibility=outcome.feasibility,
                optimization_seconds=candidate.optimization_seconds,
                repair_seconds=repair_elapsed,
                repair=outcome,
            )
        )

    raw_values = tuple(raw)
    repaired_values = tuple(repaired)
    raw_best, raw_pool_count, raw_duplicates = _pool_choice(raw_values, settings)
    repaired_best, repaired_pool_count, repaired_duplicates = _pool_choice(
        repaired_values, settings
    )

    selections: list[ProtocolSelection] = [
        _selection("A0", "native", raw_values[0], raw_values[:1], candidate_count=1),
        _selection(
            "A1", "native", raw_best, raw_values, candidate_count=raw_pool_count
        ),
        _selection(
            "A2", "native", repaired_values[0], repaired_values[:1], candidate_count=1
        ),
        _selection(
            "A3",
            "native",
            repaired_best,
            repaired_values,
            candidate_count=repaired_pool_count,
        ),
    ]

    # Fair comparison 1: identical local-optimizer launch count and starts.
    matched_raw_sc = _sc_choice(raw_values)
    matched_repaired_sc = _sc_choice(repaired_values)
    selections.extend(
        (
            _selection(
                "A0",
                "matched_starts",
                matched_raw_sc,
                raw_values,
                candidate_count=len(raw_values),
            ),
            _selection(
                "A1",
                "matched_starts",
                raw_best,
                raw_values,
                candidate_count=raw_pool_count,
            ),
            _selection(
                "A2",
                "matched_starts",
                matched_repaired_sc,
                repaired_values,
                candidate_count=len(repaired_values),
            ),
            _selection(
                "A3",
                "matched_starts",
                repaired_best,
                repaired_values,
                candidate_count=repaired_pool_count,
            ),
        )
    )

    # Fair comparison 2: replay measured launches under one shared wall-time
    # budget.  The first launch is always admitted, so slow methods are not
    # represented by an empty result.  This reuses the same measured runs and
    # does not fabricate timings.
    budget = max(
        raw_values[0].wall_clock_seconds,
        repaired_values[0].wall_clock_seconds,
    )
    budget_raw = _within_budget(raw_values, budget)
    budget_repaired = _within_budget(repaired_values, budget)
    budget_raw_best, budget_raw_count, _ = _pool_choice(budget_raw, settings)
    budget_repaired_best, budget_repaired_count, _ = _pool_choice(
        budget_repaired, settings
    )
    selections.extend(
        (
            _selection(
                "A0",
                "matched_time",
                _sc_choice(budget_raw),
                budget_raw,
                candidate_count=len(budget_raw),
                time_budget=budget,
            ),
            _selection(
                "A1",
                "matched_time",
                budget_raw_best,
                budget_raw,
                candidate_count=budget_raw_count,
                time_budget=budget,
            ),
            _selection(
                "A2",
                "matched_time",
                _sc_choice(budget_repaired),
                budget_repaired,
                candidate_count=len(budget_repaired),
                time_budget=budget,
            ),
            _selection(
                "A3",
                "matched_time",
                budget_repaired_best,
                budget_repaired,
                candidate_count=budget_repaired_count,
                time_budget=budget,
            ),
        )
    )

    return MSRSolution(
        track_name=track.name,
        seed=seed,
        initializations=guesses,
        raw_candidates=raw_values,
        repaired_candidates=repaired_values,
        selections=tuple(selections),
        raw_duplicates_removed=raw_duplicates,
        repaired_duplicates_removed=repaired_duplicates,
        wall_clock_seconds=time.perf_counter() - started,
    )


__all__ = ["METHODS", "MSRSolution", "ProtocolSelection", "solve"]

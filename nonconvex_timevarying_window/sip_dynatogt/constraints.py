"""Smooth point constraints used by SLSQP and counterexample generation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable
import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.dynamics import flatness_map
from nonconvex_timevarying_window.sc_dynatogt.collision import whole_body_clearance_residual
from .model import PolynomialTrajectory, SIPConfig, SIPProblem, Witness


@dataclass(frozen=True)
class PointFlatness:
    position: np.ndarray
    velocity: np.ndarray
    rotation: np.ndarray
    body_rate: np.ndarray
    collective_thrust: float
    rotor_thrusts: np.ndarray
    specific_force_norm2: float
    heading_cross_norm2: float


def point_singularity_residual_values(trajectory: PolynomialTrajectory, segment: int, tau: float, config: SIPConfig) -> dict[str, float]:
    """Evaluate the two flatness denominators without normalizing either one."""

    local = float(tau) * trajectory.durations[segment]
    acceleration = trajectory.evaluate_segment(segment, local, 2)
    force = acceleration + np.array([0.0, 0.0, config.quadrotor.gravity])
    force2 = float(force @ force)
    if force2 <= 0.0:
        cross2 = 0.0
    else:
        body_z = force / np.sqrt(force2)
        raw = np.cross(np.array([0.0, 1.0, 0.0]), body_z)
        cross2 = float(raw @ raw)
    floor2 = config.flatness_floor**2
    return {
        "specific_force_singularity": floor2 - force2,
        "heading_cross_singularity": floor2 - cross2,
    }


def point_flatness(trajectory: PolynomialTrajectory, segment: int, tau: float, config: SIPConfig) -> PointFlatness:
    local = float(tau) * trajectory.durations[segment]
    p = trajectory.evaluate_segment(segment, local, 0)
    v = trajectory.evaluate_segment(segment, local, 1)
    a = trajectory.evaluate_segment(segment, local, 2)
    j = trajectory.evaluate_segment(segment, local, 3)
    s = trajectory.evaluate_segment(segment, local, 4)
    force = a + np.array([0.0, 0.0, config.quadrotor.gravity])
    force2 = float(force @ force)
    if force2 <= 0: raise ValueError("specific-force singularity")
    b3 = force / np.sqrt(force2)
    raw = np.cross(np.array([0.0, 1.0, 0.0]), b3)
    cross2 = float(raw @ raw)
    state = flatness_map(a, j, s, parameters=config.quadrotor)
    return PointFlatness(p, v, np.asarray(state.rotation), np.asarray(state.body_rate), float(state.collective_thrust), np.asarray(state.rotor_thrusts), force2, cross2)


def global_time(trajectory: PolynomialTrajectory, segment: int, tau: float) -> float:
    return float(np.sum(trajectory.durations[:segment]) + tau * trajectory.durations[segment])


def safety_residual_value(problem: SIPProblem, trajectory: PolynomialTrajectory, witness: Witness, config: SIPConfig, *, planning: bool = False) -> float:
    if witness.window_index is None or witness.boundary_segment is None or witness.boundary_parameter is None:
        raise ValueError("incomplete safety witness")
    flat = point_flatness(trajectory, witness.trajectory_segment, witness.normalized_time, config)
    t = global_time(trajectory, witness.trajectory_segment, witness.normalized_time)
    window = problem.windows[witness.window_index]
    return _safety_residual_from_components(
        window,
        witness.boundary_segment,
        witness.boundary_parameter,
        flat,
        window.state_at(t),
        config,
        planning=planning,
    )


def _safety_residual_from_components(
    window,
    boundary_segment: int,
    boundary_parameter: float,
    flat: PointFlatness,
    window_state,
    config: SIPConfig,
    *,
    planning: bool,
) -> float:
    center, rotation, scale = window_state
    q = np.asarray(window.boundary[boundary_segment].evaluate(boundary_parameter))
    y = center + rotation @ np.array([scale * q[0], scale * q[1], 0.0])
    clearance = config.planning_clearance if planning else config.clearance
    return float(
        whole_body_clearance_residual(
            y, flat.position, flat.rotation, config.body, clearance
        )
    )


def _bounds(config: SIPConfig, planning: bool) -> dict[str, float]:
    l, g = config.dynamic_limits, config.dynamic_guard_fraction if planning else 0.0
    rotor_width = l.max_rotor_thrust - l.min_rotor_thrust
    result = {"v": l.max_velocity * (1-g), "xy": l.max_body_rate_xy * (1-g), "z": l.max_body_rate_z * (1-g), "rmin": l.min_rotor_thrust + g*rotor_width, "rmax": l.max_rotor_thrust - g*rotor_width, "cmin": l.min_collective_thrust, "cmax": l.max_collective_thrust}
    if np.isfinite(l.max_collective_thrust):
        width = l.max_collective_thrust-l.min_collective_thrust
        result["cmin"] += g*width; result["cmax"] -= g*width
    return result


def dynamic_residual_values(trajectory: PolynomialTrajectory, segment: int, tau: float, config: SIPConfig, *, planning: bool = False) -> dict[str, float]:
    f = point_flatness(trajectory, segment, tau, config)
    return _dynamic_residuals_from_flatness(f, config, planning=planning)


def _dynamic_residuals_from_flatness(
    f: PointFlatness,
    config: SIPConfig,
    *,
    planning: bool,
) -> dict[str, float]:
    b = _bounds(config, planning)
    r = {"velocity": float(f.velocity@f.velocity-b["v"]**2), "specific_force_singularity": config.flatness_floor**2-f.specific_force_norm2, "heading_cross_singularity": config.flatness_floor**2-f.heading_cross_norm2, "collective_lower": b["cmin"]-f.collective_thrust, "body_rate_xy": float(f.body_rate[0]**2+f.body_rate[1]**2-b["xy"]**2), "body_rate_z": float(f.body_rate[2]**2-b["z"]**2)}
    if np.isfinite(b["cmax"]): r["collective_upper"] = f.collective_thrust-b["cmax"]
    for i, thrust in enumerate(f.rotor_thrusts):
        r[f"rotor_{i}_lower"] = b["rmin"]-float(thrust); r[f"rotor_{i}_upper"] = float(thrust)-b["rmax"]
    return r


def dynamic_constraint_names(config: SIPConfig) -> tuple[str, ...]:
    names = ["velocity", "specific_force_singularity", "heading_cross_singularity", "collective_lower", "body_rate_xy", "body_rate_z"]
    if np.isfinite(config.dynamic_limits.max_collective_thrust): names.append("collective_upper")
    for i in range(4): names.extend((f"rotor_{i}_lower", f"rotor_{i}_upper"))
    return tuple(names)


def _dynamic_scale(kind: str, config: SIPConfig) -> float:
    limits = config.dynamic_limits
    if kind == "velocity": return max(1.0, limits.max_velocity**2)
    if kind == "body_rate_xy": return max(1.0, limits.max_body_rate_xy**2)
    if kind == "body_rate_z": return max(1.0, limits.max_body_rate_z**2)
    if kind.startswith("rotor_"): return max(1.0, limits.max_rotor_thrust-limits.min_rotor_thrust)
    if kind.startswith("collective_") and np.isfinite(limits.max_collective_thrust): return max(1.0, limits.max_collective_thrust-limits.min_collective_thrust)
    return 1.0


def initial_witnesses(problem: SIPProblem, segments: int, config: SIPConfig) -> tuple[Witness, ...]:
    result = []
    for k in range(segments):
        result.extend(Witness("dynamic", k, tau, 0.0, source="initial") for tau in config.initial_nodes)
        for wi, window in enumerate(problem.windows):
            for bi in range(len(window.boundary)):
                for tau in config.initial_nodes:
                    for u in config.initial_nodes: result.append(Witness("safety", k, tau, 0.0, wi, bi, u, "initial"))
    return tuple(result)


def witness_constraint_values(problem: SIPProblem, trajectory: PolynomialTrajectory, witnesses: Iterable[Witness], config: SIPConfig) -> np.ndarray:
    values: list[float] = []
    residual_cache: dict[tuple[int,float],dict[str,float]] = {}
    flat_cache: dict[tuple[int,float],PointFlatness] = {}
    window_cache: dict[tuple[int,int,float],tuple[np.ndarray,np.ndarray,float]] = {}
    for w in witnesses:
        try:
            key = (w.trajectory_segment, w.normalized_time)
            if key not in flat_cache:
                flat_cache[key] = point_flatness(trajectory, *key, config)
            if w.kind == "safety":
                if w.window_index is None or w.boundary_segment is None or w.boundary_parameter is None:
                    raise ValueError("incomplete safety witness")
                scale=max(config.planning_clearance**2,1e-4)
                instant = global_time(trajectory, *key)
                state_key = (w.window_index, w.trajectory_segment, w.normalized_time)
                if state_key not in window_cache:
                    window_cache[state_key] = problem.windows[w.window_index].state_at(instant)
                residual = _safety_residual_from_components(
                    problem.windows[w.window_index],
                    w.boundary_segment,
                    w.boundary_parameter,
                    flat_cache[key],
                    window_cache[state_key],
                    config,
                    planning=True,
                )
                values.append(-residual/scale)
                continue
            if key not in residual_cache:
                residual_cache[key] = _dynamic_residuals_from_flatness(
                    flat_cache[key], config, planning=True
                )
            residuals = residual_cache[key]
            if w.kind == "dynamic": values.extend(-v/_dynamic_scale(kind,config) for kind,v in residuals.items())
            else: values.append(-residuals[w.kind]/_dynamic_scale(w.kind,config))
        except (ValueError, FloatingPointError, np.linalg.LinAlgError, KeyError):
            values.extend([-1e12] * (len(dynamic_constraint_names(config)) if w.kind == "dynamic" else 1))
    return np.asarray(values)


__all__ = ["dynamic_constraint_names", "dynamic_residual_values", "global_time", "initial_witnesses", "point_flatness", "point_singularity_residual_values", "safety_residual_value", "witness_constraint_values"]

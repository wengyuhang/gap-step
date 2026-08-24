from __future__ import annotations

import json
import numpy as np
import pytest

from nonconvex_timevarying_window.sc_dynatogt.dynamics import DynamicLimits
from nonconvex_timevarying_window.sc_dynatogt.environment import MotionProfile, SCDynamicWindow, SCWindowTrack
from nonconvex_timevarying_window.sc_dynatogt.sc_mapping import SCDiskMap
from nonconvex_timevarying_window.sip_dynatogt import (
    CertificateStatus,
    SIPConfig,
    SIPProblem,
    load_run,
    save_run,
    solve,
)
from nonconvex_timevarying_window.sip_dynatogt.certificate import certify
from nonconvex_timevarying_window.sip_dynatogt.verify import main as verify_main


def _track(dynamic: bool):
    physical = np.array([[-2.0, -2.0], [2.0, -2.0], [2.0, 2.0], [-2.0, 2.0]])
    safe = np.array([[-1.5, -1.5], [1.5, -1.5], [1.5, 1.5], [-1.5, 1.5]])
    mapping = SCDiskMap.fit(safe, quadrature_order=32, max_nfev=300)
    motion = (
        MotionProfile(
            np.array([0.0, 0.05, 0.03]),
            np.zeros(3),
            0.0,
            translation_period=5.0,
            rotation_enabled=False,
            scale_enabled=False,
        )
        if dynamic else MotionProfile.static()
    )
    window = SCDynamicWindow(
        "square", mapping, safe, np.zeros(3), np.array([0.0, np.pi / 2, 0.0]), motion, physical
    )
    return SCWindowTrack(
        "dynamic" if dynamic else "static",
        np.array([-3.0, 0.0, 0.0]),
        np.array([3.0, 0.0, 0.0]),
        (window,),
        (0,),
    )


def _config():
    limits = DynamicLimits(
        max_velocity=100.0,
        max_body_rate_xy=100.0,
        max_body_rate_z=100.0,
        min_rotor_thrust=-100.0,
        max_rotor_thrust=100.0,
    )
    return SIPConfig(
        dynamic_limits=limits,
        slsqp_max_iterations=1,
        max_exchange_iterations=1,
        precision_bits=(128,),
        max_cells=100_000,
        max_depth=20,
    )


@pytest.mark.parametrize("dynamic", [False, True])
def test_static_and_dynamic_end_to_end_candidates_are_certified(dynamic):
    problem = SIPProblem.from_track(_track(dynamic))
    result = solve(problem, _config())
    assert result.status is CertificateStatus.CERTIFIED_FEASIBLE
    assert result.success
    assert result.certificate.checked_cells > 0
    np.testing.assert_allclose(
        result.trajectory.evaluate(result.traversal_times[0]), result.waypoints[0], atol=2e-8
    )


def test_saved_run_replays_the_same_certificate(tmp_path):
    problem = SIPProblem.from_track(_track(False))
    config = _config()
    result = solve(problem, config)
    run = save_run(tmp_path / "run", problem, config, result)
    replay_problem, replay_config, replay_trajectory, stored = load_run(run)
    replay = certify(replay_problem, replay_trajectory, replay_config)
    assert replay.status.value == stored["status"] == "CERTIFIED_FEASIBLE"
    assert verify_main(["--run", str(run)]) == 0
    verification = json.loads((run / "verification.json").read_text(encoding="utf-8"))
    assert verification["status_matches"] is True

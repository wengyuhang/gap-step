"""The single fail-closed execution gate used by the simplified algorithm."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from nonconvex_timevarying_window.sip_dynatogt.certificate import _at_precision, _coarse
from nonconvex_timevarying_window.sip_dynatogt.intervals import (
    IntervalDependencyError,
    exact_ball,
    require_flint,
    window_state_interval,
)
from nonconvex_timevarying_window.sip_dynatogt.model import (
    CertificateResult,
    CertificateStatus,
    SIPConfig,
    SIPProblem,
)

from .planner import PlanProposal


@dataclass(frozen=True)
class AdmissionResult:
    accepted: bool
    proposal: PlanProposal
    certificate: CertificateResult


CertificateFunction = Callable[[SIPProblem, object, SIPConfig | None], CertificateResult]


@dataclass(frozen=True)
class _ShiftedWindow:
    base: object
    offset: float

    @property
    def name(self):
        return self.base.name

    @property
    def boundary(self):
        return self.base.boundary

    @property
    def motion(self):
        return self.base.motion

    def state_at(self, local_time: float):
        return self.base.state_at(float(local_time) + self.offset)

    def state_interval(self, local_time):
        return window_state_interval(self.base, local_time + exact_ball(self.offset))


def _shifted_problem(problem: SIPProblem, offset: float) -> SIPProblem:
    if offset == 0.0:
        return problem
    windows = tuple(_ShiftedWindow(window, float(offset)) for window in problem.windows)
    return SIPProblem(f"{problem.name}@{offset:.9g}", windows, problem.order)


def _certify_any_piece_count(
    problem: SIPProblem, trajectory: object, config: SIPConfig | None
) -> CertificateResult:
    """Use the strict continuous checker without SIP's N+1 solver convention."""

    settings = config or SIPConfig()
    try:
        require_flint()
        coarse = _coarse(problem, trajectory, settings)
    except (IntervalDependencyError, TypeError, ValueError) as error:
        return CertificateResult(
            CertificateStatus.NUMERICAL_FAILURE, str(error), 0, 0, 0, None, None
        )
    except Exception as error:
        return CertificateResult(
            CertificateStatus.NUMERICAL_FAILURE,
            f"coarse evaluation failed closed: {type(error).__name__}: {error}",
            0,
            0,
            0,
            None,
            None,
        )
    if coarse:
        return CertificateResult(
            CertificateStatus.VIOLATED,
            "finite points expose violations; no safety claim is made",
            0,
            0,
            0,
            None,
            None,
            coarse,
        )
    last = None
    for bits in settings.precision_bits:
        try:
            last = _at_precision(problem, trajectory, settings, int(bits))
        except Exception as error:
            last = CertificateResult(
                CertificateStatus.NUMERICAL_FAILURE,
                f"interval evaluation failed closed: {type(error).__name__}: {error}",
                int(bits),
                0,
                0,
                None,
                None,
            )
        if last.status in (
            CertificateStatus.CERTIFIED_FEASIBLE,
            CertificateStatus.VIOLATED,
        ):
            return last
    assert last is not None
    return last


def admit(
    problem: SIPProblem,
    proposal: PlanProposal,
    config: SIPConfig | None = None,
    *,
    certificate_function: CertificateFunction | None = None,
) -> AdmissionResult:
    """Accept exactly and only a rigorously certified candidate."""

    local_problem = _shifted_problem(problem, proposal.start_time)
    checker = certificate_function or _certify_any_piece_count
    certificate = checker(local_problem, proposal.trajectory, config)
    return AdmissionResult(certificate.certified, proposal, certificate)


class SafePlanManager:
    """Atomically retain the last certified plan when a proposal is rejected."""

    def __init__(self) -> None:
        self._current: PlanProposal | None = None
        self._last_certificate: CertificateResult | None = None

    @property
    def current(self) -> PlanProposal | None:
        return self._current

    @property
    def last_certificate(self) -> CertificateResult | None:
        return self._last_certificate

    def submit(self, result: AdmissionResult) -> bool:
        self._last_certificate = result.certificate
        if not result.accepted:
            return False
        self._current = result.proposal
        return True

    def require_takeoff_plan(self) -> PlanProposal:
        if self._current is None:
            raise RuntimeError("takeoff forbidden: no certified plan is installed")
        return self._current

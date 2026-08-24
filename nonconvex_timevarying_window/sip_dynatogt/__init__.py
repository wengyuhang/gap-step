"""Certified semi-infinite-programming extension of SC-DynaTOGT."""

from .certificate import certify
from .io import load_run, save_run
from .model import (
    CertificateResult,
    CertificateStatus,
    CuboidBody,
    PolynomialTrajectory,
    SIPConfig,
    SIPProblem,
    SIPResult,
    SIPWindow,
    Witness,
)
from .solver import solve

__all__ = [
    "CertificateResult",
    "CertificateStatus",
    "CuboidBody",
    "PolynomialTrajectory",
    "SIPConfig",
    "SIPProblem",
    "SIPResult",
    "SIPWindow",
    "Witness",
    "certify",
    "load_run",
    "save_run",
    "solve",
]

"""Portable, pickle-free run artifacts for SIP-DynaTOGT."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from .model import (
    PolynomialTrajectory,
    SIPConfig,
    SIPProblem,
    SIPResult,
    problem_from_dict,
    problem_to_dict,
)


FORMAT_VERSION = 1


def _json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return "Infinity" if value > 0.0 else "-Infinity" if value < 0.0 else "NaN"
    return value


def _canonical_json(data: Any) -> bytes:
    return json.dumps(data, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def save_run(
    directory: str | Path,
    problem: SIPProblem,
    config: SIPConfig,
    result: SIPResult,
) -> Path:
    root = Path(directory).expanduser()
    root.mkdir(parents=True, exist_ok=True)
    problem_data = problem_to_dict(problem)
    config_data = config.to_dict()
    problem_bytes = _canonical_json(problem_data)
    config_bytes = _canonical_json(config_data)
    (root / "problem.json").write_text(
        json.dumps(problem_data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    (root / "config.json").write_text(
        json.dumps(config_data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    np.savez_compressed(
        root / "candidate.npz",
        durations=result.trajectory.durations,
        coefficients=result.trajectory.coefficients,
        x=result.x,
        traversal_times=result.traversal_times,
        waypoints=result.waypoints,
    )
    result_data = _json_safe(result.to_dict())
    (root / "certificate.json").write_text(
        json.dumps(result_data, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    candidate_hash = hashlib.sha256((root / "candidate.npz").read_bytes()).hexdigest()
    manifest = {
        "format_version": FORMAT_VERSION,
        "algorithm": "SIP-DynaTOGT",
        "problem_sha256": _sha256(problem_bytes),
        "config_sha256": _sha256(config_bytes),
        "candidate_sha256": candidate_hash,
        "status": result.status.value,
        "files": {
            "problem": "problem.json",
            "config": "config.json",
            "candidate": "candidate.npz",
            "certificate": "certificate.json",
        },
    }
    (root / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return root


def load_run(
    directory: str | Path,
) -> tuple[SIPProblem, SIPConfig, PolynomialTrajectory, dict[str, Any]]:
    root = Path(directory).expanduser()
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    if int(manifest.get("format_version", -1)) != FORMAT_VERSION:
        raise ValueError("unsupported SIP-DynaTOGT run format")
    if manifest.get("algorithm") != "SIP-DynaTOGT":
        raise ValueError("run manifest names a different algorithm")
    files = manifest["files"]
    problem_path = root / files["problem"]
    config_path = root / files["config"]
    candidate_path = root / files["candidate"]
    problem_data = json.loads(problem_path.read_text(encoding="utf-8"))
    config_data = json.loads(config_path.read_text(encoding="utf-8"))
    if _sha256(_canonical_json(problem_data)) != manifest["problem_sha256"]:
        raise ValueError("problem.json hash mismatch")
    if _sha256(_canonical_json(config_data)) != manifest["config_sha256"]:
        raise ValueError("config.json hash mismatch")
    if hashlib.sha256(candidate_path.read_bytes()).hexdigest() != manifest["candidate_sha256"]:
        raise ValueError("candidate.npz hash mismatch")
    with np.load(candidate_path, allow_pickle=False) as arrays:
        trajectory = PolynomialTrajectory(
            np.asarray(arrays["durations"], dtype=float),
            np.asarray(arrays["coefficients"], dtype=float),
        )
    stored = json.loads((root / files["certificate"]).read_text(encoding="utf-8"))
    if stored.get("status") != manifest.get("status"):
        raise ValueError("certificate and manifest status disagree")
    return problem_from_dict(problem_data), SIPConfig.from_dict(config_data), trajectory, stored


__all__ = ["FORMAT_VERSION", "load_run", "save_run"]

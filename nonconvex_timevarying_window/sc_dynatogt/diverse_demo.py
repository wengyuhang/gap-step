"""Six-window polygon/smooth/mixed SC-DynaTOGT closed-loop demo."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .environment import SCWindowTrack
from .experiments import (
    ExperimentSettings,
    _designated_crossings_valid,
    _jsonable,
    _optimization_config,
    _preprocessing_config,
    _sampled_dynamic_limits_satisfied,
    _write_json,
)
from .optimizer import optimize_track
from .minco import BoundaryState, MincoSnap
from .results_manager import (
    RESULTS_ROOT,
    timestamped_run_directory,
    update_current_demo,
    write_run_manifest,
)
from .scenarios import PAPER_REFERENCE_GATE_ORDER, build_diverse_scenario
from .validation import validate_sc_mapping
from .visualization import (
    export_dynamic_window_gif,
    export_trajectory_csv,
    plot_crossing_grid,
    plot_preprocessing,
    plot_route_overview,
    plot_scale_profile,
)


def run_diverse_demo(
    output: str | Path,
    *,
    mode: str = "full",
    quality: str = "smoke",
    layout: str = "paper_irregular",
    motion_scale: float = 3.5,
    validation_samples: int = 1_000,
    make_gif: bool = True,
) -> dict[str, object]:
    """Optimize and export one ordered six-window diverse-shape trajectory."""

    if mode not in {"static", "translation", "full"}:
        raise ValueError("mode must be static, translation, or full")
    if quality not in {"smoke", "default"}:
        raise ValueError("quality must be smoke or default")
    if layout not in {"compact", "spacious", "paper_irregular"}:
        raise ValueError("layout must be compact, spacious, or paper_irregular")
    if validation_samples < 1:
        raise ValueError("validation_samples must be positive")

    root = Path(output).expanduser()
    root.mkdir(parents=True, exist_ok=True)
    settings = ExperimentSettings(suite=quality)
    scenario = build_diverse_scenario(
        mode=mode,
        preprocessing_config=_preprocessing_config(settings),
        layout=layout,
        motion_scale=motion_scale,
    )
    config = _optimization_config(settings)
    result = optimize_track(scenario.track, config=config)

    for index, gate in enumerate(scenario.preprocessed_gates):
        gate_root = root / "preprocessing" / f"{index:02d}_{gate.name}"
        gate.save(gate_root)
        plot_preprocessing(gate, gate_root / "preprocessing.png", samples_per_line=40)
    plot_route_overview(
        scenario.track, result, root / "figures/route_overview.png", num_samples=401
    )
    plot_crossing_grid(scenario.track, result, root / "figures/crossings_grid.png")
    plot_scale_profile(scenario.track, result, root / "figures/scale_profile.png")
    export_trajectory_csv(result, root / "data/trajectory.csv", num_samples=501)
    if make_gif:
        export_dynamic_window_gif(
            scenario.track,
            result,
            root / "media/dynamic_windows.gif",
            num_frames=72,
        )

    mapping = [
        _jsonable(
            validate_sc_mapping(
                window.sc_map,
                sample_count=validation_samples,
                seed=index,
                batch_size=1024,
            )
        )
        for index, window in enumerate(scenario.track.windows)
    ]
    legal = _designated_crossings_valid(scenario.track, result)
    centers = np.asarray([window.center0 for window in scenario.track.windows])
    route_points = np.vstack((scenario.track.start, centers, scenario.track.goal))
    payload: dict[str, object] = {
        "demo": "diverse_six_window",
        "mode": mode,
        "quality": quality,
        "layout": layout,
        "paper_reference_gate_order": (
            list(PAPER_REFERENCE_GATE_ORDER) if layout == "paper_irregular" else None
        ),
        "motion_scale": motion_scale,
        "window_names": [window.name for window in scenario.track.windows],
        "window_count": len(scenario.track.windows),
        "order": list(scenario.track.order),
        "start": scenario.track.start.tolist(),
        "goal": scenario.track.goal.tolist(),
        "closed_loop": bool(np.allclose(scenario.track.start, scenario.track.goal)),
        "window_center_bounds": {
            "minimum": centers.min(axis=0).tolist(),
            "maximum": centers.max(axis=0).tolist(),
        },
        "route_leg_distances": np.linalg.norm(np.diff(route_points, axis=0), axis=1).tolist(),
        "initial_window_poses": [
            {
                "name": window.name,
                "center": window.center0.tolist(),
                "angles_rpy": window.angles0.tolist(),
            }
            for window in scenario.track.windows
        ],
        "motion_amplitudes": [
            {
                "name": window.name,
                "translation": window.motion.translation_amplitude.tolist(),
                "rotation_rpy": window.motion.rotation_amplitude.tolist(),
                "uniform_scale": window.motion.scale_amplitude,
            }
            for window in scenario.track.windows
        ],
        "optimization_success": result.success,
        "designated_order_legal": legal,
        "sampled_dynamic_limits_satisfied": _sampled_dynamic_limits_satisfied(result, config),
        "mapping_validation": mapping,
        "result": result.to_dict(),
        "passed": bool(result.success and legal and all(item["passed"] for item in mapping)),
    }
    _write_json(root / "summary.json", payload)
    write_run_manifest(
        root,
        run_id=root.name,
        kind="demo",
        role="run",
        featured=False,
    )
    return _jsonable(payload)


def load_diverse_demo(summary_path: str | Path) -> tuple[SCWindowTrack, MincoSnap]:
    """Reconstruct a saved diverse-demo track and MINCO trajectory."""

    source = Path(summary_path).expanduser()
    payload = json.loads(source.read_text(encoding="utf-8"))
    result = payload["result"]
    scenario = build_diverse_scenario(
        mode=str(payload["mode"]),
        preprocessing_config=_preprocessing_config(
            ExperimentSettings(suite=str(payload["quality"]))
        ),
        layout=str(payload["layout"]),
        motion_scale=float(payload["motion_scale"]),
    )
    trajectory = MincoSnap(
        BoundaryState(scenario.track.start),
        BoundaryState(scenario.track.goal),
        np.asarray(result["waypoints"], dtype=float),
        np.asarray(result["durations"], dtype=float),
    )
    return scenario.track, trajectory


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run a six-window polygon/smooth/mixed SC-DynaTOGT closed-loop demo"
    )
    parser.add_argument(
        "--outdir", type=Path,
        help="exact run directory; omitted paths are timestamped below results/demos/runs",
    )
    parser.add_argument("--mode", choices=("static", "translation", "full"), default="full")
    parser.add_argument("--quality", choices=("smoke", "default"), default="smoke")
    parser.add_argument(
        "--layout",
        choices=("compact", "spacious", "paper_irregular"),
        default="paper_irregular",
        help="paper_irregular is the new closed-loop default; spacious and compact preserve old layouts",
    )
    parser.add_argument(
        "--motion-scale",
        type=float,
        default=3.5,
        help="multiplier for translation, rotation, and scale amplitudes",
    )
    parser.add_argument("--validation-samples", type=int, default=1_000)
    parser.add_argument("--no-gif", action="store_true")
    args = parser.parse_args(argv)
    output = args.outdir or timestamped_run_directory(
        RESULTS_ROOT,
        "demos/runs",
        f"{args.layout}_{args.mode}",
    )
    payload = run_diverse_demo(
        output,
        mode=args.mode,
        quality=args.quality,
        layout=args.layout,
        motion_scale=args.motion_scale,
        validation_samples=args.validation_samples,
        make_gif=not args.no_gif,
    )
    if payload["passed"]:
        try:
            output.resolve().relative_to(RESULTS_ROOT.resolve())
        except ValueError:
            pass
        else:
            update_current_demo(RESULTS_ROOT, output)
    print(json.dumps(_jsonable({
        "outdir": output,
        "windows": payload["window_names"],
        "passed": payload["passed"],
    }), ensure_ascii=False))
    return 0 if payload["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["load_diverse_demo", "main", "run_diverse_demo"]

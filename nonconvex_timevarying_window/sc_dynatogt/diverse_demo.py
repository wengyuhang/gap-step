"""Six-window polygon/smooth/mixed SC-DynaTOGT visualization demo."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

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
from .scenarios import build_diverse_scenario
from .validation import validate_sc_mapping
from .visualization import (
    export_dynamic_window_gif,
    export_trajectory_csv,
    plot_preprocessing,
    plot_trajectory,
)


def run_diverse_demo(
    output: str | Path,
    *,
    mode: str = "full",
    quality: str = "smoke",
    layout: str = "spacious",
    motion_scale: float = 2.5,
    validation_samples: int = 1_000,
    make_gif: bool = True,
) -> dict[str, object]:
    """Optimize and export one ordered six-window diverse-shape trajectory."""

    if mode not in {"static", "translation", "full"}:
        raise ValueError("mode must be static, translation, or full")
    if quality not in {"smoke", "default"}:
        raise ValueError("quality must be smoke or default")
    if layout not in {"compact", "spacious"}:
        raise ValueError("layout must be compact or spacious")
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
        gate_root = root / "preprocessed_gates" / f"{index:02d}_{gate.name}"
        gate.save(gate_root)
        plot_preprocessing(gate, gate_root / "preprocessing.png", samples_per_line=40)
    plot_trajectory(scenario.track, result, root / "trajectory.png", num_samples=401)
    export_trajectory_csv(result, root / "trajectory.csv", num_samples=501)
    if make_gif:
        export_dynamic_window_gif(
            scenario.track,
            result,
            root / "dynamic_windows.gif",
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
    payload: dict[str, object] = {
        "demo": "diverse_six_window",
        "mode": mode,
        "quality": quality,
        "layout": layout,
        "motion_scale": motion_scale,
        "window_names": [window.name for window in scenario.track.windows],
        "window_count": len(scenario.track.windows),
        "order": list(scenario.track.order),
        "start": scenario.track.start.tolist(),
        "goal": scenario.track.goal.tolist(),
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
    return _jsonable(payload)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run a six-window polygon/smooth/mixed SC-DynaTOGT demo"
    )
    parser.add_argument("--outdir", type=Path, default=Path(
        "nonconvex_timevarying_window/sc_dynatogt/results/diverse_demo"
    ))
    parser.add_argument("--mode", choices=("static", "translation", "full"), default="full")
    parser.add_argument("--quality", choices=("smoke", "default"), default="smoke")
    parser.add_argument(
        "--layout",
        choices=("compact", "spacious"),
        default="spacious",
        help="spacious distributes centres over x/y/z; compact uses the old x-axis layout",
    )
    parser.add_argument(
        "--motion-scale",
        type=float,
        default=2.5,
        help="multiplier for translation, rotation, and scale amplitudes",
    )
    parser.add_argument("--validation-samples", type=int, default=1_000)
    parser.add_argument("--no-gif", action="store_true")
    args = parser.parse_args(argv)
    payload = run_diverse_demo(
        args.outdir,
        mode=args.mode,
        quality=args.quality,
        layout=args.layout,
        motion_scale=args.motion_scale,
        validation_samples=args.validation_samples,
        make_gif=not args.no_gif,
    )
    print(json.dumps(_jsonable({
        "outdir": args.outdir,
        "windows": payload["window_names"],
        "passed": payload["passed"],
    }), ensure_ascii=False))
    return 0 if payload["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["main", "run_diverse_demo"]

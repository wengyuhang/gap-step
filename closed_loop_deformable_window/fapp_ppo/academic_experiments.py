"""Paired academic evaluation for time-critical deformable-window traversal."""

from __future__ import annotations

import argparse
import csv
import json
import platform
from dataclasses import asdict, dataclass, replace
from datetime import datetime
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import scipy
import shapely
import torch
from scipy.stats import binomtest

from .config import EnvironmentConfig
from .environment import ClosedLoopWindowEnv
from .evaluate import load_policy
from .scenario import build_scenario

plt.rcParams.update(
    {
        "font.sans-serif": ["Microsoft YaHei", "Noto Sans CJK SC", "DejaVu Sans"],
        "axes.unicode_minus": False,
    }
)


@dataclass(frozen=True)
class EvaluationCondition:
    name: str
    opportunity_mode: str
    opportunity_width: float
    motion_amplitude_multiplier: float
    deformation_amplitude_multiplier: float
    opportunity_schedule_jitter: float


def conditions_for_suite(suite: str) -> tuple[EvaluationCondition, ...]:
    if suite == "smoke":
        return (
            EvaluationCondition("id", "irregular_repeated", 1.40, 1.8, 2.0, 0.22),
            EvaluationCondition("tight", "irregular_repeated", 1.10, 1.8, 2.0, 0.22),
        )
    if suite == "default":
        return (
            EvaluationCondition("id", "irregular_repeated", 1.40, 1.8, 2.0, 0.22),
            EvaluationCondition("wide_1p80", "irregular_repeated", 1.80, 1.8, 2.0, 0.22),
            EvaluationCondition("tight_1p10", "irregular_repeated", 1.10, 1.8, 2.0, 0.22),
            EvaluationCondition("tight_0p80", "irregular_repeated", 0.80, 1.8, 2.0, 0.22),
            EvaluationCondition("motion_1p20", "irregular_repeated", 1.40, 1.2, 2.0, 0.22),
            EvaluationCondition("motion_2p30", "irregular_repeated", 1.40, 2.3, 2.0, 0.22),
            EvaluationCondition("deform_2p60", "irregular_repeated", 1.40, 1.8, 2.6, 0.22),
            EvaluationCondition("jitter_0p45", "irregular_repeated", 1.40, 1.8, 2.0, 0.45),
            EvaluationCondition(
                "single_shot_stress", "single_shot", 1.40, 1.8, 2.0, 0.22
            ),
        )
    raise ValueError("suite must be smoke or default")


def _condition_environment(
    base: EnvironmentConfig,
    condition: EvaluationCondition,
    *,
    schedule_aware_nominal: bool,
) -> EnvironmentConfig:
    return replace(
        base,
        opportunity_mode=condition.opportunity_mode,
        opportunity_width=condition.opportunity_width,
        motion_amplitude_multiplier=condition.motion_amplitude_multiplier,
        deformation_amplitude_multiplier=condition.deformation_amplitude_multiplier,
        opportunity_schedule_jitter=condition.opportunity_schedule_jitter,
        opportunity_aware_nominal=schedule_aware_nominal,
    )


def _policy_action(model, observation, environment, device) -> np.ndarray:
    actor = torch.as_tensor(
        observation.actor[None, :], dtype=torch.float32, device=device
    )
    critic = torch.as_tensor(
        observation.critic[None, :], dtype=torch.float32, device=device
    )
    with torch.no_grad():
        residual, _, _ = model.sample(actor, critic, deterministic=True)
    return environment.compose_action(residual.squeeze(0).cpu().numpy())


def run_method_episode(
    *,
    method: str,
    model,
    environment_config: EnvironmentConfig,
    quadrotor_config,
    device,
    stage: str,
    seed: int,
) -> dict[str, Any]:
    environment = ClosedLoopWindowEnv(
        environment_config, quadrotor_config, stage=stage, seed=seed
    )
    observation, _ = environment.reset(seed=seed)
    episode_return = 0.0
    final_info: dict[str, Any] = {}
    while True:
        if model is None:
            action = environment.compose_action(np.zeros(4, dtype=float))
        else:
            action = _policy_action(model, observation, environment, device)
        observation, reward, terminated, truncated, final_info = environment.step(
            action
        )
        episode_return += reward
        if terminated or truncated:
            break
    slacks = np.asarray(
        [
            float(record["temporal_slack"])
            for record in environment.crossing_records
            if np.isfinite(float(record["temporal_slack"]))
        ],
        dtype=float,
    )
    crossings = len(environment.crossing_records)
    misses = int(final_info["missed_opportunities"])
    return {
        "method": method,
        "seed": seed,
        "success": int(bool(final_info["success"])),
        "failure": final_info["failure"]
        or ("timeout" if not final_info["success"] else ""),
        "flight_time": float(final_info["time"]),
        "episode_return": float(episode_return),
        "crossings": crossings,
        "capture_rate": crossings / max(crossings + misses, 1),
        "missed_opportunities": misses,
        "mean_temporal_slack": float(np.mean(slacks)) if len(slacks) else float("nan"),
        "min_temporal_slack": float(np.min(slacks)) if len(slacks) else float("nan"),
        "energy_proxy": float(final_info["energy_proxy"]),
        "saturated_step_fraction": float(final_info["saturated_step_fraction"]),
        "closed_target_time": float(final_info["closed_target_time"]),
        "position_error": float(final_info["position_error"]),
        "velocity_error": float(final_info["velocity_error"]),
        "attitude_error": float(final_info["attitude_error"]),
        "rate_error": float(final_info["rate_error"]),
    }


def wilson_interval(successes: int, trials: int, z: float = 1.96) -> tuple[float, float]:
    if trials <= 0:
        return float("nan"), float("nan")
    probability = successes / trials
    denominator = 1.0 + z * z / trials
    center = (probability + z * z / (2.0 * trials)) / denominator
    radius = (
        z
        * np.sqrt(
            probability * (1.0 - probability) / trials
            + z * z / (4.0 * trials * trials)
        )
        / denominator
    )
    return float(center - radius), float(center + radius)


def _finite_mean(values: list[float]) -> float:
    array = np.asarray(values, dtype=float)
    finite = array[np.isfinite(array)]
    return float(finite.mean()) if len(finite) else float("nan")


def summarize(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for record in records:
        grouped.setdefault((record["condition"], record["method"]), []).append(record)
    output = []
    for (condition, method), rows in sorted(grouped.items()):
        successes = int(sum(int(row["success"]) for row in rows))
        success_rows = [row for row in rows if row["success"]]
        low, high = wilson_interval(successes, len(rows))
        output.append(
            {
                "condition": condition,
                "method": method,
                "episodes": len(rows),
                "successes": successes,
                "success_rate": successes / len(rows),
                "success_ci_low": low,
                "success_ci_high": high,
                "mean_time_success": float(
                    np.mean([row["flight_time"] for row in success_rows])
                )
                if success_rows
                else float("nan"),
                "median_time_success": float(
                    np.median([row["flight_time"] for row in success_rows])
                )
                if success_rows
                else float("nan"),
                "mean_crossings": float(np.mean([row["crossings"] for row in rows])),
                "collision_rate": float(
                    np.mean([row["failure"] == "window_collision" for row in rows])
                ),
                "floor_collision_rate": float(
                    np.mean([row["failure"] == "floor_collision" for row in rows])
                ),
                "workspace_exit_rate": float(
                    np.mean([row["failure"] == "workspace_exit" for row in rows])
                ),
                "order_violation_rate": float(
                    np.mean([row["failure"] == "order_violation" for row in rows])
                ),
                "timeout_rate": float(
                    np.mean([row["failure"] == "timeout" for row in rows])
                ),
                "mean_capture_rate": float(
                    np.mean([row["capture_rate"] for row in rows])
                ),
                "mean_missed_opportunities": float(
                    np.mean([row["missed_opportunities"] for row in rows])
                ),
                "mean_temporal_slack": _finite_mean(
                    [row["mean_temporal_slack"] for row in rows]
                ),
                "mean_min_temporal_slack": _finite_mean(
                    [row["min_temporal_slack"] for row in rows]
                ),
                "mean_energy_proxy_success": float(
                    np.mean([row["energy_proxy"] for row in success_rows])
                )
                if success_rows
                else float("nan"),
                "mean_saturated_step_fraction": float(
                    np.mean([row["saturated_step_fraction"] for row in rows])
                ),
                "mean_closed_target_time": float(
                    np.mean([row["closed_target_time"] for row in rows])
                ),
            }
        )
    return output


def _bootstrap_mean_difference(
    differences: np.ndarray,
    *,
    samples: int = 10_000,
    seed: int = 2026,
) -> tuple[float, float, float]:
    if len(differences) == 0:
        return float("nan"), float("nan"), float("nan")
    rng = np.random.default_rng(seed)
    indices = rng.integers(0, len(differences), size=(samples, len(differences)))
    means = differences[indices].mean(axis=1)
    return (
        float(differences.mean()),
        float(np.quantile(means, 0.025)),
        float(np.quantile(means, 0.975)),
    )


def paired_tests(
    records: list[dict[str, Any]],
    *,
    reference: str,
) -> list[dict[str, Any]]:
    lookup = {
        (row["condition"], row["method"], int(row["seed"])): row
        for row in records
    }
    conditions = sorted({row["condition"] for row in records})
    methods = sorted({row["method"] for row in records if row["method"] != reference})
    tests = []
    for condition in conditions:
        seeds = sorted(
            {
                int(row["seed"])
                for row in records
                if row["condition"] == condition
            }
        )
        for method in methods:
            pairs = [
                (
                    lookup.get((condition, reference, seed)),
                    lookup.get((condition, method, seed)),
                )
                for seed in seeds
            ]
            pairs = [(left, right) for left, right in pairs if left and right]
            if not pairs:
                continue
            reference_only = sum(
                left["success"] == 1 and right["success"] == 0
                for left, right in pairs
            )
            comparison_only = sum(
                left["success"] == 0 and right["success"] == 1
                for left, right in pairs
            )
            discordant = reference_only + comparison_only
            mcnemar_p = (
                float(
                    binomtest(
                        min(reference_only, comparison_only),
                        discordant,
                        p=0.5,
                    ).pvalue
                )
                if discordant
                else 1.0
            )
            both_success = [
                (left, right)
                for left, right in pairs
                if left["success"] and right["success"]
            ]
            time_differences = np.asarray(
                [
                    left["flight_time"] - right["flight_time"]
                    for left, right in both_success
                ],
                dtype=float,
            )
            difference, low, high = _bootstrap_mean_difference(time_differences)
            tests.append(
                {
                    "condition": condition,
                    "reference": reference,
                    "comparison": method,
                    "paired_episodes": len(pairs),
                    "reference_only_success": reference_only,
                    "comparison_only_success": comparison_only,
                    "mcnemar_exact_p": mcnemar_p,
                    "both_success": len(both_success),
                    "mean_time_difference_reference_minus_comparison": difference,
                    "time_difference_ci_low": low,
                    "time_difference_ci_high": high,
                }
            )
    # Holm correction over success tests.
    order = sorted(range(len(tests)), key=lambda index: tests[index]["mcnemar_exact_p"])
    running_max = 0.0
    count = len(order)
    for rank, index in enumerate(order):
        adjusted = min(1.0, (count - rank) * tests[index]["mcnemar_exact_p"])
        running_max = max(running_max, adjusted)
        tests[index]["mcnemar_holm_p"] = running_max
    return tests


def geometry_audit(
    conditions: tuple[EvaluationCondition, ...],
    environment: EnvironmentConfig,
    quadrotor,
    *,
    seeds: list[int],
) -> list[dict[str, Any]]:
    rows = []
    for condition in conditions:
        settings = _condition_environment(
            environment, condition, schedule_aware_nominal=True
        )
        for seed in seeds:
            scenario = build_scenario(
                seed=seed,
                stage="full",
                environment=settings,
                quadrotor=quadrotor,
            )
            times = np.linspace(0.0, scenario.horizon, 181)
            for window_index, window in enumerate(scenario.windows):
                states = [window.state(float(time)) for time in times]
                physical_areas = np.asarray(
                    [state.polygon.area for state in states], dtype=float
                )
                safe_areas = np.asarray(
                    [state.safe_polygon.area for state in states], dtype=float
                )
                passable = np.asarray(
                    [window.is_passable_state(state) for state in states], dtype=bool
                )
                rows.append(
                    {
                        "condition": condition.name,
                        "seed": seed,
                        "window_index": window_index,
                        "physical_valid_all": int(
                            all(state.polygon.is_valid for state in states)
                        ),
                        "physical_area_min": float(physical_areas.min()),
                        "safe_area_min": float(safe_areas.min()),
                        "safe_area_max": float(safe_areas.max()),
                        "passable_fraction": float(passable.mean()),
                        "closed_fraction": float((~passable).mean()),
                        "planned_opportunities": len(window.planned_opportunities),
                    }
                )
    return rows


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _json_safe(value: Any) -> Any:
    if isinstance(value, float) and not np.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def plot_results(
    summaries: list[dict[str, Any]],
    conditions: tuple[EvaluationCondition, ...],
    output: Path,
) -> None:
    methods = sorted({row["method"] for row in summaries})
    method_labels = {
        "FAPP-PPO": "FAPP-PPO",
        "Nominal-Reactive": "名义控制（反应式）",
        "Nominal-Schedule": "名义控制（日程感知）",
    }
    width_conditions = [
        condition
        for condition in conditions
        if condition.opportunity_mode == "irregular_repeated"
        and condition.motion_amplitude_multiplier == 1.8
        and condition.deformation_amplitude_multiplier == 2.0
        and condition.opportunity_schedule_jitter == 0.22
    ]
    figure, axes = plt.subplots(1, 2, figsize=(12, 4.6))
    for method in methods:
        rows = {
            row["condition"]: row
            for row in summaries
            if row["method"] == method
        }
        selected = [
            (condition.opportunity_width, rows[condition.name])
            for condition in width_conditions
            if condition.name in rows
        ]
        selected.sort()
        if selected:
            x = [item[0] for item in selected]
            y = [item[1]["success_rate"] for item in selected]
            low = [item[1]["success_ci_low"] for item in selected]
            high = [item[1]["success_ci_high"] for item in selected]
            axes[0].plot(
                x, y, marker="o", label=method_labels.get(method, method)
            )
            axes[0].fill_between(x, low, high, alpha=0.12)
    axes[0].set_xlabel("平均开放机会宽度 [s]")
    axes[0].set_ylabel("闭环成功率")
    axes[0].set_ylim(-0.03, 1.03)
    axes[0].grid(alpha=0.3)
    axes[0].legend(fontsize=8)

    id_rows = [row for row in summaries if row["condition"] == "id"]
    labels = [method_labels.get(row["method"], row["method"]) for row in id_rows]
    values = [row["mean_missed_opportunities"] for row in id_rows]
    axes[1].bar(labels, values, color="#ef6c00")
    axes[1].set_ylabel("平均错失机会数")
    axes[1].tick_params(axis="x", rotation=20)
    axes[1].grid(axis="y", alpha=0.3)
    figure.tight_layout()
    figure.savefig(output / "academic_summary.png", dpi=180)
    plt.close(figure)


def plot_passability_timeline(
    environment: EnvironmentConfig,
    quadrotor,
    condition: EvaluationCondition,
    output: Path,
    *,
    seed: int,
) -> None:
    settings = _condition_environment(
        environment, condition, schedule_aware_nominal=True
    )
    scenario = build_scenario(
        seed=seed, stage="full", environment=settings, quadrotor=quadrotor
    )
    times = np.linspace(0.0, scenario.horizon, 361)
    figure, axis = plt.subplots(figsize=(11, 5.0))
    for index, window in enumerate(scenario.windows):
        safe_area = np.asarray(
            [window.state(float(time)).safe_polygon.area for time in times]
        )
        axis.plot(times, safe_area, label=window.name)
        for start, end in window.planned_opportunities:
            axis.axvspan(start, end, color=f"C{index}", alpha=0.07)
    axis.axhline(
        environment.minimum_safe_area,
        color="black",
        linestyle="--",
        linewidth=1.0,
        label="可通行阈值",
    )
    axis.set_xlabel("绝对时间 [s]")
    axis.set_ylabel("安全穿越面积 [m²]")
    condition_labels = {
        "id": "同分布",
        "wide_1p80": "宽机会 1.80 秒",
        "tight_1p10": "窄机会 1.10 秒",
        "tight_0p80": "窄机会 0.80 秒",
        "motion_1p20": "低运动幅度",
        "motion_2p30": "高运动幅度",
        "deform_2p60": "高形变幅度",
        "jitter_0p45": "高日程抖动",
        "single_shot_stress": "独立单次机会",
    }
    axis.set_title(
        "预先固定的独立非周期开放机会"
        f" · 条件：{condition_labels.get(condition.name, condition.name)}"
        f" · 随机种子 {seed}"
    )
    axis.grid(alpha=0.3)
    axis.legend(ncol=3, fontsize=8)
    figure.tight_layout()
    figure.savefig(output / "passability_timeline.png", dpi=180)
    plt.close(figure)


def run_academic_experiment(
    checkpoint: str | Path,
    *,
    suite: str,
    outdir: str | Path,
    seed_start: int,
    episodes: int | None = None,
    extra_checkpoints: dict[str, str] | None = None,
) -> Path:
    model, config, device, _ = load_policy(checkpoint, "auto")
    conditions = conditions_for_suite(suite)
    episode_count = (2 if suite == "smoke" else 200) if episodes is None else episodes
    run_name = datetime.now().strftime("%Y%m%d_%H%M%S_%f") + f"_{suite}"
    output = Path(outdir) / run_name
    output.mkdir(parents=True, exist_ok=False)
    methods: dict[str, tuple[Any, Any, Any]] = {
        "FAPP-PPO": (model, config, device),
    }
    for name, path in (extra_checkpoints or {}).items():
        extra_model, extra_config, extra_device, _ = load_policy(path, "auto")
        methods[name] = (extra_model, extra_config, extra_device)

    evaluation_seeds = list(range(seed_start, seed_start + episode_count))
    geometry_rows = geometry_audit(
        conditions, config.environment, config.quadrotor, seeds=evaluation_seeds
    )
    invalid_geometry = [
        row
        for row in geometry_rows
        if not row["physical_valid_all"]
        or row["physical_area_min"] <= 0.0
        or row["safe_area_max"] < config.environment.minimum_safe_area
        or row["closed_fraction"] <= 0.0
    ]
    _write_csv(output / "geometry_audit.csv", geometry_rows)
    if invalid_geometry:
        raise RuntimeError(
            "geometry audit failed; evaluation aborted without replacing seeds: "
            f"{invalid_geometry[0]}"
        )

    records: list[dict[str, Any]] = []
    for condition in conditions:
        for episode_index in range(episode_count):
            seed = seed_start + episode_index
            for baseline_name, schedule_aware in (
                ("Nominal-Reactive", False),
                ("Nominal-Schedule", True),
            ):
                settings = _condition_environment(
                    config.environment,
                    condition,
                    schedule_aware_nominal=schedule_aware,
                )
                row = run_method_episode(
                    method=baseline_name,
                    model=None,
                    environment_config=settings,
                    quadrotor_config=config.quadrotor,
                    device=device,
                    stage="full",
                    seed=seed,
                )
                row["condition"] = condition.name
                records.append(row)
            for method_name, (method_model, method_config, method_device) in methods.items():
                settings = _condition_environment(
                    method_config.environment,
                    condition,
                    schedule_aware_nominal=method_config.environment.opportunity_aware_nominal,
                )
                row = run_method_episode(
                    method=method_name,
                    model=method_model,
                    environment_config=settings,
                    quadrotor_config=method_config.quadrotor,
                    device=method_device,
                    stage="full",
                    seed=seed,
                )
                row["condition"] = condition.name
                records.append(row)
            print(
                f"{condition.name}: {episode_index + 1}/{episode_count}",
                flush=True,
            )

    summaries = summarize(records)
    tests = paired_tests(records, reference="FAPP-PPO")
    _write_csv(output / "episodes.csv", records)
    _write_csv(output / "summary.csv", summaries)
    (output / "paired_tests.json").write_text(
        json.dumps(_json_safe(tests), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    (output / "manifest.json").write_text(
        json.dumps(
            {
                "suite": suite,
                "checkpoint": str(checkpoint),
                "extra_checkpoints": extra_checkpoints or {},
                "seed_start": seed_start,
                "episodes_per_condition": episode_count,
                "conditions": [asdict(condition) for condition in conditions],
                "methods": ["Nominal-Reactive", "Nominal-Schedule", *methods],
                "software": {
                    "python": platform.python_version(),
                    "torch": torch.__version__,
                    "numpy": np.__version__,
                    "scipy": scipy.__version__,
                    "shapely": shapely.__version__,
                },
                "statistics": {
                    "success_interval": "Wilson 95%",
                    "paired_success_test": "exact McNemar with Holm correction",
                    "paired_time_interval": "10000-sample paired bootstrap 95%; both-success only",
                },
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    plot_results(summaries, conditions, output)
    plot_passability_timeline(
        config.environment,
        config.quadrotor,
        conditions[0],
        output,
        seed=seed_start,
    )
    return output


def _parse_extra(values: list[str]) -> dict[str, str]:
    output = {}
    for value in values:
        if "=" not in value:
            raise ValueError("extra checkpoints must use NAME=PATH")
        name, path = value.split("=", 1)
        output[name] = path
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--suite", choices=("smoke", "default"), default="smoke")
    parser.add_argument(
        "--outdir",
        default="closed_loop_deformable_window/fapp_ppo/results/academic",
    )
    parser.add_argument("--seed-start", type=int, default=50_000)
    parser.add_argument("--episodes", type=int, default=None)
    parser.add_argument(
        "--extra-checkpoint",
        action="append",
        default=[],
        metavar="NAME=PATH",
    )
    args = parser.parse_args()
    output = run_academic_experiment(
        args.checkpoint,
        suite=args.suite,
        outdir=args.outdir,
        seed_start=args.seed_start,
        episodes=args.episodes,
        extra_checkpoints=_parse_extra(args.extra_checkpoint),
    )
    print(f"学术实验结果：{output}")


if __name__ == "__main__":
    main()

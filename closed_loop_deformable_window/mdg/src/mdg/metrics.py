"""Metric aggregation and publication-ready tables/plots."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def collect_metric_files(root: str | Path) -> pd.DataFrame:
    rows = []
    for path in sorted(Path(root).rglob("metrics.json")):
        payload = json.loads(path.read_text(encoding="utf-8"))
        payload["result_directory"] = str(path.parent)
        rows.append(payload)
    return pd.DataFrame(rows)


def summarize(frame: pd.DataFrame) -> pd.DataFrame:
    if frame.empty:
        return frame.copy()
    numeric = [
        column
        for column in (
            "total_flight_time",
            "frontend_time",
            "backend_time",
            "total_planning_time",
            "num_graph_nodes",
            "num_graph_edges",
            "num_lazy_repairs",
            "optimizer_iterations",
            "minimum_gate_clearance",
            "maximum_speed",
            "maximum_acceleration",
            "maximum_body_rate",
            "maximum_rotor_thrust",
        )
        if column in frame
    ]
    rows = []
    group_columns = [column for column in ("experiment", "method") if column in frame]
    for keys, group in frame.groupby(group_columns, dropna=False):
        if not isinstance(keys, tuple):
            keys = (keys,)
        row = dict(zip(group_columns, keys))
        success = group["success"].astype(bool)
        row["instances"] = len(group)
        row["failure_rate"] = float(1.0 - success.mean())
        for column in numeric:
            values = pd.to_numeric(group.loc[success, column], errors="coerce").dropna()
            row[f"{column}_mean"] = float(values.mean()) if len(values) else np.nan
            row[f"{column}_std"] = float(values.std(ddof=1)) if len(values) > 1 else 0.0
            row[f"{column}_median"] = float(values.median()) if len(values) else np.nan
        rows.append(row)
    return pd.DataFrame(rows)


def rho_statistics(frame: pd.DataFrame) -> dict[str, float]:
    values: list[float] = []
    if "selected_point_offset_ratio" in frame:
        for item in frame["selected_point_offset_ratio"]:
            if isinstance(item, str):
                item = json.loads(item)
            values.extend(float(value) for value in item)
    array = np.asarray(values, dtype=float)
    if not len(array):
        return {"count": 0, "mean": np.nan, "median": np.nan, "fraction_gt_0_5": np.nan}
    return {
        "count": int(len(array)),
        "mean": float(np.mean(array)),
        "median": float(np.median(array)),
        "fraction_gt_0_5": float(np.mean(array > 0.5)),
    }


def write_summary(root: str | Path) -> tuple[Path, Path]:
    output = Path(root)
    table_dir = output / "tables"
    figure_dir = output / "figures"
    table_dir.mkdir(parents=True, exist_ok=True)
    figure_dir.mkdir(parents=True, exist_ok=True)
    raw = collect_metric_files(output)
    raw_path = table_dir / "all_runs.csv"
    raw.to_csv(raw_path, index=False)
    summary = summarize(raw)
    summary_path = table_dir / "summary.csv"
    summary.to_csv(summary_path, index=False)
    (table_dir / "summary.tex").write_text(
        summary.to_latex(index=False, float_format="%.4f"),
        encoding="utf-8",
    )
    if not raw.empty and {"method", "total_flight_time", "success"} <= set(raw):
        successful = raw[raw["success"].astype(bool)]
        if not successful.empty:
            fig, ax = plt.subplots(figsize=(8, 4.5))
            groups = [
                successful.loc[successful["method"] == method, "total_flight_time"].astype(float)
                for method in sorted(successful["method"].unique())
            ]
            ax.boxplot(groups, labels=sorted(successful["method"].unique()), showfliers=False)
            ax.set_ylabel("flight time [s]")
            ax.tick_params(axis="x", rotation=25)
            fig.tight_layout()
            fig.savefig(figure_dir / "flight_time_comparison.png", dpi=180)
            plt.close(fig)
    if not raw.empty and "selected_point_offset_ratio" in raw:
        rho = []
        for item in raw["selected_point_offset_ratio"]:
            if isinstance(item, str):
                try:
                    item = json.loads(item)
                except json.JSONDecodeError:
                    continue
            if isinstance(item, Iterable):
                rho.extend(float(value) for value in item)
        if rho:
            fig, ax = plt.subplots(figsize=(6.5, 4.2))
            ax.hist(rho, bins=np.linspace(0.0, 0.95, 20), color="#2878b5", edgecolor="white")
            ax.set_xlabel(r"$\rho_i$")
            ax.set_ylabel("count")
            fig.tight_layout()
            fig.savefig(figure_dir / "rho_histogram.png", dpi=180)
            plt.close(fig)
    if not raw.empty and {"experiment", "method", "scenario", "success", "total_flight_time"} <= set(raw):
        e4 = raw[(raw["experiment"] == "E4") & raw["success"].astype(bool)].copy()
        oracle = e4[e4["method"] == "dense_oracle"][
            ["scenario", "total_flight_time"]
        ].rename(columns={"total_flight_time": "oracle_time"})
        if not oracle.empty:
            gaps = e4[e4["method"] != "dense_oracle"].merge(
                oracle, on="scenario", how="inner"
            )
            gaps["gap_percent"] = (
                (gaps["total_flight_time"].astype(float) - gaps["oracle_time"].astype(float))
                / gaps["oracle_time"].astype(float)
                * 100.0
            )
            gaps.to_csv(table_dir / "e4_oracle_gap.csv", index=False)
        e5 = raw[(raw["experiment"] == "E5") & raw["success"].astype(bool)].copy()
        free = e5[e5["method"] == "mdg_free_default"][
            ["scenario", "total_flight_time", "selected_point_offset_ratio"]
        ].rename(columns={"total_flight_time": "free_time"})
        center = e5[e5["method"] == "mdg_center"][
            ["scenario", "total_flight_time"]
        ].rename(columns={"total_flight_time": "center_time"})
        paired = free.merge(center, on="scenario", how="inner")
        if not paired.empty:
            paired["time_improvement_percent"] = (
                (paired["center_time"].astype(float) - paired["free_time"].astype(float))
                / paired["center_time"].astype(float)
                * 100.0
            )
            paired.to_csv(table_dir / "e6_noncenter_pairs.csv", index=False)
            stats = rho_statistics(free)
            stats["mean_time_improvement_percent"] = float(
                paired["time_improvement_percent"].mean()
            )
            (table_dir / "e6_noncenter_analysis.json").write_text(
                __import__("json").dumps(stats, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
    return raw_path, summary_path


__all__ = [
    "collect_metric_files",
    "rho_statistics",
    "summarize",
    "write_summary",
]

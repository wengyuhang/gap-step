"""Comparison-specific plots built in the existing SC experiment style."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg", force=True)
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np

from nonconvex_timevarying_window.sc_dynatogt.minco import MincoSnap


def plot_clearance_comparison(
    sc_profile: dict[str, np.ndarray],
    sip_profile: dict[str, np.ndarray],
    clearance: float,
    output_path: str | Path,
) -> Path:
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(2, 1, figsize=(10.4, 7.6), constrained_layout=True, sharex=True)
    try:
        end=max(float(sc_profile["time"][-1]),float(sip_profile["time"][-1]))
        for axis in axes:
            axis.plot(sc_profile["time"],sc_profile["distance"],color="#d95f02",linewidth=1.7,label="SC-DynaTOGT")
            axis.plot(sip_profile["time"],sip_profile["distance"],color="#0072bd",linewidth=1.7,label="SIP-DynaTOGT")
            axis.axhline(clearance,color="#20262e",linewidth=1.2,linestyle="--",label=f"required = {clearance:.3f} m")
            axis.fill_between([0.0,end],0.0,clearance,color="#e63946",alpha=0.12)
            axis.set_xlim(0.0,end); axis.grid(True,color="#dce3e8",linewidth=0.5)
            axis.set_ylabel("cuboid-to-frame distance [m]")
        axes[0].set_ylim(bottom=0.0); axes[0].set_title("Full-scale clearance profile")
        zoom=max(0.05,4.0*clearance)
        axes[1].set_ylim(0.0,zoom); axes[1].set_title("Safety-threshold zoom")
        for profile,label,color in (
            (sc_profile,"SC min","#d95f02"),(sip_profile,"SIP min","#0072bd")
        ):
            index=int(np.argmin(profile["distance"])); instant=float(profile["time"][index]); minimum=float(profile["distance"][index])
            axes[1].scatter([instant],[minimum],color=color,edgecolor="white",linewidth=0.8,s=46,zorder=5)
            axes[1].annotate(f"{label} = {minimum:.4f} m",(instant,minimum),xytext=(6,8),textcoords="offset points",fontsize=8,color=color)
        axes[1].set_xlabel("global trajectory time [s]")
        axes[0].legend(loc="upper center",ncol=3,fontsize=8)
        figure.suptitle("Whole-body clearance along the closed loop",fontsize=14)
        figure.savefig(output, dpi=170, bbox_inches="tight")
    finally:
        plt.close(figure)
    return output


def plot_contact_timeline(
    sc_profile: dict[str, np.ndarray],
    sip_profile: dict[str, np.ndarray],
    clearance: float,
    output_path: str | Path,
    *,
    sc_confirmed_intersection_times: tuple[float, ...] = (),
    contact_tolerance: float = 1.0e-12,
) -> Path:
    """Plot sampled all-window minima and an explicit zero-distance timeline.

    Each profile value is the minimum over every window, every original
    boundary primitive and the profile's boundary parameter samples at that
    trajectory time.  The binary band is deliberately separate from the
    clearance zoom: a zero geometric distance means contact/intersection,
    whereas merely falling below ``clearance`` only means a net-clearance
    violation.  This is a diagnostic plot, not a replacement for certification.
    """

    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(3, 1, figsize=(11.0, 9.2), constrained_layout=True, sharex=True)
    profiles = (
        (sc_profile, "SC-DynaTOGT", "#d95f02", 1.0),
        (sip_profile, "SIP-DynaTOGT", "#0072bd", 0.0),
    )
    try:
        end = max(float(sc_profile["time"][-1]), float(sip_profile["time"][-1]))
        for profile, label, color, level in profiles:
            times = np.asarray(profile["time"], dtype=float)
            distance = np.asarray(profile["distance"], dtype=float)
            axes[0].plot(times, distance, color=color, linewidth=1.35, label=label)
            axes[1].plot(times, distance, color=color, linewidth=1.5, label=label)
            contact = distance <= contact_tolerance
            if np.any(contact):
                axes[2].scatter(
                    times[contact], np.full(np.count_nonzero(contact), level),
                    s=13, color=color, marker="|", linewidths=1.2,
                    label=f"{label}: sampled $d=0$" if level == 1.0 else f"{label}: sampled $d=0$",
                    zorder=4,
                )
            else:
                axes[2].scatter([], [], s=13, color=color, marker="|", label=f"{label}: no sampled $d=0$")

        for instant in sc_confirmed_intersection_times:
            for axis in axes:
                axis.axvline(instant, color="#7a1f1f", linestyle="--", linewidth=1.0, zorder=3)
            axes[0].scatter([instant], [0.0], s=52, color="#7a1f1f", marker="X", zorder=6)
            axes[1].scatter([instant], [0.0], s=52, color="#7a1f1f", marker="X", zorder=6)
            axes[2].annotate(
                "SC direct primitive-in-cuboid witness",
                (instant, 1.0), xytext=(5, 11), textcoords="offset points",
                fontsize=8, color="#7a1f1f", rotation=0,
            )

        axes[0].set_title("Minimum sampled distance to every window boundary (full scale)")
        axes[0].set_ylabel("min distance $d$ [m]")
        axes[0].set_ylim(bottom=0.0)
        axes[0].legend(loc="upper center", ncol=2, fontsize=9)
        axes[1].set_title("Safety-scale view: zero distance and 15 mm clearance are distinct")
        axes[1].axhline(clearance, color="#20262e", linewidth=1.1, linestyle="--", label=f"required = {clearance:.3f} m")
        axes[1].fill_between([0.0, end], 0.0, clearance, color="#e63946", alpha=0.12)
        axes[1].set_ylim(0.0, max(0.05, 4.0 * clearance))
        axes[1].set_ylabel("min distance $d$ [m]")
        axes[1].legend(loc="upper right", fontsize=8)
        axes[2].axhspan(-0.10, 0.10, color="#0072bd", alpha=0.08)
        axes[2].axhspan(0.90, 1.10, color="#d95f02", alpha=0.08)
        axes[2].set_yticks((0.0, 1.0), ("SIP", "SC"))
        axes[2].set_ylim(-0.3, 1.45)
        axes[2].set_title("Sampled zero-distance contact timeline")
        axes[2].set_xlabel("global trajectory time [s]")
        axes[2].legend(loc="upper right", fontsize=8)
        for axis in axes:
            axis.set_xlim(0.0, end)
            axis.grid(True, color="#dce3e8", linewidth=0.5)
        figure.suptitle("Whole-body distance diagnostic: all windows and original boundary primitives", fontsize=14)
        figure.savefig(output, dpi=190, bbox_inches="tight")
    finally:
        plt.close(figure)
    return output


def plot_time_comparison(
    sc_trajectory: MincoSnap,
    sip_trajectory: MincoSnap,
    output_path: str | Path,
) -> Path:
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    labels = ("SC-DynaTOGT", "SIP-DynaTOGT")
    totals = np.asarray((sc_trajectory.total_time, sip_trajectory.total_time), dtype=float)
    colors = ("#d95f02", "#0072bd")
    figure, axis = plt.subplots(figsize=(6.8, 4.5), constrained_layout=True)
    try:
        bars = axis.bar(labels, totals, color=colors, width=0.58)
        for bar, value in zip(bars, totals):
            axis.text(
                bar.get_x() + bar.get_width() / 2.0,
                value,
                f"{value:.4f} s",
                ha="center",
                va="bottom",
                fontsize=10,
            )
        axis.set_ylabel("optimized total flight time [s]")
        axis.set_title("Closed-loop flight-time comparison")
        axis.grid(True, axis="y", color="#dce3e8", linewidth=0.5)
        axis.set_ylim(0.0, 1.15 * float(np.max(totals)))
        figure.savefig(output, dpi=170, bbox_inches="tight")
    finally:
        plt.close(figure)
    return output


__all__ = ["plot_clearance_comparison", "plot_contact_timeline", "plot_time_comparison"]

"""Plot a development trace without presenting it as a Gazebo race result."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    data = np.load(args.run / "trace.npz")["rows"]
    summary = json.loads((args.run / "summary.json").read_text())
    fig, axes = plt.subplots(2, 2, figsize=(10, 7), constrained_layout=True)
    time = data[:, 0]
    axes[0, 0].plot(time, np.linalg.norm(data[:, 4:7], axis=1))
    axes[0, 0].set(xlabel="Model time (s)", ylabel="Speed (m/s)", title="Speed, not a successful lap time")
    axes[0, 1].step(time, data[:, 18], where="post")
    axes[0, 1].set(xlabel="Model time (s)", ylabel="Ordered gates completed", ylim=(-.1, 7.1))
    axes[1, 0].plot(time, data[:, 19])
    axes[1, 0].axhline(0, color="red", linestyle="--")
    axes[1, 0].set(xlabel="Model time (s)", ylabel="Remaining interval clearance (m)")
    axes[1, 1].plot(time, data[:, 20], linewidth=.6)
    axes[1, 1].axhline(20, color="orange", linestyle="--", label="20 ms target")
    axes[1, 1].axhline(100, color="red", linestyle=":", label="100 ms comparison limit")
    axes[1, 1].set(xlabel="Model time (s)", ylabel="MPPI cycle wall time (ms)")
    axes[1, 1].legend()
    for ax in axes.flat: ax.grid(alpha=.2)
    fig.suptitle(f"V3 nominal-model diagnostic ONLY — {summary['gates']}/7 gates; {summary['failure']}")
    fig.savefig(args.run / "diagnostic.png", dpi=160)
    plt.close(fig)


if __name__ == "__main__": main()

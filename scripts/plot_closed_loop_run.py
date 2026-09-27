"""Plot native closed-loop FR lift and crawl measurements from a run directory."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    path = args.run.resolve()
    with (path / "control_20ms.csv").open(encoding="utf-8", newline="") as source:
        rows = list(csv.DictReader(source))
    t = np.array([float(row["time_s"]) for row in rows])

    def series(key: str) -> np.ndarray:
        return np.array([float(row[key]) for row in rows])

    fig, axes = plt.subplots(4, 2, figsize=(15, 11), sharex=True)
    for key, name in (("fz_fl_n", "FL"), ("fz_fr_n", "FR"),
                      ("fz_rl_n", "RL"), ("fz_rr_n", "RR")):
        axes[0, 0].plot(t, series(key) / 1000., label=name)
    axes[0, 0].set_ylabel("Wheel load (kN)")
    axes[0, 0].legend(ncol=4)
    axes[0, 1].plot(t, series("zmp_lambda_min"), label="ZMP")
    axes[0, 1].plot(t, series("com_lambda_min"), label="CoM")
    axes[0, 1].axhline(0.05, color="gray", linestyle="--")
    axes[0, 1].set_ylabel("Support triangle min lambda")
    axes[0, 1].legend()
    axes[1, 0].plot(t, 1000. * series("fr_top_clearance_m"))
    axes[1, 0].axhline(10., color="gray", linestyle="--")
    axes[1, 0].set_ylabel("FR clearance above road top (mm)")
    axes[1, 1].plot(t, series("vx_kph"))
    axes[1, 1].set_ylabel("Forward speed (km/h)")
    axes[2, 0].plot(t, series("yo_m") - series("yo_m")[0])
    axes[2, 0].set_ylabel("Lateral displacement (m)")
    axes[2, 1].plot(t, series("yaw_deg") - series("yaw_deg")[0])
    axes[2, 1].set_ylabel("Yaw change (deg)")
    for key, name in (("torque_fl_nm", "FL"), ("torque_rl_nm", "RL"),
                      ("torque_rr_nm", "RR")):
        axes[3, 0].plot(t, series(key), label=name)
    axes[3, 0].set_ylabel("Drive torque (Nm)")
    axes[3, 0].legend(ncol=3)
    for key, name in (("force_corr_fl_n", "FL"),
                      ("force_corr_rl_n", "RL"),
                      ("force_corr_rr_n", "RR")):
        axes[3, 1].plot(t, series(key), label=name)
    axes[3, 1].set_ylabel("Suspension feedback (N)")
    axes[3, 1].legend(ncol=3)
    modes = [row["mode"] for row in rows]
    for index in range(1, len(modes)):
        if modes[index] != modes[index - 1]:
            for ax in axes.flat:
                ax.axvline(t[index], color="gray", alpha=0.18, linewidth=0.8)
    for ax in axes[-1]:
        ax.set_xlabel("Simulation time (s)")
    fig.suptitle("I_I FR lift and three-wheel feedback crawl")
    fig.tight_layout()
    target = path / "closed_loop_overview.png"
    fig.savefig(target, dpi=160)
    print(target)


if __name__ == "__main__":
    main()

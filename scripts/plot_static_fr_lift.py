"""Render the native FR lift/recovery evidence for one result.json."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


WHEELS = ("L1", "R1", "L2", "R2")
LABELS = ("FL", "FR", "RL", "RR")


def plot_result(result_path: Path) -> Path:
    result = json.loads(result_path.read_text(encoding="utf-8"))
    with Path(result["native"]["csv"]).open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    t = np.asarray([float(row["time_s"]) for row in rows])
    def column(name):
        return np.asarray([float(row[f"exp_{name}"]) for row in rows])
    fz = np.column_stack([column(f"Fz_{wheel}") for wheel in WHEELS])
    force = np.column_stack([column(f"FsExt_{wheel}") for wheel in WHEELS])
    clearance_mm = (column("Z_R1") - column("Zgnd_R1i") -
                    result["tyre_unloaded_radius_m"]) * 1000.
    contacts = np.stack([
        np.column_stack((column(f"Xctc_{wheel}i"),
                         column(f"Yctc_{wheel}i")))
        for wheel in WHEELS], axis=1)
    zmp = (fz[:, :, None] * contacts).sum(axis=1) / fz.sum(axis=1)[:, None]
    com = np.column_stack((column("XCG_TM"), column("YCG_TM")))
    triangle = contacts[:, [0, 2, 3], :]
    frame = np.stack((triangle[:, 1] - triangle[:, 0],
                      triangle[:, 2] - triangle[:, 0]), axis=2)
    bary = np.linalg.solve(frame, (zmp - triangle[:, 0])[:, :, None])[:, :, 0]
    zmp_margin = np.minimum.reduce((1. - bary.sum(axis=1), bary[:, 0], bary[:, 1]))
    bary_com = np.linalg.solve(frame, (com - triangle[:, 0])[:, :, None])[:, :, 0]
    com_margin = np.minimum.reduce((1. - bary_com.sum(axis=1),
                                    bary_com[:, 0], bary_com[:, 1]))
    fig, axes = plt.subplots(3, 2, figsize=(15, 11), constrained_layout=True)
    for i, name in enumerate(LABELS):
        axes[0, 0].plot(t, fz[:, i], label=name)
        axes[0, 1].plot(t, force[:, i] / 1000., label=name)
    axes[0, 0].set_title("Wheel vertical loads")
    axes[0, 0].set_ylabel("N")
    axes[0, 1].set_title("Applied suspension forces")
    axes[0, 1].set_ylabel("kN")
    axes[0, 0].legend(ncol=4)
    axes[0, 1].legend(ncol=4)
    axes[1, 0].plot(t, column("Roll_E"), label="roll")
    axes[1, 0].plot(t, column("Pitch"), label="pitch")
    axes[1, 0].set_title("Body attitude")
    axes[1, 0].set_ylabel("deg")
    axes[1, 0].legend()
    axes[1, 1].plot(t, clearance_mm, label="FR clearance")
    axes[1, 1].axhline(10., color="gray", linestyle="--", label="10 mm target")
    axes[1, 1].set_title("FR tyre clearance")
    axes[1, 1].set_ylabel("mm")
    axes[1, 1].legend()
    axes[2, 0].plot(t, zmp_margin, label="ZMP")
    axes[2, 0].plot(t, com_margin, label="CoM")
    axes[2, 0].axhline(0.05, color="gray", linestyle="--")
    axes[2, 0].set_title("Three-wheel support margin")
    axes[2, 0].set_ylabel("barycentric minimum")
    axes[2, 0].legend()
    hold = (t >= result["config"]["start_s"] + result["config"]["ramp_s"]) & (
        t <= result["config"]["start_s"] + result["config"]["ramp_s"] +
        result["config"]["hold_s"])
    axis = axes[2, 1]
    tri = triangle[hold].mean(axis=0)
    closed = np.vstack((tri, tri[0]))
    axis.plot(closed[:, 0], closed[:, 1], "k-", label="FL-RL-RR")
    axis.plot(zmp[hold, 0], zmp[hold, 1], label="ZMP")
    axis.plot(com[hold, 0], com[hold, 1], label="CoM")
    for xy, name in zip(tri, ("FL", "RL", "RR")):
        axis.annotate(name, xy)
    axis.set_title("Support triangle during lift hold")
    axis.set_xlabel("global X (m)")
    axis.set_ylabel("global Y (m)")
    axis.legend()
    for ax in axes.flat[:5]:
        ax.set_xlabel("simulation time (s)")
        ax.grid(alpha=0.25)
    path = result_path.parent / "lift_evidence.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result", type=Path)
    args = parser.parse_args()
    print(plot_result(args.result))


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Plot q, dq, ddq, dddq of every joint from a CSV written by test_s_curve.

Usage: plot_s_curve.py s_curve_custom.csv [more.csv ...]   (saves <name>.png next to each CSV)
"""
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

QUANTITIES = [("q", "rad"), ("dq", "rad/s"), ("ddq", "rad/s²"), ("dddq", "rad/s³")]

for path in sys.argv[1:]:
    data = np.genfromtxt(path, delimiter=",", names=True)
    fig, axes = plt.subplots(5, 1, figsize=(10, 13), sharex=True)
    axes[0].plot(data["t"], data["sigma"], color="k")
    axes[0].set_ylabel("sigma")
    for ax, (name, unit) in zip(axes[1:], QUANTITIES):
        for i in range(1, 8):
            ax.plot(data["t"], data[f"{name}{i}"], label=f"J{i}")
        ax.set_ylabel(f"{name} [{unit}]")
        ax.grid(alpha=0.3)
    axes[1].legend(ncol=7, fontsize=8, loc="upper center")
    axes[-1].set_xlabel("t [s]")
    fig.suptitle(path)
    fig.tight_layout()
    out = path.rsplit(".", 1)[0] + ".png"
    fig.savefig(out, dpi=90)
    print("saved", out)

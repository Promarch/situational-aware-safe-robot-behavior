#!/usr/bin/env python3
"""Plot sim_hand.csv from sim_hand_slowdown: EE speed vs. energy-limited speed and hand distance.

Usage: plot_sim.py sim_hand.csv   (saves sim_hand.png next to it)
"""
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

RADIUS = 0.05
NAMES = ["home->pre-pick", "down", "up", "pre-pick->home"]

path = sys.argv[1]
d = np.genfromtxt(path, delimiter=",", names=True)
t = d["t"] - d["t"][0]
fig, (ax_v, ax_d) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

for seg in range(len(NAMES)):
    sel = d["segment"] == seg
    if sel.any():
        for ax in (ax_v, ax_d):
            ax.axvspan(t[sel][0], t[sel][-1], alpha=0.06 if seg % 2 else 0.12, color="gray")
        ax_v.text(t[sel][0], 0, NAMES[seg], fontsize=8, va="bottom")

ax_v.plot(t, d["speed"], color="k", label="EE speed")
v_safe = np.where(d["v_safe"] > 0, d["v_safe"], np.nan)
ax_v.plot(t, v_safe, color="tab:red", lw=1, label="v_safe (while a hand is present)")
ax_v.set_ylabel("speed [m/s]")
ax_v.legend(loc="upper right")
ax_v.grid(alpha=0.3)

dist = np.where(d["hand_distance"] >= 0, d["hand_distance"], np.nan)
ax_d.plot(t, dist, color="tab:blue", label="EE–hand distance")
ax_d.axhline(RADIUS, color="tab:red", ls="--", lw=1, label=f"radius {RADIUS * 100:.0f} cm")
ax_d.set_ylabel("distance [m]")
ax_d.set_xlabel("t [s]")
ax_d.legend(loc="upper right")
ax_d.grid(alpha=0.3)

fig.tight_layout()
out = path.rsplit(".", 1)[0] + ".png"
fig.savefig(out, dpi=90)
print("saved", out)

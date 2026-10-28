#!/usr/bin/env python3
"""Plot a slowdown log (sim_hand.csv or robot_slowdown.csv): EE speed vs. energy-limited speed,
and the EE-hand distance.

Segments are placed back to back on a "motion time" axis (time spent in segments, waiting for
Enter or the gripper left out), with a small gap and no line drawn across segments.

Usage: plot_sim.py LOG.csv   (saves LOG.png next to it)
"""
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

RADIUS = 0.05
GAP = 0.3  # [s] between segments on the motion-time axis
NAMES = ["start->pre-pick", "down", "up", "pre-pick->start"]

path = sys.argv[1]
d = np.genfromtxt(path, delimiter=",", names=True)
has_segment_time = "t_segment" in d.dtype.names

fig, (ax_v, ax_d) = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
offset = 0.0
for seg in range(len(NAMES)):
    sel = d["segment"] == seg
    if not sel.any():
        continue
    if has_segment_time:
        t = offset + d["t_segment"][sel]
    else:  # older logs: wall-clock time from the segment's first frame
        t = offset + d["t"][sel] - d["t"][sel][0]
    for ax in (ax_v, ax_d):
        ax.axvspan(t[0], t[-1], alpha=0.06 if seg % 2 else 0.12, color="gray")
    ax_v.text(t[0], 0, NAMES[seg], fontsize=8, va="bottom")

    first = seg == 0
    ax_v.plot(t, d["speed"][sel], color="k", label="EE speed" if first else None)
    v_safe = np.where(d["v_safe"][sel] > 0, d["v_safe"][sel], np.nan)
    ax_v.plot(t, v_safe, color="tab:red", lw=1,
              label="v_safe (while a hand is present)" if first else None)
    dist = np.where(d["hand_distance"][sel] >= 0, d["hand_distance"][sel], np.nan)
    ax_d.plot(t, dist, color="tab:blue", label="EE–hand distance" if first else None)
    offset = t[-1] + GAP

ax_v.set_ylabel("speed [m/s]")
ax_v.legend(loc="upper right")
ax_v.grid(alpha=0.3)
ax_d.axhline(RADIUS, color="tab:red", ls="--", lw=1, label=f"radius {RADIUS * 100:.0f} cm")
ax_d.set_ylabel("distance [m]")
ax_d.set_xlabel("motion time [s]")
ax_d.legend(loc="upper right")
ax_d.grid(alpha=0.3)

fig.tight_layout()
out = path.rsplit(".", 1)[0] + ".png"
fig.savefig(out, dpi=90)
print("saved", out)

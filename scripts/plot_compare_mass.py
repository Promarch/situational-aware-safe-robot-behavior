#!/usr/bin/env python3
"""Plot compare_mass.csv from compare_mass: free-contact effective mass vs. tabulated constrained
mass along the nominal pick-and-place path, and the EE position the table is read at.

Usage: plot_compare_mass.py compare_mass.csv   (saves compare_mass.png next to it)
"""
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

NAMES = ["start->pre-pick", "down to pick", "up from pick", "pre-pick->pre-place",
         "down to place", "up from place", "pre-place->start"]

path = sys.argv[1]
d = np.genfromtxt(path, delimiter=",", names=True)
fig, (ax_m, ax_p) = plt.subplots(2, 1, figsize=(14, 7), sharex=True,
                                 gridspec_kw={"height_ratios": [2, 1]})
for mi in range(len(NAMES)):
    sel = d["motion"] == mi
    if not sel.any():
        continue
    x = mi + d["progress"][sel] * 0.9  # each motion over 0.9 of a unit, small gap between
    first = mi == 0
    ax_m.plot(x, d["m_eff"][sel], color="tab:purple", label="m_eff, free (model)" if first else None)
    ax_m.plot(x, d["m_table"][sel], color="tab:orange", ls="--",
              label="m_table, constrained (table at the EE)" if first else None)
    ax_p.plot(x, d["az"][sel], color="tab:blue", label="EE az = sqrt(x^2+y^2) [m]" if first else None)
    ax_p.plot(x, d["z"][sel], color="tab:green", label="EE z [m]" if first else None)
    for ax in (ax_m, ax_p):
        ax.axvspan(mi, mi + 0.9, alpha=0.05 if mi % 2 else 0.1, color="gray")
    ax_m.text(mi, 1.0, NAMES[mi], transform=ax_m.get_xaxis_transform(), fontsize=8, va="bottom")

# Table grid limits, to see where the lookup uses the nearest data.
ax_p.axhspan(0.295, 0.695, color="tab:blue", alpha=0.07, label="table az range")
ax_p.axhspan(0.025, 0.145, color="tab:green", alpha=0.07, label="table z range")
ax_m.set_ylabel("mass [kg]")
ax_m.set_ylim(bottom=0)
ax_m.legend(loc="upper right", fontsize=8)
ax_m.grid(alpha=0.3)
ax_p.set_ylabel("EE position [m]")
ax_p.set_xlabel("motion (progress along each motion)")
ax_p.legend(loc="upper right", fontsize=8, ncol=2)
ax_p.grid(alpha=0.3)
fig.tight_layout()
out = path.rsplit(".", 1)[0] + ".png"
fig.savefig(out, dpi=90)
print("saved", out)

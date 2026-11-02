#!/usr/bin/env python3
"""Plot pick_place.csv from pick_place (format in apps/pick_place.cpp).

Writes two figures next to the log:
  <name>_speed.png   EE speed and energy-limited speed v_safe (every tick), and the EE's
                     clearance to the nearest hand's intrusion boundary (< 0: inside)
  <name>_joints.png  measured (solid) and commanded (dashed) velocity of each joint
  <name>_mass.png    effective mass m_eff at the reference point along its motion, which sets
                     v_safe = sqrt(2 S_lim / m_eff)
Segments are placed back to back on a "motion time" axis (waits between them left out). Gray
bands mark when a hand is seen, pink ones when its position is unknown or the stream is stale; vertical lines mark the supervisor's commands from
<name>_decisions.csv (red: slow down, green: resume), labelled with the cap in % of nominal.

Usage: plot_slowdown.py LOG.csv
"""
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

GAP = 0.3  # [s] between segments on the motion-time axis
NAMES = ["start->pre-pick", "down to pick", "up from pick", "pre-pick->pre-place",
         "down to place", "up from place", "pre-place->start"]

path = sys.argv[1]
base = path.rsplit(".", 1)[0]
d = np.genfromtxt(path, delimiter=",", names=True)
decisions_path = base + "_decisions.csv"
decisions = (np.genfromtxt(decisions_path, delimiter=",", names=True, dtype=None, encoding=None)
             if os.path.exists(decisions_path) else None)
if decisions is not None and decisions.ndim == 0:
    decisions = decisions.reshape(1)

# Motion time: each segment's own time, segments back to back.
x = np.full(len(d), np.nan)
offsets = {}
offset = 0.0
for seg in range(len(NAMES)):
    sel = d["segment"] == seg
    if sel.any():
        offsets[seg] = offset
        x[sel] = offset + d["t_segment"][sel]
        offset = x[sel][-1] + GAP


def per_segment(values):
    """Values with NaN at segment changes, so lines are not drawn across segments."""
    out = np.array(values, dtype=float)
    out[1:][np.diff(d["segment"]) != 0] = np.nan
    return out


def decorate(ax, label_segments=False):
    for seg, off in offsets.items():
        sel = d["segment"] == seg
        ax.axvspan(off, x[sel][-1], alpha=0.04, color="tab:blue")
        if label_segments:
            ax.text(off, 1.0, NAMES[seg], transform=ax.get_xaxis_transform(), fontsize=8,
                    va="bottom")
    hand = per_segment(d["hand_present"]) == 1
    unlocated = per_segment(d["hand_unlocated"]) == 1
    ax.fill_between(x, 0, 1, where=hand & ~unlocated, transform=ax.get_xaxis_transform(),
                    color="gray", alpha=0.2, lw=0)
    ax.fill_between(x, 0, 1, where=unlocated, transform=ax.get_xaxis_transform(),
                    color="tab:red", alpha=0.12, lw=0)
    if decisions is not None:
        for row in decisions:
            if int(row["segment"]) not in offsets:
                continue
            xd = offsets[int(row["segment"])] + row["t_segment"]
            slow = row["action"] == "slow"
            ax.axvline(xd, color="tab:red" if slow else "tab:green", ls="--", lw=0.8)
            if label_segments:
                ax.text(xd, 0.97, f"{row['cap_percent']:.0f}%", transform=ax.get_xaxis_transform(),
                        fontsize=7, color="tab:red" if slow else "tab:green", ha="left", va="top")
    ax.grid(alpha=0.3)


# --- EE speed vs v_safe.
fig, (ax_v, ax_d) = plt.subplots(2, 1, figsize=(14, 7), sharex=True,
                                 gridspec_kw={"height_ratios": [3, 1]})
ax_v.plot(x, per_segment(d["ee_speed"]), color="k", lw=1.2,
          label="reference point speed |J(q)·dq| (object tip while held, else EE)")
ax_v.plot(x, per_segment(d["v_safe"]), color="tab:red", lw=1,
          label="v_safe = sqrt(2·S_lim / m_eff) along the motion")
ax_v.set_ylabel("speed [m/s]")
ax_v.legend(loc="upper right", fontsize=8)
decorate(ax_v, label_segments=True)
ax_d.plot(x, per_segment(d["hand_clearance"]), color="tab:blue",
          label="EE clearance to the hand (distance − sphere radius − margin)")
ax_d.axhline(0.0, color="tab:red", ls=":", lw=1, label="intrusion boundary")
ax_d.set_ylabel("clearance [m]")
ax_d.set_xlabel("motion time [s]   (gray: hand; pink: position unknown / stream stale; dashed: red slow down, green resume)")
ax_d.legend(loc="upper right", fontsize=8)
decorate(ax_d)
fig.tight_layout()
fig.savefig(base + "_speed.png", dpi=90)
print("saved", base + "_speed.png")

# --- Joint velocities.
fig, axes = plt.subplots(7, 1, figsize=(14, 12), sharex=True)
for i, ax in enumerate(axes, start=1):
    ax.plot(x, per_segment(d[f"dq{i}"]), color="k", lw=1, label="measured")
    ax.plot(x, per_segment(d[f"dq_d{i}"]), color="tab:orange", lw=0.8, ls="--", label="commanded")
    ax.set_ylabel(f"dq{i} [rad/s]", fontsize=8)
    decorate(ax, label_segments=i == 1)
axes[0].legend(loc="upper right", fontsize=8)
axes[-1].set_xlabel("motion time [s]   (gray: hand; pink: position unknown / stream stale; dashed: red slow down, green resume)")
fig.tight_layout()
fig.savefig(base + "_joints.png", dpi=90)
print("saved", base + "_joints.png")

# --- Effective mass.
fig, ax = plt.subplots(1, 1, figsize=(14, 4.5))
ax.plot(x, per_segment(d["m_eff"]), color="tab:purple", lw=1.2,
        label="m_eff = 1 / (u^T [J M^-1 J^T]_pos u) at the reference point (object tip while held)")
ax.set_ylabel("effective mass [kg]")
ax.set_ylim(bottom=0)
ax.set_xlabel("motion time [s]   (gray: hand; pink: position unknown / stream stale; dashed: red slow down, green resume)")
ax.legend(loc="upper right", fontsize=8)
decorate(ax, label_segments=True)
fig.tight_layout()
fig.savefig(base + "_mass.png", dpi=90)
print("saved", base + "_mass.png")

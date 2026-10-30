#!/usr/bin/env python3
"""Replay hand appearances from a CSV file over UDP, the way the detector sends them.

Uses HandUdpSender from hand_udp_protocol.py (the udp-implementation branch's own sender), so the
wire format and sending behaviour match hand_detect_udp.py: one packet per camera frame, also
with no hand visible (heartbeat), points in the robot base frame.

CSV, one row per appearance ('#' starts a comment):
    t_on, t_off, x, y, z, radius
  t_on, t_off  seconds since this program started (with --loop: within each period)
  x, y, z      sphere center in the robot base frame [m]; nan = hand seen without a usable
               3D position (the packet then carries the HAND_UNLOCATED flag)
  radius       sphere radius [m]

Usage: python3 replay_hands.py HANDS.csv [--dest 127.0.0.1:5005] [--fps 30] [--loop PERIOD]
       Ctrl+C to stop. Stopping the sender makes the receiver see a stale stream.
"""
import argparse
import time

import numpy as np

import hand_udp_protocol as hup


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("csv")
    p.add_argument("--dest", action="append", default=None, metavar="HOST:PORT",
                   help="UDP destination, repeatable (default 127.0.0.1:5005)")
    p.add_argument("--fps", type=float, default=30.0, help="camera frame rate (default 30)")
    p.add_argument("--loop", type=float, default=None, metavar="PERIOD",
                   help="repeat the schedule every PERIOD seconds")
    args = p.parse_args()

    rows = np.loadtxt(args.csv, delimiter=",", comments="#", ndmin=2)
    if rows.shape[1] != 6:
        raise SystemExit(f"{args.csv}: expected 6 columns t_on,t_off,x,y,z,radius")
    dests = [hup.parse_dest(d) for d in (args.dest or ["127.0.0.1:5005"])]
    sender = hup.HandUdpSender(dests, hup.FRAME_ROBOT_BASE)
    print(f"{len(rows)} appearance(s) from {args.csv} -> "
          f"{', '.join(f'{h}:{port}' for h, port in dests)} at {args.fps:g} fps")

    period = 1.0 / args.fps
    t0 = time.monotonic()
    shown = None
    try:
        for k in range(10**9):
            t_frame = t0 + k * period
            time.sleep(max(0.0, t_frame - time.monotonic()))
            t = t_frame - t0
            t_sched = t % args.loop if args.loop else t
            active = rows[(rows[:, 0] <= t_sched) & (t_sched < rows[:, 1])][: hup.MAX_HANDS]
            hands = [hup.Hand(hup.HAND_UNKNOWN, 1.0, float(r[5]),
                              np.array([r[2:5]], dtype=np.float32)) for r in active]
            sender.send(hands, time.time_ns())

            state = active.tobytes()  # bytes, so rows with NaN compare equal to themselves
            if state != shown:  # report only changes
                desc = ", ".join(
                    "unlocated" if np.isnan(r[2]) else f"({r[2]:.3f}, {r[3]:.3f}, {r[4]:.3f}) r={r[5]:.2f}"
                    for r in active) or "no hand"
                print(f"t={t:7.2f} s  seq {sender.seq - 1}: {desc}")
                shown = state
    except KeyboardInterrupt:
        pass
    finally:
        print(f"sent {sender.seq} packets, {sender.n_errors} send errors"
              + (f" (last: {sender.last_error})" if sender.last_error else ""))
        sender.close()


if __name__ == "__main__":
    main()

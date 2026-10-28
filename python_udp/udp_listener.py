#!/usr/bin/env python3
"""
Debug listener for the hand keypoint UDP stream (hand_udp_protocol.py).

Shows once per second: packet rate, lost / reordered / malformed packets,
transport latency (receive - send), image age (receive - capture), hands and
the closest valid landmark. Warns when the stream goes silent (watchdog).

    python3 udp_examples/udp_listener.py                       # 127.0.0.1:5005
    python3 udp_examples/udp_listener.py --port 5006 -v        # print every packet
    python3 udp_examples/udp_listener.py --bind 0.0.0.0        # accept from other machines
    python3 udp_examples/udp_listener.py --csv log.csv         # one row per landmark, for pandas

Latency numbers are only meaningful if sender and listener share a clock:
same machine, or both synced with chrony/PTP (check with `chronyc tracking`).
"""
import argparse
import csv
import socket
import statistics
import time

import numpy as np

import hand_udp_protocol as hup                     # python udp_examples/<script>.py

RESTART_JUMP = 1000   # seq jumping back by more than this = sender restarted


def fmt_point(p) -> str:
    return "  ---  " if np.isnan(p[0]) else f"({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f})"


def closest_valid(hands):
    """(distance to frame origin, point) of the closest valid landmark, or None."""
    best = None
    for h in hands:
        pts = h.points[~np.isnan(h.points[:, 0])]
        if len(pts):
            d = np.linalg.norm(pts, axis=1)
            i = int(d.argmin())
            if best is None or d[i] < best[0]:
                best = (float(d[i]), pts[i])
    return best


def stats_str(name, vals_ms):
    if not vals_ms:
        return f"{name} --"
    return (f"{name} {min(vals_ms):5.2f}/{statistics.fmean(vals_ms):5.2f}/"
            f"{max(vals_ms):6.2f} ms")


def main():
    ap = argparse.ArgumentParser(description="Debug listener for the hand UDP stream")
    ap.add_argument("--bind", default="127.0.0.1", help="interface to listen on")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--timeout", type=float, default=0.2,
                    help="seconds without packets before the STALE warning")
    ap.add_argument("-v", "--verbose", action="store_true", help="print every packet")
    ap.add_argument("--csv", default=None, help="log every landmark to this CSV file")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    try:
        sock.bind((args.bind, args.port))
    except OSError as e:
        raise SystemExit(f"cannot bind {args.bind}:{args.port}: {e}\n"
                         "Another program (the C++ receiver?) probably owns this port. "
                         "Let the sender stream to a second port with --dest.")
    sock.settimeout(0.05)
    print(f"listening on {args.bind}:{args.port}  (Ctrl+C to stop)")

    csv_file = writer = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow(["seq", "t_capture_ns", "t_send_ns", "t_recv_ns", "frame_id", "flags",
                         "hand", "handedness", "score", "landmark", "x", "y", "z"])

    last_seq = None
    t_last_rx = None          # monotonic, for the watchdog
    stale_reported = False
    totals = dict(rx=0, lost=0, reordered=0, bad=0)
    win = dict(rx=0, lost=0, reordered=0, bad=0, transport=[], age=[])
    last_pkt = None
    t_report = time.monotonic()

    try:
        while True:
            try:
                data, addr = sock.recvfrom(65535)
                t_recv_ns = time.time_ns()
                t_last_rx = time.monotonic()
            except socket.timeout:
                data = None

            now = time.monotonic()
            if data is not None:
                try:
                    pkt = hup.unpack(data)
                except ValueError as e:
                    win["bad"] += 1
                    totals["bad"] += 1
                    print(f"\nmalformed packet from {addr[0]}:{addr[1]}: {e}")
                    pkt = None
                if pkt is not None:
                    if stale_reported:
                        print(f"\nstream back (seq {pkt.seq})")
                        stale_reported = False
                    accept = True
                    if last_seq is not None:
                        diff = (pkt.seq - last_seq) & 0xFFFFFFFF
                        if diff == 0 or diff >= 0x80000000:          # duplicate or older
                            if ((last_seq - pkt.seq) & 0xFFFFFFFF) > RESTART_JUMP:
                                print(f"\nsender restarted? seq {last_seq} -> {pkt.seq}")
                            else:
                                win["reordered"] += 1
                                totals["reordered"] += 1
                                accept = False                      # never go back in time
                        elif diff > 1:
                            win["lost"] += diff - 1
                            totals["lost"] += diff - 1
                    if accept:
                        last_seq = pkt.seq
                        last_pkt = pkt
                        win["rx"] += 1
                        totals["rx"] += 1
                        win["transport"].append((t_recv_ns - pkt.t_send_ns) / 1e6)
                        win["age"].append((t_recv_ns - pkt.t_capture_ns) / 1e6)
                        if args.verbose:
                            print_packet(pkt, t_recv_ns)
                        if writer:
                            log_csv(writer, pkt, t_recv_ns)

            # watchdog: this is exactly what the robot side must do as well
            if t_last_rx is not None and now - t_last_rx > args.timeout and not stale_reported:
                print(f"\nSTALE: no packet for {(now - t_last_rx) * 1000:.0f} ms "
                      f"-> robot side must treat this as UNSAFE")
                stale_reported = True

            if now - t_report >= 1.0:
                dt = now - t_report
                t_report = now
                if win["rx"] or win["bad"]:
                    line = (f"{win['rx'] / dt:5.1f} Hz  lost {win['lost']}  reord {win['reordered']}  "
                            f"bad {win['bad']}  | {stats_str('transport', win['transport'])}  "
                            f"{stats_str('age', win['age'])}")
                    if last_pkt is not None:
                        line += f"  | {hup.FRAME_NAMES.get(last_pkt.frame_id, '?')} hands {len(last_pkt.hands)}"
                        if last_pkt.flags & hup.FLAG_HAND_UNLOCATED:
                            line += " UNLOCATED!"
                        c = closest_valid(last_pkt.hands)
                        if c:
                            line += f"  closest {c[0]:.3f} m {fmt_point(c[1])}"
                    if not args.verbose:
                        print(line.ljust(150), end="\r", flush=True)
                    else:
                        print("--", line)
                elif t_last_rx is None:
                    print("waiting for packets ...".ljust(60), end="\r", flush=True)
                win = dict(rx=0, lost=0, reordered=0, bad=0, transport=[], age=[])
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\ntotal: received {totals['rx']}, lost {totals['lost']}, "
              f"reordered/dup {totals['reordered']}, malformed {totals['bad']}")
        if csv_file:
            csv_file.close()
        sock.close()


def print_packet(pkt, t_recv_ns):
    flags = " UNLOCATED" if pkt.flags & hup.FLAG_HAND_UNLOCATED else ""
    print(f"seq {pkt.seq:8d}  hands {len(pkt.hands)}  "
          f"age {(t_recv_ns - pkt.t_capture_ns) / 1e6:6.2f} ms  "
          f"transport {(t_recv_ns - pkt.t_send_ns) / 1e6:5.3f} ms{flags}")
    for k, h in enumerate(pkt.hands):
        print(f"   hand {k} {hup.HANDEDNESS_NAMES.get(h.handedness, '?')} score {h.score:.2f} "
              f"valid {h.n_valid:2d}/21  wrist {fmt_point(h.points[0])}  "
              f"index tip {fmt_point(h.points[8])}")


def log_csv(writer, pkt, t_recv_ns):
    base = [pkt.seq, pkt.t_capture_ns, pkt.t_send_ns, t_recv_ns, pkt.frame_id, pkt.flags]
    if not pkt.hands:   # keep heartbeat rows so gaps in the log are real gaps
        writer.writerow(base + [""] * 7)
    for k, h in enumerate(pkt.hands):
        for j, p in enumerate(h.points):
            writer.writerow(base + [k, h.handedness, f"{h.score:.3f}", j,
                                    f"{p[0]:.5f}", f"{p[1]:.5f}", f"{p[2]:.5f}"])


if __name__ == "__main__":
    main()

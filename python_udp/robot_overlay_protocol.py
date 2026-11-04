"""Robot overlay stream: the supervisor's view of every camera frame, sent by pick_place to the
hand detector for its video overlay (display only). Wire format: src/overlay_udp.h (version 2).

    rx = OverlayReceiver(5006)       # background thread
    o = rx.latest(max_age_s=0.3)     # Overlay or None
"""
import socket
import struct
import threading
import time
from dataclasses import dataclass, field

import numpy as np

MAGIC = 0x4C564F52  # "ROVL"
VERSION = 2
_HEADER = struct.Struct("<IHBBIQf3fBBBB3f")  # 52 bytes
_POINT = struct.Struct("<3f")
_CAPSULE = struct.Struct("<7fB3x")          # 32 bytes
_HAND = struct.Struct("<4fBBHf3f")          # 36 bytes

CONTACT_FREE, CONTACT_CONSTRAINED, CONTACT_UNKNOWN = 0, 1, 2
CONTACT_NAMES = {0: "free", 1: "constrained", 2: "unknown"}
HAND_CLEAR, HAND_INTRUDING, HAND_CONSTRAINED = 0, 1, 2
# Capsule ids (robot_body::CapsuleId); NO_CAPSULE: none.
CAPSULE_NAMES = ["link2", "link3", "link4", "link5", "link5b", "link6", "link7", "link7b",
                 "hand", "fingers", "object", "reference"]
NO_CAPSULE = 255


def capsule_name(capsule_id: int) -> str:
    return CAPSULE_NAMES[capsule_id] if capsule_id < len(CAPSULE_NAMES) else "none"


@dataclass
class OverlayCapsule:
    a: np.ndarray       # (3,) segment end, robot base frame [m]
    b: np.ndarray       # (3,)
    radius: float
    id: int             # index into CAPSULE_NAMES


@dataclass
class OverlayHand:
    center: np.ndarray  # (3,) robot base frame [m]
    radius: float
    status: int         # HAND_*
    capsule: int        # nearest capsule id over the horizon, NO_CAPSULE if none
    exposed: int        # bit i: exposed to capsule id i over the horizon
    clearance: float    # [m] from the intrusion boundary to that capsule's surface, < 0: exposed
    closest: np.ndarray  # (3,) nearest robot surface point


@dataclass
class Overlay:
    seq: int
    t_send_ns: int
    valid: bool
    moving: bool
    holding_object: bool
    slowed: bool
    unlocated: bool
    contact: int
    cap_fraction: float
    reference: np.ndarray                 # (3,)
    limiting_capsule: int                 # capsule id that set the slowdown, NO_CAPSULE if none
    limiting_point: np.ndarray            # (3,) contact point that set it
    predicted: np.ndarray                 # (n, 3)
    capsules: list = field(default_factory=list)
    hands: list = field(default_factory=list)
    t_rx: float = 0.0                     # time.monotonic() at reception


def parse(buf: bytes):
    """Overlay, or None for a malformed packet or another version."""
    if len(buf) < _HEADER.size:
        return None
    (magic, version, flags, contact, seq, t_send_ns, cap, rx, ry, rz, n_points, n_hands,
     n_capsules, limiting, lx, ly, lz) = _HEADER.unpack_from(buf, 0)
    if magic != MAGIC or version != VERSION:
        return None
    if len(buf) != (_HEADER.size + n_points * _POINT.size + n_capsules * _CAPSULE.size +
                    n_hands * _HAND.size):
        return None
    off = _HEADER.size
    pts = np.frombuffer(buf, np.float32, n_points * 3, off).reshape(n_points, 3).astype(np.float64)
    off += n_points * _POINT.size
    capsules = []
    for _ in range(n_capsules):
        ax, ay, az, bx, by, bz, r, cid = _CAPSULE.unpack_from(buf, off)
        capsules.append(OverlayCapsule(np.array([ax, ay, az]), np.array([bx, by, bz]), r, cid))
        off += _CAPSULE.size
    hands = []
    for _ in range(n_hands):
        x, y, z, r, status, cid, exposed, clearance, px, py, pz = _HAND.unpack_from(buf, off)
        hands.append(OverlayHand(np.array([x, y, z]), r, status, cid, exposed, clearance,
                                 np.array([px, py, pz])))
        off += _HAND.size
    return Overlay(seq, t_send_ns, bool(flags & 1), bool(flags & 2), bool(flags & 4),
                   bool(flags & 8), bool(flags & 16), contact, cap, np.array([rx, ry, rz]),
                   limiting, np.array([lx, ly, lz]), pts, capsules, hands, time.monotonic())


class OverlayReceiver:
    """Keeps the latest overlay packet from a background thread."""

    def __init__(self, port: int, host: str = "127.0.0.1"):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((host, port))
        self._sock.settimeout(0.2)
        self._lock = threading.Lock()
        self._latest = None
        self._stop = False
        self.received = 0
        self.bad = 0
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop:
            try:
                buf = self._sock.recv(8192)
            except socket.timeout:
                continue
            except OSError:
                break
            o = parse(buf)
            if o is None:
                self.bad += 1
                continue
            with self._lock:
                self._latest = o
                self.received += 1

    def latest(self, max_age_s: float = 0.3):
        with self._lock:
            o = self._latest
        if o is None or time.monotonic() - o.t_rx > max_age_s:
            return None
        return o

    def close(self):
        self._stop = True
        self._thread.join(timeout=1.0)
        self._sock.close()

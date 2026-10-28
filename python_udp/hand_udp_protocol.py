"""
hand_udp_protocol.py -- wire format for streaming hand keypoints over UDP.

Shared by the sender (hand_detect_udp.py, fake_hand_sender.py) and the debug
listener (udp_listener.py). Contains the wire format plus a ready-to-use 
HandUdpSender and HandUdpReceiver, so copying this one file is enough to 
reuse both sides. The C++ side mirrors it in cpp/hand_udp_protocol.hpp. 
If ANYTHING changes here, bump VERSION and change the header too.

Layout (all little-endian, every field naturally aligned -> a plain C struct
matches it without #pragma pack):

  Header, 32 bytes
    off  type      name
     0   char[4]   magic        b"HAND"
     4   uint8     version      1
     5   uint8     n_hands      0..MAX_HANDS   (0 is a valid "no hands" heartbeat)
     6   uint8     flags        bit0 = FLAG_HAND_UNLOCATED
     7   uint8     frame_id     0 = camera_color_optical, 1 = robot base
     8   uint32    seq          +1 per packet, wraps at 2^32
    12   uint32    reserved     0
    16   int64     t_capture_ns CLOCK_REALTIME ns when the image was taken
    24   int64     t_send_ns    CLOCK_REALTIME ns right before sendto()

  Then n_hands x Hand, 24 bytes each
     0   uint8     handedness   0 = left, 1 = right, 255 = unknown (MediaPipe label!)
     1   uint8[3]  placeholder  Only there to align the following floats for memcpy in c++   
     4   float32   score        handedness confidence 0..1
     8   float32   s_radius     Radius of the sphere modelling the hand. Depending on 
                                which landmark we detect, the radius has to be bigger
     12   float32[3]  xyz        Center of the sphere in frame_id [meters]

Semantics the receiver MUST respect (fail-safe design):
  * A packet is sent for EVERY camera frame, also with n_hands = 0. Silence
    therefore means "detector dead / cable / crash" -> treat as UNSAFE.
  * n_hands = 0 means "no hand VISIBLE", not "no hand present" (occlusion!).
  * FLAG_HAND_UNLOCATED: a hand was seen but has no usable 3D point at all.
    Its position is unknown -> treat as close / unsafe.
  * Timestamps are for latency diagnostics. Use the RECEIVER's own monotonic
    clock + seq for the watchdog, never the sender's clock.
"""
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

MAGIC = b"HAND"
VERSION = 1
N_LANDMARKS = 1 # Could be changed to more landmarks if we want but currently only 1 landmark is sent + safety_radius

FLAG_HAND_UNLOCATED = 1 << 0

FRAME_CAMERA = 0      # camera_color_optical_frame: x right, y down, z forward
FRAME_ROBOT_BASE = 1  # after applying the camera->robot extrinsic calibration
FRAME_NAMES = {FRAME_CAMERA: "camera", FRAME_ROBOT_BASE: "robot_base"}

HAND_LEFT, HAND_RIGHT, HAND_UNKNOWN = 0, 1, 255
HANDEDNESS_NAMES = {HAND_LEFT: "L", HAND_RIGHT: "R", HAND_UNKNOWN: "?"}

HEADER = struct.Struct("<4sBBBBIIqq")   # 32 bytes
HAND_HEADER = struct.Struct("<B3xff")     # 12 bytes, followed by 3 float32 for the coordinates
HAND_SIZE = HAND_HEADER.size + N_LANDMARKS * 3 * 4   # 24 (the "*4" is there because a float is 4 bytes)

# Ethernet MTU 1500 - 20 (IPv4) - 8 (UDP) = 1472 bytes of payload per frame.
# We want the packet to be smaller than that, else it would be split into IP fragments, and
# losing ONE fragment drops the whole datagram. On loopback (MTU 65536) it would
# not matter, but this keeps the format valid if the receiver moves to another PC.
MAX_HANDS = 6
MAX_PACKET_SIZE = HEADER.size + MAX_HANDS * HAND_SIZE

assert HEADER.size == 32 and HAND_SIZE == 24 and MAX_PACKET_SIZE <= 1472


@dataclass
class Hand:
    handedness: int = HAND_UNKNOWN
    score: float = 0.0
    r_sphere: Optional[float] = None
    # (1, 3) float32, metres; rows of NaN where depth was missing
    points: np.ndarray = field(
        default_factory=lambda: np.full((N_LANDMARKS, 3), np.nan, np.float32))

    @property
    def n_valid(self) -> int:
        return int(np.count_nonzero(~np.isnan(self.points[:, 0])))


@dataclass
class Packet:
    seq: int
    flags: int
    frame_id: int
    t_capture_ns: int
    t_send_ns: int
    hands: List[Hand]


def handedness_from_label(label: str) -> int:
    return {"left": HAND_LEFT, "right": HAND_RIGHT}.get(label.lower(), HAND_UNKNOWN)


def pack(seq: int, t_capture_ns: int, hands: Sequence[Hand],
         frame_id: int = FRAME_CAMERA, t_send_ns: Optional[int] = None) -> bytes:
    if len(hands) > MAX_HANDS:
        raise ValueError(f"{len(hands)} hands > MAX_HANDS={MAX_HANDS}")
    flags = 0
    body = []
    for h in hands:
        pts = np.ascontiguousarray(h.points, dtype="<f4")
        if pts.shape != (N_LANDMARKS, 3):
            raise ValueError(f"points must be (1, 3), got {pts.shape} with value {pts}")
        n_valid = h.n_valid
        if n_valid == 0:
            flags |= FLAG_HAND_UNLOCATED
        body.append(HAND_HEADER.pack(h.handedness & 0xFF, float(h.score), float(h.r_sphere)))
        body.append(pts.tobytes())
    if t_send_ns is None:
        t_send_ns = time.time_ns()
    head = HEADER.pack(MAGIC, VERSION, len(hands), flags, frame_id,
                       seq & 0xFFFFFFFF, 0, int(t_capture_ns), int(t_send_ns))
    return head + b"".join(body)


def unpack(buf: bytes) -> Packet:
    """Parse and validate a datagram. Raises ValueError on anything malformed."""
    if len(buf) < HEADER.size:
        raise ValueError(f"short packet ({len(buf)} B)")
    magic, ver, n, flags, frame_id, seq, _res, t_cap, t_send = HEADER.unpack_from(buf, 0)
    if magic != MAGIC:
        raise ValueError(f"bad magic {magic!r}")
    if ver != VERSION:
        raise ValueError(f"version {ver}, expected {VERSION}")
    if n > MAX_HANDS:
        raise ValueError(f"n_hands={n} > {MAX_HANDS}")
    expected = HEADER.size + n * HAND_SIZE
    if len(buf) != expected:
        raise ValueError(f"length {len(buf)} != expected {expected} for {n} hands")
    hands = []
    off = HEADER.size
    for _ in range(n):
        hd, score, r_sphere = HAND_HEADER.unpack_from(buf, off)
        pts = np.frombuffer(buf, dtype="<f4", count=N_LANDMARKS * 3,
                            offset=off + HAND_HEADER.size).reshape(N_LANDMARKS, 3).copy()
        hands.append(Hand(hd, score, r_sphere, pts))
        off += HAND_SIZE
    return Packet(seq, flags, frame_id, t_cap, t_send, hands)


def parse_dest(s: str) -> Tuple[str, int]:
    host, _, port = s.rpartition(":")
    if not host or not port.isdigit():
        raise ValueError(f"destination must look like host:port, got {s!r}")
    return host, int(port)


class HandUdpSender:
    """Fire-and-forget UDP sender. Never blocks and never raises in send():
    the vision loop must not stall because nobody is listening."""

    def __init__(self, dests: Sequence[Tuple[str, int]], frame_id: int = FRAME_CAMERA):
        self.dests = list(dests)
        self.frame_id = frame_id
        self.seq = 0
        self.n_errors = 0
        self.last_error: Optional[str] = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)

    def send(self, hands: Sequence[Hand], t_capture_ns: int) -> int:
        """Send one packet to every destination. Returns the seq that was used."""
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        data = pack(seq, t_capture_ns, hands, self.frame_id)
        for d in self.dests:
            try:
                self.sock.sendto(data, d)
            except OSError as e:   # BlockingIOError, ECONNREFUSED, ENETUNREACH, ...
                self.n_errors += 1
                self.last_error = f"{d[0]}:{d[1]}: {e}"
        return seq

    def close(self) -> None:
        self.sock.close()


RESTART_JUMP = 1000   # seq jumping back by more than this = sender restarted


class HandUdpReceiver:
    """Receives the hand stream in a background thread and keeps the latest packet.

    The caller polls latest() / is_stale, or passes on_packet to process every
    accepted packet (called from the receiver thread -- keep it short and lock
    any state it shares with other threads). Duplicates and packets older than
    the last accepted one are dropped, so on_packet never goes back in time.
    """

    def __init__(self, bind: str = "127.0.0.1", port: int = 5005, timeout_s: float = 0.2,
                 on_packet: Optional[Callable[[Packet, int], None]] = None):
        self.addr = (bind, port)
        self.timeout_s = timeout_s
        self.on_packet = on_packet
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        try:
            self.sock.bind(self.addr)
        except OSError as e:
            self.sock.close()
            raise OSError(f"cannot bind {bind}:{port}: {e}. Another program (udp_listener.py, "
                          "the C++ receiver?) probably owns this port. Let the sender stream "
                          "to a second port with --dest.") from e
        self.sock.settimeout(0.05)   # lets the thread notice stop() without a packet arriving
        self._lock = threading.Lock()
        self._latest: Optional[Tuple[Packet, int]] = None
        self._last_seq: Optional[int] = None
        self._t_last_rx: Optional[float] = None   # monotonic, for the watchdog
        self._stats = dict(rx=0, lost=0, reordered=0, bad=0)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.sock.close()

    def latest(self) -> Optional[Tuple[Packet, int]]:
        """(last accepted Packet, its receive time in CLOCK_REALTIME ns), or None."""
        with self._lock:
            return self._latest

    @property
    def is_stale(self) -> bool:
        """True if nothing arrived yet or nothing within timeout_s -> treat as UNSAFE.
        Uses the receiver's own monotonic clock, never the sender's timestamps."""
        with self._lock:
            t = self._t_last_rx
        return t is None or time.monotonic() - t > self.timeout_s

    @property
    def stats(self) -> dict:
        with self._lock:
            return dict(self._stats)

    def _accept_seq(self, seq: int) -> bool:
        """Sequence bookkeeping; False = duplicate or reordered packet, drop it."""
        if self._last_seq is not None:
            diff = (seq - self._last_seq) & 0xFFFFFFFF
            if diff == 0 or diff >= 0x80000000:                     # duplicate or older
                if ((self._last_seq - seq) & 0xFFFFFFFF) <= RESTART_JUMP:
                    self._stats["reordered"] += 1
                    return False
                # jumped far back: sender restarted, start counting again
            elif diff > 1:
                self._stats["lost"] += diff - 1
        self._last_seq = seq
        return True

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data, _addr = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break   # socket closed
            t_recv_ns = time.time_ns()
            try:
                pkt = unpack(data)
            except ValueError:
                with self._lock:
                    self._stats["bad"] += 1
                continue
            with self._lock:
                now = time.monotonic()
                # After a silence no reordering is possible: accept any seq as a fresh
                # start (a sender restarted early would otherwise be dropped as "old").
                if self._t_last_rx is None or now - self._t_last_rx > self.timeout_s:
                    self._last_seq = None
                # Only well-formed packets feed the watchdog: garbage on the port
                # must not make a dead sender look alive.
                self._t_last_rx = now
                if not self._accept_seq(pkt.seq):
                    continue
                self._stats["rx"] += 1
                self._latest = (pkt, t_recv_ns)
            if self.on_packet is not None:
                self.on_packet(pkt, t_recv_ns)

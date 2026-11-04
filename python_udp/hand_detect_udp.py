#!/usr/bin/env python3
"""
Hand detection (Intel RealSense D455 + MediaPipe HandLandmarker) that streams the
3D hand keypoints over UDP to a non-ROS consumer (e.g. a libfranka/FCI C++ program).

Pipeline per frame:
  1. Grab colour + depth from the D455, align depth to the colour image.
  2. Run MediaPipe HandLandmarker on the colour image -> 21 2D landmarks / hand.
  3. Look up depth around each landmark (robust patch statistic, not one pixel).
  4. Deproject pixel + depth -> 3D point in the colour camera frame (metres).
  5. Optionally transform into the robot base frame (--extrinsic).
  6. Send ONE UDP packet per frame (also when no hand is visible = heartbeat).
     Wire format: hand_udp_protocol.py / cpp/hand_udp_protocol.hpp.

Usage:
    python3 udp_examples/hand_detect_udp.py                                # -> 127.0.0.1:5005
    python3 udp_examples/hand_detect_udp.py --dest 127.0.0.1:5005 --dest 127.0.0.1:5006
    python3 udp_examples/hand_detect_udp.py --dest 192.168.10.2:5005 --no-gui
    python3 udp_examples/hand_detect_udp.py --extrinsic T_base_cam.txt     # 4x4, metres

Debug:   python3 udp_examples/udp_listener.py --port 5005     (in a second terminal)

Robot overlay (with --extrinsic): --robot-port 5006 receives pick_place's supervisor view
(robot_overlay_protocol.py, version 2) and draws it into the image:
  * the robot's capsules (links, hand, fingers, held object) as the supervisor models them: grey,
    orange where an intruding hand is exposed to them, magenta for a constrained contact
    (--no-capsules: off),
  * the robot's reference point (red) and its predicted path over the next 0.5 s as a
    translucent tube (yellow, orange while slowed down),
  * the hand status (skeleton orange: intrudes, free contact; magenta: intrudes near the table
    while the robot moves down = constrained contact) with a line to the nearest robot point,
  * the contact point that set the slowdown (white-ringed dot) and its capsule in the status text.
--record out.mp4 writes the annotated colour image to a video, also with --no-gui.

Install (Python 3.10-3.12): pyrealsense2 mediapipe opencv-python numpy
hand_udp_protocol.py must be in the same directory. Run from the repo root.

Keys:  q / ESC = quit (GUI mode), Ctrl+C otherwise
NOTE: the camera can only be opened by ONE process. Stop realsense-viewer or
the realsense2_camera ROS node before running this.
"""
import argparse
import shutil
import subprocess
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision

import hand_udp_protocol as hup                     # python udp_examples/<script>.py
import robot_overlay_protocol as rop

# --------------------------------------------------------------------------- config
MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
             "hand_landmarker/float16/latest/hand_landmarker.task")
MODEL_PATH = Path(__file__).resolve().parent.parent / "models" / "hand_landmarker.task"

WIDTH, HEIGHT, FPS = 640, 480, 30   # same size for colour & depth keeps alignment simple
NUM_HANDS = 4           # 2 people. Must be <= hup.MAX_HANDS (5) so a packet fits one Ethernet frame
PATCH_HALF = 2          # depth window = (2*PATCH_HALF+1)^2 pixels around each landmark
DEPTH_PERCENTILE = 25   # <50 biases towards the NEAR surface (the hand, not the background)
MAX_VALID_DEPTH_M = 4.0 # ignore depth readings further than this (noise / background)

# Radius of the safety sphere
R_SPHERE_BEST = 0.1
R_SPHERE_MIDDLE = 0.13
R_SPHERE_MAX = 0.18

assert NUM_HANDS <= hup.MAX_HANDS, "more hands than fit into one UDP packet"

# The 21-point hand skeleton (MediaPipe landmark indices).
# 0 = wrist, 4 = thumb tip, 8 = index tip, 12 = middle tip, 16 = ring tip, 20 = pinky tip
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),            # thumb
    (0, 5), (5, 6), (6, 7), (7, 8),            # index
    (5, 9), (9, 10), (10, 11), (11, 12),       # middle
    (9, 13), (13, 14), (14, 15), (15, 16),     # ring
    (13, 17), (0, 17), (17, 18), (18, 19), (19, 20),  # pinky + palm
]


def parse_args():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--dest", action="append", default=None, metavar="HOST:PORT",
                   help="UDP destination, repeatable (default 127.0.0.1:5005). One unicast "
                        "port can only be read by ONE program, so give the debug listener "
                        "its own port if the robot program is also running.")
    p.add_argument("--extrinsic", type=Path, default=None,
                   help="4x4 homogeneous T_base_cam (text, np.loadtxt format). Points are "
                        "then sent in the robot base frame (frame_id=1) instead of the camera frame.")
    p.add_argument("--robot-port", type=int, default=None,
                   help="UDP port for the robot overlay from pick_place (e.g. 5006); needs "
                        "--extrinsic to project robot base frame points into the image")
    p.add_argument("--tube-radius", type=float, default=0.03, metavar="M",
                   help="radius of the drawn predicted-path tube [m] (default 0.03)")
    p.add_argument("--no-capsules", action="store_true",
                   help="do not draw the robot's capsules")
    p.add_argument("--record", type=Path, default=None, metavar="OUT.mp4",
                   help="write the annotated colour image to this video file")
    p.add_argument("--no-gui", action="store_true",
                   help="no OpenCV window (saves a few ms per frame, needed when headless)")
    return p.parse_args()


def load_extrinsic(path: Path) -> np.ndarray:
    T = np.loadtxt(path, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"{path}: expected a 4x4 matrix, got {T.shape}")
    R = T[:3, :3]
    # A wrong file (degrees instead of radians, transposed, mm instead of m) is
    # a classic silent error -> sanity-check the rotation part.
    if not np.allclose(R @ R.T, np.eye(3), atol=1e-3) or not np.isclose(np.linalg.det(R), 1.0, atol=1e-3):
        raise ValueError(f"{path}: rotation part is not orthonormal (det={np.linalg.det(R):.4f})")
    if np.linalg.norm(T[:3, 3]) > 10.0:
        raise ValueError(f"{path}: translation {T[:3, 3]} > 10 m -- millimetres instead of metres?")
    return T


def ensure_model() -> None:
    if not MODEL_PATH.exists():
        print(f"Downloading hand landmarker model to {MODEL_PATH} ...")
        urllib.request.urlretrieve(MODEL_URL, MODEL_PATH)


def sample_depth(depth_m: np.ndarray, u: int, v: int):
    """Robust depth (metres) around pixel (u, v); None if not enough valid pixels.

    Why a patch and not a single pixel: landmarks sit on finger edges and the
    D455 depth has holes (value 0) and 'flying pixels' at object borders. A
    single pixel frequently hits the background behind a finger, which would make
    the hand appear much further away than it is -- the dangerous direction.
    """
    h, w = depth_m.shape
    patch = depth_m[max(v - PATCH_HALF, 0):min(v + PATCH_HALF + 1, h),
                    max(u - PATCH_HALF, 0):min(u + PATCH_HALF + 1, w)]
    valid = patch[(patch > 0) & (patch < MAX_VALID_DEPTH_M)]
    if valid.size < 3:
        return None
    return float(np.percentile(valid, DEPTH_PERCENTILE))


# Robot overlay colours (BGR).
COLOR_TIP = (0, 0, 255)            # red: reference point (EE, or the held object's tip)
COLOR_PATH = (0, 255, 255)         # yellow: predicted path
COLOR_PATH_SLOWED = (0, 140, 255)  # orange: predicted path while slowed down
COLOR_HAND = {rop.HAND_INTRUDING: (0, 165, 255),     # orange: intrudes, free contact
              rop.HAND_CONSTRAINED: (255, 0, 255)}   # magenta: constrained contact
HAND_MATCH_M = 0.15   # a detected hand is the robot's hand when their centres are this close


TUBE_ALPHA = 0.4      # opacity of the predicted-path tube
COLOR_CAPSULE = (160, 160, 160)  # grey: robot capsule no hand is exposed to
CAPSULE_ALPHA = 0.25             # opacity of the robot's capsules


def project(T_cam_base: np.ndarray, intr, p_base: np.ndarray, radius: float = 0.0):
    """Pixel (u, v) of a robot base frame point, None if behind the camera; with `radius`
    [m], ((u, v), pixel radius of a sphere of that radius there)."""
    p = T_cam_base[:3, :3] @ p_base + T_cam_base[:3, 3]
    if p[2] < 0.05:
        return None
    u, v = rs.rs2_project_point_to_pixel(intr, [float(p[0]), float(p[1]), float(p[2])])
    uv = int(round(u)), int(round(v))
    if not radius:
        return uv
    return uv, max(1, int(round(intr.fx * radius / p[2])))


def add_tube(mask: np.ndarray, circles) -> None:
    """Adds the tube through projected spheres [((u, v), r), ...] to a mask: a disc per sphere
    and the trapezoid between neighbours."""
    for (c, r) in circles:
        cv2.circle(mask, c, r, 255, -1, cv2.LINE_AA)
    for (c0, r0), (c1, r1) in zip(circles, circles[1:]):
        d = np.subtract(c1, c0).astype(np.float64)
        n = np.linalg.norm(d)
        if n < 1e-6:
            continue
        nrm = np.array([-d[1], d[0]]) / n
        quad = np.array([c0 + nrm * r0, c1 + nrm * r1, c1 - nrm * r1, c0 - nrm * r0])
        cv2.fillConvexPoly(mask, np.round(quad).astype(np.int32), 255, cv2.LINE_AA)


def blend_mask(bgr: np.ndarray, mask: np.ndarray, color, alpha: float = TUBE_ALPHA) -> None:
    """Blends `color` into the masked area once, so that overlaps do not darken, and outlines it."""
    a = (mask.astype(np.float32) * (alpha / 255.0))[..., None]
    bgr[:] = (bgr * (1.0 - a) + np.array(color, np.float32) * a).astype(np.uint8)
    contours, _ = cv2.findContours((mask > 127).astype(np.uint8), cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(bgr, contours, -1, color, 1, cv2.LINE_AA)


def draw_tube(bgr: np.ndarray, circles, color) -> None:
    """Translucent tube through projected spheres [((u, v), r), ...]."""
    mask = np.zeros(bgr.shape[:2], np.uint8)
    add_tube(mask, circles)
    blend_mask(bgr, mask, color)


def draw_capsules(bgr: np.ndarray, overlay, T_cam_base: np.ndarray, intr) -> None:
    """The robot's capsules, coloured by the worst status of the hands exposed to them."""
    worst = {}  # capsule id -> worst status of a hand exposed to it
    for h in overlay.hands:
        if h.status == rop.HAND_CLEAR:
            continue
        for cid in range(len(rop.CAPSULE_NAMES)):
            if h.exposed & (1 << cid):
                worst[cid] = max(worst.get(cid, rop.HAND_CLEAR), h.status)
    masks = {}
    for c in overlay.capsules:
        # The reference point (radius 0, without the link capsules) is drawn as a small ball.
        ends = [project(T_cam_base, intr, p, max(c.radius, 0.01)) for p in (c.a, c.b)]
        if any(e is None for e in ends):
            continue
        status = worst.get(c.id, rop.HAND_CLEAR)
        if status not in masks:
            masks[status] = np.zeros(bgr.shape[:2], np.uint8)
        add_tube(masks[status], ends)
    for status, mask in sorted(masks.items()):
        blend_mask(bgr, mask, COLOR_HAND.get(status, COLOR_CAPSULE),
                   CAPSULE_ALPHA if status == rop.HAND_CLEAR else TUBE_ALPHA)


def hand_status(overlay, center_base: np.ndarray) -> int:
    """Status the robot gave the hand nearest to this detected hand (its previous frame)."""
    if overlay is None or not overlay.hands or not np.all(np.isfinite(center_base)):
        return rop.HAND_CLEAR
    d = [np.linalg.norm(h.center - center_base) for h in overlay.hands]
    k = int(np.argmin(d))
    return overlay.hands[k].status if d[k] < HAND_MATCH_M else rop.HAND_CLEAR


def draw_robot(bgr: np.ndarray, overlay, T_cam_base: np.ndarray, intr,
               tube_radius: float, capsules: bool = True) -> None:
    if overlay is None:
        cv2.putText(bgr, "robot: no overlay", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 0, 255), 2, cv2.LINE_AA)
        return
    if not overlay.valid:
        return
    if capsules:
        draw_capsules(bgr, overlay, T_cam_base, intr)
    if overlay.moving and len(overlay.predicted):
        circles = [project(T_cam_base, intr, p, tube_radius) for p in overlay.predicted]
        circles = [c for c in circles if c is not None]
        if circles:
            draw_tube(bgr, circles, COLOR_PATH_SLOWED if overlay.slowed else COLOR_PATH)
    tip = project(T_cam_base, intr, overlay.reference)
    if tip is not None:
        cv2.circle(bgr, tip, 7, COLOR_TIP, -1, cv2.LINE_AA)
        cv2.circle(bgr, tip, 8, (255, 255, 255), 1, cv2.LINE_AA)
    # Each intruding hand: a line to the nearest robot surface point over the horizon.
    for h in overlay.hands:
        if h.status == rop.HAND_CLEAR or h.capsule == rop.NO_CAPSULE:
            continue
        a, b = project(T_cam_base, intr, h.center), project(T_cam_base, intr, h.closest)
        if a is not None and b is not None:
            cv2.line(bgr, a, b, COLOR_HAND[h.status], 1, cv2.LINE_AA)
    if overlay.slowed and overlay.limiting_capsule != rop.NO_CAPSULE:
        lp = project(T_cam_base, intr, overlay.limiting_point)
        if lp is not None:
            lcolor = COLOR_HAND[rop.HAND_CONSTRAINED] if overlay.contact != rop.CONTACT_FREE \
                else COLOR_PATH_SLOWED
            cv2.circle(bgr, lp, 5, lcolor, -1, cv2.LINE_AA)
            cv2.circle(bgr, lp, 6, (255, 255, 255), 1, cv2.LINE_AA)
    text = ("robot: " + ("moving" if overlay.moving else "at rest") +
            ("  object tip" if overlay.holding_object else "  EE"))
    color = (0, 255, 0)
    if overlay.slowed:
        text += (f"  SLOWED {overlay.cap_fraction * 100:.0f}%  "
                 f"{rop.CONTACT_NAMES.get(overlay.contact, '?')} contact"
                 f"  {rop.capsule_name(overlay.limiting_capsule)}")
        color = COLOR_HAND[rop.HAND_CONSTRAINED] if overlay.contact != rop.CONTACT_FREE \
            else COLOR_PATH_SLOWED
    if overlay.unlocated:
        text += "  hand position unknown"
    cv2.putText(bgr, text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)


class VideoWriter:
    """H.264 video through the system ffmpeg (crf 18: visually near lossless). OpenCV's own
    mp4v encoder blurs and blocks the 640x480 image badly."""

    def __init__(self, path: Path, w: int, h: int, fps: int):
        if shutil.which("ffmpeg") is None:
            raise SystemExit("--record needs ffmpeg (apt install ffmpeg)")
        self._proc = subprocess.Popen(
            ["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "bgr24",
             "-s", f"{w}x{h}", "-r", str(fps), "-i", "-", "-c:v", "libx264", "-preset",
             "veryfast", "-crf", "18", "-pix_fmt", "yuv420p", str(path)],
            stdin=subprocess.PIPE)

    def write(self, bgr: np.ndarray) -> None:
        self._proc.stdin.write(np.ascontiguousarray(bgr).tobytes())

    def release(self) -> None:
        self._proc.stdin.close()
        self._proc.wait()


def capture_time_ns(color_frame, arrival_ns: int) -> int:
    """Best estimate of when the image was exposed, in CLOCK_REALTIME ns.

    With the default 'global time' domain librealsense maps the camera's hardware
    clock onto host system time, so the frame timestamp includes exposure + USB
    transfer -- that is the latency that matters for safety. If the domain is
    hardware_clock (camera-internal, unrelated to host time) or the value looks
    implausible, fall back to the host arrival time (underestimates the age).
    """
    domain = color_frame.get_frame_timestamp_domain()
    if domain in (rs.timestamp_domain.global_time, rs.timestamp_domain.system_time):
        t_ns = int(color_frame.get_timestamp() * 1e6)   # ms -> ns
        if 0 <= arrival_ns - t_ns < 1_000_000_000:
            return t_ns
    return arrival_ns


def main() -> None:
    args = parse_args()
    dests = [hup.parse_dest(d) for d in (args.dest or ["127.0.0.1:5005"])]
    T_base_cam = load_extrinsic(args.extrinsic) if args.extrinsic else None
    frame_id = hup.FRAME_ROBOT_BASE if T_base_cam is not None else hup.FRAME_CAMERA
    sender = hup.HandUdpSender(dests, frame_id)
    print(f"Streaming to {', '.join(f'{h}:{p}' for h, p in dests)}  "
          f"frame={hup.FRAME_NAMES[frame_id]}")

    if args.robot_port is not None and T_base_cam is None:
        raise SystemExit("--robot-port needs --extrinsic (robot points are in the base frame)")
    T_cam_base = np.linalg.inv(T_base_cam) if T_base_cam is not None else None
    robot_rx = rop.OverlayReceiver(args.robot_port) if args.robot_port is not None else None
    if robot_rx is not None:
        print(f"Robot overlay from UDP port {args.robot_port}")
    writer = None

    ensure_model()

    # ---------------------------------------------------------------- RealSense
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    cfg.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    profile = pipeline.start(cfg)

    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()  # usually 0.001
    align = rs.align(rs.stream.color)  # reproject depth into the colour camera's pixel grid
    color_intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    print(f"Colour intrinsics: fx={color_intr.fx:.1f} fy={color_intr.fy:.1f} "
          f"cx={color_intr.ppx:.1f} cy={color_intr.ppy:.1f}  depth_scale={depth_scale}")

    # Let auto-exposure settle; the first frames are often dark / unstable.
    for _ in range(30):
        pipeline.wait_for_frames()

    # ---------------------------------------------------------------- MediaPipe
    options = vision.HandLandmarkerOptions(
        base_options=mp_python.BaseOptions(model_asset_path=str(MODEL_PATH)),
        running_mode=vision.RunningMode.VIDEO,   # uses tracking between frames -> faster, steadier
        num_hands=NUM_HANDS,
        min_hand_detection_confidence=0.5,
        min_hand_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    landmarker = vision.HandLandmarker.create_from_options(options)

    t0 = time.monotonic()
    last_ts_ms = -1
    fps_smooth = 0.0
    t_prev = time.perf_counter()

    try:
        while True:
            frames = align.process(pipeline.wait_for_frames())
            arrival_ns = time.time_ns()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue
            t_capture_ns = capture_time_ns(color_frame, arrival_ns)

            bgr = np.asanyarray(color_frame.get_data())
            depth_m = np.asanyarray(depth_frame.get_data()).astype(np.float32) * depth_scale
            h, w = depth_m.shape

            # VIDEO mode requires strictly increasing timestamps in ms.
            ts_ms = int((time.monotonic() - t0) * 1000)
            if ts_ms <= last_ts_ms:
                ts_ms = last_ts_ms + 1
            last_ts_ms = ts_ms

            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)  # MediaPipe expects RGB, OpenCV gives BGR
            t_inf = time.perf_counter()
            result = landmarker.detect_for_video(
                mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), ts_ms)
            infer_ms = (time.perf_counter() - t_inf) * 1000

            hands_out = []          # what goes into the UDP packet
            overlay = robot_rx.latest() if robot_rx is not None else None
            closest_overall = None
            for i, landmarks in enumerate(result.hand_landmarks):
                cat = result.handedness[i][0]
                pix = []
                pts_cam = np.full((21, 3), np.nan, np.float32)  # NaN = no depth
                for j, lm in enumerate(landmarks):
                    # Landmarks are normalised [0,1]; they CAN lie outside the image.
                    u, v = int(round(lm.x * w)), int(round(lm.y * h))
                    if not (0 <= u < w and 0 <= v < h):
                        pix.append(None)
                        continue
                    pix.append((u, v))
                    z = sample_depth(depth_m, u, v)
                    # NOTE: lm.z from MediaPipe is RELATIVE depth, not metres -> use the camera.
                    if z is not None:
                        pts_cam[j] = rs.rs2_deproject_pixel_to_point(
                            color_intr, [float(u), float(v)], z)

                valid_mask = ~np.isnan(pts_cam[:, 0])
                n_valid = int(valid_mask.sum())

                # Check for the point to send via udp. We want the MCP joint as the center of the hand
                # Alternatively, whichever hand landmark with a bigger safety sphere
                center_sphere = np.full((1, 3), np.nan, np.float32)
                r_sphere = np.nan
                middle_finger_landmark = 9
                middle_landmarks = [5, 13, 17]
                idx = None
                if valid_mask[middle_finger_landmark]:
                    idx, r_sphere = middle_finger_landmark, R_SPHERE_BEST
                elif valid_mask[middle_landmarks].any():
                    idx = next(lm for lm in middle_landmarks if valid_mask[lm])
                    r_sphere = R_SPHERE_MIDDLE
                elif n_valid:
                    idx = int(np.argmax(valid_mask))   # first valid landmark
                    r_sphere = R_SPHERE_MAX
                if idx is not None:
                    center_sphere = pts_cam[idx:idx + 1]   # slice keeps shape (1, 3)


                # ---- packet content (optionally in robot base frame)
                pts_out = center_sphere.copy()
                if T_base_cam is not None and n_valid:
                    p = center_sphere.astype(np.float64)
                    pts_out = (p @ T_base_cam[:3, :3].T + T_base_cam[:3, 3]).astype(np.float32)

                hands_out.append(hup.Hand(hup.handedness_from_label(cat.category_name),
                                          cat.score, r_sphere, pts_out))

                # ---- drawing (red joints = no valid depth there, blue = landmark sent via UDP)
                # Skeleton white, or in the robot's status colour for this hand.
                hstatus = (hand_status(overlay, pts_out[0].astype(np.float64))
                           if T_base_cam is not None and n_valid else rop.HAND_CLEAR)
                bone = COLOR_HAND.get(hstatus, (255, 255, 255))
                for a, b in HAND_CONNECTIONS:
                    if pix[a] and pix[b]:
                        cv2.line(bgr, pix[a], pix[b], bone, 4 if hstatus else 2)
                for j, (p, ok) in enumerate(zip(pix, valid_mask)):
                    if p:
                        if j == idx:
                            color = (255, 0, 0)   # BGR blue
                        else:
                            color = (0, 200, 0) if ok else (0, 0, 255)
                        cv2.circle(bgr, p, 4, color, -1)

                label = f"{cat.category_name} {cat.score:.2f}  depth ok {n_valid}/21"
                if hstatus == rop.HAND_CONSTRAINED:
                    label = "CONSTRAINED  " + label
                elif hstatus == rop.HAND_INTRUDING:
                    label = "INTRUDING  " + label
                if n_valid:
                    dists = np.linalg.norm(pts_cam[valid_mask], axis=1)  # to camera, for display only
                    dist = float(dists.min())
                    label += f"  closest {dist:.2f} m"
                    if closest_overall is None or dist < closest_overall:
                        closest_overall = dist
                anchor = pix[0] or next((p for p in pix if p), (10, 60))
                cv2.putText(bgr, label, (anchor[0], max(anchor[1] - 15, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

            # ---- send: EVERY frame, also with zero hands (= heartbeat for the watchdog)
            seq = sender.send(hands_out, t_capture_ns)
            age_ms = (time.time_ns() - t_capture_ns) / 1e6   # image age at send time

            # timing overlay
            now = time.perf_counter()
            fps_smooth = 0.9 * fps_smooth + 0.1 * (1.0 / max(now - t_prev, 1e-6))
            t_prev = now
            status = (f"{fps_smooth:4.1f} FPS  inf {infer_ms:4.1f} ms  age {age_ms:5.1f} ms  "
                      f"hands {len(hands_out)}  seq {seq}")
            if sender.n_errors:
                status += f"  send errors {sender.n_errors}"

            if robot_rx is not None:
                draw_robot(bgr, overlay, T_cam_base, color_intr, args.tube_radius,
                           not args.no_capsules)
            if not args.no_gui or args.record is not None:
                cv2.putText(bgr, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (0, 255, 0), 2, cv2.LINE_AA)
            if args.record is not None:
                cv2.putText(bgr, time.strftime("%H:%M:%S") + f"  seq {seq}", (10, h - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                if writer is None:
                    writer = VideoWriter(args.record, w, h, FPS)
                writer.write(bgr)

            if not args.no_gui:
                # Colourised depth next to RGB: you will SEE holes (black) around hands.
                depth_vis = cv2.applyColorMap(
                    cv2.convertScaleAbs(np.clip(depth_m, 0, MAX_VALID_DEPTH_M),
                                        alpha=255 / MAX_VALID_DEPTH_M),
                    cv2.COLORMAP_JET)
                depth_vis[depth_m == 0] = 0
                cv2.imshow("hands (left: RGB, right: aligned depth)", np.hstack([bgr, depth_vis]))
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break

            line = status
            if closest_overall is not None:
                line += f"  closest-to-cam {closest_overall:.3f} m"
            print(line.ljust(110), end="\r")
    except KeyboardInterrupt:
        pass
    finally:
        print()
        if sender.last_error:
            print(f"last send error: {sender.last_error}")
        sender.close()
        if writer is not None:
            writer.release()
            print(f"video: {args.record}")
        if robot_rx is not None:
            print(f"robot overlay: {robot_rx.received} packets, {robot_rx.bad} malformed")
            robot_rx.close()
        landmarker.close()
        pipeline.stop()
        if not args.no_gui:   # would raise with opencv-python-headless
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

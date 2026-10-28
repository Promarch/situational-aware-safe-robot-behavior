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

Install (Python 3.10-3.12): pyrealsense2 mediapipe opencv-python numpy
hand_udp_protocol.py must be in the same directory. Run from the repo root.

Keys:  q / ESC = quit (GUI mode), Ctrl+C otherwise
NOTE: the camera can only be opened by ONE process. Stop realsense-viewer or
the realsense2_camera ROS node before running this.
"""
import argparse
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
                for a, b in HAND_CONNECTIONS:
                    if pix[a] and pix[b]:
                        cv2.line(bgr, pix[a], pix[b], (255, 255, 255), 2)
                for j, (p, ok) in enumerate(zip(pix, valid_mask)):
                    if p:
                        if j == idx:
                            color = (255, 0, 0)   # BGR blue
                        else:
                            color = (0, 200, 0) if ok else (0, 0, 255)
                        cv2.circle(bgr, p, 4, color, -1)

                label = f"{cat.category_name} {cat.score:.2f}  depth ok {n_valid}/21"
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

            if not args.no_gui:
                cv2.putText(bgr, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (0, 255, 0), 2, cv2.LINE_AA)
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
        landmarker.close()
        pipeline.stop()
        if not args.no_gui:   # would raise with opencv-python-headless
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

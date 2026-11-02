# Situational-aware safe robot behavior — robot control

Pick and place on a Franka Emika Panda (libfranka 0.9.2) that slows down when a human hand
approaches its path. Hands come over UDP from the RealSense/MediaPipe detector on the
`udp-implementation` branch (hand_udp protocol: each hand is a sphere, center + radius).

## How it works

- **Motion**: joint-space S-curves in the air (all joints share one jerk-limited profile, so the
  path is a straight line in joint space), vertical Cartesian moves near the table.
- **Supervision**: for every camera frame, the EE positions over the next 0.5 s are predicted
  with the robot model. If one comes closer to a hand than its sphere radius + 1 cm, the speed is
  capped so that every predicted point satisfies the energy limit
  `v_safe = sqrt(2 S_lim / m_eff)`, `m_eff` being the effective mass in the motion direction.
- **Re-planning**: the 1 kHz control loop applies a cap by re-planning its S-curve from its own
  current state with the same jerk and acceleration, so the path never changes and the motion
  still stops exactly on its goal. The supervisor runs in its own thread; the two exchange data
  through lock-free seqlocks and never wait for each other.

## Layout

| Path | Contents |
|---|---|
| `src/s_curve.*` | 1-D and synchronized 7-axis S-curves, online re-planning (pure math) |
| `src/motion.*` | joint and vertical motions inside `robot.control()`, per-tick log |
| `src/supervision.*` | prediction, intrusion test, energy-limited speed, slow down / resume |
| `src/hand_receiver.*` | UDP receiver thread with the protocol's fail-safe rules |
| `src/hand_udp_protocol.hpp` | wire format, copied from `udp-implementation` |
| `src/seqlock.h`, `src/poses.h`, `src/confirm.h` | thread handoff, fixed poses, operator prompt |
| `apps/pick_place.cpp` | the supervised pick and place task |
| `apps/record_pose.cpp` | record a joint pose with the tool snapped to straight down |
| `apps/calibrate_camera.cpp` | camera extrinsic `T_base_cam` from hand/EE point pairs |
| `apps/test_hand_receiver.cpp` | UDP reception check without the robot |
| `apps/read_state.cpp` | print robot and gripper state |
| `tests/` | offline tests of the S-curves and re-planning |
| `scripts/` | `replay_hands.py` (hands from CSV over UDP), plots, protocol module |
| `config/` | `T_base_cam.txt` (calibration in use), calibration pairs, replay schedule |

## Build

```bash
git clone --recursive <repo>      # or: git submodule update --init --recursive
mkdir build && cd build && cmake .. && make -j4
./test_s_curve && ./test_replan   # offline tests
```

## Run

```bash
# detector (udp-implementation checkout, Python 3.12 via uv)
uv run --python 3.12 python python_udp/hand_detect_udp.py --no-gui \
    --extrinsic <this repo>/config/T_base_cam.txt
# or, without a camera:  python3 scripts/replay_hands.py config/hands_replay.csv
cd build && ./pick_place [S_lim]
python3 ../scripts/plot_slowdown.py pick_place.csv
```

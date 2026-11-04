# Collision detection

How the robot notices a human hand, which situations it tells apart and what it does in each.
The supervisor tests the **whole robot**, modeled as capsules. The single-point test it used up to
commit `62aed97` is summarized at the end, under [Previous behavior](#previous-behavior-single-reference-point).

## Two layers

| Layer | Where | When it acts | What it covers |
|---|---|---|---|
| **Predictive supervisor** | [src/supervision.cpp](../src/supervision.cpp) | before contact, once per camera frame (~30 Hz) | the robot's capsules (links 2–7, hand, fingers, held object) against the detected hands |
| **Robot's contact reflex** | libfranka, set in [apps/pick_place.cpp](../apps/pick_place.cpp) | after contact, inside the robot (1 kHz) | the whole body, through the estimated external joint torques |

The supervisor never stops the robot. It only lowers the speed, so that a collision, if it
happens, transfers at most `S_lim` of kinetic energy. The reflex is the backstop once contact has
happened.

### Contact reflex (libfranka)

`robot.setCollisionBehavior(...)` is called once before the task:

- **Upper thresholds**:
  - joint torques of 3.0 / 5.0 / 2.5 / 2.5 / 1.0 / 2.0 / 1.0 Nm, which is 1.5 × the largest
    `|tau_ext|` of a full-speed run, rounded up;
  - a Cartesian force/torque of 10 N / 10 Nm.
- **Lower thresholds** (1.5 / 2.5 / 1.5 / 1.5 / 0.5 / 1.0 / 0.5 Nm, 5 N) only raise the robot's
  contact flags.
- If any estimated external torque or force goes over its upper threshold, the robot stops with a
  reflex:
  - `robot.control()` throws `franka::ControlException`.
  - `pick_place` catches it, runs `automaticErrorRecovery()` and still writes its logs.
  - The task is then reported as *not completed*.

## Predictive supervisor

### Inputs

**Hands**: the detector sends one UDP packet per camera frame
([hand_udp_protocol.hpp](../src/hand_udp_protocol.hpp)). Each hand is a *sphere*, a center in the
robot base frame plus a radius (0.10 / 0.13 / 0.18 m from the detector). The HandReceiver thread
([src/hand_receiver.cpp](../src/hand_receiver.cpp)) turns every packet into a
`HandObservation` and calls `Supervisor::onFrame` with it.

A hand is treated as **unlocated** (seen, but position unknown) when:

| Cause | Result |
|---|---|
| packet has the `HAND_UNLOCATED` flag | `unlocated = true` |
| a hand's center or radius is not finite | that hand dropped, `unlocated = true` |
| packet not in the robot base frame (sender without extrinsic calibration) and it has hands | `unlocated = true` |
| no valid packet for 100 ms (stream dead) | a synthetic frame with `unlocated = true` every 20 ms |

Duplicate and out-of-order packets are dropped. Malformed packets are counted and ignored, and they
do not keep a dead stream alive.

**Robot**: the 1 kHz control loop ([src/motion.cpp](../src/motion.cpp)) publishes a `Snapshot`
every tick: the running motion, its S-curve profile, the current joints and the motion's path.

**Held object**: `Shared::holding_object` is set by the main program after a successful grasp
and cleared after the release, both times with the robot at rest. While it is set, the object
is an extra capsule from the EE to `object_tip`, with radius `object_radius`. For the
screwdriver these are (0.15, 0, 0) m and 0.015 m.

### Robot model: capsules

[src/robot_body.h](../src/robot_body.h) models the robot as capsules. A capsule is a segment plus a
radius, fixed in one of libfranka's frames. Its pose comes from
`model.pose(Frame::kJoint1..kJoint7 / kFlange / kEndEffector, q)`. The dimensions are
franka_description's self-collision capsules:

| id | capsule | frame | radius [m] | group |
|---|---|---|---|---|
| 0–5 | link2, link3, link4, link5, link5b (rod), link6 | joint frames 2–6 | 0.06 (link5b 0.025, link6 0.05) | arm |
| 6, 7 | link7, link7b (side) | joint frame 7 | 0.04, 0.03 | tool |
| 8, 9 | hand, fingers (Franka Hand) | flange (hand turned −45° about z) | 0.04, 0.02 | tool |
| 10 | object (while held) | EE | `object_radius` | tool |
| 11 | reference point (only without `check_links` or `franka_hand`) | EE | 0 | tool |

- **Left out:** link 0 (the base does not move) and link 1 (it only spins about its own axis, so its
  capsule never moves toward anything).
- **Frame for the hand:** the hand capsules hang on the flange, so changing the end effector in
  Desk does not move them.
- **Groups:** the **tool** group is within about 20 cm of the EE, where the constrained-collision
  mass table was measured. The **arm** group (links 2–6) is not.
- **Checking:** `show_capsules` checks the capsules against the real robot (see
  [Checking](#checking)).

**Switches** (`supervision::Params`). They only change *which geometry is tested*, not the energy
model, the contact classes or the timing:

| Switch | Default | Effect |
|---|---|---|
| `check_links` | true | `false`: only the reference point (EE, or the object tip while held) with radius 0. That is exactly the test before the capsules. Use it for comparison runs only: links 2–7 and the gripper are then untested. |
| `franka_hand` | true | `false` (another tool): no hand or finger capsules. The reference point is tested instead. Add the tool's capsules in `fixedCapsules()`. |
| `link_safety_distance` | 0 | Added to every capsule radius (not to the reference point). It is for robot parts the capsules miss (cables, covers, a loose fit). `radius_margin` is the margin for the hand. Both add up in the intrusion distance. |

### Prediction

For every frame the supervisor predicts the joint configuration over the next `horizon` = 0.5 s,
one sample every `sample_dt` = 10 ms (51 samples), from the motion's current S-curve profile:

- **Joint motion** (`moveJointSCurve`): `q = q_start + delta · sigma(t)`, a straight line in joint
  space.
- **Vertical motion** (`moveVerticalTo`): from the current joints, the vertical EE line is followed
  in 2 mm resolved-rate steps (damped pseudo-inverse of the model's Jacobian).

At every sample all capsules are placed with the model.

### Intrusion test: which capsules a hand is exposed to

A hand is **exposed** to a capsule when, at some predicted sample, the capsule's surface comes
closer to the hand's sphere center than the sphere radius plus `radius_margin` (0.20 m):

```
|center_hand − closest point on the capsule axis| − r_capsule  <  r_hand + radius_margin
```

The supervisor records, for each hand:
- the set of capsules it is exposed to, which is what limits the speed;
- the nearest capsule over the horizon, its clearance and the nearest robot surface point, for the
  overlay and the logs.

### Scenarios

These are evaluated **per hand and capsule**. The hand status is what the overlay shows; the
contact decides the mass in the energy limit.

| # | Situation | Trigger | Hand status | Contact | Action |
|---|---|---|---|---|---|
| S0 | No hand | `n_hands == 0` and not unlocated | – | – | nominal speed; resume if slowed |
| S1 | Hand clear | no capsule comes within its intrusion distance over the horizon | `clear` | – | nominal speed; resume if slowed |
| S2 | Hand exposed, free contact | exposed to ≥ 1 capsule during a joint motion, an upward motion, or a downward motion with the hand center more than 5 cm above the table | `intruding` | **free** | cap from the exposed capsules |
| S3 | Hand trapped against the table | exposed during a **downward** vertical motion with the hand center at most `constrained_band` = 5 cm above `table_z` | `constrained` | **constrained** | cap from the exposed capsules |
| S4 | Hand position unknown | `unlocated` during a joint or upward motion | – | **free** | cap from **every** capsule |
| S5 | Hand position unknown, moving down | `unlocated` during a downward vertical motion | – | **unknown** | cap from **every** capsule |

A cap computed in S2–S5 may still be at or above the nominal speed. For example, a hand near the
slow links close to the base may need no slowdown at all. Its status still reads `intruding`, but
no command is sent.

### Mass per contact and capsule group

| Contact | Tool group (link7, link7b, hand, fingers, object) | Arm group (links 2–6) |
|---|---|---|
| free | `m_eff` at the contact point | `m_eff` at the contact point |
| constrained | tabulated mass at the EE position | `max(m_eff, largest tabulated mass)` |
| unknown | `max(m_eff, tabulated mass at the EE)` | `max(m_eff, largest tabulated mass)` |

- `m_eff = 1 / (u^T [J_p M^-1 J_p^T] u)`, with `J_p` the linear Jacobian of the contact point and
  `u` its direction of motion.
- The mass table ([src/mass_table.h](../src/mass_table.h),
  [config/constrained_collision_mass.csv](../config/constrained_collision_mass.csv)) is looked up
  at the EE position: radial distance `sqrt(x² + y²)` and height above the table.
- The table has no data for links 2–6, so they fall back to its largest value. That is
  conservative.

### Speed cap (energy limit)

At every predicted sample and for every exposed (hand, capsule) pair, the candidate contact points
lie on the capsule's **axis**. There are three of them:
- the axis point closest to the hand,
- the two ends of the stretch of the axis that lies inside the hand's intrusion sphere.

An unlocated hand could be anywhere, so its candidates are both ends of every capsule.

For each candidate point p:

```
v_per_unit = J_p · (delta  or  verticalJointRate)    point velocity per unit of the motion's
gain       = |v_per_unit|,  u = v_per_unit / gain    speed variable (sigma rate, or vertical m/s)
cap        = min over all candidates of  sqrt(2 · S_lim / m) / gain
```

- **Why axis points.** A hand touches the capsule at a surface point `s = p + r·n`, where n is
  the normal toward the hand. Its velocity is `v_s = v_p + ω × r·n`. Along n, the surface point
  and the axis point give exactly the same approach speed (`n·(ω × r n) = 0`) and the same
  effective mass (`nᵀ[r n]× = 0`). The surface only adds sliding, which cannot press into a hand.
  With u taken along the velocity, as here, surface points would therefore overestimate. One
  example is a link spinning about its own axis.
- **The Jacobian.** `J_p` is libfranka's `zeroJacobian` of the capsule's frame, shifted from the
  frame origin o to the point: `J_v − [p − o]× J_ω`, i.e. `v_p = v_o + ω × (p − o)`. It is computed
  once per frame and sample. On 2026-11-04 `show_capsules` confirmed on the robot that it matches
  finite differences of `model.pose` (deviation ≤ 1e-10 m/rad), and that libfranka's Jacobian of
  joint frame N contains joint N, which the shift needs.
- **Remaining approximation.** u is the direction of the point's motion. This is the usual
  convention, and it was also used by the old EE check. It can *underestimate* the energy when the
  real contact normal points along a direction in which the robot is much heavier. Sound
  alternatives would be:
  - the contact normal (axis point → hand center);
  - the worst case over all directions, `½ vᵀ Λ v`, which is bounded by the robot's kinetic
    energy `½ q̇ᵀ M q̇`.
- **S_lim** is pick_place's first argument. The default is `Params::s_lim` = 0.01 J; the usage
  text says 0.05.
- **Applying the cap.** The cap is in the motion's own speed variable. The control loop applies it
  by re-planning its S-curve from its current state on the same path, so the goal is still reached
  exactly.
- **Reporting.** The decision and the overlay report the contact and the capsule of the point that
  set the cap.

### Timing (`Supervisor::decide`)

- **Slow down**: if there is an intrusion and the new cap is lower than the last commanded one, the
  cap is sent. After a slowdown the supervisor ignores frames until three things are true: the
  command was applied, the velocity change has finished, and the reached speed has been held for
  `hold_after_slowdown` = 0.3 s.
- **Lower only**: while a hand stays exposed, the cap can only go down. A higher cap computed in
  between is not sent.
- **Resume**: as soon as a frame shows no exposed hand (and no unlocated one), the nominal speed is
  sent. A resume is not waited for. Every following frame is still checked, and a new slowdown
  replaces a resume that is still running.
- Every motion starts at its nominal speed. Between motions nothing is evaluated.

### Implications of the whole-robot test

1. **Earlier slowdowns.** The intrusion distance is now measured to the capsule surface, so the
   gripper's few centimeters of volume count. Before, the EE was a bare point.
2. **Unknown hand position.** When the hand position is unknown (or the stream is stale), every
   capsule counts. Runs without a working detector may be slower than before.
3. **Hands near the base.** A hand near the shoulder (link 2) is exposed, but that link moves
   slowly, so often no slowdown results. The status still reads `intruding`.
4. **Holding an object.** While an object is held, the gripper is tested too, not only the
   object's tip.
5. **Not covered:**
   - pinching between links (for example the elbow closing),
   - trapping against anything other than the table,
   - trapping during joint motions.

### Overlay stream (display only)

After every frame the supervisor sends what it saw to the detector over UDP (port 5006,
[src/overlay_udp.h](../src/overlay_udp.h), **version 2**; Python side:
[scripts/robot_overlay_protocol.py](../scripts/robot_overlay_protocol.py)). The detector draws
it into its video, and nothing reads it back. Per frame it carries:
- the flags valid / moving / holding_object / slowed / unlocated,
- the contact and capsule that set the active slowdown, its contact point, and the cap as a
  fraction of the nominal speed,
- the current reference point (EE or object tip) and the predicted reference points over the
  horizon,
- **all capsules at the current joints**,
- per hand: its sphere, its status, the nearest capsule, its clearance, the nearest robot surface
  point and the bitmask of the capsules it is exposed to.

A version 1 receiver rejects version 2 packets. The detector on branch `detector-overlay`
draws the capsules: grey, orange where an intruding hand is exposed to them, magenta when
constrained.

### Logs

`pick_place.csv` has one row per control tick:
- `ee_speed`, `v_safe`, `m_eff`, `m_limit`: for the **reference point** (EE, or object tip while
  held), as before.
- `contact`: the capsule test at this tick's joints.
- `hand_clearance`: the nearest hand's intrusion boundary to the nearest capsule surface, < 0 inside.
- `clearance_capsule`: the name of that capsule.

`pick_place_decisions.csv` lists every slowdown and resume, with the contact and capsule of the
point that set the cap. At the end, pick_place prints the longest supervisor frame time.

### Checking

- **`test_robot_body`** (offline) covers:
  - the segment geometry and the capsule placement, including the three switches;
  - the shift of a frame Jacobian to a point (`v_p = v_o + ω × r`).
- **`show_capsules`** (robot connected, the robot does not move) does four things:
  - It prints the capsules at the current joints.
  - It compares the velocity of every capsule end as the supervisor computes it (libfranka's
    `zeroJacobian(frame)` shifted to the point) with finite differences of `model.pose`. It also
    reports whether libfranka's Jacobian of joint frame N contains joint N. It exits with code 2
    if the check fails.
  - With `--poses`, it prints the capsules along the task's motions and exits.
  - Otherwise it streams the capsules to the detector at 30 Hz. Guide the robot by hand and check
    in the video that the capsules cover the links.

---

## Previous behavior: single reference point

Up to commit `62aed97` the supervisor tested one **reference point** per predicted sample: the
EE (fingertip center), or the held object's far end while the object was held. A hand intruded
when `|p_ref(q_k) − center| < r_hand + radius_margin`. The scenarios were:

| # | Trigger | Hand status | Contact → mass |
|---|---|---|---|
| S0 | no hand | – | – |
| S1 | no predicted reference point within the intrusion radius | clear | – |
| S2 | intrusion; joint motion, upward motion, or downward motion with the hand > 5 cm above the table | intruding | free → `m_eff` at the reference point |
| S3 | intrusion during a downward motion, hand ≤ 5 cm above the table | constrained | tabulated mass at the EE |
| S4 | unlocated, not moving down | – | free → `m_eff` |
| S5 | unlocated, moving down | – | max(`m_eff`, tabulated) |

The cap was the minimum over all predicted samples of `sqrt(2 S_lim / m) / gain` at the reference
point. The timing was the same as now. Its limits, which the capsules address:
- Links 1–7, the hand body and the fingers were never tested. A hand in the path of the elbow
  but away from the EE path was `clear`, and only the contact reflex reacted, after the hit.
- While an object was held, only its tip was tested, not the gripper holding it.
- The reference point had no volume.

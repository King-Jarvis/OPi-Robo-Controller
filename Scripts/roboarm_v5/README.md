# RoboArm V5

V5 is the V4 controller (HMI, PS4 pendant, AGV link, teach/replay, homing,
E-STOP, fan) split into modules. It drives **any serial arm with 1–7 revolute
joints**, and everything about the arm lives in one **robot file**.

```bash
cd Scripts
python3 -m roboarm_v5                                # last robot used (default: robots/roboarm_opi.json)
python3 -m roboarm_v5 --robot robots/example_4dof.json
python3 -m roboarm_v5 --sim --windowed               # no hardware: simulated board, runs on any PC
python3 -m unittest discover -s tests -v             # 42 tests, no hardware needed (~40 s)
```

Needs `numpy`, plus `lgpio` and `pygame` on the Pi. `ikpy` is no longer used.
Settings live in `~/.roboarm/config_v5.json`. V4's `~/.roboarm/config.json` is
left untouched, and its fan and AGV settings are imported the first time V5 runs.

## Layout

| package | what | from V4 |
|---|---|---|
| `robot/` | `RobotModel`, DH and URDF loaders, robot-file validation | new |
| `kinematics/` | FK, Jacobian, damped-least-squares IK, motor↔kinematic angle mapping | replaces ikpy chain |
| `hw/` | lgpio backend + `SimGPIO`, step model, safety loop, fan | :441-678 |
| `motion/` | absolute and coordinated moves, homing, Cartesian jog and linear moves | :824-1078 |
| `io/` | AGV link, programs and playback, PS4 pendant, config | :359-430, :1084-1362 |
| `ui/` | theme, widgets, overlays, main screen | :136-238, :1411-3422 |
| `runtime.py` | wires the layers together (replaces V4's module globals) | |

## The robot file

See `robots/roboarm_opi.json` for a complete example. Keys starting with `_` are notes and are ignored.

- **`kinematics`**: the geometry, given in one of two ways.
  - `{"type": "dh", "convention": "standard"|"modified", "dh": [{"a", "alpha", "d", "theta_offset"}, …]}`
    has one row per joint. Lengths are in mm and angles in degrees.
  - `{"type": "urdf", "file": "arm.urdf", "base": "base_link", "tip": "flange"}`
    uses a URDF file, for example one exported from Fusion 360, SolidWorks or Onshape.
    Fixed joints are folded in, and joint limits fall back to the URDF's own.
- **`tool`**: `{"xyz": [mm], "rpy_deg": […]}` gives the flange-to-tool-point offset. `base` is optional.
- **`joints`**: one entry per joint, in chain order:
  - `step_pin`, `dir_pin`, `limit_pin`
  - `steps_per_deg`, `max_freq` (Hz at 100 % speed)
  - `limits_deg`: kinematic degrees. These are the only joint limits.
  - `invert_dir`: true if the dir pin driven high makes the angle go negative.
  - `home`: `{"switch": "min"|"max"|null, "datum_deg", "park_deg"}`. `datum_deg`
    is the angle where the switch closes and defaults to that end of
    `limits_deg`. `park_deg` is where the joint goes after homing.
- **`board`**: `gpiochip`, `estop_pin`, `fan_pin`, and the trigger level of the switches.
- **`ik`**: the IK acceptance thresholds `pos_tol_mm` and `ori_tol_deg`.

The IK mode follows the joint count. Six or more joints get the full pose.
Five joints get position plus the tool axis. Fewer get position only. The
Target panel can override this.

## Describing a new arm

1. Pick the **zero pose**: the pose where every kinematic angle is 0, usually
   the arm straight up or straight out. Measure the links in that pose.
2. Write the DH table (or export a URDF) for that pose. Check it with the
   tests: add a test like `TestForwardKinematics` that compares FK against a
   few poses you work out by hand.
3. For each joint, decide which way is positive. With the right-hand rule
   about the joint axis, positive turns the way your fingers curl. Then
   decide which end of travel the switch is at.
4. Run `--sim` and jog every joint. The TCP readout should move the way you expect.
5. Then work through the bench checks below on the real arm.

## Bench checks (before V5 drives the arm)

The geometry in `robots/roboarm_opi.json` is the V4 numbers, corrected, but it
has **not yet been checked on the arm**. Work through this with the E-STOP in
reach and speed overrides low.

1. **Direction.** With the arm unhomed, jog each joint `+` briefly. The angle
   must increase in the direction the DH model says. If it moves the wrong way,
   toggle INVERT for that joint (SYSTEM → MOTORS) and save.
2. **Switch side.** For each joint, check which end of travel its switch is
   at. The file assumes J1 and J5 at `max` and the rest at `min`, taken from
   V4's `HOME_DIR`. HOME must drive each joint *toward* its switch.
3. **Home.** Each joint seeks its switch, backs off 3°, creeps back on at
   2°/s, then moves *away* to park. If a joint drives back into its switch,
   its `switch` side (or `invert_dir`) is wrong.
4. **Geometry.** At the zero pose and at two or three other poses, measure the
   tool tip against the TCP readout (tape, or a pointer on the tool). The open
   items are **L3**: is the 26 mm elbow offset really perpendicular to the
   forearm, and toward +X? And **J4**: does it roll about the forearm?
5. **Cartesian.** In WORLD mode, a held Z jog must keep X and Y steady. In
   TOOL mode, Z must move along the tool.
6. **Counted moves.** Absolute moves now send an exact pulse count with
   `lgpio.tx_pulse(…, pulse_cycles=N)`. Move a joint 90°, mark it, move it
   back, and check that it returns to the mark.

## What the IK review found in V4, and what V5 changes

1. IK angles and motor angles used different zeros. Motor angles were counted
   from the limit switch (J1 0…350), but IK used −175…175 with no offset
   between them. Every IK and Cartesian move was off by the home offset, and
   negative results were clamped to 0. → `kinematics/mapping.py` is now the
   only conversion between the two, and there is one set of limits.
2. J4 was modelled tilting about world X instead of rolling about the forearm.
   L3 was colinear with the forearm, so it wasn't an offset at all. → Corrected
   in the DH table, and covered by `test_forearm_roll_spins_wrist_about_forearm`.
3. Cartesian jog passed the J4–J6 joint angles in as Rx/Ry/Rz. Tool-frame jog
   used only J5/J6. Orientation 0/0/0 meant "ignore orientation".
   `orientation_mode='Z'` dropped Rz, and orientation error was never checked.
   → Jogs are rigid motions of the real FK pose, solved in full-pose mode with
   both errors checked.
4. J1 and J5 homed toward `+` and were then sent to a `+` target, straight back
   into the switch. → The switch side is now explicit, and parking always backs away.
5. Every absolute move ran a free pulse train and stopped it when the thread
   woke up, so scheduling delay turned into overshoot. → Exact pulse counts.
   Homing re-approaches the switch slowly, reading the pin directly.
6. The motor dictionaries were shared between threads with no locking. → Each
   motor has its own lock.
7. pygame's SDL swallowed SIGTERM, so `kill` couldn't stop the controller.
   → Fixed, and SIGTERM now runs the normal shutdown.

## Known limits

- None of this has run on the real arm yet; everything was checked in the simulator.
- A speed change made during an absolute move applies from the next move. A
  counted pulse train can't be retuned without losing count. Jogs still retune live.
- Linear moves pause briefly at each 5 mm waypoint, as V4's did.
- V4 program files are converted on load, but check every point before running one.

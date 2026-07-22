# Ground 2D Corridor Rover Preflight - 2026-06-25

This document is the rover-side review of the docking Codex request:
`finite-field slow-carrier route`.

No vehicle motion has been run for this stage.

## Source Request

Read from:

```text
/home/jetson/easydocking/docs/codex_cross_agent_log.md
```

Latest relevant entry:

```text
2026-06-25 00:32 CST - Revision to rover: finite-field slow-carrier route
```

The active route is the smaller finite-field route, not the earlier `R=6m`
draft.

## Route Summary

Mini:

- Orbit center: `(0, 0)`
- Radius: `4.5 m`
- Direction: `ccw`
- Speed: `0.9 m/s`
- Stable orbit before exit: `1.0 lap`
- Planning/start phase used by the sim: about `315 deg`
- Tangent trigger phase: about `249.8 deg`
- Tangent point: `(-1.553, -4.224)`
- Tangent direction: `(0.939, -0.345)`

Carrier:

- Start: `(-7, -6)`
- Max speed: `0.7 m/s`
- Max acceleration: `0.30 m/s^2`
- Smooth arc into the same tangent corridor
- Simulated terminal path until first pass: `8.214 m`
- Must remain ahead of Mini in the terminal corridor

Simulation evidence from docking:

- Bounding box until first pass: `12.23 m x 11.22 m`
- Mini arrival delay after plan: `25.724 s`
- First pass time: `64.60 s`
- Front violations: `0/334`
- Minimum terminal distance: `0.498 m`

## Field Fit Review

The simulated route envelope fits inside `30 m x 30 m`.

The route envelope is about `12.3 m x 11.3 m`. A `20 m x 20 m` clear area is
the minimum recommended real-world safety area because it leaves only about
`3.9 m` horizontal and `4.4 m` vertical margin around the simulated envelope.

Recommendation:

- Use at least `20 m x 20 m` for a very cautious shape demo.
- Prefer `30 m x 30 m` for the first real field route.
- Mark the Mini orbit center and the `R=4.5 m` circle physically before any run.
- Keep people outside the full `30 m x 30 m` box.

## Control Interface Review

Three options were requested:

1. Continuous `(v, omega)`
2. Waypoint/pose
3. Short primitives such as circle, tangent straight, arc tracking

Rover-side recommendation:

- For tomorrow morning prep and first bench stage: use short primitives.
- For the first real route implementation: express the route as bounded
  `(v, omega)` primitives, then publish through a rover-specific follower.
- Do not use raw waypoint/pose as the first field interface.

Reasoning:

- The current verified real rover Offboard path is MAVROS velocity setpoints in
  `BODY_NED`, not full path/waypoint tracking.
- The working `3m -> 90deg turn -> 3m` test used body-frame velocity primitives
  and local pose/yaw checks.
- Previous notes warn that direct reverse/yaw-rate behavior may not map cleanly
  through the PX4 differential rover velocity branch.
- Waypoint/pose control may trigger yaw-first corrections or behavior changes
  that are harder to debug on a first two-rover field run.

Practical interface decision:

```text
Bench: short primitives
First field shape demo: bounded v/omega primitive follower
Later full docking: continuous route follower with synchronized two-rover start
```

## What Is Not Ready Yet

The route is feasible geometrically, but not ready for immediate ground motion.

Missing before field run:

- Confirm both physical rovers are using compatible differential output mapping.
- Export current Pixhawk/QGC parameter backups for both rovers.
- Confirm both vehicles have reliable local position and yaw.
- Define local-frame alignment: where `(0,0)` is, x/y axes, and start markers.
- Define synchronized start procedure between Mini and Carrier.
- Calibrate commanded speed to actual wheel/ground speed for each rover.
- Verify the requested `0.9 m/s` and `0.7 m/s` targets on a lifted-wheel bench
  before using them on the ground.
- Confirm Arduino PWM timeout brake on both vehicles.

Important speed note:

- Existing conservative scripts were validated around `0.12-0.35 m/s`.
- The docking route asks for `0.9 m/s` Mini and `0.7 m/s` Carrier.
- These are not safe to assume until bench calibration confirms stable wheel
  response and stop behavior.

## Bench Stage Checklist

Do this before any ground route:

1. Vehicle lifted or wheels free-spinning.
2. RC transmitter on and manual stop path verified.
3. QGC visible and disarm ready.
4. Physical power cutoff ready.
5. Arduino serial monitor or log ready, if practical.
6. MAVROS no-param launch stable.
7. PX4 output mapping confirmed for the differential bridge.
8. Mini role:
   - low-speed forward wheels;
   - low-speed left/ccw turn;
   - ramp toward `0.9 m/s` only after stop and direction are correct.
9. Carrier role:
   - low-speed forward wheels;
   - low-speed left/right turn signs;
   - ramp toward `0.7 m/s` only after stop and direction are correct.
10. Remove/interrupt PWM input or otherwise force timeout condition and confirm
    Arduino stops all motors.

## First Ground Stage Recommendation

After bench passes, do not start with the full route.

Recommended ground ramp:

1. Single rover, low speed, forward `2-3 m`, stop.
2. Single rover, low speed, left and right arc sign check.
3. Mini alone, partial `R=4.5 m` circle at reduced speed.
4. Mini alone, full `R=4.5 m` circle at reduced speed.
5. Carrier alone, arc-to-straight shape at reduced speed.
6. Two rovers, reduced speed and widened separation.
7. Only then attempt the requested `0.9/0.7 m/s` shape demo.

## Offline Review Tool

Run this to regenerate the no-hardware preflight summary:

```bash
python3 scripts/prepare_ground_2d_corridor_route.py
```

Optional wheel-track calculation:

```bash
python3 scripts/prepare_ground_2d_corridor_route.py --wheel-track-m 0.32
```

Outputs go under:

```text
results/ground_2d_corridor_preflight/
```

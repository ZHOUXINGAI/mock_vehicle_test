# Beginner RC Rover Step By Step

Goal: use a remote controller to drive the rover forward, backward, left, and
right through Pixhawk 6C and QGroundControl.

This stage does not use ROS, offboard, or the Lubancat companion computer.

## 0. What You Need

Minimum hardware:

- Pixhawk 6C
- QGroundControl on a laptop
- RC transmitter and receiver
- rover chassis with left and right motors
- one dual-channel motor driver or two ESCs
- motor battery
- Pixhawk power module or stable 5V supply
- USB cable
- physical power switch or a way to unplug the motor battery quickly

Important: Pixhawk PWM pins are signal pins. They cannot power motors.

## 1. Identify The Motor Type

Before wiring, identify your motors:

- brushed DC motor: two thick wires per motor
- brushless motor: three thick wires per motor

For brushed motors, use one of these:

- RC car brushed ESC with reverse
- VESC configured for brushed or appropriate motor
- Sabertooth/Cytron motor driver in RC PWM mode

For brushless motors, use reversible ESCs or VESCs.

The motor driver must accept servo-style RC PWM input, usually:

- 1000 us: reverse or minimum
- 1500 us: stop or neutral
- 2000 us: forward or maximum

If your driver only accepts `IN1/IN2/PWM` logic pins, it is better for
Arduino/ESP32 training, not the first Pixhawk/PX4 path.

## 2. Wiring

Do not connect propellers or put the rover on the ground for first tests.

Basic signal wiring:

```text
RC receiver SBUS/CRSF/PPM -> Pixhawk 6C RC IN

Pixhawk MAIN OUT 1 signal -> right motor driver RC input
Pixhawk MAIN OUT 2 signal -> left motor driver RC input

Pixhawk MAIN OUT GND -> motor driver signal GND
Motor battery -> motor driver power input
Pixhawk power module or regulated 5V -> Pixhawk power input
```

Rules:

- common ground is required between Pixhawk and motor driver signal ground
- do not power motors from Pixhawk
- do not power Pixhawk from a random high-current motor driver unless it has a
  clean regulated 5V BEC
- keep a physical motor battery disconnect available

## 3. QGC Firmware

Connect Pixhawk 6C to the laptop with USB and open QGC.

In QGC:

1. Open `Vehicle Setup`.
2. Open `Firmware`.
3. Flash PX4 firmware with rover support if available.
4. If QGC only installs normal PX4 firmware and no rover airframes appear later,
   you need a PX4 rover build for Pixhawk 6C.

PX4 rover docs say rover support is a rover-specific build. For differential
rover, the airframe id is `50000`.

## 4. Airframe

In QGC:

1. Open `Vehicle Setup`.
2. Open `Airframe`.
3. Select `Generic Rover Differential`.
4. Click `Apply and Restart`.

If it does not appear:

1. Open `Parameters`.
2. Search `SYS_AUTOSTART`.
3. Set it to `50000`.
4. Reboot Pixhawk.

## 5. Sensors

Still in QGC:

1. Calibrate accelerometer.
2. Calibrate compass if QGC asks.
3. Calibrate level horizon if QGC asks.

For first manual rover test, GPS is not required. For geofence/return-home later,
GPS or another reliable position source is required.

## 6. Radio Calibration

In QGC:

1. Open `Vehicle Setup`.
2. Open `Radio`.
3. Turn on transmitter.
4. Follow calibration prompts.
5. Confirm QGC sees throttle and steering channels moving.

Expected control meaning:

- throttle stick: forward/backward
- steering/yaw/roll stick: left/right

Do not arm yet.

## 7. Flight Modes

Set one transmitter switch for modes if possible:

- position 1: Manual
- position 2: Hold or Position, if available later

For the first test, use Manual.

Set one switch for Arm/Disarm or Kill if your transmitter/QGC setup supports it.

## 8. Actuator Mapping

In QGC:

1. Open `Vehicle Setup`.
2. Open `Actuators`.
3. Assign right wheel/right motor to `MAIN 1`.
4. Assign left wheel/left motor to `MAIN 2`.
5. Keep values conservative.

Then use `Actuator Testing`.

The rover must be lifted so the wheels are not touching the ground.

Test one output at a time:

- MAIN 1 should spin only the right wheel or right side
- MAIN 2 should spin only the left wheel or left side

If a wheel direction is wrong, reverse it in QGC actuator output settings or in
the ESC/motor wiring. Do not compensate in your driving habit.

## 9. First Manual Test With Wheels Lifted

Checklist:

- rover lifted
- motor battery connected
- Pixhawk powered
- QGC connected
- transmitter on
- Manual mode selected
- throttle centered

Then:

1. Arm.
2. Push throttle forward slightly.
3. Both wheels should drive forward.
4. Pull throttle back slightly.
5. Both wheels should drive backward.
6. Push steering left.
7. Left/right wheels should create a left turn.
8. Push steering right.
9. Left/right wheels should create a right turn.
10. Disarm.

If it jumps to high speed, disconnect motor battery first, then debug.

## 10. First Ground Test

Only after the lifted test is correct:

1. Put rover on the ground in an open area.
2. Set speed limits low in the ESC or transmitter if possible.
3. Arm.
4. Move forward 0.5m.
5. Stop.
6. Move backward 0.5m.
7. Stop.
8. Turn left/right slowly.
9. Disarm.

Do not start with a 10m run.

## 11. What Not To Do Yet

Do not add Lubancat into the control loop yet.

Do not run offboard yet.

Do not rely on geofence yet.

Do not test on the ground before actuator directions are verified with wheels
lifted.

## 12. Why This Order

The manual RC path is the safety baseline:

```text
transmitter -> receiver -> Pixhawk -> motor driver -> motors
```

Later, the offboard path becomes:

```text
Lubancat/ROS -> PX4 offboard -> Pixhawk -> motor driver -> motors
```

The manual RC path must still work when the companion computer or ROS crashes.

## 13. Next Step After Manual Control Works

After you can drive manually:

1. Connect Lubancat to Pixhawk by telemetry UART or USB.
2. Read PX4 state only.
3. Log mode, arm state, battery, local position, RC channels.
4. Only then run offboard forward 3m and return.

That is the clean path toward the two-aircraft setup later.

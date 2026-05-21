# Pixhawk 6C Manual Rover First Step

Before offboard, make the manual chain work.

## Recommended Chain

Remote controller -> receiver -> Pixhawk 6C -> reversible ESC or motor driver -> left/right motors.

Use QGC to configure PX4 as a rover:

1. Flash PX4 firmware for Pixhawk 6C.
2. Select `Generic Rover Differential`.
3. Calibrate radio.
4. Configure actuators:
   - right wheel or right motor on output 1
   - left wheel or left motor on output 2
5. Test actuators with the vehicle lifted.
6. Set a physical kill switch or disarm switch on the transmitter.

## Important Hardware Notes

Pixhawk PWM outputs do not drive motors directly. Use a reversible RC ESC, VESC,
Sabertooth/Cytron in RC PWM mode, or another driver that accepts servo-style PWM.

Keep Pixhawk ground and motor-driver signal ground common. Power the motors from
their own battery path, and power Pixhawk through a power module or regulated 5V.

## Safety Order

1. Wheels off the ground.
2. QGC radio input moves correctly.
3. Actuator test has correct left/right direction.
4. Arm only at low throttle.
5. Verify RC failsafe stops the rover.

The companion computer should not be required for manual takeover. This is the
same safety architecture you will want on the aircraft later.

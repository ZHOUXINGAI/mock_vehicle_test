#include <AFMotor.h>

AF_DCMotor leftMotor(1);   // M1
AF_DCMotor rightMotor(2);  // M2

const uint8_t THROTTLE_PIN = A1;  // RF209S CH1 signal: stick up/down
const uint8_t STEERING_PIN = A0;  // RF209S CH2 signal: stick left/right

const uint8_t LEFT_FORWARD_CMD = BACKWARD;
const uint8_t LEFT_BACKWARD_CMD = FORWARD;
const uint8_t RIGHT_FORWARD_CMD = FORWARD;
const uint8_t RIGHT_BACKWARD_CMD = BACKWARD;

const int CENTER_US = 1500;
const int DEAD_BAND_US = 70;
const int MIN_VALID_US = 900;
const int MAX_VALID_US = 2100;
const int MAX_STICK_US = 450;
const int MIN_MOTOR_PWM = 80;
const int MAX_MOTOR_PWM = 220;

// Change these only if the stick direction feels reversed after bench testing.
const bool INVERT_THROTTLE = false;
const bool INVERT_STEERING = false;

void stopAll() {
  leftMotor.run(RELEASE);
  rightMotor.run(RELEASE);
}

int readRcChannel(uint8_t pin) {
  unsigned long pulse = pulseIn(pin, HIGH, 25000);
  if (pulse < MIN_VALID_US || pulse > MAX_VALID_US) {
    return 0;
  }

  int centered = static_cast<int>(pulse) - CENTER_US;
  if (abs(centered) < DEAD_BAND_US) {
    return 0;
  }

  centered = constrain(centered, -MAX_STICK_US, MAX_STICK_US);
  return map(centered, -MAX_STICK_US, MAX_STICK_US, -255, 255);
}

void runPhysicalMotor(
  AF_DCMotor &motor,
  int command,
  uint8_t forwardCommand,
  uint8_t backwardCommand
) {
  if (abs(command) < 10) {
    motor.run(RELEASE);
    return;
  }

  int speed = constrain(abs(command), MIN_MOTOR_PWM, MAX_MOTOR_PWM);
  motor.setSpeed(speed);
  motor.run(command > 0 ? forwardCommand : backwardCommand);
}

void setup() {
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(STEERING_PIN, INPUT);

  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  stopAll();

  // Safety pause after reset before RC control becomes active.
  delay(3000);
}

void loop() {
  int throttle = readRcChannel(THROTTLE_PIN);
  int steering = readRcChannel(STEERING_PIN);

  if (INVERT_THROTTLE) {
    throttle = -throttle;
  }
  if (INVERT_STEERING) {
    steering = -steering;
  }

  int leftCommand = constrain(throttle + steering, -255, 255);
  int rightCommand = constrain(throttle - steering, -255, 255);

  runPhysicalMotor(leftMotor, leftCommand, LEFT_FORWARD_CMD, LEFT_BACKWARD_CMD);
  runPhysicalMotor(rightMotor, rightCommand, RIGHT_FORWARD_CMD, RIGHT_BACKWARD_CMD);

  delay(20);
}

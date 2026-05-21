#include <AFMotor.h>

AF_DCMotor leftMotor(1);   // M1
AF_DCMotor rightMotor(2);  // M2

const uint8_t TEST_SPEED = 220;
const uint8_t WHEEL_FORWARD = BACKWARD;
const uint8_t WHEEL_BACKWARD = FORWARD;

void stopAll() {
  leftMotor.run(RELEASE);
  rightMotor.run(RELEASE);
}

void setup() {
  leftMotor.setSpeed(TEST_SPEED);
  rightMotor.setSpeed(TEST_SPEED);

  stopAll();

  // Safety pause after reset before any motor moves.
  delay(5000);

  // Left wheel: forward 2s, then backward 2s.
  leftMotor.run(WHEEL_FORWARD);
  delay(2000);
  leftMotor.run(RELEASE);
  delay(1000);

  leftMotor.run(WHEEL_BACKWARD);
  delay(2000);
  leftMotor.run(RELEASE);

  // Wait 5s before testing the right wheel.
  delay(5000);

  // Right wheel: forward 2s, then backward 2s.
  rightMotor.run(WHEEL_BACKWARD);
  delay(2000);
  rightMotor.run(RELEASE);
  delay(1000);

  rightMotor.run(WHEEL_FORWARD);
  delay(2000);
  rightMotor.run(RELEASE);

  // Stay stopped.
  while (true) {
    stopAll();
    delay(1000);
  }
}

void loop() {
}

#include <AFMotor.h>

AF_DCMotor leftMotor(1);   // M1 left wheel
AF_DCMotor rightMotor(2);  // M2 right wheel

const uint8_t THROTTLE_PIN = A1;  // RF209S CH1 signal: stick up/down
const uint8_t STEERING_PIN = A0;  // RF209S CH2 signal: stick left/right

const uint8_t LEFT_FORWARD_CMD = BACKWARD;
const uint8_t LEFT_BACKWARD_CMD = FORWARD;
const uint8_t RIGHT_FORWARD_CMD = FORWARD;
const uint8_t RIGHT_BACKWARD_CMD = BACKWARD;

const int CENTER_US = 1500;
const int DEAD_BAND_US = 100;
const int MIN_VALID_US = 900;
const int MAX_VALID_US = 2100;
const int MAX_STICK_US = 450;
const int MIN_MOTOR_PWM = 200;
const int MAX_MOTOR_PWM = 255;

const bool INVERT_THROTTLE = false;
const bool INVERT_STEERING = false;

void stopAll() {
  leftMotor.run(RELEASE);
  rightMotor.run(RELEASE);
}

unsigned long readPulseUs(uint8_t pin) {
  return pulseIn(pin, HIGH, 25000);
}

int pulseToCommand(unsigned long pulse) {
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
  Serial.begin(115200);
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(STEERING_PIN, INPUT);

  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  stopAll();

  delay(3000);
  Serial.println("RF209S PWM debug started");
}

void loop() {
  unsigned long throttlePulse = readPulseUs(THROTTLE_PIN);
  unsigned long steeringPulse = readPulseUs(STEERING_PIN);

  int throttle = pulseToCommand(throttlePulse);
  int steering = pulseToCommand(steeringPulse);

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

  static unsigned long lastPrintMs = 0;
  unsigned long now = millis();
  if (now - lastPrintMs > 250) {
    lastPrintMs = now;
    Serial.print("CH2 throttle us=");
    Serial.print(throttlePulse);
    Serial.print(" cmd=");
    Serial.print(throttle);
    Serial.print(" | CH1 steering us=");
    Serial.print(steeringPulse);
    Serial.print(" cmd=");
    Serial.print(steering);
    Serial.print(" | left=");
    Serial.print(leftCommand);
    Serial.print(" right=");
    Serial.println(rightCommand);
  }

  delay(20);
}

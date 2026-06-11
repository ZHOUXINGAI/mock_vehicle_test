// Pixhawk 6C -> Arduino UNO -> D153B/TB6612 -> MG513X brushed DC motors.
//
// This sketch reads two Pixhawk PWM outputs and converts them to D153B H-bridge
// direction + PWM signals.
//
// Intended first mode:
//   Pixhawk output 1 -> left wheel command
//   Pixhawk output 2 -> right wheel command
//
// Safety behavior:
//   - Invalid/missing PWM input stops both motors.
//   - PWM around 1500us is treated as neutral.
//   - Motor output is released on startup.

// Pixhawk PWM input pins on Arduino.
const uint8_t LEFT_RC_PIN = 2;
const uint8_t RIGHT_RC_PIN = 6;

// D153B wiring to Arduino UNO.
const uint8_t PWMA_PIN = 3;
const uint8_t AIN2_PIN = 4;
const uint8_t AIN1_PIN = 5;
const uint8_t STBY_PIN = 7;
const uint8_t BIN1_PIN = 8;
const uint8_t BIN2_PIN = 9;
const uint8_t PWMB_PIN = 10;

const int CENTER_US = 1500;
const int DEAD_BAND_US = 70;
const int MIN_VALID_US = 900;
const int MAX_VALID_US = 2100;
const int MAX_STICK_US = 450;

// MG513X + rover load usually needs non-trivial PWM to start moving.
const int MIN_DRIVE_PWM = 120;
const int MAX_DRIVE_PWM = 255;

// Stop if a PWM pulse is not seen quickly.
const unsigned long PULSE_TIMEOUT_US = 25000;

// Change only if physical motor direction is reversed.
const bool LEFT_INVERT = false;
const bool RIGHT_INVERT = false;

void stopAll() {
  analogWrite(PWMA_PIN, 0);
  analogWrite(PWMB_PIN, 0);
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN2_PIN, LOW);
  digitalWrite(BIN1_PIN, LOW);
  digitalWrite(BIN2_PIN, LOW);
  digitalWrite(STBY_PIN, LOW);
}

int readPixhawkCommand(uint8_t pin, bool *valid) {
  unsigned long pulse = pulseIn(pin, HIGH, PULSE_TIMEOUT_US);

  if (pulse < MIN_VALID_US || pulse > MAX_VALID_US) {
    *valid = false;
    return 0;
  }

  *valid = true;
  int centered = (int)pulse - CENTER_US;

  if (abs(centered) <= DEAD_BAND_US) {
    return 0;
  }

  centered = constrain(centered, -MAX_STICK_US, MAX_STICK_US);
  return map(centered, -MAX_STICK_US, MAX_STICK_US, -255, 255);
}

void setOneMotor(
  int command,
  uint8_t pwmPin,
  uint8_t in1Pin,
  uint8_t in2Pin,
  bool invert
) {
  command = constrain(command, -255, 255);

  if (abs(command) < 5) {
    analogWrite(pwmPin, 0);
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    return;
  }

  bool forward = command > 0;
  if (invert) {
    forward = !forward;
  }

  int pwm = map(abs(command), 1, 255, MIN_DRIVE_PWM, MAX_DRIVE_PWM);
  pwm = constrain(pwm, MIN_DRIVE_PWM, MAX_DRIVE_PWM);

  digitalWrite(in1Pin, forward ? HIGH : LOW);
  digitalWrite(in2Pin, forward ? LOW : HIGH);
  analogWrite(pwmPin, pwm);
}

void driveMotors(int leftCommand, int rightCommand) {
  digitalWrite(STBY_PIN, HIGH);
  setOneMotor(leftCommand, PWMA_PIN, AIN1_PIN, AIN2_PIN, LEFT_INVERT);
  setOneMotor(rightCommand, PWMB_PIN, BIN1_PIN, BIN2_PIN, RIGHT_INVERT);
}

void setup() {
  pinMode(LEFT_RC_PIN, INPUT);
  pinMode(RIGHT_RC_PIN, INPUT);

  pinMode(PWMA_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);

  stopAll();
  Serial.begin(115200);
  Serial.println("D153B Pixhawk PWM bridge ready");
  delay(1000);
}

void loop() {
  bool leftValid = false;
  bool rightValid = false;
  int leftCommand = readPixhawkCommand(LEFT_RC_PIN, &leftValid);
  int rightCommand = readPixhawkCommand(RIGHT_RC_PIN, &rightValid);

  if (!leftValid || !rightValid) {
    stopAll();
    delay(20);
    return;
  }

  driveMotors(leftCommand, rightCommand);
  delay(10);
}

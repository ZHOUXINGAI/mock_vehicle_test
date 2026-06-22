// Pixhawk 6C -> Arduino UNO -> D24A/TB6612 -> four DC motors.
//
// Pixhawk to Arduino:
//   Pixhawk MAIN/AUX output 1 signal -> Arduino D2   // throttle/forward-back
//   Pixhawk MAIN/AUX output 2 signal -> Arduino D13  // steering/left-right
//   Pixhawk PWM GND                  -> Arduino GND
//
// D24A wiring to Arduino UNO:
//   PWMA -> D3    AIN1 -> D4    AIN2 -> D7
//   PWMB -> D5    BIN1 -> D8    BIN2 -> D12
//   PWMC -> D6    CIN1 -> D10   CIN2 -> D11
//   PWMD -> D9    DIN1 -> A0    DIN2 -> A1
//   STBY -> A2
//   GND  -> Arduino GND
//
// Safety behavior:
//   - Invalid/missing Pixhawk PWM input stops all motors.
//   - PWM around 1500us is neutral.
//   - Motor output is released on startup.

struct MotorPins {
  uint8_t pwm;
  uint8_t in1;
  uint8_t in2;
};

const uint8_t THROTTLE_RC_PIN = 2;
const uint8_t STEERING_RC_PIN = 13;

const MotorPins MOTOR_A = {3, 4, 7};    // right-front, raw backward = physical forward
const MotorPins MOTOR_B = {5, 8, 12};   // left-front, raw backward = physical forward
const MotorPins MOTOR_C = {6, 10, 11};  // left-rear, raw forward = physical forward
const MotorPins MOTOR_D = {9, A0, A1};  // right-rear, raw forward = physical forward
const uint8_t STBY_PIN = A2;

const int CENTER_US = 1500;
const int DEAD_BAND_US = 80;
const int MIN_VALID_US = 900;
const int MAX_VALID_US = 2100;
const int MAX_STICK_US = 450;
const unsigned long PULSE_TIMEOUT_US = 25000;

const int MIN_DRIVE_PWM = 70;
const int MAX_DRIVE_PWM = 140;

const bool INVERT_THROTTLE = false;
const bool INVERT_STEERING = false;

unsigned long lastPrintMs = 0;

void stopOneMotor(const MotorPins &motor) {
  analogWrite(motor.pwm, 0);
  digitalWrite(motor.in1, LOW);
  digitalWrite(motor.in2, LOW);
}

void stopAll() {
  stopOneMotor(MOTOR_A);
  stopOneMotor(MOTOR_B);
  stopOneMotor(MOTOR_C);
  stopOneMotor(MOTOR_D);
  digitalWrite(STBY_PIN, LOW);
}

void setOneRawMotor(const MotorPins &motor, int command) {
  command = constrain(command, -255, 255);

  if (command == 0) {
    stopOneMotor(motor);
    return;
  }

  bool rawForward = command > 0;
  int pwm = constrain(abs(command), 0, 255);

  digitalWrite(motor.in1, rawForward ? HIGH : LOW);
  digitalWrite(motor.in2, rawForward ? LOW : HIGH);
  analogWrite(motor.pwm, pwm);
}

int applyMotorFloor(int command) {
  command = constrain(command, -MAX_DRIVE_PWM, MAX_DRIVE_PWM);

  if (command == 0) {
    return 0;
  }

  int magnitude = abs(command);
  magnitude = map(magnitude, 1, MAX_DRIVE_PWM, MIN_DRIVE_PWM, MAX_DRIVE_PWM);
  magnitude = constrain(magnitude, MIN_DRIVE_PWM, MAX_DRIVE_PWM);

  return command > 0 ? magnitude : -magnitude;
}

void drivePhysical(int leftCommand, int rightCommand) {
  leftCommand = applyMotorFloor(leftCommand);
  rightCommand = applyMotorFloor(rightCommand);

  int rawA = -rightCommand;
  int rawB = -leftCommand;
  int rawC = leftCommand;
  int rawD = rightCommand;

  digitalWrite(STBY_PIN, HIGH);
  setOneRawMotor(MOTOR_A, rawA);
  setOneRawMotor(MOTOR_B, rawB);
  setOneRawMotor(MOTOR_C, rawC);
  setOneRawMotor(MOTOR_D, rawD);
}

int readPixhawkCommand(uint8_t pin, bool &valid, int &pulseUs) {
  unsigned long pulse = pulseIn(pin, HIGH, PULSE_TIMEOUT_US);
  pulseUs = (int)pulse;

  if (pulse < MIN_VALID_US || pulse > MAX_VALID_US) {
    valid = false;
    return 0;
  }

  valid = true;
  int centered = (int)pulse - CENTER_US;

  if (abs(centered) <= DEAD_BAND_US) {
    return 0;
  }

  centered = constrain(centered, -MAX_STICK_US, MAX_STICK_US);
  return map(centered, -MAX_STICK_US, MAX_STICK_US, -MAX_DRIVE_PWM, MAX_DRIVE_PWM);
}

void setup() {
  pinMode(THROTTLE_RC_PIN, INPUT);
  pinMode(STEERING_RC_PIN, INPUT);

  pinMode(MOTOR_A.pwm, OUTPUT);
  pinMode(MOTOR_A.in1, OUTPUT);
  pinMode(MOTOR_A.in2, OUTPUT);
  pinMode(MOTOR_B.pwm, OUTPUT);
  pinMode(MOTOR_B.in1, OUTPUT);
  pinMode(MOTOR_B.in2, OUTPUT);
  pinMode(MOTOR_C.pwm, OUTPUT);
  pinMode(MOTOR_C.in1, OUTPUT);
  pinMode(MOTOR_C.in2, OUTPUT);
  pinMode(MOTOR_D.pwm, OUTPUT);
  pinMode(MOTOR_D.in1, OUTPUT);
  pinMode(MOTOR_D.in2, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);

  stopAll();
  Serial.begin(115200);
  Serial.println("D24A Pixhawk PWM bridge ready");
  delay(1000);
}

void loop() {
  bool throttleValid = false;
  bool steeringValid = false;
  int throttleUs = 0;
  int steeringUs = 0;

  int throttle = readPixhawkCommand(THROTTLE_RC_PIN, throttleValid, throttleUs);
  int steering = readPixhawkCommand(STEERING_RC_PIN, steeringValid, steeringUs);

  if (INVERT_THROTTLE) {
    throttle = -throttle;
  }
  if (INVERT_STEERING) {
    steering = -steering;
  }

  int leftCommand = constrain(throttle + steering, -MAX_DRIVE_PWM, MAX_DRIVE_PWM);
  int rightCommand = constrain(throttle - steering, -MAX_DRIVE_PWM, MAX_DRIVE_PWM);

  if (!throttleValid || !steeringValid) {
    stopAll();
    leftCommand = 0;
    rightCommand = 0;
  } else {
    drivePhysical(leftCommand, rightCommand);
  }

  unsigned long now = millis();
  if (now - lastPrintMs > 250) {
    lastPrintMs = now;
    Serial.print("thr_us=");
    Serial.print(throttleValid ? throttleUs : 0);
    Serial.print(" thr=");
    Serial.print(throttle);
    Serial.print(" steer_us=");
    Serial.print(steeringValid ? steeringUs : 0);
    Serial.print(" steer=");
    Serial.print(steering);
    Serial.print(" left=");
    Serial.print(leftCommand);
    Serial.print(" right=");
    Serial.println(rightCommand);
  }

  delay(10);
}

// Pixhawk 6C differential rover PWM -> Arduino UNO -> D24A/TB6612 -> four DC motors.
//
// This sketch is for PX4 differential-rover output mapping:
//   PWM_MAIN_FUNC1=101  // Motor 1
//   PWM_MAIN_FUNC2=102  // Motor 2
//
// Pixhawk to Arduino, physical wiring unchanged:
//   Pixhawk PWM output 1 signal -> Arduino D2
//   Pixhawk PWM output 2 signal -> Arduino D13
//   Pixhawk PWM GND             -> Arduino GND
//
// In this sketch the two Pixhawk PWM inputs are left/right wheel commands,
// not throttle/steering. Use this only with differential-rover output mapping.

struct MotorPins {
  uint8_t pwm;
  uint8_t in1;
  uint8_t in2;
};

const uint8_t LEFT_RC_PIN = 2;
const uint8_t RIGHT_RC_PIN = 13;

const MotorPins MOTOR_A = {3, 4, 7};    // right-front, raw backward = physical forward
const MotorPins MOTOR_B = {5, 8, 12};   // left-front, raw backward = physical forward
const MotorPins MOTOR_C = {6, 10, 11};  // left-rear, raw forward = physical forward
const MotorPins MOTOR_D = {9, A0, A1};  // right-rear, raw forward = physical forward
const uint8_t STBY_PIN = A2;

const int CENTER_US = 1500;
const int DEAD_BAND_US = 35;
const int MIN_VALID_US = 900;
const int MAX_VALID_US = 2100;
const int MAX_STICK_US = 450;
const unsigned long PULSE_TIMEOUT_US = 25000;

const int MIN_DRIVE_PWM = 70;
const int MAX_DRIVE_PWM = 140;

const bool SWAP_LEFT_RIGHT_INPUTS = false;
const bool INVERT_LEFT_COMMAND = false;
const bool INVERT_RIGHT_COMMAND = false;

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
  pinMode(LEFT_RC_PIN, INPUT);
  pinMode(RIGHT_RC_PIN, INPUT);

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
  Serial.println("D24A Pixhawk differential PWM bridge ready");
  delay(1000);
}

void loop() {
  bool input1Valid = false;
  bool input2Valid = false;
  int input1Us = 0;
  int input2Us = 0;

  int input1 = readPixhawkCommand(LEFT_RC_PIN, input1Valid, input1Us);
  int input2 = readPixhawkCommand(RIGHT_RC_PIN, input2Valid, input2Us);

  int leftCommand = SWAP_LEFT_RIGHT_INPUTS ? input2 : input1;
  int rightCommand = SWAP_LEFT_RIGHT_INPUTS ? input1 : input2;

  if (INVERT_LEFT_COMMAND) {
    leftCommand = -leftCommand;
  }
  if (INVERT_RIGHT_COMMAND) {
    rightCommand = -rightCommand;
  }

  if (!input1Valid || !input2Valid) {
    stopAll();
    leftCommand = 0;
    rightCommand = 0;
  } else {
    drivePhysical(leftCommand, rightCommand);
  }

  unsigned long now = millis();
  if (now - lastPrintMs > 250) {
    lastPrintMs = now;
    Serial.print("in1_us=");
    Serial.print(input1Valid ? input1Us : 0);
    Serial.print(" in1=");
    Serial.print(input1);
    Serial.print(" in2_us=");
    Serial.print(input2Valid ? input2Us : 0);
    Serial.print(" in2=");
    Serial.print(input2);
    Serial.print(" left=");
    Serial.print(leftCommand);
    Serial.print(" right=");
    Serial.println(rightCommand);
  }

  delay(10);
}

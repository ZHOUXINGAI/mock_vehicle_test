// Orin Nano/PC -> USB serial -> Arduino UNO -> D24A/TB6612 -> four DC motors.
//
// D24A wiring to Arduino UNO:
//   PWMA -> D3    AIN1 -> D4    AIN2 -> D7
//   PWMB -> D5    BIN1 -> D8    BIN2 -> D12
//   PWMC -> D6    CIN1 -> D10   CIN2 -> D11
//   PWMD -> D9    DIN1 -> A0    DIN2 -> A1
//   STBY -> A2
//   GND  -> Arduino GND
//
// Keep Arduino D0/D1 free for USB serial.
//
// Raw serial commands at 115200 baud:
//   AF [pwm] [ms]  channel A forward
//   AB [pwm] [ms]  channel A backward
//   BF [pwm] [ms]  channel B forward
//   BB [pwm] [ms]  channel B backward
//   CF [pwm] [ms]  channel C forward
//   CB [pwm] [ms]  channel C backward
//   DF [pwm] [ms]  channel D forward
//   DB [pwm] [ms]  channel D backward
//   S              stop
//
// Current physical calibration:
//   A forward  -> right-front backward
//   A backward -> right-front forward
//   B forward  -> left-front backward
//   B backward -> left-front forward
//   C forward  -> left-rear forward
//   C backward -> left-rear backward
//   D forward  -> right-rear forward
//   D backward -> right-rear backward

struct MotorPins {
  uint8_t pwm;
  uint8_t in1;
  uint8_t in2;
};

const MotorPins MOTOR_A = {3, 4, 7};
const MotorPins MOTOR_B = {5, 8, 12};
const MotorPins MOTOR_C = {6, 10, 11};
const MotorPins MOTOR_D = {9, A0, A1};
const uint8_t STBY_PIN = A2;

const int DEFAULT_PWM = 120;
const unsigned long DEFAULT_DURATION_MS = 700;
const unsigned long MAX_DURATION_MS = 3000;
const unsigned long WATCHDOG_MS = 700;

unsigned long stopAtMs = 0;
unsigned long lastCommandMs = 0;

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

void setOneMotor(const MotorPins &motor, int command) {
  command = constrain(command, -255, 255);

  if (command == 0) {
    stopOneMotor(motor);
    return;
  }

  bool forward = command > 0;
  int pwm = constrain(abs(command), 0, 255);

  digitalWrite(motor.in1, forward ? HIGH : LOW);
  digitalWrite(motor.in2, forward ? LOW : HIGH);
  analogWrite(motor.pwm, pwm);
}

int parseIntOrDefault(char *token, int fallback) {
  if (token == NULL) {
    return fallback;
  }
  return atoi(token);
}

void driveRaw(const char *cmd, int pwm) {
  setOneMotor(MOTOR_A, 0);
  setOneMotor(MOTOR_B, 0);
  setOneMotor(MOTOR_C, 0);
  setOneMotor(MOTOR_D, 0);

  if (strcmp(cmd, "AF") == 0) setOneMotor(MOTOR_A, pwm);
  else if (strcmp(cmd, "AB") == 0) setOneMotor(MOTOR_A, -pwm);
  else if (strcmp(cmd, "BF") == 0) setOneMotor(MOTOR_B, pwm);
  else if (strcmp(cmd, "BB") == 0) setOneMotor(MOTOR_B, -pwm);
  else if (strcmp(cmd, "CF") == 0) setOneMotor(MOTOR_C, pwm);
  else if (strcmp(cmd, "CB") == 0) setOneMotor(MOTOR_C, -pwm);
  else if (strcmp(cmd, "DF") == 0) setOneMotor(MOTOR_D, pwm);
  else if (strcmp(cmd, "DB") == 0) setOneMotor(MOTOR_D, -pwm);
}

void driveHighLevel(const char *cmd, int pwm) {
  int a = 0;
  int b = 0;
  int c = 0;
  int d = 0;

  if (strcmp(cmd, "F") == 0) {
    a = -pwm;
    b = -pwm;
    c = pwm;
    d = pwm;
  } else if (strcmp(cmd, "B") == 0) {
    a = pwm;
    b = pwm;
    c = -pwm;
    d = -pwm;
  } else if (strcmp(cmd, "L") == 0) {
    a = -pwm;
    b = pwm;
    c = -pwm;
    d = pwm;
  } else if (strcmp(cmd, "R") == 0) {
    a = pwm;
    b = -pwm;
    c = pwm;
    d = -pwm;
  }

  setOneMotor(MOTOR_A, a);
  setOneMotor(MOTOR_B, b);
  setOneMotor(MOTOR_C, c);
  setOneMotor(MOTOR_D, d);
}

bool isRawCommand(const char *cmd) {
  return strcmp(cmd, "AF") == 0 || strcmp(cmd, "AB") == 0 ||
         strcmp(cmd, "BF") == 0 || strcmp(cmd, "BB") == 0 ||
         strcmp(cmd, "CF") == 0 || strcmp(cmd, "CB") == 0 ||
         strcmp(cmd, "DF") == 0 || strcmp(cmd, "DB") == 0;
}

bool isHighLevelCommand(const char *cmd) {
  return strcmp(cmd, "F") == 0 || strcmp(cmd, "B") == 0 ||
         strcmp(cmd, "L") == 0 || strcmp(cmd, "R") == 0;
}

void upperToken(char *token) {
  for (uint8_t i = 0; token[i] != '\0'; i++) {
    token[i] = (char)toupper(token[i]);
  }
}

void handleCommand(char *line) {
  char *cmd = strtok(line, " \t\r\n");
  if (cmd == NULL) {
    return;
  }

  upperToken(cmd);

  if (strcmp(cmd, "S") == 0) {
    stopAll();
    stopAtMs = 0;
    Serial.println("OK S");
    return;
  }

  int requestedPwm = parseIntOrDefault(strtok(NULL, " \t\r\n"), DEFAULT_PWM);
  int pwm = constrain(requestedPwm, 0, 255);
  unsigned long requestedDuration = (unsigned long)parseIntOrDefault(
    strtok(NULL, " \t\r\n"),
    DEFAULT_DURATION_MS
  );
  unsigned long duration = constrain(requestedDuration, 0, MAX_DURATION_MS);

  if (!isRawCommand(cmd) && !isHighLevelCommand(cmd)) {
    Serial.println("ERR unknown command");
    return;
  }

  digitalWrite(STBY_PIN, HIGH);
  if (isRawCommand(cmd)) {
    driveRaw(cmd, pwm);
  } else {
    driveHighLevel(cmd, pwm);
  }

  lastCommandMs = millis();
  stopAtMs = lastCommandMs + duration;

  Serial.print("OK ");
  Serial.print(cmd);
  Serial.print(" pwm=");
  Serial.print(pwm);
  Serial.print(" ms=");
  Serial.println(duration);
}

void setup() {
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
  Serial.println("D24A four-motor serial bridge ready");
}

void loop() {
  static char line[64];
  static uint8_t pos = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (pos > 0) {
        line[pos] = '\0';
        handleCommand(line);
        pos = 0;
      }
    } else if (pos < sizeof(line) - 1) {
      line[pos++] = c;
    }
  }

  unsigned long now = millis();
  if (stopAtMs != 0 && now >= stopAtMs) {
    stopAll();
    stopAtMs = 0;
    Serial.println("OK auto-stop");
  }

  if (lastCommandMs != 0 && now - lastCommandMs > WATCHDOG_MS && stopAtMs == 0) {
    stopAll();
    lastCommandMs = 0;
  }
}

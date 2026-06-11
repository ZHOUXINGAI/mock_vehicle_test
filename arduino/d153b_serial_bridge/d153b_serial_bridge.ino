// Lubancat/PC -> USB serial -> Arduino UNO -> D153B/TB6612 -> two DC motors.
//
// D153B wiring to Arduino UNO:
//   PWMA -> D3
//   AIN2 -> D4
//   AIN1 -> D5
//   STBY -> D7
//   BIN1 -> D8
//   BIN2 -> D9
//   PWMB -> D10
//   GND  -> Arduino GND
//
// Serial commands at 115200 baud:
//   F [pwm] [ms]  forward
//   B [pwm] [ms]  backward
//   L [pwm] [ms]  pivot left
//   R [pwm] [ms]  pivot right
//   S             stop
//   Q [pwm] [ms]  left wheel forward only
//   A [pwm] [ms]  left wheel backward only
//   E [pwm] [ms]  right wheel forward only
//   D [pwm] [ms]  right wheel backward only

const uint8_t PWMA_PIN = 3;
const uint8_t AIN2_PIN = 4;
const uint8_t AIN1_PIN = 5;
const uint8_t STBY_PIN = 7;
const uint8_t BIN1_PIN = 8;
const uint8_t BIN2_PIN = 9;
const uint8_t PWMB_PIN = 10;

const int DEFAULT_PWM = 220;
const unsigned long DEFAULT_DURATION_MS = 700;
const unsigned long MAX_DURATION_MS = 3000;
const unsigned long WATCHDOG_MS = 700;

// Change these if physical forward/backward is reversed.
const bool LEFT_INVERT = false;
const bool RIGHT_INVERT = false;

unsigned long stopAtMs = 0;
unsigned long lastCommandMs = 0;

void stopAll() {
  analogWrite(PWMA_PIN, 0);
  analogWrite(PWMB_PIN, 0);
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN2_PIN, LOW);
  digitalWrite(BIN1_PIN, LOW);
  digitalWrite(BIN2_PIN, LOW);
  digitalWrite(STBY_PIN, LOW);
}

void setOneMotor(
  int command,
  uint8_t pwmPin,
  uint8_t in1Pin,
  uint8_t in2Pin,
  bool invert
) {
  command = constrain(command, -255, 255);

  if (command == 0) {
    analogWrite(pwmPin, 0);
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    return;
  }

  bool forward = command > 0;
  if (invert) {
    forward = !forward;
  }

  int pwm = constrain(abs(command), 0, 255);
  digitalWrite(in1Pin, forward ? HIGH : LOW);
  digitalWrite(in2Pin, forward ? LOW : HIGH);
  analogWrite(pwmPin, pwm);
}

void driveMotors(int leftCommand, int rightCommand) {
  digitalWrite(STBY_PIN, HIGH);
  setOneMotor(leftCommand, PWMA_PIN, AIN1_PIN, AIN2_PIN, LEFT_INVERT);
  setOneMotor(rightCommand, PWMB_PIN, BIN1_PIN, BIN2_PIN, RIGHT_INVERT);
}

int parseIntOrDefault(char *token, int fallback) {
  if (token == NULL) {
    return fallback;
  }
  return atoi(token);
}

void handleCommand(char *line) {
  char *cmdToken = strtok(line, " \t\r\n");
  if (cmdToken == NULL) {
    return;
  }

  char cmd = toupper(cmdToken[0]);
  int requestedPwm = parseIntOrDefault(strtok(NULL, " \t\r\n"), DEFAULT_PWM);
  int pwm = constrain(requestedPwm, 0, 255);
  unsigned long requestedDuration = (unsigned long)parseIntOrDefault(
    strtok(NULL, " \t\r\n"),
    DEFAULT_DURATION_MS
  );
  unsigned long duration = constrain(requestedDuration, 0, MAX_DURATION_MS);

  int left = 0;
  int right = 0;

  switch (cmd) {
    case 'F':
      left = pwm;
      right = pwm;
      break;
    case 'B':
      left = -pwm;
      right = -pwm;
      break;
    case 'L':
      left = -pwm;
      right = pwm;
      break;
    case 'R':
      left = pwm;
      right = -pwm;
      break;
    case 'Q':
      left = pwm;
      right = 0;
      break;
    case 'A':
      left = -pwm;
      right = 0;
      break;
    case 'E':
      left = 0;
      right = pwm;
      break;
    case 'D':
      left = 0;
      right = -pwm;
      break;
    case 'S':
      stopAll();
      stopAtMs = 0;
      Serial.println("OK S");
      return;
    default:
      Serial.println("ERR unknown command");
      return;
  }

  driveMotors(left, right);
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
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);

  stopAll();
  Serial.begin(115200);
  Serial.println("D153B serial bridge ready");
}

void loop() {
  static char line[48];
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

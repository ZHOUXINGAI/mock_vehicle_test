// R9DS PWM input debug only.
//
// This sketch NEVER enables the D24A motor driver.
// Use it to identify which R9DS PWM output follows each transmitter stick/switch.
//
// Test inputs:
//   R9DS signal under test -> Arduino A3
//   Optional signal        -> Arduino A4
//   Optional signal        -> Arduino A5
//   R9DS +                 -> Arduino 5V
//   R9DS -                 -> Arduino GND

const uint8_t INPUT_A3 = A3;
const uint8_t INPUT_A4 = A4;
const uint8_t INPUT_A5 = A5;

const uint8_t STBY_PIN = A2;
const unsigned long PULSE_TIMEOUT_US = 25000;
const unsigned long PRINT_INTERVAL_MS = 200;

unsigned long lastPrintMs = 0;

int readPulse(uint8_t pin) {
  unsigned long pulse = pulseIn(pin, HIGH, PULSE_TIMEOUT_US);

  if (pulse < 800 || pulse > 2200) {
    return 0;
  }

  return (int)pulse;
}

void setup() {
  pinMode(INPUT_A3, INPUT);
  pinMode(INPUT_A4, INPUT);
  pinMode(INPUT_A5, INPUT);

  // Keep D24A disabled if STBY is still wired to Arduino A2.
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, LOW);

  Serial.begin(115200);
  Serial.println("R9DS PWM input debug ready");
  Serial.println("No motor outputs are driven by this sketch.");
}

void loop() {
  unsigned long now = millis();
  if (now - lastPrintMs < PRINT_INTERVAL_MS) {
    return;
  }
  lastPrintMs = now;

  int a3 = readPulse(INPUT_A3);
  int a4 = readPulse(INPUT_A4);
  int a5 = readPulse(INPUT_A5);

  Serial.print("A3_us=");
  Serial.print(a3);
  Serial.print(" A4_us=");
  Serial.print(a4);
  Serial.print(" A5_us=");
  Serial.println(a5);
}

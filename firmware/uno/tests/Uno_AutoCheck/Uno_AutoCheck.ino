// AUTO-CHECK: measures every direction setting and prints the lines to paste
// into the balancing code. Serial Monitor 115200, line ending "Newline".
// Follow the prompts. You only press Enter, or type y / n.
#include <Wire.h>

const uint8_t MPU = 0x68, EEP = 8, AIN1 = 5, AIN2 = 6, BIN1 = 9, BIN2 = 10;
volatile long eL = 0, eR = 0;
void isrL() { if (PIND & _BV(4)) eL--; else eL++; }    // same counting as the balancing code
void isrR() { if (PIND & _BV(7)) eR--; else eR++; }

int16_t r[7];
bool rd() {
  Wire.beginTransmission(MPU); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, (uint8_t)14) != 14) return false;
  for (uint8_t i = 0; i < 7; i++) { uint8_t h = Wire.read(), l = Wire.read(); r[i] = (int16_t)((h << 8) | l); }
  return true;
}
void avg(float& ax, float& ay, float& gx, float& gy) {
  ax = ay = gx = gy = 0; int n = 0;
  for (int i = 0; i < 200; i++) { if (rd()) { ax += r[0]; ay += r[1]; gx += r[4]; gy += r[5]; n++; } delay(4); }
  ax /= n * 8192.0; ay /= n * 8192.0; gx /= n; gy /= n;
}
void waitEnter(const __FlashStringHelper* msg) {
  Serial.println(msg); Serial.println(F("   ...then press Enter"));
  while (!Serial.available()) {}
  delay(30); while (Serial.available()) Serial.read();
}
bool askYes(const __FlashStringHelper* q) {
  Serial.print(q); Serial.println(F("  (type y or n, Enter)"));
  while (true) {
    while (!Serial.available()) {}
    char c = Serial.read();
    if (c == 'y' || c == 'Y') { delay(30); while (Serial.available()) Serial.read(); return true; }
    if (c == 'n' || c == 'N') { delay(30); while (Serial.available()) Serial.read(); return false; }
  }
}
void drive(uint8_t in1, uint8_t in2, int p) { analogWrite(in1, 255); analogWrite(in2, 255 - p); }   // slow decay, + only
void stopAll() { analogWrite(AIN1, 0); analogWrite(AIN2, 0); analogWrite(BIN1, 0); analogWrite(BIN2, 0); }

void setup() {
  digitalWrite(EEP, LOW); pinMode(EEP, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); stopAll();
  TCCR1A = (TCCR1A & ~_BV(WGM11)) | _BV(WGM10); TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
  pinMode(2, INPUT_PULLUP); pinMode(4, INPUT_PULLUP); pinMode(3, INPUT_PULLUP); pinMode(7, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(3), isrR, RISING);
  Serial.begin(115200);
  Wire.begin(); digitalWrite(SDA, LOW); digitalWrite(SCL, LOW); Wire.setClock(400000);
  Wire.beginTransmission(MPU); Wire.write(0x6B); Wire.write(0x01); Wire.endTransmission();
  Wire.beginTransmission(MPU); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission();
  Wire.beginTransmission(MPU); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();   // 500 dps
  Wire.beginTransmission(MPU); Wire.write(0x1C); Wire.write(0x08); Wire.endTransmission();   // 4 g
  delay(100);
  if (!rd()) { Serial.println(F("MPU not found - check A4/A5")); while (true) {} }
  Serial.println(F("\n========= AUTO-CHECK ========="));

  // ---- 1. which axis is forward, and which sign ----
  float ax0, ay0, gx0, gy0, ax1, ay1, gx1, gy1;
  waitEnter(F("\n1) Hold the robot UPRIGHT and STILL"));
  avg(ax0, ay0, gx0, gy0);
  waitEnter(F("2) Now tilt it FORWARD (towards its front) about 20 deg and HOLD still"));
  avg(ax1, ay1, gx1, gy1);
  float dx = ax1 - ax0, dy = ay1 - ay0;
  int imuF; bool useX = fabs(dx) > fabs(dy);
  if (max(fabs(dx), fabs(dy)) < 0.12) { Serial.println(F("   Hardly any change - tilt more and reset the Uno")); while (true) {} }
  if (useX) imuF = dx < 0 ? 0 : 1; else imuF = dy < 0 ? 2 : 3;

  // ---- 2. gyro direction: integrate while tipping forward ----
  waitEnter(F("3) Hold it UPRIGHT again. After Enter you have 3 s: tip it FORWARD about 20 deg and hold"));
  float integ = 0; uint32_t t0 = millis(), last = micros();
  while (millis() - t0 < 3000) {
    if (!rd()) continue;
    float dt = (micros() - last) / 1e6; last = micros();
    int8_t s = (imuF == 0 || imuF == 2) ? 1 : -1;
    float pr = (imuF < 2) ? s * (r[5] - gy0) : -s * (r[4] - gx0);   // pitch raw, + = tipping forward
    integ += pr / 65.5 * dt;
    delay(3);
  }
  bool gyroInv = integ < 0;
  Serial.print(F("   gyro measured ")); Serial.print(integ, 1); Serial.println(F(" deg (expected about +20)"));
  if (fabs(integ) < 5) Serial.println(F("   (small - tip it more firmly; result may be unreliable)"));

  // ---- 3. motors + encoders (wheels in the air) ----
  waitEnter(F("\n4) Hold the robot with BOTH WHEELS IN THE AIR. The LEFT wheel will spin for 1 s"));
  digitalWrite(EEP, HIGH);
  long l0 = eL; drive(AIN1, AIN2, 120); delay(1000); stopAll(); long dL = eL - l0;
  bool lFwd = askYes(F("   Did the LEFT wheel's TOP move towards the robot's FRONT?"));
  waitEnter(F("5) Wheels still in the air. The RIGHT wheel will spin for 1 s"));
  long r0 = eR; drive(BIN1, BIN2, 120); delay(1000); stopAll(); long dR = eR - r0;
  bool rFwd = askYes(F("   Did the RIGHT wheel's TOP move towards the robot's FRONT?"));
  digitalWrite(EEP, LOW);

  // ---- results ----
  Serial.println(F("\n========= RESULTS ========="));
  Serial.print(F("left encoder counted ")); Serial.print(dL); Serial.print(F(", right counted ")); Serial.println(dR);
  if (labs(dL) < 30) Serial.println(F("!! LEFT encoder hardly counts - check green D2 / yellow D4 / power"));
  if (labs(dR) < 30) Serial.println(F("!! RIGHT encoder hardly counts - check green D3 / yellow D7 / power"));
  bool encLInv = (dL > 0) != lFwd;    // counts must be + when the wheel rolls forward
  bool encRInv = (dR > 0) != rFwd;
  Serial.println(F("\nPaste these lines into the balancing code (replace the old ones):\n"));
  Serial.print(F("#define MOTOR_L_INVERT  ")); Serial.println(lFwd ? F("false") : F("true"));
  Serial.print(F("#define MOTOR_R_INVERT  ")); Serial.println(rFwd ? F("false") : F("true"));
  Serial.print(F("#define ENC_L_INVERT    ")); Serial.println(encLInv ? F("true") : F("false"));
  Serial.print(F("#define ENC_R_INVERT    ")); Serial.println(encRInv ? F("true") : F("false"));
  Serial.print(F("#define IMU_FORWARD     ")); Serial.println(imuF);
  Serial.print(F("#define GYRO_INVERT     ")); Serial.println(gyroInv ? 1 : 0);
  Serial.println(F("\nAfter this, the TRIM sign may change: measure it again (trim finder)."));
}
void loop() {}

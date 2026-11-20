/*
  TWO MOTORS + CONTROL TEST  (Arduino Uno, no display)
  ----------------------------------------------------
  Serial Monitor 115200 baud, line ending "Newline". Type a letter + Enter.

  MODES
   1  Manual drive   : w = forward, s = back, a = left, d = right, x = stop,
                       + / - = power. Each key runs for 1 s, then stops.
   2  Speed control  : both wheels held at a target rpm using the encoders
                       (closed loop). Grab a wheel lightly: power goes up.
                       + / - = target rpm, x = stop.
   3  Tilt test      : needs the MPU-6050 on A4/A5. Tilt the robot by hand:
                       the wheels must spin TOWARDS the side it leans to
                       (lean forward -> wheels roll forward). This is the
                       last check before balancing. x = stop.
   0  stop everything / back to the menu

  WHEELS OFF THE GROUND for every mode.

  WIRING (final layout)
    DRV8833 IN1->D5 IN2->D6 IN3->D9 IN4->D10  EEP->5V  GND->GND  VCC->battery +
    LEFT  motor: red OUT1, white OUT2, black 5V, blue GND, green D2, yellow D4
    RIGHT motor: red OUT3, white OUT4, black 5V, blue GND, green D3, yellow D7
    MPU-6050: VCC 3.3V, GND, SDA A4, SCL A5
    Battery + -> DRV VCC and Uno VIN,  battery - -> GND row (with Uno GND)
*/
#include <Wire.h>

// ---------------- settings (copy the final values into the balancing code) ----
#define MOTOR_L_INVERT false
#define MOTOR_R_INVERT true
#define ENC_L_INVERT   false
#define ENC_R_INVERT   true
#define IMU_FORWARD    0      // 0=+X 1=-X 2=+Y 3=-Y arrow points to the robot's front
#define IMU_ACCEL_NO_Z 1      // your MPU's Z axis is faulty -> tilt from the forward axis

const uint8_t AIN1 = 5, AIN2 = 6, BIN1 = 9, BIN2 = 10;
const uint8_t ENC_L_A = 2, ENC_R_A = 3;      // B channels: D4, D7
const float COUNTS_REV = 350.0f;
const int PWM_CAP = 204;                     // 80 %: 6 V motors on an 8 V battery

// ---------------- encoders ----------------
volatile long encL = 0, encR = 0;
void isrL() { if (PIND & _BV(4)) encL--; else encL++; }
void isrR() { if (PIND & _BV(7)) encR--; else encR++; }
long cntL() { noInterrupts(); long c = encL; interrupts(); return ENC_L_INVERT ? -c : c; }
long cntR() { noInterrupts(); long c = encR; interrupts(); return ENC_R_INVERT ? -c : c; }

// ---------------- motors (slow decay, + = forward) ----------------
void driveOne(uint8_t in1, uint8_t in2, bool inv, int p) {
  if (inv) p = -p;
  p = constrain(p, -255, 255);
  if (p >= 0) { analogWrite(in1, 255); analogWrite(in2, 255 - p); }
  else        { analogWrite(in1, 255 + p); analogWrite(in2, 255); }
}
void motors(int l, int r) {
  l = constrain(l, -PWM_CAP, PWM_CAP); r = constrain(r, -PWM_CAP, PWM_CAP);
  driveOne(AIN1, AIN2, MOTOR_L_INVERT, l); driveOne(BIN1, BIN2, MOTOR_R_INVERT, r);
}
void stopAll() { analogWrite(AIN1, 0); analogWrite(AIN2, 0); analogWrite(BIN1, 0); analogWrite(BIN2, 0); }

// ---------------- MPU-6050 ----------------
const uint8_t MPU = 0x68;
bool mpuOk = false;
float gyroOff = 0, angle = 0;
void mpuWrite(uint8_t r, uint8_t v) { Wire.beginTransmission(MPU); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
bool mpuRead(int16_t raw[7]) {
  Wire.beginTransmission(MPU); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, (uint8_t)14) != 14) return false;
  for (uint8_t i = 0; i < 7; i++) { uint8_t h = Wire.read(), l = Wire.read(); raw[i] = (int16_t)((h << 8) | l); }
  return true;
}
// forward-axis accel (g) and pitch rate (deg/s, + = tipping forward) for a flat, chip-up board
void imuAxes(int16_t raw[7], float& fwd, float& up, float& pitchRaw) {
  int ax = 0, sign = 1;
  if (IMU_FORWARD == 0) { ax = 0; sign = 1; } else if (IMU_FORWARD == 1) { ax = 0; sign = -1; }
  else if (IMU_FORWARD == 2) { ax = 1; sign = 1; } else { ax = 1; sign = -1; }
  fwd = sign * raw[ax] / 8192.0f;
  up = raw[2] / 8192.0f;
  // pitch axis = up x forward: +X fwd -> +Y, -X -> -Y, +Y fwd -> -X, -Y -> +X
  if (ax == 0) pitchRaw = sign * raw[5];
  else         pitchRaw = -sign * raw[4];
}
float tiltFromAccel(float fwd, float up) {
#if IMU_ACCEL_NO_Z
  return asin(constrain(-fwd, -1.0f, 1.0f)) * RAD_TO_DEG;
#else
  return atan2(-fwd, up) * RAD_TO_DEG;
#endif
}
bool mpuBegin() {
  Wire.begin(); digitalWrite(SDA, LOW); digitalWrite(SCL, LOW); Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  mpuWrite(0x6B, 0x80); delay(100);
  mpuWrite(0x6B, 0x01); mpuWrite(0x1A, 0x03); mpuWrite(0x1B, 0x08); mpuWrite(0x1C, 0x08);
  int16_t raw[7];
  if (!mpuRead(raw)) return false;
  // gyro calibration (keep still)
  float s = 0, f, u, p; int n = 0;
  for (int i = 0; i < 300; i++) { if (mpuRead(raw)) { imuAxes(raw, f, u, p); s += p; n++; } delay(3); }
  gyroOff = n ? s / n : 0;
  imuAxes(raw, f, u, p); angle = tiltFromAccel(f, u);
  return true;
}

// ---------------- state ----------------
uint8_t mode = 0;
int power = 120;          // mode 1
float targetRpm = 120;    // mode 2
uint32_t driveUntil = 0;
int manL = 0, manR = 0;

void menu() {
  stopAll();
  Serial.println(F("\n===== CONTROL TEST (wheels off the ground) ====="));
  Serial.println(F(" 1 = manual drive   (w a s d, x stop, +/- power)"));
  Serial.println(F(" 2 = speed control  (encoders hold the rpm, +/- target, x stop)"));
  Serial.print  (F(" 3 = tilt test      (MPU "));
  Serial.print(mpuOk ? F("found") : F("NOT found - check A4/A5"));
  Serial.println(F(")"));
  Serial.println(F(" 0 = stop / this menu"));
}

void setup() {
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  TCCR1A = (TCCR1A & ~_BV(WGM11)) | _BV(WGM10);        // D9/D10 ~976 Hz like D5/D6
  TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
  stopAll();
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(4, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(7, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrR, RISING);
  Serial.println(F("\nStarting. Keep the robot STILL for 1 s (gyro calibration)..."));
  mpuOk = mpuBegin();
  menu();
}

// ---------------- mode 2: closed-loop speed (PI per wheel) ----------------
float iL = 0, iR = 0, rpmL = 0, rpmR = 0;
long lastCL = 0, lastCR = 0;
void speedLoop(float dt) {
  long cL = cntL(), cR = cntR();
  float mL = (cL - lastCL) / COUNTS_REV * 60.0f / dt, mR = (cR - lastCR) / COUNTS_REV * 60.0f / dt;
  lastCL = cL; lastCR = cR;
  rpmL += 0.3f * (mL - rpmL); rpmR += 0.3f * (mR - rpmR);
  const float kff = 0.62f, kp = 0.8f, ki = 4.0f;     // PWM per rpm
  float eL = targetRpm - rpmL, eR = targetRpm - rpmR;
  iL = constrain(iL + eL * dt, -60, 60); iR = constrain(iR + eR * dt, -60, 60);
  motors((int)(kff * targetRpm + kp * eL + ki * iL), (int)(kff * targetRpm + kp * eR + ki * iR));
}

// ---------------- mode 3: tilt -> wheels (pre-balance check) ----------------
void tiltLoop(float dt) {
  int16_t raw[7]; float f, u, p;
  if (!mpuRead(raw)) { stopAll(); return; }
  imuAxes(raw, f, u, p);
  float rate = (p - gyroOff) / 65.5f;
  angle = 0.98f * (angle + rate * dt) + 0.02f * tiltFromAccel(f, u);
  int pwm = 0;
  if (fabs(angle) < 40) pwm = (int)(angle * 8);        // gentle: 10 deg -> PWM 80
  motors(pwm, pwm);
}

void loop() {
  static uint32_t last = micros(), lastPrint = 0;
  // ---- commands ----
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r' || c == ' ') return;
    if (c == '0') { mode = 0; menu(); }
    else if (c == '1') { mode = 1; stopAll(); Serial.println(F("MANUAL: w a s d, x stop, +/- power")); }
    else if (c == '2') { mode = 2; iL = iR = 0; lastCL = cntL(); lastCR = cntR();
                         Serial.println(F("SPEED CONTROL: +/- target rpm, x stop")); }
    else if (c == '3') { if (!mpuOk) Serial.println(F("MPU not found - check A4/A5, 3.3V, GND"));
                         else { mode = 3; Serial.println(F("TILT TEST: lean forward -> wheels forward. x stop")); } }
    else if (c == 'x') { stopAll(); manL = manR = 0; if (mode == 2) targetRpm = 0; Serial.println(F("stop")); }
    else if (mode == 1) {
      if (c == '+') { power = min(PWM_CAP, power + 20); Serial.print(F("power ")); Serial.println(power); }
      if (c == '-') { power = max(0, power - 20);       Serial.print(F("power ")); Serial.println(power); }
      if (c == 'w') { manL = power;  manR = power; }
      if (c == 's') { manL = -power; manR = -power; }
      if (c == 'a') { manL = -power; manR = power; }      // spin left
      if (c == 'd') { manL = power;  manR = -power; }     // spin right
      if (strchr("wasd", c)) { motors(manL, manR); driveUntil = millis() + 1000; }
    } else if (mode == 2) {
      if (c == '+') targetRpm = min(250.0f, targetRpm + 30);
      if (c == '-') targetRpm = max(-250.0f, targetRpm - 30);
      Serial.print(F("target ")); Serial.print(targetRpm, 0); Serial.println(F(" rpm"));
    }
  }

  // ---- 5 ms control tick ----
  uint32_t now = micros();
  if (now - last < 5000) return;
  float dt = (now - last) / 1e6f; last = now;

  if (mode == 1 && driveUntil && millis() > driveUntil) { stopAll(); driveUntil = 0; }
  if (mode == 2) speedLoop(dt);
  if (mode == 3) tiltLoop(dt);

  // ---- status print, 4 times a second ----
  if (millis() - lastPrint > 250 && mode) {
    lastPrint = millis();
    Serial.print(F("encL ")); Serial.print(cntL()); Serial.print(F("  encR ")); Serial.print(cntR());
    if (mode == 2) { Serial.print(F("  rpm L ")); Serial.print(rpmL, 0); Serial.print(F(" R ")); Serial.print(rpmR, 0);
                     Serial.print(F(" (target ")); Serial.print(targetRpm, 0); Serial.print(')'); }
    if (mode == 3) { Serial.print(F("  tilt ")); Serial.print(angle, 1); Serial.print(F(" deg")); }
    Serial.println();
  }
}

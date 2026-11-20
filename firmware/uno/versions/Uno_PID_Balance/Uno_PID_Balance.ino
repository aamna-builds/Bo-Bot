/*
 * Uno_PID_Balance - self-balancing robot (Arduino Uno + MPU-6050 + DRV8833)
 * Serial Monitor: 115200 baud, line ending "Newline".
 *
 * 1. Paste the 6 lines from the Auto-Check into section 1 below, then upload.
 * 2. Type x (stop). Hold the robot at its balance point, still. Type t. Type save.
 * 3. Type g to balance. Power-up later: hold upright + still -> it calibrates
 *    and balances by itself.
 *
 * Commands: g start | x stop | t measure trim | p live readout | show | save | defaults
 *           kp ki kd trim kv kx cap decay minp <value>
 *
 * Wiring: DRV8833 IN1 D5, IN2 D6, IN3 D9, IN4 D10, EEP D8 (+10k D8 to GND), VCC battery +, GND
 *         LEFT  motor: red OUT1, white OUT2, black 5V, blue GND, green D2, yellow D4
 *         RIGHT motor: red OUT3, white OUT4, black 5V, blue GND, green D3, yellow D7
 *         MPU-6050: VCC 3.3V, GND, SDA A4, SCL A5
 *         Battery + -> DRV VCC and Uno VIN; battery - -> GND
 */
#include <Wire.h>
#include <EEPROM.h>

// ===================== 1. PASTE THE 6 AUTO-CHECK LINES HERE ==================
#define MOTOR_L_INVERT  false
#define MOTOR_R_INVERT  true
#define ENC_L_INVERT    false
#define ENC_R_INVERT    true
#define IMU_FORWARD     1
#define GYRO_INVERT     0
// =============================================================================
#define IMU_ACCEL_NO_Z  1       // your MPU's Z accelerometer is dead - keep 1

// ===================== 2. PID VALUES (defaults) =============================
struct Gains {
  float kp   = 45.0;   // power per degree of tilt         (main push back)
  float ki   = 0.0;    // keep 0
  float kd   = 0.25;   // power per deg/s of tilt speed    (damping; too high = buzz)
  float trim = 0.0;    // balance point in degrees - measure with the t command
  float kv   = 20.0;   // deg of lean per m/s   (stops it running away - keep on)
  float kx   = 0.9;    // deg of lean per metre (pulls it back to where it started)
  float cap  = 230;    // max motor power 0..255
  float decay = 0;     // 0 = slow-decay drive (stronger at low power)
  float minp = 0;      // extra power for tiny corrections (0 = off, avoids trembling)
};
Gains g;
const float FALL_ANGLE  = 40.0;  // motors off beyond this
#define AUTO_START      1        // 1 = calibrate + balance as soon as powered
#define TRIM_AT_CALIBRATION 0    // 1 = the angle you hold it at during calibration becomes the trim
const float START_ANGLE = 15.0;  // at power-up: start at once if within this of trim
const float ARM_ANGLE   = 4.0;   // after a fall: restart when stood up within this...
const uint16_t ARM_MS   = 500;   // ...for this long
const float MAX_LEAN    = 10.0;  // most the speed loop may lean the robot

// ===================== pins / constants =====================================
const uint8_t AIN1 = 5, AIN2 = 6, BIN1 = 9, BIN2 = 10;
const uint8_t DRV_EEP = 8;     // DRV8833 EEP/nSLEEP: LOW = driver asleep, HIGH = awake
const uint8_t ENC_L_A = 2, ENC_R_A = 3;          // B channels on D4, D7
const float COUNTS_REV = 350.0, WHEEL_R = 0.017;
const uint16_t LOOP_US = 5000;                    // 200 Hz
const float DT = LOOP_US / 1e6;

// ===================== encoders =============================================
volatile long encL = 0, encR = 0;
void isrL() { if (PIND & _BV(4)) encL--; else encL++; }
void isrR() { if (PIND & _BV(7)) encR--; else encR++; }
long cntL() { noInterrupts(); long c = encL; interrupts(); return ENC_L_INVERT ? -c : c; }
long cntR() { noInterrupts(); long c = encR; interrupts(); return ENC_R_INVERT ? -c : c; }

// ===================== motors (slow decay, + = forward) =====================
void driveOne(uint8_t in1, uint8_t in2, bool inv, int p) {
  if (inv) p = -p;
  p = constrain(p, -255, 255);
  if (g.decay >= 0.5) {                      // simple drive, exactly like the 2-motor test
    if (p >= 0) { analogWrite(in1, p); digitalWrite(in2, LOW); }
    else        { digitalWrite(in1, LOW); analogWrite(in2, -p); }
  } else {                                   // slow decay
    if (p >= 0) { analogWrite(in1, 255); analogWrite(in2, 255 - p); }
    else        { analogWrite(in1, 255 + p); analogWrite(in2, 255); }
  }
}
void motors(int p) { driveOne(AIN1, AIN2, MOTOR_L_INVERT, p); driveOne(BIN1, BIN2, MOTOR_R_INVERT, p); }
void motorsOff() { analogWrite(AIN1, 0); analogWrite(AIN2, 0); analogWrite(BIN1, 0); analogWrite(BIN2, 0); }
void driverSleep(bool sleep) { digitalWrite(DRV_EEP, sleep ? LOW : HIGH); }

// ===================== MPU-6050 =============================================
const uint8_t MPU = 0x68;
float gyroOff = 0;
void mpuWrite(uint8_t r, uint8_t v) { Wire.beginTransmission(MPU); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
// reads forward-axis accel (g), up accel (g) and RAW pitch gyro (+ = tipping forward)
bool mpuRead(float& fwd, float& up, float& pitchRaw) {
  Wire.beginTransmission(MPU); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, (uint8_t)14) != 14) return false;
  int16_t r[7];
  for (uint8_t i = 0; i < 7; i++) { uint8_t h = Wire.read(), l = Wire.read(); r[i] = (int16_t)((h << 8) | l); }
  int8_t s = (IMU_FORWARD == 0 || IMU_FORWARD == 2) ? 1 : -1;
  if (IMU_FORWARD < 2) { fwd = s * r[0] / 8192.0; pitchRaw =  s * r[5]; }   // X forward -> pitch about Y
  else                 { fwd = s * r[1] / 8192.0; pitchRaw = -s * r[4]; }   // Y forward -> pitch about X
  up = r[2] / 8192.0;
#if GYRO_INVERT
  pitchRaw = -pitchRaw;
#endif
  return true;
}
float accelAngle(float fwd, float up) {
#if IMU_ACCEL_NO_Z
  return asin(constrain(-fwd, -1.0, 1.0)) * RAD_TO_DEG;
#else
  return atan2(-fwd, up) * RAD_TO_DEG;
#endif
}
// average the gyro (offset) and the tilt angle (trim) while held still; false if it moved
float calTrim = 0;
bool calibrate() {
  float f, u, p, s = 0, q = 0, a = 0; int n = 0;
  for (int i = 0; i < 400; i++) { if (mpuRead(f, u, p)) { s += p; q += p * p; a += accelAngle(f, u); n++; } delay(3); }
  if (n < 200) return false;
  gyroOff = s / n;
  calTrim = a / n;
  return (q / n - gyroOff * gyroOff) < 197.0 * 197.0;   // < 3 deg/s of shaking (a steady hand is OK)
}

// ===================== state =================================================
enum State : uint8_t { DISARMED, ARMED, BALANCING };
State state = DISARMED;
float angle = 0, rate = 0, speed = 0, position = 0, target = 0, iTerm = 0;
long lastL = 0, lastR = 0;
long win[5]; uint8_t wi = 0, wn = 0;
uint32_t uprightSince = 0;
bool printOn = false;
int out = 0;

void stopBalancing(const __FlashStringHelper* why, bool autoRestart) {
  motorsOff(); driverSleep(true); out = 0; uprightSince = 0;
  state = autoRestart ? ARMED : DISARMED;
  Serial.print(F("STOPPED: ")); Serial.println(why);
  Serial.println(autoRestart ? F("Stand me up near my balance point to restart.")
                             : F("Type g to start again."));
}
void startBalancing(const __FlashStringHelper* how) {
  motorsOff(); driverSleep(false);
  state = BALANCING; iTerm = 0; speed = 0; position = 0; wn = 0; uprightSince = 0;
  Serial.print(F("BALANCING (")); Serial.print(how); Serial.println(')');
}

// ===================== the control loop (every 5 ms) =========================
void control() {
  float f, u, p;
  if (!mpuRead(f, u, p)) { motorsOff(); return; }
  rate = (p - gyroOff) / 65.5;                                   // deg/s
  angle = 0.98 * (angle + rate * DT) + 0.02 * accelAngle(f, u);  // complementary filter

  // wheel speed + position from the encoders
  long cL = cntL(), cR = cntR();
  long d = (cL - lastL) + (cR - lastR); lastL = cL; lastR = cR;
  win[wi] = d; wi = (wi + 1) % 5; if (wn < 5) wn++;
  long sum = 0; for (uint8_t i = 0; i < wn; i++) sum += win[i];
  float wheelRate = sum * 0.5 / COUNTS_REV * TWO_PI / (wn * DT);          // rad/s vs body
  speed += 0.3 * (WHEEL_R * (wheelRate + rate * DEG_TO_RAD) - speed);    // m/s over ground
  position += speed * DT;

  float err = angle - g.trim;

  // ---- waiting / starting ----
  if (state == DISARMED) return;             // stopped with x: wait for g
  if (state == ARMED) {                      // waiting to be stood up
    if (fabs(err) < ARM_ANGLE) {
      if (!uprightSince) uprightSince = millis();
      if (millis() - uprightSince > ARM_MS) startBalancing(F("stood up"));
    } else uprightSince = 0;
    return;
  }

  // ---- BALANCING ----
  if (fabs(err) > FALL_ANGLE) { stopBalancing(F("fell over"), AUTO_START); return; }

  // outer loop: lean a little to stop rolling away and return to the start
  target = constrain(-(g.kv * speed + g.kx * position), -MAX_LEAN, MAX_LEAN);

  // inner loop: PID on the tilt angle
  float e = err - target;
  iTerm = constrain(iTerm + g.ki * e * DT, -100, 100);
  int cap = (int)constrain(g.cap, 0, 255);
  float o = g.kp * e + iTerm + g.kd * rate;
  if (fabs(o) > 2) o += (o > 0 ? g.minp : -g.minp);       // overcome motor/gear stiction
  out = (int)constrain(o, -cap, cap);
  motors(out);
}

// ===================== Serial commands =======================================
struct Ref { const char* n; float* v; };
Ref refs[] = {{"kp", &g.kp}, {"ki", &g.ki}, {"kd", &g.kd}, {"trim", &g.trim}, {"kv", &g.kv}, {"kx", &g.kx},
              {"cap", &g.cap}, {"decay", &g.decay}, {"minp", &g.minp}};
const uint16_t MAGIC = 0xBA1E;           // new number: old saved values are ignored once

void show() {
  for (Ref& r : refs) { Serial.print(r.n); Serial.print('='); Serial.print(*r.v, 3); Serial.print("  "); }
  Serial.println();
}
void command(char* line) {
  char* c = strtok(line, " "); char* a = strtok(nullptr, " ");
  if (!c) return;
  if (!strcmp(c, "g")) {                     // start balancing RIGHT NOW, wherever it is
    if (fabs(angle - g.trim) > FALL_ANGLE) { Serial.println(F("too far tilted - stand it up first, then g")); return; }
    startBalancing(F("g"));
  }
  else if (!strcmp(c, "x")) stopBalancing(F("x typed"), false);
  else if (!strcmp(c, "p")) printOn = !printOn;
  else if (!strcmp(c, "t")) {                // measure the balance point (trim)
    if (state == BALANCING) { Serial.println(F("type x first, then hold it at its balance point and type t")); return; }
    Serial.println(F("measuring - hold it at the balance point, still..."));
    float f, u, p, a = 0; int n = 0;
    for (int i = 0; i < 300; i++) { if (mpuRead(f, u, p)) { a += accelAngle(f, u); n++; } delay(5); }
    if (n < 100) { Serial.println(F("MPU read failed")); return; }
    g.trim = a / n;
    Serial.print(F("trim = ")); Serial.print(g.trim, 2); Serial.println(F("   (type save to keep)"));
  }
  else if (!strcmp(c, "show")) show();
  else if (!strcmp(c, "save")) { EEPROM.put(0, MAGIC); EEPROM.put(2, g); Serial.println(F("saved")); }
  else if (!strcmp(c, "defaults")) { g = Gains(); Serial.println(F("defaults loaded (type save to keep)")); show(); }
  else {
    for (Ref& r : refs) if (!strcmp(c, r.n) && a) { *r.v = atof(a); Serial.print(c); Serial.print(" = "); Serial.println(*r.v, 3); return; }
    Serial.println(F("commands: g x t p show save defaults | kp ki kd trim kv kx cap decay minp <value>"));
  }
}
void serialPoll() {
  static char buf[24]; static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { buf[n] = 0; command(buf); n = 0; }
    else if (n < sizeof(buf) - 1) buf[n++] = c;
  }
}

// ===================== setup / loop ==========================================
void setup() {
  // driver asleep + motors OFF first, before anything else
  digitalWrite(DRV_EEP, LOW); pinMode(DRV_EEP, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  motorsOff();
  TCCR1A = (TCCR1A & ~_BV(WGM11)) | _BV(WGM10);   // D9/D10 ~976 Hz like D5/D6
  TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);

  Serial.begin(115200);
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(4, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(7, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrR, RISING);

  uint16_t m; EEPROM.get(0, m);
  if (m == MAGIC) { EEPROM.get(2, g); Serial.println(F("\nsaved PID values loaded")); }

  Wire.begin(); digitalWrite(SDA, LOW); digitalWrite(SCL, LOW); Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  mpuWrite(0x6B, 0x80); delay(100);
  mpuWrite(0x6B, 0x01); mpuWrite(0x1A, 0x03); mpuWrite(0x1B, 0x08); mpuWrite(0x1C, 0x08);
  float f, u, p;
  if (!mpuRead(f, u, p)) { Serial.println(F("MPU-6050 NOT FOUND - check A4/A5, 3.3V, GND")); while (true) delay(1000); }

  Serial.println(F("Calibrating - hold the robot upright and STILL for 2 s..."));
  while (!calibrate()) Serial.println(F("  it moved - keep it still..."));
  mpuRead(f, u, p); angle = accelAngle(f, u);
#if TRIM_AT_CALIBRATION
  if (fabs(calTrim) < 20) {
    g.trim = calTrim;
    Serial.print(F("trim measured at calibration: ")); Serial.println(g.trim, 2);
  } else {
    Serial.print(F("calibration angle ")); Serial.print(calTrim, 1);
    Serial.println(F(" deg is too far from upright - keeping the saved trim"));
  }
#endif
  show();
#if AUTO_START
  // let the filter settle on the real angle for 0.2 s, then go
  for (int i = 0; i < 40; i++) { if (mpuRead(f, u, p)) angle = 0.9 * angle + 0.1 * accelAngle(f, u); delay(5); }
  if (fabs(angle - g.trim) < START_ANGLE) startBalancing(F("auto start"));
  else { state = ARMED; Serial.println(F("Not upright - stand me up near my balance point to start.")); }
#else
  Serial.println(F("Ready. Motors are OFF. Hold it upright on the floor and type g."));
#endif
}

void loop() {
  static uint32_t next = micros(), lastPrint = 0;
  serialPoll();
  if ((int32_t)(micros() - next) < 0) return;
  if ((int32_t)(micros() - next) > (int32_t)LOOP_US) next = micros();
  next += LOOP_US;
  control();
  if (printOn && millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print(F("angle ")); Serial.print(angle, 2); Serial.print(F("  target ")); Serial.print(target, 2);
    Serial.print(F("  out ")); Serial.print(out);
    Serial.print(F("  speed_cm/s ")); Serial.print(speed * 100, 1);
    Serial.print(F("  trim ")); Serial.print(g.trim, 1); Serial.print(F("  state "));
    Serial.println(state == BALANCING ? F("BALANCING") : state == ARMED ? F("ARMED") : F("off"));
  }
}

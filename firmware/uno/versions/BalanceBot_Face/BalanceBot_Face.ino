/*
 * BalanceBot - ONE sketch: setup wizard + self-balancing + OLED face (Uno + MPU-6050 + DRV8833 + SSD1306)
 * Serial Monitor: 115200 baud, line ending "Newline".
 *
 * FIRST TIME: upload, open the Serial Monitor, follow the 6 prompts (Enter / y / n).
 *   It measures every direction + the balance point and saves them. Then it balances.
 * AFTER THAT: hold upright + still, plug in the battery -> it balances by itself.
 *
 * Commands: g start | x stop | t re-measure trim | check redo the wizard | p readout
 *           enc 0 = no encoders (speed guessed from motor power)
 *           enc 1 = encoders (green + yellow wires)
 *           enc 2 = encoders WITHOUT yellow wires (pulses from green, direction from motor power)
 *           etest = wheels in the air: checks both encoders forward + backward
 *           show | save | defaults | kp ki kd trim kv kx cap decay minp <value>
 *
 * Wiring: DRV8833 IN1 D5, IN2 D6, IN3 D9, IN4 D10, EEP D8 (+10k D8 to GND), VCC battery +, GND
 *         LEFT  motor: red OUT1, white OUT2, black 5V, blue GND, green D2, yellow D4
 *         RIGHT motor: red OUT3, white OUT4, black 5V, blue GND, green D3, yellow D7
 *         MPU-6050: VCC 3.3V, GND, SDA A4, SCL A5
 *         OLED 0.96" 128x64 (4-pin I2C): VCC 3.3V, GND, SDA A4, SCL A5  (shares A4/A5 with the MPU)
 *         Battery + -> DRV VCC and Uno VIN; battery - -> GND
 */
#include <Wire.h>
#include <EEPROM.h>

#define IMU_ACCEL_NO_Z  1       // your MPU's Z accelerometer is dead - keep 1

// direction settings - measured by the setup wizard and saved in EEPROM
struct Dirs { uint8_t mL = 0, mR = 1, eL = 0, eR = 1, imuF = 1, gInv = 0; };
Dirs dz;

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
volatile unsigned long pulL = 0, pulR = 0;      // plain pulse counts (green wire only)
void isrL() { pulL++; if (PIND & _BV(4)) encL--; else encL++; }
void isrR() { pulR++; if (PIND & _BV(7)) encR--; else encR++; }
unsigned long pulsesL() { noInterrupts(); unsigned long c = pulL; interrupts(); return c; }
unsigned long pulsesR() { noInterrupts(); unsigned long c = pulR; interrupts(); return c; }
long cntL() { noInterrupts(); long c = encL; interrupts(); return dz.eL ? -c : c; }
long cntR() { noInterrupts(); long c = encR; interrupts(); return dz.eR ? -c : c; }

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
void motors(int p) { driveOne(AIN1, AIN2, dz.mL, p); driveOne(BIN1, BIN2, dz.mR, p); }
void motorsOff() { analogWrite(AIN1, 0); analogWrite(AIN2, 0); analogWrite(BIN1, 0); analogWrite(BIN2, 0); }
void driverSleep(bool sleep) { digitalWrite(DRV_EEP, sleep ? LOW : HIGH); }

// ===================== MPU-6050 =============================================
const uint8_t MPU = 0x68;
float gyroOff = 0;
void mpuWrite(uint8_t r, uint8_t v) { Wire.beginTransmission(MPU); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
// reads forward-axis accel (g), up accel (g) and RAW pitch gyro (+ = tipping forward)
int16_t r[7];
bool mpuRaw() {
  Wire.beginTransmission(MPU); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, (uint8_t)14) != 14) return false;
  for (uint8_t i = 0; i < 7; i++) { uint8_t h = Wire.read(), l = Wire.read(); r[i] = (int16_t)((h << 8) | l); }
  return true;
}
bool mpuRead(float& fwd, float& up, float& pitchRaw) {
  if (!mpuRaw()) return false;
  int8_t s = (dz.imuF == 0 || dz.imuF == 2) ? 1 : -1;
  if (dz.imuF < 2) { fwd = s * r[0] / 8192.0; pitchRaw =  s * r[5]; }   // X forward -> pitch about Y
  else             { fwd = s * r[1] / 8192.0; pitchRaw = -s * r[4]; }   // Y forward -> pitch about X
  up = r[2] / 8192.0;
  if (dz.gInv) pitchRaw = -pitchRaw;
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
unsigned long lastPL = 0, lastPR = 0;
long win[5]; uint8_t wi = 0, wn = 0;
uint32_t uprightSince = 0;
bool printOn = false;
int out = 0;
uint32_t fellAt = 0;    // when it last fell (for the face)
uint8_t encOn = 1;      // 0 = motor-power guess, 1 = encoders, 2 = green-wire pulses + motor direction
float vm = 0;           // motor-power speed guess (m/s)
const float VMAX = 0.6; // wheel speed at full power (m/s)

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
  lastL = cntL(); lastR = cntR(); lastPL = pulsesL(); lastPR = pulsesR(); vm = 0;
  Serial.print(F("BALANCING (")); Serial.print(how); Serial.println(')');
}

// ===================== the control loop (every 5 ms) =========================
void control() {
  float f, u, p;
  if (!mpuRead(f, u, p)) { motorsOff(); return; }
  rate = (p - gyroOff) / 65.5;                                   // deg/s
  angle = 0.996 * (angle + rate * DT) + 0.004 * accelAngle(f, u);  // trust the gyro more

  // wheel speed + position from the encoders
  long cL = cntL(), cR = cntR();
  long d = (cL - lastL) + (cR - lastR); lastL = cL; lastR = cR;
  unsigned long pL = pulsesL(), pR = pulsesR();
  long dp = (long)(pL - lastPL) + (long)(pR - lastPR); lastPL = pL; lastPR = pR;
  if (encOn == 2) d = (out >= 0) ? dp : -dp;            // direction = the way the motors are pushed
  win[wi] = d; wi = (wi + 1) % 5; if (wn < 5) wn++;
  long sum = 0; for (uint8_t i = 0; i < wn; i++) sum += win[i];
  float wheelRate = sum * 0.5 / COUNTS_REV * TWO_PI / (wn * DT);          // rad/s vs body
  vm += ((float)out / 255.0 * VMAX - vm) * (DT / 0.1);                   // motors take ~0.1 s to respond
  float vNow = encOn ? WHEEL_R * wheelRate : vm;
  speed += 0.3 * (vNow + WHEEL_R * rate * DEG_TO_RAD - speed);            // m/s over ground
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
  if (fabs(err) > FALL_ANGLE) { fellAt = millis() | 1; stopBalancing(F("fell over"), AUTO_START); return; }

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


// ===================== setup wizard (measures all directions + trim) =========
void flushIn() { delay(30); while (Serial.available()) Serial.read(); }
void waitEnter(const __FlashStringHelper* msg) {
  Serial.println(msg); Serial.println(F("   ...then press Enter"));
  while (!Serial.available()) {}
  flushIn();
}
bool askYes(const __FlashStringHelper* q) {
  Serial.print(q); Serial.println(F("  (type y or n, Enter)"));
  while (true) {
    while (!Serial.available()) {}
    char c = Serial.read();
    if (c == 'y' || c == 'Y') { flushIn(); return true; }
    if (c == 'n' || c == 'N') { flushIn(); return false; }
  }
}
void rawAvg(float& ax, float& ay, float& gx, float& gy) {
  ax = ay = gx = gy = 0; int n = 0;
  for (int i = 0; i < 200; i++) { if (mpuRaw()) { ax += r[0]; ay += r[1]; gx += r[4]; gy += r[5]; n++; } delay(4); }
  ax /= n * 8192.0; ay /= n * 8192.0; gx /= n; gy /= n;
}
void spin(uint8_t in1, uint8_t in2) { analogWrite(in1, 255); analogWrite(in2, 255 - 120); delay(1000); motorsOff(); }
float measureTrim() {
  float f, u, p, a = 0; int n = 0;
  for (int i = 0; i < 300; i++) { if (mpuRead(f, u, p)) { a += accelAngle(f, u); n++; } delay(5); }
  return n ? a / n : 0;
}
void saveAll();
void wizard() {
  motorsOff(); driverSleep(true);
  Serial.println(F("\n========= SETUP WIZARD ========="));
  float ax0, ay0, gx0, gy0, ax1, ay1, gx1, gy1;
  while (true) {
    waitEnter(F("\n1) Hold the robot UPRIGHT and STILL"));
    rawAvg(ax0, ay0, gx0, gy0);
    waitEnter(F("2) Tilt it FORWARD (towards its front) about 20 deg and HOLD still"));
    rawAvg(ax1, ay1, gx1, gy1);
    if (max(fabs(ax1 - ax0), fabs(ay1 - ay0)) > 0.12) break;
    Serial.println(F("   Hardly any change - tilt it more. Again:"));
  }
  float dx = ax1 - ax0, dy = ay1 - ay0;
  if (fabs(dx) > fabs(dy)) dz.imuF = dx < 0 ? 0 : 1; else dz.imuF = dy < 0 ? 2 : 3;
  dz.gInv = 0;

  float integ = 0;
  while (true) {
    waitEnter(F("3) Hold it UPRIGHT. After Enter you have 3 s: tip it FORWARD about 20 deg and hold"));
    integ = 0; uint32_t t0 = millis(), last = micros();
    while (millis() - t0 < 3000) {
      if (!mpuRaw()) continue;
      float dt = (micros() - last) / 1e6; last = micros();
      int8_t s = (dz.imuF == 0 || dz.imuF == 2) ? 1 : -1;
      float pr = (dz.imuF < 2) ? s * (r[5] - gy0) : -s * (r[4] - gx0);
      integ += pr / 65.5 * dt;
      delay(3);
    }
    Serial.print(F("   gyro saw ")); Serial.print(integ, 1); Serial.println(F(" deg"));
    if (fabs(integ) > 5) break;
    Serial.println(F("   Too small - tip it more firmly. Again:"));
  }
  dz.gInv = integ < 0;

  waitEnter(F("\n4) Hold the robot with BOTH WHEELS IN THE AIR. The LEFT wheel spins for 1 s"));
  driverSleep(false);
  long l0 = encL; spin(AIN1, AIN2); long dL = encL - l0;
  bool lFwd = askYes(F("   Did the LEFT wheel's TOP move towards the robot's FRONT?"));
  waitEnter(F("5) Wheels still in the air. The RIGHT wheel spins for 1 s"));
  long r0 = encR; spin(BIN1, BIN2); long dR = encR - r0;
  bool rFwd = askYes(F("   Did the RIGHT wheel's TOP move towards the robot's FRONT?"));
  driverSleep(true);
  dz.mL = !lFwd; dz.mR = !rFwd;
  dz.eL = (dL > 0) != lFwd; dz.eR = (dR > 0) != rFwd;
  Serial.print(F("   encoder counts: left ")); Serial.print(dL); Serial.print(F(", right ")); Serial.println(dR);
  if (labs(dL) < 30) Serial.println(F("   !! LEFT encoder hardly counts - check green D2 / yellow D4 / black 5V / blue GND"));
  if (labs(dR) < 30) Serial.println(F("   !! RIGHT encoder hardly counts - check green D3 / yellow D7 / black 5V / blue GND"));

  waitEnter(F("\n6) Hold the robot at its BALANCE POINT (it would fall neither way), STILL"));
  g.trim = measureTrim();
  Serial.print(F("   trim = ")); Serial.println(g.trim, 2);

  saveAll();
  Serial.println(F("\nAll saved. Now: hold it upright on the floor, STILL..."));
  delay(1500);
}

// ===================== OLED face (SSD1306 128x64 on I2C) =====================
// No screen buffer (the Uno has too little RAM): every byte is worked out on the fly
// and sent in small 16-byte pieces between control cycles, so balancing is never delayed.
#define OLED_ENABLED 1
uint8_t oledAddr = 0;                                   // 0 = no display found -> face off
const uint8_t FX0 = 16, FX1 = 111, FP0 = 1, FP1 = 7;    // face area: columns 16..111, pages 1..7
enum Face : uint8_t { F_SLEEP, F_WAIT, F_HAPPY, F_FELL, F_SETUP };
struct FaceState { uint8_t face; int8_t eyeDy; uint8_t blink; uint8_t mouth; } fs;
int8_t faceForce = F_SETUP;                             // >= 0: show this face whatever the state
uint8_t oPage = FP0, oCol = FX0; bool oNewFrame = true;
const uint8_t EYE_LX = 40, EYE_RX = 88, EYE_Y = 24, MOUTH_X = 64;

void oledCmd(const uint8_t* c, uint8_t n) {
  Wire.beginTransmission(oledAddr); Wire.write(0x00);
  for (uint8_t i = 0; i < n; i++) Wire.write(c[i]);
  Wire.endTransmission();
}
void oledWindow(uint8_t c0, uint8_t c1, uint8_t p0, uint8_t p1) { uint8_t c[] = {0x21, c0, c1, 0x22, p0, p1}; oledCmd(c, 6); }
void oledInit() {
#if OLED_ENABLED
  for (uint8_t a = 0x3C; a <= 0x3D; a++) { Wire.beginTransmission(a); if (Wire.endTransmission() == 0) { oledAddr = a; break; } }
  if (!oledAddr) { Serial.println(F("(no OLED found - face off)")); return; }
  const uint8_t init1[] = {0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x00};
  const uint8_t init2[] = {0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0x2E, 0xAF};
  oledCmd(init1, sizeof(init1)); oledCmd(init2, sizeof(init2));
  oledWindow(0, 127, 0, 7);                              // clear the whole screen
  for (uint8_t k = 0; k < 64; k++) { Wire.beginTransmission(oledAddr); Wire.write(0x40); for (uint8_t i = 0; i < 16; i++) Wire.write(0); Wire.endTransmission(); }
#endif
}
// pixels a..b (inclusive) of one column, as the byte for page p
uint8_t span(int a, int b, uint8_t p) {
  int y0 = p * 8, y1 = y0 + 7;
  if (b < y0 || a > y1 || b < a) return 0;
  if (a < y0) a = y0;
  if (b > y1) b = y1;
  return (uint8_t)((0xFF >> (7 - (b - a))) << (a - y0));
}
uint8_t isq(int v) { uint8_t r = 0; while ((int)(r + 1) * (r + 1) <= v) r++; return r; }
uint8_t faceByte(uint8_t x, uint8_t p) {
  uint8_t b = 0;
  for (uint8_t e = 0; e < 2; e++) {                      // ---- eyes ----
    int dx = (int)x - (e ? EYE_RX : EYE_LX), adx = abs(dx), cy = EYE_Y + fs.eyeDy;
    if (fs.face == F_HAPPY) {                            // ^ ^ happy arcs
      if (adx <= 12) { int o = isq(156 - dx * dx), i = adx < 8 ? isq(72 - dx * dx) : 0; b |= span(cy + 4 - o, cy + 4 - i, p); }
    } else if (fs.face == F_FELL) {                      // x x
      if (adx <= 8) b |= span(cy + dx - 1, cy + dx + 1, p) | span(cy - dx - 1, cy - dx + 1, p);
    } else if (fs.face == F_SLEEP) {                     // closed - -
      if (adx <= 11) b |= span(cy + 1, cy + 3, p);
    } else {                                             // open rounded eyes, blinking
      if (adx <= 11) {
        int h = fs.blink ? 2 : 11, cut = 0;
        if (adx > 6) { int k = adx - 6; cut = 5 - isq(30 - k * k); }
        int hh = h - cut; if (hh < 1) hh = 1;
        b |= span(cy - hh, cy + hh, p);
      }
    }
  }
  int dx = (int)x - MOUTH_X, adx = abs(dx);              // ---- mouth ----
  if (fs.face == F_HAPPY) {
    if (adx <= 12) {
      if (fs.mouth > 2) { int o = (int)isq(156 - dx * dx) * fs.mouth / 12; b |= span(42, 42 + o, p); }   // open D mouth
      else { int o = isq(156 - dx * dx), i = adx < 9 ? isq(90 - dx * dx) : 0; b |= span(40 + i, 40 + o, p); } // smile
    }
  } else if (fs.face == F_FELL) {                        // frown
    if (adx <= 12) { int o = isq(156 - dx * dx), i = adx < 9 ? isq(90 - dx * dx) : 0; b |= span(58 - o, 58 - i, p); }
  } else if (fs.face == F_WAIT) {                        // small o
    if (adx <= 5) { int o = isq(30 - dx * dx), i = adx < 3 ? isq(12 - dx * dx) : 0; b |= span(48 - o, 48 - i, p) | span(48 + i, 48 + o, p); }
  } else if (fs.face == F_SLEEP) { if (adx <= 5) b |= span(48, 49, p); }
  else { if (adx <= 8) b |= span(47, 48, p); }           // setup: straight mouth
  return b;
}
void latchFace() {                                       // pick the face once per frame
  uint32_t now = millis();
  uint8_t f;
  if (faceForce >= 0) f = faceForce;
  else if (state == BALANCING) f = F_HAPPY;
  else if (state == ARMED) f = (fellAt && now - fellAt < 3000) ? F_FELL : F_WAIT;
  else f = F_SLEEP;
  fs.face = f;
  fs.blink = (now % 3500) < 160;
  float e = angle - g.trim;                              // lean forward -> eyes look down
  fs.eyeDy = (f == F_HAPPY || f == F_WAIT) ? (int8_t)constrain(e * 0.8, -4.0, 4.0) : 0;
  uint8_t m = (uint8_t)constrain(fabs(speed) * 60.0, 0.0, 12.0);   // moving fast -> open mouth
  uint32_t t = now % 7000;                               // a little "chatter" every 7 s
  if (t < 900) { uint8_t c = ((t / 150) & 1) ? 9 : 4; if (c > m) m = c; }
  fs.mouth = m;
}
bool oledChunk() {                                       // send 16 bytes; true when a frame finished
  if (oNewFrame) { latchFace(); oledWindow(FX0, FX1, FP0, FP1); oPage = FP0; oCol = FX0; oNewFrame = false; }
  Wire.beginTransmission(oledAddr); Wire.write(0x40);
  for (uint8_t i = 0; i < 16; i++) {
    Wire.write(faceByte(oCol, oPage));
    if (++oCol > FX1) { oCol = FX0; if (++oPage > FP1) { oNewFrame = true; break; } }
  }
  Wire.endTransmission();
  return oNewFrame;
}
void oledTask() { if (oledAddr) for (uint8_t n = 0; n < 2; n++) if (oledChunk()) break; }   // ~0.9 ms
void oledShowNow(int8_t f) {                             // draw a whole face right now (setup only)
  if (!oledAddr) return;
  faceForce = f; oNewFrame = true;
  while (!oledChunk()) {}
}

// ===================== encoder test (wheels in the air) =======================
void encTestOne(const __FlashStringHelper* name, uint8_t in1, uint8_t in2, bool inv, bool left) {
  long a0 = left ? cntL() : cntR();
  driveOne(in1, in2, inv, 120); delay(700); motorsOff(); delay(300);
  long a1 = left ? cntL() : cntR();
  driveOne(in1, in2, inv, -120); delay(700); motorsOff(); delay(300);
  long a2 = left ? cntL() : cntR();
  long fw = a1 - a0, bw = a2 - a1;
  Serial.print(name); Serial.print(F(": forward ")); Serial.print(fw); Serial.print(F(", backward ")); Serial.print(bw);
  if (labs(fw) < 30 || labs(bw) < 30) Serial.println(F("  -> BAD: hardly counts (green wire / 5V / GND)"));
  else if (fw > 0 && bw < 0)          Serial.println(F("  -> OK"));
  else if ((fw > 0) == (bw > 0))      Serial.println(F("  -> BAD: same sign both ways = YELLOW wire not read"));
  else                                Serial.println(F("  -> reversed: type check to redo the setup"));
}
void encoderTest() {
  if (state == BALANCING) stopBalancing(F("encoder test"), false);
  Serial.println(F("Hold the robot with BOTH WHEELS IN THE AIR - starting in 2 s"));
  delay(2000); driverSleep(false);
  encTestOne(F("LEFT "), AIN1, AIN2, dz.mL, true);
  encTestOne(F("RIGHT"), BIN1, BIN2, dz.mR, false);
  driverSleep(true);
  Serial.println(F("Both OK -> enc 1.  Any 'YELLOW wire' BAD -> enc 2.  'hardly counts' -> enc 0."));
}

// ===================== Serial commands =======================================
struct Ref { const char* n; float* v; };
Ref refs[] = {{"kp", &g.kp}, {"ki", &g.ki}, {"kd", &g.kd}, {"trim", &g.trim}, {"kv", &g.kv}, {"kx", &g.kx},
              {"cap", &g.cap}, {"decay", &g.decay}, {"minp", &g.minp}};
const uint16_t MAGIC = 0xBA20;           // new number: old saved values are ignored once
void saveAll() { EEPROM.put(0, MAGIC); EEPROM.put(2, g); EEPROM.put(2 + sizeof(Gains), dz); Serial.println(F("saved")); }
void calibrateAndStart();

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
    g.trim = measureTrim();
    Serial.print(F("trim = ")); Serial.print(g.trim, 2); Serial.println(F("   (type save to keep)"));
  }
  else if (!strcmp(c, "enc") && a) {
    encOn = constrain(atoi(a), 0, 2); EEPROM.update(100, encOn);
    if (encOn == 0) Serial.println(F("speed from MOTOR POWER - encoders ignored (saved)"));
    if (encOn == 1) Serial.println(F("speed from ENCODERS (saved)"));
    if (encOn == 2) Serial.println(F("speed from GREEN-wire pulses + motor direction (saved)"));
  }
  else if (!strcmp(c, "etest")) encoderTest();
  else if (!strcmp(c, "show")) { show(); Serial.print(F("enc=")); Serial.println(encOn); }
  else if (!strcmp(c, "save")) saveAll();
  else if (!strcmp(c, "check")) { oledShowNow(F_SETUP); wizard(); calibrateAndStart(); }
  else if (!strcmp(c, "defaults")) { g = Gains(); Serial.println(F("defaults loaded (type save to keep)")); show(); }
  else {
    for (Ref& r : refs) if (!strcmp(c, r.n) && a) { *r.v = atof(a); Serial.print(c); Serial.print(" = "); Serial.println(*r.v, 3); return; }
    Serial.println(F("commands: g x t check enc etest p show save defaults | kp ki kd trim kv kx cap decay minp <value>"));
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
  bool haveSaved = (m == MAGIC);
  if (haveSaved) { EEPROM.get(2, g); EEPROM.get(2 + sizeof(Gains), dz); Serial.println(F("\nsaved settings loaded")); }
  { uint8_t e = EEPROM.read(100); if (e <= 2) encOn = e; }

  Wire.begin(); digitalWrite(SDA, LOW); digitalWrite(SCL, LOW); Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  mpuWrite(0x6B, 0x80); delay(100);
  mpuWrite(0x6B, 0x01); mpuWrite(0x1A, 0x03); mpuWrite(0x1B, 0x08); mpuWrite(0x1C, 0x08);
  float f, u, p;
  if (!mpuRead(f, u, p)) { Serial.println(F("MPU-6050 NOT FOUND - check A4/A5, 3.3V, GND")); while (true) delay(1000); }
  oledInit();
  oledShowNow(F_SETUP);

  if (!haveSaved) wizard();                 // first time: measure everything
  else Serial.println(F("(type check to redo the setup wizard)"));
  calibrateAndStart();
}

void calibrateAndStart() {
  float f, u, p;
  motorsOff(); driverSleep(true); state = DISARMED;
  oledShowNow(F_SETUP);
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
  faceForce = -1; oNewFrame = true;                      // face now follows the robot's state
}

void loop() {
  static uint32_t next = micros(), lastPrint = 0;
  serialPoll();
  if ((int32_t)(micros() - next) < 0) return;
  if ((int32_t)(micros() - next) > (int32_t)LOOP_US) next = micros();
  next += LOOP_US;
  control();
  if ((int32_t)(micros() - (next - LOOP_US)) < 3000) oledTask();   // only if there's time left this cycle
  if (printOn && millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print(F("angle ")); Serial.print(angle, 2); Serial.print(F("  target ")); Serial.print(target, 2);
    Serial.print(F("  out ")); Serial.print(out);
    Serial.print(F("  speed_cm/s ")); Serial.print(speed * 100, 1);
    Serial.print(F("  encL ")); Serial.print(cntL()); Serial.print(F("  encR ")); Serial.print(cntR());
    Serial.print(F("  trim ")); Serial.print(g.trim, 1); Serial.print(F("  state "));
    Serial.println(state == BALANCING ? F("BALANCING") : state == ARMED ? F("ARMED") : F("off"));
  }
}

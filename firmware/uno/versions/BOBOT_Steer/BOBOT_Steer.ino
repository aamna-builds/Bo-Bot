/*
 * BO-BOT - ONE sketch: setup wizard + self-balancing + 16-face OLED (Uno + MPU-6050 + DRV8833 + SH1106/SSD1306)
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
 *           oled = look for the OLED now (it also looks by itself every 0.5 s - even while balancing -
 *                  notices if it gets unplugged, and re-wakes the screen every 3 s after power glitches)
 *           faces = show all 16 faces one by one | face <0-15> = show one face for 5 s
 *           show | save | defaults | kp ki kd trim kv kx cap decay minp <value>
 *           ks <value>   = keep-straight strength (encoders hold its heading; 0 = off)
 *           mbal <value> = motor balance in %: turns RIGHT by itself -> negative, turns LEFT -> positive
 * BUTTON:   press it -> motors stop, let go, hold the robot upright + still -> it recalibrates
 *           (about 1 s after you let go) and starts balancing again by itself.
 *
 * Wiring: DRV8833 IN1 D5, IN2 D6, IN3 D9, IN4 D10, EEP D8 (+10k D8 to GND), VCC battery +, GND
 *         LEFT  motor: red OUT1, white OUT2, black 5V, blue GND, green D2, yellow D4
 *         RIGHT motor: red OUT3, white OUT4, black 5V, blue GND, green D3, yellow D7
 *         MPU-6050: VCC 3.3V, GND, SDA A4, SCL A5
 *         OLED 128x64 SH1106 or SSD1306 (4-pin I2C): VCC 3.3V, GND, SDA, SCL (the SDA/SCL pins next to AREF = A4/A5)
 *         Button (4-pin): one leg -> D12, the DIAGONALLY opposite leg -> GND (no resistor needed)
 *         Battery + -> DRV VCC and Uno VIN; battery - -> GND
 */
#include <Wire.h>
#include <EEPROM.h>

#define IMU_ACCEL_NO_Z  1       // your MPU's Z accelerometer is dead - keep 1

// direction settings - measured by the setup wizard and saved in EEPROM
struct Dirs { uint8_t mL = 0, mR = 1, eL = 0, eR = 1, imuF = 1, gInv = 0; };
Dirs dz;

// ===================== PID VALUES (defaults) ================================
struct Gains {
  float kp   = 28.0;   // power per degree of tilt         (main push back)
  float ki   = 0.0;    // keep 0
  float kd   = 0.6;   // power per deg/s of tilt speed    (damping; too high = buzz)
  float trim = 0.0;    // balance point in degrees - measure with the t command
  float kv   = 30.0;   // deg of lean per m/s   (stops it running away - keep on)
  float kx   = 3.0;    // deg of lean per metre (pulls it back to where it started)
  float cap  = 230;    // max motor power 0..255
  float decay = 0;     // 0 = slow-decay drive (stronger at low power)
  float minp = 0;      // extra power for tiny corrections (0 = off, avoids trembling)
};
Gains g;
const float FALL_ANGLE  = 40.0;  // motors off beyond this
#define AUTO_START      1        // 1 = calibrate + balance as soon as powered
#define TRIM_AT_CALIBRATION 0    // 1 = the angle you hold it at during calibration becomes the trim
const float START_ANGLE = 23.0;  // at power-up: start at once if within this of trim
const float ARM_ANGLE   = 8.0;   // after a fall: restart when stood up within this...
const uint16_t ARM_MS   = 300;   // ...for this long
const float MAX_LEAN    = 10.0;  // most the speed loop may lean the robot

// ===================== pins / constants =====================================
const uint8_t AIN1 = 5, AIN2 = 6, BIN1 = 9, BIN2 = 10;
const uint8_t DRV_EEP = 8;     // DRV8833 EEP/nSLEEP: LOW = driver asleep, HIGH = awake
const uint8_t ENC_L_A = 2, ENC_R_A = 3;          // B channels on D4, D7
const uint8_t BTN = 12;                           // recalibrate button: D12 -> button -> GND
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
void motorsLR(int l, int r) { driveOne(AIN1, AIN2, dz.mL, l); driveOne(BIN1, BIN2, dz.mR, r); }
void motorsOff() { analogWrite(AIN1, 0); analogWrite(AIN2, 0); analogWrite(BIN1, 0); analogWrite(BIN2, 0); }
void driverSleep(bool sleep) { digitalWrite(DRV_EEP, sleep ? LOW : HIGH); }

// ===================== MPU-6050 =============================================
const uint8_t MPU = 0x68;
float gyroOff = 0;
void mpuWrite(uint8_t r, uint8_t v) { Wire.beginTransmission(MPU); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
int16_t r[7];
bool mpuRaw() {
  Wire.beginTransmission(MPU); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, (uint8_t)14) != 14) return false;
  for (uint8_t i = 0; i < 7; i++) { uint8_t h = Wire.read(), l = Wire.read(); r[i] = (int16_t)((h << 8) | l); }
  return true;
}
// forward-axis accel (g), up accel (g) and RAW pitch gyro (+ = tipping forward)
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
// average the gyro (offset) while held still; false if it moved
float calTrim = 0;
bool calibrate() {                                       // takes exactly 1 second
  float f, u, p, s = 0, q = 0, a = 0; int n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 1000) { if (mpuRead(f, u, p)) { s += p; q += p * p; a += accelAngle(f, u); n++; } delay(3); }
  if (n < 150) return false;
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
float peakRate = 0, peakErr = 0;            // for the face: biggest tilt speed / tilt since last frame
uint32_t balSince = 0, armedSince = 0;      // for the face
void noteFall();
uint8_t encOn = 1;      // 0 = motor-power guess, 1 = encoders, 2 = green-wire pulses + motor direction
float vm = 0;           // motor-power speed guess (m/s)
float ks = 1.5;         // keep-straight: power per encoder count of left/right difference (0 = off)
float mbal = 0;         // motor balance %: + = left motor stronger, - = right motor stronger
long headDiff = 0;      // left wheel travel minus right wheel travel since balancing started (counts)
const float VMAX = 0.6; // wheel speed at full power (m/s)

void stopBalancing(const __FlashStringHelper* why, bool autoRestart) {
  motorsOff(); driverSleep(true); out = 0; uprightSince = 0;
  state = autoRestart ? ARMED : DISARMED;
  armedSince = millis();
  Serial.print(F("STOPPED: ")); Serial.println(why);
  Serial.println(autoRestart ? F("Stand me up near my balance point to restart.")
                             : F("Type g to start again."));
}
void startBalancing(const __FlashStringHelper* how) {
  motorsOff(); driverSleep(false);
  state = BALANCING; iTerm = 0; speed = 0; position = 0; wn = 0; uprightSince = 0;
  lastL = cntL(); lastR = cntR(); lastPL = pulsesL(); lastPR = pulsesR(); vm = 0; headDiff = 0;
  balSince = millis();
  Serial.print(F("BALANCING (")); Serial.print(how); Serial.println(')');
}

// ===================== the control loop (every 5 ms) =========================
void control() {
  float f, u, p;
  if (!mpuRead(f, u, p)) { motorsOff(); return; }
  rate = (p - gyroOff) / 65.5;                                     // deg/s
  angle = 0.996 * (angle + rate * DT) + 0.004 * accelAngle(f, u);  // trust the gyro more

  // wheel speed + position
  long cL = cntL(), cR = cntR();
  long dLw = cL - lastL, dRw = cR - lastR; lastL = cL; lastR = cR;
  unsigned long pL = pulsesL(), pR = pulsesR();
  long dpL = (long)(pL - lastPL), dpR = (long)(pR - lastPR); lastPL = pL; lastPR = pR;
  if (encOn == 2) { int8_t sg = (out >= 0) ? 1 : -1; dLw = sg * dpL; dRw = sg * dpR; }   // direction = the way the motors are pushed
  long d = dLw + dRw;
  headDiff += dLw - dRw;                                // >0 = left wheel went further = turned right
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
  if (fabs(err) > FALL_ANGLE) { fellAt = millis() | 1; noteFall(); stopBalancing(F("fell over"), AUTO_START); return; }
  if (fabs(rate) > peakRate) peakRate = fabs(rate);       // for the face
  if (fabs(err) > peakErr) peakErr = fabs(err);

  // outer loop: lean a little to stop rolling away and return to the start
  target = constrain(-(g.kv * speed + g.kx * position), -MAX_LEAN, MAX_LEAN);

  // inner loop: PID on the tilt angle
  float e = err - target;
  iTerm = constrain(iTerm + g.ki * e * DT, -100, 100);
  int cap = (int)constrain(g.cap, 0, 255);
  float o = g.kp * e + iTerm + g.kd * rate;
  if (fabs(o) > 2) o += (o > 0 ? g.minp : -g.minp);       // overcome motor/gear stiction
  out = (int)constrain(o, -cap, cap);

  // keep straight: same push on both wheels, plus a small left/right difference
  float steer = encOn ? constrain(ks * headDiff, -60.0, 60.0) : 0;   // encoders hold the heading
  int l = (int)constrain(out * (1 + mbal / 100.0) - steer, -255.0, 255.0);
  int r = (int)constrain(out * (1 - mbal / 100.0) + steer, -255.0, 255.0);
  motorsLR(l, r);
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

// ===================== OLED face (SH1106 / SSD1306 128x64 on I2C) =============
// No screen buffer (the Uno has too little RAM): every byte is worked out on the fly
// and sent in small 16-byte pieces between control cycles, so balancing is never delayed.
//
// 16 FACES and when they show:
//   0 sleep      stopped with x                    8 love       idle emote (heart eyes)
//   1 wait       waiting to be stood up            9 surprised  pushed / bumped hard
//   2 sleepy     waiting more than 30 s           10 dizzy      big wobble (>15 deg) but saved it
//   3 setup      calibrating / setup wizard       11 angry      pushed 4 times within 10 s
//   4 normal     balancing calmly (blinks, looks) 12 confused   drifted more than 40 cm from start
//   5 happy      idle emote                       13 cool       balancing for over 60 s (sunglasses)
//   6 excited    moving fast                      14 dead       just fell over
//   7 wink       idle emote                       15 sad        fell 3 times within 30 s
// Commands: faces = show all 16 one by one | face <0-15> = show one for 5 s
#define OLED_ENABLED 1
#define OLED_COL_OFFSET 2   // 2 = SH1106 screens (yours), 0 = SSD1306 (with 2 the face just shifts 2 px there)
uint8_t oledAddr = 0;                                   // 0 = no display found -> face off
const uint8_t FX0 = 16, FX1 = 111, FP0 = 0, FP1 = 7;    // face area: columns 16..111, all 8 pages
uint8_t oPage = FP0, oCol = FX0; bool oNewFrame = true;

#define OLED_RETRY_MS 500  // how often to look for a missing OLED (ms)
uint8_t oClearLeft = 0, oClearPage = 0, oClearCol = 0;  // screen wipe, done a piece at a time

void oledLost() { if (oledAddr) Serial.println(F("OLED lost - looking for it again")); oledAddr = 0; }
bool oledCmd(const uint8_t* c, uint8_t n) {
  Wire.beginTransmission(oledAddr); Wire.write(0x00);
  for (uint8_t i = 0; i < n; i++) Wire.write(c[i]);
  if (Wire.endTransmission() != 0) { oledLost(); return false; }
  return true;
}
// page + column address: works on BOTH SSD1306 and SH1106 screens
bool oledSetPos(uint8_t page, uint8_t col) { uint8_t c[] = {(uint8_t)(0xB0 | page), (uint8_t)(col & 0x0F), (uint8_t)(0x10 | (col >> 4))}; return oledCmd(c, 3); }
void oledSendInit() {                                    // ~0.7 ms; also re-wakes a screen that reset itself
  const uint8_t init1[] = {0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x02};
  const uint8_t init2[] = {0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0x2E, 0xAF};
  if (oledCmd(init1, sizeof(init1))) oledCmd(init2, sizeof(init2));
}
bool oledProbe() {                                       // ~0.1 ms: is a screen answering?
  for (uint8_t a = 0x3C; a <= 0x3D; a++) { Wire.beginTransmission(a); if (Wire.endTransmission() == 0) { oledAddr = a; return true; } }
  return false;
}
void oledStart(bool quiet, uint8_t tries) {              // look for the OLED; if found, wake it + wipe it
#if OLED_ENABLED
  oledAddr = 0;
  for (uint8_t t = 0; t < tries; t++) { if (oledProbe()) break; if (t + 1 < tries) delay(20); }
  if (!oledAddr) { if (!quiet) Serial.println(F("(no OLED found - face off, will keep looking)")); return; }
  Serial.print(F("OLED found at 0x")); Serial.println(oledAddr, HEX);
  oledSendInit();
  oClearLeft = 8 * 9; oClearPage = 0; oClearCol = 0;     // wipe: 8 pages x 9 pieces, sent between control cycles
  oNewFrame = true;
#endif
}

// FACE-DRAW-BEGIN
enum Face : uint8_t { F_SLEEP, F_WAIT, F_SLEEPY, F_SETUP, F_NORMAL, F_HAPPY, F_EXCITED, F_WINK, F_LOVE,
                      F_SURPRISED, F_DIZZY, F_ANGRY, F_CONFUSED, F_COOL, F_DEAD, F_SAD, F_COUNT };
enum Eye : uint8_t { E_OPEN, E_HAPPY, E_X, E_CLOSED, E_HALF, E_HEART, E_BIG, E_SPIRAL, E_SMALL, E_SHADES };
enum Mouth : uint8_t { M_SMILE, M_OPEN, M_FROWN, M_O, M_BIGO, M_FLAT, M_WAVY, M_SMIRK, M_SMALLFLAT };
enum Extra : uint8_t { X_NONE, X_ZZZ, X_BROWUP, X_ANGRY, X_CONFUSED, X_SHADES, X_SAD };
const uint8_t FACE_PARTS[F_COUNT][4] PROGMEM = {         // left eye, right eye, mouth, extra
  {E_CLOSED, E_CLOSED, M_SMALLFLAT, X_ZZZ},     // sleep
  {E_OPEN,   E_OPEN,   M_O,         X_NONE},    // wait
  {E_HALF,   E_HALF,   M_SMALLFLAT, X_NONE},    // sleepy
  {E_OPEN,   E_OPEN,   M_FLAT,      X_NONE},    // setup
  {E_OPEN,   E_OPEN,   M_SMILE,     X_NONE},    // normal
  {E_HAPPY,  E_HAPPY,  M_SMILE,     X_NONE},    // happy
  {E_HAPPY,  E_HAPPY,  M_OPEN,      X_NONE},    // excited
  {E_OPEN,   E_HAPPY,  M_SMIRK,     X_NONE},    // wink
  {E_HEART,  E_HEART,  M_SMILE,     X_NONE},    // love
  {E_BIG,    E_BIG,    M_BIGO,      X_BROWUP},  // surprised
  {E_SPIRAL, E_SPIRAL, M_WAVY,      X_NONE},    // dizzy
  {E_HALF,   E_HALF,   M_FROWN,     X_ANGRY},   // angry
  {E_OPEN,   E_SMALL,  M_WAVY,      X_CONFUSED},// confused
  {E_SHADES, E_SHADES, M_SMIRK,     X_SHADES},  // cool
  {E_X,      E_X,      M_FROWN,     X_NONE},    // dead
  {E_SMALL,  E_SMALL,  M_FROWN,     X_SAD},     // sad
};
const char FACE_NAMES[] PROGMEM = "sleep\0wait\0sleepy\0setup\0normal\0happy\0excited\0wink\0love\0"
                                  "surprised\0dizzy\0angry\0confused\0cool\0dead\0sad\0";
struct FaceState { uint8_t face; int8_t eyeDy, eyeDx; uint8_t blink, mouth, anim; } fs;
const uint8_t EYE_LX = 40, EYE_RX = 88, EYE_Y = 26, MOUTH_X = 64;
const uint8_t SQ[161] PROGMEM = {0, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12};
uint8_t isq(int v) { if (v <= 0) return 0; if (v > 160) v = 160; return pgm_read_byte(&SQ[v]); }
// pixels a..b (inclusive) of one column, as the byte for page p
uint8_t span(int a, int b, uint8_t p) {
  int y0 = p * 8, y1 = y0 + 7;
  if (b < y0 || a > y1 || b < a) return 0;
  if (a < y0) a = y0;
  if (b > y1) b = y1;
  return (uint8_t)((0xFF >> (7 - (b - a))) << (a - y0));
}
uint8_t disc(int dx, int cy, int R, uint8_t p) { if (abs(dx) > R) return 0; int h = isq(R * R + R - dx * dx); return span(cy - h, cy + h, p); }
uint8_t ring(int dx, int cy, int R, int t, uint8_t p) {
  int adx = abs(dx), ri = R - t; if (adx > R) return 0;
  int o = isq(R * R + R - dx * dx);
  if (adx > ri) return span(cy - o, cy + o, p);
  int i = isq(ri * ri + ri - dx * dx);
  return span(cy - o, cy - i - 1, p) | span(cy + i + 1, cy + o, p);
}
uint8_t arcTop(int dx, int cy, int R, int t, uint8_t p) {   // upper half of a ring  (^ shape)
  int adx = abs(dx), ri = R - t; if (adx > R) return 0;
  int o = isq(R * R + R - dx * dx), i = adx < ri ? isq(ri * ri + ri - dx * dx) : 0;
  return span(cy - o, cy - i, p);
}
uint8_t arcBot(int dx, int cy, int R, int t, uint8_t p) {   // lower half of a ring  (U shape)
  int adx = abs(dx), ri = R - t; if (adx > R) return 0;
  int o = isq(R * R + R - dx * dx), i = adx < ri ? isq(ri * ri + ri - dx * dx) : 0;
  return span(cy + i, cy + o, p);
}
uint8_t line(int dx, int x0, int y0, int x1, int y1, int th, uint8_t p) {   // thick line, x0 < x1
  if (dx < x0 || dx > x1) return 0;
  int y = y0 + (int)((long)(y1 - y0) * (dx - x0) / (x1 - x0));
  return span(y, y + th - 1, p);
}
uint8_t eyeCol(uint8_t type, int dx, int cy, uint8_t p) {
  int adx = abs(dx);
  switch (type) {
    case E_OPEN: case E_SMALL: case E_HALF: {
      if (adx > 11) return 0;
      int h = (type == E_SMALL) ? 6 : ((fs.blink && type == E_OPEN) ? 2 : 11), cut = 0;
      if (adx > 6) { int k = adx - 6; cut = 5 - isq(30 - k * k); }
      int hh = h - cut; if (hh < 1) hh = 1;
      int top = cy - hh;
      if (type == E_HALF && top < cy - 1) top = cy - 1;          // heavy eyelids
      return span(top, cy + hh, p);
    }
    case E_HAPPY:  return arcTop(dx, cy + 4, 12, 4, p);
    case E_X:      if (adx > 8) return 0; return span(cy + dx - 1, cy + dx + 1, p) | span(cy - dx - 1, cy - dx + 1, p);
    case E_CLOSED: if (adx > 11) return 0; return span(cy + 1, cy + 3, p);
    case E_HEART: {
      uint8_t b = disc(dx + 6, cy - 4, 6, p) | disc(dx - 6, cy - 4, 6, p);
      if (adx <= 12) b |= span(cy - 4, cy - 4 + (12 - adx) * 14 / 12, p);
      return b;
    }
    case E_BIG:    return ring(dx, cy, 12, 3, p) | disc(dx, cy, 3, p);
    case E_SPIRAL: { int s = (fs.anim & 2) ? 2 : -2; return ring(dx, cy, 12, 2, p) | ring(dx + s, cy, 7, 2, p) | disc(dx - s, cy, 2, p); }
    case E_SHADES: {
      if (adx > 13) return 0;
      int cut = 0; if (adx > 8) { int k = adx - 8; cut = 5 - isq(30 - k * k); }
      return span(cy - 8, cy + 5 - cut, p);
    }
  }
  return 0;
}
uint8_t mouthCol(uint8_t type, int dx, uint8_t p) {
  int adx = abs(dx);
  switch (type) {
    case M_SMILE:     return arcBot(dx, 42, 12, 3, p);
    case M_OPEN:      if (adx > 12) return 0; return span(44, 44 + (int)isq(156 - dx * dx) * fs.mouth / 12, p);
    case M_FROWN:     return arcTop(dx, 58, 12, 3, p);
    case M_O:         return ring(dx, 50, 5, 2, p);
    case M_BIGO:      return ring(dx, 51, 8, 3, p);
    case M_FLAT:      if (adx > 10) return 0; return span(49, 50, p);
    case M_SMALLFLAT: if (adx > 5) return 0; return span(50, 51, p);
    case M_WAVY:      { if (adx > 14) return 0; int ph = (dx + 14 + fs.anim * 2) % 8; int y = 47 + (ph < 4 ? ph : 8 - ph); return span(y, y + 1, p); }
    case M_SMIRK:     { if (dx < -10 || dx > 12) return 0; int y = dx < 4 ? 50 : 50 - (dx - 4) / 2; return span(y, y + 1, p); }
  }
  return 0;
}
uint8_t extraCol(uint8_t type, int x, int cy, int exL, int exR, uint8_t p) {
  int dl = x - exL, dr = x - exR;
  switch (type) {
    case X_ZZZ: {                                         // little z z floating up
      uint8_t b = 0; int up = fs.anim % 4;
      int zx = x - 98, zy = 10 - up;
      if (zx >= 0 && zx <= 6) b |= span(zy, zy, p) | span(zy + 6, zy + 6, p) | span(zy + 6 - zx, zy + 6 - zx, p);
      zx = x - 106; zy = 3 - up / 2;
      if (zx >= 0 && zx <= 4) b |= span(zy, zy, p) | span(zy + 4, zy + 4, p) | span(zy + 4 - zx, zy + 4 - zx, p);
      return b;
    }
    case X_BROWUP:   return arcTop(dl, cy - 14, 9, 2, p) | arcTop(dr, cy - 14, 9, 2, p);
    case X_ANGRY:    return line(dl, -11, cy - 17, 9, cy - 11, 3, p) | line(dr, -9, cy - 11, 11, cy - 17, 3, p);
    case X_SAD:      {
      uint8_t b = line(dl, -11, cy - 10, 9, cy - 16, 2, p) | line(dr, -9, cy - 16, 11, cy - 10, 2, p);
      int ty = cy + 13 + (fs.anim % 5) * 3;               // a falling tear under the left eye
      b |= disc(dl - 7, ty, 2, p); if (dl == 7) b |= span(ty - 4, ty, p);
      return b;
    }
    case X_CONFUSED: return line(dr, -9, cy - 19, 9, cy - 15, 2, p) | line(dl, -9, cy - 16, 9, cy - 16, 2, p);
    case X_SHADES:   return line(x, exL + 13, cy - 7, exR - 13, cy - 7, 2, p) | line(dl, -16, cy - 7, -13, cy - 7, 2, p) | line(dr, 13, cy - 7, 16, cy - 7, 2, p);
  }
  return 0;
}
uint8_t faceByte(uint8_t x, uint8_t p) {
  const uint8_t* fp = FACE_PARTS[fs.face];
  uint8_t eL = pgm_read_byte(fp), eR = pgm_read_byte(fp + 1), mo = pgm_read_byte(fp + 2), ex = pgm_read_byte(fp + 3);
  int exL = EYE_LX + fs.eyeDx, exR = EYE_RX + fs.eyeDx, cy = EYE_Y + fs.eyeDy;
  uint8_t b = eyeCol(eL, (int)x - exL, cy, p) | eyeCol(eR, (int)x - exR, cy, p) | mouthCol(mo, (int)x - MOUTH_X, p);
  if (ex) b |= extraCol(ex, x, cy, exL, exR, p);
  return b;
}
// FACE-DRAW-END

// ---- mood: decides which face to show ----
int8_t faceForce = F_SETUP;                             // >= 0: show this face whatever the state
uint32_t emoteUntil = 0, nextEmote = 0;
uint32_t surpriseUntil = 0, dizzyUntil = 0, angryUntil = 0, pushBlock = 0, sadUntil = 0;
uint32_t pushT[4] = {0, 0, 0, 0}, fallT[3] = {0, 0, 0}; uint8_t pushI = 0, fallI = 0;
uint8_t emoteFace = F_HAPPY, emoteI = 0;
int8_t demoFace = -1; uint32_t demoUntil = 0; bool demoAll = false;
const uint8_t EMOTES[] = {F_HAPPY, F_WINK, F_LOVE, F_HAPPY, F_WINK};
const uint16_t EMOTE_MS[] = {2500, 800, 2500, 2000, 800};
bool active(uint32_t until) { return (int32_t)(millis() - until) < 0; }
void printFaceName(uint8_t i) {
  const char* s = FACE_NAMES;
  while (i--) while (pgm_read_byte(s++)) {}
  char c; while ((c = pgm_read_byte(s++))) Serial.print(c);
}
void noteFall() { fallT[fallI++ % 3] = millis(); uint8_t n = 0; for (uint8_t i = 0; i < 3; i++) if (fallT[i] && millis() - fallT[i] < 30000) n++; if (n >= 3) sadUntil = millis() + 6000; }

void latchFace() {                                       // pick the face once per frame
  uint32_t now = millis();
  float pr = peakRate, pe = peakErr; peakRate = 0; peakErr = 0;
  fs.anim++;
  uint8_t f;
  if (demoFace >= 0) {                                   // "faces" / "face n" preview
    if (!active(demoUntil)) {
      if (demoAll && demoFace < F_COUNT - 1) { demoFace++; demoUntil = now + 2500; Serial.print(F("face ")); Serial.print(demoFace); Serial.print(' '); printFaceName(demoFace); Serial.println(); }
      else demoFace = -1;
    }
  }
  if (demoFace >= 0) f = demoFace;
  else if (faceForce >= 0) f = faceForce;
  else if (state == BALANCING) {
    if (pr > 180 && !active(pushBlock)) {                // bumped / pushed hard
      pushBlock = now + 800; surpriseUntil = now + 1200;
      pushT[pushI++ & 3] = now;
      uint32_t oldest = pushT[pushI & 3];
      if (oldest && now - oldest < 10000) angryUntil = now + 4000;   // 4 pushes in 10 s
    }
    if (pe > 15) dizzyUntil = now + 2000;
    if (active(angryUntil)) f = F_ANGRY;
    else if (active(dizzyUntil)) f = F_DIZZY;
    else if (active(surpriseUntil)) f = F_SURPRISED;
    else if (fabs(speed) > 0.15) f = F_EXCITED;
    else if (fabs(position) > 0.4) f = F_CONFUSED;
    else if (active(emoteUntil)) f = emoteFace;
    else if (now - balSince > 60000) f = F_COOL;
    else f = F_NORMAL;
    if ((f == F_NORMAL || f == F_COOL) && !active(nextEmote)) {     // idle emote every 9-15 s
      emoteFace = EMOTES[emoteI]; emoteUntil = now + EMOTE_MS[emoteI]; emoteI = (emoteI + 1) % sizeof(EMOTES);
      nextEmote = now + 9000 + (now % 6000);
    }
  }
  else if (state == ARMED) {
    if (active(sadUntil)) f = F_SAD;
    else if (fellAt && now - fellAt < 3000) f = F_DEAD;
    else if (now - armedSince > 30000) f = F_SLEEPY;
    else f = F_WAIT;
  }
  else f = F_SLEEP;
  fs.face = f;
  fs.blink = (now % 3500) < 160;
  float e = angle - g.trim;                              // lean forward -> eyes look down
  bool look = (f == F_NORMAL || f == F_WAIT || f == F_HAPPY || f == F_EXCITED || f == F_LOVE || f == F_COOL);
  fs.eyeDy = (look && demoFace < 0) ? (int8_t)constrain(e * 0.6, -3.0, 3.0) : 0;
  uint32_t gp = now % 9000;                              // glance left / right now and then
  fs.eyeDx = (f == F_NORMAL || f == F_WAIT) ? (gp < 700 ? -6 : (gp < 1400 ? 6 : 0)) : 0;
  uint8_t m = (uint8_t)constrain(fabs(speed) * 60.0, 0.0, 12.0);   // faster -> bigger open mouth
  if (m < 7) m = 7;
  fs.mouth = m;
}
bool oledChunk() {                                       // send one 16-byte piece; true when a frame finished
  if (oClearLeft) {                                      // wipe the whole screen first, a piece at a time
    if (!oledSetPos(oClearPage, oClearCol)) return false;
    uint8_t n = (132 - oClearCol < 16) ? 132 - oClearCol : 16;
    Wire.beginTransmission(oledAddr); Wire.write(0x40);
    for (uint8_t i = 0; i < n; i++) Wire.write(0);
    if (Wire.endTransmission() != 0) { oledLost(); return false; }
    oClearCol += n; if (oClearCol >= 132) { oClearCol = 0; oClearPage++; }
    oClearLeft--;
    return false;
  }
  if (oNewFrame) { latchFace(); oPage = FP0; oCol = FX0; oNewFrame = false; }
  if (!oledSetPos(oPage, oCol + OLED_COL_OFFSET)) return false;
  Wire.beginTransmission(oledAddr); Wire.write(0x40);
  for (uint8_t i = 0; i < 16; i++) {
    Wire.write(faceByte(oCol, oPage));
    if (++oCol > FX1) { oCol = FX0; if (++oPage > FP1) oNewFrame = true; break; }
  }
  if (Wire.endTransmission() != 0) { oledLost(); return false; }
  return oNewFrame;
}
void oledTask(uint32_t cycleStart) {                     // up to 2 pieces, only while there's time
  for (uint8_t n = 0; n < 2; n++) {
    if (!oledAddr || micros() - cycleStart > 3500) break;
    if (oledChunk()) break;
  }
}
void oledShowNow(int8_t f) {                             // draw a whole face right now (setup only)
  faceForce = f; oNewFrame = true;
  while (oledAddr) { bool done = oledChunk(); if (done && !oClearLeft) break; }
}
void oledKeepAlive() {                                   // runs every loop; each step takes < 1 ms
  static uint32_t lastTry = 0, lastWake = 0;
  if (!oledAddr) {
    if (millis() - lastTry > OLED_RETRY_MS) { lastTry = millis(); oledStart(true, 1); }
  } else if (millis() - lastWake > 3000) { lastWake = millis(); oledSendInit(); }
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

// ===================== recalibrate button =====================================
void calibrateAndStart();
void buttonTask() {                                      // runs every loop, never blocks while waiting
  static uint8_t st = 0; static uint32_t t = 0;
  bool down = digitalRead(BTN) == LOW;
  switch (st) {
    case 0: if (down) { st = 1; t = millis(); } break;                       // maybe pressed
    case 1: if (!down) st = 0;                                               // just noise
            else if (millis() - t > 40) {                                    // really pressed
              st = 2; stopBalancing(F("button"), false); faceForce = F_SETUP; oNewFrame = true;
              Serial.println(F("Button: let go, then hold the robot UPRIGHT and STILL..."));
            } break;
    case 2: if (!down) { st = 3; t = millis(); } break;                      // wait until you let go
    case 3: if (down) st = 2;
            else if (millis() - t > 1000) { st = 0; calibrateAndStart(); }   // 1 s after letting go
            break;
  }
}

// ===================== Serial commands =======================================
struct Ref { const char* n; float* v; };
Ref refs[] = {{"kp", &g.kp}, {"ki", &g.ki}, {"kd", &g.kd}, {"trim", &g.trim}, {"kv", &g.kv}, {"kx", &g.kx},
              {"cap", &g.cap}, {"decay", &g.decay}, {"minp", &g.minp}, {"ks", &ks}, {"mbal", &mbal}};
const uint16_t MAGIC = 0xBA20;
void saveAll() {
  EEPROM.put(0, MAGIC); EEPROM.put(2, g); EEPROM.put(2 + sizeof(Gains), dz);
  EEPROM.update(103, 0x5A); EEPROM.put(104, ks); EEPROM.put(108, mbal);      // steering settings
  Serial.println(F("saved"));
}
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
  else if (!strcmp(c, "oled")) oledStart(false, 1);
  else if (!strcmp(c, "faces")) { demoAll = true; demoFace = 0; demoUntil = millis() + 2500; Serial.println(F("face 0 sleep")); }
  else if (!strcmp(c, "face") && a) {
    demoAll = false; demoFace = constrain(atoi(a), 0, F_COUNT - 1); demoUntil = millis() + 5000;
    Serial.print(F("face ")); Serial.print(demoFace); Serial.print(' '); printFaceName(demoFace); Serial.println();
  }
  else if (!strcmp(c, "show")) { show(); Serial.print(F("enc=")); Serial.println(encOn); }
  else if (!strcmp(c, "save")) saveAll();
  else if (!strcmp(c, "check")) { oledShowNow(F_SETUP); wizard(); calibrateAndStart(); }
  else if (!strcmp(c, "defaults")) { g = Gains(); Serial.println(F("defaults loaded (type save to keep)")); show(); }
  else {
    for (Ref& r : refs) if (!strcmp(c, r.n) && a) { *r.v = atof(a); Serial.print(c); Serial.print(" = "); Serial.println(*r.v, 3); return; }
    Serial.println(F("commands: g x t check enc etest oled faces face p show save defaults | kp ki kd trim kv kx cap decay minp ks mbal <value>"));
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
  pinMode(BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrR, RISING);

  uint16_t m; EEPROM.get(0, m);
  bool haveSaved = (m == MAGIC);
  if (haveSaved) { EEPROM.get(2, g); EEPROM.get(2 + sizeof(Gains), dz); Serial.println(F("\nsaved settings loaded")); }
  { uint8_t e = EEPROM.read(100); if (e <= 2) encOn = e; }
  if (EEPROM.read(103) == 0x5A) {                         // saved steering settings
    float a, b; EEPROM.get(104, a); EEPROM.get(108, b);
    if (a >= 0 && a <= 50) ks = a;
    if (b >= -50 && b <= 50) mbal = b;
  }

  Wire.begin(); digitalWrite(SDA, LOW); digitalWrite(SCL, LOW); Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  mpuWrite(0x6B, 0x80); delay(100);
  mpuWrite(0x6B, 0x01); mpuWrite(0x1A, 0x03); mpuWrite(0x1B, 0x08); mpuWrite(0x1C, 0x08);
  float f, u, p;
  if (!mpuRead(f, u, p)) { Serial.println(F("MPU-6050 NOT FOUND - check A4/A5, 3.3V, GND")); while (true) delay(1000); }
  oledStart(false, 3);
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
  else { state = ARMED; armedSince = millis(); Serial.println(F("Not upright - stand me up near my balance point to start.")); }
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
  oledTask(next - LOOP_US);                                         // face, only if there's time left this cycle
  oledKeepAlive();                                                  // find / re-find / re-wake the OLED
  buttonTask();                                                     // recalibrate button
  if (printOn && millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print(F("angle ")); Serial.print(angle, 2); Serial.print(F("  target ")); Serial.print(target, 2);
    Serial.print(F("  out ")); Serial.print(out);
    Serial.print(F("  speed_cm/s ")); Serial.print(speed * 100, 1);
    Serial.print(F("  encL ")); Serial.print(cntL()); Serial.print(F("  encR ")); Serial.print(cntR());
    Serial.print(F("  turn ")); Serial.print(headDiff);
    Serial.print(F("  trim ")); Serial.print(g.trim, 1); Serial.print(F("  state "));
    Serial.println(state == BALANCING ? F("BALANCING") : state == ARMED ? F("ARMED") : F("off"));
  }
}
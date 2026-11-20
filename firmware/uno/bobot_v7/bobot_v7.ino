/*
 * BO-BOT - Uno + MPU-6050 + DRV8833 + SH1106/SSD1306 OLED.   Serial Monitor: 115200, "Newline".
 * Power on while holding it upright + still -> calibrates (1 s) and balances by itself.
 * Button = stop, let go, hold still -> recalibrates and balances again.
 * Commands: g start | x stop | t measure trim | check setup wizard | p live readout | show | save
 *           oled (look for screen) | faces (show all 16) | face <0-15>
 *           kp kd trim kv kx cap ks mbal <value>   (ks = keep-straight, mbal = motor balance %)
 * Wiring:  DRV8833 IN1 D5, IN2 D6, IN3 D9, IN4 D10, EEP D8 (+10k D8->GND), VCC battery+
 *          Left encoder green D2 yellow D4, right encoder green D3 yellow D7 (black 5V, blue GND)
 *          MPU-6050 + OLED: VCC 3.3V, GND, SDA A4, SCL A5      Button: D12 -> diagonal leg -> GND
 */
#include <Wire.h>
#include <EEPROM.h>

// saved settings (layout kept the same as before, so your saved values still load)
struct Gains { float kp = 28, ki = 0, kd = 0.6, trim = 0, kv = 30, kx = 3, cap = 230, decay = 0, minp = 0; } g;
struct Dirs { uint8_t mL = 0, mR = 1, eL = 0, eR = 1, imuF = 1, gInv = 0; } dz;   // set by the wizard
float ks = 1.5, mbal = 0;                       // keep-straight strength, motor balance %
const uint16_t MAGIC = 0xBA20;

const uint8_t AIN1 = 5, AIN2 = 6, BIN1 = 9, BIN2 = 10, EEP = 8, BTN = 12, MPU = 0x68;
const float FALL = 40, START = 23, ARM = 8, MAX_LEAN = 10;   // degrees
const float COUNTS_REV = 350, WHEEL_R = 0.017, DT = 0.005;   // 200 Hz loop

// ---------------- encoders + motors ----------------
volatile long encL = 0, encR = 0;
void isrL() { if (PIND & _BV(4)) encL--; else encL++; }
void isrR() { if (PIND & _BV(7)) encR--; else encR++; }
long cnt(volatile long& e, uint8_t inv) { noInterrupts(); long c = e; interrupts(); return inv ? -c : c; }
void drive(uint8_t a, uint8_t b, bool inv, int p) {          // slow decay, + = forward
  p = constrain(inv ? -p : p, -255, 255);
  if (p >= 0) { analogWrite(a, 255); analogWrite(b, 255 - p); } else { analogWrite(a, 255 + p); analogWrite(b, 255); }
}
void motorsOff() { analogWrite(AIN1, 0); analogWrite(AIN2, 0); analogWrite(BIN1, 0); analogWrite(BIN2, 0); }

// ---------------- MPU-6050 (Z accelerometer dead -> tilt from the forward axis only) ----------------
int16_t r[7];
bool mpuRaw() {
  Wire.beginTransmission(MPU); Wire.write(0x3B);
  if (Wire.endTransmission(false) || Wire.requestFrom(MPU, (uint8_t)14) != 14) return false;
  for (uint8_t i = 0; i < 7; i++) { uint8_t h = Wire.read(), l = Wire.read(); r[i] = (int16_t)((h << 8) | l); }
  return true;
}
bool mpuRead(float& fwd, float& pr) {                          // forward accel (g), raw pitch gyro (+ = tipping forward)
  if (!mpuRaw()) return false;
  int8_t s = (dz.imuF == 0 || dz.imuF == 2) ? 1 : -1;
  if (dz.imuF < 2) { fwd = s * r[0] / 8192.0; pr = s * r[5]; } else { fwd = s * r[1] / 8192.0; pr = -s * r[4]; }
  if (dz.gInv) pr = -pr;
  return true;
}
float accAngle(float fwd) { return asin(constrain(-fwd, -1.0, 1.0)) * RAD_TO_DEG; }
float measureAngle(int n) { float f, p, a = 0; int k = 0; for (int i = 0; i < n; i++) { if (mpuRead(f, p)) { a += accAngle(f); k++; } delay(5); } return k ? a / k : 0; }
float gyroOff = 0;
bool calibrate() {                                             // 1 s: gyro zero; false if it moved
  float f, p, s = 0, q = 0; int n = 0; uint32_t t0 = millis();
  while (millis() - t0 < 1000) { if (mpuRead(f, p)) { s += p; q += p * p; n++; } delay(3); }
  if (n < 150) return false;
  gyroOff = s / n;
  return q / n - gyroOff * gyroOff < 197.0 * 197.0;
}

// ---------------- balancing ----------------
enum State : uint8_t { OFF, ARMED, BALANCING } state = OFF;
float angle, rate, speed, position, posRef, target, peakRate, peakErr;
int out; long lastL, lastR, headDiff, win[5]; uint8_t wi, wn; bool printOn;
uint32_t uprightSince, fellAt, balSince, armedSince;
void noteFall();

void stopBal(const __FlashStringHelper* why, bool rearm) {
  motorsOff(); digitalWrite(EEP, LOW); out = 0; uprightSince = 0;
  state = rearm ? ARMED : OFF; armedSince = millis();
  Serial.print(F("STOPPED: ")); Serial.println(why);
}
void startBal() {
  digitalWrite(EEP, HIGH); state = BALANCING; speed = position = posRef = 0; wn = 0; headDiff = 0; uprightSince = 0;
  lastL = cnt(encL, dz.eL); lastR = cnt(encR, dz.eR); balSince = millis();
  Serial.println(F("BALANCING"));
}
void control() {                                               // every 5 ms
  float f, p;
  if (!mpuRead(f, p)) { motorsOff(); return; }
  rate = (p - gyroOff) / 65.5;                                 // deg/s
  angle = 0.996 * (angle + rate * DT) + 0.004 * accAngle(f);   // mostly gyro
  long cL = cnt(encL, dz.eL), cR = cnt(encR, dz.eR), dL = cL - lastL, dR = cR - lastR;
  lastL = cL; lastR = cR; headDiff += dL - dR;                 // headDiff > 0 = turned right
  win[wi] = dL + dR; wi = (wi + 1) % 5; if (wn < 5) wn++;
  long sum = 0; for (uint8_t i = 0; i < wn; i++) sum += win[i];
  speed += 0.3 * (WHEEL_R * (sum * 0.5 / COUNTS_REV * TWO_PI / (wn * DT) + rate * DEG_TO_RAD) - speed);  // m/s
  position += speed * DT;
  float err = angle - g.trim;
  if (state == OFF) return;
  if (state == ARMED) {                                        // restart when stood up
    if (fabs(err) < ARM) { if (!uprightSince) uprightSince = millis(); if (millis() - uprightSince > 300) startBal(); }
    else uprightSince = 0;
    return;
  }
  if (fabs(err) > FALL) { fellAt = millis() | 1; noteFall(); stopBal(F("fell over"), true); return; }
  peakRate = max(peakRate, fabs(rate)); peakErr = max(peakErr, fabs(err));
  target = constrain(-(g.kv * speed + g.kx * position), -MAX_LEAN, MAX_LEAN);   // lean to stop / come back
  out = constrain(g.kp * (err - target) + g.kd * rate, -g.cap, g.cap);
  float steer = constrain(ks * headDiff, -60.0, 60.0);                          // keep straight
  drive(AIN1, AIN2, dz.mL, out * (1 + mbal / 100) - steer);
  drive(BIN1, BIN2, dz.mR, out * (1 - mbal / 100) + steer);
}

// ---------------- setup wizard ----------------
void saveAll() {
  EEPROM.put(0, MAGIC); EEPROM.put(2, g); EEPROM.put(2 + sizeof(Gains), dz);
  EEPROM.update(103, 0x5A); EEPROM.put(104, ks); EEPROM.put(108, mbal);
  Serial.println(F("saved"));
}
void flushIn() { delay(30); while (Serial.available()) Serial.read(); }
void waitEnter(const __FlashStringHelper* m) { Serial.println(m); while (!Serial.available()) {} flushIn(); }
bool askYes(const __FlashStringHelper* q) {
  Serial.println(q);
  while (true) { while (!Serial.available()) {} char c = Serial.read() | 0x20; if (c == 'y' || c == 'n') { flushIn(); return c == 'y'; } }
}
void rawAvg(float* v) {                                        // ax, ay, gx, gy (raw)
  float a[4] = {0, 0, 0, 0}; int n = 0;
  for (int i = 0; i < 200; i++) { if (mpuRaw()) { a[0] += r[0]; a[1] += r[1]; a[2] += r[4]; a[3] += r[5]; n++; } delay(4); }
  for (uint8_t k = 0; k < 4; k++) v[k] = a[k] / n;
}
long spinCount(uint8_t a, uint8_t b, volatile long& e) {
  long c0 = cnt(e, 0); digitalWrite(EEP, HIGH); analogWrite(a, 255); analogWrite(b, 135);
  delay(1000); motorsOff(); digitalWrite(EEP, LOW); return cnt(e, 0) - c0;
}
void wizard() {
  motorsOff(); digitalWrite(EEP, LOW);
  float a[4], b[4], integ;
  Serial.println(F("\n=== SETUP WIZARD ==="));
  do {
    waitEnter(F("1) Hold it UPRIGHT + STILL, press Enter")); rawAvg(a);
    waitEnter(F("2) Tilt it FORWARD ~20 deg, hold still, press Enter")); rawAvg(b);
  } while (max(fabs(b[0] - a[0]), fabs(b[1] - a[1])) < 983);  // 0.12 g
  float dx = b[0] - a[0], dy = b[1] - a[1];
  dz.imuF = fabs(dx) > fabs(dy) ? (dx < 0 ? 0 : 1) : (dy < 0 ? 2 : 3);
  dz.gInv = 0;
  do {
    waitEnter(F("3) Hold it UPRIGHT, press Enter, then within 3 s tip it FORWARD ~20 deg"));
    integ = 0; uint32_t t0 = millis(), last = micros(); int8_t s = (dz.imuF == 0 || dz.imuF == 2) ? 1 : -1;
    while (millis() - t0 < 3000) {
      if (!mpuRaw()) continue;
      float dt = (micros() - last) / 1e6; last = micros();
      integ += ((dz.imuF < 2) ? s * (r[5] - a[3]) : -s * (r[4] - a[2])) / 65.5 * dt;
      delay(3);
    }
  } while (fabs(integ) < 5);
  dz.gInv = integ < 0;
  waitEnter(F("4) Wheels in the AIR. Press Enter - the LEFT wheel spins"));
  long dL = spinCount(AIN1, AIN2, encL);
  bool lF = askYes(F("   Did its TOP move towards the FRONT? (y/n)"));
  waitEnter(F("5) Press Enter - the RIGHT wheel spins"));
  long dR = spinCount(BIN1, BIN2, encR);
  bool rF = askYes(F("   Did its TOP move towards the FRONT? (y/n)"));
  dz.mL = !lF; dz.mR = !rF; dz.eL = (dL > 0) != lF; dz.eR = (dR > 0) != rF;
  if (labs(dL) < 30 || labs(dR) < 30) Serial.println(F("!! an encoder hardly counts - check its wires"));
  waitEnter(F("6) Hold it at its BALANCE POINT, still, press Enter"));
  g.trim = measureAngle(300);
  saveAll();
}

// ---------------- OLED (no screen buffer: each byte is worked out on the fly, sent 16 at a time) ----------------
#define OLED_COL_OFFSET 2                                      // 2 = SH1106 (yours), 0 = SSD1306
uint8_t oledAddr = 0, oPage, oCol, oC0, oC1; bool oNewFrame = true, oClear = true;
const uint8_t FX0 = 16, FX1 = 111;                             // face area: columns 16..111, pages 0..7
void oledCmd(const uint8_t* c, uint8_t n) { Wire.beginTransmission(oledAddr); Wire.write(0x00); Wire.write(c, n); Wire.endTransmission(); }
void oledSetPos(uint8_t pg, uint8_t col) { uint8_t c[] = {(uint8_t)(0xB0 | pg), (uint8_t)(col & 0x0F), (uint8_t)(0x10 | (col >> 4))}; oledCmd(c, 3); }
void oledInit(bool quiet) {
  oledAddr = 0;
  for (uint8_t t = 0; t < 3 && !oledAddr; t++) {
    for (uint8_t a = 0x3C; a <= 0x3D && !oledAddr; a++) { Wire.beginTransmission(a); if (Wire.endTransmission() == 0) oledAddr = a; }
    if (!oledAddr && !quiet) delay(20);
  }
  if (!oledAddr) { if (!quiet) Serial.println(F("no OLED found (will keep looking)")); return; }
  Serial.println(F("OLED found"));
  const uint8_t i1[] = {0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x02};
  const uint8_t i2[] = {0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0x2E, 0xAF};
  oledCmd(i1, sizeof(i1)); oledCmd(i2, sizeof(i2));
  oNewFrame = oClear = true;                                   // next frame wipes the whole screen
}

// ---------------- 16 faces ----------------
//  0 sleep (stopped)   1 wait (stand me up)  2 sleepy (waiting >30 s)  3 setup (calibrating)
//  4 normal (calm)     5 happy / 7 wink / 8 love (idle emotes)          6 excited (moving fast)
//  9 surprised (pushed hard)   10 dizzy (big wobble, saved)   11 angry (pushed 4x in 10 s)
// 12 confused (wandered 40 cm) 13 cool (balancing >60 s)      14 dead (fell)  15 sad (fell 3x in 30 s)
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
uint8_t faceByte(int x, uint8_t p) {
  if (x < FX0 || x > FX1) return 0;
  const uint8_t* fp = FACE_PARTS[fs.face];
  uint8_t eL = pgm_read_byte(fp), eR = pgm_read_byte(fp + 1), mo = pgm_read_byte(fp + 2), ex = pgm_read_byte(fp + 3);
  int exL = EYE_LX + fs.eyeDx, exR = EYE_RX + fs.eyeDx, cy = EYE_Y + fs.eyeDy;
  uint8_t b = eyeCol(eL, (int)x - exL, cy, p) | eyeCol(eR, (int)x - exR, cy, p) | mouthCol(mo, (int)x - MOUTH_X, p);
  if (ex) b |= extraCol(ex, x, cy, exL, exR, p);
  int d = fs.anim % 28; d = d < 14 ? d : 28 - d;           // "alive" dot: slides along the bottom edge, one step per frame
  if (abs(x - (FX0 + 6 + d * 6)) <= 2) b |= span(61, 63, p);
  return b;
}

int8_t faceForce = F_SETUP, demoFace = -1; bool demoAll;
uint32_t emoteUntil, nextEmote, surpriseUntil, dizzyUntil, angryUntil, confusedUntil, pushBlock, sadUntil, demoUntil;
uint32_t pushT[4], fallT[3]; uint8_t pushI, fallI, emoteI, emoteFace;
const uint8_t EMOTES[] = {F_HAPPY, F_WINK, F_LOVE, F_HAPPY, F_WINK};
const uint16_t EMOTE_MS[] = {2500, 800, 2500, 2000, 800};
bool active(uint32_t until) { return (int32_t)(millis() - until) < 0; }
void printFaceName(uint8_t i) { const char* s = FACE_NAMES; while (i--) while (pgm_read_byte(s++)) {} char c; while ((c = pgm_read_byte(s++))) Serial.print(c); Serial.println(); }
void noteFall() {
  fallT[fallI++ % 3] = millis(); uint8_t n = 0;
  for (uint8_t i = 0; i < 3; i++) if (fallT[i] && millis() - fallT[i] < 30000) n++;
  if (n >= 3) sadUntil = millis() + 6000;
}
void latchFace() {                                             // pick the face, once per frame
  uint32_t now = millis();
  float pr = peakRate, pe = peakErr; peakRate = peakErr = 0;
  fs.anim++;
  if (demoFace >= 0 && !active(demoUntil)) {
    if (demoAll && demoFace < F_COUNT - 1) { demoFace++; demoUntil = now + 2500; printFaceName(demoFace); } else demoFace = -1;
  }
  uint8_t f;
  if (demoFace >= 0) f = demoFace;
  else if (faceForce >= 0) f = faceForce;
  else if (state == BALANCING) {
    if (pr > 180 && !active(pushBlock)) {                      // bumped hard
      pushBlock = now + 800; surpriseUntil = now + 1200; pushT[pushI++ & 3] = now;
      if (pushT[pushI & 3] && now - pushT[pushI & 3] < 10000) angryUntil = now + 4000;
    }
    if (pe > 15) dizzyUntil = now + 2000;
    if (fabs(position - posRef) > 0.4) { posRef = position; confusedUntil = now + 2500; }   // wandered 40 cm
    f = active(angryUntil) ? F_ANGRY : active(dizzyUntil) ? F_DIZZY : active(surpriseUntil) ? F_SURPRISED
      : fabs(speed) > 0.15 ? F_EXCITED : active(confusedUntil) ? F_CONFUSED : active(emoteUntil) ? emoteFace
      : now - balSince > 60000 ? F_COOL : F_NORMAL;
    if ((f == F_NORMAL || f == F_COOL) && !active(nextEmote)) {     // idle emote every 9-15 s
      emoteFace = EMOTES[emoteI]; emoteUntil = now + EMOTE_MS[emoteI]; emoteI = (emoteI + 1) % sizeof(EMOTES);
      nextEmote = now + 9000 + now % 6000;
    }
  }
  else if (state == ARMED) f = active(sadUntil) ? F_SAD : (fellAt && now - fellAt < 3000) ? F_DEAD : now - armedSince > 30000 ? F_SLEEPY : F_WAIT;
  else f = F_SLEEP;
  fs.face = f;
  fs.blink = now % 3500 < 160;
  bool look = f == F_NORMAL || f == F_WAIT || f == F_HAPPY || f == F_EXCITED || f == F_LOVE || f == F_COOL;
  fs.eyeDy = (look && demoFace < 0) ? (int8_t)constrain((angle - g.trim) * 0.6, -3.0, 3.0) : 0;   // lean -> look down/up
  uint32_t gp = now % 9000;
  fs.eyeDx = (f == F_NORMAL || f == F_WAIT) ? (gp < 700 ? -6 : gp < 1400 ? 6 : 0) : 0;             // glance around
  fs.mouth = max((uint8_t)7, (uint8_t)constrain(fabs(speed) * 60.0, 0.0, 12.0));
}
bool oledChunk() {                                             // send 16 bytes; true at end of a frame
  if (oNewFrame) {
    latchFace(); oNewFrame = false; oPage = 0;
    oC0 = oClear ? 0 : FX0 + OLED_COL_OFFSET; oC1 = oClear ? 131 : FX1 + OLED_COL_OFFSET; oClear = false; oCol = oC0;
  }
  oledSetPos(oPage, oCol);
  Wire.beginTransmission(oledAddr); Wire.write(0x40);
  for (uint8_t i = 0; i < 16; i++) {
    Wire.write(faceByte(oCol - OLED_COL_OFFSET, oPage));
    if (++oCol > oC1) { oCol = oC0; if (++oPage > 7) oNewFrame = true; break; }
  }
  if (Wire.endTransmission()) { oledAddr = 0; oNewFrame = true; Serial.println(F("OLED lost - reconnecting")); }
  return oNewFrame;
}
void oledShowNow(int8_t f) { faceForce = f; oNewFrame = true; while (oledAddr && !oledChunk()) {} }

// ---------------- calibrate + start, button, commands ----------------
void calibrateAndStart() {
  motorsOff(); digitalWrite(EEP, LOW); state = OFF;
  oledShowNow(F_SETUP);
  Serial.println(F("Calibrating - hold it upright + still..."));
  while (!calibrate()) Serial.println(F("  it moved - again..."));
  angle = measureAngle(40);                                    // 0.2 s: accurate starting tilt
  if (fabs(angle - g.trim) < START) startBal(); else { state = ARMED; armedSince = millis(); Serial.println(F("stand me up")); }
  faceForce = -1; oNewFrame = true;
}
void buttonTask() {                                            // press -> stop; 1 s after letting go -> recalibrate
  static uint8_t st = 0; static uint32_t t = 0;
  bool down = digitalRead(BTN) == LOW;
  if (st == 0 && down) { st = 1; t = millis(); }
  else if (st == 1) { if (!down) st = 0; else if (millis() - t > 40) { st = 2; stopBal(F("button - let go, hold still"), false); faceForce = F_SETUP; oNewFrame = true; } }
  else if (st == 2 && !down) { st = 3; t = millis(); }
  else if (st == 3) { if (down) st = 2; else if (millis() - t > 1000) { st = 0; calibrateAndStart(); } }
}
struct Ref { const char* n; float* v; };
Ref refs[] = {{"kp", &g.kp}, {"kd", &g.kd}, {"trim", &g.trim}, {"kv", &g.kv}, {"kx", &g.kx}, {"cap", &g.cap}, {"ks", &ks}, {"mbal", &mbal}};
void show() { for (Ref& x : refs) { Serial.print(x.n); Serial.print('='); Serial.print(*x.v, 2); Serial.print(' '); } Serial.println(); }
void command(char* line) {
  char* c = strtok(line, " "); char* a = strtok(nullptr, " ");
  if (!c) return;
  if (!strcmp(c, "g")) { if (fabs(angle - g.trim) < FALL) startBal(); }
  else if (!strcmp(c, "x")) stopBal(F("x"), false);
  else if (!strcmp(c, "p")) printOn = !printOn;
  else if (!strcmp(c, "t")) { if (state == BALANCING) Serial.println(F("type x first")); else { g.trim = measureAngle(300); Serial.print(F("trim=")); Serial.println(g.trim, 2); } }
  else if (!strcmp(c, "show")) show();
  else if (!strcmp(c, "save")) saveAll();
  else if (!strcmp(c, "check")) { oledShowNow(F_SETUP); wizard(); calibrateAndStart(); }
  else if (!strcmp(c, "oled")) { oledInit(false); oNewFrame = true; }
  else if (!strcmp(c, "faces")) { demoAll = true; demoFace = 0; demoUntil = millis() + 2500; printFaceName(0); }
  else if (!strcmp(c, "face") && a) { demoAll = false; demoFace = constrain(atoi(a), 0, F_COUNT - 1); demoUntil = millis() + 5000; printFaceName(demoFace); }
  else {
    for (Ref& x : refs) if (!strcmp(c, x.n) && a) { *x.v = atof(a); show(); return; }
    Serial.println(F("? g x t check p show save oled faces face | kp kd trim kv kx cap ks mbal <value>"));
  }
}
void serialPoll() {
  static char buf[24]; static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { buf[n] = 0; command(buf); n = 0; }
    else if (c != '\r' && n < sizeof(buf) - 1) buf[n++] = c;
  }
}

void setup() {
  digitalWrite(EEP, LOW); pinMode(EEP, OUTPUT);                // driver asleep + motors off first
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); motorsOff();
  TCCR1A = (TCCR1A & ~_BV(WGM11)) | _BV(WGM10); TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);   // D9/D10 ~976 Hz
  Serial.begin(115200);
  pinMode(2, INPUT_PULLUP); pinMode(3, INPUT_PULLUP); pinMode(4, INPUT_PULLUP); pinMode(7, INPUT_PULLUP); pinMode(BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), isrL, RISING); attachInterrupt(digitalPinToInterrupt(3), isrR, RISING);
  uint16_t m; EEPROM.get(0, m);
  bool saved = m == MAGIC;
  if (saved) { EEPROM.get(2, g); EEPROM.get(2 + sizeof(Gains), dz); }
  if (EEPROM.read(103) == 0x5A) { float a, b; EEPROM.get(104, a); EEPROM.get(108, b); if (a >= 0 && a <= 50) ks = a; if (b >= -50 && b <= 50) mbal = b; }
  Wire.begin(); digitalWrite(SDA, LOW); digitalWrite(SCL, LOW); Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);
  const uint8_t mpuInit[][2] = {{0x6B, 0x80}, {0x6B, 0x01}, {0x1A, 0x03}, {0x1B, 0x08}, {0x1C, 0x08}};   // reset, clock, filter, 500 dps, 4 g
  for (uint8_t i = 0; i < 5; i++) { Wire.beginTransmission(MPU); Wire.write(mpuInit[i], 2); Wire.endTransmission(); delay(i ? 1 : 100); }
  if (!mpuRaw()) { Serial.println(F("MPU-6050 NOT FOUND - check A4/A5")); while (true) {} }
  oledInit(false);
  if (!saved) { oledShowNow(F_SETUP); wizard(); }
  calibrateAndStart();
}

void loop() {
  static uint32_t next = micros(), lastPrint, lastOledTry;
  serialPoll();
  if ((int32_t)(micros() - next) < 0) return;
  if ((int32_t)(micros() - next) > 5000) next = micros();
  uint32_t cycle = next; next += 5000;
  control();
  for (uint8_t n = 0; n < 2 && oledAddr && micros() - cycle < 3500; n++) if (oledChunk()) break;   // face, only with spare time
  if (!oledAddr && millis() - lastOledTry > 1000 && micros() - cycle < 3000) { lastOledTry = millis(); oledInit(true); }   // screen glitched -> bring it back
  buttonTask();
  if (printOn && millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print(F("angle ")); Serial.print(angle, 1); Serial.print(F(" out ")); Serial.print(out);
    Serial.print(F(" speed ")); Serial.print(speed * 100, 1); Serial.print(F(" turn ")); Serial.print(headDiff);
    Serial.print(F(" trim ")); Serial.print(g.trim, 1); Serial.print(' ');
    Serial.println(state == BALANCING ? F("BALANCING") : state == ARMED ? F("ARMED") : F("off"));
  }
}
/*
 * ============================================================================
 *  BalanceBotUno — the balancing robot firmware for an ARDUINO UNO
 * ============================================================================
 *  Same control logic and gains as the ESP32 version (port of controller.cpp),
 *  squeezed onto a 16 MHz Uno:
 *    - 200 Hz balance loop (every 5 ms) in loop(), timed with micros()
 *    - MPU-6050 and OLED share the one I2C bus (A4 / A5), 400 kHz
 *    - the OLED face is drawn in 8 strips, one every other 5 ms tick
 *      (U8g2 page mode: only 128 bytes of RAM), ~12 frames per second
 *    - encoders: one interrupt per motor (D2, D3) -> 350 counts per wheel turn
 *    - gains: Serial Monitor (115200, Newline), saved in EEPROM
 *  No Wi-Fi / phone page on the Uno. Driving: "drive 0.2 0" in the Serial Monitor.
 *
 *  Library: U8g2 by olikraus (Library Manager). Board: Arduino Uno.
 *
 *  WIRING (Uno)
 *    MPU-6050  VCC->3.3V  GND->GND  SDA->A4  SCL->A5
 *    OLED      VCC->3.3V  GND->GND  SDA->A4  SCL->A5   (same bus, address 0x3C)
 *    DRV8833   IN1->D5  IN2->D6  IN3->D9  IN4->D10  EEP->5V  GND->GND  VCC/VM->battery +
 *    Left motor   red->OUT1 white->OUT2  black->5V blue->GND  green->D2 yellow->D4
 *    Right motor  red->OUT3 white->OUT4  black->5V blue->GND  green->D3 yellow->D7
 *    Battery  + -> DRV VCC/VM and Uno VIN     - -> GND
 *    (USB from the laptop can stay plugged in for the Serial Monitor.)
 *
 *  START-UP: keep the robot still while the screen says "Calibrating",
 *  then hold it upright at its balance point: it starts after 0.6 s.
 * ============================================================================
 */
#include <Wire.h>
#include <U8g2lib.h>
#include <EEPROM.h>

// ---------------------------------------------------------------- settings
// Pins
const uint8_t AIN1 = 5, AIN2 = 6;     // left motor  (DRV IN1 / IN2)
const uint8_t BIN1 = 9, BIN2 = 10;    // right motor (DRV IN3 / IN4)
const uint8_t ENC_L_A = 2, ENC_R_A = 3;   // interrupt pins (green)
// ENC_L_B = D4, ENC_R_B = D7 (yellow) are read directly from port D in the ISRs

// Directions (same meaning as config.h of the ESP32 version)
#define MOTOR_L_INVERT  false
#define MOTOR_R_INVERT  true
#define ENC_L_INVERT    false
#define ENC_R_INVERT    true

// MPU-6050 mounting (run T2 / the axis test to find these)
#define IMU_FORWARD      0      // 0=+X 1=-X 2=+Y 3=-Y points to the robot's front
#define IMU_UPSIDE_DOWN  false
#define IMU_ACCEL_NO_Z   1      // 1 = your board's Z accelerometer is faulty: use the forward axis only

// Robot constants
const float WHEEL_R      = 0.017f;    // m
const float TRACK        = 0.078f;    // m (set to your real track if different)
const float COUNTS_REV   = 350.0f;    // 7 pulses x 50:1, counting one edge of A
const float EMF_V_PER_MPS = 10.4f;
const float VBAT_ASSUMED = 7.6f;      // no battery sensing on this build

// Timing
const uint16_t LOOP_US = 5000;        // 200 Hz
const float DT = LOOP_US / 1e6f;

// ---------------------------------------------------------------- gains
struct Gains {
  float kp = 55, kd = 0.35f, ki = 0, kpv = 20, kiv = 0.9f, kpt = 0.3f, kit = 1.0f;
  float alpha = 0.98f, trim = 0, kff = 1.0f, cap = 204, fall = 40, lean = 10;
};
Gains g;
const uint16_t EE_MAGIC = 0xB417;

// ---------------------------------------------------------------- OLED
U8G2_SH1106_128X64_NONAME_1_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
// If the picture is shifted 2 pixels, your module is an SSD1306:
// U8G2_SSD1306_128X64_NONAME_1_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

// ---------------------------------------------------------------- state
volatile long encL = 0, encR = 0;
float fwdV[3], upV[3], axleV[3];
float gyroOff[3] = {0, 0, 0};

enum State : uint8_t { WAIT_UPRIGHT, BALANCING, FALLEN };
State state = WAIT_UPRIGHT;
float angle = 0, rate = 0, target = 0, speed = 0, yawRate = 0;
float vCmd = 0, yawCmd = 0, vIn = 0, yawIn = 0;
uint32_t cmdMs = 0;
float posI = 0, angI = 0, headI = 0;
long lastL = 0, lastR = 0;
long win[5]; uint8_t winIdx = 0, winN = 0;
int pwmL = 0, pwmR = 0;
uint32_t uprightSince = 0;
bool plot = false;
uint16_t loopUs = 0;

// ---------------------------------------------------------------- encoders
// D4 = PIND bit 4, D7 = PIND bit 7 (fast direct reads)
void isrL() { if (PIND & _BV(4)) encL--; else encL++; }
void isrR() { if (PIND & _BV(7)) encR--; else encR++; }

// ---------------------------------------------------------------- motors
void driveOne(uint8_t in1, uint8_t in2, bool inv, int p) {
  if (inv) p = -p;
  p = constrain(p, -255, 255);
  if (p >= 0) { analogWrite(in1, 255); analogWrite(in2, 255 - p); }   // slow decay
  else        { analogWrite(in1, 255 + p); analogWrite(in2, 255); }
}
void motorsSet(int l, int r) { driveOne(AIN1, AIN2, MOTOR_L_INVERT, l); driveOne(BIN1, BIN2, MOTOR_R_INVERT, r); }
void motorsCoast() { analogWrite(AIN1, 0); analogWrite(AIN2, 0); analogWrite(BIN1, 0); analogWrite(BIN2, 0); }

// ---------------------------------------------------------------- MPU-6050
const uint8_t MPU = 0x68;
void mpuWrite(uint8_t r, uint8_t v) { Wire.beginTransmission(MPU); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
bool mpuRead(int16_t raw[7]) {
  Wire.beginTransmission(MPU); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, (uint8_t)14) != 14) return false;
  for (uint8_t i = 0; i < 7; i++) { uint8_t h = Wire.read(), l = Wire.read(); raw[i] = (int16_t)((h << 8) | l); }
  return true;
}
float dot3(const float a[3], const float b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
void setupAxes() {
  for (uint8_t i = 0; i < 3; i++) fwdV[i] = upV[i] = 0;
  if (IMU_FORWARD == 0) fwdV[0] = 1; else if (IMU_FORWARD == 1) fwdV[0] = -1;
  else if (IMU_FORWARD == 2) fwdV[1] = 1; else fwdV[1] = -1;
  upV[2] = IMU_UPSIDE_DOWN ? -1 : 1;
  axleV[0] = upV[1] * fwdV[2] - upV[2] * fwdV[1];
  axleV[1] = upV[2] * fwdV[0] - upV[0] * fwdV[2];
  axleV[2] = upV[0] * fwdV[1] - upV[1] * fwdV[0];
}
// returns false if the robot moved during calibration
bool calibrateGyro() {
  float s[3] = {0, 0, 0}, q[3] = {0, 0, 0}; int16_t raw[7]; int n = 0;
  for (int i = 0; i < 300; i++) {
    if (mpuRead(raw)) { for (uint8_t k = 0; k < 3; k++) { s[k] += raw[4 + k]; q[k] += (float)raw[4 + k] * raw[4 + k]; } n++; }
    delay(3);
  }
  if (n < 150) return false;
  bool still = true;
  for (uint8_t k = 0; k < 3; k++) {
    gyroOff[k] = s[k] / n;
    if (q[k] / n - gyroOff[k] * gyroOff[k] > 131.0f * 131.0f) still = false;   // > 2 deg/s shaking
  }
  return still;
}

// ---------------------------------------------------------------- control (5 ms)
void controlStep() {
  int16_t raw[7];
  if (!mpuRead(raw)) { motorsCoast(); return; }
  float acc[3] = {raw[0] / 8192.0f, raw[1] / 8192.0f, raw[2] / 8192.0f};
  float gyr[3] = {(raw[4] - gyroOff[0]) / 65.5f, (raw[5] - gyroOff[1]) / 65.5f, (raw[6] - gyroOff[2]) / 65.5f};
  rate = dot3(gyr, axleV);
  yawRate = dot3(gyr, upV);
#if IMU_ACCEL_NO_Z
  float accAngle = asin(constrain(-dot3(acc, fwdV), -1.0f, 1.0f)) * RAD_TO_DEG;
#else
  float accAngle = atan2(-dot3(acc, fwdV), dot3(acc, upV)) * RAD_TO_DEG;
#endif
  angle = g.alpha * (angle + rate * DT) + (1 - g.alpha) * accAngle;

  noInterrupts(); long cL = encL, cR = encR; interrupts();
  if (ENC_L_INVERT) cL = -cL;
  if (ENC_R_INVERT) cR = -cR;
  long dL = cL - lastL, dR = cR - lastR; lastL = cL; lastR = cR;
  win[winIdx] = dL + dR; winIdx = (winIdx + 1) % 5; if (winN < 5) winN++;
  long sum = 0; for (uint8_t i = 0; i < winN; i++) sum += win[i];
  float rel = sum * 0.5f / COUNTS_REV * TWO_PI / (winN * DT);
  speed += 0.3f * (WHEEL_R * (rel + rate * DEG_TO_RAD) - speed);

  // drive command from Serial expires after 0.5 s (failsafe)
  float vi = (millis() - cmdMs < 500) ? vIn : 0, yi = (millis() - cmdMs < 500) ? yawIn : 0;

  uint32_t now = millis();
  if (state == BALANCING && fabs(angle) > g.fall) state = FALLEN;
  if (state != BALANCING) {
    if (fabs(angle - g.trim) < 4) {
      if (!uprightSince) uprightSince = now;
      if (now - uprightSince > 600) {
        state = BALANCING; posI = angI = headI = 0; speed = 0; vCmd = yawCmd = 0; winN = 0;
      }
    } else uprightSince = 0;
    motorsCoast(); pwmL = pwmR = 0; target = 0;
    return;
  }

  vCmd += constrain(vi - vCmd, -0.6f * DT, 0.6f * DT);
  yawCmd += constrain(yi - yawCmd, -400 * DT, 400 * DT);

  float ev = vCmd - speed, lim = 6.0f / max(g.kiv, 1e-6f);
  posI = constrain(posI + ev * DT, -lim, lim);
  target = constrain(g.kpv * ev + g.kiv * posI, -g.lean, g.lean);

  float e = angle - g.trim - target;
  angI = constrain(angI + e * DT, -50, 50);
  float u = g.kp * e + g.kd * rate + g.ki * angI;
  u += g.kff * vCmd * EMF_V_PER_MPS / VBAT_ASSUMED * 255;

  float ey = yawCmd - yawRate;
  headI = constrain(headI + ey * DT, -90, 90);
  float diff = g.kpt * ey + g.kit * headI;
  diff += g.kff * (yawCmd * DEG_TO_RAD) * TRACK * 0.5f * EMF_V_PER_MPS / VBAT_ASSUMED * 255;

  pwmL = (int)constrain(u - diff, -g.cap, g.cap);
  pwmR = (int)constrain(u + diff, -g.cap, g.cap);
  motorsSet(pwmL, pwmR);
}

// ---------------------------------------------------------------- face
const char* mood = "idle";
float holdT = 0, blinkT = -1, nextBlink = 2, faceT = 0;

int faceOy = 0;            // frozen for the whole frame so all strips match
void drawFace(float blink) {
  const int lx = 36, rx = 92, ey = 26;
  const int eyes[2] = {lx, rx};
  int oy = faceOy;
  oled.setDrawColor(1);
  if (!strcmp(mood, "dizzy")) {
    for (int cx : eyes) for (int o = -2; o <= 2; o++) {
      oled.drawLine(cx - 11 + o, ey - 11, cx + 11 + o, ey + 11);
      oled.drawLine(cx - 11 + o, ey + 11, cx + 11 + o, ey - 11);
    }
    for (int i = 0; i < 5; i++) for (int o = 0; o < 3; o++)
      oled.drawLine(46 + i * 8, 52 + (i % 2) * 4 + o, 54 + i * 8, 52 + ((i + 1) % 2) * 4 + o);
  } else if (!strcmp(mood, "surprised")) {
    for (int cx : eyes) {
      oled.drawFilledEllipse(cx, ey, 15, 17); oled.setDrawColor(0);
      oled.drawFilledEllipse(cx, ey, 9, 11);  oled.setDrawColor(1);
      oled.drawFilledEllipse(cx, ey, 5, 6);
    }
    oled.drawFilledEllipse(64, 53, 6, 7); oled.setDrawColor(0); oled.drawFilledEllipse(64, 53, 3, 4); oled.setDrawColor(1);
  } else if (!strcmp(mood, "happy")) {
    for (int cx : eyes) {
      oled.drawFilledEllipse(cx, ey + 8 + oy, 13, 12, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
      oled.setDrawColor(0); oled.drawFilledEllipse(cx, ey + 8 + oy, 7, 6, U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
      oled.drawBox(cx - 14, ey + 9 + oy, 29, 14); oled.setDrawColor(1);
    }
    oled.drawFilledEllipse(64, 44, 14, 12, U8G2_DRAW_LOWER_LEFT | U8G2_DRAW_LOWER_RIGHT);
    oled.setDrawColor(0); oled.drawFilledEllipse(64, 44, 10, 8, U8G2_DRAW_LOWER_LEFT | U8G2_DRAW_LOWER_RIGHT);
    oled.drawBox(48, 36, 33, 8); oled.setDrawColor(1);
  } else {   // idle / worried / waiting
    bool worried = !strcmp(mood, "worried");
    int h = (int)(30 * blink);
    for (int i = 0; i < 2; i++) {
      int cx = i ? rx : lx;
      if (h < 4) { oled.drawBox(cx - 13, ey + oy - 1, 27, 3); continue; }
      oled.drawRBox(cx - 13, ey - h / 2 + oy, 27, h, h > 14 ? 7 : 2);
      if (worried) {   // slanted brow cut
        oled.setDrawColor(0);
        if (i == 0) oled.drawTriangle(cx - 14, ey - 16 + oy, cx + 14, ey - 16 + oy, cx + 14, ey - 4 + oy);
        else        oled.drawTriangle(cx - 14, ey - 16 + oy, cx + 14, ey - 16 + oy, cx - 14, ey - 4 + oy);
        oled.setDrawColor(1);
      }
    }
    if (worried) {
      for (int i = 0; i < 4; i++) for (int o = 0; o < 3; o++)
        oled.drawLine(50 + i * 7, 54 + (i % 2) * 3 + o, 57 + i * 7, 54 + ((i + 1) % 2) * 3 + o);
    } else {
      oled.drawEllipse(64, 47, 9, 6, U8G2_DRAW_LOWER_LEFT | U8G2_DRAW_LOWER_RIGHT);
      oled.drawEllipse(64, 47, 9, 7, U8G2_DRAW_LOWER_LEFT | U8G2_DRAW_LOWER_RIGHT);
      oled.drawEllipse(64, 47, 10, 7, U8G2_DRAW_LOWER_LEFT | U8G2_DRAW_LOWER_RIGHT);
    }
    if (state == WAIT_UPRIGHT) {
      oled.setDrawColor(0); oled.drawBox(0, 56, 128, 8); oled.setDrawColor(1);
      oled.setFont(u8g2_font_5x7_tr); oled.drawStr(18, 63, "hold me upright...");
    }
  }
}

// The OLED runs in "page mode": U8g2 keeps only one 8-pixel strip (128 bytes)
// in RAM. Every second 5 ms tick, drawFace() renders that strip and sends it
// (~3.5 ms), so a whole frame takes 8 strips = 80 ms (12 fps) and the balance
// loop is never blocked for long.
bool inFrame = false;
uint8_t faceTickDiv = 0;
float frameBlink = 1;
void faceTick() {
  if (++faceTickDiv & 1) return;               // every other tick
  if (!inFrame) {                              // start of a new frame: decide what to show
    float dt = 16 * DT; faceT += dt; holdT = max(0.0f, holdT - dt);
    if (state == FALLEN) mood = "dizzy";
    else if (fabs(rate) > 90) { mood = "surprised"; holdT = 1.2f; }
    else if (holdT > 0) { /* keep */ }
    else if (fabs(angle) > 12) mood = "worried";
    else if (vCmd > 0.05f) mood = "happy";
    else mood = "idle";
    frameBlink = 1;
    if (faceT > nextBlink && !strcmp(mood, "idle")) { blinkT = 0; nextBlink = faceT + 2 + random(0, 3000) / 1000.0f; }
    if (blinkT >= 0) { blinkT += dt; float ph = blinkT / 0.16f; frameBlink = ph < 1 ? fabs(1 - 2 * ph) : 1; if (ph >= 1) blinkT = -1; }
    faceOy = (int)constrain(angle / 3, -5, 5);
    oled.firstPage();
    inFrame = true;
  }
  drawFace(frameBlink);
  if (!oled.nextPage()) inFrame = false;       // sends this strip, moves to the next
}

void faceMessage(const char* a, const char* b) {    // only used in setup(), blocking is fine
  oled.firstPage();
  do {
    oled.setFont(u8g2_font_helvB10_tr); oled.drawStr(0, 16, a);
    oled.setFont(u8g2_font_6x10_tr);    oled.drawStr(0, 36, b);
  } while (oled.nextPage());
}

// ---------------------------------------------------------------- settings / Serial
struct GainRef { const char* name; float* p; };
GainRef refs[] = {{"kp", &g.kp}, {"kd", &g.kd}, {"ki", &g.ki}, {"kpv", &g.kpv}, {"kiv", &g.kiv},
                  {"kpt", &g.kpt}, {"kit", &g.kit}, {"alpha", &g.alpha}, {"trim", &g.trim},
                  {"kff", &g.kff}, {"cap", &g.cap}, {"fall", &g.fall}, {"lean", &g.lean}};
const uint8_t NREFS = sizeof(refs) / sizeof(refs[0]);

void saveGains() { EEPROM.put(0, EE_MAGIC); EEPROM.put(2, g); }
void loadGains() { uint16_t m; EEPROM.get(0, m); if (m == EE_MAGIC) { EEPROM.get(2, g); Serial.println(F("gains loaded from EEPROM")); } }

void showGains() {
  for (uint8_t i = 0; i < NREFS; i++) { Serial.print(refs[i].name); Serial.print('='); Serial.print(*refs[i].p, 3); Serial.print(i % 5 == 4 ? '\n' : ' '); }
  Serial.println();
}

void handleLine(char* line) {
  char* cmd = strtok(line, " ");
  if (!cmd) return;
  char* a1 = strtok(nullptr, " ");
  char* a2 = strtok(nullptr, " ");
  if (!strcmp(cmd, "help")) Serial.println(F("show | <gain> <value> | save | defaults | drive <v> <yaw> | plot on/off | status"));
  else if (!strcmp(cmd, "show")) showGains();
  else if (!strcmp(cmd, "save")) { saveGains(); Serial.println(F("saved")); }
  else if (!strcmp(cmd, "defaults")) { g = Gains(); Serial.println(F("defaults (type save to keep)")); }
  else if (!strcmp(cmd, "plot")) plot = a1 && !strcmp(a1, "on");
  else if (!strcmp(cmd, "drive")) { vIn = a1 ? constrain(atof(a1), -0.35, 0.35) : 0; yawIn = a2 ? constrain(atof(a2), -120, 120) : 0; cmdMs = millis(); }
  else if (!strcmp(cmd, "status")) {
    Serial.print(state == BALANCING ? F("balancing") : state == FALLEN ? F("fallen") : F("waiting"));
    Serial.print(F(" angle=")); Serial.print(angle); Serial.print(F(" pwm=")); Serial.print(pwmL);
    Serial.print('/'); Serial.print(pwmR); Serial.print(F(" loop_us=")); Serial.println(loopUs);
  } else {
    for (uint8_t i = 0; i < NREFS; i++)
      if (!strcmp(cmd, refs[i].name) && a1) { *refs[i].p = atof(a1); Serial.print(cmd); Serial.print(F(" = ")); Serial.println(atof(a1), 3); return; }
    Serial.println(F("unknown, type help"));
  }
}

void serialPoll() {
  static char buf[32]; static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { buf[n] = 0; handleLine(buf); n = 0; }
    else if (n < sizeof(buf) - 1) buf[n++] = c;
  }
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  // Timer1 (D9/D10) to 8-bit fast PWM ~976 Hz, same as D5/D6, so both motors behave alike
  TCCR1A = (TCCR1A & ~(_BV(WGM11))) | _BV(WGM10);
  TCCR1B = (TCCR1B & ~(_BV(WGM13))) | _BV(WGM12);
  motorsCoast();
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(4, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(7, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrR, RISING);

  Wire.begin();
  digitalWrite(SDA, LOW); digitalWrite(SCL, LOW);   // no 5 V pull-ups on the 3.3 V bus
  Wire.setClock(400000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
#endif
  oled.setBusClock(400000);
  oled.begin();
  loadGains();
  setupAxes();

  faceMessage("BalanceBot", "starting MPU...");
  mpuWrite(0x6B, 0x80); delay(100);
  mpuWrite(0x6B, 0x01); mpuWrite(0x19, 0x00); mpuWrite(0x1A, 0x03);
  mpuWrite(0x1B, 0x08); mpuWrite(0x1C, 0x08);           // +/-500 dps, +/-4 g
  int16_t raw[7];
  if (!mpuRead(raw)) {
    faceMessage("MPU-6050", "NOT FOUND - check A4/A5");
    Serial.println(F("MPU-6050 not found on A4/A5"));
    while (true) delay(1000);
  }
  faceMessage("Calibrating", "keep me still...");
  while (!calibrateGyro()) { faceMessage("Calibrating", "I moved - keep STILL"); delay(300); }
  Serial.println(F("Ready. Hold me upright. Type help for commands."));
}

void loop() {
  static uint32_t next = micros();
  serialPoll();
  if ((int32_t)(micros() - next) < 0) return;
  if ((int32_t)(micros() - next) > (int32_t)LOOP_US) next = micros();   // fell behind
  next += LOOP_US;
  uint32_t t0 = micros();
  controlStep();
  loopUs = micros() - t0;
  faceTick();
  if (plot && (faceTickDiv & 1)) {
    Serial.print(F("angle:")); Serial.print(angle); Serial.print(F(" target:")); Serial.print(target);
    Serial.print(F(" pwm/10:")); Serial.println(pwmL / 10.0f);
  }
}

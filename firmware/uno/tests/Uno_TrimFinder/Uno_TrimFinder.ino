// TRIM FINDER - motors stay OFF. Serial Monitor 115200.
// Hold the robot on the floor at the point where it feels weightless
// (not falling forward or back). Read "TRIM =" and use it.
#include <Wire.h>

#define IMU_FORWARD     1     // same as in your balancing code
#define IMU_ACCEL_NO_Z  1

const uint8_t MPU = 0x68;
float gyroOff = 0, angle = 0;

void mpuWrite(uint8_t r, uint8_t v) { Wire.beginTransmission(MPU); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
bool mpuRead(float& fwd, float& up, float& pitchRaw) {
  Wire.beginTransmission(MPU); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, (uint8_t)14) != 14) return false;
  int16_t r[7];
  for (uint8_t i = 0; i < 7; i++) { uint8_t h = Wire.read(), l = Wire.read(); r[i] = (int16_t)((h << 8) | l); }
  int8_t s = (IMU_FORWARD == 0 || IMU_FORWARD == 2) ? 1 : -1;
  if (IMU_FORWARD < 2) { fwd = s * r[0] / 8192.0; pitchRaw =  s * r[5]; }
  else                 { fwd = s * r[1] / 8192.0; pitchRaw = -s * r[4]; }
  up = r[2] / 8192.0;
  return true;
}
float accelAngle(float fwd, float up) {
#if IMU_ACCEL_NO_Z
  return asin(constrain(-fwd, -1.0, 1.0)) * RAD_TO_DEG;
#else
  return atan2(-fwd, up) * RAD_TO_DEG;
#endif
}

void setup() {
  pinMode(8, OUTPUT); digitalWrite(8, LOW);          // DRV8833 asleep (EEP on D8)
  const uint8_t pins[4] = {5, 6, 9, 10};
  for (uint8_t p : pins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  Serial.begin(115200);
  Wire.begin(); digitalWrite(SDA, LOW); digitalWrite(SCL, LOW); Wire.setClock(400000);
  mpuWrite(0x6B, 0x80); delay(100);
  mpuWrite(0x6B, 0x01); mpuWrite(0x1A, 0x03); mpuWrite(0x1B, 0x08); mpuWrite(0x1C, 0x08);
  float f, u, p, s = 0; int n = 0;
  Serial.println(F("\nKeep still 1 s (gyro)..."));
  for (int i = 0; i < 300; i++) { if (mpuRead(f, u, p)) { s += p; n++; } delay(3); }
  gyroOff = n ? s / n : 0;
  mpuRead(f, u, p); angle = accelAngle(f, u);
  Serial.println(F("Now hold it at the weightless point. TRIM = average of the last 2 s."));
}

void loop() {
  static uint32_t last = micros(), lastPrint = 0;
  static float avg = 0; static bool init = false;
  float f, u, p;
  if (micros() - last < 5000) return;
  float dt = (micros() - last) / 1e6; last = micros();
  if (!mpuRead(f, u, p)) return;
  float rate = (p - gyroOff) / 65.5;
  angle = 0.98 * (angle + rate * dt) + 0.02 * accelAngle(f, u);
  if (!init) { avg = angle; init = true; }
  avg += 0.0025 * (angle - avg);                     // ~2 s average
  if (millis() - lastPrint > 300) {
    lastPrint = millis();
    Serial.print(F("angle now ")); Serial.print(angle, 2);
    Serial.print(F("    TRIM = ")); Serial.print(avg, 1);
    Serial.println(fabs(rate) < 3 ? F("   (steady - good)") : F("   (moving)"));
  }
}

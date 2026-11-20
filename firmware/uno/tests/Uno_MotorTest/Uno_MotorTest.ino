/*
  BOTH-MOTOR TEST for the Arduino Uno balancing robot (final pin layout)
  ----------------------------------------------------------------------
  WHEELS OFF THE GROUND. Serial Monitor 115200 baud, line ending "Newline".
  Runs an automatic test 3 s after reset, then accepts commands:
    t = run the auto test again     f = both forward    b = both backward
    l = left only    r = right only   + / - = PWM       s = stop
    w = watch encoders (turn wheels by hand), any key stops watching

  FINAL WIRING (same as BOBOT_Uno.ino)
    DRV8833  IN1->D5   IN2->D6   (left motor,  channel A)
             IN3->D9   IN4->D10  (right motor, channel B)
             EEP->5V   GND->GND   VCC/VM->battery +  (or Uno 5V for a weak USB test)
    LEFT  motor: red->OUT1  white->OUT2  black->5V  blue->GND  green->D2  yellow->D4
    RIGHT motor: red->OUT3  white->OUT4  black->5V  blue->GND  green->D3  yellow->D7
    Battery + -> DRV VCC/VM and Uno VIN,  Battery - -> GND
*/
const uint8_t AIN1 = 5, AIN2 = 6;     // left
const uint8_t BIN1 = 9, BIN2 = 10;    // right
const uint8_t ENC_L_A = 2, ENC_L_B = 4;
const uint8_t ENC_R_A = 3, ENC_R_B = 7;

// Set these so "forward" rolls the robot towards its FRONT, then copy them
// into BOBOT_Uno.ino (same names there).
#define MOTOR_L_INVERT false
#define MOTOR_R_INVERT true
#define ENC_L_INVERT   false
#define ENC_R_INVERT   true

const float COUNTS_REV = 350.0f;      // 7 pulses x 50:1, one edge of A

volatile long encL = 0, encR = 0;
void isrL() { if (PIND & _BV(4)) encL--; else encL++; }   // D4
void isrR() { if (PIND & _BV(7)) encR--; else encR++; }   // D7
long cntL() { noInterrupts(); long c = encL; interrupts(); return ENC_L_INVERT ? -c : c; }
long cntR() { noInterrupts(); long c = encR; interrupts(); return ENC_R_INVERT ? -c : c; }

int pwm = 150;

void driveOne(uint8_t in1, uint8_t in2, bool inv, int p) {   // slow decay, + = forward
  if (inv) p = -p;
  p = constrain(p, -255, 255);
  if (p >= 0) { analogWrite(in1, 255); analogWrite(in2, 255 - p); }
  else        { analogWrite(in1, 255 + p); analogWrite(in2, 255); }
}
void motors(int l, int r) { driveOne(AIN1, AIN2, MOTOR_L_INVERT, l); driveOne(BIN1, BIN2, MOTOR_R_INVERT, r); }
void stopAll() { analogWrite(AIN1, 0); analogWrite(AIN2, 0); analogWrite(BIN1, 0); analogWrite(BIN2, 0); }

// run, return rpm of each wheel (+ = forward according to the encoder)
void runMeasure(int l, int r, float& rpmL, float& rpmR) {
  motors(l, r);
  delay(700);
  long l0 = cntL(), r0 = cntR();
  delay(1000);
  long l1 = cntL(), r1 = cntR();
  stopAll();
  delay(400);
  rpmL = (l1 - l0) / COUNTS_REV * 60.0f;
  rpmR = (r1 - r0) / COUNTS_REV * 60.0f;
}

void printRpm(const __FlashStringHelper* what, float a, float b) {
  Serial.print(what); Serial.print(F("  LEFT ")); Serial.print(a, 0);
  Serial.print(F(" rpm   RIGHT ")); Serial.print(b, 0); Serial.println(F(" rpm"));
}

int fails = 0;
void check(bool ok, const __FlashStringHelper* what, const __FlashStringHelper* hint) {
  Serial.print(ok ? F("  [PASS] ") : F("  [FAIL] ")); Serial.println(what);
  if (!ok) { Serial.print(F("         -> ")); Serial.println(hint); fails++; }
}

void autoTest() {
  fails = 0;
  float l, r, l2, r2;
  Serial.println(F("\n============ BOTH MOTORS TEST ============"));
  Serial.println(F("Watch the wheels: forward = robot rolls towards its FRONT.\n"));

  runMeasure(pwm, 0, l, r);   printRpm(F("LEFT only  fwd:"), l, r);
  check(fabs(l) > 30 && fabs(r) < 10, F("left motor + left encoder"),
        F("no left rpm: check D5/D6, OUT1/OUT2, green D2 / yellow D4. Right moved? wires crossed"));
  runMeasure(0, pwm, l, r);   printRpm(F("RIGHT only fwd:"), l, r);
  check(fabs(r) > 30 && fabs(l) < 10, F("right motor + right encoder"),
        F("no right rpm: check D9/D10, OUT3/OUT4, green D3 / yellow D7. Left moved? wires crossed"));

  runMeasure(pwm, pwm, l, r);    printRpm(F("BOTH forward:  "), l, r);
  runMeasure(-pwm, -pwm, l2, r2); printRpm(F("BOTH backward: "), l2, r2);
  check(l > 0 && r > 0, F("encoders count UP when driving forward"),
        F("a negative wheel: flip that ENC_x_INVERT (or swap its green/yellow)"));
  check(l2 < 0 && r2 < 0, F("encoders count DOWN when driving backward"), F("loose encoder wire"));
  float big = max(fabs(l), fabs(r));
  check(big > 0 && fabs(fabs(l) - fabs(r)) < 0.2f * big, F("both wheels within 20 % speed"),
        F("one wheel much slower: stiff gearbox, rubbing wheel or weak wire"));

  // start-up PWM of each motor
  int startL = -1, startR = -1;
  for (int p = 10; p <= 160 && (startL < 0 || startR < 0); p += 5) {
    long a = cntL(), b = cntR();
    motors(startL < 0 ? p : 0, startR < 0 ? p : 0); delay(250);
    if (startL < 0 && labs(cntL() - a) > 3) startL = p;
    if (startR < 0 && labs(cntR() - b) > 3) startR = p;
  }
  stopAll(); delay(300);
  Serial.print(F("Start-up PWM: left ")); Serial.print(startL); Serial.print(F(", right ")); Serial.println(startR);

  Serial.println(F("\nDIRECTION (only you can see this):"));
  Serial.println(F("  During 'BOTH forward' did BOTH wheels roll the robot to its FRONT?"));
  Serial.println(F("  A wheel went backwards -> flip that MOTOR_x_INVERT (and its ENC_x_INVERT)."));
  Serial.println(F("=========================================="));
  Serial.println(fails ? F("Some checks FAILED - see hints above.") : F("ALL CHECKS PASSED"));
  Serial.println(F("Commands: t f b l r + - s w"));
}

void setup() {
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  // D9/D10 (Timer1) to ~976 Hz fast PWM, same as D5/D6, exactly like the final firmware
  TCCR1A = (TCCR1A & ~_BV(WGM11)) | _BV(WGM10);
  TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
  stopAll();
  pinMode(ENC_L_A, INPUT_PULLUP); pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP); pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrR, RISING);
  Serial.println(F("\nMotor test starts in 3 s - WHEELS OFF THE GROUND!"));
  delay(3000);
  autoTest();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  float l, r;
  switch (c) {
    case 't': autoTest(); break;
    case 'f': runMeasure(pwm, pwm, l, r);   printRpm(F("forward: "), l, r); break;
    case 'b': runMeasure(-pwm, -pwm, l, r); printRpm(F("backward:"), l, r); break;
    case 'l': runMeasure(pwm, 0, l, r);     printRpm(F("left:    "), l, r); break;
    case 'r': runMeasure(0, pwm, l, r);     printRpm(F("right:   "), l, r); break;
    case '+': pwm = min(255, pwm + 20); Serial.print(F("PWM = ")); Serial.println(pwm); break;
    case '-': pwm = max(0, pwm - 20);   Serial.print(F("PWM = ")); Serial.println(pwm); break;
    case 's': stopAll(); Serial.println(F("stop")); break;
    case 'w':
      Serial.println(F("Turn the wheels by hand (forward = count up). Any key to stop."));
      delay(50); while (Serial.available()) Serial.read();
      while (!Serial.available()) {
        Serial.print(F("left ")); Serial.print(cntL()); Serial.print(F("   right ")); Serial.println(cntR());
        delay(300);
      }
      while (Serial.available()) Serial.read();
      break;
  }
}

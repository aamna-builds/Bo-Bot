#include <cstdint>
#include <cstdlib>
#include <cstdio>
#define PROGMEM
#define pgm_read_byte(p) (*(const uint8_t*)(p))
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

int main(){
  for (int f=0; f<F_COUNT; f++) for (int an=0; an<2; an++) {
    fs.face=f; fs.eyeDy=0; fs.eyeDx=0; fs.blink=0; fs.mouth=9; fs.anim=an*2;
    printf("F %d %d\n", f, an);
    for (int p=0;p<8;p++){ for(int x=16;x<=111;x++) printf("%d ", faceByte(x,p)); printf("\n"); }
  }
}

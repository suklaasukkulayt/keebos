#include <Arduino.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <string.h>

#define OLED_ADDR   0x3C
#define PIN_SDA     0
#define PIN_SCL     2
#define BTN_MASK    0x1A

#if F_CPU >= 8000000UL
  #define NOPS asm volatile("nop\n\tnop\n\tnop")
#else
  #define NOPS
#endif
#define SDA_L DDRB |=  _BV(PIN_SDA)
#define SDA_H DDRB &= ~_BV(PIN_SDA)
#define SCL_L DDRB |=  _BV(PIN_SCL)
#define SCL_H DDRB &= ~_BV(PIN_SCL)

static void i2cStart() { SDA_H; SCL_H; NOPS; SDA_L; NOPS; SCL_L; }
static void i2cStop()  { SDA_L; NOPS; SCL_H; NOPS; SDA_H; NOPS; }
static void i2cWrite(uint8_t b) {
  for (uint8_t i = 8; i; i--) {
    if (b & 0x80) { SDA_H; } else { SDA_L; }
    b <<= 1; NOPS; SCL_H; NOPS; SCL_L;
  }
  SDA_H; NOPS; SCL_H; NOPS; SCL_L;
}

static const uint8_t OLED_INIT[] PROGMEM = {
  0xAE, 0xD5, 0x80, 0xA8, 0x1F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x02,
  0xDA, 0x02, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF
};

struct Settings { uint8_t magic, map[3], reserved, sleepIdx; int8_t trim; };
struct EE_t { Settings s; };
static EE_t EE EEMEM;
static Settings S;
static uint8_t dirtyS;

static void oledCmd(uint8_t c) { i2cStart(); i2cWrite(OLED_ADDR << 1); i2cWrite(0x00); i2cWrite(c); i2cStop(); }
static void oledSetup() { oledCmd(0xA1); oledCmd(0xC8); }
static void oledInit() {
  i2cStart(); i2cWrite(OLED_ADDR << 1); i2cWrite(0x00);
  for (uint8_t i = 0; i < sizeof OLED_INIT; i++) i2cWrite(pgm_read_byte(&OLED_INIT[i]));
  i2cStop();
  oledSetup();
}

static uint8_t buf[128], pg;
static int8_t py;
enum { M_SET, M_CLR, M_INV };

static void sendPage() {
#ifdef HOST_TEST
  extern void host_page(uint8_t, const uint8_t*); host_page(pg, buf); return;
#endif
  i2cStart(); i2cWrite(OLED_ADDR << 1); i2cWrite(0x00);
  i2cWrite(0xB0 | pg); i2cWrite(0x00); i2cWrite(0x10); i2cStop();
  i2cStart(); i2cWrite(OLED_ADDR << 1); i2cWrite(0x40);
  for (uint8_t i = 0; i < 128; i++) i2cWrite(buf[i]);
  i2cStop();
}

static void rect(int8_t x, int8_t y, uint8_t w, uint8_t h, uint8_t m = M_SET) {
  int8_t lo = y > py ? y : py, hi = y + h < py + 8 ? y + h : py + 8;
  if (lo >= hi) return;
  uint8_t mask = (uint8_t)((0xFF >> (8 - (hi - lo))) << (lo - py));
  if (x < 0) { if (w <= -x) return; w += x; x = 0; }
  if (x + w > 128) w = 128 - x;
  for (uint8_t i = 0; i < w; i++) {
    if (m == M_SET) buf[(uint8_t)x + i] |= mask; else if (m == M_CLR) buf[(uint8_t)x + i] &= ~mask; else buf[(uint8_t)x + i] ^= mask;
  }
}

static void spr(int8_t x, int8_t y, const uint8_t* p) {
  int8_t dy = y - py; if (dy <= -8 || dy >= 8) return;
  for (uint8_t i = 0; i < 8; i++) {
    uint8_t b = pgm_read_byte(p + i);
    buf[(uint8_t)x + i] |= dy >= 0 ? (uint8_t)(b << dy) : (uint8_t)(b >> (-dy));
  }
}

static const uint16_t FONT[] PROGMEM = {
  0x0000, 0x02E0, 0x4C99, 0x11C4, 0x1084, 0x0200, 0x0C98, 0x7E3F,
  0x43F2, 0x5EBD, 0x7EB5, 0x7C87, 0x76B7, 0x76BF, 0x0FA1, 0x7EBF,
  0x7EB7, 0x0140, 0x78BE, 0x2ABF, 0x462E, 0x3A3F, 0x46BF, 0x04BF,
  0x762E, 0x7C9F, 0x47F1, 0x3E08, 0x6C9F, 0x421F, 0x7CDF, 0x7DDF,
  0x7E3F, 0x08BF, 0x7D2F, 0x68BF, 0x26B2, 0x07E1, 0x7E1F, 0x3E0F,
  0x7D9F, 0x6C9B, 0x0F83, 0x4EB9
};

static uint8_t tx, tr, ts = 1;
static void at(uint8_t x, uint8_t r, uint8_t s = 1) { tx = x; tr = r; ts = s; }
static void pc(char c) {
  uint8_t u = c, ix = 0;
  if (u >= '0' && u <= '9') ix = u - 41;
  else if (u >= 'A' && u <= 'Z') ix = u - 47;
  else switch (u) {
    case '!': ix = 1; break;
    case '%': ix = 2; break;
    case '+': ix = 3; break;
    case '-': ix = 4; break;
    case '.': ix = 5; break;
    case '/': ix = 6; break;
    case ':': ix = 17; break;
  }
  if (pg == tr || (ts == 2 && pg == tr + 1)) {
    uint16_t g = pgm_read_word(FONT + ix);
    for (uint8_t i = 0; i < 3; i++, g >>= 5) {
      uint8_t b = g & 31;
      if (ts == 2) {
        uint16_t v = 0;
        for (uint8_t k = 0; k < 5; k++) if (b & (1 << k)) v |= 3 << (2 * k);
        b = pg == tr ? v : v >> 8;
      }
      for (uint8_t r = 0; r < ts; r++) { uint8_t cx = tx + i * ts + r; if (cx < 128) buf[cx] |= b; }
    }
  }
  tx += 4 * ts;
}
static void ps(const char* s) { char c; while ((c = pgm_read_byte(s++))) pc(c); }
static void psc(const char* s, uint8_t r, uint8_t sc = 1) { uint8_t n = 0; while (pgm_read_byte(s + n)) n++; at(64 - 2 * sc * n, r, sc); ps(s); }
static const uint16_t P10[] PROGMEM = { 10000, 1000, 100, 10, 1 };
static void pn(uint16_t v, uint8_t d) {
  uint8_t i = 5 - d;
  if (d < 5) { uint16_t m = pgm_read_word(P10 + i - 1); while (v >= m) v -= m; }
  do {
    uint16_t p = pgm_read_word(P10 + i++);
    uint8_t n = 0;
    while (v >= p) { v -= p; n++; }
    pc('0' + n);
  } while (--d);
}

static uint8_t rs = 1;
static uint8_t rnd() { rs ^= (uint8_t)(rs << 7); rs ^= rs >> 5; rs ^= (uint8_t)(rs << 3); return rs; }
static void pnu(uint16_t v) { uint8_t d = 1; if (v >= 10) { d++; if (v >= 100) { d++; if (v >= 1000) { d++; if (v >= 10000) d++; } } } pn(v, d); }

static uint8_t bm[3];
static uint8_t held, pr, rp, lock, hc;
static uint8_t rawBtn() {
  uint8_t p = ~PINB, r = 0;
  if (p & bm[0]) r |= 1;
  if (p & bm[1]) r |= 2;
  if (p & bm[2]) r |= 4;
  return r;
}
static uint16_t idleS;
static void input() {
  uint8_t r = rawBtn(); lock &= r; r &= ~lock;
  pr = r & ~held; if (r != held) hc = 0; else if (hc < 255) hc++;
  held = r; rp = pr; if (hc >= 8 && !(hc & 1)) rp |= held & 5;
  if (r) idleS = 0;
  if (pr) { rs ^= (uint8_t)millis(); if (!rs) rs = 1; }
}

static uint8_t swRun, swm, sws, swc, lm, ls, lc, lapOn;
static uint8_t tmRun, tmAlarm;
static uint16_t tmRem = 300, tmDur;
static uint16_t l16, acc, swa;
static void bg() {
  uint16_t n = millis(), d = n - l16; l16 = n;
  if (swRun) {
    swa += d;
    while (swa >= 10) { swa -= 10; if (++swc >= 100) { swc = 0; if (++sws >= 60) { sws = 0; swm++; } } }
  }
  acc += d;
  uint16_t th = 1000 - S.trim;
  while (acc >= th) {
    acc -= th; idleS++;
    if (tmRun && !--tmRem) { tmRun = 0; tmAlarm = 1; }
  }
}

static uint8_t fr, over, ovT;
static uint16_t score;
static void gStart() { over = 0; score = 0; }
static void gOver() { over = 1; ovT = 0; }
static uint8_t ovGo() { if (ovT < 255) ovT++; return ovT > 18 && pr; }
static void ovDraw() {
  rect(14, 0, 100, 32, M_CLR); rect(14, 0, 1, 32); rect(113, 0, 1, 32);
  psc(PSTR("GAME OVER"), 1); at(42, 2); ps(PSTR("SCORE ")); pnu(score);
}

static union {
  struct { uint8_t head, len, dir, pend, food, t, spd, x0, seg[64]; } sn;
  struct { int8_t jy, jv, ox[2]; uint8_t oh[2], spd, nx; uint16_t f; } dn;
  struct { int8_t ox[4]; uint8_t ol[4], lane, spd, nx, offs; uint16_t f; } aj;
  struct { uint8_t seq[8], len, pos, phase, tick; } mm;
  struct { uint16_t mv; uint16_t last; } ak;
} G;

static uint8_t snFree(uint8_t p) {
  auto& s = G.sn;
  for (uint8_t i = 0; i < s.len; i++) if (s.seg[(s.head - i) & 63] == p) return 0;
  return 1;
}
static void snFood() { uint8_t p; do { p = (rnd() & 31) | ((rnd() & 7) << 5); } while (!snFree(p)); G.sn.food = p; }
static void snInit() {
  auto& s = G.sn; gStart(); memset(&s, 0, sizeof s);
  s.len = 3; s.head = 2; s.seg[0] = (4 << 5) | 12; s.seg[1] = (4 << 5) | 13; s.seg[2] = (4 << 5) | 14;
  s.spd = 7; snFood();
}
static void snStep() {
  auto& s = G.sn;
  if (over) { if (ovGo()) snInit(); return; }
  if (!s.pend) { if (pr & 1) s.pend = 3; else if (pr & 4) s.pend = 1; }
  if (++s.t < s.spd) return;
  s.t = 0; s.dir = (s.dir + s.pend) & 3; s.pend = 0;
  uint8_t h = s.seg[s.head], x = h & 31, y = h >> 5;
  switch (s.dir) { case 0: x++; break; case 1: y++; break; case 2: x--; break; default: y--; }
  uint8_t np = ((y & 7) << 5) | (x & 31), eat = (np == s.food);
  for (uint8_t i = 0; i < s.len - !eat; i++)
    if (s.seg[(s.head - i) & 63] == np) { gOver(); return; }
  s.head = (s.head + 1) & 63; s.seg[s.head] = np;
  if (eat) { if (s.len < 63) s.len++; score++; snFood(); if (s.spd > 3 && !(score & 3)) s.spd--; }
}
static void snDraw() {
  auto& s = G.sn;
  for (uint8_t i = 0; i < s.len; i++) { uint8_t p = s.seg[(s.head - i) & 63]; rect((p & 31) << 2, (p >> 5) << 2, 3, 3); }
  if (fr & 4) rect(((s.food & 31) << 2) + 1, ((s.food >> 5) << 2) + 1, 2, 2);
  if (over) ovDraw();
}

static const uint8_t DINO_A[8] PROGMEM = { 0x18,0x30,0xE0,0x70,0xF8,0x3F,0x05,0x07 };
static const uint8_t DINO_B[8] PROGMEM = { 0x18,0x30,0xE0,0x70,0x78,0xBF,0x05,0x07 };
static void dnInit() { auto& s = G.dn; gStart(); memset(&s, 0, sizeof s); s.ox[0] = s.ox[1] = -128; s.spd = 2; s.nx = 100; }
static void dnStep() {
  auto& s = G.dn;
  if (over) { if (ovGo()) dnInit(); return; }
  s.f++; score = s.f >> 3;
  if (pr && !s.jy) s.jv = 12;
  if (s.jy || s.jv) { s.jy += s.jv; s.jv--; if (s.jy <= 0) s.jy = s.jv = 0; }
  s.spd = 2 + (s.f >> 8); if (s.spd > 4) s.spd = 4;
  for (uint8_t i = 0; i < 2; i++) if (s.ox[i] != -128) {
    s.ox[i] -= s.spd; if (s.ox[i] < -4) s.ox[i] = -128;
    else if ((s.ox[i] <= 18 && s.ox[i] >= 10 && (s.jy >> 2) + 1 < s.oh[i]) || ((s.oh[i] & 1) && s.ox[i] <= 15 && s.ox[i] >= 5 && (s.jy >> 2) + 1 < s.oh[i] - 3)) gOver();
  }
  if (s.nx > s.spd) s.nx -= s.spd;
  else {
    for (uint8_t i = 0; i < 2; i++) if (s.ox[i] == -128) { s.ox[i] = 127; s.oh[i] = 7 + (rnd() & 7); break; }
    s.nx = 72 + (rnd() & 63) + (s.spd << 3);
  }
}
static void dnDraw() {
  auto& s = G.dn;
  rect(0, 31, 128, 1);
  spr(12, 23 - (s.jy >> 2), (s.jy || !(s.f & 4)) ? DINO_A : DINO_B);
  for (uint8_t i = 0; i < 2; i++) if (s.ox[i] != -128) {
    int8_t x = s.ox[i], h = s.oh[i];
    rect(x + 1, 31 - h, 2, h); rect(x, 34 - h, 1, 3); rect(x + 3, 33 - h, 1, 3);
    if (h & 1) { uint8_t h2 = h - 3; rect(x + 5, 31 - h2, 2, h2); rect(x + 4, 34 - h2, 1, 3); rect(x + 7, 33 - h2, 1, 3); }
  }
  at(104, 0); pn(score, 4);
  if (over) ovDraw();
}

static void carDraw(int8_t x, uint8_t lane, uint8_t me) {
  int8_t y = 2 + lane * 11;
  rect(x, y, 14, 6);
  if (me) rect(x + 8, y + 1, 3, 4, M_CLR); else rect(x + 1, y + 1, 12, 4, M_CLR);
  rect(x + 2, y - 1, 3, 1); rect(x + 2, y + 6, 3, 1); rect(x + 9, y - 1, 3, 1); rect(x + 9, y + 6, 3, 1);
}
static void ajPlace(uint8_t lane) {
  auto& s = G.aj;
  for (uint8_t i = 0; i < 4; i++) if (s.ox[i] == -128) { s.ox[i] = 127; s.ol[i] = lane; return; }
}
static void ajInit() { auto& s = G.aj; gStart(); memset(&s, 0, sizeof s); memset(s.ox, 0x80, 4); s.lane = 1; s.spd = 2; s.nx = 80; }
static void ajStep() {
  auto& s = G.aj;
  if (over) { if (ovGo()) ajInit(); return; }
  if ((pr & 1) && s.lane > 0) s.lane--;
  if ((pr & 4) && s.lane < 2) s.lane++;
  s.f++; score = s.f >> 4;
  s.spd = 2 + (s.f >> 8); if (s.spd > 5) s.spd = 5;
  s.offs = (s.offs + s.spd) & 15;
  for (uint8_t i = 0; i < 4; i++) if (s.ox[i] != -128) {
    s.ox[i] -= s.spd; if (s.ox[i] < -14) s.ox[i] = -128;
    else if (s.ox[i] < 22 && s.ox[i] > -6 && s.ol[i] == s.lane) gOver();
  }
  if (s.nx > s.spd) s.nx -= s.spd;
  else {
    uint8_t l = rnd() % 3; ajPlace(l); if (rnd() & 1) ajPlace((l + 1 + (rnd() & 1)) % 3);
    s.nx = 50 + (rnd() & 31) + (s.spd << 3);
  }
}
static void ajDraw() {
  auto& s = G.aj;
  rect(0, 0, 128, 1); rect(0, 31, 128, 1);
  for (uint8_t i = 0; i < 9; i++) { uint8_t x = i * 16 - s.offs; if (x < 128) { rect(x, 10, 6, 1); rect(x, 21, 6, 1); } }
  carDraw(8, s.lane, 1);
  for (uint8_t i = 0; i < 4; i++) if (s.ox[i] != -128) carDraw(s.ox[i], s.ol[i], 0);
  if (over) ovDraw();
}

static void swStep() {
  if (pr & 2) swRun ^= 1;
  if ((pr & 1) && swRun) { lm = swm; ls = sws; lc = swc; lapOn = 1; }
  if ((pr & 4) && !swRun) { swm = sws = swc = swa = lapOn = 0; }
}
static void swDraw() {
  at(32, 0, 2); pn(swm, 2); pc(':'); pn(sws, 2); pc('.'); pn(swc, 2);
  if (lapOn) { at(42, 3); ps(PSTR("LAP ")); pn(lm, 2); pc(':'); pn(ls, 2); pc('.'); pn(lc, 2); }
  else psc(swRun ? PSTR("L:LAP M:STOP") : PSTR("M:START R:RESET"), 3);
}

static void tmInit() { if (!tmRun && !tmAlarm) tmRem = 300; }
static void tmStep() {
  if (tmAlarm) {
    if (pr) { tmAlarm = 0; tmRem = tmDur; }
    return;
  }
  if (!tmRun) {
    if ((rp & 1) && tmRem > 60) tmRem -= 60;
    if ((rp & 4) && tmRem < 5940) tmRem += 60;
  }
  if (pr & 2) { if (!tmRun) tmDur = tmRem; tmRun ^= 1; }
}
static void tmDraw() {
  if (tmAlarm) { psc(PSTR("DONE"), 1, 2); if (fr & 4) rect(0, 0, 128, 32, M_INV); return; }
  at(0, 0); ps(PSTR("TIMER"));
  at(44, 1, 2); pn(tmRem / 60, 2); pc(':'); pn(tmRem % 60, 2);
  psc(tmRun ? PSTR("M: PAUSE") : PSTR("L/R:SET M:START"), 3);
}

static uint8_t diceFace = 1, diceRoll;
static void diceInit() { diceFace = 1; diceRoll = 0; }
static void diceStep() {
  if (pr & 2) diceRoll = 8;
  if (diceRoll) { diceFace = 1 + rnd() % 6; diceRoll--; }
}
static void diceDraw() {
  psc(PSTR("DICE"), 0);
  at(60, 1, 2); pn(diceFace, 1);
  psc(PSTR("M: ROLL"), 3);
}

static uint16_t readAdc() {
  PRR &= ~_BV(PRADC);
  ADMUX = _BV(MUX3) | _BV(MUX2);
#if F_CPU >= 8000000UL
  ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1);
#else
  ADCSRA = _BV(ADEN) | _BV(ADPS1) | _BV(ADPS0);
#endif
  uint16_t t = millis(); while ((uint16_t)millis() - t < 2) ;
  uint16_t v = 0;
  for (uint8_t i = 0; i < 2; i++) { ADCSRA |= _BV(ADSC); while (ADCSRA & _BV(ADSC)) ; v = ADC; }
  ADCSRA = 0; PRR |= _BV(PRADC);
  return v;
}
static void akInit() { G.ak.mv = readAdc(); G.ak.last = millis(); }
static void akStep() { if ((uint16_t)millis() - G.ak.last > 700) akInit(); }
static void akDraw() {
  uint16_t v = G.ak.mv;
  uint8_t pct = v <= 375 ? 100 : v >= 563 ? 0 : (563 - v) * 100 / 188;
  uint8_t d = v >> 2;
  uint8_t volt = 0;
  if (d) { uint16_t n = 0, limit = 2813U + (d >> 1); while (n + d <= limit) { n += d; volt++; } }
  at(pct >= 100 ? 48 : pct >= 10 ? 52 : 56, 1, 2); pn(pct, pct >= 100 ? 3 : pct >= 10 ? 2 : 1); pc('%');
  at(56, 3); pc('0' + volt / 10); pc('.'); pc('0' + volt % 10); pc('V');
}

static void mmInit() {
  auto& s = G.mm;
  memset(&s, 0, sizeof s);
  s.len = 2; s.seq[0] = rnd() % 3; s.seq[1] = rnd() % 3;
}
static void mmStep() {
  auto& s = G.mm;
  if (!s.phase) {
    if (++s.tick >= 7) {
      s.tick = 0;
      if (++s.pos >= s.len) { s.pos = 0; s.phase = 1; }
    }
  } else if (s.phase == 1) {
    uint8_t b = pr & 7;
    if (b) {
      if (b != (1 << s.seq[s.pos])) s.phase = 2;
      else if (++s.pos >= s.len) {
        if (s.len == 8) s.phase = 3;
        else { s.seq[s.len++] = rnd() % 3; s.pos = 0; s.phase = 0; }
      }
    }
  } else if (pr & 2) mmInit();
}
static void mmDraw() {
  auto& s = G.mm;
  if (!s.phase) {
    psc(PSTR("REMEMBER"), 0);
    if (s.tick < 4) { at(60, 1, 2); pc(s.seq[s.pos] == 0 ? 'L' : s.seq[s.pos] == 1 ? 'C' : 'R'); }
    at(50, 3); ps(PSTR("LEVEL ")); pn(s.len, 1);
  } else if (s.phase == 1) {
    psc(PSTR("REPEAT"), 0); psc(PSTR("L C R"), 1, 2);
    at(46, 3); ps(PSTR("INPUT")); pn(s.pos + 1, 1); pc('/'); pn(s.len, 1);
  } else if (s.phase == 2) {
    psc(PSTR("WRONG"), 1); psc(PSTR("M: RETRY"), 3);
  } else {
    psc(PSTR("WIN!"), 1); psc(PSTR("M: AGAIN"), 3);
  }
}

static const uint8_t SLP[5] PROGMEM = { 15, 30, 60, 180, 0 };
static uint8_t sp;
static void calibrate();
static void seInit() { sp = 0; }
static void seStep() {
  if (pr & 2) { if (++sp > 2) sp = 0; }
  uint8_t q = sp == 1 ? rp : pr;
  int8_t d = ((q & 4) ? 1 : 0) - ((q & 1) ? 1 : 0);
  if (!d) return;
  int8_t v;
  switch (sp) {
    case 0: v = S.sleepIdx + d; if (v >= 0 && v <= 4) S.sleepIdx = v; break;
    case 1: v = S.trim + d; if (v >= -99 && v <= 99) S.trim = v; break;
    default: calibrate(); return;
  }
  dirtyS = 1;
}
static const char TTL[] PROGMEM = "SLEEP       TIMER ADJUSTBUTTONS     ";
static void seDraw() {
  at(0, 0); for (uint8_t k = 0; k < 12; k++) pc(pgm_read_byte(&TTL[sp * 12 + k]));
  for (uint8_t b = 0; b < 3; b++) { rect(104 + b * 8, 0, 6, 6); if (!(held & (1 << b))) rect(105 + b * 8, 1, 4, 4, M_CLR); }
  at(48, 1, 2);
  if (sp == 0) {
    uint8_t v = pgm_read_byte(&SLP[S.sleepIdx]);
    if (v > 99) pn(v, 3); else if (v) pn(v, 2); else ps(PSTR("OFF"));
  } else if (sp == 1) {
    int8_t a = S.trim < 0 ? -S.trim : S.trim;
    pc(S.trim < 0 ? '-' : '+'); pn(a / 10, 1); pc('.'); pn(a % 10, 1);
  } else ps(PSTR("L/R"));
  psc(PSTR("M:NEXT HOLD:EXIT"), 3);
}

static uint8_t calK;
static const char CALW[] PROGMEM = "LEFT  CENTERRIGHT ";
static void calDraw() {
  psc(PSTR("PRESS BUTTON"), 0);
  at(44, 1, 2); for (uint8_t k = 0; k < 5; k++) pc(pgm_read_byte(&CALW[calK * 6 + k]));
  at(58, 3); pn(calK + 1, 1); ps(PSTR("/3"));
}
static void (*drawF)();
void render() { for (pg = 0; pg < 4; pg++) { py = pg << 3; memset(buf, 0, 128); drawF(); sendPage(); } }
static void saveS() { eeprom_update_block(&S, &EE.s, sizeof S); dirtyS = 0; }
static void calibrate() {
  void (*old)() = drawF; drawF = calDraw;
  uint8_t used = 0;
  for (calK = 0; calK < 3; calK++) {
    for (;;) {
      render();
      uint8_t m = (~PINB) & BTN_MASK & ~used;
      if (m && !(m & (m - 1))) {
        uint8_t b = 0; while (!((m >> b) & 1)) b++;
        S.map[calK] = b; used |= m; break;
      }
    }
    while ((~PINB) & BTN_MASK) ;
    uint16_t t = millis(); while ((uint16_t)millis() - t < 150) ;
  }
  for (uint8_t i = 0; i < 3; i++) bm[i] = _BV(S.map[i]);
  S.magic = 0xA8; saveS(); lock = 7; drawF = old;
}

#define NAPPS 9
static const char NAMES[] PROGMEM =
  "SNAKE       " "DINO        " "RACE        " "STOPWATCH   "
  "TIMER       " "DICE        " "MEMORY      " "BATTERY     "
  "SETTINGS    ";
typedef void (*Fn)();
static void nop() {}
static const Fn A_INIT[NAPPS] PROGMEM = { snInit, dnInit, ajInit, nop, tmInit, diceInit, mmInit, akInit, seInit };
static const Fn A_STEP[NAPPS] PROGMEM = { snStep, dnStep, ajStep, swStep, tmStep, diceStep, mmStep, akStep, seStep };
static const Fn A_DRAW[NAPPS] PROGMEM = { snDraw, dnDraw, ajDraw, swDraw, tmDraw, diceDraw, mmDraw, akDraw, seDraw };
static const uint8_t A_FRM[NAPPS] PROGMEM = { 25, 25, 25, 40, 50, 50, 50, 50, 50 };

#define MENU 255
static uint8_t cur = MENU, sel;
static void menuStep() {
  if (pr & 1) sel = sel ? sel - 1 : NAPPS - 1;
  if (pr & 4) sel = sel < NAPPS - 1 ? sel + 1 : 0;
  if (pr & 2) { cur = sel; ((Fn)pgm_read_word(&A_INIT[cur]))(); }
}
static void menuDraw() {
  uint8_t first = sel <= 1 ? 0 : (sel >= NAPPS - 2 ? NAPPS - 4 : sel - 1);
  uint8_t i = first + pg;
  at(6, pg);
  for (uint8_t k = 0; k < 12; k++) pc(pgm_read_byte(&NAMES[i * 12 + k]));
  if (i == sel) rect(0, pg * 8, 122, 8, M_INV);
  rect(126, 0, 1, 32); rect(124, (uint8_t)(sel * 24 / (NAPPS - 1)), 5, 8);
}
static uint16_t mhT;
static void goMenu() {
  cur = MENU; lock |= 2; swRun = 0; tmRun = 0; tmAlarm = 0; idleS = 0; mhT = 0;
  if (dirtyS) saveS();
}

ISR(PCINT0_vect) {}
static void doSleep() {
  if (dirtyS) saveS();
  oledCmd(0x8D); oledCmd(0x10); oledCmd(0xAE);
  PCMSK = bm[0] | bm[1] | bm[2]; GIMSK |= _BV(PCIE);
  for (;;) {
    cli(); GIFR = _BV(PCIF);
    if (!rawBtn()) {
      set_sleep_mode(SLEEP_MODE_PWR_DOWN);
      sleep_enable();
#ifdef sleep_bod_disable
      sleep_bod_disable();
#endif
      sei(); sleep_cpu(); sleep_disable();
    } else sei();
    if (rawBtn()) {
      uint16_t t = millis(); uint8_t released = 0;
      while ((uint16_t)(millis() - t) < 2000 && released < 4) {
        if (rawBtn()) released = 0; else released++;
        delay(10);
      }
      if (released < 4) break;
    }
  }
  GIMSK &= ~_BV(PCIE);
  oledInit();
  lock |= rawBtn(); idleS = 0;
}

static uint8_t lastF;

void setup() {
  PORTB = (PORTB & ~(_BV(PIN_SDA) | _BV(PIN_SCL))) | BTN_MASK;
  DDRB &= ~BTN_MASK; ADCSRA = 0; PRR |= _BV(PRADC);
  eeprom_read_block(&S, &EE.s, sizeof S);
  rs = (uint8_t)(millis() | 1);
  uint8_t old = S.magic == 0xA7;
  uint8_t fresh = !old && S.magic != 0xA8;
  if (fresh) {
    S.magic = 0xA8; S.reserved = 0; S.sleepIdx = 0; S.trim = 0;
    S.map[0] = 4; S.map[1] = 3; S.map[2] = 1;
  } else if (old) {
    S.magic = 0xA8; S.reserved = 0; S.sleepIdx = 0; saveS();
  }
  for (uint8_t i = 0; i < 3; i++) bm[i] = _BV(S.map[i] & 7);
  oledInit();
  if (fresh) calibrate();
  drawF = menuDraw;
  l16 = millis(); lastF = l16;
}

void loop() {
  bg();
  uint8_t fm = cur == MENU ? 50 : pgm_read_byte(&A_FRM[cur]);
  if ((uint8_t)(millis() - lastF) < fm) {
    if (TIMSK & _BV(TOIE0)) { set_sleep_mode(SLEEP_MODE_IDLE); sleep_mode(); }
    return;
  }
  lastF = millis();
  input();
  if (tmAlarm && cur != 4) { cur = 4; idleS = 0; }
  if (held & 2) mhT += fm; else mhT = 0;
  if (cur != MENU && mhT >= 700) goMenu();
  fr++;
  if (cur == MENU) { menuStep(); drawF = menuDraw; }
  else { ((Fn)pgm_read_word(&A_STEP[cur]))(); drawF = (Fn)pgm_read_word(&A_DRAW[cur]); }
  render();
  uint8_t sl = pgm_read_byte(&SLP[S.sleepIdx]);
  if (cur == MENU && sl && idleS >= sl) doSleep();
}
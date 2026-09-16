#include "display_1px.h"
#include "config.h"
#include "hw.h"
#include "settings.h"
#include <math.h>

// สถานะการวาด — เก็บใน RTC memory จึงอยู่รอดข้าม deep sleep
RTC_DATA_ATTR static int  s_shownHour  = -1;
RTC_DATA_ATTR static int  s_shownMin   = -1;
RTC_DATA_ATTR static int  s_shownFace  = -1;
RTC_DATA_ATTR static bool s_shownValid = false;
RTC_DATA_ATTR static bool s_recDot     = false;

// ───────── อนาล็อก ─────────
static const int CX = LCD_W / 2;
static const int CY = LCD_H / 2;
static const int R  = 195;

static void polar(float deg, int len, int &x, int &y) {
  float a = (deg - 90.0f) * (float)DEG_TO_RAD;
  x = CX + (int)lroundf(cosf(a) * len);
  y = CY + (int)lroundf(sinf(a) * len);
}

static void drawTicks(uint16_t c) {
  for (int i = 0; i < 12; i++) {
    int x1, y1, x2, y2;
    polar(i * 30.0f, R - 12, x1, y1);
    polar(i * 30.0f, R,      x2, y2);
    gfx->drawLine(x1, y1, x2, y2, c);
  }
}

static void drawHands(int hour, int minute, uint16_t c) {
  int x, y;
  // เข็มชั่วโมง — สั้น
  polar((hour % 12 + minute / 60.0f) * 30.0f, R * 55 / 100, x, y);
  gfx->drawLine(CX, CY, x, y, c);
  // เข็มนาที — ยาว
  polar(minute * 6.0f, R - 20, x, y);
  gfx->drawLine(CX, CY, x, y, c);
}

// ───────── ดิจิทัล 7-segment เส้น 1px ─────────
static const int DW = 170, DH = 220, DGAP = 30, DX0 = 20, DY1 = 20, DY2 = 262;
//                              gfedcba
static const uint8_t SEG[10] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F };

static void digitOrigin(int idx, int &x, int &y) {
  // idx 0,1 = ชั่วโมง (แถวบน) ; 2,3 = นาที (แถวล่าง)
  x = DX0 + (idx & 1 ? DW + DGAP : 0);
  y = (idx < 2) ? DY1 : DY2;
}

static void drawDigit(int d, int x, int y, uint16_t c) {
  if (d < 0 || d > 9) return;
  uint8_t s = SEG[d];
  int m = y + DH / 2, r = x + DW - 1, b = y + DH - 1;
  if (s & 0x01) gfx->drawFastHLine(x, y, DW, c);       // a
  if (s & 0x02) gfx->drawFastVLine(r, y, DH / 2, c);   // b
  if (s & 0x04) gfx->drawFastVLine(r, m, DH / 2, c);   // c
  if (s & 0x08) gfx->drawFastHLine(x, b, DW, c);       // d
  if (s & 0x10) gfx->drawFastVLine(x, m, DH / 2, c);   // e
  if (s & 0x20) gfx->drawFastVLine(x, y, DH / 2, c);   // f
  if (s & 0x40) gfx->drawFastHLine(x, m, DW, c);       // g
}

static void digitsOf(int hour, int minute, int *out4) {
  out4[0] = hour / 10;  out4[1] = hour % 10;
  out4[2] = minute / 10; out4[3] = minute % 10;
}

static void drawAllDigits(int hour, int minute, uint16_t c) {
  int d[4]; digitsOf(hour, minute, d);
  for (int i = 0; i < 4; i++) { int x, y; digitOrigin(i, x, y); drawDigit(d[i], x, y, c); }
}

// ───────── จุดแดงบอกสถานะอัดเสียง ─────────
static void paintRecDot() {
  gfx->fillCircle(LCD_W - 18, 18, 6, s_recDot ? RGB565_RED : RGB565_BLACK);
}

void faceSetRecording(bool on) {
  if (s_recDot == on) return;
  s_recDot = on;
  paintRecDot();
}

// ───────── API ─────────
void faceInvalidate() { s_shownValid = false; }

void faceDrawFull(const struct tm &t) {
  gfx->fillScreen(RGB565_BLACK);
  if (settings.faceMode == 0) {
    drawTicks(RGB565_WHITE);
    drawHands(t.tm_hour, t.tm_min, RGB565_WHITE);
  } else {
    drawAllDigits(t.tm_hour, t.tm_min, RGB565_WHITE);
  }
  paintRecDot();
  s_shownHour = t.tm_hour;
  s_shownMin  = t.tm_min;
  s_shownFace = settings.faceMode;
  s_shownValid = true;
}

void faceUpdate(const struct tm &t) {
  if (!s_shownValid || s_shownFace != (int)settings.faceMode) { faceDrawFull(t); return; }
  if (s_shownHour == t.tm_hour && s_shownMin == t.tm_min) return;

  if (settings.faceMode == 0) {
    // อนาล็อก: ลบเข็มเก่า 2 เส้น → วาดเข็มใหม่ 2 เส้น แล้วซ่อมขีดที่อาจโดนลบ
    drawHands(s_shownHour, s_shownMin, RGB565_BLACK);
    drawHands(t.tm_hour, t.tm_min, RGB565_WHITE);
    drawTicks(RGB565_WHITE);
  } else {
    // ดิจิทัล: แตะเฉพาะหลักที่เปลี่ยนจริง
    int oldD[4], newD[4];
    digitsOf(s_shownHour, s_shownMin, oldD);
    digitsOf(t.tm_hour, t.tm_min, newD);
    for (int i = 0; i < 4; i++) {
      if (oldD[i] == newD[i]) continue;
      int x, y; digitOrigin(i, x, y);
      drawDigit(oldD[i], x, y, RGB565_BLACK);
      drawDigit(newD[i], x, y, RGB565_WHITE);
    }
  }
  s_shownHour = t.tm_hour;
  s_shownMin  = t.tm_min;
}

void faceShowToast(const char *msg) {
  int16_t h = 44;
  gfx->fillRect(0, CY - h / 2, LCD_W, h, RGB565_BLACK);
  gfx->drawRect(10, CY - h / 2, LCD_W - 20, h, RGB565_WHITE);
  gfx->setTextColor(RGB565_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(24, CY - 8);
  gfx->print(msg);
  faceInvalidate();     // ครั้งถัดไปให้วาดหน้าปัดใหม่ทั้งจอ
}

#include "touch.h"
#include "config.h"
#include "hw.h"

#define SWIPE_MIN_PX   60     // ระยะขั้นต่ำที่ถือว่า "ปัด"
#define TAP_MAX_PX     25     // ขยับไม่เกินนี้ = แตะเฉย ๆ
#define TAP_MAX_MS     500

static bool s_ok = false;
static bool s_down = false;
static int  s_x0 = 0, s_y0 = 0, s_xLast = 0, s_yLast = 0;
static uint32_t s_t0 = 0;

bool touchOk() { return s_ok; }

bool touchInit() {
  s_ok = i2cPresent(ADDR_TOUCH);
  Serial.println(s_ok ? F("[touch] พบตัวควบคุมจอสัมผัส") : F("[touch] ไม่พบจอสัมผัส"));
  return s_ok;
}

bool touchPressed(int &x, int &y) {
  if (!s_ok) return false;
  uint8_t b[5];
  // 0x02 = จำนวนจุด, 0x03..0x06 = XH XL YH YL
  if (!i2cRead(ADDR_TOUCH, 0x02, b, 5)) return false;
  if ((b[0] & 0x0F) == 0) return false;
  x = ((b[1] & 0x0F) << 8) | b[2];
  y = ((b[3] & 0x0F) << 8) | b[4];
  if (x >= LCD_W || y >= LCD_H) return false;   // ค่าขยะ
  return true;
}

Gesture touchPoll() {
  if (!s_ok) return GST_NONE;

  int x, y;
  bool down = touchPressed(x, y);

  if (down) {
    if (!s_down) {                 // เริ่มแตะ
      s_down = true;
      s_x0 = s_xLast = x;
      s_y0 = s_yLast = y;
      s_t0 = millis();
    } else {
      s_xLast = x; s_yLast = y;
    }
    return GST_NONE;
  }

  if (!s_down) return GST_NONE;    // ไม่มีอะไรค้างอยู่

  // ปล่อยนิ้ว → ตัดสินท่าทาง
  s_down = false;
  int dx = s_xLast - s_x0;
  int dy = s_yLast - s_y0;
  uint32_t dt = millis() - s_t0;

  if (abs(dx) < TAP_MAX_PX && abs(dy) < TAP_MAX_PX)
    return (dt <= TAP_MAX_MS) ? GST_TAP : GST_NONE;

  if (abs(dx) > abs(dy)) {
    if (abs(dx) < SWIPE_MIN_PX) return GST_NONE;
    return dx > 0 ? GST_RIGHT : GST_LEFT;
  } else {
    if (abs(dy) < SWIPE_MIN_PX) return GST_NONE;
    return dy > 0 ? GST_DOWN : GST_UP;
  }
}

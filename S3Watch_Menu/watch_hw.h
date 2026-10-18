// ชั้นแปลฮาร์ดแวร์: ให้โค้ดเมนูของ ES3C28P (จอ 320×240, ILI9341) รันบนนาฬิกา Waveshare ESP32-S3-Touch-AMOLED-2.06
// [ผู้ใช้สั่ง 2026-10-18] "อยากให้นาฬิกามีเมนูเหมือน c28p" — ทำทั้ง 3 เฟสพร้อมกัน
// ขาทั้งหมดอ้างจากโค้ดผู้ผลิต (waveshareteam/ESP32-S3-Touch-AMOLED-2.06: pin_config.h, BSP, ตัวอย่าง 05/07/08)
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include "es8311.h"

// ---------- ขา (ผู้ผลิต) ----------
#define LCD_D0 4
#define LCD_D1 5
#define LCD_D2 6
#define LCD_D3 7
#define LCD_SCLK 11
#define LCD_CS 12
#define LCD_RST 8
#define LCD_W 410
#define LCD_H 502
#define LCD_COL_OFF 22                   // ค่าเดียวกับ S3Watch เดิม
#define I2C_SDA 15
#define I2C_SCL 14
#define I2S_MCLK 16
#define I2S_BCLK 41
#define I2S_WS 45
#define I2S_DOUT 40                      // → ES8311 (ลำโพง)
#define I2S_DIN 42                       // ← ES7210 (ไมค์)
#define PIN_PA 46                        // แอมป์ลำโพง (HIGH = เปิด ตามตัวอย่าง 08_ES8311)
#define PIN_BOOT 0
#define TP_RST 9
#define SD_CLK 2                         // SD_MMC 1 บิต ตามตัวอย่าง 07_LVGL_SD_Test
#define SD_CMD 1
#define SD_D0 3
#define ADDR_PMU 0x34                    // AXP2101
#define ADDR_TOUCH 0x38                  // FT3168
#define ADDR_ES7210 0x40

// จอ CO5300: เขียนช่วงที่กว้างเป็นเลขคี่ (เส้น 1 px / จุด) ไม่ขึ้นจอ — ตัวอย่างผู้ผลิตปัดพิกัดเป็นคู่ (rounder_event_cb ใน 05_LVGL_AXP2101_ADC_Data)
// → วาดลงหน่วยความจำ (canvas ใน PSRAM 412 KB) แล้วส่งขึ้นจอทีละแถวเต็มความกว้าง 410 px (เลขคู่) เฉพาะแถวที่เปลี่ยน
extern Arduino_CO5300 *panel;            // จอจริง (ความสว่าง)
extern Arduino_Canvas *gfx;              // วาดทุกอย่างที่นี่
inline int dirtyY0 = 10000, dirtyY1 = -1;
inline void markDirty(int y0, int y1) { if (y0 < dirtyY0) dirtyY0 = y0; if (y1 > dirtyY1) dirtyY1 = y1; }
inline void markAll() { markDirty(0, LCD_H); }
inline void flushDirty() {               // ส่งแถวที่เปลี่ยนขึ้นจอ (แถวเริ่มและจำนวนแถวเป็นเลขคู่)
  if (dirtyY1 < 0) return;
  int y0 = max(0, dirtyY0) & ~1, y1 = min(LCD_H, dirtyY1 + 1);
  if ((y1 - y0) & 1) y1 = min(LCD_H, y1 + 1);
  if (y1 > y0) panel->draw16bitRGBBitmap(0, y0, gfx->getFramebuffer() + (int32_t)y0 * LCD_W, LCD_W, y1 - y0);
  dirtyY0 = 10000; dirtyY1 = -1;
}

// ---------- สี (ค่าเดียวกับ ILI9341 ของ Adafruit) ----------
#define ILI9341_BLACK 0x0000
#define ILI9341_NAVY 0x000F
#define ILI9341_DARKGREEN 0x03E0
#define ILI9341_DARKCYAN 0x03EF
#define ILI9341_MAROON 0x7800
#define ILI9341_PURPLE 0x780F
#define ILI9341_OLIVE 0x7BE0
#define ILI9341_LIGHTGREY 0xC618
#define ILI9341_DARKGREY 0x7BEF
#define ILI9341_BLUE 0x001F
#define ILI9341_GREEN 0x07E0
#define ILI9341_CYAN 0x07FF
#define ILI9341_RED 0xF800
#define ILI9341_MAGENTA 0xF81F
#define ILI9341_YELLOW 0xFFE0
#define ILI9341_WHITE 0xFFFF
#define ILI9341_ORANGE 0xFD20
#define ILI9341_GREENYELLOW 0xAFE5
#define ILI9341_PINK 0xFC18

// ---------- จอ: แปลงพิกัด 320×240 → พื้นที่ใช้งานบนจอนาฬิกา (เว้นขอบมน) ----------
// x: 0..320 → 21..389 (×1.15) · y: 0..240 → 30..474 (×1.85) · ตัวอักษรกว้าง ×1 สูง ×2 (อ่านง่ายบนจอความละเอียดสูง)
#define TFT_KX 1.15f
#define TFT_OX 21
#define TFT_KY 1.85f
#define TFT_OY 30
inline int SX(int x) { return TFT_OX + (int)lroundf(x * TFT_KX); }
inline int SY(int y) { return TFT_OY + (int)lroundf(y * TFT_KY); }

class TftShim : public Print {
 public:
  void begin() { gfx->setTextWrap(false); }
  void fillScreen(uint16_t c) { gfx->fillScreen(c); markAll(); }
  void fillRect(int x, int y, int w, int h, uint16_t c) { gfx->fillRect(SX(x), SY(y), SX(x + w) - SX(x), SY(y + h) - SY(y), c); markDirty(SY(y), SY(y + h)); }
  void drawRect(int x, int y, int w, int h, uint16_t c) { gfx->drawRect(SX(x), SY(y), SX(x + w) - SX(x), SY(y + h) - SY(y), c); markDirty(SY(y), SY(y + h)); }
  void fillRoundRect(int x, int y, int w, int h, int r, uint16_t c) { gfx->fillRoundRect(SX(x), SY(y), SX(x + w) - SX(x), SY(y + h) - SY(y), r * 3 / 2, c); markDirty(SY(y), SY(y + h)); }
  void drawRoundRect(int x, int y, int w, int h, int r, uint16_t c) { gfx->drawRoundRect(SX(x), SY(y), SX(x + w) - SX(x), SY(y + h) - SY(y), r * 3 / 2, c); markDirty(SY(y), SY(y + h)); }
  void drawFastHLine(int x, int y, int w, uint16_t c) { gfx->drawFastHLine(SX(x), SY(y), SX(x + w) - SX(x), c); markDirty(SY(y), SY(y) + 1); }
  void setTextSize(int s) { size_ = s; gfx->setTextSize(s, s * 2); }
  void setTextColor(uint16_t c) { gfx->setTextColor(c); }
  void setTextColor(uint16_t c, uint16_t bg) { gfx->setTextColor(c, bg); }
  void setCursor(int x, int y) { gfx->setCursor(SX(x), SY(y)); }
  // ขนาดข้อความในหน่วย 320×240 (ใช้จัดกลางปุ่ม)
  void getTextBounds(const char *s, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h) {
    int16_t a, b; uint16_t ww, hh;
    gfx->getTextBounds(s, 0, 0, &a, &b, &ww, &hh);
    *x1 = x; *y1 = y; *w = (uint16_t)(ww / TFT_KX); *h = (uint16_t)(hh / TFT_KY);
  }
  size_t write(uint8_t c) override { int y = gfx->getCursorY(); markDirty(y, y + 16 * size_); return gfx->write(c); }
  using Print::write;
 private:
  int size_ = 1;
};

// ---------- I2C ----------
inline uint8_t i2cRd(uint8_t a, uint8_t r) {
  Wire.beginTransmission(a); Wire.write(r);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  Wire.requestFrom(a, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}
inline bool i2cWr(uint8_t a, uint8_t r, uint8_t v) { Wire.beginTransmission(a); Wire.write(r); Wire.write(v); return Wire.endTransmission() == 0; }

// ---------- AXP2101: แบต + ปุ่ม PWR (เลขรีจิสเตอร์ตาม XPowersLib / ตัวอย่าง 05_LVGL_AXP2101_ADC_Data) ----------
inline void pmuInit() {
  i2cWr(ADDR_PMU, 0x30, i2cRd(ADDR_PMU, 0x30) | 0x01);   // วัดแรงดันแบต
  i2cWr(ADDR_PMU, 0x68, i2cRd(ADDR_PMU, 0x68) | 0x01);   // ตรวจแบต
  i2cWr(ADDR_PMU, 0x18, i2cRd(ADDR_PMU, 0x18) | 0x08);   // เกจ %
  i2cWr(ADDR_PMU, 0x41, i2cRd(ADDR_PMU, 0x41) | 0x08);   // IRQ PKEY_SHORT (bit 11)
  i2cWr(ADDR_PMU, 0x49, 0x08);                           // ล้าง
}
inline bool pwrPressed() {               // กด PWR สั้น ๆ ตั้งแต่ครั้งก่อน? (อ่านแล้วล้าง)
  uint8_t st = i2cRd(ADDR_PMU, 0x49);
  if (st == 0xFF || !(st & 0x08)) return false;
  i2cWr(ADDR_PMU, 0x49, 0x08);
  return true;
}
inline bool battPresent() { return i2cRd(ADDR_PMU, 0x00) & 0x08; }
inline bool battCharging() { return (i2cRd(ADDR_PMU, 0x01) >> 5) == 1; }
inline float batteryVolts() { return battPresent() ? (((i2cRd(ADDR_PMU, 0x34) & 0x1F) << 8) | i2cRd(ADDR_PMU, 0x35)) / 1000.0f : 0; }
inline int batteryPercent(float) { return battPresent() ? i2cRd(ADDR_PMU, 0xA4) : 0; }   // เกจใน AXP2101 (แม่นกว่าเดาจากแรงดัน)

// ---------- ทัช FT3168 (ทะเบียนตาม Arduino_FT3x68.h ของผู้ผลิต: 0x02 FingerNum, 0x03-0x06 X1/Y1) ----------
inline void touchInit() { pinMode(TP_RST, OUTPUT); digitalWrite(TP_RST, LOW); delay(10); digitalWrite(TP_RST, HIGH); delay(300); }
inline bool touchRaw(int &x, int &y) {
  uint8_t d[5];
  Wire.beginTransmission(ADDR_TOUCH); Wire.write(0x02);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom((uint8_t)ADDR_TOUCH, (uint8_t)5) != 5) return false;
  Wire.readBytes(d, 5);
  int n = d[0] & 0x0F;
  if (n == 0 || n > 2) return false;
  x = ((d[1] & 0x0F) << 8) | d[2];
  y = ((d[3] & 0x0F) << 8) | d[4];
  return true;
}
// พิกัดจอนาฬิกา → พิกัด 320×240 ของเมนู
inline bool touchScreen(int &sx, int &sy) {
  int x, y;
  if (!touchRaw(x, y)) return false;
  sx = (int)lroundf((x - TFT_OX) / TFT_KX); sy = (int)lroundf((y - TFT_OY) / TFT_KY);
  return true;
}

// ---------- ไมค์ ES7210 (ลำดับรีจิสเตอร์ตาม esp_codec_dev es7210.c ของ Espressif) ----------
inline void es7Bits(uint8_t r, uint8_t mask, uint8_t v) { i2cWr(ADDR_ES7210, r, (i2cRd(ADDR_ES7210, r) & ~mask) | (v & mask)); }
inline void es7210Gain(int db) {         // 0..36 dB ขั้นละ 3 dB (รหัส 0..12) — MIC1 + MIC2
  uint8_t code = (uint8_t)constrain(db / 3, 0, 12);
  es7Bits(0x43, 0x0F, code); es7Bits(0x44, 0x0F, code);
}
inline bool es7210Init(int gainDb) {
  const uint8_t a = ADDR_ES7210;
  bool ok = i2cWr(a, 0x00, 0xFF);
  ok &= i2cWr(a, 0x00, 0x41);
  i2cWr(a, 0x01, 0x3F);
  i2cWr(a, 0x09, 0x30); i2cWr(a, 0x0A, 0x30);
  i2cWr(a, 0x23, 0x2A); i2cWr(a, 0x22, 0x0A); i2cWr(a, 0x20, 0x0A); i2cWr(a, 0x21, 0x2A);
  es7Bits(0x08, 0x01, 0x00);             // slave
  i2cWr(a, 0x40, 0x43); i2cWr(a, 0x41, 0x70); i2cWr(a, 0x42, 0x70);
  i2cWr(a, 0x07, 0x20); i2cWr(a, 0x02, 0xC1);
  for (int i = 0; i < 4; i++) es7Bits(0x43 + i, 0x10, 0x00);
  i2cWr(a, 0x4B, 0xFF); i2cWr(a, 0x4C, 0xFF);
  es7Bits(0x01, 0x0B, 0x00); i2cWr(a, 0x4B, 0x00);   // MIC1 + MIC2
  es7Bits(0x43, 0x10, 0x10); es7Bits(0x44, 0x10, 0x10);
  i2cWr(a, 0x12, 0x00);                  // ไม่ใช้ TDM
  es7210Gain(gainDb);
  uint8_t off = i2cRd(a, 0x01);
  es7Bits(0x11, 0xE0, 0x60);             // 16 บิต
  es7Bits(0x11, 0x03, 0x00);             // I2S ปกติ
  i2cWr(a, 0x01, off); i2cWr(a, 0x06, 0x00); i2cWr(a, 0x40, 0x43);
  for (int i = 0; i < 4; i++) i2cWr(a, 0x47 + i, 0x08);
  i2cWr(a, 0x40, 0x43); i2cWr(a, 0x00, 0x71); i2cWr(a, 0x00, 0x41);
  return ok;
}
inline void es7210Stop() {               // ตาม es7210_stop()
  for (int i = 0; i < 6; i++) i2cWr(ADDR_ES7210, 0x47 + i, 0xFF);
  i2cWr(ADDR_ES7210, 0x40, 0xC0); i2cWr(ADDR_ES7210, 0x01, 0x7F); i2cWr(ADDR_ES7210, 0x06, 0x07);
}

// ---------- ลำโพง ES8311 (ไดรเวอร์ผู้ผลิต es8311.c · ตัวอย่าง 08_ES8311) ----------
inline bool es8311PlayInit(uint32_t rate) {
  static bool done = false;
  if (done) return true;
  es8311_handle_t h = es8311_create(0, ES8311_ADDRRES_0);
  if (!h) return false;
  const es8311_clock_config_t clk = {false, false, true, (int)(rate * 256), (int)rate};
  if (es8311_init(h, &clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16) != ESP_OK) return false;
  es8311_sample_frequency_config(h, clk.mclk_frequency, clk.sample_frequency);
  es8311_voice_volume_set(h, 85, NULL);   // เท่าตัวอย่างผู้ผลิต
  done = true;
  return true;
}

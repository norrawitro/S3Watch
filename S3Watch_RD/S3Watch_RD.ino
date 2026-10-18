// S3Watch_RD — ระบบเสียง RD0/RD1/RD2 (ย้ายจาก esp32s3/rec_s3) บนนาฬิกา Waveshare ESP32-S3-Touch-AMOLED-2.06
// แผน ข (ผู้ใช้เลือก 2026-10-18): sketch แยก ไม่แตะ S3Watch เดิม
// ขาทั้งหมดอ้างจากโค้ดผู้ผลิต (waveshareteam/ESP32-S3-Touch-AMOLED-2.06: pin_config.h, BSP, ตัวอย่าง 08_ES8311)
// ไมค์: ใช้ไดรเวอร์ ES8311 ของผู้ผลิต (es8311.c) · 32 kHz · MCLK = 256 × fs = 8.192 MHz (มีในตารางของไดรเวอร์)
// แบต: อ่านจาก AXP2101 (เลขรีจิสเตอร์ตาม XPowersLib) — ขา 9 เป็นรีเซ็ตทัช ไม่ใช่ ADC แบต
#include <Wire.h>
#include <ESP_I2S.h>
#include <Arduino_GFX_Library.h>
#include <SensorPCF85063.hpp>
#include <SensorQMI8658.hpp>
#include "esp_sntp.h"
#include "es8311.h"
#include "rd1.h"
#include "rd2_live.h"
#include "batt.h"

// FW_VERSION = YYMMDDHHMM ใส่ตอนคอมไพล์โดย publish_ota.sh (แบบเดียวกับ NRW_OTA ของ ES3C28P)
#ifndef FW_VERSION
#define FW_VERSION 0
#endif
// OTA: ดึงจาก repo ส่วนตัว NRW_OTA โฟลเดอร์ s3watch/ · Wi-Fi + กุญแจอ่านอยู่ใน secrets.h (ไม่ขึ้น GitHub)
#include <WiFi.h>
#if __has_include("secrets.h")
#include "secrets.h"
#endif
#ifndef WIFI_SSID
#define WIFI_SSID ""
#define WIFI_PASS ""
#endif
#ifndef WIFI_SSID2
#define WIFI_SSID2 ""
#define WIFI_PASS2 ""
#endif
#if __has_include("secrets_wkw.h")
#include "secrets_wkw.h"                // WKW key ของนาฬิกา (แยกจาก es3c28p) — ทับค่าใน secrets.h
#endif
#include "wkw.h"                         // ส่งผลวิเคราะห์เสียงเข้า WKW (ต้นแบบ rec_s3/wkw.h) · ไม่มี key = ปิดเอง
#define OTA_MANIFEST "s3watch/manifest.json"
#include "net_ota.h"
#define PIN_BOOT 0                       // ปุ่ม BOOT (ข้างนาฬิกา) กด = UPDATE
// ---- ขา (ผู้ผลิต) ----
#define LCD_D0 4
#define LCD_D1 5
#define LCD_D2 6
#define LCD_D3 7
#define LCD_SCLK 11
#define LCD_CS 12
#define LCD_RST 8
#define LCD_W 410
#define LCD_H 502
#define LCD_COL_OFF 22          // ค่าเดียวกับ S3Watch เดิม
#define I2C_SDA 15
#define I2C_SCL 14
#define I2S_MCLK 16
#define I2S_BCLK 41
#define I2S_WS 45
#define I2S_DOUT 40
#define I2S_DIN 42
#define PIN_PA 46
#define ADDR_PMU 0x34
#define ADDR_TOUCH 0x38                  // FT3168 (pin_config.h / Arduino_FT3x68.h ของผู้ผลิต)
#define TP_RST 9
#define MIC_GAIN ((es8311_mic_gain_t)3)   // เท่าตัวอย่างผู้ผลิต (0–7)

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_D0, LCD_D1, LCD_D2, LCD_D3);
Arduino_CO5300 *gfx = new Arduino_CO5300(bus, LCD_RST, 0, LCD_W, LCD_H, LCD_COL_OFF, 0, 0, 0);
SensorPCF85063 rtc;
SensorQMI8658 imu;                       // IMU 6 แกน + นับก้าว (ตั้งค่าตามตัวอย่าง SensorLib QMI8658_PedometerExample)
static bool imuOk = false, timeOk = false;   // timeOk = ได้เวลาจาก NTP แล้ว (ก่อนนั้นไม่ใส่ ts)
I2SClass i2s;
Rd1 rd1;
Rd2Live rd2;
Wkw wkw;
struct EvAcc { int n, nv; float f0, db; uint8_t clar; uint32_t startSeq; } evAcc;   // สะสมต่อเหตุการณ์ RD2 → ส่ง WKW
static bool rtcOk = false, micOk = false;

// ---------- AXP2101 ----------
static uint8_t pmuRead(uint8_t r) {
  Wire.beginTransmission(ADDR_PMU); Wire.write(r);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  Wire.requestFrom(ADDR_PMU, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}
static void pmuWrite(uint8_t r, uint8_t v) { Wire.beginTransmission(ADDR_PMU); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
static void pmuInit() {
  pmuWrite(0x30, pmuRead(0x30) | 0x01);   // เปิดวัดแรงดันแบต (ADC_CHANNEL_CTRL bit0)
  pmuWrite(0x68, pmuRead(0x68) | 0x01);   // เปิดตรวจแบต (BAT_DET_CTRL bit0)
  pmuWrite(0x18, pmuRead(0x18) | 0x08);   // เปิดเกจวัด % (CHARGE_GAUGE_WDT_CTRL bit3)
  // ปุ่ม PWR: ตัวอย่างผู้ผลิต (05_LVGL_AXP2101_ADC_Data) เปิด IRQ PKEY_SHORT แล้วอ่านสถานะทาง I2C (ไม่มีขา IRQ ต่อ ESP32)
  // PKEY_SHORT = bit 11 → INTEN2 (0x41) bit3 · สถานะ INTSTS2 (0x49) bit3 · ล้างโดยเขียน 1
  pmuWrite(0x41, pmuRead(0x41) | 0x08);
  pmuWrite(0x49, 0x08);
}
static bool pwrPressed() {               // กด PWR สั้น ๆ ตั้งแต่ครั้งก่อน? (อ่านแล้วล้าง)
  uint8_t st = pmuRead(0x49);
  if (st == 0xFF || !(st & 0x08)) return false;
  pmuWrite(0x49, 0x08);
  return true;
}

static Batt readBatt() {
  Batt b{};
  b.present = pmuRead(0x00) & 0x08;
  b.charging = (pmuRead(0x01) >> 5) == 1;
  if (b.present) { b.mv = ((pmuRead(0x34) & 0x1F) << 8) | pmuRead(0x35); b.pct = pmuRead(0xA4); }
  else b.pct = -1;
  return b;
}

// ---------- ไมค์ ----------
// v0.3.1: ไมค์จริงอยู่ที่ ES7210 (เฟิร์มแวร์โรงงานใช้ ES7210 รับเสียง, ES8311 เล่นเสียง)
// ลำดับรีจิสเตอร์ตาม esp_codec_dev/device/es7210/es7210.c (Espressif): open → mic_select(MIC1|MIC2) → gain 30 dB
// → set_fs (16 บิต, I2S ปกติ) → start · โหมด slave (ESP32 ส่ง MCLK/BCLK/WS) → ไม่ต้องตั้งตัวหารนาฬิกา
#define ADDR_ES7210 0x40
static bool es7Write(uint8_t r, uint8_t v) { Wire.beginTransmission(ADDR_ES7210); Wire.write(r); Wire.write(v); return Wire.endTransmission() == 0; }
static uint8_t es7Read(uint8_t r) {
  Wire.beginTransmission(ADDR_ES7210); Wire.write(r);
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom(ADDR_ES7210, 1);
  return Wire.available() ? Wire.read() : 0;
}
static void es7Bits(uint8_t r, uint8_t mask, uint8_t v) { es7Write(r, (es7Read(r) & ~mask) | (v & mask)); }
static bool es7210Init() {
  bool ok = es7Write(0x00, 0xFF);                 // reset
  ok &= es7Write(0x00, 0x41);
  es7Write(0x01, 0x3F);
  es7Write(0x09, 0x30); es7Write(0x0A, 0x30);
  es7Write(0x23, 0x2A); es7Write(0x22, 0x0A); es7Write(0x20, 0x0A); es7Write(0x21, 0x2A);
  es7Bits(0x08, 0x01, 0x00);                      // slave
  es7Write(0x40, 0x43); es7Write(0x41, 0x70); es7Write(0x42, 0x70);
  es7Write(0x07, 0x20); es7Write(0x02, 0xC1);
  for (int i = 0; i < 4; i++) es7Bits(0x43 + i, 0x10, 0x00);   // mic_select: ปิดทุกตัวก่อน
  es7Write(0x4B, 0xFF); es7Write(0x4C, 0xFF);
  es7Bits(0x01, 0x0B, 0x00); es7Write(0x4B, 0x00);             // MIC1 + MIC2
  es7Bits(0x43, 0x10, 0x10); es7Bits(0x44, 0x10, 0x10);
  es7Write(0x12, 0x00);                           // ไม่ใช้ TDM (2 ไมค์ = I2S สเตอริโอ)
  es7Bits(0x43, 0x0F, 0x0A); es7Bits(0x44, 0x0F, 0x0A);        // gain 30 dB
  uint8_t off = es7Read(0x01);
  es7Bits(0x11, 0xE0, 0x60);                      // 16 บิต
  es7Bits(0x11, 0x03, 0x00);                      // รูปแบบ I2S ปกติ
  es7Write(0x01, off); es7Write(0x06, 0x00); es7Write(0x40, 0x43);   // start
  for (int i = 0; i < 4; i++) es7Write(0x47 + i, 0x08);
  es7Write(0x40, 0x43); es7Write(0x00, 0x71); es7Write(0x00, 0x41);
  return ok;
}

static bool micInitEs8311() {   // v0.3.0 — ให้เสียงเป็นศูนย์ล้วนบนนาฬิกาจริง (เก็บไว้อ้างอิง ไม่ได้เรียกใช้)
  es8311_handle_t h = es8311_create(0, ES8311_ADDRRES_0);
  if (!h) return false;
  const es8311_clock_config_t clk = {false, false, true, RD_SAMPLE_RATE * 256, RD_SAMPLE_RATE};
  if (es8311_init(h, &clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16) != ESP_OK) return false;
  if (es8311_sample_frequency_config(h, clk.mclk_frequency, clk.sample_frequency) != ESP_OK) return false;
  if (es8311_microphone_config(h, false) != ESP_OK) return false;
  es8311_microphone_gain_set(h, MIC_GAIN);
  return true;
}

// ---------- จอ ----------
static const char *NOTE[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
static uint8_t shown[RD1_BANDS];
static void drawStatic() {
  gfx->fillScreen(RGB565_BLACK);
  gfx->setTextColor(RGB565_DARKGREY, RGB565_BLACK);
  gfx->setTextSize(2);
  gfx->setCursor(67, 450); gfx->print("RD1 32 bands  0..16 kHz");
  memset(shown, 0, sizeof(shown));
}
// แถบ 32 ช่วง: y 150..450 · ค่า RD1 100..255 → 0..300 px
static void drawBars(const Rd1Frame &f, const Rd2Live &r2) {
  const int x0 = 45, bw = 10, y1 = 435, hmax = 265;   // จอขอบมน: เว้นขอบซ้ายขวา ~45 px, ล่าง ~65 px
  for (int b = 0; b < RD1_BANDS; b++) {
    int h = constrain(((int)f.band[b] - 100) * hmax / 155, 0, hmax);
    int x = x0 + b * bw;
    uint16_t c = r2.ex[b] > 0 ? RGB565_ORANGE : (b >= 8 && b < 28 ? RGB565_CYAN : RGB565_BLUE);
    gfx->fillRect(x, y1 - hmax, bw - 2, hmax - h, RGB565_BLACK);
    gfx->fillRect(x, y1 - h, bw - 2, h, c);
  }
}
static void drawTop(const Rd1Frame &f, const Rd2Live &r2, const Batt &bt) {
  char s[48];
  gfx->setTextSize(4); gfx->setTextColor(RGB565_WHITE, RGB565_BLACK);
  gfx->setCursor(55, 50);
  if (rtcOk) { RTC_DateTime d = rtc.getDateTime(); snprintf(s, sizeof(s), "%02d:%02d", d.getHour(), d.getMinute()); }
  else strcpy(s, "--:--");
  gfx->print(s);
  gfx->setTextSize(2); gfx->setCursor(235, 50);
  if (bt.present) snprintf(s, sizeof(s), "%3d%% %c  ", bt.pct, bt.charging ? '+' : ' ');
  else strcpy(s, "no batt ");
  gfx->print(s);
  gfx->setCursor(235, 72); snprintf(s, sizeof(s), "%4dmV", bt.mv); gfx->print(s);
  gfx->setTextSize(4); gfx->setCursor(55, 110);
  if (f.f0_cents) {
    int m = (f.f0_cents + 50) / 100, c = f.f0_cents - m * 100;
    snprintf(s, sizeof(s), "%-2s%d %+3d  ", NOTE[m % 12], m / 12 - 1, c);
  } else strcpy(s, "--       ");
  gfx->setTextColor(f.f0_cents ? RGB565_GREEN : RGB565_DARKGREY, RGB565_BLACK); gfx->print(s);
  gfx->setTextSize(2); gfx->setCursor(280, 110);
  gfx->setTextColor(r2.inEvent ? RGB565_ORANGE : RGB565_WHITE, RGB565_BLACK);
  if (r2.learning()) snprintf(s, sizeof(s), "learning  ");
  else snprintf(s, sizeof(s), "ev %-5lu", (unsigned long)r2.events);
  gfx->print(s);
  gfx->setCursor(280, 132); snprintf(s, sizeof(s), "%4.1f dB ", r2.score); gfx->print(s);
  gfx->setTextColor(wkw.lastCode == 200 ? RGB565_GREEN : RGB565_DARKGREY, RGB565_BLACK);
  gfx->setCursor(55, 148);
  if (!wkw.enabled()) snprintf(s, sizeof(s), "WKW off              ");
  else snprintf(s, sizeof(s), "%s WKW %lu/%lu (%d)   ", WiFi.status() == WL_CONNECTED ? "WiFi" : "----",
                (unsigned long)wkw.sent, (unsigned long)wkw.failed, wkw.lastCode);
  gfx->print(s);
}

static void selfTest() {
  Rd1Frame f; uint32_t us;
  const float hz[] = {440, 1000, 5000, 12000};
  for (float h : hz) {
    int b = rd1SelfTest(rd1, h, 0.5f, f, us);
    Serial.printf("[selftest] %6.0f Hz -> band %2d (%u-%u Hz) val %u  f0_cents %u  clarity %u  %lu us\n",
                  h, b, RD1_EDGES_HZ[b], RD1_EDGES_HZ[b + 1], f.band[b], f.f0_cents, f.clarity, (unsigned long)us);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.printf("\n[S3Watch_RD] fw %lu  RD1 %d bands @ %d Hz\n", (unsigned long)FW_VERSION, RD1_BANDS, RD_SAMPLE_RATE);
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  pmuInit();
  pinMode(TP_RST, OUTPUT); digitalWrite(TP_RST, LOW); delay(10); digitalWrite(TP_RST, HIGH); delay(60);   // ปลดรีเซ็ตทัช
  rtcOk = rtc.begin(Wire, I2C_SDA, I2C_SCL);
  imuOk = imu.begin(Wire, QMI8658_L_SLAVE_ADDRESS, I2C_SDA, I2C_SCL);
  if (imuOk) {
    imu.configAccelerometer(SensorQMI8658::ACC_RANGE_2G, SensorQMI8658::ACC_ODR_62_5Hz);
    imu.enableAccelerometer();
    imu.configGyroscope(SensorQMI8658::GYR_RANGE_512DPS, SensorQMI8658::GYR_ODR_56_05Hz);
    imu.enableGyroscope();
    imu.configPedometer(50, 200, 100, 200, 20, 10, 0, 4);   // ค่าตามตัวอย่าง SensorLib
    imu.enablePedometer(SensorQMI8658::INTERRUPT_PIN_DISABLE);
  }
  if (!gfx->begin()) Serial.println("[lcd] begin failed");
  gfx->setBrightness(160);
  drawStatic();

  rd1.begin();
  selfTest();

  i2s.setPins(I2S_BCLK, I2S_WS, I2S_DOUT, I2S_DIN, I2S_MCLK);
  if (!i2s.begin(I2S_MODE_STD, RD_SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH))
    Serial.println("[i2s] begin failed");
  micOk = es7210Init();
  pinMode(PIN_PA, OUTPUT); digitalWrite(PIN_PA, LOW);   // ไม่ใช้ลำโพง
  pinMode(PIN_BOOT, INPUT_PULLUP);
  wkw.begin();                           // ไม่มี key = ปิดเอง
  Serial.printf("[mic] es7210 %s  rtc %s  imu %s\n", micOk ? "ok" : "FAIL", rtcOk ? "ok" : "FAIL", imuOk ? "ok" : "FAIL");
}

static int16_t st[RD_FRAME_N * 2], pcm[RD_FRAME_N];
static uint32_t seq = 0;

// ---------- ทัช (FT3168) ----------
// ทะเบียนตามไดรเวอร์ผู้ผลิต Arduino_FT3x68.h: 0x02 = FingerNum (จำนวนนิ้ว)
static bool touched() {
  Wire.beginTransmission(ADDR_TOUCH); Wire.write(0x02);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(ADDR_TOUCH, 1);
  return Wire.available() && (Wire.read() & 0x0F) > 0;
}

// ---------- โหมดประหยัดพลังงาน (ผู้ใช้สั่ง 2026-10-18) ----------
// ไม่แตะจอครบ 5 นาที (หรือกด PWR) → นาฬิกาเข็มเส้น 2 px บนพื้นดำ · ปิด Wi-Fi + ไมค์ · หลับ (light sleep) ทีละ 1 วิ
// ตื่นมาอ่านปุ่ม PWR (AXP2101) · เปลี่ยนนาทีค่อยวาดใหม่ · กด PWR = ตื่น (เริ่มเครื่องใหม่ให้ระบบเสียง/Wi-Fi กลับมาครบ)
#define IDLE_MS (5UL * 60UL * 1000UL)
#define FACE_CX (LCD_W / 2)
#define FACE_CY (LCD_H / 2)
#define FACE_R  150                      // จอขอบมน → เว้นขอบ
static void es7210Stop() {               // ลำดับตาม es7210_stop() ของ Espressif
  for (int i = 0; i < 6; i++) es7Write(0x47 + i, 0xFF);
  es7Write(0x40, 0xC0); es7Write(0x01, 0x7F); es7Write(0x06, 0x07);
}
static void handLine(float deg, int len, uint16_t c) {   // เส้นกว้าง 2 px
  float a = deg * PI / 180.0f;
  int x = FACE_CX + lroundf(len * sinf(a)), y = FACE_CY - lroundf(len * cosf(a));
  gfx->drawLine(FACE_CX, FACE_CY, x, y, c);
  gfx->drawLine(FACE_CX + 1, FACE_CY, x + 1, y, c);
}
static void faceTicks() {
  for (int i = 0; i < 12; i++) {
    float a = i * 30 * PI / 180.0f;
    int in = (i % 3 == 0) ? FACE_R - 16 : FACE_R - 8;
    for (int w = 0; w < 2; w++)
      gfx->drawLine(FACE_CX + w + lroundf(in * sinf(a)), FACE_CY - lroundf(in * cosf(a)),
                    FACE_CX + w + lroundf(FACE_R * sinf(a)), FACE_CY - lroundf(FACE_R * cosf(a)), RGB565_DARKGREY);
  }
}
static void nowHM(int &h, int &m) {
  if (rtcOk) { RTC_DateTime d = rtc.getDateTime(); h = d.getHour(); m = d.getMinute(); return; }
  time_t t = time(nullptr); struct tm tmv; localtime_r(&t, &tmv); h = tmv.tm_hour; m = tmv.tm_min;
}
static void enterSaver() {
  Serial.println("[power] saver: clock only (press PWR to wake)");
  Serial.flush();
  WiFi.disconnect(true); WiFi.mode(WIFI_OFF);
  i2s.end(); es7210Stop();
  gfx->fillScreen(RGB565_BLACK);
  gfx->setBrightness(60);
  faceTicks();
  int lh = -1, lm = -1;
  pwrPressed();                          // ล้างการกดที่ค้างอยู่ (ปุ่มที่พาเข้าโหมดนี้)
  for (;;) {
    int h, m; nowHM(h, m);
    if (m != lm || h != lh) {
      if (lm >= 0) { handLine((lh % 12) * 30 + lm * 0.5f, 85, RGB565_BLACK); handLine(lm * 6, 130, RGB565_BLACK); }   // ลบเข็มเก่า
      faceTicks();
      handLine((h % 12) * 30 + m * 0.5f, 85, RGB565_WHITE);
      handLine(m * 6, 130, RGB565_WHITE);
      lh = h; lm = m;
    }
    if (pwrPressed()) { Serial.println("[power] PWR -> wake"); delay(50); ESP.restart(); }
    esp_sleep_enable_timer_wakeup(1000000ULL);   // หลับ 1 วิ แล้วตื่นมาเช็คปุ่ม/นาที
    esp_light_sleep_start();
  }
}

// ---------- WKW ----------
// ไม่ใส่ "device" (ถ้าใส่ต้องตรงชื่อที่ลงทะเบียน ไม่ตรง = 403) — เซิร์ฟเวอร์รู้อุปกรณ์จาก key อยู่แล้ว
static void isoTime(char *b, size_t n) {
  time_t now = time(nullptr); struct tm tmv; localtime_r(&now, &tmv);
  if (!timeOk || tmv.tm_year + 1900 < 2024) { b[0] = 0; return; }
  strftime(b, n, "%Y-%m-%dT%H:%M:%S+07:00", &tmv);
}
static String soundEventJson(uint32_t endSeq) {
  EvAcc &a = evAcc;
  int n = max(1, a.n);
  float f0 = a.nv ? a.f0 / a.nv : 0;
  char note[8] = "";
  if (f0 > 0) { int m = lroundf(69 + 12 * log2f(f0 / 440.0f)); snprintf(note, sizeof(note), "%s%d", NOTE[m % 12], m / 12 - 1); }
  char ts[32]; isoTime(ts, sizeof(ts));
  char buf[400];
  snprintf(buf, sizeof(buf),
    "{\"src\":\"s3watch\",\"fw\":%lu,\"type\":\"sound_event\"%s%s%s,\"event\":%lu,\"ms\":%lu,"
    "\"feat\":{\"f0_hz\":%.1f,\"note\":\"%s\",\"clarity\":%u,\"db\":%.1f,\"voiced_frames\":%d,\"frames\":%d}}",
    (unsigned long)FW_VERSION, ts[0] ? ",\"ts\":\"" : "", ts, ts[0] ? "\"" : "", (unsigned long)rd2.events,
    (unsigned long)((endSeq - a.startSeq) * 20), f0, note, a.clar, a.db / n, a.nv, a.n);
  return String(buf);
}
static String statusJson(const Batt &bt) {
  char ts[32]; isoTime(ts, sizeof(ts));
  char imuJs[200] = "null";
  uint32_t steps = 0;
  if (imuOk) {
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    imu.getAccelerometer(ax, ay, az); imu.getGyroscope(gx, gy, gz);
    steps = imu.getPedometerCounter();
    snprintf(imuJs, sizeof(imuJs), "{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.1f,\"gy\":%.1f,\"gz\":%.1f,\"temp_c\":%.1f}",
             ax, ay, az, gx, gy, gz, imu.getTemperature_C());
  }
  char buf[560];
  snprintf(buf, sizeof(buf),
    "{\"src\":\"s3watch\",\"fw\":%lu,\"type\":\"status\"%s%s%s,\"batt_v\":%.2f,\"batt_pct\":%d,\"sd_free_mb\":0,\"rssi\":%d,\"uptime_s\":%lu,"
    "\"charging\":%s,\"steps\":%lu,\"imu\":%s}",
    (unsigned long)FW_VERSION, ts[0] ? ",\"ts\":\"" : "", ts, ts[0] ? "\"" : "", bt.mv / 1000.0f, bt.pct,
    (int)WiFi.RSSI(), (unsigned long)(millis() / 1000), bt.charging ? "true" : "false", (unsigned long)steps, imuJs);
  return String(buf);
}
// Wi-Fi เปิดค้างเมื่อมี WKW key — หลุดแล้วต่อใหม่ทุก 30 วิ (สลับวงหลัก/วงสำรอง)
static void wifiKeep() {
  static uint32_t last = 0; static int which = 0; static bool ntp = false;
  if (!wkw.enabled() || !WIFI_SSID[0]) return;
  if (WiFi.status() == WL_CONNECTED) {
    if (!ntp) { configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com"); ntp = true; }
    if (!timeOk && esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED) {   // ได้เวลาจริงแล้ว → ตั้งชิป RTC ด้วย
      timeOk = true;
      time_t now = time(nullptr); struct tm t; localtime_r(&now, &t);
      if (rtcOk) rtc.setDateTime(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
      Serial.println("[time] NTP ok -> RTC set");
    }
    return;
  }
  if (last && millis() - last < 30000) return;
  last = millis();
  static bool mainSeen = true;
  if (!mainSeen && WIFI_SSID2[0]) which = 1;   // สแกนแล้วไม่เห็นวงหลัก → ใช้วงสำรองเลย
  const char *ss = which == 0 || !WIFI_SSID2[0] ? WIFI_SSID : WIFI_SSID2;
  const char *pw = which == 0 || !WIFI_SSID2[0] ? WIFI_PASS : WIFI_PASS2;
  which ^= 1;
  static bool scanned = false;
  if (!scanned) {                        // ครั้งแรก: สแกนดูว่าเห็นวง Wi-Fi ไหม (ไม่พิมพ์ชื่อ/รหัส)
    scanned = true;
    WiFi.mode(WIFI_STA);
    int n = WiFi.scanNetworks();
    int r1 = -999, r2 = -999;
    for (int i = 0; i < n; i++) {
      if (WiFi.SSID(i) == WIFI_SSID) r1 = max(r1, (int)WiFi.RSSI(i));
      if (WIFI_SSID2[0] && WiFi.SSID(i) == WIFI_SSID2) r2 = max(r2, (int)WiFi.RSSI(i));
    }
    Serial.printf("[wifi] scan %d nets main %s (%d dBm) backup %s (%d dBm)\n", n,
                  r1 > -999 ? "seen" : "NOT seen", r1, r2 > -999 ? "seen" : "NOT seen", r2);
    WiFi.scanDelete();
    mainSeen = r1 > -999;
    if (!mainSeen && WIFI_SSID2[0]) { ss = WIFI_SSID2; pw = WIFI_PASS2; }
  }
  Serial.printf("[wifi] try %s (status %d)\n", ss == WIFI_SSID ? "main" : "backup", (int)WiFi.status());
  WiFi.disconnect(true); delay(100);
  WiFi.mode(WIFI_STA); WiFi.begin(ss, pw);   // ไม่รอ — ต่อเสร็จเองเบื้องหลัง
}

// ---------- UPDATE (OTA) ----------
static void otaSay(const char *m) {
  Serial.printf("[ota] %s\n", m);
  gfx->fillRect(45, 160, 320, 40, RGB565_BLACK);
  gfx->setTextSize(2); gfx->setTextColor(RGB565_YELLOW, RGB565_BLACK);
  gfx->setCursor(55, 170); gfx->print(m);
}
static void otaProg(int pct) { char m[24]; snprintf(m, sizeof(m), "download %d%%", pct); otaSay(m); }
static bool wifiTry(const char *ssid, const char *pass) {
  if (!ssid[0]) return false;
  WiFi.mode(WIFI_STA); WiFi.begin(ssid, pass);
  for (int i = 0; i < 150 && WiFi.status() != WL_CONNECTED; i++) delay(100);   // รอสูงสุด 15 วิ
  return WiFi.status() == WL_CONNECTED;
}
static void doUpdate() {
  i2s.end();                             // หยุดอ่านเสียงระหว่างอัปเดต
  otaSay("WiFi connecting...");
  if (!wifiTry(WIFI_SSID, WIFI_PASS) && !wifiTry(WIFI_SSID2, WIFI_PASS2)) otaSay("WiFi failed (secrets.h?)");
  else if (otaPull(otaSay, otaProg)) { delay(800); ESP.restart(); }
  WiFi.disconnect(true); WiFi.mode(WIFI_OFF);
  delay(2500);
  ESP.restart();                         // เริ่มใหม่ให้ระบบเสียงกลับมาปกติ
}

void loop() {
  if (Serial.available()) {
    int c = Serial.read();
    if (c == 't') selfTest();            // พิมพ์ t = ทดสอบ RD1 ในตัวซ้ำ
    if (c == 'u') doUpdate();            // พิมพ์ u = UPDATE
  }
  if (digitalRead(PIN_BOOT) == LOW) doUpdate();   // กดปุ่ม BOOT = UPDATE
  size_t got = i2s.readBytes((char *)st, sizeof(st));
  uint8_t flags = 0;
  if (got < sizeof(st)) { memset((uint8_t *)st + got, 0, sizeof(st) - got); flags |= RD1_FLAG_GAP; }
  double eL = 0, eR = 0;
  for (int i = 0; i < RD_FRAME_N; i++) { pcm[i] = st[2 * i]; eL += (double)st[2 * i] * st[2 * i]; eR += (double)st[2 * i + 1] * st[2 * i + 1]; }
  Rd1Frame f;
  uint32_t us = rd1.compute(pcm, seq++, flags, f);
  bool wasEvent = rd2.inEvent;
  rd2.update(f);
  if (rd2.inEvent && !wasEvent) { memset(&evAcc, 0, sizeof(evAcc)); evAcc.startSeq = f.seq; }
  if (rd2.inEvent) {
    evAcc.n++; evAcc.db += (f.total - RD1_CFG.log_offset) * RD1_CFG.log_step_db;
    if (f.f0_cents) { evAcc.nv++; evAcc.f0 += 440.0f * powf(2.0f, (f.f0_cents / 100.0f - 69) / 12.0f); evAcc.clar = max(evAcc.clar, f.clarity); }
  }
  if (!rd2.inEvent && wasEvent && wkw.enabled()) wkw.queue(soundEventJson(f.seq));
  wifiKeep();

  static uint32_t lastDraw = 0, lastLog = 0;
  static Batt bt = readBatt();
  static uint32_t lastTouch = millis();
  if (millis() - lastDraw >= 100) {
    lastDraw = millis();
    if (touched()) lastTouch = millis();
    if (millis() - lastTouch >= IDLE_MS || pwrPressed()) enterSaver();   // ไม่แตะ 5 นาที หรือกด PWR → โหมดประหยัด
    drawBars(f, rd2);
    drawTop(f, rd2, bt);
  }
  if (millis() - lastLog >= 1000) {
    lastLog = millis();
    bt = readBatt();
    static uint32_t lastSt = 0;          // WKW: สถานะอุปกรณ์ทุก 60 วิ (เมื่อ Wi-Fi ต่ออยู่)
    if (wkw.enabled() && WiFi.status() == WL_CONNECTED && (!lastSt || millis() - lastSt >= 60000)) { lastSt = millis(); wkw.queue(statusJson(bt)); }
    Serial.printf("[rd1] seq %lu  %lu us  rmsL %.0f rmsR %.0f  total %u  f0 %u cl %u  rd2 %s score %.1f ev %lu  batt %d%% %dmV%s  bands:",
                  (unsigned long)f.seq, (unsigned long)us, sqrt(eL / RD_FRAME_N), sqrt(eR / RD_FRAME_N), f.total, f.f0_cents, f.clarity,
                  rd2.learning() ? "learn" : (rd2.inEvent ? "EVENT" : "idle"), rd2.score, (unsigned long)rd2.events,
                  bt.pct, bt.mv, bt.charging ? " chg" : "");
    for (int b = 0; b < RD1_BANDS; b++) Serial.printf(" %u", f.band[b]);
    Serial.println();
  }
}

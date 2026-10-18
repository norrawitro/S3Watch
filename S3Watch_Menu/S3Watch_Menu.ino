// S3Watch_Menu — เมนูแบบ ES3C28P บนนาฬิกา Waveshare ESP32-S3-Touch-AMOLED-2.06 [ผู้ใช้สั่ง 2026-10-18 ทำทั้ง 3 เฟสพร้อมกัน]
// ต้นแบบ: esp32s3/rec_s3/rec_s3.ino (branch claude/es3c28p-experiments) — โค้ดเมนู/เสียงเหมือนเดิม เปลี่ยนเฉพาะชั้นฮาร์ดแวร์ (watch_hw.h)
// หน้าหลักสถานะ + MENU · เมนู 3×4: RECORDER, RD1 LOG, RD2 TEST / PHONEME, META, TABLE / WIFI, UPDATE, SETTING / RESET, NEXT, BACK
// NEXT → MENU 2: REC+META (อัดเสียงลง SD /rm/NNNN/audio.wav + events.csv + ส่ง WKW)
// ปุ่ม BOOT = UPDATE (ดึงโค้ดจาก NRW_OTA) — เช็กต้น loop แยกจากทัช/เมนู: OTA ต้องใช้ได้เสมอ [ผู้ใช้สั่ง: OTA ห้ามพัง]
// ปุ่ม PWR = โหมดประหยัด / ปลุก · ไม่แตะจอ 5 นาที = โหมดประหยัด (นาฬิกาเข็ม)
// คำสั่งทาง USB: r อัด · s หยุด · p เล่น · + - gain · d ส่ง WAV · w Wi-Fi · u UPDATE · t ทดสอบ RD1 · m RD1 LOG · l g ไฟล์ LOG
//                k ตาราง RD3 · T ทดสอบ SD · F ฟอร์แมต SD · z โหมดประหยัดทันที
// ค่าลับ: secrets.h (Wi-Fi + OTA) + secrets_wkw.h (WKW key ของ s3watch) — ไม่ขึ้น GitHub
#include <Wire.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESP_I2S.h>
#include <SD_MMC.h>
#include <Preferences.h>
#include <esp_rom_crc.h>
#include <esp_ota_ops.h>
#include <time.h>
#include "esp_sntp.h"
#include "driver/gpio.h"
#include <SensorPCF85063.hpp>
#include <SensorQMI8658.hpp>
#include "watch_hw.h"                   // ขา + จอ (แปลงพิกัด 320×240) + PMU + ทัช + ES7210 + ES8311
// ค่าลับอยู่ในโฟลเดอร์ S3Watch_RD (ผู้ใช้คัดลอกไว้) — ใช้ร่วมกัน ไม่คัดลอกซ้ำ · มีในโฟลเดอร์นี้ = ใช้ของที่นี่ก่อน
#if __has_include("secrets.h")
#include "secrets.h"
#elif __has_include("C:/Users/u/OneDrive/Desktop/S3watch/S3Watch_RD/secrets.h")
#include "C:/Users/u/OneDrive/Desktop/S3watch/S3Watch_RD/secrets.h"
#endif
#if __has_include("secrets_wkw.h")
#include "secrets_wkw.h"                // WKW key ของนาฬิกา (แยกจาก es3c28p) — ทับค่าใน secrets.h
#elif __has_include("C:/Users/u/OneDrive/Desktop/S3watch/S3Watch_RD/secrets_wkw.h")
#include "C:/Users/u/OneDrive/Desktop/S3watch/S3Watch_RD/secrets_wkw.h"
#endif
#ifndef WIFI_SSID
#define WIFI_SSID ""
#define WIFI_PASS ""
#endif
#ifndef OTA_PASS
#define OTA_PASS ""
#endif
#define OTA_MANIFEST "s3watch/manifest.json"
#include "net_ota.h"
#include "wkw.h"                        // ส่งผลวิเคราะห์เสียงเข้า WKW
#include "rd1.h"                         // RD1 = FFT 32 ช่วง + f0 + ฟอร์แมนต์ ทุก 20 ms
#include "sd_tools.h"                   // USB F (+ YES) = ฟอร์แมต SD · T = ทดสอบเขียน/อ่าน 1 MB
#include "sound_meta.h"                 // Sound Metadata ต่อเหตุการณ์
#include "rd3_table.h"                  // RD3 ตารางเทียบเสียง /rd3/table.csv
#include "rd2_live.h"                   // RD2: พื้นหลัง + ตรวจจับเหตุการณ์
#include "rd1_log.h"                    // RD1 LOG: PSRAM → ไฟล์ข้อความ 1 ไฟล์ต่อวินาที

#define rgbLedWrite(...) ((void)0)        // นาฬิกาไม่มีไฟ RGB
const bool usbDriveOn = false;           // นาฬิกาใช้ USB แบบ CDC ในชิป (ไม่มีโหมดไดรฟ์ SD)

const uint32_t RATE = RD_SAMPLE_RATE;    // RD0: 32 kHz (เดิม 16 kHz — เปลี่ยน 2026-10-17 ให้ RD1 เห็นถึง 16 kHz)
const uint32_t MAX_SEC = 60;
const uint32_t MAX_SAMPLES = RATE * MAX_SEC;   // 3.84 MB ใน PSRAM
const int CHUNK = RD_FRAME_N;            // 20 ms ต่อชิ้น (640 ตัวอย่าง)
const int GAIN_MIN = 0, GAIN_MAX = 36, GAIN_STEP = 6, GAIN_DEFAULT = 30;   // dB (ES7210 PGA ขั้นละ 3 dB · v0.3 ใช้ 30 dB)
const char *OTA_HOST = "s3watch";        // ชื่อบนเครือข่าย (s3watch.local)
const uint32_t WIFI_TIMEOUT_MS = 10000;  // ต่อวงละกี่ ms ก่อนลองวงถัดไป
#ifndef WIFI_SSID2                       // secrets.h รุ่นเก่าไม่มีวงสำรอง
#define WIFI_SSID2 ""
#define WIFI_PASS2 ""
#endif
struct WifiNet { const char *ssid, *pass; };
const WifiNet NETS[] = {{WIFI_SSID, WIFI_PASS}, {WIFI_SSID2, WIFI_PASS2}};   // วงหลัก → วงสำรอง
const int N_NETS = sizeof(NETS) / sizeof(NETS[0]);
int netIdx = 0;                          // วงที่กำลังลอง

I2SClass i2s;
Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_D0, LCD_D1, LCD_D2, LCD_D3);
Arduino_CO5300 *gfx = new Arduino_CO5300(bus, LCD_RST, 0, LCD_W, LCD_H, LCD_COL_OFF, 0, 0, 0);
TftShim tft;                             // วาดด้วยพิกัด 320×240 แบบ C28P
SensorPCF85063 rtc;
SensorQMI8658 imu;                       // IMU 6 แกน + นับก้าว (ค่าตามตัวอย่าง SensorLib QMI8658_PedometerExample)
bool rtcOk = false, imuOk = false, timeOk = false;   // timeOk = ได้เวลาจาก NTP แล้ว (ก่อนนั้นไม่ใส่ ts ใน WKW)
uint8_t brightness = 160;
Preferences prefs;
bool sdOk = false;
int16_t *pcm = nullptr;                  // เสียงโมโนใน PSRAM
uint32_t pcmLen = 0;                     // ตัวอย่างที่อัดได้
uint32_t pos = 0;                        // ตำแหน่งที่กำลังอัด/เล่น
int gainDb = GAIN_DEFAULT;
char lastName[40] = "";

// RD1: 1 เฟรมต่อชิ้น 20 ms → 60 วิ = 3,000 เฟรม × 38 ไบต์ ≈ 114 KB ใน PSRAM
const uint32_t MAX_FRAMES = MAX_SAMPLES / RD_FRAME_N;
Rd1 rd1;
Rd1Frame *rd1buf = nullptr;
uint32_t rd1Len = 0;                     // เฟรมที่ได้ในการอัดล่าสุด
uint32_t chunkSeq = 0;                   // เลขลำดับชิ้นนับจากเปิดเครื่อง
uint32_t rd1SumUs = 0, rd1MaxUs = 0;     // เวลาคำนวณต่อชิ้น (ต้องน้อยกว่า 20 ms มาก)
Rd1Log rd1log;

enum State { IDLE, REC, PLAY, LOG, RD2T, RECM } state = IDLE;   // RECM = REC+META: เสียงดิบลง SD + วิเคราะห์ RD1→RD2→RD3 ไปพร้อมกัน
//   // LOG = โหมด RD1 LOG (ไม่เก็บ WAV) · RD2T = หน้า RD2 TEST ฟังสด
Rd2Live rd2live;
Rd3Table rd3;
Rd3Result rd3last;
Wkw wkw;
// สะสมต่อเหตุการณ์เพื่อส่ง WKW: โฟนีมที่ตรงบ่อยสุด + ค่าเฉลี่ย
struct EvAcc {
  uint16_t cnt[RD3_MAX_ROWS]; float dsum[RD3_MAX_ROWS];
  int n, nv, nf; float f0, F1, F2, F3, cent, hilo, db; uint8_t clar;
  void reset() { memset(this, 0, sizeof(*this)); }
} evAcc;
SoundMetaBuilder smb;                     // รวมเฟรมของเหตุการณ์ RD2 → รหัสเมตาดาต้า
SoundMeta smLast;
bool smHave = false;                        // ผลเทียบล่าสุด (หน้า RD2 TEST)
enum WifiState { W_OFF, W_CONNECTING, W_READY } wifiState = W_OFF;
uint32_t wifiStartedAt = 0;

// ---------- ปุ่มบนจอ (แนวนอน 320x240) ----------
// [ผู้ใช้สั่ง 2026-10-18] หน้าหลัก = แสดงสถานะ + MENU · โปรแกรมอัดเสียงย้ายเข้าเมนู (RECORDER) · RD2 TEST อยู่ในเมนู
//   P_MAIN สถานะ · P_MENU ตาราง 3×3 · P_REC อัดเสียง RD0 (REC STOP PLAY GAIN) · P_SET SETTING · P_RD2 ทดสอบ RD2 เรียลไทม์
enum Page : uint8_t { P_MAIN, P_MENU, P_SET, P_REC, P_RD2, P_SAVER, P_TBL, P_PHO, P_META, P_MENU2, P_RM } page = P_MAIN;   // P_MENU2 = เมนูหน้า 2 · P_RM = REC+META (อัดเสียง + เมตาพร้อมกัน)
//   // P_PHO = ตารางโฟนีมสด · P_META = ค่า Sound Metadata   // P_TBL = แก้ตารางเทียบเสียง   // P_SAVER = พักจอ (นาฬิกา)
Page savedPage = P_MAIN;                 // หน้าที่อยู่ก่อนพักจอ
struct Btn { int x, y, w, h; const char *label; uint16_t color; Page page; };
enum { B_REC, B_STOP, B_PLAY, B_GDN, B_GVAL, B_GUP, B_WIFI, B_UPDATE, B_RESET, B_MENU, B_BACK, B_LOG, B_SET, B_USB, B_SBACK,
       B_RECPG, B_RD2PG, B_RBACK, B_2BACK,
       B_TBLPG, B_TPREV, B_TNEXT, B_TMINUS, B_TPLUS, B_TCLR, B_TACT, B_TSAVE, B_TBACK,
       B_PHOPG, B_METAPG, B_PBACK, B_MBACK,
       B_NEXT, B_RMPG, B_M2BACK, B_RMGO, B_RMBACK, B_COUNT };
bool pendingUpdate = false;              // แตะ UPDATE ตอน Wi-Fi ยังไม่ต่อ → ต่อเสร็จแล้วค่อยเช็ก
const uint32_t RESET_CONFIRM_MS = 3000;  // แตะ RESET ครั้งแรก = SURE? · แตะซ้ำภายในเวลานี้ = รีสตาร์ต
uint32_t resetArmedAt = 0;               // 0 = ยังไม่ได้แตะครั้งแรก
Btn btns[B_COUNT] = {
  {2, 92, 104, 60, "REC", ILI9341_RED, P_REC},
  {108, 92, 104, 60, "STOP", ILI9341_LIGHTGREY, P_REC},
  {214, 92, 104, 60, "PLAY", ILI9341_GREEN, P_REC},
  {2, 156, 104, 38, "GAIN -", ILI9341_CYAN, P_REC},
  {108, 156, 104, 38, "", ILI9341_DARKGREY, P_REC},       // แสดงค่า gain (ไม่ใช่ปุ่ม)
  {214, 156, 104, 38, "GAIN +", ILI9341_CYAN, P_REC},
  {2, 126, 104, 44, "WIFI", ILI9341_BLUE, P_MENU},
  {108, 126, 104, 44, "UPDATE", ILI9341_MAGENTA, P_MENU},
  {2, 174, 104, 44, "RESET", ILI9341_ORANGE, P_MENU},
  {2, 178, 316, 58, "MENU", ILI9341_YELLOW, P_MAIN},
  {214, 174, 104, 44, "BACK", ILI9341_YELLOW, P_MENU},
  {108, 30, 104, 44, "RD1 LOG", ILI9341_GREENYELLOW, P_MENU},
  {214, 126, 104, 44, "SETTING", ILI9341_LIGHTGREY, P_MENU},
  {2, 36, 316, 70, "", ILI9341_CYAN, P_SET},              // USB: SERIAL / DRIVE (แตะเพื่อสลับ)
  {2, 194, 316, 42, "BACK", ILI9341_YELLOW, P_SET},
  {2, 30, 104, 44, "RECORDER", ILI9341_RED, P_MENU},
  {214, 30, 104, 44, "RD2 TEST", ILI9341_CYAN, P_MENU},
  {2, 198, 316, 38, "BACK", ILI9341_YELLOW, P_REC},
  {2, 198, 316, 38, "BACK", ILI9341_YELLOW, P_RD2},
  {214, 78, 104, 44, "TABLE", ILI9341_GREENYELLOW, P_MENU},   // หน้าแก้ตารางเทียบเสียง (RD3)
  {2, 30, 50, 36, "<", ILI9341_CYAN, P_TBL},
  {268, 30, 50, 36, ">", ILI9341_CYAN, P_TBL},
  {2, 164, 76, 34, "-", ILI9341_WHITE, P_TBL},
  {80, 164, 76, 34, "+", ILI9341_WHITE, P_TBL},
  {158, 164, 78, 34, "CLR", ILI9341_ORANGE, P_TBL},
  {238, 164, 80, 34, "", ILI9341_GREEN, P_TBL},             // ON / OFF (ใช้เทียบหรือไม่)
  {2, 202, 156, 36, "SAVE", ILI9341_MAGENTA, P_TBL},
  {162, 202, 156, 36, "BACK", ILI9341_YELLOW, P_TBL},
  {2, 78, 104, 44, "PHONEME", ILI9341_GREEN, P_MENU},       // ตารางโฟนีม ฟังสด (ตรงตัวไหน = พื้นเขียว)
  {108, 78, 104, 44, "META", ILI9341_ORANGE, P_MENU},       // ค่า Sound Metadata (ไม่มีกราฟแท่ง — ดูได้ใน RD2 TEST)
  {2, 208, 316, 30, "BACK", ILI9341_YELLOW, P_PHO},
  {2, 208, 316, 30, "BACK", ILI9341_YELLOW, P_META},
  // [ผู้ใช้ขอ 2026-10-18] ปุ่ม NEXT → เมนูหน้า 2 · โปรแกรม REC+META
  {108, 174, 104, 44, "NEXT", ILI9341_WHITE, P_MENU},
  {2, 30, 104, 44, "REC+META", ILI9341_RED, P_MENU2},
  {214, 174, 104, 44, "BACK", ILI9341_YELLOW, P_MENU2},
  {2, 140, 316, 56, "START", ILI9341_RED, P_RM},           // START / STOP
  {2, 202, 316, 36, "BACK", ILI9341_YELLOW, P_RM},
};
// หน้า TABLE: แถวที่เลือก (โฟนีม) + ช่องที่เลือก (พารามิเตอร์ k × 2 + 0 = lo / 1 = hi)
int tblRow = 0, tblCell = 0;
const int TBL_Y = 69, TBL_LH = 17, TBL_LOX = 70, TBL_HIX = 172, TBL_CW = 96;

// บรรทัดข้อความ (สถานะ/ไฟล์ และ Wi-Fi) อยู่คนละที่ในแต่ละหน้า · -1 = หน้านี้ไม่แสดง
int infoY() { return page == P_REC ? 68 : page == P_SET ? 162 : -1; }
int wifiY() { return page == P_MAIN ? 84 : page == P_REC ? 80 : page == P_SET ? 176 : page == P_MENU ? 226 : -1; }

// ---------- ไมค์ ES7210 (watch_hw.h) ----------
void applyGain() {
  es7210Gain(gainDb);
  prefs.putInt("gain", gainDb);
  Serial.printf("gain %+d dB (ES7210 PGA)\n", gainDb);
}

// ---------- หน้าจอ ----------
void drawBtn(int i, bool active) {
  Btn &b = btns[i];
  if (b.page != page) return;            // ปุ่มของอีกหน้า ไม่ต้องวาด
  uint16_t bg = active ? b.color : ILI9341_BLACK, fg = active ? ILI9341_BLACK : b.color;
  if (i == B_WIFI && active) fg = ILI9341_WHITE;
  tft.fillRoundRect(b.x, b.y, b.w, b.h, 8, bg);
  tft.drawRoundRect(b.x, b.y, b.w, b.h, 8, b.color);
  tft.setTextSize(2); tft.setTextColor(i == B_GVAL ? ILI9341_WHITE : fg);
  char buf[16];
  const char *t = b.label;
  if (i == B_GVAL) { snprintf(buf, sizeof(buf), "%+d dB", gainDb); t = buf; }
  if (i == B_RESET && active) t = "SURE?";
  char bb[24];
  if (i == B_USB) { snprintf(bb, sizeof(bb), "BRIGHT %u", brightness); t = bb; }
  if (i == B_RMGO) t = state == RECM ? "STOP" : "START";
  if (i == B_TACT) t = (rd3.n && rd3.rows[tblRow].active) ? "ON" : "OFF";
  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds(t, 0, 0, &x1, &y1, &w, &h);
  if (w > b.w - 4) { tft.setTextSize(1); tft.getTextBounds(t, 0, 0, &x1, &y1, &w, &h); }   // ชื่อยาว (RECORDER) ย่อ
  tft.setCursor(b.x + (b.w - w) / 2, b.y + (b.h - h) / 2);
  tft.print(t);
}

bool btnActive(int i) {
  return (i == B_REC && state == REC) || (i == B_PLAY && state == PLAY) || (i == B_LOG && state == LOG) || (i == B_RMGO && state == RECM) ||
         (i == B_RESET && resetArmedAt) || (i == B_WIFI && wifiState != W_OFF);
}

void drawButtons() {
  for (int i = 0; i < B_COUNT; i++) drawBtn(i, btnActive(i));
}

void wifiMsg(uint16_t color, const char *msg) {   // ข้อความบนบรรทัด Wi-Fi
  if (wifiY() < 0) return;
  tft.fillRect(0, wifiY() - 2, 320, 12, ILI9341_BLACK);
  tft.setTextSize(1); tft.setTextColor(color); tft.setCursor(4, wifiY());
  tft.print(msg);
}

void drawWifiLine() {                    // บรรทัดสถานะ Wi-Fi
  if (wifiY() < 0) return;
  tft.fillRect(0, wifiY() - 2, 320, 12, ILI9341_BLACK);
  tft.setTextSize(1); tft.setCursor(4, wifiY());
  bool saved = prefs.getBool("wifi", false);
  if (wifiState == W_OFF)             { tft.setTextColor(ILI9341_DARKGREY); tft.printf("WiFi: off%s", saved ? " (saved ON)" : ""); }
  else if (wifiState == W_CONNECTING) {
    tft.setTextColor(ILI9341_YELLOW);
    tft.printf("WiFi: connecting (%s)...", netIdx == 0 ? "main" : "backup");
  }
  else {
    tft.setTextColor(ILI9341_GREEN);
    tft.printf("WiFi: OTA ready  %s  (%s.local)", WiFi.localIP().toString().c_str(), OTA_HOST);
  }
}

// หน้าหลัก: รายการสถานะ (ขนาดตัวอักษร 1 · บรรทัดละ 13 px) — Wi-Fi อยู่บรรทัดที่ wifiY() = 85
void statusLine(int y, uint16_t c, const char *fmt, ...) {
  char buf[64];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  tft.setCursor(4, y); tft.setTextColor(c); tft.print(buf);
}

// ---------- เวลา + แบตเตอรี่ (หน้าหลัก) ----------
// เวลา: ชิป RTC PCF85063 (มีไฟจากแบตผ่าน AXP2101) → ตั้งเวลาระบบตอนเปิดเครื่อง · NTP ตั้ง RTC ใหม่เมื่อ Wi-Fi ต่อได้
// แบต: AXP2101 (แรงดัน + % จากเกจในชิป + กำลังชาร์จ) — watch_hw.h
void drawClockBattery() {                // วาดเฉพาะบรรทัดเวลา + แบต (อัปเดตทุก 1 วิ ไม่กระพริบทั้งหน้า)
  if (page != P_MAIN) return;
  tft.setTextSize(1);
  tft.fillRect(0, 94, 320, 24, ILI9341_BLACK);
  time_t now = time(nullptr);
  struct tm tmv; localtime_r(&now, &tmv);
  if (tmv.tm_year + 1900 >= 2024) statusLine(96, ILI9341_WHITE, "Time: %02d:%02d:%02d   %02d/%02d/%04d", tmv.tm_hour, tmv.tm_min, tmv.tm_sec, tmv.tm_mday, tmv.tm_mon + 1, tmv.tm_year + 1900);
  else statusLine(96, ILI9341_DARKGREY, "Time: not set (needs WiFi)");
  float v = batteryVolts();
  int pc = batteryPercent(v);
  statusLine(108, pc > 40 ? ILI9341_GREEN : pc > 15 ? ILI9341_YELLOW : ILI9341_RED, "Battery: %.2f V  %d%%%s", v, pc, battCharging() ? "  charging" : "");
}

void drawMainStatus() {                  // ตัวอักษรขนาด 1 · บรรทัดละ 12 px · Wi-Fi อยู่บรรทัด wifiY() = 84
  tft.fillRect(0, 70, 320, 106, ILI9341_BLACK);
  tft.setTextSize(1);
  static const char *RR[] = {"?", "power on", "ext", "sw", "PANIC", "int wdt", "task wdt", "wdt", "deep sleep", "BROWNOUT", "sdio"};
  int rr = esp_reset_reason();           // สาเหตุรีสตาร์ตครั้งล่าสุด (BROWNOUT = ไฟตก · PANIC/wdt = โปรแกรมค้าง/หน่วยความจำ)
  statusLine(72, ILI9341_LIGHTGREY, "fw %u  (%s)  reset: %s", (unsigned)FW_VERSION, esp_ota_get_running_partition()->label, rr >= 0 && rr <= 10 ? RR[rr] : "?");
  drawWifiLine();
  drawClockBattery();
  if (sdOk) statusLine(120, ILI9341_GREEN, "SD: OK  %u MB free", (unsigned)((SD_MMC.totalBytes() - SD_MMC.usedBytes()) >> 20));
  else      statusLine(120, ILI9341_ORANGE, "SD: none");
  statusLine(132, ILI9341_LIGHTGREY, "IMU: %s  steps %lu", imuOk ? "ok" : "none", imuOk ? (unsigned long)imu.getPedometerCounter() : 0UL);
  statusLine(144, ILI9341_LIGHTGREY, "Mic gain: %+d dB", gainDb);
  if (state == LOG && rd1log.useSd)
    statusLine(156, ILI9341_GREENYELLOW, "RD1 LOG: ON  %s  PSRAM %u  SD %u", rd1log.dir(), (unsigned)rd1log.filesInPsram(), (unsigned)rd1log.filesOnSd());
  else if (state == LOG) statusLine(156, ILI9341_GREENYELLOW, "RD1 LOG: ON  %s  files %u", rd1log.dir(), (unsigned)rd1log.filesDone());
  else                   statusLine(156, ILI9341_DARKGREY, "RD1 LOG: off");
  if (wkw.enabled()) statusLine(168, wkw.lastCode == 200 ? ILI9341_GREEN : ILI9341_LIGHTGREY, "Last rec %.1f s   WKW sent %u fail %u (%d)",
                                pcmLen / (float)RATE, (unsigned)wkw.sent, (unsigned)wkw.failed, wkw.lastCode);
  else statusLine(168, ILI9341_LIGHTGREY, "Last recording: %.1f s   WKW: off", pcmLen / (float)RATE);
}

void drawStatus() {
  if (page == P_SAVER) return;           // พักจออยู่: ไม่วาดหน้าอื่นทับนาฬิกา
  tft.fillRect(0, 0, 320, 26, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setCursor(4, 6);
  if (page == P_MAIN)      { tft.setTextColor(ILI9341_CYAN);   tft.print("N.tag  S3 Watch"); drawMainStatus(); }
  else if (page == P_MENU) { tft.setTextColor(ILI9341_YELLOW); tft.print("MENU"); }
  else if (page == P_MENU2) { tft.setTextColor(ILI9341_YELLOW); tft.print("MENU 2"); }
  else if (page == P_RM)   { tft.setTextColor(state == RECM ? ILI9341_RED : ILI9341_CYAN); tft.print(state == RECM ? "REC+META  recording" : "REC+META"); }
  else if (page == P_RD2)  { tft.setTextColor(ILI9341_CYAN);   tft.print("RD2 TEST"); }
  else if (page == P_TBL)  { tft.setTextColor(ILI9341_GREENYELLOW); tft.print(rd3.dirty ? "TABLE *unsaved" : "TABLE"); }
  else if (page == P_PHO)  { tft.setTextSize(1); tft.setTextColor(ILI9341_GREEN); tft.setCursor(4, 4); tft.print("PHONEME  (green = matches the sound)"); }
  else if (page == P_META) { tft.setTextColor(ILI9341_ORANGE); tft.print("SOUND META"); }
  else if (page == P_SET) {
    tft.setTextColor(ILI9341_YELLOW); tft.print("SETTING");
    tft.fillRect(0, 110, 320, 48, ILI9341_BLACK);   // คำอธิบายโหมด USB ใต้ปุ่ม
    tft.setTextSize(1); tft.setTextColor(ILI9341_LIGHTGREY);
    tft.setCursor(4, 112); tft.print("BRIGHT: tap to change screen brightness");
    tft.setCursor(4, 124); tft.print("Saver: 5 min no touch or PWR = clock");
    tft.setCursor(4, 136); tft.print("BOOT button = UPDATE (always works)");
    tft.setTextColor(sdOk ? ILI9341_GREEN : ILI9341_ORANGE);
    tft.setCursor(4, 148); tft.print(sdOk ? "SD card: OK" : "SD card: none");
  } else if (page == P_REC) {
    if (state == REC)       { tft.setTextColor(ILI9341_RED);   tft.print("REC"); }
    else if (state == PLAY) { tft.setTextColor(ILI9341_GREEN); tft.print("PLAY"); }
    else                    { tft.setTextColor(ILI9341_CYAN);  tft.print("RECORDER"); }
  }
  if (infoY() < 0) return;
  tft.fillRect(0, infoY() - 2, 236, 12, ILI9341_BLACK);   // บรรทัด SD / ไฟล์ล่าสุด / เวอร์ชัน
  tft.setTextSize(1); tft.setTextColor(sdOk ? ILI9341_GREEN : ILI9341_ORANGE);
  tft.setCursor(4, infoY()); tft.print(sdOk ? "SD: OK" : "SD: none");
  tft.setTextColor(ILI9341_LIGHTGREY);
  tft.printf("  last %.1f s  fw %u", pcmLen / (float)RATE, (uint32_t)FW_VERSION);
}

void drawTime() {                        // เวลาอัด/เล่น มุมขวาบน (หน้า RECORDER)
  if (page != P_REC) return;
  float t = (state == IDLE ? pcmLen : pos) / (float)RATE;
  tft.setTextSize(2); tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setCursor(196, 6);
  tft.printf("%5.1f s ", t);
}

void drawLevel(float db) {               // แถบระดับ −90..0 dBFS (หน้า RECORDER) · หน้าหลักใช้พื้นที่นี้เป็นแถบ UPDATE
  if (page != P_REC) return;
  int w = constrain((int)((db + 90) / 90 * 230), 0, 230);
  tft.fillRect(4, 40, w, 24, db > -20 ? ILI9341_RED : db > -50 ? ILI9341_GREEN : ILI9341_DARKGREEN);
  tft.fillRect(4 + w, 40, 230 - w, 24, ILI9341_DARKGREY);
  tft.setTextSize(2); tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  tft.setCursor(240, 44); tft.printf("%4.0f", db);
  tft.setTextSize(1); tft.setCursor(240, 68); tft.print("dBFS");
}

void deepSaver(); void redrawAll(); void startRd2(); void stopRd2(); void drawRmPage(); void drawRmLive(); void rmAdd(const int16_t *m); void rmEventDone();
extern uint32_t rmSeq0, rmEvStartMs;
bool listenPage(Page p) { return p == P_RD2 || p == P_PHO || p == P_META; }   // หน้าที่ฟังไมค์สด (RD1 → RD2 → RD3)
void stopAll();
void showPage(Page p) {
  if (page == P_RM && p != P_RM && state == RECM) stopAll();   // ออกจากหน้า = หยุดอัด + ปิดไฟล์
  if (listenPage(page) && !listenPage(p)) stopRd2();
  bool start = listenPage(p) && !listenPage(page);
  page = p;
  resetArmedAt = 0;                      // ออกจากหน้าเมนู = ยกเลิก SURE?
  redrawAll();
  if (start) startRd2();
}

// ระหว่างอัด: แท่ง 32 ช่วง RD1 แทนแถบระดับเสียง (กว้าง 7 px สูงตามค่า 0-255)
void drawBands(const Rd1Frame &f, float db) {
  if (page != P_REC) return;
  const int X = 4, Y = 40, H = 24;
  for (int b = 0; b < RD1_BANDS; b++) {
    int h = f.band[b] * H / 255;
    uint16_t c = b < 8 ? ILI9341_ORANGE : b < 28 ? ILI9341_GREEN : ILI9341_CYAN;   // 0–3 kHz · 3–8.7 kHz (โฟกัส) · 8.7–16 kHz
    tft.fillRect(X + b * 7, Y, 6, H - h, ILI9341_DARKGREY);
    tft.fillRect(X + b * 7, Y + H - h, 6, h, c);
  }
  tft.fillRect(X + 224, Y, 6, H, ILI9341_BLACK);
  tft.setTextSize(2); tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  tft.setCursor(240, 44); tft.printf("%4.0f", db);
}

// ---------- หน้า RD2 TEST ----------
// 32 แท่งเรียลไทม์: เทาเข้ม = ระดับพื้นหลังที่เรียนไว้ · สี = ส่วนที่ดังเกินพื้นหลัง (เขียว = มีเสียง)
// ช่องเอาต์พุต: โน้ตจาก f0 (ใช้ได้แล้ว) · อักษร/โฟนีม (รอคลัง RD3)
const int R2_X = 4, R2_Y = 30, R2_H = 112, R2_W = 9;   // 32 × 9 = 288 px
void drawRd2Bars(const Rd1Frame &f, const Rd2Live &r) {
  for (int b = 0; b < RD1_BANDS; b++) {
    int lvl = f.band[b] * R2_H / 255;
    int bgH = r.learning() ? lvl : constrain((int)(r.bg[b] / RD1_CFG.log_step_db) * R2_H / 255, 0, R2_H);
    int x = R2_X + b * R2_W, low = min(lvl, bgH);
    uint16_t c = r.learning() ? ILI9341_DARKGREY : r.ex[b] > 0 ? (r.inEvent ? ILI9341_GREEN : ILI9341_YELLOW) : ILI9341_BLUE;
    tft.fillRect(x, R2_Y, R2_W - 1, R2_H - lvl, ILI9341_BLACK);
    tft.fillRect(x, R2_Y + R2_H - lvl, R2_W - 1, lvl - low, c);          // ส่วนเหนือพื้นหลัง
    tft.fillRect(x, R2_Y + R2_H - low, R2_W - 1, low, 0x3186);           // ส่วนที่เป็นพื้นหลัง (เทาเข้ม)
    tft.drawFastHLine(x, R2_Y + R2_H - bgH, R2_W - 1, ILI9341_LIGHTGREY);   // เส้นระดับพื้นหลัง
  }
}

void drawRd2Status(const Rd1Frame &f, const Rd2Live &r) {
  tft.fillRect(0, 146, 320, 12, ILI9341_BLACK);   // บรรทัดสถานะใต้แท่ง
  tft.setTextSize(1); tft.setCursor(4, 148);
  if (r.learning()) { tft.setTextColor(ILI9341_ORANGE); tft.printf("learning background... keep quiet (%d/%d)", r.learned, r.cfg.learn_frames); }
  else if (!r.inEvent && smHave) {         // หลังจบเหตุการณ์: รหัสเมตาดาต้าที่วัดได้ (ครบทุกตัวทาง USB)
    tft.setTextColor(ILI9341_LIGHTGREY);
    tft.printf("#%u %ums M4.%u M7.%u M8.%u M9.%u M11.%u T3.%u", (unsigned)r.events, (unsigned)smLast.ms,
               smLast.M4, smLast.M7, smLast.M8, smLast.M9, smLast.M11, smLast.T3);
  } else {
    tft.setTextColor(r.inEvent ? ILI9341_GREEN : ILI9341_DARKGREY);
    tft.printf("%s  score %4.1f dB  events %u", r.inEvent ? "SOUND" : "quiet", r.score, (unsigned)r.events);
  }
  tft.drawRect(2, 160, 316, 34, ILI9341_DARKGREY);   // ช่องเอาต์พุต
  tft.fillRect(3, 161, 314, 32, ILI9341_BLACK);
  tft.setTextSize(2); tft.setCursor(8, 163);
  if (f.f0_cents && f.clarity >= 200) {  // [ข้อเสนอ] ความชัด ≥ 200/255 จึงนับเป็นโน้ต
    static const char *N[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    int m = (f.f0_cents + 50) / 100;
    float hz = 440.0f * powf(2.0f, (f.f0_cents / 100.0f - 69) / 12.0f);
    tft.setTextColor(ILI9341_GREEN); tft.printf("NOTE %s%d  %5.1f Hz", N[m % 12], m / 12 - 1, hz);
  } else { tft.setTextColor(ILI9341_DARKGREY); tft.print("NOTE  --"); }
  tft.setTextSize(1); tft.setCursor(200, 163);           // ฟอร์แมนต์ (แยกสระ) มุมขวาของช่อง
  if (f.fmt[0]) { tft.setTextColor(ILI9341_CYAN); tft.printf("F1 %4u  F2 %4u", f.fmt[0], f.fmt[1]); }
  else { tft.setTextColor(ILI9341_DARKGREY); tft.print("F1 --   F2 --"); }
  tft.setTextSize(1); tft.setCursor(8, 183);             // RD3: เทียบกับตาราง /rd3/table.csv
  if (r.inEvent && rd3last.best >= 0) {
    tft.setTextColor(ILI9341_YELLOW);
    tft.printf("LETTER: %s  (d %.2f)", rd3.rows[rd3last.best].show, rd3last.dist);
    if (rd3last.second >= 0) { tft.setTextColor(ILI9341_DARKGREY); tft.printf("   2nd %s %.2f", rd3.rows[rd3last.second].show, rd3last.dist2); }
  } else { tft.setTextColor(ILI9341_DARKGREY); tft.printf("LETTER: --   (table %d rows)", rd3.n); }
}

// ---------- พักจอ: นาฬิกาดิจิตอล [ผู้ใช้สั่ง 2026-10-18] ----------
// ไม่แตะจอ 5 นาที → นาฬิกาเต็มจอ · งานเบื้องหลัง (อัดเสียง / RD1 LOG / RD2 วิเคราะห์เสียง) ทำต่อ ไม่หยุด
// ใช้วิธีสลับ page เป็น P_SAVER: ทุกฟังก์ชันวาดเช็ก page อยู่แล้ว → ไม่วาดทับนาฬิกา แต่งานเสียงยังวิ่ง
// แตะจอครั้งแรก = ปลุกกลับหน้าเดิมเท่านั้น (ไม่กดปุ่มที่อยู่ตรงนั้น)
const uint32_t SAVER_AFTER_MS = 5UL * 60 * 1000;
uint32_t lastTouchMs = 0;

// [ผู้ใช้สั่ง 2026-10-18 สำหรับนาฬิกา] หน้าปัดเข็มเส้นหนา 3 px บนพื้นดำ อัปเดตทุกนาที
// งานเบื้องหลังวิ่งอยู่ (REC / RD1 LOG / REC+META / PLAY) = แสดงนาฬิกาแต่ไม่หลับ แตะจอ = กลับหน้าเดิม
// ว่างอยู่ = ปิด Wi-Fi + ไมค์ แล้วหลับ (light sleep) ทีละ 1 วิ · กด PWR = ตื่น (เริ่มเครื่องใหม่)
#define FACE_CX (LCD_W / 2)
#define FACE_CY (LCD_H / 2)
#define FACE_R  150                      // จอขอบมน → เว้นขอบ
int faceH = -1, faceM = -1;
void handLine(float deg, int len, uint16_t c) {
  float a = deg * PI / 180.0f;
  int x = FACE_CX + lroundf(len * sinf(a)), y = FACE_CY - lroundf(len * cosf(a));
  gfx->drawLine(FACE_CX, FACE_CY, x, y, c);
  gfx->drawLine(FACE_CX + 1, FACE_CY, x + 1, y, c);
  gfx->drawLine(FACE_CX, FACE_CY + 1, x, y + 1, c);
}
void faceTicks() {
  for (int i = 0; i < 12; i++) {
    float a = i * 30 * PI / 180.0f;
    int in = (i % 3 == 0) ? FACE_R - 16 : FACE_R - 8;
    for (int w = 0; w < 3; w++)
      gfx->drawLine(FACE_CX + w + lroundf(in * sinf(a)), FACE_CY - lroundf(in * cosf(a)),
                    FACE_CX + w + lroundf(FACE_R * sinf(a)), FACE_CY - lroundf(FACE_R * cosf(a)), ILI9341_WHITE);
  }
}
void drawSaver(bool full) {              // วาดใหม่เฉพาะเมื่อเปลี่ยนนาที
  time_t now = time(nullptr); struct tm t; localtime_r(&now, &t);
  if (full) { gfx->fillScreen(ILI9341_BLACK); faceH = faceM = -1; }
  if (!full && t.tm_min == faceM && t.tm_hour == faceH) return;
  if (faceM >= 0) { handLine((faceH % 12) * 30 + faceM * 0.5f, 85, ILI9341_BLACK); handLine(faceM * 6, 130, ILI9341_BLACK); }
  faceTicks();
  handLine((t.tm_hour % 12) * 30 + t.tm_min * 0.5f, 85, ILI9341_WHITE);
  handLine(t.tm_min * 6, 130, ILI9341_WHITE);
  faceH = t.tm_hour; faceM = t.tm_min;
}

void saverOn() {                         // งานเบื้องหลังวิ่งอยู่: นาฬิกา (ไม่หลับ)
  if (page == P_SAVER) return;
  savedPage = page; page = P_SAVER;
  Serial.println("screensaver on (job running, no sleep)");
  drawSaver(true);
}

void saverOff() {                        // กลับหน้าเดิม (งานเบื้องหลังไม่ถูกแตะ)
  if (page != P_SAVER) return;
  page = savedPage;
  Serial.println("screensaver off");
  redrawAll();
}

void deepSaver() {                       // ว่าง: หลับจนกด PWR
  Serial.println("[power] saver: clock only, sleep (press PWR to wake)");
  Serial.flush();
  if (state != IDLE) stopAll();
  ArduinoOTA.end(); WiFi.disconnect(true); WiFi.mode(WIFI_OFF);
  i2s.end(); es7210Stop();
  page = P_SAVER;
  gfx->setBrightness(120);
  drawSaver(true);
  pwrPressed();                          // ล้างการกดที่พาเข้าโหมดนี้
  for (;;) {
    drawSaver(false);
    if (pwrPressed()) { Serial.println("[power] PWR -> wake"); delay(50); ESP.restart(); }
    if (digitalRead(PIN_BOOT) == LOW) ESP.restart();   // BOOT ระหว่างหลับ = ตื่น (กดอีกครั้งหลังตื่น = UPDATE)
    // ระหว่างหลับขาที่ไม่ hold จะลอย → ขารีเซ็ตจอลง LOW = จอดับ (พบใน S3Watch_RD fw 2610182015)
    const gpio_num_t keep[] = {(gpio_num_t)LCD_RST, (gpio_num_t)LCD_CS, (gpio_num_t)TP_RST, (gpio_num_t)PIN_PA};
    for (gpio_num_t g : keep) { gpio_sleep_sel_dis(g); gpio_hold_en(g); }
    esp_sleep_enable_timer_wakeup(1000000ULL);
    esp_light_sleep_start();
    for (gpio_num_t g : keep) gpio_hold_dis(g);
  }
}

// ---------- หน้า TABLE: แก้ตารางเทียบเสียงบนบอร์ด [ผู้ใช้ขอ 2026-10-18] ----------
// < > เลือกโฟนีม · แตะช่องตัวเลข = เลือก · − / + ปรับ (F 10 Hz · cent 50 Hz · hilo 1 dB) · CLR = ไม่ใช้พารามิเตอร์นั้น
// ON/OFF = ใช้เทียบหรือไม่ · SAVE = เขียน /rd3/table.csv (ไฟล์เดิม → table.bak) · จอแสดงไทยไม่ได้ → ชื่อ show ภาษาอังกฤษ
const char *TBL_PAR[RD3_NPAR] = {"F1", "F2", "F3", "cent", "hilo"};
const float TBL_STEP[RD3_NPAR] = {10, 10, 10, 50, 1};
const float TBL_DEF[RD3_NPAR][2] = {{300, 500}, {1000, 1500}, {2400, 3000}, {1000, 3000}, {-10, 10}};   // ค่าเริ่มเมื่อกด + ที่ช่องว่าง

void tblMsg(uint16_t c, const char *m) {
  tft.fillRect(0, TBL_Y + RD3_NPAR * TBL_LH, 320, 9, ILI9341_BLACK);
  tft.setTextSize(1); tft.setTextColor(c); tft.setCursor(4, TBL_Y + RD3_NPAR * TBL_LH + 1); tft.print(m);
}

void drawTable() {
  if (page != P_TBL) return;
  tft.fillRect(54, 30, 212, 36, ILI9341_BLACK);
  if (!rd3.n) { tft.setTextSize(2); tft.setTextColor(ILI9341_RED); tft.setCursor(60, 40); tft.print("no table"); return; }
  const Rd3Row &w = rd3.rows[tblRow];
  tft.setTextSize(2); tft.setTextColor(w.active ? ILI9341_WHITE : ILI9341_DARKGREY);
  tft.setCursor(60, 32); tft.printf("%s", w.show);
  tft.setTextSize(1); tft.setTextColor(ILI9341_LIGHTGREY);
  tft.setCursor(140, 32); tft.printf("%d/%d", tblRow + 1, rd3.n);
  tft.setCursor(140, 44); tft.print(w.kind);
  tft.setCursor(60, 56); tft.printf("voiced %c  %s", w.voiced == 1 ? 'Y' : w.voiced == 0 ? 'N' : '-', w.active ? "ON (used)" : "OFF (kept)");
  for (int k = 0; k < RD3_NPAR; k++) {
    int y = TBL_Y + k * TBL_LH;
    tft.fillRect(0, y, 320, TBL_LH - 1, ILI9341_BLACK);
    tft.setTextSize(2); tft.setTextColor(ILI9341_CYAN); tft.setCursor(4, y + 1); tft.print(TBL_PAR[k]);
    for (int s = 0; s < 2; s++) {
      float v = s ? w.hi[k] : w.lo[k];
      bool sel = tblCell == k * 2 + s;
      int x = s ? TBL_HIX : TBL_LOX;
      if (sel) tft.fillRect(x, y, TBL_CW, TBL_LH - 1, ILI9341_YELLOW);
      tft.setTextColor(sel ? ILI9341_BLACK : (v == RD3_NONE ? ILI9341_DARKGREY : ILI9341_WHITE));
      tft.setCursor(x + 4, y + 1);
      if (v == RD3_NONE) tft.print("--"); else tft.printf("%g", v);
    }
  }
}

void tblChanged() { rd3.dirty = true; drawStatus(); drawTable(); }

void tblStep(int dir) {
  if (!rd3.n) return;
  Rd3Row &w = rd3.rows[tblRow];
  int k = tblCell / 2, s = tblCell % 2;
  float &v = s ? w.hi[k] : w.lo[k];
  if (v == RD3_NONE) {                   // ช่องว่าง: เริ่มจากค่ากลาง ๆ (ถ้าอีกข้างว่างด้วย ใส่ทั้งคู่)
    float &o = s ? w.lo[k] : w.hi[k];
    if (o == RD3_NONE) o = TBL_DEF[k][s ? 0 : 1];
    v = TBL_DEF[k][s];
  } else v += dir * TBL_STEP[k];
  if (w.lo[k] != RD3_NONE && w.hi[k] != RD3_NONE && w.lo[k] > w.hi[k]) { float t = w.lo[k]; w.lo[k] = w.hi[k]; w.hi[k] = t; }
  tblChanged();
}

// ---------- หน้า PHONEME: ตารางโฟนีม 7 × 6 ฟังสด [ผู้ใช้ขอ 2026-10-18] ----------
// เรียงตาม /rd3/table.csv · ตัวขาว = ใช้เทียบ (active) · ตัวเทา = ปิดไว้ · เสียงตรงตัวไหน (ระหว่างมีเสียง) = พื้นเขียว
const int PHO_COLS = 7, PHO_ROWS = 6, PHO_X = 2, PHO_Y = 18, PHO_W = 45, PHO_H = 30;
int phoHi = -1;                          // ช่องที่เป็นสีเขียวอยู่

void drawPhoCell(int i, bool hi) {
  if (i < 0 || i >= rd3.n || i >= PHO_COLS * PHO_ROWS) return;
  int x = PHO_X + (i % PHO_COLS) * PHO_W, y = PHO_Y + (i / PHO_COLS) * PHO_H;
  const Rd3Row &w = rd3.rows[i];
  tft.fillRect(x + 1, y + 1, PHO_W - 2, PHO_H - 2, hi ? ILI9341_GREEN : ILI9341_BLACK);
  tft.drawRect(x, y, PHO_W, PHO_H, ILI9341_DARKGREY);
  int len = strlen(w.show), sz = len <= 3 ? 2 : 1;
  tft.setTextSize(sz);
  tft.setTextColor(hi ? ILI9341_BLACK : w.active ? ILI9341_WHITE : ILI9341_DARKGREY);
  tft.setCursor(x + (PHO_W - len * 6 * sz) / 2, y + (PHO_H - 8 * sz) / 2);
  tft.print(w.show);
}

void drawPhoGrid() {
  if (page != P_PHO) return;
  phoHi = -1;
  for (int i = 0; i < min(rd3.n, PHO_COLS * PHO_ROWS); i++) drawPhoCell(i, false);
}

void updatePhoGrid(const Rd1Frame &f) {
  int now = (rd2live.inEvent && rd3last.best >= 0 && rd3last.best < PHO_COLS * PHO_ROWS) ? rd3last.best : -1;
  if (now != phoHi) { drawPhoCell(phoHi, false); drawPhoCell(now, true); phoHi = now; }
  tft.fillRect(0, 199, 320, 9, ILI9341_BLACK);
  tft.setTextSize(1); tft.setCursor(4, 199);
  if (rd2live.learning()) { tft.setTextColor(ILI9341_ORANGE); tft.print("learning background... keep quiet"); }
  else if (now >= 0) { tft.setTextColor(ILI9341_GREEN); tft.printf("%s  d %.2f   F1 %u F2 %u  cent %.0f", rd3.rows[now].show, rd3last.dist, f.fmt[0], f.fmt[1], rd3last.par[3]); }
  else { tft.setTextColor(ILI9341_DARKGREY); tft.printf("%s   table %d rows (%d on)", rd2live.inEvent ? "sound - no match" : "quiet", rd3.n, rd3.activeCount()); }
}

// ---------- หน้า META: ค่า Sound Metadata [ผู้ใช้ขอ 2026-10-18] ----------
// เหตุการณ์ล่าสุด: รหัส + ชื่อสั้น (จอแสดงไทยไม่ได้) · บรรทัดล่าง = ค่าสดของเสียงตอนนี้ · ไม่มีกราฟแท่ง (มีใน RD2 TEST)
const char *const META_NAME[12] = {"M4 syllables", "M6 rhythm", "M7 envelope", "M8 main band", "M9 pitch/tone", "M10 voice",
                                   "M11 loudness", "T1 band", "T3 timbre", "T4 loudness", "T6 envelope", "T7 pitch"};
const char *const META_TXT[12][10] = {
  {"1 syllable", "2", "3", "4", "5", "6-7", "8-10", ">10", "short burst", "long continuous"},
  {"steady", "repeating set", "short stops", "long continuous", "speeding up", "slowing down", "short-long", "accented", "irregular", "natural / n.a."},
  {"sudden, fast decay", "swell + fade", "steady long + fade", "soft -> loud", "loud -> soft", "gapped", "fluctuating", "peak middle", "peak end", "normal"},
  {"very low", "low", "mid-low", "mid", "mid-high", "high", "very high", "mixed", "unclear", "fast-changing"},
  {"flat (saman)", "rising (tri)", "falling (ek)", "rise-fall (tho)", "fall-rise (chattawa)", "jump up", "jump down", "small wobble", "non-standard", "no pitch"},
  {"clear", "dull/heavy", "resonant", "hoarse", "soft", "shrill", "breathy", "room echo", "sharp high", "normal"},
  {"whisper", "soft steady", "normal steady", "very loud", "fast change", "slow change", "soft -> loud", "loud -> soft", "alternating", "unstable"},
  {"<60 Hz", "bass", "low-mid", "mid", "high-mid", "high", "treble", ">8 kHz", "broadband", "varying"},
  {"pure tone", "clear harmonic", "complex harmonic", "noise / fricative", "short hit", "tremolo", "cracked", "ringing", "layered", "unknown"},
  {"inaudible", "very soft", "soft", "conversation", "loud", "very loud", "dangerous", "getting louder", "getting softer", "swinging"},
  {"hit, fast end", "hit, long", "rise slowly", "fade slowly", "steady long", "up-down", "broken", "< 100 ms", "endless", "varying"},
  {"flat", "rising", "falling", "rise-fall", "fall-rise", "stepwise", "vibrato", "no pitch", "chord", "free"},
};

void drawMetaPage() {
  if (page != P_META) return;
  tft.fillRect(0, 28, 320, 152, ILI9341_BLACK);
  tft.setTextSize(1);
  if (!smHave) { tft.setTextColor(ILI9341_DARKGREY); tft.setCursor(4, 32); tft.print("no sound event yet - make a sound (learning first 0.5 s)"); return; }
  const uint8_t v[12] = {smLast.M4, smLast.M6, smLast.M7, smLast.M8, smLast.M9, smLast.M10, smLast.M11,
                         smLast.T1, smLast.T3, smLast.T4, smLast.T6, smLast.T7};
  const uint8_t dim[12] = {4, 6, 7, 8, 9, 10, 11, 1, 3, 4, 6, 7};
  tft.setTextColor(ILI9341_WHITE); tft.setCursor(4, 30);
  tft.printf("last event #%u   %u ms   %u syllable(s)", (unsigned)rd2live.events, (unsigned)smLast.ms, smLast.syll);
  for (int i = 0; i < 12; i++) {
    int y = 44 + i * 11;
    tft.setTextColor(i < 7 ? ILI9341_CYAN : ILI9341_GREENYELLOW); tft.setCursor(4, y); tft.print(META_NAME[i]);
    tft.setTextColor(ILI9341_WHITE); tft.setCursor(92, y);
    if (v[i] >= 1 && v[i] <= 10) tft.printf("%u.%-2u %s", dim[i], v[i], META_TXT[i][v[i] - 1]);
    else tft.print("--");
  }
}

void drawMetaLive(const Rd1Frame &f) {     // ค่าสดของเฟรมปัจจุบัน
  tft.fillRect(0, 182, 320, 24, ILI9341_BLACK);
  tft.setTextSize(1);
  float par[RD3_NPAR]; Rd3Table::features(f, par);
  tft.setTextColor(rd2live.learning() ? ILI9341_ORANGE : rd2live.inEvent ? ILI9341_GREEN : ILI9341_DARKGREY);
  tft.setCursor(4, 184);
  tft.printf("now: %s  %.0f dB  cent %.0f Hz  hilo %+.1f", rd2live.learning() ? "learning" : rd2live.inEvent ? "SOUND" : "quiet",
             (f.total - RD1_CFG.log_offset) * RD1_CFG.log_step_db, par[3], par[4]);
  tft.setCursor(4, 196);
  if (f.f0_cents) {
    static const char *N[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    int m = (f.f0_cents + 50) / 100;
    tft.printf("f0 %s%d %.1f Hz  F1 %u F2 %u F3 %u", N[m % 12], m / 12 - 1, 440.0f * powf(2.0f, (f.f0_cents / 100.0f - 69) / 12.0f), f.fmt[0], f.fmt[1], f.fmt[2]);
  } else tft.print("f0 --  (no pitch)");
}

void redrawAll() {
  if (page == P_SAVER) { drawSaver(true); return; }
  tft.fillScreen(ILI9341_BLACK);
  drawStatus(); drawTime(); drawLevel(-90); drawWifiLine(); drawButtons();
  if (page == P_TBL) drawTable();
  if (page == P_PHO) drawPhoGrid();
  if (page == P_META) drawMetaPage();
  if (page == P_RM) drawRmPage();
}

// ---------- WAV ----------
void wavHeader(uint8_t *h, uint32_t dataBytes) {
  auto u32 = [&](int o, uint32_t v) { memcpy(h + o, &v, 4); };
  auto u16 = [&](int o, uint16_t v) { memcpy(h + o, &v, 2); };
  memcpy(h, "RIFF", 4); u32(4, 36 + dataBytes); memcpy(h + 8, "WAVEfmt ", 8);
  u32(16, 16); u16(20, 1); u16(22, 1); u32(24, RATE); u32(28, RATE * 2); u16(32, 2); u16(34, 16);
  memcpy(h + 36, "data", 4); u32(40, dataBytes);
}

void saveRecording() {
  if (!sdOk) { snprintf(lastName, sizeof(lastName), "psram_rec.wav"); return; }
  SD_MMC.mkdir("/train");
  for (int i = 1; i < 10000; i++) {
    snprintf(lastName, sizeof(lastName), "/train/rec_%04d.wav", i);
    if (!SD_MMC.exists(lastName)) break;
  }
  File f = SD_MMC.open(lastName, FILE_WRITE);
  uint8_t h[44]; wavHeader(h, pcmLen * 2);
  bool ok = f && f.write(h, 44) == 44 && f.write((uint8_t *)pcm, pcmLen * 2) == pcmLen * 2;
  if (f) f.close();
  Serial.printf("SD save %s: %s\n", lastName, ok ? "OK" : "FAILED");

  // RD1 คู่กัน: ชื่อเดียวกัน นามสกุล .rd1
  char rdName[40];
  strlcpy(rdName, lastName, sizeof(rdName));
  strcpy(strrchr(rdName, '.'), ".rd1");
  Rd1FileHeader rh; rd1.fillHeader(rh, rd1Len, FW_VERSION);
  File g = SD_MMC.open(rdName, FILE_WRITE);
  size_t body = rd1Len * sizeof(Rd1Frame);
  ok = g && g.write((uint8_t *)&rh, sizeof(rh)) == sizeof(rh) && g.write((uint8_t *)rd1buf, body) == body;
  if (g) g.close();
  Serial.printf("SD save %s: %s\n", rdName, ok ? "OK" : "FAILED");
}

// ส่งไฟล์ .rd1 ล่าสุดกลับทาง USB: "RD1 <ขนาด> <crc32> <ชื่อ>\n" + ข้อมูลดิบ (tools/get_wav.py --rd1)
void dumpRd1() {
  if (!rd1Len) { Serial.println("ERR no rd1"); return; }
  Rd1FileHeader rh; rd1.fillHeader(rh, rd1Len, FW_VERSION);
  size_t body = rd1Len * sizeof(Rd1Frame);
  uint32_t crc = esp_rom_crc32_le(0, (uint8_t *)&rh, sizeof(rh));
  crc = esp_rom_crc32_le(crc, (uint8_t *)rd1buf, body);
  char name[40];
  const char *base = strrchr(lastName, '/') ? strrchr(lastName, '/') + 1 : lastName;
  strlcpy(name, base, sizeof(name));
  if (strrchr(name, '.')) strcpy(strrchr(name, '.'), ".rd1");
  Serial.printf("RD1 %u %08X %s\n", (unsigned)(sizeof(rh) + body), crc, name);
  Serial.flush();
  Serial.write((uint8_t *)&rh, sizeof(rh));
  const uint8_t *p = (uint8_t *)rd1buf;
  for (size_t o = 0; o < body; o += 4096) Serial.write(p + o, min<size_t>(4096, body - o));
  Serial.flush();
}

// ทดสอบทุกช่วง: ไซน์ที่ความถี่กลางของช่วง b ต้องได้ค่าสูงสุดที่ช่วง b · + เวลาคำนวณ
void rd1Test() {
  Rd1Frame f; uint32_t us, maxUs = 0;
  int pass = 0;
  for (int e = 0; e < RD1_BANDS; e++) {
    float hz = (RD1_EDGES_HZ[e] + RD1_EDGES_HZ[e + 1]) / 2.0f;
    int b = rd1SelfTest(rd1, hz, 0.5f, f, us);
    maxUs = max(maxUs, us);
    if (b == e) pass++;
    Serial.printf("RD1 test %7.1f Hz -> band %2d (expect %2d) %s  value=%u total=%u  %u us\n",
                  hz, b + 1, e + 1, b == e ? "PASS" : "FAIL", f.band[b], f.total, us);
  }
  int16_t z[RD_FRAME_N] = {0};
  rd1.compute(z, 0, 0, f);
  Serial.printf("RD1 test silence -> total=%u f0=%u (expect 0 0)\n", f.total, f.f0_cents);
  // f0: ไซน์โน้ต → เลขโน้ต MIDI × 100 (A2=4500 · C4=6000 · A4=6900 · A5=8100)
  struct { float hz; int midi; } notes[] = {{110.0f, 45}, {261.63f, 60}, {440.0f, 69}, {880.0f, 81}};
  int pp = 0;
  for (auto &n : notes) {
    rd1SelfTest(rd1, n.hz, 0.5f, f, us);
    int got = (f.f0_cents + 50) / 100;
    if (got == n.midi) pp++;
    Serial.printf("RD1 f0 test %6.1f Hz -> cents %u = midi %d (expect %d) %s clarity %u\n",
                  n.hz, f.f0_cents, got, n.midi, got == n.midi ? "PASS" : "FAIL", f.clarity);
  }
  Serial.printf("RD1 f0 test %d/4 PASS\n", pp);
  // ฟอร์แมนต์: สระสังเคราะห์ (ชีพจรเสียง 120 Hz ผ่านตัวกรองเรโซแนนซ์ 3 ตัว) → F1 F2 ต้องคลาดไม่เกิน 100 Hz [ข้อเสนอ]
  struct { const char *name; float F[3]; } vw[] = {{"a (aa)", {800, 1250, 2600}}, {"i (ii)", {300, 2250, 2950}}, {"u (uu)", {320, 800, 2300}}};
  int fp = 0;
  for (auto &v : vw) {
    static int16_t s[RD_FRAME_N];
    static float y[RD_FRAME_N];
    for (int i = 0; i < RD_FRAME_N; i++) y[i] = (i % (RD_SAMPLE_RATE / 120) == 0) ? 1.0f : 0.0f;
    const float B[3] = {80, 100, 120};
    for (int k = 0; k < 3; k++) {
      float r = expf(-PI * B[k] / RD_SAMPLE_RATE), a1 = 2 * r * cosf(2 * PI * v.F[k] / RD_SAMPLE_RATE), a2 = -r * r, y1 = 0, y2 = 0;
      for (int i = 0; i < RD_FRAME_N; i++) { float o = y[i] + a1 * y1 + a2 * y2; y2 = y1; y1 = o; y[i] = o; }
    }
    float m = 0;
    for (int i = 0; i < RD_FRAME_N; i++) m = max(m, fabsf(y[i]));
    for (int i = 0; i < RD_FRAME_N; i++) s[i] = (int16_t)(y[i] / m * 16000);
    uint32_t t0 = micros();
    rd1.compute(s, 0, 0, f);
    us = micros() - t0;
    bool ok = abs((int)f.fmt[0] - (int)v.F[0]) <= 100 && abs((int)f.fmt[1] - (int)v.F[1]) <= 100;
    if (ok) fp++;
    Serial.printf("RD1 formant test %-6s true %4.0f %4.0f %4.0f -> %4u %4u %4u  %s  f0 %u  %u us\n", v.name, v.F[0], v.F[1], v.F[2],
                  f.fmt[0], f.fmt[1], f.fmt[2], ok ? "PASS" : "FAIL", f.f0_cents, us);
  }
  Serial.printf("RD1 formant test %d/3 PASS\n", fp);
  Serial.printf("RD1 test %d/%d PASS  max %u us per 20 ms\n", pass, RD1_BANDS, maxUs);
}

void dump() {
  if (!pcmLen) { Serial.println("ERR no recording"); return; }
  uint32_t bytes = 44 + pcmLen * 2;
  uint8_t h[44]; wavHeader(h, pcmLen * 2);
  uint32_t crc = esp_rom_crc32_le(0, h, 44);
  crc = esp_rom_crc32_le(crc, (uint8_t *)pcm, pcmLen * 2);
  const char *name = strrchr(lastName, '/') ? strrchr(lastName, '/') + 1 : lastName;
  Serial.printf("WAV %u %08X %s\n", bytes, crc, name);
  Serial.flush();
  Serial.write(h, 44);
  const uint8_t *p = (uint8_t *)pcm;
  for (uint32_t o = 0; o < pcmLen * 2; o += 4096) Serial.write(p + o, min<uint32_t>(4096, pcmLen * 2 - o));
  Serial.flush();
}

// ---------- สถานะเสียง ----------
void startRec() {
  if (state != IDLE || !pcm) return;
  if (usbDriveOn) { wifiMsg(ILI9341_ORANGE, "USB DRIVE on: REC off (SETTING)"); return; }
  state = REC; pos = 0;
  rd1Len = 0; rd1SumUs = 0; rd1MaxUs = 0;
  rgbLedWrite(PIN_RGB, 40, 0, 0);
  Serial.println("REC start");
  drawStatus(); drawButtons();
}

// โหมด RD1 LOG: ฟังต่อเนื่อง ไม่เก็บ WAV · ทุก 1 วินาที = 1 ไฟล์ข้อความ (rd1_log.h)
void startLog() {
  if (state != IDLE) return;
  if (usbDriveOn) { wifiMsg(ILI9341_ORANGE, "USB DRIVE on: RD1 LOG off (SETTING)"); return; }
  if (!rd1log.start(FW_VERSION)) {
    Serial.println("RD1 LOG: no PSRAM buffer");
    wifiMsg(ILI9341_RED, "RD1 LOG: no PSRAM buffer");
    return;
  }
  state = LOG; pos = 0;
  rd1Len = 0; rd1SumUs = 0; rd1MaxUs = 0;
  rgbLedWrite(PIN_RGB, 20, 40, 0);
  drawStatus(); drawButtons();
}

void startPlay() {
  if (state != IDLE || !pcmLen) return;
  if (!es8311PlayInit(RATE)) { Serial.println("PLAY: ES8311 init FAILED"); return; }
  state = PLAY; pos = 0;
  digitalWrite(PIN_PA, HIGH);            // เปิดแอมป์ลำโพง
  rgbLedWrite(PIN_RGB, 0, 40, 0);
  Serial.println("PLAY start");
  drawStatus(); drawButtons();
}

// ---------- WKW: JSON ของ 1 เหตุการณ์เสียง (รูปแบบที่ตกลงกับเซสชัน WKW) ----------
void accFrame(const Rd1Frame &f) {
  EvAcc &a = evAcc;
  a.n++;
  if (rd3last.best >= 0 && rd3last.best < RD3_MAX_ROWS) { a.cnt[rd3last.best]++; a.dsum[rd3last.best] += rd3last.dist; }
  a.cent += rd3last.par[3]; a.hilo += rd3last.par[4];
  a.db += (f.total - RD1_CFG.log_offset) * RD1_CFG.log_step_db;
  if (f.f0_cents) { a.nv++; a.f0 += 440.0f * powf(2.0f, (f.f0_cents / 100.0f - 69) / 12.0f); a.clar = max(a.clar, f.clarity); }
  if (f.fmt[0]) { a.nf++; a.F1 += f.fmt[0]; a.F2 += f.fmt[1]; a.F3 += f.fmt[2]; }
}

void isoTime(char *b, size_t n) {        // "2026-10-18T13:50:12+07:00" · ยังไม่ได้ตั้งเวลา = ""
  time_t now = time(nullptr); struct tm tmv; localtime_r(&now, &tmv);
  if (!timeOk || tmv.tm_year + 1900 < 2024) { b[0] = 0; return; }   // ts หลัง NTP เท่านั้น
  strftime(b, n, "%Y-%m-%dT%H:%M:%S+07:00", &tmv);
}

String soundEventJson() {
  EvAcc &a = evAcc;
  int b1 = -1, b2 = -1;                  // โฟนีมที่ตรงบ่อยสุด และอันดับ 2
  for (int i = 0; i < rd3.n && i < RD3_MAX_ROWS; i++) {
    if (!a.cnt[i]) continue;
    if (b1 < 0 || a.cnt[i] > a.cnt[b1]) { b2 = b1; b1 = i; } else if (b2 < 0 || a.cnt[i] > a.cnt[b2]) b2 = i;
  }
  int n = max(1, a.n);
  char ts[32]; isoTime(ts, sizeof(ts));
  char match[200] = "null";
  if (b1 >= 0) snprintf(match, sizeof(match),
      "{\"label\":\"%s\",\"show\":\"%s\",\"ipa\":\"%s\",\"dist\":%.2f,\"frames\":%u,\"second\":\"%s\",\"frames2\":%u}",
      rd3.rows[b1].label, rd3.rows[b1].show, rd3.rows[b1].ipa, a.dsum[b1] / a.cnt[b1], a.cnt[b1],
      b2 >= 0 ? rd3.rows[b2].show : "", b2 >= 0 ? a.cnt[b2] : 0);
  char note[8] = "";
  float f0 = a.nv ? a.f0 / a.nv : 0;
  if (f0 > 0) { static const char *N[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    int m = lroundf(69 + 12 * log2f(f0 / 440.0f)); snprintf(note, sizeof(note), "%s%d", N[m % 12], m / 12 - 1); }
  char buf[1100];
  snprintf(buf, sizeof(buf),
    "{\"src\":\"s3watch\",\"fw\":%u,\"type\":\"sound_event\"%s%s%s,\"event\":%u,\"ms\":%u,\"syll\":%u,\"match\":%s,"
    "\"feat\":{\"f0_hz\":%.1f,\"note\":\"%s\",\"clarity\":%u,\"F1\":%.0f,\"F2\":%.0f,\"F3\":%.0f,\"cent_hz\":%.0f,\"hilo_db\":%.1f,\"db\":%.1f,\"voiced_frames\":%d,\"frames\":%d},"
    "\"meta\":{\"M4\":%u,\"M6\":%u,\"M7\":%u,\"M8\":%u,\"M9\":%u,\"M10\":%u,\"M11\":%u,\"T1\":%u,\"T3\":%u,\"T4\":%u,\"T6\":%u,\"T7\":%u}}",
    (unsigned)FW_VERSION, ts[0] ? ",\"ts\":\"" : "", ts, ts[0] ? "\"" : "", (unsigned)rd2live.events, (unsigned)smLast.ms, smLast.syll, match,
    f0, note, a.clar, a.nf ? a.F1 / a.nf : 0, a.nf ? a.F2 / a.nf : 0, a.nf ? a.F3 / a.nf : 0,
    a.cent / n, a.hilo / n, a.db / n, a.nv, a.n,
    smLast.M4, smLast.M6, smLast.M7, smLast.M8, smLast.M9, smLast.M10, smLast.M11, smLast.T1, smLast.T3, smLast.T4, smLast.T6, smLast.T7);
  return String(buf);
}

String statusJson() {                    // ทุก 60 วิ — ตัวเลขชั้นบนขึ้นกราฟใน WKW (+ IMU ของนาฬิกา)
  char ts[32]; isoTime(ts, sizeof(ts));
  float v = batteryVolts();
  char imuJs[220] = "";
  if (imuOk) {
    float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    imu.getAccelerometer(ax, ay, az); imu.getGyroscope(gx, gy, gz);
    snprintf(imuJs, sizeof(imuJs),
             ",\"steps\":%lu,\"acc_x\":%.3f,\"acc_y\":%.3f,\"acc_z\":%.3f,\"gyro_x\":%.1f,\"gyro_y\":%.1f,\"gyro_z\":%.1f,\"imu_temp_c\":%.1f",
             (unsigned long)imu.getPedometerCounter(), ax, ay, az, gx, gy, gz, imu.getTemperature_C());
  }
  char buf[600];
  snprintf(buf, sizeof(buf),
    "{\"src\":\"s3watch\",\"fw\":%u,\"type\":\"status\"%s%s%s,\"batt_v\":%.2f,\"batt_pct\":%d,\"sd_free_mb\":%u,\"rssi\":%d,\"uptime_s\":%u,\"charging\":%d%s}",
    (unsigned)FW_VERSION, ts[0] ? ",\"ts\":\"" : "", ts, ts[0] ? "\"" : "", v, batteryPercent(v),
    sdOk ? (unsigned)((SD_MMC.totalBytes() - SD_MMC.usedBytes()) >> 20) : 0, (int)WiFi.RSSI(), (unsigned)(millis() / 1000),
    battCharging() ? 1 : 0, imuJs);
  return String(buf);
}

// หน้า RD2 TEST: ฟังสดต่อเนื่อง (ไม่บันทึก) · เข้าแล้วเรียนพื้นหลัง 0.5 วิแรก
void startRd2() {
  if (state != IDLE) { wifiMsg(ILI9341_ORANGE, "busy: stop RD1 LOG / REC first"); return; }
  if (!rd3.dirty) rd3.load(sdOk && !usbDriveOn ? (fs::FS *)&SD_MMC : nullptr);   // อ่านตารางใหม่ (เผื่อแก้ไฟล์มา) · มีแก้ค้างในหน้า TABLE = ใช้ของในหน่วยความจำ
  rd2live.reset();
  smb.resetRhythm(); smHave = false;
  state = RD2T;
  Serial.println("RD2 TEST start (learning background 0.5 s - keep quiet)");
}

void stopRd2() {
  if (state != RD2T) return;
  Serial.printf("RD2 TEST stop  events %u\n", (unsigned)rd2live.events);
  state = IDLE;
}

// ---------- REC+META [ผู้ใช้ขอ 2026-10-18]: อัดเสียงดิบลง SD + ผลวิเคราะห์ทุกเหตุการณ์ไปพร้อมกัน ----------
// ใช้ให้ AI (Claude) ฟังเสียงจริงเทียบกับเมตา → ปรับตารางโฟนีม / จับคู่คำ
// ไฟล์: /rm/NNNN/audio.wav (32 kHz 16 บิต โมโน · ยาวได้จนการ์ดเต็ม) + events.csv (start_ms = ตำแหน่งเสียงในไฟล์ WAV)
// เหตุการณ์ยังส่ง WKW เหมือนหน้า PHONEME แต่เพิ่ม "rec" + "start_ms" ชี้ตำแหน่งในไฟล์เสียง
const int RM_BUF_CH = 16;                // พักในหน่วยความจำ 16 ชิ้น (320 ms) แล้วเขียนลง SD ทีเดียว
int16_t *rmBuf = nullptr;
int rmFill = 0;                          // ตัวอย่างที่พักอยู่
uint32_t rmSamples = 0, rmSeq0 = 0, rmEvStartMs = 0, rmEvents = 0;
bool rmFail = false;
char rmDir[16] = "";
char rmLast[56] = "";                    // เสียงล่าสุด (แสดงบนจอ)
File rmWav, rmCsv;

void rmMsg(uint16_t c, const char *m) {
  if (page != P_RM) return;
  tft.fillRect(0, 118, 320, 14, ILI9341_BLACK);
  tft.setTextSize(1); tft.setTextColor(c); tft.setCursor(4, 122); tft.print(m);
}

void drawRmLive() {                      // โฟลเดอร์ · เวลา · จำนวนเสียง · เสียงล่าสุด
  if (page != P_RM) return;
  tft.setTextSize(2); tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setCursor(4, 34); tft.printf("%-8s %7.1f s ", rmDir[0] ? rmDir : "-", (rmSamples + rmFill) / (float)RATE);
  tft.setCursor(4, 56); tft.printf("events %-5u %-7s", (unsigned)rmEvents, state == RECM && rd2live.learning() ? "quiet.." : "");
  tft.setTextSize(1); tft.setTextColor(ILI9341_GREENYELLOW, ILI9341_BLACK);
  tft.setCursor(4, 82); tft.printf("last: %-46s", rmLast);
  tft.setTextColor(rmFail ? ILI9341_RED : ILI9341_LIGHTGREY, ILI9341_BLACK);
  tft.setCursor(4, 98); tft.printf("%-52s", rmFail ? "SD WRITE FAILED - stop and check card" : "audio.wav + events.csv on SD   (BOOT = stop)");
}

void drawRmPage() {
  if (page != P_RM) return;
  drawRmLive();
  if (state != RECM) rmMsg(sdOk ? ILI9341_LIGHTGREY : ILI9341_RED,
                           sdOk ? "START = record sound + metadata to /rm/NNNN/" : "no SD card - REC+META needs SD");
}

void rmFlush() {
  if (!rmFill) return;
  size_t b = rmFill * 2;
  if (!rmFail && rmWav.write((uint8_t *)rmBuf, b) != b) { rmFail = true; Serial.println("REC+META: SD write FAILED"); }
  rmSamples += rmFill; rmFill = 0;
}

void rmAdd(const int16_t *m) {
  memcpy(rmBuf + rmFill, m, CHUNK * 2);
  rmFill += CHUNK;
  if (rmFill >= RM_BUF_CH * CHUNK) rmFlush();
}

void rmEventDone() {                     // จบ 1 เหตุการณ์ → 1 แถวใน events.csv + ส่ง WKW (ค่าชุดเดียวกัน)
  rmEvents++;
  EvAcc &a = evAcc;
  int b1 = -1;                           // โฟนีมที่ตรงบ่อยสุด
  for (int i = 0; i < rd3.n && i < RD3_MAX_ROWS; i++) if (a.cnt[i] && (b1 < 0 || a.cnt[i] > a.cnt[b1])) b1 = i;
  int n = max(1, a.n);
  float F1 = a.nf ? a.F1 / a.nf : 0, F2 = a.nf ? a.F2 / a.nf : 0, F3 = a.nf ? a.F3 / a.nf : 0, f0 = a.nv ? a.f0 / a.nv : 0;
  char ml[96]; smLast.line(ml, sizeof(ml));
  if (rmCsv) rmCsv.printf("%u,%u,%u,%s,%s,%.0f,%.0f,%.0f,%.1f,%.0f,%.1f,%.1f,%s\n", (unsigned)rmEvents, (unsigned)rmEvStartMs,
                          (unsigned)smLast.ms, b1 >= 0 ? rd3.rows[b1].label : "", b1 >= 0 ? rd3.rows[b1].show : "",
                          F1, F2, F3, f0, a.cent / n, a.hilo / n, a.db / n, ml);
  snprintf(rmLast, sizeof(rmLast), "#%u %.1fs %s F1 %.0f F2 %.0f", (unsigned)rmEvents, rmEvStartMs / 1000.0f,
           b1 >= 0 ? rd3.rows[b1].show : "?", F1, F2);
  Serial.printf("REC+META event %s\n", rmLast);
  if (wkw.enabled()) {
    String js = soundEventJson();
    char extra[64]; snprintf(extra, sizeof(extra), ",\"rec\":\"%s\",\"start_ms\":%u}", rmDir, (unsigned)rmEvStartMs);
    js.remove(js.length() - 1); js += extra;
    wkw.queue(js);
  }
}

void startRm() {
  if (state != IDLE) { rmMsg(ILI9341_ORANGE, "busy: stop RD1 LOG / REC first"); return; }
  if (!sdOk || usbDriveOn) { rmMsg(ILI9341_RED, usbDriveOn ? "USB DRIVE on - switch to SERIAL" : "no SD card"); return; }
  if (!rmBuf) rmBuf = (int16_t *)ps_malloc(RM_BUF_CH * CHUNK * 2);
  if (!rmBuf) { rmMsg(ILI9341_RED, "no PSRAM buffer"); return; }
  SD_MMC.mkdir("/rm");
  for (int i = 1; i < 10000; i++) { snprintf(rmDir, sizeof(rmDir), "/rm/%04d", i); if (!SD_MMC.exists(rmDir)) break; }
  SD_MMC.mkdir(rmDir);
  char path[40];
  snprintf(path, sizeof(path), "%s/audio.wav", rmDir); rmWav = SD_MMC.open(path, FILE_WRITE);
  snprintf(path, sizeof(path), "%s/events.csv", rmDir); rmCsv = SD_MMC.open(path, FILE_WRITE);
  if (!rmWav || !rmCsv) { if (rmWav) rmWav.close(); if (rmCsv) rmCsv.close(); rmMsg(ILI9341_RED, "SD open FAILED"); return; }
  uint8_t h[44]; wavHeader(h, 0); rmWav.write(h, 44);   // ขนาดจริงเขียนทับตอนหยุด
  char ts[32]; isoTime(ts, sizeof(ts));
  rmCsv.printf("# REC+META fw %u start %s gain %+d dB (start_ms = position in audio.wav)\n", (unsigned)FW_VERSION, ts, gainDb);
  rmCsv.println("event,start_ms,ms,label,show,F1,F2,F3,f0_hz,cent_hz,hilo_db,db,meta");
  rmFill = 0; rmSamples = 0; rmEvents = 0; rmFail = false; rmLast[0] = 0;
  if (!rd3.dirty) rd3.load(&SD_MMC);
  rd2live.reset(); smb.resetRhythm(); smHave = false;
  rmSeq0 = chunkSeq;
  state = RECM;
  rgbLedWrite(PIN_RGB, 40, 0, 20);
  Serial.printf("REC+META start %s\n", rmDir);
  drawStatus(); drawButtons(); drawRmLive();
  rmMsg(ILI9341_ORANGE, "learning background 0.5 s - keep quiet, then speak");
}

void stopRm() {
  rmFlush();
  uint8_t h[44]; wavHeader(h, rmSamples * 2);
  rmWav.seek(0); rmWav.write(h, 44); rmWav.close();
  rmCsv.close();
  Serial.printf("REC+META stop %s  %.1f s  events %u%s\n", rmDir, rmSamples / (float)RATE, (unsigned)rmEvents, rmFail ? "  SD FAILED" : "");
  state = IDLE;
  drawRmLive();
  char m[64]; snprintf(m, sizeof(m), "saved %s  %.1f s  %u events", rmDir, rmSamples / (float)RATE, (unsigned)rmEvents);
  rmMsg(rmFail ? ILI9341_RED : ILI9341_GREEN, m);
}

void stopAll() {
  if (state == RD2T) { stopRd2(); return; }
  if (state == REC) {
    pcmLen = pos;
    Serial.printf("REC done %u samples (%.1f s)\n", pcmLen, pcmLen / (float)RATE);
    Serial.printf("RD1 %u frames  avg %u us  max %u us per 20 ms\n", rd1Len,
                  rd1Len ? rd1SumUs / rd1Len : 0, rd1MaxUs);
    drawStatus(); tft.setTextSize(2); tft.setTextColor(ILI9341_YELLOW);
    tft.setCursor(4, 6); tft.print("SAVING...   ");
    saveRecording();
    if (!sdOk) Serial.println("saved in PSRAM (no SD)");
  } else if (state == RECM) {
    stopRm();
  } else if (state == PLAY) {
    digitalWrite(PIN_PA, LOW);           // ปิดแอมป์
    Serial.println("PLAY stop");
  } else if (state == LOG) {
    rd1log.stop();                       // มี SD: ย้ายไฟล์ที่เหลือใน PSRAM ลง SD ทั้งหมด
    Serial.printf("RD1 LOG stop %s  %.1f s  %u frames = %u files  SD %u (fail %u)  dropped %u  avg %u us max %u us\n",
                  rd1log.dir(), pos / (float)RATE, rd1log.frames(), rd1log.files(), rd1log.filesOnSd(),
                  rd1log.sdFail, rd1log.dropped,
                  rd1Len ? rd1SumUs / rd1Len : 0, rd1MaxUs);
  }
  state = IDLE;
  rgbLedWrite(PIN_RGB, 0, 0, wifiState == W_READY ? 20 : 0);   // ฟ้าจาง = รอรับ OTA
  drawStatus(); drawTime(); drawLevel(-90); drawButtons();
  if (page == P_RM) { if (state == IDLE && !rmDir[0]) drawRmPage(); else drawRmLive(); }
}

void changeGain(int d) {
  int g = constrain(gainDb + d, GAIN_MIN, GAIN_MAX);
  if (g == gainDb) return;
  gainDb = g; applyGain(); drawBtn(B_GVAL, false);
}

// ---------- Wi-Fi + OTA ----------
void otaSetup() {
  ArduinoOTA.setHostname(OTA_HOST);
  ArduinoOTA.setPassword(OTA_PASS);
  ArduinoOTA.onStart([]() {
    if (page == P_SAVER) page = savedPage;   // OTA วาดหน้าจอเอง
    if (state != IDLE) stopAll();        // หยุดอัด/เล่น (อัดอยู่ = บันทึกก่อน)
    Serial.println("OTA start");
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextSize(3); tft.setTextColor(ILI9341_CYAN);
    tft.setCursor(60, 60); tft.print("OTA UPDATE");
    rgbLedWrite(PIN_RGB, 0, 0, 40);
  });
  ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
    static int lastPct = -1;
    int pct = done * 100 / total;
    if (pct == lastPct) return;
    lastPct = pct;
    tft.fillRect(20, 120, pct * 280 / 100, 24, ILI9341_CYAN);
    tft.drawRect(20, 120, 280, 24, ILI9341_WHITE);
    tft.setTextSize(2); tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
    tft.setCursor(130, 160); tft.printf("%3d %%", pct);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("OTA done -> restart");
    tft.setCursor(80, 190); tft.setTextColor(ILI9341_GREEN); tft.print("DONE, restart");
  });
  ArduinoOTA.onError([](ota_error_t e) {
    Serial.printf("OTA error %u\n", e);
    tft.setCursor(40, 190); tft.setTextColor(ILI9341_RED); tft.printf("OTA ERROR %u", e);
    delay(2000);
    redrawAll();
  });
}

// เริ่มต่อวงที่ idx ขึ้นไปที่มีชื่อ · คืน false ถ้าไม่เหลือวงให้ลอง
bool wifiTry(int idx) {
  for (; idx < N_NETS; idx++) {
    if (strlen(NETS[idx].ssid) == 0) continue;
    netIdx = idx;
    WiFi.disconnect(true);
    WiFi.begin(NETS[idx].ssid, NETS[idx].pass);
    wifiStartedAt = millis();
    Serial.printf("WiFi connecting (%s)\n", idx == 0 ? "main" : "backup");   // ไม่พิมพ์ชื่อ/รหัส
    drawWifiLine();
    return true;
  }
  return false;
}

void wifiOn() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(OTA_HOST);
  wifiState = W_CONNECTING;
  if (!wifiTry(0)) {
    Serial.println("WiFi: no SSID in secrets.h");
    WiFi.mode(WIFI_OFF);
    wifiState = W_OFF;
    pendingUpdate = false;
    wifiMsg(ILI9341_RED, "WiFi: no SSID in secrets.h");
    return;
  }
  drawBtn(B_WIFI, true);
}

// ผู้ใช้แตะ WIFI (หรือ USB w): สลับ + จำในแฟลช — การปิดเองเพราะต่อไม่ติดไม่ผ่านตรงนี้
void toggleWifiSaved() {
  bool on = wifiState == W_OFF;
  if (on) wifiOn(); else wifiOff();
  prefs.putBool("wifi", on);
  Serial.printf("WiFi saved %s\n", on ? "ON" : "OFF");
}

void wifiOff() {
  if (wifiState == W_READY) ArduinoOTA.end();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiState = W_OFF;
  pendingUpdate = false;
  Serial.println("WiFi off");
  if (state == IDLE) rgbLedWrite(PIN_RGB, 0, 0, 0);
  drawWifiLine(); drawBtn(B_WIFI, false);
}

// ---------- UPDATE: ดึงโค้ดใหม่จาก NRW_OTA ----------
void pullSay(const char *msg) {
  Serial.printf("UPDATE: %s\n", msg);
  wifiMsg(ILI9341_MAGENTA, msg);
}

void pullProgress(int pct) {             // ใช้แถบระดับเสียงเป็นแถบดาวน์โหลด
  tft.fillRect(4, 40, pct * 230 / 100, 24, ILI9341_MAGENTA);
  tft.fillRect(4 + pct * 230 / 100, 40, 230 - pct * 230 / 100, 24, ILI9341_DARKGREY);
  tft.setTextSize(2); tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  tft.setCursor(240, 44); tft.printf("%3d%%", pct);
}

void runPull() {
  if (state != IDLE) stopAll();          // อัดอยู่ = หยุด + บันทึกก่อน
  if (page != P_MAIN) showPage(P_MAIN);  // แถบความคืบหน้าอยู่หน้าหลัก
  drawBtn(B_UPDATE, true);
  rgbLedWrite(PIN_RGB, 30, 0, 30);
  // [แก้ 2026-10-18] UPDATE รีสตาร์ตกลางทาง: WKW ค้าง TLS ไว้ + GitHub อีกก้อน = หน่วยความจำไม่พอ → หยุด WKW ก่อน
  wkw.paused = true;
  for (int i = 0; i < 30 && !wkw.idle; i++) delay(100);
  Serial.printf("OTA: free heap %u (largest %u)\n", ESP.getFreeHeap(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  bool ok = otaPull(pullSay, pullProgress);
  wkw.paused = false;
  if (ok) {
    Serial.flush();
    delay(800);
    ESP.restart();
  }
  rgbLedWrite(PIN_RGB, 0, 0, wifiState == W_READY ? 20 : 0);
  drawBtn(B_UPDATE, false);
  drawLevel(-90);
}

void startUpdate() {
  if (wifiState == W_READY) runPull();
  else { pendingUpdate = true; if (wifiState == W_OFF) wifiOn(); }
}

void wifiPoll() {
  if (wifiState == W_CONNECTING) {
    if (WiFi.status() == WL_CONNECTED) {
      ArduinoOTA.begin();
      configTzTime("ICT-7", "pool.ntp.org", "time.google.com");   // ตั้งนาฬิกาจากอินเทอร์เน็ต (เวลาไทย)
      wifiState = W_READY;
      // พิมพ์ IP ทาง USB ให้ upload.sh ใช้ (IP ในวงแลนบ้าน — ไม่บันทึกลงไฟล์ที่ขึ้น GitHub)
      Serial.printf("OTA ready ip=%s host=%s.local\n", WiFi.localIP().toString().c_str(), OTA_HOST);
      if (state == IDLE) rgbLedWrite(PIN_RGB, 0, 0, 20);
      drawWifiLine();
      if (pendingUpdate) { pendingUpdate = false; runPull(); }
    } else if (millis() - wifiStartedAt > WIFI_TIMEOUT_MS) {
      Serial.printf("WiFi timeout (%s)\n", netIdx == 0 ? "main" : "backup");
      if (wifiTry(netIdx + 1)) return;   // ลองวงถัดไป
      wifiOff();
      wifiMsg(ILI9341_RED, "WiFi: connect failed");
    }
  } else if (wifiState == W_READY) {
    ArduinoOTA.handle();
    if (!timeOk && esp_sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED) {   // ได้เวลาจริง → ตั้งชิป RTC ด้วย
      timeOk = true;
      time_t now = time(nullptr); struct tm t; localtime_r(&now, &t);
      if (rtcOk) rtc.setDateTime(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
      Serial.println("[time] NTP ok -> RTC set");
    }
  }
}

// ---------- ปุ่ม ----------
void onTouch(int x, int y) {
  // หน้า TABLE: แตะช่องค่า (lo / hi ของพารามิเตอร์) = เลือกช่องที่จะปรับด้วย − / +
  if (page == P_TBL && y >= TBL_Y && y < TBL_Y + RD3_NPAR * TBL_LH && x >= TBL_LOX && x < TBL_HIX + TBL_CW) {
    tblCell = ((y - TBL_Y) / TBL_LH) * 2 + (x >= TBL_HIX ? 1 : 0);
    drawTable();
    return;
  }
  for (int i = 0; i < B_COUNT; i++) {
    Btn &b = btns[i];
    if (b.page != page) continue;
    if (x < b.x || x >= b.x + b.w || y < b.y || y >= b.y + b.h) continue;
    Serial.printf("button %s\n", i == B_GVAL ? "(gain value)" : b.label);
    switch (i) {
      case B_REC:  startRec(); break;
      case B_STOP: stopAll(); break;
      case B_PLAY: startPlay(); break;
      case B_GDN:  changeGain(-GAIN_STEP); break;
      case B_GUP:  changeGain(+GAIN_STEP); break;
      case B_WIFI: toggleWifiSaved(); break;
      case B_UPDATE: startUpdate(); break;
      case B_LOG:  if (state == LOG) stopAll(); else startLog(); break;
      case B_MENU: showPage(P_MENU); break;
      case B_RECPG: showPage(P_REC); break;
      case B_RD2PG: showPage(P_RD2); break;
      case B_RBACK: case B_2BACK: showPage(P_MENU); break;
      case B_TBLPG: tblRow = constrain(tblRow, 0, max(0, rd3.n - 1)); showPage(P_TBL); break;
      case B_TPREV: if (rd3.n) { tblRow = (tblRow + rd3.n - 1) % rd3.n; drawTable(); drawBtn(B_TACT, false); } break;
      case B_TNEXT: if (rd3.n) { tblRow = (tblRow + 1) % rd3.n; drawTable(); drawBtn(B_TACT, false); } break;
      case B_TMINUS: tblStep(-1); break;
      case B_TPLUS: tblStep(+1); break;
      case B_TCLR: if (rd3.n) { Rd3Row &w = rd3.rows[tblRow]; w.lo[tblCell / 2] = w.hi[tblCell / 2] = RD3_NONE; tblChanged(); } break;
      case B_TACT: if (rd3.n) { rd3.rows[tblRow].active = !rd3.rows[tblRow].active; tblChanged(); drawBtn(B_TACT, false); } break;
      case B_TSAVE:
        if (sdOk && !usbDriveOn && rd3.save(&SD_MMC)) { drawStatus(); tblMsg(ILI9341_GREEN, "saved /rd3/table.csv (old -> table.bak)"); }
        else tblMsg(ILI9341_RED, usbDriveOn ? "USB DRIVE on - switch to SERIAL first" : "no SD card - not saved");
        break;
      case B_TBACK: showPage(P_MENU); break;
      case B_PHOPG: showPage(P_PHO); break;
      case B_METAPG: showPage(P_META); break;
      case B_PBACK: case B_MBACK: showPage(P_MENU); break;
      case B_NEXT: showPage(P_MENU2); break;
      case B_M2BACK: showPage(P_MENU); break;
      case B_RMPG: showPage(P_RM); break;
      case B_RMGO: if (state == RECM) stopAll(); else startRm(); break;
      case B_RMBACK: showPage(P_MENU2); break;
      case B_BACK: showPage(P_MAIN); break;
      case B_SET:  showPage(P_SET); break;
      case B_SBACK: showPage(P_MENU); break;
      case B_USB: {                      // นาฬิกา: ปุ่มนี้ = ความสว่างจอ (วนค่า)
        static const uint8_t LV[] = {60, 120, 160, 220, 255};
        int k = 0; while (k < 5 && LV[k] <= brightness) k++;
        brightness = LV[k % 5];
        gfx->setBrightness(brightness); prefs.putUChar("bright", brightness);
        drawBtn(B_USB, false);
        break;
      }
      case B_RESET:
        if (resetArmedAt && millis() - resetArmedAt < RESET_CONFIRM_MS) {
          if (state != IDLE) stopAll();  // อัดอยู่ → หยุด + บันทึกก่อน
          Serial.println("RESET by touch");
          tft.fillScreen(ILI9341_BLACK);
          tft.setTextSize(3); tft.setTextColor(ILI9341_ORANGE);
          tft.setCursor(70, 108); tft.print("RESTART...");
          Serial.flush();
          delay(300);
          ESP.restart();
        }
        resetArmedAt = millis();
        drawBtn(B_RESET, true);
        break;
    }
    return;
  }
}

void setup() {
  Serial.setTxTimeoutMs(5000);           // ส่งไฟล์ใหญ่ทาง USB ไม่ให้ทิ้งข้อมูล
  Serial.begin(115200);
  pinMode(PIN_PA, OUTPUT); digitalWrite(PIN_PA, LOW);   // ปิดแอมป์
  pinMode(PIN_BOOT, INPUT_PULLUP);
  delay(1500);
  Serial.println("=== N.tag S3 Watch menu (ES3C28P port) ===");
  Serial.printf("running from %s  fw %u\n", esp_ota_get_running_partition()->label, (uint32_t)FW_VERSION);

  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  pmuInit();
  touchInit();
  rtcOk = rtc.begin(Wire, I2C_SDA, I2C_SCL);
  if (rtcOk) {                           // เวลาระบบจากชิป RTC (มีแบตสำรองผ่าน AXP2101)
    RTC_DateTime d = rtc.getDateTime();
    struct tm t = {}; t.tm_year = d.getYear() - 1900; t.tm_mon = d.getMonth() - 1; t.tm_mday = d.getDay();
    t.tm_hour = d.getHour(); t.tm_min = d.getMinute(); t.tm_sec = d.getSecond();
    setenv("TZ", "ICT-7", 1); tzset();
    time_t e = mktime(&t); struct timeval tv = {e, 0}; settimeofday(&tv, nullptr);
  }
  imuOk = imu.begin(Wire, QMI8658_L_SLAVE_ADDRESS, I2C_SDA, I2C_SCL);
  if (imuOk) {
    imu.configAccelerometer(SensorQMI8658::ACC_RANGE_2G, SensorQMI8658::ACC_ODR_62_5Hz);
    imu.enableAccelerometer();
    imu.configGyroscope(SensorQMI8658::GYR_RANGE_512DPS, SensorQMI8658::GYR_ODR_56_05Hz);
    imu.enableGyroscope();
    imu.configPedometer(50, 200, 100, 200, 20, 10, 0, 4);
    imu.enablePedometer(SensorQMI8658::INTERRUPT_PIN_DISABLE);
  }

  if (!gfx->begin()) Serial.println("[lcd] begin failed");
  tft.begin();
  prefs.begin("rd0", false);
  brightness = prefs.getUChar("bright", 160);
  gfx->setBrightness(brightness);

  SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0); // 1 บิต ตามตัวอย่างผู้ผลิต
  sdOk = SD_MMC.begin("/sdcard", true);

  pcm = (int16_t *)ps_malloc(MAX_SAMPLES * 2);
  Serial.printf("PSRAM buffer %u bytes: %s\n", MAX_SAMPLES * 2, pcm ? "OK" : "FAILED");
  rd1.begin();
  rd1buf = (Rd1Frame *)ps_malloc(MAX_FRAMES * sizeof(Rd1Frame));
  if (sdOk) Serial.printf("SD OK size=%llu MB\n", SD_MMC.cardSize() >> 20);
  else Serial.println("SD not found -> PSRAM only");
  rd3.load(sdOk ? (fs::FS *)&SD_MMC : nullptr);
  rd1log.begin(sdOk);

  i2s.setPins(I2S_BCLK, I2S_WS, I2S_DOUT, I2S_DIN, I2S_MCLK);
  if (!i2s.begin(I2S_MODE_STD, RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH))
    Serial.println("I2S begin FAILED");
  gainDb = constrain(prefs.getInt("gain", GAIN_DEFAULT), GAIN_MIN, GAIN_MAX);
  bool c = es7210Init(gainDb);
  Serial.printf("mic es7210 %s  rtc %s  imu %s\n", c ? "ok" : "FAILED", rtcOk ? "ok" : "FAIL", imuOk ? "ok" : "FAIL");

  WiFi.mode(WIFI_OFF);
  otaSetup();
  wkw.begin();                           // ไม่มี key = ปิดเอง
  redrawAll();
  // นาฬิกา: Wi-Fi เปิดเป็นค่าเริ่ม (WKW + UPDATE) · ผู้ใช้ปิดได้ที่ปุ่ม WIFI (จำในแฟลช)
  if (prefs.getBool("wifi", true)) { Serial.println("WiFi: saved ON -> connecting"); wifiOn(); }
  Serial.printf("WiFi config: main=%s backup=%s\n", strlen(WIFI_SSID) ? "set" : "empty", strlen(WIFI_SSID2) ? "set" : "empty");
  Serial.println("ready: touch / BOOT = UPDATE / PWR = saver / r s p + - d w u t m l g k T F z");
}

void loop() {
  // ปุ่ม BOOT = UPDATE — เช็กก่อนทุกอย่าง แยกจากทัช/เมนู (OTA ต้องใช้ได้เสมอ)
  if (digitalRead(PIN_BOOT) == LOW && millis() > 3000) {
    Serial.println("BOOT -> UPDATE");
    if (page == P_SAVER) saverOff();
    startUpdate();
    while (digitalRead(PIN_BOOT) == LOW) delay(10);
  }
  static bool wasTouched = false;
  static uint32_t lastUi = 0;
  int16_t st[CHUNK * 2];

  // งานเสียง (ทำก่อนทุกอย่าง — ชิ้นละ 20 ms)
  float db = -90;
  static const Rd1Frame *rd1Last = nullptr;
  if (state == REC) {
    // RD0: ชิ้น 20 ms (640 ตัวอย่าง) จากช่องซ้าย (L = R ทดสอบแล้ว) · ได้ไม่ครบ = เติมศูนย์ + ธง gap
    size_t got = i2s.readBytes((char *)st, sizeof(st));
    int n = min((int)(got / 4), CHUNK);
    int16_t *rd0 = pcm + pos;            // เก็บเสียงดิบลง WAV โดยตรง
    double s = 0;
    for (int i = 0; i < CHUNK; i++) { rd0[i] = i < n ? st[2 * i] : 0; s += (double)rd0[i] * rd0[i]; }
    pos += CHUNK;
    // RD0 → RD1 (FFT 32 ช่วง)
    if (rd1buf && rd1Len < MAX_FRAMES) {
      uint32_t us = rd1.compute(rd0, chunkSeq, n < CHUNK ? RD1_FLAG_GAP : 0, rd1buf[rd1Len]);
      rd1Last = &rd1buf[rd1Len++];
      rd1SumUs += us; rd1MaxUs = max(rd1MaxUs, us);
    }
    chunkSeq++;
    db = 20 * log10f(max(sqrtf(s / CHUNK), 0.5f) / 32768.0f);
    if (pos + CHUNK > MAX_SAMPLES) stopAll();
  } else if (state == LOG) {
    // RD1 LOG: ชิ้น 20 ms → FFT 32 ช่วง → 1 แถวในไฟล์ของวินาทีนี้ (ไม่เก็บเสียงดิบ)
    size_t got = i2s.readBytes((char *)st, sizeof(st));
    int n = min((int)(got / 4), CHUNK);
    static int16_t mono[CHUNK];
    double s = 0;
    for (int i = 0; i < CHUNK; i++) { mono[i] = i < n ? st[2 * i] : 0; s += (double)mono[i] * mono[i]; }
    pos += CHUNK;
    static Rd1Frame lf;
    uint32_t us = rd1.compute(mono, chunkSeq++, n < CHUNK ? RD1_FLAG_GAP : 0, lf);
    rd1SumUs += us; rd1MaxUs = max(rd1MaxUs, us); rd1Len++;
    rd1log.add(lf);
    rd1Last = &lf;
    db = 20 * log10f(max(sqrtf(s / CHUNK), 0.5f) / 32768.0f);
    if (rd1log.full) { Serial.println("RD1 LOG: PSRAM full -> stop"); stopAll(); }
  } else if (state == RD2T || state == RECM) {
    // RD2 TEST: ชิ้น 20 ms → RD1 (32 ช่วง + f0) → RD2 พื้นหลัง/เหตุการณ์ · ไม่บันทึก
    size_t got = i2s.readBytes((char *)st, sizeof(st));
    int n = min((int)(got / 4), CHUNK);
    static int16_t mono2[CHUNK];
    for (int i = 0; i < CHUNK; i++) mono2[i] = i < n ? st[2 * i] : 0;
    if (state == RECM) rmAdd(mono2);              // REC+META: เสียงดิบลง SD
    static Rd1Frame tf;
    rd1.compute(mono2, chunkSeq++, n < CHUNK ? RD1_FLAG_GAP : 0, tf);
    bool wasEvent = rd2live.inEvent;
    rd2live.update(tf);
    // Sound Metadata: เก็บทุกเฟรมระหว่างเหตุการณ์ → จบเหตุการณ์ = คิดรหัส + พิมพ์ทาง USB
    if (rd2live.inEvent && !wasEvent) { smb.begin(tf.seq * SM_FRAME_MS); evAcc.reset(); rmEvStartMs = (tf.seq - rmSeq0) * SM_FRAME_MS; }
    if (rd2live.inEvent) { smb.add(tf); accFrame(tf); }
    if (!rd2live.inEvent && wasEvent) {
      smLast = smb.finish(rd2live.cfg.end_frames); smHave = true;
      char ml[96]; smLast.line(ml, sizeof(ml));
      Serial.printf("META #%u %u ms syll %u  %s\n", (unsigned)rd2live.events, (unsigned)smLast.ms, smLast.syll, ml);
      if (state == RECM) rmEventDone();
      else if (wkw.enabled()) wkw.queue(soundEventJson());
    }
    rd3last = rd3.match(tf);
    static int lastBest = -2;              // พิมพ์ทาง USB เมื่อคำตอบเปลี่ยน (ชื่อไทยแสดงได้ทาง USB)
    int nowBest = rd2live.inEvent ? rd3last.best : -2;
    if (nowBest != lastBest) {
      lastBest = nowBest;
      if (nowBest >= 0) Serial.printf("RD3 %s (%s) d=%.2f  F1 %.0f F2 %.0f cent %.0f hilo %.1f\n", rd3.rows[nowBest].label,
                                      rd3.rows[nowBest].show, rd3last.dist, rd3last.par[0], rd3last.par[1], rd3last.par[3], rd3last.par[4]);
    }
    if (rd2live.inEvent && !wasEvent) Serial.printf("RD2 event %u start seq %u\n", (unsigned)rd2live.events, (unsigned)tf.seq);
    static uint32_t lastDraw = 0;
    if (page == P_RD2 && millis() - lastDraw > 100) { lastDraw = millis(); drawRd2Bars(tf, rd2live); drawRd2Status(tf, rd2live); }
    if (page == P_PHO && millis() - lastDraw > 100) { lastDraw = millis(); updatePhoGrid(tf); }
    if (page == P_META && millis() - lastDraw > 200) { lastDraw = millis(); drawMetaLive(tf); }
    if (page == P_META && !rd2live.inEvent && wasEvent) drawMetaPage();
    if (page == P_RM && millis() - lastDraw > 500) { lastDraw = millis(); drawRmLive(); }   // จบเหตุการณ์ = อัปเดตรหัสทั้งหน้า
  } else if (state == PLAY) {
    int n = min<uint32_t>(CHUNK, pcmLen - pos);
    double s = 0;
    for (int i = 0; i < n; i++) { int16_t v = pcm[pos + i]; st[2 * i] = st[2 * i + 1] = v; s += (double)v * v; }
    i2s.write((uint8_t *)st, n * 4);
    pos += n;
    db = 20 * log10f(max(sqrtf(s / max(n, 1)), 0.5f) / 32768.0f);
    if (pos >= pcmLen) stopAll();
  }
  if (state != IDLE && state != RD2T && state != RECM && millis() - lastUi > 100) {
    lastUi = millis(); drawTime();
    if ((state == REC || state == LOG) && rd1Last) drawBands(*rd1Last, db); else drawLevel(db);
    static uint32_t shownFiles = 0;      // จำนวนไฟล์บนบรรทัดสถานะ (วาดใหม่เมื่อเปลี่ยน)
    if (state == LOG && rd1log.filesDone() != shownFiles) { shownFiles = rd1log.filesDone(); drawStatus(); }
  }

  wifiPoll();

  static uint32_t lastWkwStatus = 0;       // WKW: สถานะอุปกรณ์ทุก 60 วิ (เมื่อ Wi-Fi ต่ออยู่)
  if (wkw.enabled() && wifiState == W_READY && millis() - lastWkwStatus >= 60000) { lastWkwStatus = millis(); wkw.queue(statusJson()); }

  static uint32_t lastClock = 0;           // หน้าหลัก: เวลา + แบต ทุก 1 วิ
  if (millis() - lastClock >= 1000 && (page == P_MAIN || page == P_SAVER)) {
    lastClock = millis();
    if (page == P_SAVER) drawSaver(false); else drawClockBattery();
  }
  // ไม่แตะจอ 5 นาที หรือกด PWR → โหมดประหยัด (มีงานอัด/LOG วิ่ง = นาฬิกาแต่ไม่หลับ · ว่าง = หลับ)
  bool pwr = millis() > 3000 && pwrPressed();
  if (page == P_SAVER && pwr) { lastTouchMs = millis(); saverOff(); pwr = false; }
  if (page != P_SAVER && (pwr || millis() - lastTouchMs >= SAVER_AFTER_MS) && !pendingUpdate) {
    bool job = state == REC || state == LOG || state == RECM || state == PLAY;
    if (job) saverOn(); else deepSaver();
  }

  // RESET: แตะครั้งแรกแล้วไม่แตะซ้ำภายใน 3 วิ → ยกเลิก
  if (resetArmedAt && millis() - resetArmedAt >= RESET_CONFIRM_MS) {
    resetArmedAt = 0;
    drawBtn(B_RESET, false);
  }

  // จอสัมผัส: ทำงานตอนเริ่มแตะ
  int x, y;
  bool t = touchScreen(x, y);
  if (t && !wasTouched) {
    lastTouchMs = millis();
    if (page == P_SAVER) saverOff();     // แตะแรกแค่ปลุกจอ
    else { Serial.printf("touch x=%d y=%d\n", x, y); onTouch(x, y); }
  }
  wasTouched = t;


  // คำสั่ง USB
  int cmd = Serial.available() ? Serial.read() : -1;
  switch (cmd) {
    case 'r': startRec(); break;
    case 's': stopAll(); break;
    case 'p': startPlay(); break;
    case '+': changeGain(+GAIN_STEP); break;
    case '-': changeGain(-GAIN_STEP); break;
    case 'd': if (state == IDLE) dump(); break;
    case 'w': toggleWifiSaved(); break;
    case 'u': startUpdate(); break;
    case 'f': if (state == IDLE) dumpRd1(); break;
    case 't': if (state == IDLE) rd1Test(); break;
    case 'm': if (state == LOG) stopAll(); else startLog(); break;
    case 'l': if (state == IDLE) rd1log.list(); break;
    case 'g': if (state == IDLE) rd1log.dumpLast(); break;
    case 'k': rd3.load(sdOk && !usbDriveOn ? (fs::FS *)&SD_MMC : nullptr); rd3.print(); break;   // อ่าน + พิมพ์ตารางเทียบเสียง
    case 'T': if (state == IDLE) sdTest(); break;
    case 'z': if (state == IDLE) deepSaver(); else saverOn(); break;   // ทดสอบโหมดประหยัด
    case 'F':                            // ฟอร์แมต SD: ต้องพิมพ์ YES ภายใน 10 วิ (กันพลาด)
      if (state == IDLE) {
        Serial.println("SD format: type YES within 10 s to erase the whole card");
        String a = ""; uint32_t t0 = millis();
        while (millis() - t0 < 10000 && a.length() < 3) if (Serial.available()) { char c = Serial.read(); if (c > ' ') a += c; }
        if (a == "YES") {
          sdOk = sdFormat(SD_CLK, SD_CMD, SD_D0);
          // rd1log.begin() เลือกโหมดเก็บตอนบูต → รีสตาร์ตให้เริ่มใหม่กับการ์ดที่ฟอร์แมตแล้ว
          Serial.printf("SD format %s -> restart\n", sdOk ? "done" : "FAILED");
          Serial.flush(); delay(300); ESP.restart();
        } else Serial.println("SD format: cancelled");
      }
      break;
  }
  if (state == IDLE) delay(10);
}

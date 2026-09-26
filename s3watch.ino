// ============================================================================
//  S3 Watch — เฟิร์มแวร์นาฬิกา + เครื่องบันทึกเสียง (ไฟล์รวมเดียว — S3Watch.ino)
//  บอร์ด : Waveshare ESP32-S3-Touch-AMOLED-2.06 (ESP32-S3, PSRAM 8 MB)
//  Core  : Arduino-ESP32 3.x
//  ไลบรารี : GFX Library for Arduino, SensorLib (Lewis He), LVGL (ถ้า USE_LVGL=1)
//
//  ไฟล์นี้รวมมาจากหลายไฟล์ (config.h, hw.*, settings.*, display_1px.*, imu_wake.*,
//  touch.*, power.*, audio_codec.*, audio_rec.*, ota.*, ui_menu.*, modules.*)
//  เพื่อให้เป็น .ino ไฟล์เดียว — ลำดับภายในคือ:
//    1) ค่าคงที่ (เดิมคือ config.h)
//    2) ส่วนประกาศ (prototype/struct/enum) ของทุกโมดูล
//    3) ส่วนเขียนจริง (implementation) ของทุกโมดูล ตามลำดับ dependency
//    4) setup() / loop() (เดิมคือ S3Watch.ino)
//
//  แนวคิดหลัก — นาฬิกาคือแกน ห้ามถูกรบกวน
//    • หน้าปัดเส้น 1 พิกเซล (อนาล็อก/ดิจิทัล) อัปเดตแบบลบเฉพาะสิ่งที่เปลี่ยน
//    • เลือกโหมดหลับอัตโนมัติ : DEEP / LIGHT / ACTIVE
//    • หงายข้อมือ ±15° ค้าง 200 ms → เร่งแสง 10 วินาที แล้วหรี่
//    • อัดเสียงเป็น background : DMA → RAM → PSRAM → SD (WAV)
//    • ปุ่ม BOOT ค้าง 2 วินาที = เข้า/ออกเมนู Settings
//
//  ปุ่มและท่าทาง
//    BOOT ค้าง 2 วิ : เข้า/ออก Settings
//    ปัดขึ้น/ลง     : เลื่อนรายการ
//    ปัดขวา         : เลือก / ตกลง
//    ปัดซ้าย        : ย้อนกลับ / ออกจากเมนู
//    แตะ            : ปลุกจอ
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <ESP_I2S.h>
#include <Arduino_GFX_Library.h>
#include <SensorPCF85063.hpp>
#include <SensorQMI8658.hpp>
#include <Preferences.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_sleep.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"

// ต้องอยู่บนสุดของไฟล์ ไม่ใช่ตรงจุดที่ใช้จริง (บรรทัด ~1630/1637 เดิม):
// ตัวสร้าง prototype อัตโนมัติของ Arduino IDE ยกฟังก์ชันทุกตัวมาประกาศไว้ตรงนี้ก่อน
// เนื้อไฟล์ทั้งหมด ถ้า <lvgl.h> ยังไม่ถูก include และ struct MenuItem ยังไม่ถูกนิยาม
// ณ จุดนี้ prototype ที่ถูกยกมาจะอ้างชนิดที่ยังไม่รู้จัก
struct MenuItem;
#ifndef USE_LVGL
#define USE_LVGL 1
#endif
#if USE_LVGL
#include <lvgl.h>
#endif

// ============================== config.h ==============================
// ============================================================================
//  config.h — S3 Watch : พิน / ค่าคงที่ / สวิตช์เปิดปิดฟีเจอร์
//  บอร์ด: Waveshare ESP32-S3-Touch-AMOLED-2.06 (CO5300 410x502, QMI8658,
//         PCF85063, AXP2101, microSD, I2S mic)
// ============================================================================

#define FW_VERSION "0.2.0"

// ─────────────── สวิตช์คอมไพล์ ───────────────
// UI ของเมนู Settings : 1 = LVGL (ตามสเปก, ต้องติดตั้ง lvgl + lv_conf.h)
//                       0 = วาดเองด้วย Arduino_GFX (ไม่ต้องพึ่ง lvgl)
// ตรรกะเมนู/เจสเจอร์เหมือนกันทั้งสองแบบ ต่างกันแค่ตัววาด
#ifndef USE_LVGL
#define USE_LVGL 1
#endif

#ifndef ENABLE_AUDIO
#define ENABLE_AUDIO 1
#endif

#ifndef ENABLE_OTA
#define ENABLE_OTA 1
#endif

// 1 = ไม่หลับเลย (ดีบักผ่าน USB Serial ได้ เพราะ CDC จะหลุดเมื่อหลับ)
#ifndef DEBUG_NO_SLEEP
#define DEBUG_NO_SLEEP 0
#endif

// ─────────────── จอ AMOLED (QSPI) ───────────────
#define LCD_D0        4
#define LCD_D1        5
#define LCD_D2        6
#define LCD_D3        7
#define LCD_SCLK      11
#define LCD_CS        12
#define LCD_RST       8
#define LCD_W         410
#define LCD_H         502
#define LCD_COL_OFF   22     // ถ้าภาพเลื่อนซ้าย/ขวา ให้ปรับค่านี้

// ─────────────── I2C (RTC / IMU / Touch / PMU / Codec) ───────────────
#define I2C_SDA       15
#define I2C_SCL       14
#define I2C_FREQ      400000

#define ADDR_TOUCH    0x38   // FT3168 / FT6336 compatible
#define ADDR_PMU      0x34   // AXP2101 (ถ้าไม่มีจะข้ามอัตโนมัติ)
#define ADDR_ES7210   0x40
#define ADDR_ES8311   0x18

// ─────────────── ขาอินพุต ───────────────
#define PIN_TP_INT    38     // ⚠ ไม่ใช่ขา RTC → ปลุกจาก deep sleep ไม่ได้
#define PIN_IMU_INT   21     // ขา RTC → ปลุกได้ทั้ง light + deep sleep
#define PIN_BOOT      0      // ขา RTC → ปุ่มเดียวของระบบ (กดค้าง 2 วิ = Settings)

// ─────────────── microSD (SPI) ───────────────
#define SD_MOSI       1
#define SD_SCK        2
#define SD_MISO       3
#define SD_CS         17
#define SD_FREQ_HZ    20000000UL   // อย่าสูงกว่านี้ (ตามข้อควรระวังในเอกสาร)

// ─────────────── I2S ───────────────
#define I2S_MCLK      16
#define I2S_BCLK      41
#define I2S_WS        45
#define I2S_DOUT      40     // -> ES8311 (ลำโพง, ยังไม่ใช้)
#define I2S_DIN       42     // <- ไมโครโฟน
#define PIN_PA        46     // แอมป์ลำโพง (LOW = ปิด)

// ─────────────── ค่าเริ่มต้นของ Settings ───────────────
#define DEF_FACE_MODE        0        // 0 = อนาล็อก, 1 = ดิจิทัล
#define DEF_BRIGHT_HIGH_PCT  90
#define DEF_BRIGHT_DIM_PCT   25
#define DEF_BRIGHT_HOLD_MS   10000    // สว่าง 10 วินาทีแล้วหรี่
#define DEF_WRIST_WAKE       true
#define DEF_FACE_ANGLE_DEG   15       // ระนาบ ±15° → z > cos(15°) = 0.966
#define DEF_FACE_HOLD_MS     200
#define DEF_WOM_MG           150
#define DEF_MIC_CHIP         0        // 0 = ES7210, 1 = ES8311
#define DEF_SAMPLE_RATE      16000
#define DEF_FLUSH_PCT        20       // flush ลง SD เมื่อพื้นที่ว่างในริงเหลือ < 20%
#define DEF_RING_MB          4        // ริงบัฟเฟอร์ใน PSRAM (MB)
#define DEF_CPU_MHZ          80
#define DEF_TZ_MIN           420      // ไทย UTC+7

// ─────────────── ค่าคงที่ระบบ ───────────────
#define LONG_PRESS_MS        2000     // กดปุ่ม BOOT ค้าง = เข้า/ออก Settings
#define I2S_STAGE_BYTES      8192     // บัฟเฟอร์ขั้นกลางใน SRAM (DMA → RAM)
#define SD_CHUNK_BYTES       32768    // ขนาดก้อนที่เขียนลง SD ต่อครั้ง
#define REC_FILE_MAX_BYTES   (1500UL * 1024UL * 1024UL)  // ตัดไฟล์ที่ ~1.5 GB
#define SETTINGS_PATH        "/settings.cfg"
#define REC_DIR              "/rec"


// ============================================================================
//  ส่วนประกาศ (prototypes / struct / enum) — รวมจากไฟล์ .h ทั้งหมด
// ============================================================================

// ---------- settings.h ----------
// ============================================================================
//  settings.h — ค่าตั้งทั้งหมดของเครื่อง
//  เก็บหลักที่ SD card (/settings.cfg แบบ key=value แก้ด้วย Notepad ได้)
//  มิเรอร์ลง NVS ด้วย → ถอด SD ออกแล้วเครื่องยังจำค่าเดิมและรันต่อได้
// ============================================================================
#include <Arduino.h>

struct Settings {
  // หน้าปัด + จอ
  uint8_t  faceMode;         // 0 = analog, 1 = digital
  uint8_t  brightHighPct;    // ความสว่างตอนหงายข้อมือ (%)
  uint8_t  brightDimPct;     // ความสว่างปกติ (%)
  uint16_t brightHoldMs;     // สว่างค้างกี่ ms ก่อนหรี่

  // หงายข้อมือ
  bool     wristWake;        // เปิด/ปิดฟีเจอร์ (ปิด = ใช้ deep sleep ได้)
  uint8_t  faceAngleDeg;     // ระนาบ ±กี่องศาถือว่าหงาย
  uint16_t faceHoldMs;       // ต้องค้างกี่ ms ก่อนเร่งแสง
  uint16_t womThresholdMg;   // ความไวของ Wake-on-Motion

  // บันทึกเสียง
  uint8_t  micChip;          // 0 = ES7210, 1 = ES8311
  uint32_t sampleRate;       // 8000 / 16000 / 32000
  uint8_t  flushPct;         // flush ลง SD เมื่อริงว่างเหลือ < กี่ %
  uint8_t  ringMB;           // ขนาดริงบัฟเฟอร์ใน PSRAM (MB)

  // เครือข่าย / OTA
  bool     wifiEnabled;
  char     wifiSsid[33];
  char     wifiPass[65];
  char     otaUrl[160];      // URL ของไฟล์ manifest บน GitHub

  // ระบบ
  int16_t  tzMinutes;        // เขตเวลา (นาที) ไทย = 420
  uint16_t cpuMhz;           // 80 / 160 / 240
};

extern Settings settings;

void settingsDefaults();                 // เขียนค่าเริ่มต้นลงโครงสร้าง
bool settingsLoad();                     // SD → ถ้าไม่ได้ ลอง NVS → ถ้าไม่ได้ ใช้ default
bool settingsSave();                     // เขียนทั้ง SD และ NVS
void settingsClampAll();                 // กันค่าที่ผู้ใช้แก้จนเกินช่วง
float settingsFaceUpZ();                 // cos(faceAngleDeg) สำหรับเทียบแกน Z


// ---------- hw.h ----------
// ============================================================================
//  hw.h — อุปกรณ์ทั้งหมดบนบอร์ด + การเริ่มต้นใช้งาน
// ============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include <SensorPCF85063.hpp>
#include <SensorQMI8658.hpp>
#include <time.h>

extern Arduino_CO5300  *gfx;
extern SensorPCF85063   rtc;
extern SensorQMI8658    imu;

// ── init ──
void hwInitPins();
void hwInitI2C();
void hwInitPmu();                       // AXP2101 (ตรวจว่ามีหรือไม่ — ดู PMU_FORCE_RAILS)
bool hwInitDisplay(bool keepImage);     // keepImage = true ตอนตื่นจาก deep sleep
bool hwInitRtc();
bool hwInitImu();
bool hwInitSd();

// ── สถานะ ──
bool hwRtcOk();
bool hwImuOk();
bool hwSdOk();
bool hwPmuOk();

// ── จอ ──
void hwSetBrightnessPct(uint8_t pct);
uint8_t hwBrightnessPct();

// ── เวลา ──
void hwTimeFromRtc();                   // RTC → system time
void hwTimeToRtc();                     // system time → RTC (ใช้หลัง NTP)
void hwNowLocal(struct tm &out);

// ── I2C helper ──
bool i2cPresent(uint8_t addr);
bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val);
bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len);


// ---------- display_1px.h ----------
// ============================================================================
//  display_1px.h — หน้าปัด "เส้น 1 พิกเซล" ทั้งอนาล็อกและดิจิทัล
//  หลักการ: อัปเดตทุกนาทีโดย "ลบเฉพาะสิ่งที่เปลี่ยน" ไม่ล้างทั้งจอ
//           → ส่งข้อมูลผ่าน QSPI น้อย → MCU ตื่นสั้น → ประหยัดไฟ
// ============================================================================
#include <Arduino.h>
#include <time.h>

void faceInvalidate();                         // บังคับให้วาดใหม่ทั้งจอครั้งถัดไป
void faceDrawFull(const struct tm &t);         // ล้างจอ + วาดใหม่ทั้งหมด
void faceUpdate(const struct tm &t);           // วาดเฉพาะส่วนที่เปลี่ยน
void faceSetRecording(bool on);                // จุดแดงมุมขวาบน = กำลังอัดเสียง
void faceShowToast(const char *msg);           // ข้อความสั้น ๆ กลางจอ (เช่นผลการอัปเดต)


// ---------- imu_wake.h ----------
// ============================================================================
//  imu_wake.h — QMI8658 : Wake-on-Motion + ตรวจ "หงายข้อมือ"
//  เงื่อนไขหงาย: หน้าจอชี้ขึ้นภายใน ±faceAngleDeg (ค่าเริ่มต้น 15°)
//                และต้องค้างต่อเนื่อง faceHoldMs (ค่าเริ่มต้น 200 ms)
// ============================================================================
#include <Arduino.h>

void imuConfigureWom();        // ตั้ง Wake-on-Motion ให้ INT ปลุก MCU
bool imuFaceUpNow();           // อ่านค่าครั้งเดียว
bool imuFaceUpHeld();          // ต้องเข้าเงื่อนไขค้างครบเวลาถึงจะคืน true
int  imuIntIdleLevel();        // ระดับลอจิกของขา INT ตอนไม่มีการขยับ
void imuLatchIntIdle();        // จำระดับปัจจุบันไว้ก่อนเข้า sleep


// ---------- touch.h ----------
// ============================================================================
//  touch.h — จอสัมผัส (FT3168 / FT6336 register map, I2C 0x38)
//  ท่าทางตามสเปก: ปัดขึ้น/ลง = เลื่อนเมนู, ปัดขวา = เลือก/ตกลง,
//                 ปัดซ้าย = ย้อนกลับ, แตะ = ปลุกจอ
// ============================================================================
#include <Arduino.h>

enum Gesture : uint8_t {
  GST_NONE = 0,
  GST_TAP,
  GST_UP,
  GST_DOWN,
  GST_LEFT,
  GST_RIGHT
};

bool    touchInit();
bool    touchPressed(int &x, int &y);   // true ถ้ามีนิ้วแตะอยู่ตอนนี้
Gesture touchPoll();                    // เรียกถี่ ๆ ตอนตื่น — คืนท่าทางเมื่อปล่อยนิ้ว
bool    touchOk();


// ---------- power.h ----------
// ============================================================================
//  power.h — เลือกโหมดพลังงานอัตโนมัติตามสเปก
//
//    ไม่อัดเสียง + ปิดหงายข้อมือ   → DEEP   (ตื่นด้วย timer ต้นนาที / ปุ่ม BOOT)
//    ไม่อัดเสียง + เปิดหงายข้อมือ  → LIGHT  (ตื่นด้วย timer / IMU INT / สัมผัส / ปุ่ม)
//    กำลังอัดเสียง                 → ACTIVE (ห้ามหลับ — I2S ต้องมี clock ต่อเนื่อง)
//
//  ⚠ ขา TP_INT (GPIO38) ไม่ใช่ขา RTC จึงปลุกจาก deep sleep ไม่ได้
//    ในโหมด DEEP การแตะจอจะไม่ปลุกเครื่อง — ใช้ปุ่ม BOOT แทน
// ============================================================================
#include <Arduino.h>

enum PowerMode : uint8_t { PM_ACTIVE = 0, PM_LIGHT, PM_DEEP };
enum WakeReason : uint8_t { WK_NONE = 0, WK_TIMER, WK_MOTION, WK_TOUCH, WK_BUTTON };

PowerMode  powerPickMode(bool recording, bool uiOpen);
WakeReason powerSleep(PowerMode mode, uint32_t brightUntilMs);
const char *powerModeName(PowerMode m);
bool       powerWokeFromDeepSleep();     // เรียกได้ใน setup()


// ---------- audio_codec.h ----------
// ============================================================================
//  audio_codec.h — เปิด/ปิดชิปไมโครโฟน
//  บอร์ดรุ่นนี้ยังไม่ยืนยันว่าใช้ ES7210 หรือ ES8311 → เลือกได้ใน Settings
//  ⚠ ชุดรีจิสเตอร์ทั้งสองชุดเรียบเรียงจาก driver ของ Espressif — ยังไม่ทดสอบ
// ============================================================================
#include <Arduino.h>

bool codecMicStart(uint32_t sampleRate);   // เปิดไมค์ตามชิปที่เลือกใน Settings
void codecMicStop();                       // ปิดไมค์ + เข้าสลีป (ไม่กินไฟ)
bool codecDetect(uint8_t &foundChip);      // 0 = ES7210, 1 = ES8311 (คืน false ถ้าไม่เจอ)


// ---------- audio_rec.h ----------
// ============================================================================
//  audio_rec.h — บันทึกเสียงตามเส้นทาง  DMA → RAM → PSRAM → SD
//
//    I2S DMA  ──► บัฟเฟอร์ขั้นกลางใน SRAM (8 KB)
//                 ──► ริงบัฟเฟอร์ใน PSRAM (ค่าเริ่มต้น 4 MB)
//                      ──► เขียนลง SD เป็นไฟล์ WAV เมื่อพื้นที่ว่างเหลือ < 20%
//
//  ตัวอ่าน (core 0) และตัวเขียน (core 1) เป็นคนละ task
//  → ระหว่างเขียน SD ก้อนใหญ่ เสียงยังถูกอ่านต่อเนื่อง ไม่ขาดช่วง
// ============================================================================
#include <Arduino.h>

bool   audioInit();                 // จอง PSRAM + สร้าง task (เรียกครั้งเดียวใน setup)
bool   audioStart();                // เริ่มอัด (false = ไม่มี SD / จองบัฟเฟอร์ไม่ได้)
void   audioStop();                 // หยุดอัด + เขียนที่ค้างในริงลง SD ให้ครบ
bool   audioIsRecording();
bool   audioIsBusy();               // กำลังเขียน SD อยู่
uint8_t audioRingUsedPct();
uint64_t audioBytesWritten();
const char *audioLastError();
const char *audioCurrentFile();


// ---------- ota.h ----------
// ============================================================================
//  ota.h — WiFi + ตรวจเวอร์ชันจาก GitHub + อัปเดตเฟิร์มแวร์
//
//  WiFi เปิดเฉพาะตอนสั่งอัปเดต/ตั้งเวลา แล้วปิดทันที (ประหยัดไฟ)
//
//  ไฟล์ manifest ที่ otaUrl ชี้ไป เป็นข้อความธรรมดา 2 บรรทัด:
//      version=0.2.1
//      url=https://github.com/<user>/<repo>/releases/download/v0.2.1/S3Watch.bin
// ============================================================================
#include <Arduino.h>

typedef void (*OtaProgressCb)(const char *msg);

bool otaWifiConnect(uint32_t timeoutMs = 15000);
void otaWifiOff();
bool otaSyncNtp(OtaProgressCb cb);            // ตั้งเวลาจากอินเทอร์เน็ต → เขียนลง RTC
bool otaCheckAndUpdate(OtaProgressCb cb);     // ตรวจ + อัปเดต (รีบูตเองถ้าสำเร็จ)


// ---------- ui_menu.h ----------
// ============================================================================
//  ui_menu.h — เมนู Settings
//  เข้า/ออก: กดปุ่ม BOOT ค้าง 2 วินาที
//  ท่าทาง : ปัดขึ้น/ลง = เลื่อนรายการ, ปัดขวา = เลือก/ตกลง, ปัดซ้าย = ย้อนกลับ
//
//  ตรรกะของเมนูเป็นชุดเดียว ส่วนการวาดเลือกได้ 2 แบบด้วย USE_LVGL ใน config.h
//  ⚠ ป้ายเมนูใช้อักษรอังกฤษ เพราะฟอนต์เริ่มต้นของทั้ง Arduino_GFX และ LVGL
//    ไม่มีตัวอักษรไทย
// ============================================================================
#include <Arduino.h>

void uiInit();
void uiOpen();
void uiClose();
bool uiIsOpen();
void uiGesture(Gesture g);
void uiTick();                 // เรียกทุกลูปตอนเมนูเปิดอยู่
void uiToast(const char *msg); // ข้อความสถานะ (ใช้เป็น callback ของ OTA ได้)


// ---------- modules.h ----------
// ============================================================================
//  modules.h — จุดเสียบฟีเจอร์ใหม่ในอนาคต (n-lang, RD4 event, 8D index, STT)
//  เพิ่มฟังก์ชันใหม่ได้โดยไม่ต้องแตะแกนนาฬิกาเลย — แค่ใส่รายการใน MODULES[]
//
//  hook 3 จุด:
//     init()          — ตอนบูต
//     onMinute(tm)    — ทุกครั้งที่นาทีเปลี่ยน (MCU ตื่นอยู่แล้ว)
//     onRaise()       — ทุกครั้งที่ผู้ใช้หงายข้อมือ
// ============================================================================
#include <Arduino.h>
#include <time.h>

struct Module {
  const char *name;
  void (*init)();
  void (*onMinute)(const struct tm &t);
  void (*onRaise)();
};

void modulesInit();
void modulesMinute(const struct tm &t);
void modulesRaise();


// ============================================================================
//  ส่วนเขียนจริง (implementation) — รวมจากไฟล์ .cpp ทั้งหมด
// ============================================================================

// ---------- hw.cpp ----------
#include <SPI.h>
#include <SD.h>
#include <sys/time.h>

// ⚠ 1 = สั่งเปิดรางไฟของ AXP2101 เอง (แตะเฉพาะบิต enable ไม่แตะค่าแรงดัน)
//    ปกติปล่อยเป็น 0 ไว้ก่อน — เปิดเฉพาะกรณีจอไม่ติดเลย
#ifndef PMU_FORCE_RAILS
#define PMU_FORCE_RAILS 0
#endif

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_D0, LCD_D1, LCD_D2, LCD_D3);
// ลำดับพารามิเตอร์ตามเวอร์ชันไลบรารีที่ติดตั้งจริง (ไม่มี ips):
// (bus, rst, rotation, w, h, col_offset1, row_offset1, col_offset2, row_offset2)
Arduino_CO5300 *gfx = new Arduino_CO5300(bus, LCD_RST, 0 /*rotation*/,
                                         LCD_W, LCD_H, LCD_COL_OFF, 0, 0, 0);
SensorPCF85063 rtc;
SensorQMI8658  imu;

static bool s_rtcOk = false, s_imuOk = false, s_sdOk = false, s_pmuOk = false;
static uint8_t s_brightPct = 0;
static SPIClass sdSpi(HSPI);

bool hwRtcOk() { return s_rtcOk; }
bool hwImuOk() { return s_imuOk; }
bool hwSdOk()  { return s_sdOk;  }
bool hwPmuOk() { return s_pmuOk; }

// ─────────────── I2C helper ───────────────
bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)len) != (int)len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// ─────────────── พิน ───────────────
void hwInitPins() {
  pinMode(PIN_BOOT,    INPUT_PULLUP);
  pinMode(PIN_TP_INT,  INPUT_PULLUP);
  pinMode(PIN_IMU_INT, INPUT);
  pinMode(PIN_PA,      OUTPUT);
  digitalWrite(PIN_PA, LOW);        // ปิดแอมป์ลำโพงไว้เสมอ (ประหยัดไฟ)
}

void hwInitI2C() {
  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
  delay(5);
}

// ─────────────── AXP2101 ───────────────
void hwInitPmu() {
  s_pmuOk = i2cPresent(ADDR_PMU);
  if (!s_pmuOk) { Serial.println(F("[pmu] ไม่พบ AXP2101 — ข้าม")); return; }

  uint8_t ldoEn = 0;
  i2cRead(ADDR_PMU, 0x90, &ldoEn, 1);
  Serial.printf("[pmu] AXP2101 พบแล้ว, LDO enable (0x90) = 0x%02X\n", ldoEn);

#if PMU_FORCE_RAILS
  // เปิด ALDO1..4 + BLDO1..2 โดย "ไม่แตะค่าแรงดัน" ที่บอร์ดตั้งไว้แล้ว
  i2cWrite8(ADDR_PMU, 0x90, ldoEn | 0x3F);
  delay(20);
  Serial.println(F("[pmu] บังคับเปิดราง LDO แล้ว"));
#endif
}

// ─────────────── จอ ───────────────
bool hwInitDisplay(bool keepImage) {
  if (!gfx->begin()) {
    Serial.println(F("[gfx] begin() ล้มเหลว"));
    return false;
  }
  if (!keepImage) gfx->fillScreen(RGB565_BLACK);
  hwSetBrightnessPct(settings.brightDimPct);
  return true;
}

void hwSetBrightnessPct(uint8_t pct) {
  if (pct > 100) pct = 100;
  s_brightPct = pct;
  gfx->setBrightness((uint8_t)((uint16_t)pct * 255 / 100));
}

uint8_t hwBrightnessPct() { return s_brightPct; }

// ─────────────── RTC (PCF85063) ───────────────
bool hwInitRtc() {
  s_rtcOk = rtc.begin(Wire, I2C_SDA, I2C_SCL);
  if (!s_rtcOk) { Serial.println(F("[rtc] ไม่พบ PCF85063")); return false; }
  hwTimeFromRtc();

  // ถ้า RTC ยังไม่เคยถูกตั้ง → ใช้เวลาคอมไพล์ตั้งให้ครั้งแรก
  struct tm t; hwNowLocal(t);
  if (t.tm_year + 1900 < 2025) {
    char mon[4] = {0}; int d = 1, y = 2025, hh = 0, mm = 0, ss = 0;
    sscanf(__DATE__, "%3s %d %d", mon, &d, &y);
    sscanf(__TIME__, "%d:%d:%d", &hh, &mm, &ss);
    static const char *MONS = "JanFebMarAprMayJunJulAugSepOctNovDec";
    const char *p = strstr(MONS, mon);
    int m = p ? (int)((p - MONS) / 3) + 1 : 1;
    rtc.setDateTime(y, m, d, hh, mm, ss);
    hwTimeFromRtc();
    Serial.printf("[rtc] ตั้งเวลาจาก compile time: %04d-%02d-%02d %02d:%02d:%02d\n", y, m, d, hh, mm, ss);
  }
  return true;
}

void hwTimeFromRtc() {
  if (!s_rtcOk) return;
  RTC_DateTime d = rtc.getDateTime();
  struct tm t = {};
  t.tm_year = d.getYear() - 1900;
  t.tm_mon  = d.getMonth() - 1;
  t.tm_mday = d.getDay();
  t.tm_hour = d.getHour();
  t.tm_min  = d.getMinute();
  t.tm_sec  = d.getSecond();
  t.tm_isdst = 0;
  time_t sec = mktime(&t);
  if (sec < 0) return;
  struct timeval tv = { .tv_sec = sec, .tv_usec = 0 };
  settimeofday(&tv, nullptr);
}

void hwTimeToRtc() {
  if (!s_rtcOk) return;
  struct tm t; hwNowLocal(t);
  rtc.setDateTime(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
}

void hwNowLocal(struct tm &out) {
  // เก็บ system time เป็น "เวลาท้องถิ่น" ตรง ๆ (TZ = UTC) เพื่อไม่ให้ offset ซ้อนกัน
  time_t now = time(nullptr);
  localtime_r(&now, &out);
}

// ─────────────── IMU (QMI8658) ───────────────
bool hwInitImu() {
  s_imuOk = imu.begin(Wire, QMI8658_L_SLAVE_ADDRESS, I2C_SDA, I2C_SCL);
  if (!s_imuOk) { Serial.println(F("[imu] ไม่พบ QMI8658 — ปิดฟีเจอร์หงายข้อมือ")); return false; }
  imu.configAccelerometer(SensorQMI8658::ACC_RANGE_2G,
                          SensorQMI8658::ACC_ODR_LOWPOWER_21Hz,
                          SensorQMI8658::LPF_MODE_0);
  imu.enableAccelerometer();
  Serial.println(F("[imu] QMI8658 พร้อม"));
  return true;
}

// ─────────────── microSD ───────────────
bool hwInitSd() {
  sdSpi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  s_sdOk = SD.begin(SD_CS, sdSpi, SD_FREQ_HZ);
  if (!s_sdOk) { Serial.println(F("[sd] ไม่พบการ์ด — ใช้ค่าจาก NVS และอัดเสียงไม่ได้")); return false; }
  if (!SD.exists(REC_DIR)) SD.mkdir(REC_DIR);
  Serial.printf("[sd] พร้อม, ขนาด %llu MB\n", SD.cardSize() / (1024ULL * 1024ULL));
  return true;
}


// ---------- settings.cpp ----------
#include <SD.h>
#include <Preferences.h>
#include <math.h>

Settings settings;
static Preferences prefs;

// ─────────────────────────────────────────────────────────────
void settingsDefaults() {
  memset(&settings, 0, sizeof(settings));
  settings.faceMode       = DEF_FACE_MODE;
  settings.brightHighPct  = DEF_BRIGHT_HIGH_PCT;
  settings.brightDimPct   = DEF_BRIGHT_DIM_PCT;
  settings.brightHoldMs   = DEF_BRIGHT_HOLD_MS;
  settings.wristWake      = DEF_WRIST_WAKE;
  settings.faceAngleDeg   = DEF_FACE_ANGLE_DEG;
  settings.faceHoldMs     = DEF_FACE_HOLD_MS;
  settings.womThresholdMg = DEF_WOM_MG;
  settings.micChip        = DEF_MIC_CHIP;
  settings.sampleRate     = DEF_SAMPLE_RATE;
  settings.flushPct       = DEF_FLUSH_PCT;
  settings.ringMB         = DEF_RING_MB;
  settings.wifiEnabled    = false;
  settings.wifiSsid[0]    = 0;
  settings.wifiPass[0]    = 0;
  settings.otaUrl[0]      = 0;
  settings.tzMinutes      = DEF_TZ_MIN;
  settings.cpuMhz         = DEF_CPU_MHZ;
}

void settingsClampAll() {
  settings.faceMode      = settings.faceMode ? 1 : 0;
  settings.brightHighPct = constrain(settings.brightHighPct, (uint8_t)5, (uint8_t)100);
  settings.brightDimPct  = constrain(settings.brightDimPct,  (uint8_t)0, (uint8_t)100);
  if (settings.brightDimPct > settings.brightHighPct) settings.brightDimPct = settings.brightHighPct;
  settings.brightHoldMs  = constrain(settings.brightHoldMs, (uint16_t)1000, (uint16_t)60000);
  settings.faceAngleDeg  = constrain(settings.faceAngleDeg, (uint8_t)5, (uint8_t)60);
  settings.faceHoldMs    = constrain(settings.faceHoldMs, (uint16_t)0, (uint16_t)2000);
  settings.womThresholdMg= constrain(settings.womThresholdMg, (uint16_t)20, (uint16_t)1000);
  settings.micChip       = settings.micChip ? 1 : 0;
  if (settings.sampleRate != 8000 && settings.sampleRate != 16000 && settings.sampleRate != 32000)
    settings.sampleRate = DEF_SAMPLE_RATE;
  settings.flushPct      = constrain(settings.flushPct, (uint8_t)5, (uint8_t)50);
  settings.ringMB        = constrain(settings.ringMB, (uint8_t)1, (uint8_t)6);
  settings.tzMinutes     = constrain(settings.tzMinutes, (int16_t)-720, (int16_t)840);
  if (settings.cpuMhz != 80 && settings.cpuMhz != 160 && settings.cpuMhz != 240)
    settings.cpuMhz = DEF_CPU_MHZ;
}

float settingsFaceUpZ() {
  return cosf((float)settings.faceAngleDeg * (float)DEG_TO_RAD);
}

// ─────────────── ตัวช่วยอ่านไฟล์ key=value ───────────────
static void applyKV(const String &key, const String &val) {
  if      (key == "faceMode")     settings.faceMode       = val.toInt();
  else if (key == "brightHigh")   settings.brightHighPct  = val.toInt();
  else if (key == "brightDim")    settings.brightDimPct   = val.toInt();
  else if (key == "brightHoldMs") settings.brightHoldMs   = val.toInt();
  else if (key == "wristWake")    settings.wristWake      = (val.toInt() != 0);
  else if (key == "faceAngleDeg") settings.faceAngleDeg   = val.toInt();
  else if (key == "faceHoldMs")   settings.faceHoldMs     = val.toInt();
  else if (key == "womMg")        settings.womThresholdMg = val.toInt();
  else if (key == "micChip")      settings.micChip        = val.toInt();
  else if (key == "sampleRate")   settings.sampleRate     = val.toInt();
  else if (key == "flushPct")     settings.flushPct       = val.toInt();
  else if (key == "ringMB")       settings.ringMB         = val.toInt();
  else if (key == "wifiEnabled")  settings.wifiEnabled    = (val.toInt() != 0);
  else if (key == "wifiSsid")     strlcpy(settings.wifiSsid, val.c_str(), sizeof(settings.wifiSsid));
  else if (key == "wifiPass")     strlcpy(settings.wifiPass, val.c_str(), sizeof(settings.wifiPass));
  else if (key == "otaUrl")       strlcpy(settings.otaUrl,  val.c_str(), sizeof(settings.otaUrl));
  else if (key == "tzMinutes")    settings.tzMinutes      = val.toInt();
  else if (key == "cpuMhz")       settings.cpuMhz         = val.toInt();
}

static bool loadFromSd() {
  if (!hwSdOk()) return false;
  File f = SD.open(SETTINGS_PATH, FILE_READ);
  if (!f) return false;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0 || line[0] == '#') continue;
    int eq = line.indexOf('=');
    if (eq <= 0) continue;
    String k = line.substring(0, eq);      k.trim();
    String v = line.substring(eq + 1);     v.trim();
    applyKV(k, v);
  }
  f.close();
  return true;
}

static bool saveToSd() {
  if (!hwSdOk()) return false;
  File f = SD.open(SETTINGS_PATH, FILE_WRITE);   // FILE_WRITE = ตัดไฟล์เดิมทิ้ง
  if (!f) return false;
  f.printf("# S3 Watch settings (fw %s) — แก้ไฟล์นี้ด้วย Notepad ได้เลย\n", FW_VERSION);
  f.printf("faceMode=%u\n",     settings.faceMode);
  f.printf("brightHigh=%u\n",   settings.brightHighPct);
  f.printf("brightDim=%u\n",    settings.brightDimPct);
  f.printf("brightHoldMs=%u\n", settings.brightHoldMs);
  f.printf("wristWake=%u\n",    settings.wristWake ? 1 : 0);
  f.printf("faceAngleDeg=%u\n", settings.faceAngleDeg);
  f.printf("faceHoldMs=%u\n",   settings.faceHoldMs);
  f.printf("womMg=%u\n",        settings.womThresholdMg);
  f.printf("micChip=%u\n",      settings.micChip);
  f.printf("sampleRate=%lu\n",  (unsigned long)settings.sampleRate);
  f.printf("flushPct=%u\n",     settings.flushPct);
  f.printf("ringMB=%u\n",       settings.ringMB);
  f.printf("wifiEnabled=%u\n",  settings.wifiEnabled ? 1 : 0);
  f.printf("wifiSsid=%s\n",     settings.wifiSsid);
  f.printf("wifiPass=%s\n",     settings.wifiPass);
  f.printf("otaUrl=%s\n",       settings.otaUrl);
  f.printf("tzMinutes=%d\n",    settings.tzMinutes);
  f.printf("cpuMhz=%u\n",       settings.cpuMhz);
  f.close();
  return true;
}

// ─────────────── NVS (สำรองตอนไม่มี SD) ───────────────
static bool loadFromNvs() {
  if (!prefs.begin("s3watch", true)) return false;
  size_t n = prefs.getBytesLength("cfg");
  bool ok = false;
  if (n == sizeof(Settings)) {
    prefs.getBytes("cfg", &settings, sizeof(Settings));
    ok = true;
  }
  prefs.end();
  return ok;
}

static bool saveToNvs() {
  if (!prefs.begin("s3watch", false)) return false;
  prefs.putBytes("cfg", &settings, sizeof(Settings));
  prefs.end();
  return true;
}

// ─────────────── API ───────────────
bool settingsLoad() {
  settingsDefaults();
  bool fromSd = loadFromSd();
  if (!fromSd) {
    if (loadFromNvs()) {
      Serial.println(F("[settings] โหลดจาก NVS (ไม่พบ SD/ไฟล์ตั้งค่า)"));
    } else {
      Serial.println(F("[settings] ใช้ค่าเริ่มต้น"));
      settingsClampAll();
      saveToNvs();
      saveToSd();          // สร้างไฟล์ตัวอย่างไว้บน SD ให้ผู้ใช้แก้ได้
      return false;
    }
  } else {
    Serial.println(F("[settings] โหลดจาก SD"));
  }
  settingsClampAll();
  if (fromSd) saveToNvs();  // มิเรอร์ค่าจาก SD ลง NVS ทุกครั้งที่บูต
  return true;
}

bool settingsSave() {
  settingsClampAll();
  bool a = saveToNvs();
  bool b = saveToSd();
  return a || b;
}


// ---------- display_1px.cpp ----------
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


// ---------- imu_wake.cpp ----------
#include <math.h>

// ถ้าขยับแล้วไม่ตื่น ให้ลองเปลี่ยนเป็น INTERRUPT_PIN_2
#define IMU_INT_SEL SensorQMI8658::INTERRUPT_PIN_1
// ถ้าหงายจอแล้วไม่สว่าง (แกน Z กลับด้านบนบอร์ดจริง) ให้เปลี่ยนเป็น -1.0f
#define FACE_SIGN   1.0f

RTC_DATA_ATTR static int s_intIdle = 1;

void imuConfigureWom() {
  if (!hwImuOk() || !settings.wristWake) return;
  // หมายเหตุ: อาร์กิวเมนต์ของ configWakeOnMotion ต่างกันตามเวอร์ชันของ SensorLib
  imu.configWakeOnMotion(settings.womThresholdMg,
                         SensorQMI8658::ACC_ODR_LOWPOWER_21Hz,
                         IMU_INT_SEL);
  imuLatchIntIdle();
}

int imuIntIdleLevel() { return s_intIdle; }

void imuLatchIntIdle() {
  s_intIdle = gpio_get_level((gpio_num_t)PIN_IMU_INT);
}

bool imuFaceUpNow() {
  if (!hwImuOk()) return false;           // IMU เสีย → ไม่ปลุกด้วยท่าทาง
  float x, y, z;
  if (!imu.getAccelerometer(x, y, z)) return false;
  z *= FACE_SIGN;
  float mag = sqrtf(x * x + y * y + z * z);
  if (mag < 0.5f) return false;           // ค่าผิดปกติ
  return (z / mag) > settingsFaceUpZ();   // มุมเอียงจากแนวราบ < faceAngleDeg
}

bool imuFaceUpHeld() {
  if (!hwImuOk()) return false;
  uint16_t hold = settings.faceHoldMs;
  uint32_t t0 = millis();
  uint32_t deadline = t0 + hold + 150;    // ให้เวลาเผื่ออีกเล็กน้อย
  uint32_t okSince = 0;
  while (millis() < deadline) {
    if (imuFaceUpNow()) {
      if (okSince == 0) okSince = millis();
      if (millis() - okSince >= hold) return true;
    } else {
      okSince = 0;                        // หลุดเงื่อนไข → เริ่มนับใหม่
    }
    delay(10);
  }
  return hold == 0 ? imuFaceUpNow() : false;
}


// ---------- touch.cpp ----------

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


// ---------- power.cpp ----------
#include <sys/time.h>

const char *powerModeName(PowerMode m) {
  switch (m) {
    case PM_ACTIVE: return "ACTIVE";
    case PM_LIGHT:  return "LIGHT";
    default:        return "DEEP";
  }
}

bool powerWokeFromDeepSleep() {
  return esp_reset_reason() == ESP_RST_DEEPSLEEP;
}

PowerMode powerPickMode(bool recording, bool uiOpen) {
  if (recording || uiOpen) return PM_ACTIVE;
  return settings.wristWake ? PM_LIGHT : PM_DEEP;
}

// เวลาที่เหลือจนถึงต้นนาทีถัดไป (ไมโครวินาที) — เผื่อ 20 ms กันตื่นก่อนวินาที 0
static uint64_t usUntilNextMinute() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  int64_t us = (int64_t)(60 - (tv.tv_sec % 60)) * 1000000LL - tv.tv_usec + 20000LL;
  if (us < 200000LL) us += 60000000LL;      // ใกล้เกินไป → ข้ามไปนาทีถัดไป
  return (uint64_t)us;
}

static uint64_t sleepBudgetUs(uint32_t brightUntilMs) {
  uint64_t us = usUntilNextMinute();
  if (brightUntilMs) {
    uint32_t now = millis();
    uint64_t leftUs = (brightUntilMs > now) ? (uint64_t)(brightUntilMs - now) * 1000ULL : 0;
    if (leftUs + 1000ULL < us) us = leftUs + 1000ULL;   // ตื่นมาหรี่จอให้ตรงเวลา
  }
  return us;
}

WakeReason powerSleep(PowerMode mode, uint32_t brightUntilMs) {
#if DEBUG_NO_SLEEP
  delay(50);
  return WK_NONE;
#endif

  if (mode == PM_ACTIVE) { delay(20); return WK_NONE; }

  uint64_t us = sleepBudgetUs(brightUntilMs);

  // ───────── DEEP SLEEP ─────────
  if (mode == PM_DEEP) {
    esp_sleep_enable_timer_wakeup(us);
    // ปุ่ม BOOT (GPIO0) เป็นขา RTC → ใช้ ext1 ปลุกได้
    rtc_gpio_pullup_en((gpio_num_t)PIN_BOOT);
    rtc_gpio_pulldown_dis((gpio_num_t)PIN_BOOT);
    esp_sleep_enable_ext1_wakeup(1ULL << PIN_BOOT, ESP_EXT1_WAKEUP_ANY_LOW);
    Serial.flush();
    esp_deep_sleep_start();      // ไม่กลับมาที่บรรทัดนี้ — บูตใหม่เข้าสู่ setup()
  }

  // ───────── LIGHT SLEEP ─────────
  esp_sleep_enable_timer_wakeup(us);

  if (hwImuOk() && settings.wristWake) {
    // ขา INT สลับระดับเมื่อขยับ → ตั้งให้ตื่นที่ระดับตรงข้ามกับตอนนิ่ง
    imuLatchIntIdle();
    gpio_wakeup_enable((gpio_num_t)PIN_IMU_INT,
                       imuIntIdleLevel() ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL);
  }
  gpio_wakeup_enable((gpio_num_t)PIN_BOOT,   GPIO_INTR_LOW_LEVEL);
  gpio_wakeup_enable((gpio_num_t)PIN_TP_INT, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();

  esp_light_sleep_start();

  // ───────── ตื่นแล้ว — แยกสาเหตุ ─────────
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  gpio_wakeup_disable((gpio_num_t)PIN_BOOT);
  gpio_wakeup_disable((gpio_num_t)PIN_TP_INT);
  if (hwImuOk() && settings.wristWake) gpio_wakeup_disable((gpio_num_t)PIN_IMU_INT);

  if (cause == ESP_SLEEP_WAKEUP_TIMER) return WK_TIMER;
  if (cause != ESP_SLEEP_WAKEUP_GPIO)  return WK_NONE;

  if (digitalRead(PIN_BOOT) == LOW)    return WK_BUTTON;
  if (digitalRead(PIN_TP_INT) == LOW)  return WK_TOUCH;
  if (hwImuOk() && gpio_get_level((gpio_num_t)PIN_IMU_INT) != imuIntIdleLevel()) return WK_MOTION;
  return WK_NONE;
}


// ---------- audio_codec.cpp ----------

// ───────────────────────── ES7210 (ADC 4 ช่อง) ─────────────────────────
static void es7210Start(uint32_t rate) {
  const uint8_t a = ADDR_ES7210;
  i2cWrite8(a, 0x00, 0xFF); delay(10);      // reset
  i2cWrite8(a, 0x00, 0x41);
  i2cWrite8(a, 0x01, 0x3F);
  i2cWrite8(a, 0x09, 0x30); i2cWrite8(a, 0x0A, 0x30);
  i2cWrite8(a, 0x23, 0x2A); i2cWrite8(a, 0x22, 0x0A);
  i2cWrite8(a, 0x20, 0x0A); i2cWrite8(a, 0x21, 0x2A);
  i2cWrite8(a, 0x08, 0x00);                 // slave mode
  i2cWrite8(a, 0x40, 0x43); i2cWrite8(a, 0x41, 0x70); i2cWrite8(a, 0x42, 0x70);
  i2cWrite8(a, 0x07, 0x20);
  // ตัวหารนาฬิกา: MCLK = 256 × fs
  uint8_t clkdiv = (rate == 8000) ? 0x82 : (rate == 32000 ? 0x80 : 0x81);
  i2cWrite8(a, 0x02, clkdiv);
  i2cWrite8(a, 0x03, 0x00); i2cWrite8(a, 0x04, 0x01); i2cWrite8(a, 0x05, 0x00);
  i2cWrite8(a, 0x11, 0x60); i2cWrite8(a, 0x12, 0x00);   // I2S 16-bit, ไม่ใช้ TDM
  i2cWrite8(a, 0x43, 0x1A); i2cWrite8(a, 0x44, 0x1A);   // MIC1/2 gain ~30 dB
  i2cWrite8(a, 0x45, 0x00); i2cWrite8(a, 0x46, 0x00);
  i2cWrite8(a, 0x01, 0x20); i2cWrite8(a, 0x06, 0x00); i2cWrite8(a, 0x40, 0x42);
  i2cWrite8(a, 0x47, 0x08); i2cWrite8(a, 0x48, 0x08);
  i2cWrite8(a, 0x49, 0xFF); i2cWrite8(a, 0x4A, 0xFF);
  i2cWrite8(a, 0x4B, 0x00); i2cWrite8(a, 0x4C, 0xFF);   // เปิด MIC1/2 ปิด MIC3/4
}

static void es7210Stop() {
  const uint8_t a = ADDR_ES7210;
  i2cWrite8(a, 0x4B, 0xFF); i2cWrite8(a, 0x4C, 0xFF);
  i2cWrite8(a, 0x47, 0xFF); i2cWrite8(a, 0x48, 0xFF);
  i2cWrite8(a, 0x49, 0xFF); i2cWrite8(a, 0x4A, 0xFF);
  i2cWrite8(a, 0x40, 0x80); i2cWrite8(a, 0x01, 0x7F); i2cWrite8(a, 0x06, 0x07);
}

// ───────────────────────── ES8311 (codec 1 ช่อง) ─────────────────────────
static void es8311MicStart(uint32_t rate) {
  const uint8_t a = ADDR_ES8311;
  i2cWrite8(a, 0x00, 0x1F); delay(20);      // reset
  i2cWrite8(a, 0x00, 0x00);
  i2cWrite8(a, 0x01, 0x30);                 // เปิด MCLK, เลือก clock source
  i2cWrite8(a, 0x02, 0x10);                 // ตัวหาร (MCLK = 256 × fs)
  i2cWrite8(a, 0x03, 0x10);
  i2cWrite8(a, 0x16, 0x24);
  i2cWrite8(a, 0x04, 0x10); i2cWrite8(a, 0x05, 0x00);
  i2cWrite8(a, 0x06, 0x03); i2cWrite8(a, 0x07, 0x00); i2cWrite8(a, 0x08, 0xFF);
  i2cWrite8(a, 0x09, 0x0C);                 // SDP In  : I2S 16-bit
  i2cWrite8(a, 0x0A, 0x0C);                 // SDP Out : I2S 16-bit
  i2cWrite8(a, 0x0B, 0x00); i2cWrite8(a, 0x0C, 0x00);
  i2cWrite8(a, 0x10, 0x03); i2cWrite8(a, 0x11, 0x7B);
  i2cWrite8(a, 0x00, 0x80);                 // slave mode
  i2cWrite8(a, 0x0D, 0x01);                 // power up analog
  i2cWrite8(a, 0x0E, 0x02);                 // power up ADC
  i2cWrite8(a, 0x12, 0x00);
  i2cWrite8(a, 0x13, 0x00);
  i2cWrite8(a, 0x1C, 0x6A);
  i2cWrite8(a, 0x37, 0x08);
  i2cWrite8(a, 0x17, 0xC8);                 // ADC volume
  i2cWrite8(a, 0x16, 0x03);                 // PGA gain (0..7 ยิ่งมากยิ่งดัง)
  (void)rate;
}

static void es8311MicStop() {
  const uint8_t a = ADDR_ES8311;
  i2cWrite8(a, 0x0E, 0xFF);                 // ปิด ADC
  i2cWrite8(a, 0x0D, 0xFA);                 // ปิดภาค analog
  i2cWrite8(a, 0x00, 0x00);
  i2cWrite8(a, 0x01, 0x30);
  i2cWrite8(a, 0x01, 0x00);                 // ปิด clock → สลีป
}

// ───────────────────────── API ─────────────────────────
bool codecDetect(uint8_t &foundChip) {
  if (i2cPresent(ADDR_ES7210)) { foundChip = 0; return true; }
  if (i2cPresent(ADDR_ES8311)) { foundChip = 1; return true; }
  return false;
}

bool codecMicStart(uint32_t sampleRate) {
  uint8_t chip = settings.micChip;
  uint8_t addr = chip ? ADDR_ES8311 : ADDR_ES7210;
  if (!i2cPresent(addr)) {
    // ชิปที่เลือกไว้ไม่ตอบ → ลองตรวจอัตโนมัติแล้วใช้ตัวที่เจอ
    uint8_t found;
    if (!codecDetect(found)) {
      Serial.println(F("[codec] ไม่พบชิปไมโครโฟนบนบัส I2C"));
      return false;
    }
    Serial.printf("[codec] เปลี่ยนไปใช้ชิปที่ตรวจพบ: %s\n", found ? "ES8311" : "ES7210");
    chip = found;
    settings.micChip = found;
  }
  if (chip) es8311MicStart(sampleRate);
  else      es7210Start(sampleRate);
  delay(10);
  return true;
}

void codecMicStop() {
  if (i2cPresent(ADDR_ES7210)) es7210Stop();
  if (i2cPresent(ADDR_ES8311)) es8311MicStop();
}


// ---------- audio_rec.cpp ----------

#if ENABLE_AUDIO

#include <ESP_I2S.h>
#include <SD.h>
#include <esp_heap_caps.h>

static I2SClass i2s;

// ───────── ริงบัฟเฟอร์ SPSC (ผู้ผลิต 1 ผู้บริโภค 1 → ไม่ต้องใช้ mutex) ─────────
static uint8_t *ring = nullptr;
static size_t   ringSize = 0;
static volatile size_t ringHead = 0;     // ตัวอ่าน I2S เขียนที่นี่
static volatile size_t ringTail = 0;     // ตัวเขียน SD อ่านจากที่นี่

static volatile bool s_wanted   = false;
static volatile bool s_running  = false;
static volatile bool s_writing  = false;
static volatile bool s_fileOpen = false;
static volatile uint64_t s_bytesWritten = 0;
static char s_err[64]  = "";
static char s_file[48] = "";

static File recFile;
static uint32_t recDataBytes = 0;        // ขนาด data chunk ของไฟล์ที่เปิดอยู่

static inline size_t ringUsed() {
  size_t h = ringHead, t = ringTail;
  return (h >= t) ? (h - t) : (ringSize - t + h);
}
static inline size_t ringFree() { return ringSize - ringUsed() - 1; }

// ───────── WAV ─────────
static void wavWriteHeader(File &f, uint32_t dataLen, uint32_t rate) {
  uint32_t byteRate = rate * 2, chunk = 36 + dataLen, fmtLen = 16;
  uint16_t pcm = 1, ch = 1, align = 2, bits = 16;
  uint32_t pos = f.position();
  f.seek(0);
  f.write((const uint8_t *)"RIFF", 4);      f.write((uint8_t *)&chunk, 4);
  f.write((const uint8_t *)"WAVEfmt ", 8);  f.write((uint8_t *)&fmtLen, 4);
  f.write((uint8_t *)&pcm, 2);              f.write((uint8_t *)&ch, 2);
  f.write((uint8_t *)&rate, 4);             f.write((uint8_t *)&byteRate, 4);
  f.write((uint8_t *)&align, 2);            f.write((uint8_t *)&bits, 2);
  f.write((const uint8_t *)"data", 4);      f.write((uint8_t *)&dataLen, 4);
  f.seek(pos < 44 ? 44 : pos);
}

static bool fileOpenNew() {
  if (!hwSdOk()) { strlcpy(s_err, "No SD card", sizeof(s_err)); return false; }
  if (!SD.exists(REC_DIR)) SD.mkdir(REC_DIR);
  struct tm t; hwNowLocal(t);
  char name[48];
  strftime(name, sizeof(name), REC_DIR "/%Y%m%d_%H%M%S.wav", &t);
  recFile = SD.open(name, FILE_WRITE);
  if (!recFile) { strlcpy(s_err, "Cannot open file", sizeof(s_err)); return false; }
  recDataBytes = 0;
  wavWriteHeader(recFile, 0, settings.sampleRate);
  strlcpy(s_file, name, sizeof(s_file));
  s_fileOpen = true;
  Serial.printf("[rec] ไฟล์ใหม่: %s\n", name);
  return true;
}

static void fileCloseCurrent() {
  if (!s_fileOpen) return;
  wavWriteHeader(recFile, recDataBytes, settings.sampleRate);
  recFile.flush();
  recFile.close();
  s_fileOpen = false;
  Serial.printf("[rec] ปิดไฟล์ %s (%lu bytes)\n", s_file, (unsigned long)recDataBytes);
}

// ───────── ตัวอ่าน : I2S DMA → SRAM → PSRAM ─────────
static bool i2sStart() {
  bool stereo = (settings.micChip == 0);        // ES7210 ส่งมา 2 ช่อง
  i2s.setPins(I2S_BCLK, I2S_WS, -1 /*DOUT*/, I2S_DIN, I2S_MCLK);
  if (!i2s.begin(I2S_MODE_STD, (int)settings.sampleRate, I2S_DATA_BIT_WIDTH_16BIT,
                 stereo ? I2S_SLOT_MODE_STEREO : I2S_SLOT_MODE_MONO)) {
    strlcpy(s_err, "i2s.begin failed", sizeof(s_err));
    return false;
  }
  return true;
}

static void ringPush(const uint8_t *src, size_t n) {
  size_t space = ringFree();
  if (n > space) n = space;                     // ริงเต็ม → ทิ้งส่วนเกิน (ไม่ค้างระบบ)
  size_t h = ringHead;
  size_t first = min(n, ringSize - h);
  memcpy(ring + h, src, first);
  if (n > first) memcpy(ring, src + first, n - first);
  ringHead = (h + n) % ringSize;
}

static void readerTask(void *) {
  static uint8_t *stage = nullptr;               // บัฟเฟอร์ใน SRAM (ไม่ใช่ PSRAM)
  if (!stage) stage = (uint8_t *)heap_caps_malloc(I2S_STAGE_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

  for (;;) {
    if (s_wanted && !s_running) {
      ringHead = ringTail = 0;
      s_err[0] = 0;
      if (!codecMicStart(settings.sampleRate) || !i2sStart()) {
        s_wanted = false;
      } else {
        s_running = true;
        Serial.printf("[rec] เริ่มอัด %lu Hz, ring %u MB\n",
                      (unsigned long)settings.sampleRate, (unsigned)(ringSize / (1024 * 1024)));
      }
    }

    if (!s_wanted && s_running) {
      s_running = false;                         // ตัวเขียนจะไล่เก็บที่ค้างแล้วปิดไฟล์
      i2s.end();
      codecMicStop();
      Serial.println(F("[rec] หยุดอัด"));
    }

    if (!s_running || !stage) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

    size_t n = i2s.readBytes((char *)stage, I2S_STAGE_BYTES);
    if (n == 0) { vTaskDelay(1); continue; }

    if (settings.micChip == 0) {
      // สเตอริโอ → เก็บเฉพาะช่องซ้าย (MIC1) ให้เป็น mono
      int16_t *p = (int16_t *)stage;
      size_t frames = n / 4;
      for (size_t i = 0; i < frames; i++) p[i] = p[i * 2];
      n = frames * 2;
    }
    ringPush(stage, n);
  }
}

// ───────── ตัวเขียน : PSRAM → SD ─────────
static void drainOnce(size_t maxBytes) {
  size_t used = ringUsed();
  size_t n = min(used, maxBytes);
  if (n == 0) return;
  size_t t = ringTail;
  size_t first = min(n, ringSize - t);

  s_writing = true;
  recFile.write(ring + t, first);
  if (n > first) recFile.write(ring, n - first);
  s_writing = false;

  ringTail = (t + n) % ringSize;
  recDataBytes   += n;
  s_bytesWritten += n;
}

static void writerTask(void *) {
  for (;;) {
    if (!s_running && !s_fileOpen) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }

    if (s_running && !s_fileOpen) {
      if (!fileOpenNew()) { s_wanted = false; vTaskDelay(pdMS_TO_TICKS(200)); continue; }
    }

    size_t used = ringUsed();
    size_t flushAt = (size_t)((uint64_t)ringSize * (100 - settings.flushPct) / 100);
    size_t drainTo = ringSize / 4;               // ระบายลงมาเหลือ ~25% แล้วหยุด

    if (!s_running) {
      // กำลังปิดงาน → ไล่เขียนให้หมดแล้วปิดไฟล์
      while (s_fileOpen && ringUsed() > 0) drainOnce(SD_CHUNK_BYTES);
      fileCloseCurrent();
      continue;
    }

    if (used >= flushAt) {
      uint32_t t0 = millis();
      while (s_running && s_fileOpen && ringUsed() > drainTo) {
        drainOnce(SD_CHUNK_BYTES);
        if (recDataBytes >= REC_FILE_MAX_BYTES) {   // ตัดไฟล์เมื่อใหญ่เกิน
          fileCloseCurrent();
          if (!fileOpenNew()) { s_wanted = false; break; }
        }
        vTaskDelay(1);                              // ให้ core อื่นได้หายใจ
      }
      if (!s_fileOpen) continue;
      wavWriteHeader(recFile, recDataBytes, settings.sampleRate);
      recFile.flush();
      Serial.printf("[rec] flush ลง SD %lu ms, ริงเหลือ %u%%\n",
                    (unsigned long)(millis() - t0), (unsigned)audioRingUsedPct());
    } else {
      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }
}

// ───────── API ─────────
bool audioInit() {
  size_t want = (size_t)settings.ringMB * 1024UL * 1024UL;
  size_t avail = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
  if (avail < 512 * 1024) { strlcpy(s_err, "No PSRAM", sizeof(s_err)); return false; }
  if (want > avail - 256 * 1024) want = avail - 256 * 1024;   // เผื่อไว้ให้ระบบอื่น

  ring = (uint8_t *)heap_caps_malloc(want, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!ring) { strlcpy(s_err, "PSRAM alloc failed", sizeof(s_err)); return false; }
  ringSize = want;
  Serial.printf("[rec] ริงบัฟเฟอร์ PSRAM %u KB (~%lu วินาที)\n",
                (unsigned)(ringSize / 1024),
                (unsigned long)(ringSize / (settings.sampleRate * 2)));

  xTaskCreatePinnedToCore(readerTask, "audioRd", 4096, nullptr, 6, nullptr, 0);
  xTaskCreatePinnedToCore(writerTask, "audioWr", 6144, nullptr, 3, nullptr, 1);
  return true;
}

bool audioStart() {
  if (!ring) { strlcpy(s_err, "Buffer not ready", sizeof(s_err)); return false; }
  if (!hwSdOk()) { strlcpy(s_err, "No SD card", sizeof(s_err)); return false; }
  s_wanted = true;
  uint32_t t0 = millis();
  while (!s_running && millis() - t0 < 3000) delay(20);
  return s_running;
}

void audioStop() {
  s_wanted = false;
  uint32_t t0 = millis();
  while ((s_running || s_fileOpen) && millis() - t0 < 15000) delay(50);
}

bool audioIsRecording()  { return s_running; }
bool audioIsBusy()       { return s_writing; }
uint64_t audioBytesWritten() { return s_bytesWritten; }
const char *audioLastError()  { return s_err; }
const char *audioCurrentFile(){ return s_file; }

uint8_t audioRingUsedPct() {
  if (!ringSize) return 0;
  return (uint8_t)((uint64_t)ringUsed() * 100 / ringSize);
}

#else   // ENABLE_AUDIO == 0

bool audioInit() { return false; }
bool audioStart() { return false; }
void audioStop() {}
bool audioIsRecording() { return false; }
bool audioIsBusy() { return false; }
uint8_t audioRingUsedPct() { return 0; }
uint64_t audioBytesWritten() { return 0; }
const char *audioLastError() { return "Audio disabled at build"; }
const char *audioCurrentFile() { return ""; }

#endif


// ---------- ota.cpp ----------

#if ENABLE_OTA

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>

static void say(OtaProgressCb cb, const char *msg) {
  Serial.printf("[ota] %s\n", msg);
  if (cb) cb(msg);
}

bool otaWifiConnect(uint32_t timeoutMs) {
  if (settings.wifiSsid[0] == 0) return false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(settings.wifiSsid, settings.wifiPass);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeoutMs) delay(200);
  return WiFi.status() == WL_CONNECTED;
}

void otaWifiOff() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

bool otaSyncNtp(OtaProgressCb cb) {
  bool hadWifi = (WiFi.status() == WL_CONNECTED);
  if (!hadWifi && !otaWifiConnect()) { say(cb, "WiFi connect failed"); return false; }

  say(cb, "Syncing time...");
  // เก็บ system time เป็นเวลาท้องถิ่นตรง ๆ → ใส่ offset ตอน configTime เลย
  configTime(settings.tzMinutes * 60, 0, "pool.ntp.org", "time.google.com");
  struct tm t;
  bool ok = getLocalTime(&t, 10000);
  if (ok) {
    hwTimeToRtc();
    say(cb, "Time synced");
  } else {
    say(cb, "Time sync failed");
  }
  if (!hadWifi) otaWifiOff();
  return ok;
}

// เทียบเวอร์ชันแบบ semver ง่าย ๆ : คืน true ถ้า remote ใหม่กว่า local
static bool isNewer(const String &remote, const char *local) {
  int r[3] = {0, 0, 0}, l[3] = {0, 0, 0};
  sscanf(remote.c_str(), "%d.%d.%d", &r[0], &r[1], &r[2]);
  sscanf(local,          "%d.%d.%d", &l[0], &l[1], &l[2]);
  for (int i = 0; i < 3; i++) {
    if (r[i] > l[i]) return true;
    if (r[i] < l[i]) return false;
  }
  return false;
}

static bool httpGetString(const String &url, String &out) {
  WiFiClientSecure client;
  client.setInsecure();                    // ไม่ตรวจใบรับรอง (ยอมรับได้สำหรับ manifest สาธารณะ)
  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  if (!http.begin(client, url)) return false;
  int code = http.GET();
  if (code != HTTP_CODE_OK) { http.end(); return false; }
  out = http.getString();
  http.end();
  return true;
}

bool otaCheckAndUpdate(OtaProgressCb cb) {
  if (settings.otaUrl[0] == 0) { say(cb, "otaUrl not set"); return false; }
  if (!otaWifiConnect()) { say(cb, "WiFi connect failed"); return false; }

  say(cb, "Checking version...");
  String manifest;
  if (!httpGetString(String(settings.otaUrl), manifest)) {
    say(cb, "Manifest read failed");
    otaWifiOff();
    return false;
  }

  String version, binUrl;
  int start = 0;
  while (start < (int)manifest.length()) {
    int nl = manifest.indexOf('\n', start);
    String line = (nl < 0) ? manifest.substring(start) : manifest.substring(start, nl);
    line.trim();
    if (line.startsWith("version=")) version = line.substring(8);
    else if (line.startsWith("url="))  binUrl = line.substring(4);
    if (nl < 0) break;
    start = nl + 1;
  }
  version.trim(); binUrl.trim();

  if (version.length() == 0 || binUrl.length() == 0) { say(cb, "Bad manifest format"); otaWifiOff(); return false; }
  if (!isNewer(version, FW_VERSION)) {
    char msg[64];
    snprintf(msg, sizeof(msg), "Already newest (%s)", FW_VERSION);
    say(cb, msg);
    otaWifiOff();
    return false;
  }

  char msg[64];
  snprintf(msg, sizeof(msg), "Found %s, downloading...", version.c_str());
  say(cb, msg);

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  if (!http.begin(client, binUrl)) { say(cb, "Cannot open bin URL"); otaWifiOff(); return false; }
  int code = http.GET();
  int len = http.getSize();
  if (code != HTTP_CODE_OK || len <= 0) { say(cb, "Download failed"); http.end(); otaWifiOff(); return false; }

  if (!Update.begin(len)) { say(cb, "Not enough OTA space"); http.end(); otaWifiOff(); return false; }
  size_t written = Update.writeStream(http.getStream());
  http.end();

  if (written != (size_t)len || !Update.end(true) || !Update.isFinished()) {
    say(cb, "Flash write failed");
    otaWifiOff();
    return false;
  }

  say(cb, "Update OK, rebooting");
  otaWifiOff();
  delay(1200);
  ESP.restart();
  return true;
}

#else   // ENABLE_OTA == 0

bool otaWifiConnect(uint32_t) { return false; }
void otaWifiOff() {}
bool otaSyncNtp(OtaProgressCb cb) { if (cb) cb("OTA disabled at build"); return false; }
bool otaCheckAndUpdate(OtaProgressCb cb) { if (cb) cb("OTA disabled at build"); return false; }

#endif


// ---------- ui_menu.cpp ----------

#if USE_LVGL
#include <lvgl.h>
#endif

// ───────────────────────── โมเดลของเมนู ─────────────────────────
enum ItemType : uint8_t { IT_ENUM, IT_INT, IT_BOOL, IT_ACTION, IT_INFO };
enum VType    : uint8_t { V_NONE, V_U8, V_U16, V_U32, V_I16, V_BOOL };

struct MenuItem {
  const char *label;
  ItemType    type;
  VType       vt;
  void       *ptr;
  int32_t     vmin, vmax, vstep, vscale;   // vscale = ตัวหารตอนแสดงผล (ms→s ใช้ 1000)
  const char *const *opts;
  uint8_t     optCount;
  const int32_t *optVals;                  // ค่าจริงของแต่ละตัวเลือก (nullptr = 0..n-1)
  void      (*action)();
  const char *(*info)();                   // ข้อความสำหรับ IT_INFO / IT_ACTION
};

static const char *OPT_FACE[]  = { "Analog", "Digital" };
static const char *OPT_MIC[]   = { "ES7210", "ES8311" };
static const char *OPT_RATE[]  = { "8 kHz", "16 kHz", "32 kHz" };
static const int32_t VAL_RATE[] = { 8000, 16000, 32000 };
static const char *OPT_CPU[]   = { "80 MHz", "160 MHz", "240 MHz" };
static const int32_t VAL_CPU[] = { 80, 160, 240 };

static char s_toast[40] = "";
static uint32_t s_toastUntil = 0;

void uiToast(const char *msg) {
  strlcpy(s_toast, msg, sizeof(s_toast));
  s_toastUntil = millis() + 4000;
}

// ───────── การทำงานของรายการแบบปุ่มกด ─────────
static void actToggleRecord() {
  if (audioIsRecording()) { audioStop(); uiToast("Recording stopped"); }
  else if (audioStart())  { uiToast("Recording started"); }
  else                    { uiToast(audioLastError()); }
}
static void actSave()        { uiToast(settingsSave() ? "Settings saved" : "Save failed"); }
static void actCheckUpdate() { otaCheckAndUpdate(uiToast); }
static void actSyncTime()    { otaSyncNtp(uiToast); }
static void actFactory()     { settingsDefaults(); settingsSave(); uiToast("Defaults restored"); }
static void actReboot()      { settingsSave(); delay(300); ESP.restart(); }

static const char *infoRecord() { return audioIsRecording() ? "REC" : "off"; }
static const char *infoRing()   { static char b[16]; snprintf(b, sizeof(b), "%u%%", audioRingUsedPct()); return b; }
static const char *infoSsid()   { return settings.wifiSsid[0] ? settings.wifiSsid : "(not set)"; }
static const char *infoOta()    { return settings.otaUrl[0] ? "set" : "(not set)"; }
static const char *infoVer()    { return FW_VERSION; }
static const char *infoHw()     {
  static char b[24];
  snprintf(b, sizeof(b), "%s%s%s%s",
           hwRtcOk() ? "RTC " : "", hwImuOk() ? "IMU " : "",
           touchOk() ? "TP "  : "", hwSdOk()  ? "SD"  : "");
  return b;
}

#define ITEM_ENUM(l, p, vt, o, ov)  { l, IT_ENUM,   vt, p, 0,0,0,1, o, (uint8_t)(sizeof(o)/sizeof(o[0])), ov, nullptr, nullptr }
#define ITEM_INT(l, p, vt, a, b, s, sc) { l, IT_INT, vt, p, a, b, s, sc, nullptr, 0, nullptr, nullptr, nullptr }
#define ITEM_BOOL(l, p)             { l, IT_BOOL,   V_BOOL, p, 0,0,0,1, nullptr, 0, nullptr, nullptr, nullptr }
#define ITEM_ACT(l, f, i)           { l, IT_ACTION, V_NONE, nullptr, 0,0,0,1, nullptr, 0, nullptr, f, i }
#define ITEM_INFO(l, i)             { l, IT_INFO,   V_NONE, nullptr, 0,0,0,1, nullptr, 0, nullptr, nullptr, i }

static MenuItem MENU[] = {
  ITEM_ENUM("Watch face",    &settings.faceMode,       V_U8,  OPT_FACE, nullptr),
  ITEM_INT ("Bright raised", &settings.brightHighPct,  V_U8,  5, 100, 5, 1),
  ITEM_INT ("Bright dim",    &settings.brightDimPct,   V_U8,  0, 100, 5, 1),
  ITEM_INT ("Bright hold s", &settings.brightHoldMs,   V_U16, 1000, 60000, 1000, 1000),
  ITEM_BOOL("Wrist wake",    &settings.wristWake),
  ITEM_INT ("Wrist angle",   &settings.faceAngleDeg,   V_U8,  5, 60, 5, 1),
  ITEM_INT ("Wrist hold ms", &settings.faceHoldMs,     V_U16, 0, 2000, 50, 1),
  ITEM_INT ("Motion thr mg", &settings.womThresholdMg, V_U16, 20, 1000, 10, 1),
  ITEM_ACT ("Recording",     actToggleRecord, infoRecord),
  ITEM_INFO("Buffer used",   infoRing),
  ITEM_ENUM("Sample rate",   &settings.sampleRate,     V_U32, OPT_RATE, VAL_RATE),
  ITEM_ENUM("Mic chip",      &settings.micChip,        V_U8,  OPT_MIC, nullptr),
  ITEM_INT ("Flush at %",    &settings.flushPct,       V_U8,  5, 50, 5, 1),
  ITEM_INT ("Ring MB",       &settings.ringMB,         V_U8,  1, 6, 1, 1),
  ITEM_BOOL("WiFi enabled",  &settings.wifiEnabled),
  ITEM_INFO("WiFi SSID",     infoSsid),
  ITEM_INFO("OTA URL",       infoOta),
  ITEM_ACT ("Check update",  actCheckUpdate, nullptr),
  ITEM_ACT ("Sync time NTP", actSyncTime,    nullptr),
  ITEM_INT ("TZ minutes",    &settings.tzMinutes,      V_I16, -720, 840, 30, 1),
  ITEM_ENUM("CPU speed",     &settings.cpuMhz,         V_U16, OPT_CPU, VAL_CPU),
  ITEM_INFO("Firmware",      infoVer),
  ITEM_INFO("Hardware",      infoHw),
  ITEM_ACT ("Save settings", actSave,    nullptr),
  ITEM_ACT ("Restore defaults", actFactory, nullptr),
  ITEM_ACT ("Reboot",        actReboot,  nullptr),
};
static const int MENU_N = sizeof(MENU) / sizeof(MENU[0]);

// ───────── อ่าน/เขียนค่าในโครงสร้าง Settings ─────────
static int32_t getVal(const MenuItem &m) {
  switch (m.vt) {
    case V_U8:   return *(uint8_t  *)m.ptr;
    case V_U16:  return *(uint16_t *)m.ptr;
    case V_U32:  return (int32_t)*(uint32_t *)m.ptr;
    case V_I16:  return *(int16_t  *)m.ptr;
    case V_BOOL: return *(bool     *)m.ptr ? 1 : 0;
    default:     return 0;
  }
}

static void setVal(const MenuItem &m, int32_t v) {
  switch (m.vt) {
    case V_U8:   *(uint8_t  *)m.ptr = (uint8_t)v;  break;
    case V_U16:  *(uint16_t *)m.ptr = (uint16_t)v; break;
    case V_U32:  *(uint32_t *)m.ptr = (uint32_t)v; break;
    case V_I16:  *(int16_t  *)m.ptr = (int16_t)v;  break;
    case V_BOOL: *(bool     *)m.ptr = (v != 0);    break;
    default: break;
  }
}

static void valueText(const MenuItem &m, char *out, size_t n) {
  switch (m.type) {
    case IT_ENUM: {
      int32_t v = getVal(m);
      int idx = 0;
      if (m.optVals) { for (uint8_t i = 0; i < m.optCount; i++) if (m.optVals[i] == v) idx = i; }
      else idx = constrain((int)v, 0, (int)m.optCount - 1);
      strlcpy(out, m.opts[idx], n);
      break;
    }
    case IT_BOOL:   strlcpy(out, getVal(m) ? "ON" : "OFF", n); break;
    case IT_INT: {
      int32_t v = getVal(m);
      if (m.vscale > 1) snprintf(out, n, "%ld", (long)(v / m.vscale));
      else              snprintf(out, n, "%ld", (long)v);
      break;
    }
    case IT_ACTION: strlcpy(out, m.info ? m.info() : ">", n); break;
    case IT_INFO:   strlcpy(out, m.info ? m.info() : "", n); break;
  }
}

// ───────── สถานะเมนู ─────────
static bool s_open = false;
static int  s_sel = 0;
static bool s_editing = false;
static bool s_dirty = true;       // ต้องวาดใหม่

static void applyLiveChanges() {
  settingsClampAll();
  hwSetBrightnessPct(settings.brightHighPct);
  setCpuFrequencyMhz(settings.cpuMhz);
}

static void doSelect() {
  MenuItem &m = MENU[s_sel];
  switch (m.type) {
    case IT_BOOL:
      setVal(m, getVal(m) ? 0 : 1);
      if (m.ptr == &settings.wristWake) imuConfigureWom();
      applyLiveChanges();
      break;
    case IT_ENUM: {
      int32_t v = getVal(m);
      int idx = 0;
      if (m.optVals) { for (uint8_t i = 0; i < m.optCount; i++) if (m.optVals[i] == v) idx = i; }
      else idx = constrain((int)v, 0, (int)m.optCount - 1);
      idx = (idx + 1) % m.optCount;
      setVal(m, m.optVals ? m.optVals[idx] : idx);
      applyLiveChanges();
      break;
    }
    case IT_INT:
      s_editing = true;                 // ปัดขึ้น/ลงเพื่อปรับค่า แล้วปัดขวาเพื่อยืนยัน
      break;
    case IT_ACTION:
      if (m.action) m.action();
      break;
    case IT_INFO:
      break;
  }
  s_dirty = true;
}

void uiGesture(Gesture g) {
  if (!s_open) return;
  MenuItem &m = MENU[s_sel];

  if (s_editing) {
    int32_t v = getVal(m);
    switch (g) {
      case GST_UP:    v += m.vstep; break;
      case GST_DOWN:  v -= m.vstep; break;
      case GST_RIGHT: s_editing = false; applyLiveChanges(); settingsSave(); s_dirty = true; return;
      case GST_LEFT:  s_editing = false; s_dirty = true; return;
      default: return;
    }
    setVal(m, constrain(v, m.vmin, m.vmax));
    s_dirty = true;
    return;
  }

  switch (g) {
    case GST_UP:    s_sel = (s_sel + 1) % MENU_N;             s_dirty = true; break;
    case GST_DOWN:  s_sel = (s_sel + MENU_N - 1) % MENU_N;    s_dirty = true; break;
    case GST_RIGHT: doSelect();                                               break;
    case GST_LEFT:  uiClose();                                                break;
    case GST_TAP:   s_dirty = true;                                           break;
    default: break;
  }
}

// ═══════════════════════ ตัววาด : LVGL ═══════════════════════
#if USE_LVGL

#define LV_ROWS 9
static lv_obj_t *s_rows[LV_ROWS];
static lv_obj_t *s_title;
static lv_obj_t *s_toastLbl;
static lv_display_t *s_disp = nullptr;
static uint8_t *s_lvBuf = nullptr;

static void lvFlush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;
  lv_draw_sw_rgb565_swap(px_map, w * h);
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);
  lv_display_flush_ready(disp);
}

static uint32_t lvTick() { return millis(); }

void uiInit() {
  lv_init();
  lv_tick_set_cb(lvTick);
  size_t bufPx = LCD_W * 40;
  s_lvBuf = (uint8_t *)heap_caps_malloc(bufPx * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  s_disp = lv_display_create(LCD_W, LCD_H);
  lv_display_set_flush_cb(s_disp, lvFlush);
  lv_display_set_buffers(s_disp, s_lvBuf, nullptr, bufPx * 2, LV_DISPLAY_RENDER_MODE_PARTIAL);

  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  s_title = lv_label_create(scr);
  lv_obj_set_style_text_color(s_title, lv_color_white(), 0);
  lv_obj_align(s_title, LV_ALIGN_TOP_MID, 0, 12);

  for (int i = 0; i < LV_ROWS; i++) {
    s_rows[i] = lv_label_create(scr);
    lv_obj_set_style_text_color(s_rows[i], lv_color_white(), 0);
    lv_obj_set_width(s_rows[i], LCD_W - 24);
    lv_obj_align(s_rows[i], LV_ALIGN_TOP_LEFT, 12, 60 + i * 42);
  }
  s_toastLbl = lv_label_create(scr);
  lv_obj_set_style_text_color(s_toastLbl, lv_color_white(), 0);
  lv_obj_align(s_toastLbl, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_obj_add_flag(scr, LV_OBJ_FLAG_HIDDEN);      // ซ่อนไว้จนกว่าจะเปิดเมนู
}

static void lvRender() {
  char v[32], line[64];
  lv_label_set_text_fmt(s_title, "SETTINGS  %s", FW_VERSION);
  int first = s_sel - LV_ROWS / 2;
  if (first < 0) first = 0;
  if (first > MENU_N - LV_ROWS) first = max(0, MENU_N - LV_ROWS);
  for (int i = 0; i < LV_ROWS; i++) {
    int idx = first + i;
    if (idx >= MENU_N) { lv_label_set_text(s_rows[i], ""); continue; }
    valueText(MENU[idx], v, sizeof(v));
    const char *mark = (idx == s_sel) ? (s_editing ? "*" : ">") : " ";
    snprintf(line, sizeof(line), "%s %-15s %s", mark, MENU[idx].label, v);
    lv_label_set_text(s_rows[i], line);
  }
  lv_label_set_text(s_toastLbl, (millis() < s_toastUntil) ? s_toast : "");
}

void uiOpen() {
  s_open = true; s_sel = 0; s_editing = false; s_dirty = true;
  lv_obj_clear_flag(lv_screen_active(), LV_OBJ_FLAG_HIDDEN);
  lv_obj_invalidate(lv_screen_active());
}

void uiClose() {
  s_open = false; s_editing = false;
  settingsSave();
  lv_obj_add_flag(lv_screen_active(), LV_OBJ_FLAG_HIDDEN);
  faceInvalidate();
}

void uiTick() {
  if (!s_open) return;
  if (s_dirty || millis() < s_toastUntil) { lvRender(); s_dirty = false; }
  lv_timer_handler();
  delay(5);
}

// ═══════════════════════ ตัววาด : Arduino_GFX ═══════════════════════
#else

#define GFX_ROWS 9
static const int ROW_H = 42, ROW_Y0 = 62;

void uiInit() {}

static void gfxRender() {
  gfx->fillScreen(RGB565_BLACK);
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565_WHITE);
  gfx->setCursor(12, 20);
  gfx->printf("SETTINGS  %s", FW_VERSION);
  gfx->drawFastHLine(0, 50, LCD_W, RGB565_WHITE);

  int first = s_sel - GFX_ROWS / 2;
  if (first < 0) first = 0;
  if (first > MENU_N - GFX_ROWS) first = max(0, MENU_N - GFX_ROWS);

  char v[32];
  for (int i = 0; i < GFX_ROWS; i++) {
    int idx = first + i;
    if (idx >= MENU_N) break;
    int y = ROW_Y0 + i * ROW_H;
    bool sel = (idx == s_sel);
    if (sel) gfx->drawRect(6, y - 6, LCD_W - 12, ROW_H - 4, RGB565_WHITE);
    valueText(MENU[idx], v, sizeof(v));
    gfx->setCursor(16, y);
    gfx->print(s_editing && sel ? "*" : (sel ? ">" : " "));
    gfx->setCursor(38, y);
    gfx->print(MENU[idx].label);
    gfx->setCursor(LCD_W - 12 - (int)strlen(v) * 12, y);
    gfx->print(v);
  }

  if (millis() < s_toastUntil) {
    gfx->fillRect(0, LCD_H - 46, LCD_W, 46, RGB565_BLACK);
    gfx->drawFastHLine(0, LCD_H - 46, LCD_W, RGB565_WHITE);
    gfx->setCursor(12, LCD_H - 32);
    gfx->print(s_toast);
  }
}

void uiOpen()  { s_open = true; s_sel = 0; s_editing = false; s_dirty = true; }
void uiClose() { s_open = false; s_editing = false; settingsSave(); faceInvalidate(); }

void uiTick() {
  if (!s_open) return;
  static uint32_t lastToast = 0;
  bool toastChanged = (millis() < s_toastUntil) != (lastToast < s_toastUntil);
  lastToast = millis();
  if (s_dirty || toastChanged) { gfxRender(); s_dirty = false; }
  delay(10);
}

#endif

bool uiIsOpen() { return s_open; }


// ---------- modules.cpp ----------

// ตัวอย่างการเพิ่มโมดูลใหม่:
//
//   static void stepsInit()                    { ... }
//   static void stepsMinute(const struct tm &t){ ... }
//
//   static const Module MODULES[] = {
//     { "steps", stepsInit, stepsMinute, nullptr },
//   };

static const Module MODULES[] = {
  { "none", nullptr, nullptr, nullptr },
};
static const size_t MODULES_N = sizeof(MODULES) / sizeof(MODULES[0]);

void modulesInit() {
  for (size_t i = 0; i < MODULES_N; i++) if (MODULES[i].init) MODULES[i].init();
}

void modulesMinute(const struct tm &t) {
  for (size_t i = 0; i < MODULES_N; i++) if (MODULES[i].onMinute) MODULES[i].onMinute(t);
}

void modulesRaise() {
  for (size_t i = 0; i < MODULES_N; i++) if (MODULES[i].onRaise) MODULES[i].onRaise();
}


// ============================================================================
//  setup() / loop() — เดิมคือ S3Watch.ino
// ============================================================================
RTC_DATA_ATTR static int s_lastMin = -1;

static bool     s_bright = false;
static uint32_t s_brightUntil = 0;
static bool     s_btnDown = false;
static uint32_t s_btnT0 = 0;
static bool     s_btnFired = false;

// ───────── ความสว่าง ─────────
static void setBright(bool on) {
  s_bright = on;
  hwSetBrightnessPct(on ? settings.brightHighPct : settings.brightDimPct);
  s_brightUntil = on ? millis() + settings.brightHoldMs : 0;
}

static void keepBrightAlive() {
  if (s_bright) s_brightUntil = millis() + settings.brightHoldMs;
  else setBright(true);
}

// ───────── ปุ่ม BOOT ─────────
static void toggleUi() {
  if (uiIsOpen()) uiClose();
  else            uiOpen();
  keepBrightAlive();
}

static void handleButton() {
  bool down = (digitalRead(PIN_BOOT) == LOW);
  if (down && !s_btnDown) {
    s_btnDown = true; s_btnT0 = millis(); s_btnFired = false;
  } else if (down && !s_btnFired && millis() - s_btnT0 >= LONG_PRESS_MS) {
    s_btnFired = true;
    toggleUi();
  } else if (!down) {
    s_btnDown = false;
  }
}

// ============================================================================
void setup() {
  bool fromDeep = powerWokeFromDeepSleep();

  Serial.begin(115200);
  if (!fromDeep) delay(300);              // ให้ USB CDC ขึ้นก่อนตอนเปิดเครื่องใหม่
  setCpuFrequencyMhz(80);

  hwInitPins();
  hwInitI2C();
  hwInitPmu();
  hwInitSd();                             // ต้องมาก่อนโหลด settings

  settingsLoad();
  setCpuFrequencyMhz(settings.cpuMhz);

  hwInitDisplay(false);
  hwInitRtc();
  hwInitImu();
  imuConfigureWom();
  touchInit();
  uiInit();

#if ENABLE_AUDIO
  audioInit();
#endif
  modulesInit();

  struct tm t; hwNowLocal(t);
  faceInvalidate();
  faceDrawFull(t);
  s_lastMin = t.tm_min;

  // ตื่นจาก deep sleep ด้วย timer → อัปเดตเวลาเงียบ ๆ ไม่ต้องเปิดจอสว่าง
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  if (!fromDeep || cause == ESP_SLEEP_WAKEUP_EXT1) setBright(true);
  else                                             setBright(false);

  Serial.printf("[boot] S3 Watch %s | %s | mode=%s\n", FW_VERSION,
                fromDeep ? "ตื่นจาก deep sleep" : "เปิดเครื่อง",
                powerModeName(powerPickMode(false, false)));
}

// ============================================================================
void loop() {
  struct tm t; hwNowLocal(t);

  handleButton();
  Gesture g = touchPoll();

  // ───────── โหมดเมนู Settings ─────────
  if (uiIsOpen()) {
    if (g != GST_NONE) { uiGesture(g); keepBrightAlive(); }
    uiTick();
    if (s_bright && millis() >= s_brightUntil) setBright(false);
    return;                               // ไม่หลับขณะเปิดเมนู
  }

  // ───────── โหมดนาฬิกา ─────────
  if (g == GST_TAP) keepBrightAlive();

  if (t.tm_min != s_lastMin) {
    faceUpdate(t);
    s_lastMin = t.tm_min;
    modulesMinute(t);
  }

  faceSetRecording(audioIsRecording());

  if (s_bright && millis() >= s_brightUntil) setBright(false);

  // กดปุ่มค้างอยู่ / เพิ่งมีการแตะจอ → ยังไม่หลับ เดี๋ยวพลาดอินพุต
  if (s_btnDown) { delay(20); return; }

  PowerMode mode = powerPickMode(audioIsRecording(), false);
  WakeReason wr  = powerSleep(mode, s_bright ? s_brightUntil : 0);

  switch (wr) {
    case WK_MOTION:
      // ขยับแล้ว — ต้องหงายจอค้างครบเวลาก่อนถึงจะเร่งแสง (กันสั่นสะเทือนหลอก)
      if (imuFaceUpHeld()) {
        keepBrightAlive();
        faceUpdate(t);
        modulesRaise();
      }
      break;
    case WK_TOUCH:
      keepBrightAlive();
      break;
    case WK_BUTTON:
      s_btnDown = true; s_btnT0 = millis(); s_btnFired = false;
      break;
    default:
      break;                              // WK_TIMER → ปล่อยให้ลูปถัดไปอัปเดตเวลา
  }
}

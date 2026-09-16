// ============================================================================
//  S3 Watch — เฟิร์มแวร์นาฬิกา + เครื่องบันทึกเสียง
//  บอร์ด : Waveshare ESP32-S3-Touch-AMOLED-2.06 (ESP32-S3, PSRAM 8 MB)
//  Core  : Arduino-ESP32 3.x
//  ไลบรารี : GFX Library for Arduino, SensorLib (Lewis He), LVGL (ถ้า USE_LVGL=1)
//
//  แนวคิดหลัก — นาฬิกาคือแกน ห้ามถูกรบกวน
//    • หน้าปัดเส้น 1 พิกเซล (อนาล็อก/ดิจิทัล) อัปเดตแบบลบเฉพาะสิ่งที่เปลี่ยน
//    • เลือกโหมดหลับอัตโนมัติ : DEEP / LIGHT / ACTIVE (ดู power.h)
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
#include "esp_sleep.h"
#include "config.h"
#include "hw.h"
#include "settings.h"
#include "display_1px.h"
#include "imu_wake.h"
#include "touch.h"
#include "power.h"
#include "audio_rec.h"
#include "ui_menu.h"
#include "modules.h"

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

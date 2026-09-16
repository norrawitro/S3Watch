#include "power.h"
#include "config.h"
#include "hw.h"
#include "settings.h"
#include "imu_wake.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
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

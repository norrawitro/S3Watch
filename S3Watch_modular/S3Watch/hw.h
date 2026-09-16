// ============================================================================
//  hw.h — อุปกรณ์ทั้งหมดบนบอร์ด + การเริ่มต้นใช้งาน
// ============================================================================
#pragma once
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

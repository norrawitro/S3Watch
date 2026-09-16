// ============================================================================
//  settings.h — ค่าตั้งทั้งหมดของเครื่อง
//  เก็บหลักที่ SD card (/settings.cfg แบบ key=value แก้ด้วย Notepad ได้)
//  มิเรอร์ลง NVS ด้วย → ถอด SD ออกแล้วเครื่องยังจำค่าเดิมและรันต่อได้
// ============================================================================
#pragma once
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

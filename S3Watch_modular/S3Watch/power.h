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
#pragma once
#include <Arduino.h>

enum PowerMode : uint8_t { PM_ACTIVE = 0, PM_LIGHT, PM_DEEP };
enum WakeReason : uint8_t { WK_NONE = 0, WK_TIMER, WK_MOTION, WK_TOUCH, WK_BUTTON };

PowerMode  powerPickMode(bool recording, bool uiOpen);
WakeReason powerSleep(PowerMode mode, uint32_t brightUntilMs);
const char *powerModeName(PowerMode m);
bool       powerWokeFromDeepSleep();     // เรียกได้ใน setup()

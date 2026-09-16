// ============================================================================
//  imu_wake.h — QMI8658 : Wake-on-Motion + ตรวจ "หงายข้อมือ"
//  เงื่อนไขหงาย: หน้าจอชี้ขึ้นภายใน ±faceAngleDeg (ค่าเริ่มต้น 15°)
//                และต้องค้างต่อเนื่อง faceHoldMs (ค่าเริ่มต้น 200 ms)
// ============================================================================
#pragma once
#include <Arduino.h>

void imuConfigureWom();        // ตั้ง Wake-on-Motion ให้ INT ปลุก MCU
bool imuFaceUpNow();           // อ่านค่าครั้งเดียว
bool imuFaceUpHeld();          // ต้องเข้าเงื่อนไขค้างครบเวลาถึงจะคืน true
int  imuIntIdleLevel();        // ระดับลอจิกของขา INT ตอนไม่มีการขยับ
void imuLatchIntIdle();        // จำระดับปัจจุบันไว้ก่อนเข้า sleep

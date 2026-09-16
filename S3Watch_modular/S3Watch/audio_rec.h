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
#pragma once
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

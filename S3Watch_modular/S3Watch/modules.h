// ============================================================================
//  modules.h — จุดเสียบฟีเจอร์ใหม่ในอนาคต (n-lang, RD4 event, 8D index, STT)
//  เพิ่มฟังก์ชันใหม่ได้โดยไม่ต้องแตะแกนนาฬิกาเลย — แค่ใส่รายการใน MODULES[]
//
//  hook 3 จุด:
//     init()          — ตอนบูต
//     onMinute(tm)    — ทุกครั้งที่นาทีเปลี่ยน (MCU ตื่นอยู่แล้ว)
//     onRaise()       — ทุกครั้งที่ผู้ใช้หงายข้อมือ
// ============================================================================
#pragma once
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

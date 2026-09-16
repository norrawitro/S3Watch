// ============================================================================
//  touch.h — จอสัมผัส (FT3168 / FT6336 register map, I2C 0x38)
//  ท่าทางตามสเปก: ปัดขึ้น/ลง = เลื่อนเมนู, ปัดขวา = เลือก/ตกลง,
//                 ปัดซ้าย = ย้อนกลับ, แตะ = ปลุกจอ
// ============================================================================
#pragma once
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

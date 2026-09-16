#include "imu_wake.h"
#include "config.h"
#include "hw.h"
#include "settings.h"
#include "driver/gpio.h"
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

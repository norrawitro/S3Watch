// ส่งผลวิเคราะห์เสียงเข้า WKW (ระบบ IoT ของผู้ใช้) [ผู้ใช้สั่ง 2026-10-18]
// สเปกจากเซสชัน WKW: HTTPS POST https://<WKW_HOST>/iot/device/events · header X-Device-Key · body {"events":[...]}
//   ≤ 64 KB/คำขอ · ≤ 100 เหตุการณ์/คำขอ · ≤ 600 คำขอ/นาที · อ่านกลับ GET ...?type=sound_event&limit=N
// ค่าลับ WKW_HOST / WKW_KEY อยู่ใน secrets.h (ไม่ขึ้น GitHub) · ไม่มี key = ปิดฟีเจอร์นี้เอง
// ส่งในงานแยก (core 0) — TLS ใช้เวลาเป็นวินาที ห้ามบล็อกงานเสียง · ส่งไม่ได้ = เก็บไว้ส่งรอบหน้า (สูงสุด ~48 KB)
// ไม่ส่งเสียงดิบ — มีแต่ตัวเลขผลวิเคราะห์
#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "wkw_ca.h"

#ifndef WKW_HOST
#define WKW_HOST ""
#endif
#ifndef WKW_KEY
#define WKW_KEY ""
#endif

class Wkw {
 public:
  volatile uint32_t sent = 0, failed = 0;   // เหตุการณ์ที่ส่งสำเร็จ / คำขอที่ล้มเหลว
  volatile int lastCode = 0;                 // รหัส HTTP ล่าสุด (200 = OK · <0 = ต่อไม่ได้)

  bool enabled() const { return strlen(WKW_HOST) && strlen(WKW_KEY); }

  void begin() {
    if (!enabled()) { Serial.println("WKW: off (no WKW_HOST/WKW_KEY in secrets.h)"); return; }
    mtx_ = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(task, "wkw", 8192, this, 1, nullptr, 0);
    Serial.printf("WKW: on -> %s\n", WKW_HOST);   // ไม่พิมพ์ key
  }

  // เพิ่ม 1 เหตุการณ์ (JSON ก้อนเดียว) เข้าคิว — งานแยกจะรวมส่งทุก 1 วิ
  void queue(const String &json) {
    if (!enabled() || !mtx_) return;
    xSemaphoreTake(mtx_, portMAX_DELAY);
    if (batch_.length() + json.length() > 48000) { batch_ = ""; n_ = 0; dropped++; }   // ค้างนานเกิน → ทิ้งชุดเก่า
    if (n_) batch_ += ',';
    batch_ += json; n_++;
    xSemaphoreGive(mtx_);
  }

  uint32_t dropped = 0;

 private:
  SemaphoreHandle_t mtx_ = nullptr;
  String batch_;
  int n_ = 0;

  int post(WiFiClientSecure &tls, const String &body) {
    HTTPClient http;
    http.begin(tls, String("https://") + WKW_HOST + "/iot/device/events");
    http.addHeader("Content-Type", "application/json");
    http.addHeader("X-Device-Key", WKW_KEY);
    http.setTimeout(8000);
    int code = http.POST(body);
    if (code != 200) Serial.printf("WKW %d %s\n", code, code > 0 ? http.getString().substring(0, 120).c_str() : "");
    http.end();
    return code;
  }

  static void task(void *arg) {
    Wkw *self = (Wkw *)arg;
    WiFiClientSecure tls;
    tls.setCACert(WKW_CA);
    for (;;) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      if (WiFi.status() != WL_CONNECTED) continue;
      String body; int n;
      xSemaphoreTake(self->mtx_, portMAX_DELAY);
      n = self->n_;
      if (n) { body = "{\"events\":[" + self->batch_ + "]}"; }
      xSemaphoreGive(self->mtx_);
      if (!n) continue;
      int code = self->post(tls, body);
      self->lastCode = code;
      // สำเร็จ หรือข้อมูลผิดรูปแบบ (ส่งซ้ำก็ไม่ผ่าน) → ล้างส่วนที่ส่งไป · อย่างอื่น (เน็ตหลุด 429 5xx) → เก็บไว้ลองใหม่
      if (code == 200 || code == 400 || code == 403 || code == 413) {
        xSemaphoreTake(self->mtx_, portMAX_DELAY);
        // ระหว่างส่งอาจมีเหตุการณ์ใหม่ต่อท้าย → ตัดเฉพาะส่วนที่ส่งไปแล้ว
        int cut = body.length() - 13;              // ความยาว batch ตอนส่ง = body − {"events":[ ]}
        self->batch_ = (int)self->batch_.length() > cut + 1 ? self->batch_.substring(cut + 1) : "";
        self->n_ -= n;
        xSemaphoreGive(self->mtx_);
        if (code == 200) self->sent += n; else self->failed++;
      } else self->failed++;
    }
  }
};

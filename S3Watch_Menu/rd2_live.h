// RD2 บนบอร์ด (หน้า RD2 TEST) — ตรรกะเดียวกับ tools/rd2.py ที่ทดสอบแล้ว (บันทึกการทดลอง 2026-10-17)
// พื้นหลัง = แต่ละช่วงจำระดับปกติ + ความแกว่งปกติ · ส่วนเกิน = ค่า − (ระดับ + 2.5 × แกว่ง)
// เฟรมมีเสียง = ค่าเฉลี่ยส่วนเกิน 8 ช่วงสูงสุด ≥ 5 dB · เริ่มเหตุการณ์ 2 เฟรมติด · จบเมื่อเงียบ 10 เฟรม
// ค่าทั้งหมด [ข้อเสนอ] — ต้องเท่ากับ CFG ใน tools/rd2.py
#pragma once
#include <Arduino.h>
#include "rd1.h"

struct Rd2Cfg {
  int learn_frames = 25;                 // 0.5 วิแรกต้องเงียบ
  float alpha = 0.02f, alpha_event = 0.002f, k_dev = 2.5f, on_db = 5.0f;
  int top_bands = 8, start_frames = 2, end_frames = 10;
};

class Rd2Live {
 public:
  Rd2Cfg cfg;
  float bg[RD1_BANDS], dev[RD1_BANDS], ex[RD1_BANDS];   // dB (เทียบกันเอง ใช้ค่า RD1 × 0.5)
  float score = 0;
  bool active = false, inEvent = false;
  uint32_t events = 0;
  int learned = 0;                       // < learn_frames = กำลังเรียนพื้นหลัง

  void reset() { learned = 0; inEvent = active = false; onRun_ = offRun_ = 0; events = 0; score = 0; }
  bool learning() const { return learned < cfg.learn_frames; }

  void update(const Rd1Frame &f) {
    float x[RD1_BANDS];
    for (int b = 0; b < RD1_BANDS; b++) x[b] = f.band[b] * RD1_CFG.log_step_db;
    if (learning()) {                    // เก็บ 25 เฟรมแรก → ระดับเฉลี่ย + ความแกว่งเฉลี่ย
      for (int b = 0; b < RD1_BANDS; b++) hist_[learned][b] = x[b];
      if (++learned == cfg.learn_frames) {
        for (int b = 0; b < RD1_BANDS; b++) {
          float m = 0, d = 0;
          for (int i = 0; i < cfg.learn_frames; i++) m += hist_[i][b];
          m /= cfg.learn_frames;
          for (int i = 0; i < cfg.learn_frames; i++) d += fabsf(hist_[i][b] - m);
          bg[b] = m; dev[b] = max(1.0f, d / cfg.learn_frames);
        }
      }
      for (int b = 0; b < RD1_BANDS; b++) ex[b] = 0;
      score = 0; active = false;
      return;
    }
    float s[RD1_BANDS];
    for (int b = 0; b < RD1_BANDS; b++) { ex[b] = max(0.0f, x[b] - bg[b] - cfg.k_dev * dev[b]); s[b] = ex[b]; }
    std::sort(s, s + RD1_BANDS, [](float a, float c) { return a > c; });
    score = 0;
    for (int i = 0; i < cfg.top_bands; i++) score += s[i];
    score /= cfg.top_bands;
    active = score >= cfg.on_db;
    onRun_ = active ? onRun_ + 1 : 0;
    offRun_ = active ? 0 : offRun_ + 1;
    if (!inEvent && onRun_ >= cfg.start_frames) { inEvent = true; events++; }
    else if (inEvent && offRun_ >= cfg.end_frames) inEvent = false;
    float a = (inEvent || active) ? cfg.alpha_event : cfg.alpha;
    for (int b = 0; b < RD1_BANDS; b++) {
      dev[b] = max(1.0f, dev[b] + a * (fabsf(x[b] - bg[b]) - dev[b]));
      bg[b] += a * (x[b] - bg[b]);
    }
  }

 private:
  float hist_[25][RD1_BANDS];
  int onRun_ = 0, offRun_ = 0;
};

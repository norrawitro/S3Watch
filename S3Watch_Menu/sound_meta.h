// Sound Metadata ส่วนที่ "วัดจากเสียงได้ตรง" (กลุ่ม 🟢 ใน docs/RHEA_sound_metadata.md) — ต่อ 1 เหตุการณ์ RD2
// [ผู้ใช้สั่ง 2026-10-18 "จากข้อมูล sound metadata ทำสิ่งที่ทำได้ก่อน"] · ตรรกะเดียวกับ tools/sound_meta.py (ทดสอบแล้วบนคอม)
// ผล = รหัสข้อย่อยตามตาราง RHEA 1–10: เมตา 4 6 7 8 9 10 11 + ต้นไม้ชั้น 1 3 4 6 7 · 0 = ไม่ได้คิด
// เกณฑ์ทุกตัว [ข้อเสนอ] ยังไม่ได้ปรับกับเสียงจริง · ความดังยังไม่ได้ปรับเทียบไมค์ (dB เทียบกันเอง)
#pragma once
#include <Arduino.h>
#include <math.h>
#include "rd1.h"

#define SM_MAX_FRAMES 250                // เก็บรายละเอียด 5 วิแรกของเหตุการณ์ (ยาวกว่านี้นับเวลาต่อ)
#define SM_FRAME_MS   20

struct SmFrame { float db, cent, hilo, flat, semi, clar; };    // semi < 0 = ไม่มีระดับเสียง

struct SoundMeta {
  uint8_t M4, M6, M7, M8, M9, M10, M11;  // เมตาดาต้า 11 มิติ (ข้อที่วัดได้)
  uint8_t T1, T3, T4, T6, T7;            // ต้นไม้เสียง 10 ชั้น (ชั้นที่วัดได้)
  uint8_t syll;                          // จำนวนพยางค์ที่นับได้
  uint32_t ms;                           // ความยาวเหตุการณ์
  void line(char *b, size_t n) const {
    snprintf(b, n, "M4.%u M6.%u M7.%u M8.%u M9.%u M10.%u M11.%u T1.%u T3.%u T4.%u T6.%u T7.%u",
             M4, M6, M7, M8, M9, M10, M11, T1, T3, T4, T6, T7);
  }
};

class SoundMetaBuilder {
 public:
  void begin(uint32_t onsetMs) { n_ = 0; total_ = 0; onset_ = onsetMs; }

  void add(const Rd1Frame &f) {
    total_++;
    if (n_ >= SM_MAX_FRAMES) return;
    SmFrame &s = fr_[n_++];
    double pw[RD1_BANDS], sum = 0, num = 0, lg = 0, lo = 0, hi = 0;
    for (int b = 0; b < RD1_BANDS; b++) {
      float db = (f.band[b] - RD1_CFG.log_offset) * RD1_CFG.log_step_db;
      pw[b] = pow(10.0, db / 10.0); sum += pw[b];
      num += pw[b] * (RD1_EDGES_HZ[b] + RD1_EDGES_HZ[b + 1]) / 2.0;
      lg += log(pw[b] + 1e-12);
      if (b < 8) lo += db; else if (b < 28) hi += db;
    }
    if (sum <= 0) sum = 1e-12;
    s.db = (f.total - RD1_CFG.log_offset) * RD1_CFG.log_step_db;
    s.cent = num / sum;
    s.hilo = hi / 20 - lo / 8;
    s.flat = exp(lg / RD1_BANDS) / (sum / RD1_BANDS);           // 1 = แบนทุกช่วง (ซ่า)
    s.semi = f.f0_cents ? f.f0_cents / 100.0f : -1;
    s.clar = f.clarity;
  }

  // ปิดเหตุการณ์ → รหัส · เก็บเวลาเริ่มไว้ดูจังหวะครั้งต่อไป
  // trim = เฟรมเงียบท้ายที่ RD2 รอก่อนปิดเหตุการณ์ (end_frames) — ตัดทิ้ง ไม่นับเป็นเสียง
  SoundMeta finish(int trim = 0) {
    trim = min(trim, (int)total_ - 1);
    if (trim > 0) { total_ -= trim; if (n_ > (int)total_) n_ = total_; }
    SoundMeta m = {};
    int n = max(1, n_);
    m.ms = total_ * SM_FRAME_MS;
    float meanDb = 0, cmean = 0, flat = 0, hilo = 0, mx = -1e9f, mn = 1e9f, jit = 0;
    int pk = 0, nv = 0;
    for (int i = 0; i < n; i++) {
      const SmFrame &s = fr_[i];
      meanDb += s.db; cmean += s.cent; flat += s.flat; hilo += s.hilo;
      if (s.db > mx) { mx = s.db; pk = i; }
      mn = min(mn, s.db);
      if (i) jit += fabsf(s.db - fr_[i - 1].db);
      if (voiced(s)) nv++;
    }
    meanDb /= n; cmean /= n; flat /= n; hilo /= n;
    jit = n > 1 ? jit / (n - 1) : 0;
    float cstd = 0;
    for (int i = 0; i < n; i++) cstd += (fr_[i].cent - cmean) * (fr_[i].cent - cmean);
    cstd = sqrtf(cstd / n);
    float vfrac = (float)nv / n;
    int half = max(1, n / 2);
    float first = 0, last = 0;
    for (int i = 0; i < half; i++) { first += fr_[i].db; last += fr_[n - half + i].db; }
    first /= half; last /= half;
    float swing = mx - mn, pos = (float)pk / max(1, n - 1);
    int peaks = 0;
    for (int i = 1; i < n - 1; i++) if (fr_[i].db > fr_[i - 1].db + 3 && fr_[i].db > fr_[i + 1].db + 3) peaks++;

    // เมตา 8 / ต้นไม้ 1: ย่านความถี่เด่น
    { const float e8[] = {150, 400, 800, 1600, 3000, 6000}; const float e1[] = {60, 250, 500, 2000, 4000, 6000, 8000};
      m.M8 = cstd > 1500 ? 10 : flat > 0.6f ? 8 : bucket(cmean, e8, 6);
      m.T1 = cstd > 1500 ? 10 : flat > 0.6f ? 9 : bucket(cmean, e1, 7); }

    // เมตา 11 / ต้นไม้ 4: ความดัง + การเปลี่ยน
    { const float e11[] = {-55, -40, -20}; const float e4[] = {-70, -60, -50, -40, -30, -15};
      m.M11 = jit > 6 ? 5 : last - first > 6 ? 7 : first - last > 6 ? 8 : swing > 15 ? 10 : bucket(meanDb, e11, 3);
      m.T4 = last - first > 6 ? 8 : first - last > 6 ? 9 : swing > 15 ? 10 : bucket(meanDb, e4, 6); }

    // เมตา 7 / ต้นไม้ 6: ซองพลังงาน
    if (n <= 5 && pos < 0.5f) m.M7 = 1;
    else if (peaks >= 3) m.M7 = 7;
    else if (pos < 0.2f && first - last > 6) m.M7 = 5;
    else if (pos > 0.8f && last - first > 6) m.M7 = pos < 0.95f ? 4 : 9;
    else if (pos >= 0.3f && pos <= 0.7f && swing > 10) m.M7 = peaks <= 1 ? 8 : 2;
    else if (swing < 8 && m.ms >= 500) m.M7 = 3;
    else m.M7 = 10;
    if (m.ms < 100) m.T6 = 8;
    else if (pos < 0.2f && first - last > 6) m.T6 = m.ms < 300 ? 1 : 2;
    else if (last - first > 6) m.T6 = 3;
    else if (first - last > 6) m.T6 = 4;
    else if (swing < 8 && m.ms >= 500) m.T6 = 5;
    else if (peaks >= 3) m.T6 = 6;
    else m.T6 = 10;

    // เมตา 9 / ต้นไม้ 7: ทิศทาง pitch (วรรณยุกต์)
    if (nv < 3) { m.M9 = 10; m.T7 = 8; }
    else {
      static float s[SM_MAX_FRAMES];
      int k = 0;
      for (int i = 0; i < n; i++) if (voiced(fr_[i])) s[k++] = fr_[i].semi;
      int t = max(1, k / 3);
      float a = 0, c = 0, b = s[k / 2], hiS = -1e9f, loS = 1e9f, upJ = -1e9f, dnJ = 1e9f;
      for (int i = 0; i < t; i++) { a += s[i]; c += s[k - t + i]; }
      a /= t; c /= t;
      for (int i = 0; i < k; i++) { hiS = max(hiS, s[i]); loS = min(loS, s[i]); if (i) { upJ = max(upJ, s[i] - s[i - 1]); dnJ = min(dnJ, s[i] - s[i - 1]); } }
      if (upJ > 3) { m.M9 = 6; m.T7 = 6; }
      else if (dnJ < -3) { m.M9 = 7; m.T7 = 6; }
      else if (hiS - loS < 1) { m.M9 = 1; m.T7 = 1; }          // คงที่ (สามัญ)
      else if (b - a > 1 && b - c > 1) { m.M9 = 4; m.T7 = 4; } // ขึ้นแล้วลง (โท)
      else if (a - b > 1 && c - b > 1) { m.M9 = 5; m.T7 = 5; } // ลงแล้วขึ้น (จัตวา)
      else if (c - a > 1) { m.M9 = 2; m.T7 = 2; }              // ค่อยๆ สูงขึ้น (ตรี)
      else if (a - c > 1) { m.M9 = 3; m.T7 = 3; }              // ค่อยๆ ต่ำลง (เอก)
      else { m.M9 = 8; m.T7 = 7; }
    }

    // เมตา 4: จำนวนพยางค์ (ช่วงมีระดับเสียงติดกัน ≥ 3 เฟรม)
    { int runs = 0, cur = 0;
      for (int i = 0; i < n; i++) { if (voiced(fr_[i])) cur++; else { runs += cur >= 3; cur = 0; } }
      runs += cur >= 3;
      m.syll = runs;
      m.M4 = runs == 0 ? (m.ms < 300 ? 9 : 10) : runs <= 5 ? runs : runs <= 7 ? 6 : runs <= 10 ? 7 : 8; }

    // ต้นไม้ 3: เนื้อเสียง · เมตา 10 (คร่าว ๆ): ลักษณะเสียง
    { float clar = 0; int c = 0;
      for (int i = 0; i < n; i++) if (voiced(fr_[i])) { clar += fr_[i].clar; c++; }
      clar = c ? clar / c : 0;
      if (m.ms < 100 && vfrac < 0.5f) m.T3 = 5;
      else if (vfrac > 0.7f && clar > 230 && flat < 0.1f) m.T3 = 1;
      else if (vfrac > 0.6f && clar > 200) m.T3 = 2;
      else if (vfrac > 0.3f) m.T3 = 3;
      else if (hilo > 3 || flat > 0.5f) m.T3 = 4;
      else m.T3 = 10;
      if (vfrac > 0.5f && clar > 220 && cmean > 1500) m.M10 = 1;
      else if (cmean < 600) m.M10 = 2;
      else if (vfrac > 0.5f && clar < 180) m.M10 = 4;
      else if (vfrac > 0.3f && hilo > 0) m.M10 = 7;
      else m.M10 = 10; }

    // เมตา 6: จังหวะ (จากเวลาเริ่มของเหตุการณ์ก่อน ๆ)
    if (nOn_ < 2) m.M6 = m.ms >= 1000 ? 4 : 10;
    else {
      float gaps[7]; int g = 0;
      for (int i = 1; i < nOn_; i++) gaps[g++] = onsets_[i] - onsets_[i - 1];
      float mean = 0; for (int i = 0; i < g; i++) mean += gaps[i]; mean /= g;
      float sd = 0; for (int i = 0; i < g; i++) sd += (gaps[i] - mean) * (gaps[i] - mean);
      float cv = sqrtf(sd / g) / max(mean, 1.0f);
      if (g >= 2 && gaps[g - 1] < gaps[0] * 0.7f) m.M6 = 5;
      else if (g >= 2 && gaps[g - 1] > gaps[0] * 1.4f) m.M6 = 6;
      else if (cv < 0.2f) m.M6 = mean < 1500 ? 2 : 1;
      else if (mean < 400) m.M6 = 3;
      else if (cv > 0.6f) m.M6 = 9;
      else m.M6 = 10;
    }
    if (nOn_ == 8) { memmove(onsets_, onsets_ + 1, 7 * sizeof(uint32_t)); nOn_ = 7; }
    onsets_[nOn_++] = onset_;            // เก็บหลังคิด → ครั้งนี้เทียบกับครั้งก่อน ๆ (เหมือน tools/sound_meta.py)
    return m;
  }

  void resetRhythm() { nOn_ = 0; }

 private:
  SmFrame fr_[SM_MAX_FRAMES];
  int n_ = 0;
  uint32_t total_ = 0, onset_ = 0, onsets_[8];
  int nOn_ = 0;
  static bool voiced(const SmFrame &s) { return s.semi >= 0 && s.clar >= 150; }
  static uint8_t bucket(float v, const float *e, int ne) { for (int i = 0; i < ne; i++) if (v < e[i]) return i + 1; return ne + 1; }
};

// RD1 = เสียงที่แปลง FFT แล้ว (ผู้ใช้กำหนด 2026-10-17 — ดู บันทึกการทดลอง.md)
//   ชิ้น 20 ms (640 ตัวอย่างที่ 32 kHz) → Hann → FFT 1024 → พลังงาน 32 ช่วงไม่เท่ากัน (โฟกัส 3–8.7 kHz) → uint8 สเกล log
// ป้ายที่มา: [ผู้ใช้] = ผู้ใช้กำหนด · [ข้อเสนอ] = Claude ตั้ง ผู้ใช้รับไว้ใช้ก่อน แก้ได้ภายหลัง (แก้ใน RD1_CFG / RD1_EDGES_HZ)
// รุ่นก่อน (RD2 ตามบลูพรินต์ N-tag v0.1: 16 kHz · FFT 512 · ภาคผนวก ข) อยู่ที่ archive/rd2_ntag_v0.1/
#pragma once
#include <Arduino.h>
#include <math.h>

#define RD_SAMPLE_RATE 32000             // [ข้อเสนอ] เดิม 16 kHz — ต้องเห็นถึง 16 kHz สำหรับเสียงเสียดแทรก
#define RD_FRAME_N     640               // 20 ms [ผู้ใช้ยังไม่ยืนยัน — เคยพูดถึง 2 ms]
#define RD1_FFT_N      1024              // ช่องละ 31.25 Hz [ข้อเสนอ]
#define RD1_BANDS      32                // [ผู้ใช้]

// ---------- ค่าตั้ง (แก้ได้โดยไม่แก้โค้ดคำนวณ) ----------
struct Rd1Config {
  float log_step_db;                     // หนึ่งหน่วย uint8 = กี่ dB          [ข้อเสนอ] 0.5
  float log_offset;                      // value = dB / step + offset        [ข้อเสนอ] ดูคำอธิบายข้างล่าง
  bool  drop_dc;                         // ตัดช่อง FFT 0 (ไฟตรง)              [ข้อเสนอ]
};
// สเกล: พลังงานหารด้วย (N/4)² → ไซน์เต็มสเกล (แอมพลิจูด 1.0) ในช่วงเดียว ≈ +1.8 dB (หน่วยใกล้ dBFS)
// offset 240 → 0 dB = 240 · ค่า 0 = −120 dB · ค่า 255 = +7.5 dB — ต้องปรับเทียบกับไมค์จริงภายหลัง
const Rd1Config RD1_CFG = {0.5f, 240.0f, true};

// ขอบช่วง (Hz) 33 ค่า = 32 ช่วง [ข้อเสนอ — ผู้ใช้รับไว้ใช้ก่อน]
// 0–3 kHz: 8 ช่วง · 3–8.7 kHz: 20 ช่วง (แคบสุด 160 Hz ที่ 5.04–5.2 kHz) · 8.7–16 kHz: 4 ช่วง
const uint16_t RD1_EDGES_HZ[RD1_BANDS + 1] = {
  0, 100, 250, 435, 700, 1060, 1530, 2180,
  3000, 3300, 3580, 3840, 4080, 4300, 4500, 4690, 4870, 5040, 5200, 5375,
  5585, 5835, 6125, 6455, 6825, 7235, 7685, 8185,
  8735, 9400, 10500, 12500, 16000};

// ---------- ระดับเสียงพื้นฐาน f0 (ผู้ใช้อนุมัติ 2026-10-18) ----------
// 32 ช่วงแยกโน้ตไม่ได้ (C4–C5 ทั้งคู่แปดตกในช่วง 3–4) → เพิ่ม f0 ต่อเฟรม ใช้แยกโน้ต/วรรณยุกต์ · ออกเป็นเลขโน้ต MIDI ได้ตรง
// วิธี YIN [ข้อเสนอ]: ลดเหลือ 16 kHz (เฉลี่ยคู่) → 320 จุด · หาคาบ 100–1000 Hz (G2–B5) · หน้าต่างเทียบ 160 จุด (10 ms)
#define RD1_F0_MIN_HZ    100.0f
#define RD1_F0_MAX_HZ    1000.0f
#define RD1_F0_THRESHOLD 0.15f           // YIN: ค่าต่ำกว่านี้ = มีคาบชัด (เสียงมีระดับเสียง)

// ---------- ข้อมูลออก — 41 ไบต์ (รุ่น 1 = 38 ไบต์ ไม่มี f0) ----------
#define RD1_FLAG_GAP 0x01                // บิต 0 = เสียงขาด (I2S ส่งไม่ครบ เติมศูนย์)
struct __attribute__((packed)) Rd1Frame {
  uint32_t seq;                          // เลขลำดับชิ้นนับจากเปิดเครื่อง (เวลา = seq × 20 ms)
  uint8_t  band[RD1_BANDS];              // พลังงาน 32 ช่วง สเกล log
  uint8_t  total;                        // พลังงานรวม สเกลเดียวกัน
  uint8_t  flags;
  uint16_t f0_cents;                     // เลขโน้ต MIDI × 100 (C4 = 6000, A4 = 6900) · 0 = ไม่มีระดับเสียง (เสียงซ่า/เงียบ)
  uint8_t  clarity;                      // ความชัดของคาบ 0–255 (255 = ชัดมาก)
};
static_assert(sizeof(Rd1Frame) == 41, "Rd1Frame must be 41 bytes");

// ---------- หัวไฟล์ .rd1 (256 ไบต์, little-endian) [ข้อเสนอ] ----------
struct __attribute__((packed)) Rd1FileHeader {
  char     magic[4];                     // "NRD1"
  uint16_t version;                      // 2 (เพิ่ม f0_cents + clarity ต่อเฟรม)
  uint16_t header_size;                  // 256
  uint32_t sample_rate;                  // 32000
  uint16_t frame_n;                      // 640
  uint16_t fft_n;                        // 1024
  uint8_t  n_bands;                      // 32
  uint8_t  drop_dc;                      // 1 = ตัดช่องไฟตรง
  uint16_t frame_size;                   // 41
  float    log_step_db;                  // 0.5
  float    log_offset;                   // 240
  uint32_t n_frames;                     // จำนวนชิ้นในไฟล์
  uint32_t fw_version;                   // FW_VERSION ที่สร้างไฟล์
  uint16_t edges_hz[RD1_BANDS + 1];      // ขอบช่วงตามตั้งค่า
  uint16_t band_start[RD1_BANDS];        // ช่อง FFT แรกของแต่ละช่วง (รวม)
  uint16_t band_end[RD1_BANDS];          // ช่อง FFT สุดท้าย (รวม)
  uint8_t  reserved[26];                 // สงวน (ให้หัวไฟล์ครบ 256 ไบต์)
};
static_assert(sizeof(Rd1FileHeader) == 256, "Rd1FileHeader must be 256 bytes");

// ---------- RD1 ----------
class Rd1 {
 public:
  uint16_t start[RD1_BANDS], end[RD1_BANDS];   // ช่อง FFT ของแต่ละช่วง (คำนวณจาก RD1_EDGES_HZ)

  void begin() {
    for (int i = 0; i < RD_FRAME_N; i++) win_[i] = 0.5f - 0.5f * cosf(2 * PI * i / RD_FRAME_N);   // Hann 640 (periodic)
    for (int k = 0; k < RD1_FFT_N / 2; k++) { cos_[k] = cosf(2 * PI * k / RD1_FFT_N); sin_[k] = -sinf(2 * PI * k / RD1_FFT_N); }
    // ช่อง k (ความถี่กลาง k × 31.25 Hz) อยู่ในช่วงที่ ขอบล่าง ≤ ความถี่ < ขอบบน
    const float hz = (float)RD_SAMPLE_RATE / RD1_FFT_N;
    for (int b = 0; b < RD1_BANDS; b++) {
      start[b] = (uint16_t)ceilf(RD1_EDGES_HZ[b] / hz);
      end[b]   = (uint16_t)ceilf(RD1_EDGES_HZ[b + 1] / hz) - 1;
    }
    if (RD1_CFG.drop_dc && start[0] == 0) start[0] = 1;
  }

  void fillHeader(Rd1FileHeader &h, uint32_t nFrames, uint32_t fw) const {
    memset(&h, 0, sizeof(h));
    memcpy(h.magic, "NRD1", 4);
    h.version = 2; h.header_size = sizeof(h);
    h.sample_rate = RD_SAMPLE_RATE; h.frame_n = RD_FRAME_N; h.fft_n = RD1_FFT_N;
    h.n_bands = RD1_BANDS; h.drop_dc = RD1_CFG.drop_dc; h.frame_size = sizeof(Rd1Frame);
    h.log_step_db = RD1_CFG.log_step_db; h.log_offset = RD1_CFG.log_offset;
    h.n_frames = nFrames; h.fw_version = fw;
    memcpy(h.edges_hz, RD1_EDGES_HZ, sizeof(h.edges_hz));
    memcpy(h.band_start, start, sizeof(h.band_start));
    memcpy(h.band_end, end, sizeof(h.band_end));
  }

  // ชิ้นเสียง 640 ตัวอย่าง → 1 เฟรม RD1 · คืนเวลาที่ใช้ (µs)
  uint32_t compute(const int16_t *pcm, uint32_t seq, uint8_t flags, Rd1Frame &out) {
    uint32_t t0 = micros();
    for (int i = 0; i < RD1_FFT_N; i++) {
      re_[i] = i < RD_FRAME_N ? pcm[i] * (1.0f / 32768.0f) * win_[i] : 0.0f;   // เติมศูนย์ 384 จุด
      im_[i] = 0.0f;
    }
    fft();
    const double norm = 1.0 / ((RD_FRAME_N / 4.0) * (RD_FRAME_N / 4.0));
    double total = 0;
    for (int b = 0; b < RD1_BANDS; b++) {
      double e = 0;
      for (int k = start[b]; k <= end[b]; k++) e += (double)re_[k] * re_[k] + (double)im_[k] * im_[k];
      e *= norm;
      out.band[b] = toLog(e);
      total += e;
    }
    out.total = toLog(total);
    out.seq = seq; out.flags = flags;
    pitch(pcm, out);
    return micros() - t0;
  }

  static uint8_t toLog(double e) {       // 10·log10(E + ε) → uint8 ขั้นละ log_step_db
    float db = 10.0f * log10f((float)e + 1e-15f);
    float v = db / RD1_CFG.log_step_db + RD1_CFG.log_offset;
    return (uint8_t)constrain(lroundf(v), 0, 255);
  }

 private:
  // f0 แบบ YIN บนเสียงที่ลดเหลือ 16 kHz · ผลเป็นเลขโน้ต MIDI × 100
  static const int P_N = RD_FRAME_N / 2;                        // 320 จุดที่ 16 kHz
  static const int P_TAU_MIN = (int)(RD_SAMPLE_RATE / 2 / RD1_F0_MAX_HZ);   // 16
  static const int P_TAU_MAX = (int)(RD_SAMPLE_RATE / 2 / RD1_F0_MIN_HZ);   // 160
  static const int P_W = P_N - P_TAU_MAX;                        // 160 จุดเทียบ
  float px_[P_N], pd_[P_TAU_MAX + 1];

  void pitch(const int16_t *pcm, Rd1Frame &out) {
    for (int i = 0; i < P_N; i++) px_[i] = (pcm[2 * i] + pcm[2 * i + 1]) * (0.5f / 32768.0f);
    float e = 0;                                                 // float: S3 มี FPU แค่ float (double ช้ามาก ~12 ms)
    for (int i = 0; i < P_N; i++) e += px_[i] * px_[i];
    out.f0_cents = 0; out.clarity = 0;
    if (e < 1e-7) return;                                        // เงียบ
    // d(τ) = Σ (x[j] − x[j+τ])² · cmndf(τ) = d(τ)·τ / Σ_{1..τ} d
    float run = 0;
    pd_[0] = 1;
    int best = -1;
    for (int t = 1; t <= P_TAU_MAX; t++) {
      float d = 0;
      for (int j = 0; j < P_W; j++) { float v = px_[j] - px_[j + t]; d += v * v; }
      run += d;
      pd_[t] = run > 0 ? (float)(d * t / run) : 1.0f;
    }
    for (int t = P_TAU_MIN; t <= P_TAU_MAX; t++) {               // ช่องแรกที่ต่ำกว่าเกณฑ์ แล้วไล่ไปจุดต่ำสุดของแอ่ง
      if (pd_[t] < RD1_F0_THRESHOLD) {
        while (t + 1 <= P_TAU_MAX && pd_[t + 1] < pd_[t]) t++;
        best = t;
        break;
      }
    }
    if (best < 0) return;                                        // ไม่มีคาบชัด = เสียงซ่า/ลม
    float tau = best;
    if (best > P_TAU_MIN && best < P_TAU_MAX) {                  // ประมาณค่าระหว่างช่อง (parabola)
      float a = pd_[best - 1], b = pd_[best], c = pd_[best + 1], den = a - 2 * b + c;
      if (fabsf(den) > 1e-9f) tau += 0.5f * (a - c) / den;
    }
    float f0 = (RD_SAMPLE_RATE / 2.0f) / tau;
    float midi = 69.0f + 12.0f * log2f(f0 / 440.0f);
    out.f0_cents = (uint16_t)constrain(lroundf(midi * 100), 1, 12700);
    out.clarity = (uint8_t)constrain(lroundf((1.0f - pd_[best]) * 255), 0, 255);
  }

  float win_[RD_FRAME_N], re_[RD1_FFT_N], im_[RD1_FFT_N];
  float cos_[RD1_FFT_N / 2], sin_[RD1_FFT_N / 2];

  void fft() {                           // radix-2 แบบวนซ้ำ (ชุดเดียวกับ mic_s3 + ตารางมุมคำนวณไว้ก่อน)
    for (int i = 1, j = 0; i < RD1_FFT_N; i++) {
      int bit = RD1_FFT_N >> 1;
      for (; j & bit; bit >>= 1) j ^= bit;
      j ^= bit;
      if (i < j) { std::swap(re_[i], re_[j]); std::swap(im_[i], im_[j]); }
    }
    for (int len = 2; len <= RD1_FFT_N; len <<= 1) {
      int step = RD1_FFT_N / len;
      for (int i = 0; i < RD1_FFT_N; i += len)
        for (int k = 0; k < len / 2; k++) {
          float wr = cos_[k * step], wi = sin_[k * step];
          int p = i + k, q = p + len / 2;
          float tr = re_[q] * wr - im_[q] * wi, ti = re_[q] * wi + im_[q] * wr;
          re_[q] = re_[p] - tr; im_[q] = im_[p] - ti;
          re_[p] += tr;         im_[p] += ti;
        }
    }
  }
};

// ---------- ทดสอบในตัว ----------
// ป้อนไซน์ความถี่ hz แอมพลิจูด amp (เต็มสเกล = 1) → คืนช่วงที่สูงสุด
inline int rd1SelfTest(Rd1 &rd1, float hz, float amp, Rd1Frame &f, uint32_t &us) {
  static int16_t s[RD_FRAME_N];
  for (int i = 0; i < RD_FRAME_N; i++) s[i] = (int16_t)lroundf(amp * 32767.0f * sinf(2 * PI * hz * i / RD_SAMPLE_RATE));
  us = rd1.compute(s, 0, 0, f);
  int best = 0;
  for (int b = 1; b < RD1_BANDS; b++) if (f.band[b] > f.band[best]) best = b;
  return best;
}

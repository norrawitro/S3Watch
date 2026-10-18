// โหมด RD1 LOG: เสียงทุก 20 ms → FFT 32 ช่วง (rd1.h) → อาร์เรย์ความดัง → ไฟล์ข้อความ 1 ไฟล์ต่อ 1 วินาที (50 แถว)
// ที่เก็บ [ผู้ใช้สั่ง 2026-10-17]:
//   PSRAM = ที่พักชั่วคราว · ครบ 20 ไฟล์ → ย้าย 15 ไฟล์ที่เก่ากว่า 5 วินาทีไป SD พร้อมกัน (เหลือ 5 ไฟล์ล่าสุดใน PSRAM)
//   กดหยุด → ย้ายที่เหลือทั้งหมดไป SD · ไฟล์บน SD: /rd1/NNNN/SSSSS.txt (NNNN = ครั้งที่กดเริ่ม)
//   ไม่มีการ์ด SD → เก็บใน PSRAM อย่างเดียว (~35 นาที เต็มแล้วหยุด · ปิดไฟ/รีเซ็ต = หาย) แบบรุ่น a44a0ec
// PSRAM เก็บเป็นเฟรม 38 ไบต์ · แปลงเป็นข้อความตอนย้ายลง SD / ส่งทาง USB
// ย้ายในงานแยก (core 0) — งานเสียงไม่ต้องรอการ์ด · วงแหวนใน PSRAM มี 40 ช่อง (ย้ายช้าได้ถึง 20 วินาทีก่อนทับ)
// รูปแบบไฟล์ (เปิดใน Excel ได้):
//   # RD1 fw <ver>, 32000 Hz, frame 640, FFT 1024, dB = (value - 240) x 0.5
//   # bands_hz,0-100,100-250,...
//   seq,flags,total,b01,...,b32,f0_hz,midi,note,clarity
//   12345,0,180,120,131,...,261.6,60.00,C4,250   ← 1 แถว = 1 ชิ้น 20 ms · ค่า 0–255 = ความดังสเกล log · f0 = ระดับเสียง
#pragma once
#include <Arduino.h>
#include <SD_MMC.h>
#include <esp_rom_crc.h>
#include "rd1.h"

const int RD1LOG_FRAMES_PER_FILE = 50;            // 50 × 20 ms = 1 วินาที [ผู้ใช้]
const uint32_t RD1LOG_MOVE_AT = 20;               // ครบ 20 ไฟล์ใน PSRAM → ย้าย [ผู้ใช้]
const uint32_t RD1LOG_KEEP = 5;                   // เหลือไว้ 5 ไฟล์ล่าสุด (ย้าย 15) [ผู้ใช้]
const uint32_t RD1LOG_RING_FILES = 40;            // ขนาดวงแหวน PSRAM (โหมดมี SD) [ข้อเสนอ]
const int RD1LOG_TXT_BYTES = 10240;               // 1 วินาที ≈ 7.5 KB ข้อความ (+ เผื่อ)
const uint32_t RD1LOG_PSRAM_SPARE = 256 * 1024;   // โหมดไม่มี SD: เหลือ PSRAM ไว้ให้ส่วนอื่น

class Rd1Log {
 public:
  uint32_t session = 0;                  // ครั้งที่กดเริ่ม (บน SD นับต่อจากโฟลเดอร์ที่มีอยู่)
  bool full = false;                     // ไม่มี SD และ PSRAM เต็ม → loop() หยุดโหมด
  bool useSd = false;
  volatile uint32_t sdFail = 0;          // ไฟล์ที่เขียนลง SD ไม่ผ่าน
  uint32_t dropped = 0;                  // ชิ้นที่ทิ้ง (SD ย้ายไม่ทัน วงแหวนเต็ม)

  void begin(bool sdOk) {
    useSd = sdOk;
    if (useSd) cap_ = RD1LOG_RING_FILES * RD1LOG_FRAMES_PER_FILE;
    else {
      uint32_t freeB = ESP.getMaxAllocPsram();
      cap_ = freeB > RD1LOG_PSRAM_SPARE ? (freeB - RD1LOG_PSRAM_SPARE) / sizeof(Rd1Frame) : 0;
      cap_ -= cap_ % RD1LOG_FRAMES_PER_FILE;
    }
    buf_ = cap_ ? (Rd1Frame *)ps_malloc(cap_ * sizeof(Rd1Frame)) : nullptr;
    if (!buf_) cap_ = 0;
    txt_ = (char *)ps_malloc(RD1LOG_TXT_BYTES);
    if (useSd) {
      q_ = xQueueCreate(4, sizeof(Job));
      xTaskCreatePinnedToCore(moverTask, "rd1move", 6144, this, 1, nullptr, 0);
    }
    Serial.printf("RD1 LOG %s: PSRAM %u KB (%u s) %s\n", useSd ? "PSRAM -> SD (move 15 at 20)" : "PSRAM only (no SD)",
                  (unsigned)(cap_ * sizeof(Rd1Frame) / 1024), (unsigned)(cap_ / RD1LOG_FRAMES_PER_FILE),
                  buf_ && txt_ ? "OK" : "FAILED");
  }

  bool start(uint32_t fw) {
    if (!buf_ || !txt_) return false;
    waitIdle();
    n_ = 0; moved_ = 0; queued_ = 0; full = false; fw_ = fw; sdFail = 0; dropped = 0;
    if (useSd) {
      SD_MMC.mkdir("/rd1");
      for (session = 1; session < 10000; session++) {
        snprintf(dir_, sizeof(dir_), "/rd1/%04u", (unsigned)session);
        if (!SD_MMC.exists(dir_)) break;
      }
      SD_MMC.mkdir(dir_);
    } else {
      session++;
      snprintf(dir_, sizeof(dir_), "PSRAM #%u", (unsigned)session);
    }
    Serial.printf("RD1 LOG start %s\n", dir_);
    return true;
  }

  // ทุก 20 ms
  void add(const Rd1Frame &f) {
    if (!useSd) {
      if (n_ < cap_) buf_[n_++] = f;
      if (n_ >= cap_) full = true;
      return;
    }
    if (n_ - moved_ * RD1LOG_FRAMES_PER_FILE >= cap_) { dropped++; return; }   // SD ย้ายไม่ทัน
    buf_[n_ % cap_] = f;
    n_++;
    // ไฟล์ครบใน PSRAM ที่ยังไม่ได้สั่งย้าย ≥ 20 → สั่งย้าย 15 ไฟล์เก่าสุด
    uint32_t done = n_ / RD1LOG_FRAMES_PER_FILE;
    if (done - queued_ >= RD1LOG_MOVE_AT) {
      Job j = {queued_, RD1LOG_MOVE_AT - RD1LOG_KEEP, 0};
      if (xQueueSend(q_, &j, 0) == pdTRUE) queued_ += j.count;
    }
  }

  // กดหยุด: ย้ายที่เหลือทั้งหมด (รวมวินาทีสุดท้ายที่ไม่ครบ) แล้วรอให้เสร็จ
  void stop() {
    if (!useSd) return;
    uint32_t total = files();
    if (total > queued_) {
      Job j = {queued_, total - queued_, n_};
      xQueueSend(q_, &j, portMAX_DELAY);
      queued_ = total;
    }
    waitIdle(15000);
  }

  void waitIdle(uint32_t timeoutMs = 5000) {
    if (!useSd) return;
    uint32_t t0 = millis();
    while ((uxQueueMessagesWaiting(q_) || busy_) && millis() - t0 < timeoutMs) delay(5);
  }

  uint32_t files() const { return (n_ + RD1LOG_FRAMES_PER_FILE - 1) / RD1LOG_FRAMES_PER_FILE; }
  uint32_t filesDone() const { return n_ / RD1LOG_FRAMES_PER_FILE; }
  uint32_t filesOnSd() const { return moved_; }
  uint32_t filesInPsram() const { return filesDone() - min((uint32_t)moved_, filesDone()); }
  uint32_t frames() const { return n_; }
  const char *dir() const { return dir_; }

  // ---------- USB ----------
  void list() {
    if (useSd) {
      File d = SD_MMC.open("/rd1");
      if (d) for (File s = d.openNextFile(); s; s = d.openNextFile()) {
        if (!s.isDirectory()) continue;
        int n = 0;
        File sd = SD_MMC.open(String("/rd1/") + s.name());
        for (File f = sd.openNextFile(); f; f = sd.openNextFile()) n++;
        Serial.printf("RD1 LOG dir %s  %d files\n", s.name(), n);
      }
      Serial.printf("RD1 LOG free %llu KB on SD\n", (SD_MMC.totalBytes() - SD_MMC.usedBytes()) / 1024);
      return;
    }
    if (session) Serial.printf("RD1 LOG dir %04u  %u files\n", (unsigned)session, (unsigned)files());
    else Serial.println("RD1 LOG: (empty)");
    Serial.printf("RD1 LOG free %u KB on PSRAM (%u s)\n", (unsigned)((cap_ - n_) * sizeof(Rd1Frame) / 1024),
                  (unsigned)((cap_ - n_) / RD1LOG_FRAMES_PER_FILE));
  }

  // 'g': ชุดล่าสุด ทีละไฟล์: "TXT <ขนาด> <crc32> <ชื่อ>\n" + ข้อความ ... "TXT END <จำนวน>" (tools/get_rd1_log.py)
  void dumpLast() {
    int nf = 0;
    if (useSd) {
      char d[16] = "";
      File root = SD_MMC.open("/rd1");
      if (root) for (File s = root.openNextFile(); s; s = root.openNextFile())
        if (s.isDirectory() && strcmp(s.name(), d) > 0) strlcpy(d, s.name(), sizeof(d));
      if (!d[0]) { Serial.println("ERR no rd1 log"); return; }
      File dir = SD_MMC.open(String("/rd1/") + d);
      for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
        int len = f.read((uint8_t *)txt_, RD1LOG_TXT_BYTES);
        char name[32]; snprintf(name, sizeof(name), "%s_%s", d, f.name());
        send(name, len); nf++;
      }
    } else {
      if (!session || !n_) { Serial.println("ERR no rd1 log"); return; }
      for (uint32_t sec = 0; sec < files(); sec++) {
        int len = render(sec, min<uint32_t>((uint32_t)n_, (sec + 1) * RD1LOG_FRAMES_PER_FILE));
        char name[32]; snprintf(name, sizeof(name), "%04u_%05u.txt", (unsigned)session, (unsigned)sec);
        send(name, len); nf++;
      }
    }
    Serial.printf("TXT END %d\n", nf);
  }

 private:
  struct Job { uint32_t first, count, endFrame; };   // endFrame 0 = ทุกไฟล์ครบ 50 ชิ้น
  Rd1Frame *buf_ = nullptr;
  char *txt_ = nullptr;                  // ใช้ร่วม: งานย้าย / ส่ง USB (ไม่ทำพร้อมกัน — USB ใช้ตอนว่าง)
  uint32_t cap_ = 0, fw_ = 0;
  volatile uint32_t n_ = 0, moved_ = 0;
  uint32_t queued_ = 0;
  QueueHandle_t q_ = nullptr;
  volatile bool busy_ = false;
  char dir_[16] = "-";

  void send(const char *name, int len) {
    uint32_t crc = esp_rom_crc32_le(0, (uint8_t *)txt_, len);
    Serial.printf("TXT %u %08X %s\n", (unsigned)len, crc, name);
    Serial.flush();
    Serial.write((uint8_t *)txt_, len);
    Serial.flush();
  }

  // f0 → ",f0_hz,midi,note,clarity" เช่น ",261.6,60.00,C4,250" · ไม่มีระดับเสียง ",0,0,,0"
  static int f0Cols(char *p, int room, const Rd1Frame &f) {
    if (!f.f0_cents) return snprintf(p, room, ",0,0,,%u", f.clarity);
    static const char *NAMES[12] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    float midi = f.f0_cents / 100.0f;
    int m = (f.f0_cents + 50) / 100;
    return snprintf(p, room, ",%.1f,%.2f,%s%d,%u", 440.0f * powf(2.0f, (midi - 69) / 12.0f), midi,
                    NAMES[m % 12], m / 12 - 1, f.clarity);
  }

  // วินาที sec → ข้อความใน txt_ (เฟรมจาก PSRAM ตำแหน่ง i % cap_)
  int render(uint32_t sec, uint32_t endFrame) {
    char *p = txt_;
    const int R = RD1LOG_TXT_BYTES;
    int n = snprintf(p, R, "# RD1 fw %u, %u Hz, frame %u, FFT %u, dB = (value - %g) x %g\n# bands_hz",
                     (unsigned)fw_, (unsigned)RD_SAMPLE_RATE, (unsigned)RD_FRAME_N, (unsigned)RD1_FFT_N,
                     RD1_CFG.log_offset, RD1_CFG.log_step_db);
    for (int b = 0; b < RD1_BANDS; b++) n += snprintf(p + n, R - n, ",%u-%u", RD1_EDGES_HZ[b], RD1_EDGES_HZ[b + 1]);
    n += snprintf(p + n, R - n, "\nseq,flags,total");
    for (int b = 0; b < RD1_BANDS; b++) n += snprintf(p + n, R - n, ",b%02d", b + 1);
    n += snprintf(p + n, R - n, ",f0_hz,midi,note,clarity,F1,F2,F3");   // f0 (รุ่น 2): 0 / ว่าง = ไม่มีระดับเสียง
    p[n++] = '\n';
    for (uint32_t i = sec * RD1LOG_FRAMES_PER_FILE; i < endFrame; i++) {
      const Rd1Frame &f = buf_[i % cap_];
      n += snprintf(p + n, R - n, "%u,%u,%u", (unsigned)f.seq, f.flags, f.total);
      for (int b = 0; b < RD1_BANDS; b++) n += snprintf(p + n, R - n, ",%u", f.band[b]);
      n += f0Cols(p + n, R - n, f);
      n += snprintf(p + n, R - n, ",%u,%u,%u", f.fmt[0], f.fmt[1], f.fmt[2]);   // ฟอร์แมนต์ (รุ่น 3) 0 = ไม่มี
      p[n++] = '\n';
    }
    return n;
  }

  static void moverTask(void *arg) {
    Rd1Log *self = (Rd1Log *)arg;
    Job j;
    for (;;) {
      if (xQueueReceive(self->q_, &j, portMAX_DELAY) != pdTRUE) continue;
      self->busy_ = true;
      uint32_t t0 = millis();
      for (uint32_t sec = j.first; sec < j.first + j.count; sec++) {
        uint32_t end = (sec + 1) * RD1LOG_FRAMES_PER_FILE;
        if (j.endFrame && end > j.endFrame) end = j.endFrame;
        int len = self->render(sec, end);
        char name[32];
        snprintf(name, sizeof(name), "%s/%05u.txt", self->dir_, (unsigned)sec);
        File f = SD_MMC.open(name, FILE_WRITE);
        bool ok = f && f.write((uint8_t *)self->txt_, len) == (size_t)len;
        if (f) f.close();
        if (!ok) self->sdFail++;
        self->moved_ = sec + 1;          // ช่องใน PSRAM ว่างให้เขียนทับได้แล้ว
      }
      Serial.printf("RD1 LOG moved %u files to SD (%u-%u) in %u ms  fail %u\n", (unsigned)j.count,
                    (unsigned)j.first, (unsigned)(j.first + j.count - 1), (unsigned)(millis() - t0), (unsigned)self->sdFail);
      self->busy_ = false;
    }
  }
};

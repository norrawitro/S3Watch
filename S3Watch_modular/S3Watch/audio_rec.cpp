#include "audio_rec.h"
#include "config.h"
#include "hw.h"
#include "settings.h"
#include "audio_codec.h"

#if ENABLE_AUDIO

#include <ESP_I2S.h>
#include <SD.h>
#include <esp_heap_caps.h>

static I2SClass i2s;

// ───────── ริงบัฟเฟอร์ SPSC (ผู้ผลิต 1 ผู้บริโภค 1 → ไม่ต้องใช้ mutex) ─────────
static uint8_t *ring = nullptr;
static size_t   ringSize = 0;
static volatile size_t ringHead = 0;     // ตัวอ่าน I2S เขียนที่นี่
static volatile size_t ringTail = 0;     // ตัวเขียน SD อ่านจากที่นี่

static volatile bool s_wanted   = false;
static volatile bool s_running  = false;
static volatile bool s_writing  = false;
static volatile bool s_fileOpen = false;
static volatile uint64_t s_bytesWritten = 0;
static char s_err[64]  = "";
static char s_file[48] = "";

static File recFile;
static uint32_t recDataBytes = 0;        // ขนาด data chunk ของไฟล์ที่เปิดอยู่

static inline size_t ringUsed() {
  size_t h = ringHead, t = ringTail;
  return (h >= t) ? (h - t) : (ringSize - t + h);
}
static inline size_t ringFree() { return ringSize - ringUsed() - 1; }

// ───────── WAV ─────────
static void wavWriteHeader(File &f, uint32_t dataLen, uint32_t rate) {
  uint32_t byteRate = rate * 2, chunk = 36 + dataLen, fmtLen = 16;
  uint16_t pcm = 1, ch = 1, align = 2, bits = 16;
  uint32_t pos = f.position();
  f.seek(0);
  f.write((const uint8_t *)"RIFF", 4);      f.write((uint8_t *)&chunk, 4);
  f.write((const uint8_t *)"WAVEfmt ", 8);  f.write((uint8_t *)&fmtLen, 4);
  f.write((uint8_t *)&pcm, 2);              f.write((uint8_t *)&ch, 2);
  f.write((uint8_t *)&rate, 4);             f.write((uint8_t *)&byteRate, 4);
  f.write((uint8_t *)&align, 2);            f.write((uint8_t *)&bits, 2);
  f.write((const uint8_t *)"data", 4);      f.write((uint8_t *)&dataLen, 4);
  f.seek(pos < 44 ? 44 : pos);
}

static bool fileOpenNew() {
  if (!hwSdOk()) { strlcpy(s_err, "No SD card", sizeof(s_err)); return false; }
  if (!SD.exists(REC_DIR)) SD.mkdir(REC_DIR);
  struct tm t; hwNowLocal(t);
  char name[48];
  strftime(name, sizeof(name), REC_DIR "/%Y%m%d_%H%M%S.wav", &t);
  recFile = SD.open(name, FILE_WRITE);
  if (!recFile) { strlcpy(s_err, "Cannot open file", sizeof(s_err)); return false; }
  recDataBytes = 0;
  wavWriteHeader(recFile, 0, settings.sampleRate);
  strlcpy(s_file, name, sizeof(s_file));
  s_fileOpen = true;
  Serial.printf("[rec] ไฟล์ใหม่: %s\n", name);
  return true;
}

static void fileCloseCurrent() {
  if (!s_fileOpen) return;
  wavWriteHeader(recFile, recDataBytes, settings.sampleRate);
  recFile.flush();
  recFile.close();
  s_fileOpen = false;
  Serial.printf("[rec] ปิดไฟล์ %s (%lu bytes)\n", s_file, (unsigned long)recDataBytes);
}

// ───────── ตัวอ่าน : I2S DMA → SRAM → PSRAM ─────────
static bool i2sStart() {
  bool stereo = (settings.micChip == 0);        // ES7210 ส่งมา 2 ช่อง
  i2s.setPins(I2S_BCLK, I2S_WS, -1 /*DOUT*/, I2S_DIN, I2S_MCLK);
  if (!i2s.begin(I2S_MODE_STD, (int)settings.sampleRate, I2S_DATA_BIT_WIDTH_16BIT,
                 stereo ? I2S_SLOT_MODE_STEREO : I2S_SLOT_MODE_MONO)) {
    strlcpy(s_err, "i2s.begin failed", sizeof(s_err));
    return false;
  }
  return true;
}

static void ringPush(const uint8_t *src, size_t n) {
  size_t space = ringFree();
  if (n > space) n = space;                     // ริงเต็ม → ทิ้งส่วนเกิน (ไม่ค้างระบบ)
  size_t h = ringHead;
  size_t first = min(n, ringSize - h);
  memcpy(ring + h, src, first);
  if (n > first) memcpy(ring, src + first, n - first);
  ringHead = (h + n) % ringSize;
}

static void readerTask(void *) {
  static uint8_t *stage = nullptr;               // บัฟเฟอร์ใน SRAM (ไม่ใช่ PSRAM)
  if (!stage) stage = (uint8_t *)heap_caps_malloc(I2S_STAGE_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

  for (;;) {
    if (s_wanted && !s_running) {
      ringHead = ringTail = 0;
      s_err[0] = 0;
      if (!codecMicStart(settings.sampleRate) || !i2sStart()) {
        s_wanted = false;
      } else {
        s_running = true;
        Serial.printf("[rec] เริ่มอัด %lu Hz, ring %u MB\n",
                      (unsigned long)settings.sampleRate, (unsigned)(ringSize / (1024 * 1024)));
      }
    }

    if (!s_wanted && s_running) {
      s_running = false;                         // ตัวเขียนจะไล่เก็บที่ค้างแล้วปิดไฟล์
      i2s.end();
      codecMicStop();
      Serial.println(F("[rec] หยุดอัด"));
    }

    if (!s_running || !stage) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

    size_t n = i2s.readBytes((char *)stage, I2S_STAGE_BYTES);
    if (n == 0) { vTaskDelay(1); continue; }

    if (settings.micChip == 0) {
      // สเตอริโอ → เก็บเฉพาะช่องซ้าย (MIC1) ให้เป็น mono
      int16_t *p = (int16_t *)stage;
      size_t frames = n / 4;
      for (size_t i = 0; i < frames; i++) p[i] = p[i * 2];
      n = frames * 2;
    }
    ringPush(stage, n);
  }
}

// ───────── ตัวเขียน : PSRAM → SD ─────────
static void drainOnce(size_t maxBytes) {
  size_t used = ringUsed();
  size_t n = min(used, maxBytes);
  if (n == 0) return;
  size_t t = ringTail;
  size_t first = min(n, ringSize - t);

  s_writing = true;
  recFile.write(ring + t, first);
  if (n > first) recFile.write(ring, n - first);
  s_writing = false;

  ringTail = (t + n) % ringSize;
  recDataBytes   += n;
  s_bytesWritten += n;
}

static void writerTask(void *) {
  for (;;) {
    if (!s_running && !s_fileOpen) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }

    if (s_running && !s_fileOpen) {
      if (!fileOpenNew()) { s_wanted = false; vTaskDelay(pdMS_TO_TICKS(200)); continue; }
    }

    size_t used = ringUsed();
    size_t flushAt = (size_t)((uint64_t)ringSize * (100 - settings.flushPct) / 100);
    size_t drainTo = ringSize / 4;               // ระบายลงมาเหลือ ~25% แล้วหยุด

    if (!s_running) {
      // กำลังปิดงาน → ไล่เขียนให้หมดแล้วปิดไฟล์
      while (s_fileOpen && ringUsed() > 0) drainOnce(SD_CHUNK_BYTES);
      fileCloseCurrent();
      continue;
    }

    if (used >= flushAt) {
      uint32_t t0 = millis();
      while (s_running && s_fileOpen && ringUsed() > drainTo) {
        drainOnce(SD_CHUNK_BYTES);
        if (recDataBytes >= REC_FILE_MAX_BYTES) {   // ตัดไฟล์เมื่อใหญ่เกิน
          fileCloseCurrent();
          if (!fileOpenNew()) { s_wanted = false; break; }
        }
        vTaskDelay(1);                              // ให้ core อื่นได้หายใจ
      }
      if (!s_fileOpen) continue;
      wavWriteHeader(recFile, recDataBytes, settings.sampleRate);
      recFile.flush();
      Serial.printf("[rec] flush ลง SD %lu ms, ริงเหลือ %u%%\n",
                    (unsigned long)(millis() - t0), (unsigned)audioRingUsedPct());
    } else {
      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }
}

// ───────── API ─────────
bool audioInit() {
  size_t want = (size_t)settings.ringMB * 1024UL * 1024UL;
  size_t avail = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
  if (avail < 512 * 1024) { strlcpy(s_err, "No PSRAM", sizeof(s_err)); return false; }
  if (want > avail - 256 * 1024) want = avail - 256 * 1024;   // เผื่อไว้ให้ระบบอื่น

  ring = (uint8_t *)heap_caps_malloc(want, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!ring) { strlcpy(s_err, "PSRAM alloc failed", sizeof(s_err)); return false; }
  ringSize = want;
  Serial.printf("[rec] ริงบัฟเฟอร์ PSRAM %u KB (~%lu วินาที)\n",
                (unsigned)(ringSize / 1024),
                (unsigned long)(ringSize / (settings.sampleRate * 2)));

  xTaskCreatePinnedToCore(readerTask, "audioRd", 4096, nullptr, 6, nullptr, 0);
  xTaskCreatePinnedToCore(writerTask, "audioWr", 6144, nullptr, 3, nullptr, 1);
  return true;
}

bool audioStart() {
  if (!ring) { strlcpy(s_err, "Buffer not ready", sizeof(s_err)); return false; }
  if (!hwSdOk()) { strlcpy(s_err, "No SD card", sizeof(s_err)); return false; }
  s_wanted = true;
  uint32_t t0 = millis();
  while (!s_running && millis() - t0 < 3000) delay(20);
  return s_running;
}

void audioStop() {
  s_wanted = false;
  uint32_t t0 = millis();
  while ((s_running || s_fileOpen) && millis() - t0 < 15000) delay(50);
}

bool audioIsRecording()  { return s_running; }
bool audioIsBusy()       { return s_writing; }
uint64_t audioBytesWritten() { return s_bytesWritten; }
const char *audioLastError()  { return s_err; }
const char *audioCurrentFile(){ return s_file; }

uint8_t audioRingUsedPct() {
  if (!ringSize) return 0;
  return (uint8_t)((uint64_t)ringUsed() * 100 / ringSize);
}

#else   // ENABLE_AUDIO == 0

bool audioInit() { return false; }
bool audioStart() { return false; }
void audioStop() {}
bool audioIsRecording() { return false; }
bool audioIsBusy() { return false; }
uint8_t audioRingUsedPct() { return 0; }
uint64_t audioBytesWritten() { return 0; }
const char *audioLastError() { return "Audio disabled at build"; }
const char *audioCurrentFile() { return ""; }

#endif

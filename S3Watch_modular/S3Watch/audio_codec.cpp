#include "audio_codec.h"
#include "config.h"
#include "hw.h"
#include "settings.h"

// ───────────────────────── ES7210 (ADC 4 ช่อง) ─────────────────────────
static void es7210Start(uint32_t rate) {
  const uint8_t a = ADDR_ES7210;
  i2cWrite8(a, 0x00, 0xFF); delay(10);      // reset
  i2cWrite8(a, 0x00, 0x41);
  i2cWrite8(a, 0x01, 0x3F);
  i2cWrite8(a, 0x09, 0x30); i2cWrite8(a, 0x0A, 0x30);
  i2cWrite8(a, 0x23, 0x2A); i2cWrite8(a, 0x22, 0x0A);
  i2cWrite8(a, 0x20, 0x0A); i2cWrite8(a, 0x21, 0x2A);
  i2cWrite8(a, 0x08, 0x00);                 // slave mode
  i2cWrite8(a, 0x40, 0x43); i2cWrite8(a, 0x41, 0x70); i2cWrite8(a, 0x42, 0x70);
  i2cWrite8(a, 0x07, 0x20);
  // ตัวหารนาฬิกา: MCLK = 256 × fs
  uint8_t clkdiv = (rate == 8000) ? 0x82 : (rate == 32000 ? 0x80 : 0x81);
  i2cWrite8(a, 0x02, clkdiv);
  i2cWrite8(a, 0x03, 0x00); i2cWrite8(a, 0x04, 0x01); i2cWrite8(a, 0x05, 0x00);
  i2cWrite8(a, 0x11, 0x60); i2cWrite8(a, 0x12, 0x00);   // I2S 16-bit, ไม่ใช้ TDM
  i2cWrite8(a, 0x43, 0x1A); i2cWrite8(a, 0x44, 0x1A);   // MIC1/2 gain ~30 dB
  i2cWrite8(a, 0x45, 0x00); i2cWrite8(a, 0x46, 0x00);
  i2cWrite8(a, 0x01, 0x20); i2cWrite8(a, 0x06, 0x00); i2cWrite8(a, 0x40, 0x42);
  i2cWrite8(a, 0x47, 0x08); i2cWrite8(a, 0x48, 0x08);
  i2cWrite8(a, 0x49, 0xFF); i2cWrite8(a, 0x4A, 0xFF);
  i2cWrite8(a, 0x4B, 0x00); i2cWrite8(a, 0x4C, 0xFF);   // เปิด MIC1/2 ปิด MIC3/4
}

static void es7210Stop() {
  const uint8_t a = ADDR_ES7210;
  i2cWrite8(a, 0x4B, 0xFF); i2cWrite8(a, 0x4C, 0xFF);
  i2cWrite8(a, 0x47, 0xFF); i2cWrite8(a, 0x48, 0xFF);
  i2cWrite8(a, 0x49, 0xFF); i2cWrite8(a, 0x4A, 0xFF);
  i2cWrite8(a, 0x40, 0x80); i2cWrite8(a, 0x01, 0x7F); i2cWrite8(a, 0x06, 0x07);
}

// ───────────────────────── ES8311 (codec 1 ช่อง) ─────────────────────────
static void es8311MicStart(uint32_t rate) {
  const uint8_t a = ADDR_ES8311;
  i2cWrite8(a, 0x00, 0x1F); delay(20);      // reset
  i2cWrite8(a, 0x00, 0x00);
  i2cWrite8(a, 0x01, 0x30);                 // เปิด MCLK, เลือก clock source
  i2cWrite8(a, 0x02, 0x10);                 // ตัวหาร (MCLK = 256 × fs)
  i2cWrite8(a, 0x03, 0x10);
  i2cWrite8(a, 0x16, 0x24);
  i2cWrite8(a, 0x04, 0x10); i2cWrite8(a, 0x05, 0x00);
  i2cWrite8(a, 0x06, 0x03); i2cWrite8(a, 0x07, 0x00); i2cWrite8(a, 0x08, 0xFF);
  i2cWrite8(a, 0x09, 0x0C);                 // SDP In  : I2S 16-bit
  i2cWrite8(a, 0x0A, 0x0C);                 // SDP Out : I2S 16-bit
  i2cWrite8(a, 0x0B, 0x00); i2cWrite8(a, 0x0C, 0x00);
  i2cWrite8(a, 0x10, 0x03); i2cWrite8(a, 0x11, 0x7B);
  i2cWrite8(a, 0x00, 0x80);                 // slave mode
  i2cWrite8(a, 0x0D, 0x01);                 // power up analog
  i2cWrite8(a, 0x0E, 0x02);                 // power up ADC
  i2cWrite8(a, 0x12, 0x00);
  i2cWrite8(a, 0x13, 0x00);
  i2cWrite8(a, 0x1C, 0x6A);
  i2cWrite8(a, 0x37, 0x08);
  i2cWrite8(a, 0x17, 0xC8);                 // ADC volume
  i2cWrite8(a, 0x16, 0x03);                 // PGA gain (0..7 ยิ่งมากยิ่งดัง)
  (void)rate;
}

static void es8311MicStop() {
  const uint8_t a = ADDR_ES8311;
  i2cWrite8(a, 0x0E, 0xFF);                 // ปิด ADC
  i2cWrite8(a, 0x0D, 0xFA);                 // ปิดภาค analog
  i2cWrite8(a, 0x00, 0x00);
  i2cWrite8(a, 0x01, 0x30);
  i2cWrite8(a, 0x01, 0x00);                 // ปิด clock → สลีป
}

// ───────────────────────── API ─────────────────────────
bool codecDetect(uint8_t &foundChip) {
  if (i2cPresent(ADDR_ES7210)) { foundChip = 0; return true; }
  if (i2cPresent(ADDR_ES8311)) { foundChip = 1; return true; }
  return false;
}

bool codecMicStart(uint32_t sampleRate) {
  uint8_t chip = settings.micChip;
  uint8_t addr = chip ? ADDR_ES8311 : ADDR_ES7210;
  if (!i2cPresent(addr)) {
    // ชิปที่เลือกไว้ไม่ตอบ → ลองตรวจอัตโนมัติแล้วใช้ตัวที่เจอ
    uint8_t found;
    if (!codecDetect(found)) {
      Serial.println(F("[codec] ไม่พบชิปไมโครโฟนบนบัส I2C"));
      return false;
    }
    Serial.printf("[codec] เปลี่ยนไปใช้ชิปที่ตรวจพบ: %s\n", found ? "ES8311" : "ES7210");
    chip = found;
    settings.micChip = found;
  }
  if (chip) es8311MicStart(sampleRate);
  else      es7210Start(sampleRate);
  delay(10);
  return true;
}

void codecMicStop() {
  if (i2cPresent(ADDR_ES7210)) es7210Stop();
  if (i2cPresent(ADDR_ES8311)) es8311MicStop();
}

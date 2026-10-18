// การ์ด SD: ฟอร์แมต (USB F แล้วพิมพ์ YES) + ทดสอบเขียน/อ่าน 1 MB (USB T) — ขั้น 1.4 ในแผน N.tag
// ฟอร์แมตเป็น FAT32 ทั้งการ์ด (ข้อมูลเดิมหายหมด) · ใช้ ESP-IDF ตรง เพราะ SD_MMC ของ Arduino ไม่มีคำสั่งฟอร์แมต
#pragma once
#include <Arduino.h>
#include <SD_MMC.h>
#include <esp_vfs_fat.h>
#include <sdmmc_cmd.h>
#include <driver/sdmmc_host.h>

// ฟอร์แมต แล้วเปิดการ์ดใหม่ด้วย SD_MMC · คืน true ถ้าสำเร็จ
// นาฬิกา: SD_MMC 1 บิต (CLK/CMD/D0 ตามตัวอย่างผู้ผลิต 07_LVGL_SD_Test)
inline bool sdFormat(int clk, int cmd, int d0) {
  SD_MMC.end();
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
  slot.width = 1;
  slot.clk = (gpio_num_t)clk; slot.cmd = (gpio_num_t)cmd; slot.d0 = (gpio_num_t)d0;
  slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
  esp_vfs_fat_sdmmc_mount_config_t mc = {};
  mc.format_if_mount_failed = true;
  mc.max_files = 5;
  mc.allocation_unit_size = 32 * 1024;   // คลัสเตอร์ 32 KB (FAT32 การ์ดใหญ่)
  sdmmc_card_t *card = nullptr;
  esp_err_t e = esp_vfs_fat_sdmmc_mount("/sdfmt", &host, &slot, &mc, &card);
  if (e == ESP_OK) {
    Serial.printf("SD format: card %llu MB, formatting FAT32 (may take a while)...\n",
                  (uint64_t)card->csd.capacity * card->csd.sector_size >> 20);
    uint32_t t0 = millis();
    e = esp_vfs_fat_sdcard_format("/sdfmt", card);
    Serial.printf("SD format: %s in %u ms\n", e == ESP_OK ? "OK" : esp_err_to_name(e), (unsigned)(millis() - t0));
    esp_vfs_fat_sdcard_unmount("/sdfmt", card);
  } else Serial.printf("SD format: mount failed %s\n", esp_err_to_name(e));
  SD_MMC.setPins(clk, cmd, d0);
  bool ok = SD_MMC.begin("/sdcard", true);
  Serial.printf("SD remount: %s\n", ok ? "OK" : "FAILED");
  return e == ESP_OK && ok;
}

// เขียน 1 MB รูปแบบรู้ค่า → อ่านกลับ เทียบทุกไบต์ + วัดความเร็ว
inline bool sdTest() {
  if (SD_MMC.cardType() == CARD_NONE) { Serial.println("SD test: no card"); return false; }
  Serial.printf("SD test: type %d  size %llu MB  total %llu MB  used %llu MB\n", SD_MMC.cardType(),
                SD_MMC.cardSize() >> 20, SD_MMC.totalBytes() >> 20, SD_MMC.usedBytes() >> 20);
  const size_t BLK = 4096, N = 256;     // 1 MB
  uint8_t *b = (uint8_t *)malloc(BLK);
  if (!b) return false;
  uint32_t t0 = millis();
  File f = SD_MMC.open("/sdtest.bin", FILE_WRITE);
  bool ok = f;
  for (size_t i = 0; ok && i < N; i++) {
    for (size_t k = 0; k < BLK; k++) b[k] = (uint8_t)(i * 31 + k * 7);
    ok = f.write(b, BLK) == BLK;
  }
  if (f) f.close();
  uint32_t tw = millis() - t0;
  t0 = millis();
  f = SD_MMC.open("/sdtest.bin", FILE_READ);
  size_t bad = 0;
  for (size_t i = 0; ok && f && i < N; i++) {
    if (f.read(b, BLK) != BLK) { ok = false; break; }
    for (size_t k = 0; k < BLK; k++) if (b[k] != (uint8_t)(i * 31 + k * 7)) bad++;
  }
  if (f) f.close();
  uint32_t tr = millis() - t0;
  SD_MMC.remove("/sdtest.bin");
  free(b);
  ok = ok && bad == 0;
  Serial.printf("SD test 1 MB: write %u ms (%.0f KB/s)  read %u ms (%.0f KB/s)  bad bytes %u  -> %s\n",
                (unsigned)tw, 1024000.0f / max<uint32_t>(tw, 1), (unsigned)tr, 1024000.0f / max<uint32_t>(tr, 1), (unsigned)bad,
                ok ? "PASS" : "FAIL");
  return ok;
}

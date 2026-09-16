// ============================================================================
//  config.h — S3 Watch : พิน / ค่าคงที่ / สวิตช์เปิดปิดฟีเจอร์
//  บอร์ด: Waveshare ESP32-S3-Touch-AMOLED-2.06 (CO5300 410x502, QMI8658,
//         PCF85063, AXP2101, microSD, I2S mic)
// ============================================================================
#pragma once

#define FW_VERSION "0.2.0"

// ─────────────── สวิตช์คอมไพล์ ───────────────
// UI ของเมนู Settings : 1 = LVGL (ตามสเปก, ต้องติดตั้ง lvgl + lv_conf.h)
//                       0 = วาดเองด้วย Arduino_GFX (ไม่ต้องพึ่ง lvgl)
// ตรรกะเมนู/เจสเจอร์เหมือนกันทั้งสองแบบ ต่างกันแค่ตัววาด
#ifndef USE_LVGL
#define USE_LVGL 1
#endif

#ifndef ENABLE_AUDIO
#define ENABLE_AUDIO 1
#endif

#ifndef ENABLE_OTA
#define ENABLE_OTA 1
#endif

// 1 = ไม่หลับเลย (ดีบักผ่าน USB Serial ได้ เพราะ CDC จะหลุดเมื่อหลับ)
#ifndef DEBUG_NO_SLEEP
#define DEBUG_NO_SLEEP 0
#endif

// ─────────────── จอ AMOLED (QSPI) ───────────────
#define LCD_D0        4
#define LCD_D1        5
#define LCD_D2        6
#define LCD_D3        7
#define LCD_SCLK      11
#define LCD_CS        12
#define LCD_RST       8
#define LCD_W         410
#define LCD_H         502
#define LCD_COL_OFF   22     // ถ้าภาพเลื่อนซ้าย/ขวา ให้ปรับค่านี้

// ─────────────── I2C (RTC / IMU / Touch / PMU / Codec) ───────────────
#define I2C_SDA       15
#define I2C_SCL       14
#define I2C_FREQ      400000

#define ADDR_TOUCH    0x38   // FT3168 / FT6336 compatible
#define ADDR_PMU      0x34   // AXP2101 (ถ้าไม่มีจะข้ามอัตโนมัติ)
#define ADDR_ES7210   0x40
#define ADDR_ES8311   0x18

// ─────────────── ขาอินพุต ───────────────
#define PIN_TP_INT    38     // ⚠ ไม่ใช่ขา RTC → ปลุกจาก deep sleep ไม่ได้
#define PIN_IMU_INT   21     // ขา RTC → ปลุกได้ทั้ง light + deep sleep
#define PIN_BOOT      0      // ขา RTC → ปุ่มเดียวของระบบ (กดค้าง 2 วิ = Settings)

// ─────────────── microSD (SPI) ───────────────
#define SD_MOSI       1
#define SD_SCK        2
#define SD_MISO       3
#define SD_CS         17
#define SD_FREQ_HZ    20000000UL   // อย่าสูงกว่านี้ (ตามข้อควรระวังในเอกสาร)

// ─────────────── I2S ───────────────
#define I2S_MCLK      16
#define I2S_BCLK      41
#define I2S_WS        45
#define I2S_DOUT      40     // -> ES8311 (ลำโพง, ยังไม่ใช้)
#define I2S_DIN       42     // <- ไมโครโฟน
#define PIN_PA        46     // แอมป์ลำโพง (LOW = ปิด)

// ─────────────── ค่าเริ่มต้นของ Settings ───────────────
#define DEF_FACE_MODE        0        // 0 = อนาล็อก, 1 = ดิจิทัล
#define DEF_BRIGHT_HIGH_PCT  90
#define DEF_BRIGHT_DIM_PCT   25
#define DEF_BRIGHT_HOLD_MS   10000    // สว่าง 10 วินาทีแล้วหรี่
#define DEF_WRIST_WAKE       true
#define DEF_FACE_ANGLE_DEG   15       // ระนาบ ±15° → z > cos(15°) = 0.966
#define DEF_FACE_HOLD_MS     200
#define DEF_WOM_MG           150
#define DEF_MIC_CHIP         0        // 0 = ES7210, 1 = ES8311
#define DEF_SAMPLE_RATE      16000
#define DEF_FLUSH_PCT        20       // flush ลง SD เมื่อพื้นที่ว่างในริงเหลือ < 20%
#define DEF_RING_MB          4        // ริงบัฟเฟอร์ใน PSRAM (MB)
#define DEF_CPU_MHZ          80
#define DEF_TZ_MIN           420      // ไทย UTC+7

// ─────────────── ค่าคงที่ระบบ ───────────────
#define LONG_PRESS_MS        2000     // กดปุ่ม BOOT ค้าง = เข้า/ออก Settings
#define I2S_STAGE_BYTES      8192     // บัฟเฟอร์ขั้นกลางใน SRAM (DMA → RAM)
#define SD_CHUNK_BYTES       32768    // ขนาดก้อนที่เขียนลง SD ต่อครั้ง
#define REC_FILE_MAX_BYTES   (1500UL * 1024UL * 1024UL)  // ตัดไฟล์ที่ ~1.5 GB
#define SETTINGS_PATH        "/settings.cfg"
#define REC_DIR              "/rec"

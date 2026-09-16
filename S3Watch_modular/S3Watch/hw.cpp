#include "hw.h"
#include "config.h"
#include "settings.h"
#include <SPI.h>
#include <SD.h>
#include <sys/time.h>

// ⚠ 1 = สั่งเปิดรางไฟของ AXP2101 เอง (แตะเฉพาะบิต enable ไม่แตะค่าแรงดัน)
//    ปกติปล่อยเป็น 0 ไว้ก่อน — เปิดเฉพาะกรณีจอไม่ติดเลย
#ifndef PMU_FORCE_RAILS
#define PMU_FORCE_RAILS 0
#endif

Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_D0, LCD_D1, LCD_D2, LCD_D3);
// หมายเหตุ: ลำดับพารามิเตอร์ของ Arduino_CO5300 อาจต่างกันตามเวอร์ชันของ Arduino_GFX
// (bus, rst, rotation, ips, w, h, col_offset1, row_offset1, col_offset2, row_offset2)
Arduino_CO5300 *gfx = new Arduino_CO5300(bus, LCD_RST, 0 /*rotation*/, false /*ips*/,
                                         LCD_W, LCD_H, LCD_COL_OFF, 0, 0, 0);
SensorPCF85063 rtc;
SensorQMI8658  imu;

static bool s_rtcOk = false, s_imuOk = false, s_sdOk = false, s_pmuOk = false;
static uint8_t s_brightPct = 0;
static SPIClass sdSpi(HSPI);

bool hwRtcOk() { return s_rtcOk; }
bool hwImuOk() { return s_imuOk; }
bool hwSdOk()  { return s_sdOk;  }
bool hwPmuOk() { return s_pmuOk; }

// ─────────────── I2C helper ───────────────
bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)len) != (int)len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

// ─────────────── พิน ───────────────
void hwInitPins() {
  pinMode(PIN_BOOT,    INPUT_PULLUP);
  pinMode(PIN_TP_INT,  INPUT_PULLUP);
  pinMode(PIN_IMU_INT, INPUT);
  pinMode(PIN_PA,      OUTPUT);
  digitalWrite(PIN_PA, LOW);        // ปิดแอมป์ลำโพงไว้เสมอ (ประหยัดไฟ)
}

void hwInitI2C() {
  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
  delay(5);
}

// ─────────────── AXP2101 ───────────────
void hwInitPmu() {
  s_pmuOk = i2cPresent(ADDR_PMU);
  if (!s_pmuOk) { Serial.println(F("[pmu] ไม่พบ AXP2101 — ข้าม")); return; }

  uint8_t ldoEn = 0;
  i2cRead(ADDR_PMU, 0x90, &ldoEn, 1);
  Serial.printf("[pmu] AXP2101 พบแล้ว, LDO enable (0x90) = 0x%02X\n", ldoEn);

#if PMU_FORCE_RAILS
  // เปิด ALDO1..4 + BLDO1..2 โดย "ไม่แตะค่าแรงดัน" ที่บอร์ดตั้งไว้แล้ว
  i2cWrite8(ADDR_PMU, 0x90, ldoEn | 0x3F);
  delay(20);
  Serial.println(F("[pmu] บังคับเปิดราง LDO แล้ว"));
#endif
}

// ─────────────── จอ ───────────────
bool hwInitDisplay(bool keepImage) {
  if (!gfx->begin()) {
    Serial.println(F("[gfx] begin() ล้มเหลว"));
    return false;
  }
  if (!keepImage) gfx->fillScreen(RGB565_BLACK);
  hwSetBrightnessPct(settings.brightDimPct);
  return true;
}

void hwSetBrightnessPct(uint8_t pct) {
  if (pct > 100) pct = 100;
  s_brightPct = pct;
  gfx->setBrightness((uint8_t)((uint16_t)pct * 255 / 100));
}

uint8_t hwBrightnessPct() { return s_brightPct; }

// ─────────────── RTC (PCF85063) ───────────────
bool hwInitRtc() {
  s_rtcOk = rtc.begin(Wire, I2C_SDA, I2C_SCL);
  if (!s_rtcOk) { Serial.println(F("[rtc] ไม่พบ PCF85063")); return false; }
  hwTimeFromRtc();

  // ถ้า RTC ยังไม่เคยถูกตั้ง → ใช้เวลาคอมไพล์ตั้งให้ครั้งแรก
  struct tm t; hwNowLocal(t);
  if (t.tm_year + 1900 < 2025) {
    char mon[4] = {0}; int d = 1, y = 2025, hh = 0, mm = 0, ss = 0;
    sscanf(__DATE__, "%3s %d %d", mon, &d, &y);
    sscanf(__TIME__, "%d:%d:%d", &hh, &mm, &ss);
    static const char *MONS = "JanFebMarAprMayJunJulAugSepOctNovDec";
    const char *p = strstr(MONS, mon);
    int m = p ? (int)((p - MONS) / 3) + 1 : 1;
    rtc.setDateTime(y, m, d, hh, mm, ss);
    hwTimeFromRtc();
    Serial.printf("[rtc] ตั้งเวลาจาก compile time: %04d-%02d-%02d %02d:%02d:%02d\n", y, m, d, hh, mm, ss);
  }
  return true;
}

void hwTimeFromRtc() {
  if (!s_rtcOk) return;
  // หมายเหตุ: ถ้า SensorLib เวอร์ชันที่ใช้เป็นแบบ getter ให้เปลี่ยนเป็น
  // d.getYear(), d.getMonth(), ... แทนการอ่านฟิลด์ตรง ๆ
  RTC_DateTime d = rtc.getDateTime();
  struct tm t = {};
  t.tm_year = d.year - 1900;
  t.tm_mon  = d.month - 1;
  t.tm_mday = d.day;
  t.tm_hour = d.hour;
  t.tm_min  = d.minute;
  t.tm_sec  = d.second;
  t.tm_isdst = 0;
  time_t sec = mktime(&t);
  if (sec < 0) return;
  struct timeval tv = { .tv_sec = sec, .tv_usec = 0 };
  settimeofday(&tv, nullptr);
}

void hwTimeToRtc() {
  if (!s_rtcOk) return;
  struct tm t; hwNowLocal(t);
  rtc.setDateTime(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
}

void hwNowLocal(struct tm &out) {
  // เก็บ system time เป็น "เวลาท้องถิ่น" ตรง ๆ (TZ = UTC) เพื่อไม่ให้ offset ซ้อนกัน
  time_t now = time(nullptr);
  localtime_r(&now, &out);
}

// ─────────────── IMU (QMI8658) ───────────────
bool hwInitImu() {
  s_imuOk = imu.begin(Wire, QMI8658_L_SLAVE_ADDRESS, I2C_SDA, I2C_SCL);
  if (!s_imuOk) { Serial.println(F("[imu] ไม่พบ QMI8658 — ปิดฟีเจอร์หงายข้อมือ")); return false; }
  imu.configAccelerometer(SensorQMI8658::ACC_RANGE_2G,
                          SensorQMI8658::ACC_ODR_LOWPOWER_21Hz,
                          SensorQMI8658::LPF_MODE_0);
  imu.enableAccelerometer();
  Serial.println(F("[imu] QMI8658 พร้อม"));
  return true;
}

// ─────────────── microSD ───────────────
bool hwInitSd() {
  sdSpi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  s_sdOk = SD.begin(SD_CS, sdSpi, SD_FREQ_HZ);
  if (!s_sdOk) { Serial.println(F("[sd] ไม่พบการ์ด — ใช้ค่าจาก NVS และอัดเสียงไม่ได้")); return false; }
  if (!SD.exists(REC_DIR)) SD.mkdir(REC_DIR);
  Serial.printf("[sd] พร้อม, ขนาด %llu MB\n", SD.cardSize() / (1024ULL * 1024ULL));
  return true;
}

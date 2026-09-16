#include "settings.h"
#include "config.h"
#include "hw.h"
#include <SD.h>
#include <Preferences.h>
#include <math.h>

Settings settings;
static Preferences prefs;

// ─────────────────────────────────────────────────────────────
void settingsDefaults() {
  memset(&settings, 0, sizeof(settings));
  settings.faceMode       = DEF_FACE_MODE;
  settings.brightHighPct  = DEF_BRIGHT_HIGH_PCT;
  settings.brightDimPct   = DEF_BRIGHT_DIM_PCT;
  settings.brightHoldMs   = DEF_BRIGHT_HOLD_MS;
  settings.wristWake      = DEF_WRIST_WAKE;
  settings.faceAngleDeg   = DEF_FACE_ANGLE_DEG;
  settings.faceHoldMs     = DEF_FACE_HOLD_MS;
  settings.womThresholdMg = DEF_WOM_MG;
  settings.micChip        = DEF_MIC_CHIP;
  settings.sampleRate     = DEF_SAMPLE_RATE;
  settings.flushPct       = DEF_FLUSH_PCT;
  settings.ringMB         = DEF_RING_MB;
  settings.wifiEnabled    = false;
  settings.wifiSsid[0]    = 0;
  settings.wifiPass[0]    = 0;
  settings.otaUrl[0]      = 0;
  settings.tzMinutes      = DEF_TZ_MIN;
  settings.cpuMhz         = DEF_CPU_MHZ;
}

void settingsClampAll() {
  settings.faceMode      = settings.faceMode ? 1 : 0;
  settings.brightHighPct = constrain(settings.brightHighPct, (uint8_t)5, (uint8_t)100);
  settings.brightDimPct  = constrain(settings.brightDimPct,  (uint8_t)0, (uint8_t)100);
  if (settings.brightDimPct > settings.brightHighPct) settings.brightDimPct = settings.brightHighPct;
  settings.brightHoldMs  = constrain(settings.brightHoldMs, (uint16_t)1000, (uint16_t)60000);
  settings.faceAngleDeg  = constrain(settings.faceAngleDeg, (uint8_t)5, (uint8_t)60);
  settings.faceHoldMs    = constrain(settings.faceHoldMs, (uint16_t)0, (uint16_t)2000);
  settings.womThresholdMg= constrain(settings.womThresholdMg, (uint16_t)20, (uint16_t)1000);
  settings.micChip       = settings.micChip ? 1 : 0;
  if (settings.sampleRate != 8000 && settings.sampleRate != 16000 && settings.sampleRate != 32000)
    settings.sampleRate = DEF_SAMPLE_RATE;
  settings.flushPct      = constrain(settings.flushPct, (uint8_t)5, (uint8_t)50);
  settings.ringMB        = constrain(settings.ringMB, (uint8_t)1, (uint8_t)6);
  settings.tzMinutes     = constrain(settings.tzMinutes, (int16_t)-720, (int16_t)840);
  if (settings.cpuMhz != 80 && settings.cpuMhz != 160 && settings.cpuMhz != 240)
    settings.cpuMhz = DEF_CPU_MHZ;
}

float settingsFaceUpZ() {
  return cosf((float)settings.faceAngleDeg * (float)DEG_TO_RAD);
}

// ─────────────── ตัวช่วยอ่านไฟล์ key=value ───────────────
static void applyKV(const String &key, const String &val) {
  if      (key == "faceMode")     settings.faceMode       = val.toInt();
  else if (key == "brightHigh")   settings.brightHighPct  = val.toInt();
  else if (key == "brightDim")    settings.brightDimPct   = val.toInt();
  else if (key == "brightHoldMs") settings.brightHoldMs   = val.toInt();
  else if (key == "wristWake")    settings.wristWake      = (val.toInt() != 0);
  else if (key == "faceAngleDeg") settings.faceAngleDeg   = val.toInt();
  else if (key == "faceHoldMs")   settings.faceHoldMs     = val.toInt();
  else if (key == "womMg")        settings.womThresholdMg = val.toInt();
  else if (key == "micChip")      settings.micChip        = val.toInt();
  else if (key == "sampleRate")   settings.sampleRate     = val.toInt();
  else if (key == "flushPct")     settings.flushPct       = val.toInt();
  else if (key == "ringMB")       settings.ringMB         = val.toInt();
  else if (key == "wifiEnabled")  settings.wifiEnabled    = (val.toInt() != 0);
  else if (key == "wifiSsid")     strlcpy(settings.wifiSsid, val.c_str(), sizeof(settings.wifiSsid));
  else if (key == "wifiPass")     strlcpy(settings.wifiPass, val.c_str(), sizeof(settings.wifiPass));
  else if (key == "otaUrl")       strlcpy(settings.otaUrl,  val.c_str(), sizeof(settings.otaUrl));
  else if (key == "tzMinutes")    settings.tzMinutes      = val.toInt();
  else if (key == "cpuMhz")       settings.cpuMhz         = val.toInt();
}

static bool loadFromSd() {
  if (!hwSdOk()) return false;
  File f = SD.open(SETTINGS_PATH, FILE_READ);
  if (!f) return false;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0 || line[0] == '#') continue;
    int eq = line.indexOf('=');
    if (eq <= 0) continue;
    String k = line.substring(0, eq);      k.trim();
    String v = line.substring(eq + 1);     v.trim();
    applyKV(k, v);
  }
  f.close();
  return true;
}

static bool saveToSd() {
  if (!hwSdOk()) return false;
  File f = SD.open(SETTINGS_PATH, FILE_WRITE);   // FILE_WRITE = ตัดไฟล์เดิมทิ้ง
  if (!f) return false;
  f.printf("# S3 Watch settings (fw %s) — แก้ไฟล์นี้ด้วย Notepad ได้เลย\n", FW_VERSION);
  f.printf("faceMode=%u\n",     settings.faceMode);
  f.printf("brightHigh=%u\n",   settings.brightHighPct);
  f.printf("brightDim=%u\n",    settings.brightDimPct);
  f.printf("brightHoldMs=%u\n", settings.brightHoldMs);
  f.printf("wristWake=%u\n",    settings.wristWake ? 1 : 0);
  f.printf("faceAngleDeg=%u\n", settings.faceAngleDeg);
  f.printf("faceHoldMs=%u\n",   settings.faceHoldMs);
  f.printf("womMg=%u\n",        settings.womThresholdMg);
  f.printf("micChip=%u\n",      settings.micChip);
  f.printf("sampleRate=%lu\n",  (unsigned long)settings.sampleRate);
  f.printf("flushPct=%u\n",     settings.flushPct);
  f.printf("ringMB=%u\n",       settings.ringMB);
  f.printf("wifiEnabled=%u\n",  settings.wifiEnabled ? 1 : 0);
  f.printf("wifiSsid=%s\n",     settings.wifiSsid);
  f.printf("wifiPass=%s\n",     settings.wifiPass);
  f.printf("otaUrl=%s\n",       settings.otaUrl);
  f.printf("tzMinutes=%d\n",    settings.tzMinutes);
  f.printf("cpuMhz=%u\n",       settings.cpuMhz);
  f.close();
  return true;
}

// ─────────────── NVS (สำรองตอนไม่มี SD) ───────────────
static bool loadFromNvs() {
  if (!prefs.begin("s3watch", true)) return false;
  size_t n = prefs.getBytesLength("cfg");
  bool ok = false;
  if (n == sizeof(Settings)) {
    prefs.getBytes("cfg", &settings, sizeof(Settings));
    ok = true;
  }
  prefs.end();
  return ok;
}

static bool saveToNvs() {
  if (!prefs.begin("s3watch", false)) return false;
  prefs.putBytes("cfg", &settings, sizeof(Settings));
  prefs.end();
  return true;
}

// ─────────────── API ───────────────
bool settingsLoad() {
  settingsDefaults();
  bool fromSd = loadFromSd();
  if (!fromSd) {
    if (loadFromNvs()) {
      Serial.println(F("[settings] โหลดจาก NVS (ไม่พบ SD/ไฟล์ตั้งค่า)"));
    } else {
      Serial.println(F("[settings] ใช้ค่าเริ่มต้น"));
      settingsClampAll();
      saveToNvs();
      saveToSd();          // สร้างไฟล์ตัวอย่างไว้บน SD ให้ผู้ใช้แก้ได้
      return false;
    }
  } else {
    Serial.println(F("[settings] โหลดจาก SD"));
  }
  settingsClampAll();
  if (fromSd) saveToNvs();  // มิเรอร์ค่าจาก SD ลง NVS ทุกครั้งที่บูต
  return true;
}

bool settingsSave() {
  settingsClampAll();
  bool a = saveToNvs();
  bool b = saveToSd();
  return a || b;
}

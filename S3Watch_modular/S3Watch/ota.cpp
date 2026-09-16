#include "ota.h"
#include "config.h"
#include "settings.h"
#include "hw.h"

#if ENABLE_OTA

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>

static void say(OtaProgressCb cb, const char *msg) {
  Serial.printf("[ota] %s\n", msg);
  if (cb) cb(msg);
}

bool otaWifiConnect(uint32_t timeoutMs) {
  if (settings.wifiSsid[0] == 0) return false;
  WiFi.mode(WIFI_STA);
  WiFi.begin(settings.wifiSsid, settings.wifiPass);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeoutMs) delay(200);
  return WiFi.status() == WL_CONNECTED;
}

void otaWifiOff() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

bool otaSyncNtp(OtaProgressCb cb) {
  bool hadWifi = (WiFi.status() == WL_CONNECTED);
  if (!hadWifi && !otaWifiConnect()) { say(cb, "WiFi connect failed"); return false; }

  say(cb, "Syncing time...");
  // เก็บ system time เป็นเวลาท้องถิ่นตรง ๆ → ใส่ offset ตอน configTime เลย
  configTime(settings.tzMinutes * 60, 0, "pool.ntp.org", "time.google.com");
  struct tm t;
  bool ok = getLocalTime(&t, 10000);
  if (ok) {
    hwTimeToRtc();
    say(cb, "Time synced");
  } else {
    say(cb, "Time sync failed");
  }
  if (!hadWifi) otaWifiOff();
  return ok;
}

// เทียบเวอร์ชันแบบ semver ง่าย ๆ : คืน true ถ้า remote ใหม่กว่า local
static bool isNewer(const String &remote, const char *local) {
  int r[3] = {0, 0, 0}, l[3] = {0, 0, 0};
  sscanf(remote.c_str(), "%d.%d.%d", &r[0], &r[1], &r[2]);
  sscanf(local,          "%d.%d.%d", &l[0], &l[1], &l[2]);
  for (int i = 0; i < 3; i++) {
    if (r[i] > l[i]) return true;
    if (r[i] < l[i]) return false;
  }
  return false;
}

static bool httpGetString(const String &url, String &out) {
  WiFiClientSecure client;
  client.setInsecure();                    // ไม่ตรวจใบรับรอง (ยอมรับได้สำหรับ manifest สาธารณะ)
  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  if (!http.begin(client, url)) return false;
  int code = http.GET();
  if (code != HTTP_CODE_OK) { http.end(); return false; }
  out = http.getString();
  http.end();
  return true;
}

bool otaCheckAndUpdate(OtaProgressCb cb) {
  if (settings.otaUrl[0] == 0) { say(cb, "otaUrl not set"); return false; }
  if (!otaWifiConnect()) { say(cb, "WiFi connect failed"); return false; }

  say(cb, "Checking version...");
  String manifest;
  if (!httpGetString(String(settings.otaUrl), manifest)) {
    say(cb, "Manifest read failed");
    otaWifiOff();
    return false;
  }

  String version, binUrl;
  int start = 0;
  while (start < (int)manifest.length()) {
    int nl = manifest.indexOf('\n', start);
    String line = (nl < 0) ? manifest.substring(start) : manifest.substring(start, nl);
    line.trim();
    if (line.startsWith("version=")) version = line.substring(8);
    else if (line.startsWith("url="))  binUrl = line.substring(4);
    if (nl < 0) break;
    start = nl + 1;
  }
  version.trim(); binUrl.trim();

  if (version.length() == 0 || binUrl.length() == 0) { say(cb, "Bad manifest format"); otaWifiOff(); return false; }
  if (!isNewer(version, FW_VERSION)) {
    char msg[64];
    snprintf(msg, sizeof(msg), "Already newest (%s)", FW_VERSION);
    say(cb, msg);
    otaWifiOff();
    return false;
  }

  char msg[64];
  snprintf(msg, sizeof(msg), "Found %s, downloading...", version.c_str());
  say(cb, msg);

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  if (!http.begin(client, binUrl)) { say(cb, "Cannot open bin URL"); otaWifiOff(); return false; }
  int code = http.GET();
  int len = http.getSize();
  if (code != HTTP_CODE_OK || len <= 0) { say(cb, "Download failed"); http.end(); otaWifiOff(); return false; }

  if (!Update.begin(len)) { say(cb, "Not enough OTA space"); http.end(); otaWifiOff(); return false; }
  size_t written = Update.writeStream(http.getStream());
  http.end();

  if (written != (size_t)len || !Update.end(true) || !Update.isFinished()) {
    say(cb, "Flash write failed");
    otaWifiOff();
    return false;
  }

  say(cb, "Update OK, rebooting");
  otaWifiOff();
  delay(1200);
  ESP.restart();
  return true;
}

#else   // ENABLE_OTA == 0

bool otaWifiConnect(uint32_t) { return false; }
void otaWifiOff() {}
bool otaSyncNtp(OtaProgressCb cb) { if (cb) cb("OTA disabled at build"); return false; }
bool otaCheckAndUpdate(OtaProgressCb cb) { if (cb) cb("OTA disabled at build"); return false; }

#endif

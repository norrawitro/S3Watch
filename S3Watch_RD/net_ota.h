// OTA แบบดึงเอง: บอร์ดอ่าน manifest จาก repo ส่วนตัว (GitHub API + กุญแจอ่านอย่างเดียว)
// ถ้าเวอร์ชันใหม่กว่า FW_VERSION → ดาวน์โหลด .bin → ตรวจ MD5 → ติดตั้งลงช่องสำรอง → รีสตาร์ต
// manifest (สร้างโดย tools/publish_ota.sh):
//   {"version": 2610171300, "file": "es3c28p/rec_s3_2610171300.bin", "size": 1086336, "md5": "..."}
// ต้องมีใน secrets.h: OTA_GH_TOKEN (กุญแจอ่าน NRW_OTA) · ต่อ Wi-Fi ก่อนเรียก
#pragma once
#include <NetworkClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include <time.h>
#include "github_ca.h"

#ifndef FW_VERSION
#define FW_VERSION 0                     // คอมไพล์ผ่าน upload.sh/publish_ota.sh จะใส่วันเวลาให้
#endif
#ifndef OTA_GH_TOKEN
#define OTA_GH_TOKEN ""
#endif
#ifndef OTA_GH_REPO
#define OTA_GH_REPO "norrawitro/NRW_OTA"
#endif
#ifndef OTA_MANIFEST
#define OTA_MANIFEST "es3c28p/manifest.json"
#endif

typedef void (*OtaSay)(const char *msg);
typedef void (*OtaProgress)(int pct);

// ดึงค่าจาก JSON แบบง่าย (manifest เราสร้างเอง รูปแบบคงที่)
static String jsonField(const String &js, const char *key) {
  int k = js.indexOf(String("\"") + key + "\"");
  if (k < 0) return "";
  int c = js.indexOf(':', k);
  int s = c + 1;
  while (s < (int)js.length() && (js[s] == ' ' || js[s] == '"')) s++;
  int e = s;
  while (e < (int)js.length() && js[e] != '"' && js[e] != ',' && js[e] != '}') e++;
  return js.substring(s, e);
}

static bool ghOpen(HTTPClient &http, NetworkClientSecure &cli, const String &path) {
  cli.setCACert(GITHUB_CA);
  cli.setTimeout(15);
  String url = String("https://api.github.com/repos/") + OTA_GH_REPO + "/contents/" + path + "?ref=main";
  if (!http.begin(cli, url)) return false;
  http.setTimeout(15000);
  http.addHeader("Authorization", String("Bearer ") + OTA_GH_TOKEN);
  http.addHeader("Accept", "application/vnd.github.raw");
  http.addHeader("X-GitHub-Api-Version", "2022-11-28");
  http.setUserAgent(OTA_MANIFEST);
  return true;
}

// ตั้งนาฬิกาจาก NTP (ใช้ตรวจวันหมดอายุใบรับรอง)
static void otaSyncTime() {
  if (time(nullptr) > 1700000000) return;
  configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com");
  for (int i = 0; i < 50 && time(nullptr) < 1700000000; i++) delay(100);
}

// คืน true = มีอัปเดตและติดตั้งสำเร็จ (ผู้เรียกต้องรีสตาร์ต) · false = ไม่มี/ผิดพลาด (ข้อความผ่าน say)
bool otaPull(OtaSay say, OtaProgress prog) {
  if (strlen(OTA_GH_TOKEN) == 0) { say("no OTA_GH_TOKEN in secrets.h"); return false; }
  otaSyncTime();

  say("checking update...");
  NetworkClientSecure cli;
  HTTPClient http;
  if (!ghOpen(http, cli, OTA_MANIFEST)) { say("connect failed"); return false; }
  int code = http.GET();
  if (code != 200) {
    char m[48]; snprintf(m, sizeof(m), "manifest HTTP %d", code); say(m);
    http.end(); return false;
  }
  String js = http.getString();
  http.end();

  uint32_t ver = strtoul(jsonField(js, "version").c_str(), nullptr, 10);
  String file = jsonField(js, "file"), md5 = jsonField(js, "md5");
  uint32_t size = strtoul(jsonField(js, "size").c_str(), nullptr, 10);
  Serial.printf("OTA manifest: version %u (running %u) size %u\n", ver, (uint32_t)FW_VERSION, size);
  if (!ver || !size || file.isEmpty() || md5.length() != 32) { say("bad manifest"); return false; }
  if (ver <= (uint32_t)FW_VERSION) { say("already up to date"); return false; }

  char m[48]; snprintf(m, sizeof(m), "downloading v%u", ver); say(m);
  if (!ghOpen(http, cli, file)) { say("connect failed"); return false; }
  code = http.GET();
  if (code != 200) {
    snprintf(m, sizeof(m), "firmware HTTP %d", code); say(m);
    http.end(); return false;
  }
  if (!Update.begin(size)) { say("not enough OTA space"); http.end(); return false; }
  Update.setMD5(md5.c_str());
  NetworkClient *s = http.getStreamPtr();
  uint8_t buf[2048];
  uint32_t done = 0, lastT = millis();
  int lastPct = -1;
  while (done < size && millis() - lastT < 15000) {
    size_t avail = s->available();
    if (!avail) { delay(5); continue; }
    int n = s->readBytes(buf, min(avail, sizeof(buf)));
    if (n <= 0) continue;
    if (Update.write(buf, n) != (size_t)n) break;
    done += n; lastT = millis();
    int pct = done * 100 / size;
    if (pct != lastPct) { lastPct = pct; prog(pct); }
  }
  http.end();
  if (done != size) { Update.abort(); say("download incomplete"); return false; }
  if (!Update.end()) {                   // ตรวจ MD5 ตรงนี้ — ไม่ตรง = ไม่ติดตั้ง
    snprintf(m, sizeof(m), "verify failed (%s)", Update.errorString()); say(m);
    return false;
  }
  say("update OK, restarting");
  return true;
}

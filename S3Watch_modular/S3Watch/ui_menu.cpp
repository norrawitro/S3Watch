#include "ui_menu.h"
#include "config.h"
#include "hw.h"
#include "settings.h"
#include "display_1px.h"
#include "audio_rec.h"
#include "imu_wake.h"
#include "ota.h"

#if USE_LVGL
#include <lvgl.h>
#endif

// ───────────────────────── โมเดลของเมนู ─────────────────────────
enum ItemType : uint8_t { IT_ENUM, IT_INT, IT_BOOL, IT_ACTION, IT_INFO };
enum VType    : uint8_t { V_NONE, V_U8, V_U16, V_U32, V_I16, V_BOOL };

struct MenuItem {
  const char *label;
  ItemType    type;
  VType       vt;
  void       *ptr;
  int32_t     vmin, vmax, vstep, vscale;   // vscale = ตัวหารตอนแสดงผล (ms→s ใช้ 1000)
  const char *const *opts;
  uint8_t     optCount;
  const int32_t *optVals;                  // ค่าจริงของแต่ละตัวเลือก (nullptr = 0..n-1)
  void      (*action)();
  const char *(*info)();                   // ข้อความสำหรับ IT_INFO / IT_ACTION
};

static const char *OPT_FACE[]  = { "Analog", "Digital" };
static const char *OPT_MIC[]   = { "ES7210", "ES8311" };
static const char *OPT_RATE[]  = { "8 kHz", "16 kHz", "32 kHz" };
static const int32_t VAL_RATE[] = { 8000, 16000, 32000 };
static const char *OPT_CPU[]   = { "80 MHz", "160 MHz", "240 MHz" };
static const int32_t VAL_CPU[] = { 80, 160, 240 };

static char s_toast[40] = "";
static uint32_t s_toastUntil = 0;

void uiToast(const char *msg) {
  strlcpy(s_toast, msg, sizeof(s_toast));
  s_toastUntil = millis() + 4000;
}

// ───────── การทำงานของรายการแบบปุ่มกด ─────────
static void actToggleRecord() {
  if (audioIsRecording()) { audioStop(); uiToast("Recording stopped"); }
  else if (audioStart())  { uiToast("Recording started"); }
  else                    { uiToast(audioLastError()); }
}
static void actSave()        { uiToast(settingsSave() ? "Settings saved" : "Save failed"); }
static void actCheckUpdate() { otaCheckAndUpdate(uiToast); }
static void actSyncTime()    { otaSyncNtp(uiToast); }
static void actFactory()     { settingsDefaults(); settingsSave(); uiToast("Defaults restored"); }
static void actReboot()      { settingsSave(); delay(300); ESP.restart(); }

static const char *infoRecord() { return audioIsRecording() ? "REC" : "off"; }
static const char *infoRing()   { static char b[16]; snprintf(b, sizeof(b), "%u%%", audioRingUsedPct()); return b; }
static const char *infoSsid()   { return settings.wifiSsid[0] ? settings.wifiSsid : "(not set)"; }
static const char *infoOta()    { return settings.otaUrl[0] ? "set" : "(not set)"; }
static const char *infoVer()    { return FW_VERSION; }
static const char *infoHw()     {
  static char b[24];
  snprintf(b, sizeof(b), "%s%s%s%s",
           hwRtcOk() ? "RTC " : "", hwImuOk() ? "IMU " : "",
           touchOk() ? "TP "  : "", hwSdOk()  ? "SD"  : "");
  return b;
}

#define ITEM_ENUM(l, p, vt, o, ov)  { l, IT_ENUM,   vt, p, 0,0,0,1, o, (uint8_t)(sizeof(o)/sizeof(o[0])), ov, nullptr, nullptr }
#define ITEM_INT(l, p, vt, a, b, s, sc) { l, IT_INT, vt, p, a, b, s, sc, nullptr, 0, nullptr, nullptr, nullptr }
#define ITEM_BOOL(l, p)             { l, IT_BOOL,   V_BOOL, p, 0,0,0,1, nullptr, 0, nullptr, nullptr, nullptr }
#define ITEM_ACT(l, f, i)           { l, IT_ACTION, V_NONE, nullptr, 0,0,0,1, nullptr, 0, nullptr, f, i }
#define ITEM_INFO(l, i)             { l, IT_INFO,   V_NONE, nullptr, 0,0,0,1, nullptr, 0, nullptr, nullptr, i }

static MenuItem MENU[] = {
  ITEM_ENUM("Watch face",    &settings.faceMode,       V_U8,  OPT_FACE, nullptr),
  ITEM_INT ("Bright raised", &settings.brightHighPct,  V_U8,  5, 100, 5, 1),
  ITEM_INT ("Bright dim",    &settings.brightDimPct,   V_U8,  0, 100, 5, 1),
  ITEM_INT ("Bright hold s", &settings.brightHoldMs,   V_U16, 1000, 60000, 1000, 1000),
  ITEM_BOOL("Wrist wake",    &settings.wristWake),
  ITEM_INT ("Wrist angle",   &settings.faceAngleDeg,   V_U8,  5, 60, 5, 1),
  ITEM_INT ("Wrist hold ms", &settings.faceHoldMs,     V_U16, 0, 2000, 50, 1),
  ITEM_INT ("Motion thr mg", &settings.womThresholdMg, V_U16, 20, 1000, 10, 1),
  ITEM_ACT ("Recording",     actToggleRecord, infoRecord),
  ITEM_INFO("Buffer used",   infoRing),
  ITEM_ENUM("Sample rate",   &settings.sampleRate,     V_U32, OPT_RATE, VAL_RATE),
  ITEM_ENUM("Mic chip",      &settings.micChip,        V_U8,  OPT_MIC, nullptr),
  ITEM_INT ("Flush at %",    &settings.flushPct,       V_U8,  5, 50, 5, 1),
  ITEM_INT ("Ring MB",       &settings.ringMB,         V_U8,  1, 6, 1, 1),
  ITEM_BOOL("WiFi enabled",  &settings.wifiEnabled),
  ITEM_INFO("WiFi SSID",     infoSsid),
  ITEM_INFO("OTA URL",       infoOta),
  ITEM_ACT ("Check update",  actCheckUpdate, nullptr),
  ITEM_ACT ("Sync time NTP", actSyncTime,    nullptr),
  ITEM_INT ("TZ minutes",    &settings.tzMinutes,      V_I16, -720, 840, 30, 1),
  ITEM_ENUM("CPU speed",     &settings.cpuMhz,         V_U16, OPT_CPU, VAL_CPU),
  ITEM_INFO("Firmware",      infoVer),
  ITEM_INFO("Hardware",      infoHw),
  ITEM_ACT ("Save settings", actSave,    nullptr),
  ITEM_ACT ("Restore defaults", actFactory, nullptr),
  ITEM_ACT ("Reboot",        actReboot,  nullptr),
};
static const int MENU_N = sizeof(MENU) / sizeof(MENU[0]);

// ───────── อ่าน/เขียนค่าในโครงสร้าง Settings ─────────
static int32_t getVal(const MenuItem &m) {
  switch (m.vt) {
    case V_U8:   return *(uint8_t  *)m.ptr;
    case V_U16:  return *(uint16_t *)m.ptr;
    case V_U32:  return (int32_t)*(uint32_t *)m.ptr;
    case V_I16:  return *(int16_t  *)m.ptr;
    case V_BOOL: return *(bool     *)m.ptr ? 1 : 0;
    default:     return 0;
  }
}

static void setVal(const MenuItem &m, int32_t v) {
  switch (m.vt) {
    case V_U8:   *(uint8_t  *)m.ptr = (uint8_t)v;  break;
    case V_U16:  *(uint16_t *)m.ptr = (uint16_t)v; break;
    case V_U32:  *(uint32_t *)m.ptr = (uint32_t)v; break;
    case V_I16:  *(int16_t  *)m.ptr = (int16_t)v;  break;
    case V_BOOL: *(bool     *)m.ptr = (v != 0);    break;
    default: break;
  }
}

static void valueText(const MenuItem &m, char *out, size_t n) {
  switch (m.type) {
    case IT_ENUM: {
      int32_t v = getVal(m);
      int idx = 0;
      if (m.optVals) { for (uint8_t i = 0; i < m.optCount; i++) if (m.optVals[i] == v) idx = i; }
      else idx = constrain((int)v, 0, (int)m.optCount - 1);
      strlcpy(out, m.opts[idx], n);
      break;
    }
    case IT_BOOL:   strlcpy(out, getVal(m) ? "ON" : "OFF", n); break;
    case IT_INT: {
      int32_t v = getVal(m);
      if (m.vscale > 1) snprintf(out, n, "%ld", (long)(v / m.vscale));
      else              snprintf(out, n, "%ld", (long)v);
      break;
    }
    case IT_ACTION: strlcpy(out, m.info ? m.info() : ">", n); break;
    case IT_INFO:   strlcpy(out, m.info ? m.info() : "", n); break;
  }
}

// ───────── สถานะเมนู ─────────
static bool s_open = false;
static int  s_sel = 0;
static bool s_editing = false;
static bool s_dirty = true;       // ต้องวาดใหม่

static void applyLiveChanges() {
  settingsClampAll();
  hwSetBrightnessPct(settings.brightHighPct);
  setCpuFrequencyMhz(settings.cpuMhz);
}

static void doSelect() {
  MenuItem &m = MENU[s_sel];
  switch (m.type) {
    case IT_BOOL:
      setVal(m, getVal(m) ? 0 : 1);
      if (m.ptr == &settings.wristWake) imuConfigureWom();
      applyLiveChanges();
      break;
    case IT_ENUM: {
      int32_t v = getVal(m);
      int idx = 0;
      if (m.optVals) { for (uint8_t i = 0; i < m.optCount; i++) if (m.optVals[i] == v) idx = i; }
      else idx = constrain((int)v, 0, (int)m.optCount - 1);
      idx = (idx + 1) % m.optCount;
      setVal(m, m.optVals ? m.optVals[idx] : idx);
      applyLiveChanges();
      break;
    }
    case IT_INT:
      s_editing = true;                 // ปัดขึ้น/ลงเพื่อปรับค่า แล้วปัดขวาเพื่อยืนยัน
      break;
    case IT_ACTION:
      if (m.action) m.action();
      break;
    case IT_INFO:
      break;
  }
  s_dirty = true;
}

void uiGesture(Gesture g) {
  if (!s_open) return;
  MenuItem &m = MENU[s_sel];

  if (s_editing) {
    int32_t v = getVal(m);
    switch (g) {
      case GST_UP:    v += m.vstep; break;
      case GST_DOWN:  v -= m.vstep; break;
      case GST_RIGHT: s_editing = false; applyLiveChanges(); settingsSave(); s_dirty = true; return;
      case GST_LEFT:  s_editing = false; s_dirty = true; return;
      default: return;
    }
    setVal(m, constrain(v, m.vmin, m.vmax));
    s_dirty = true;
    return;
  }

  switch (g) {
    case GST_UP:    s_sel = (s_sel + 1) % MENU_N;             s_dirty = true; break;
    case GST_DOWN:  s_sel = (s_sel + MENU_N - 1) % MENU_N;    s_dirty = true; break;
    case GST_RIGHT: doSelect();                                               break;
    case GST_LEFT:  uiClose();                                                break;
    case GST_TAP:   s_dirty = true;                                           break;
    default: break;
  }
}

// ═══════════════════════ ตัววาด : LVGL ═══════════════════════
#if USE_LVGL

#define LV_ROWS 9
static lv_obj_t *s_rows[LV_ROWS];
static lv_obj_t *s_title;
static lv_obj_t *s_toastLbl;
static lv_display_t *s_disp = nullptr;
static uint8_t *s_lvBuf = nullptr;

static void lvFlush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;
  lv_draw_sw_rgb565_swap(px_map, w * h);
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)px_map, w, h);
  lv_display_flush_ready(disp);
}

static uint32_t lvTick() { return millis(); }

void uiInit() {
  lv_init();
  lv_tick_set_cb(lvTick);
  size_t bufPx = LCD_W * 40;
  s_lvBuf = (uint8_t *)heap_caps_malloc(bufPx * 2, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  s_disp = lv_display_create(LCD_W, LCD_H);
  lv_display_set_flush_cb(s_disp, lvFlush);
  lv_display_set_buffers(s_disp, s_lvBuf, nullptr, bufPx * 2, LV_DISPLAY_RENDER_MODE_PARTIAL);

  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  s_title = lv_label_create(scr);
  lv_obj_set_style_text_color(s_title, lv_color_white(), 0);
  lv_obj_align(s_title, LV_ALIGN_TOP_MID, 0, 12);

  for (int i = 0; i < LV_ROWS; i++) {
    s_rows[i] = lv_label_create(scr);
    lv_obj_set_style_text_color(s_rows[i], lv_color_white(), 0);
    lv_obj_set_width(s_rows[i], LCD_W - 24);
    lv_obj_align(s_rows[i], LV_ALIGN_TOP_LEFT, 12, 60 + i * 42);
  }
  s_toastLbl = lv_label_create(scr);
  lv_obj_set_style_text_color(s_toastLbl, lv_color_white(), 0);
  lv_obj_align(s_toastLbl, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_obj_add_flag(scr, LV_OBJ_FLAG_HIDDEN);      // ซ่อนไว้จนกว่าจะเปิดเมนู
}

static void lvRender() {
  char v[32], line[64];
  lv_label_set_text_fmt(s_title, "SETTINGS  %s", FW_VERSION);
  int first = s_sel - LV_ROWS / 2;
  if (first < 0) first = 0;
  if (first > MENU_N - LV_ROWS) first = max(0, MENU_N - LV_ROWS);
  for (int i = 0; i < LV_ROWS; i++) {
    int idx = first + i;
    if (idx >= MENU_N) { lv_label_set_text(s_rows[i], ""); continue; }
    valueText(MENU[idx], v, sizeof(v));
    const char *mark = (idx == s_sel) ? (s_editing ? "*" : ">") : " ";
    snprintf(line, sizeof(line), "%s %-15s %s", mark, MENU[idx].label, v);
    lv_label_set_text(s_rows[i], line);
  }
  lv_label_set_text(s_toastLbl, (millis() < s_toastUntil) ? s_toast : "");
}

void uiOpen() {
  s_open = true; s_sel = 0; s_editing = false; s_dirty = true;
  lv_obj_clear_flag(lv_screen_active(), LV_OBJ_FLAG_HIDDEN);
  lv_obj_invalidate(lv_screen_active());
}

void uiClose() {
  s_open = false; s_editing = false;
  settingsSave();
  lv_obj_add_flag(lv_screen_active(), LV_OBJ_FLAG_HIDDEN);
  faceInvalidate();
}

void uiTick() {
  if (!s_open) return;
  if (s_dirty || millis() < s_toastUntil) { lvRender(); s_dirty = false; }
  lv_timer_handler();
  delay(5);
}

// ═══════════════════════ ตัววาด : Arduino_GFX ═══════════════════════
#else

#define GFX_ROWS 9
static const int ROW_H = 42, ROW_Y0 = 62;

void uiInit() {}

static void gfxRender() {
  gfx->fillScreen(RGB565_BLACK);
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565_WHITE);
  gfx->setCursor(12, 20);
  gfx->printf("SETTINGS  %s", FW_VERSION);
  gfx->drawFastHLine(0, 50, LCD_W, RGB565_WHITE);

  int first = s_sel - GFX_ROWS / 2;
  if (first < 0) first = 0;
  if (first > MENU_N - GFX_ROWS) first = max(0, MENU_N - GFX_ROWS);

  char v[32];
  for (int i = 0; i < GFX_ROWS; i++) {
    int idx = first + i;
    if (idx >= MENU_N) break;
    int y = ROW_Y0 + i * ROW_H;
    bool sel = (idx == s_sel);
    if (sel) gfx->drawRect(6, y - 6, LCD_W - 12, ROW_H - 4, RGB565_WHITE);
    valueText(MENU[idx], v, sizeof(v));
    gfx->setCursor(16, y);
    gfx->print(s_editing && sel ? "*" : (sel ? ">" : " "));
    gfx->setCursor(38, y);
    gfx->print(MENU[idx].label);
    gfx->setCursor(LCD_W - 12 - (int)strlen(v) * 12, y);
    gfx->print(v);
  }

  if (millis() < s_toastUntil) {
    gfx->fillRect(0, LCD_H - 46, LCD_W, 46, RGB565_BLACK);
    gfx->drawFastHLine(0, LCD_H - 46, LCD_W, RGB565_WHITE);
    gfx->setCursor(12, LCD_H - 32);
    gfx->print(s_toast);
  }
}

void uiOpen()  { s_open = true; s_sel = 0; s_editing = false; s_dirty = true; }
void uiClose() { s_open = false; s_editing = false; settingsSave(); faceInvalidate(); }

void uiTick() {
  if (!s_open) return;
  static uint32_t lastToast = 0;
  bool toastChanged = (millis() < s_toastUntil) != (lastToast < s_toastUntil);
  lastToast = millis();
  if (s_dirty || toastChanged) { gfxRender(); s_dirty = false; }
  delay(10);
}

#endif

bool uiIsOpen() { return s_open; }

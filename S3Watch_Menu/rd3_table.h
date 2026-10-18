// RD3 ตารางเทียบเสียง (แก้ไขได้) [ผู้ใช้ขอ 2026-10-18]
// ไฟล์ /rd3/table.csv บนการ์ด SD — แก้ได้ 2 ทาง: (1) หน้า TABLE บนบอร์ด (MENU → TABLE) (2) Excel/Notepad ผ่านโหมด USB DRIVE
// ไม่มีไฟล์ → บอร์ดเขียนตารางเริ่มต้น (RD3_DEFAULT_CSV ข้างล่าง = เหมือน tools/rd3_table.csv)
//
// 1 แถว = 1 โฟนีม · ช่องว่าง = ไม่ใช้พารามิเตอร์นั้น
// ⚠️ ค่าเริ่มต้นเป็นช่วงความถี่ "คร่าว ๆ" ตามตำราสัทศาสตร์ทั่วไป (เสียงผู้ชาย) ยังไม่ได้วัดจากเสียงผู้ใช้ — ต้องปรับ
//   label  ตัวอักษรไทย   show  ชื่อสั้นภาษาอังกฤษ (จอแสดงไทยไม่ได้)   ipa  สัทอักษร   kind  ชนิดเสียง
//   active 1 = ใช้เทียบ · 0 = เก็บไว้แต่ไม่เทียบ (แยกจากคู่ของมันได้ด้วย "เวลา" เท่านั้น เช่น สระสั้น/ยาว ลมตาม — งาน RD3 ขั้นต่อไป)
//   voiced Y = ต้องมีระดับเสียง (f0) · N = ต้องไม่มี · ว่าง = ไม่สน
//   F1 F2 F3 ฟอร์แมนต์ (Hz) · cent จุดศูนย์ถ่วงสเปกตรัม (Hz) · hilo ความดังย่าน 3–8.7 kHz ลบ 0–3 kHz (dB)
// การเทียบ: ทุกพารามิเตอร์ที่กรอก — อยู่ในช่วง = 0 · นอกช่วง = (ระยะห่าง ÷ ความกว้างช่วง)² · รวม = คะแนนห่าง
//           แถว active ที่ห่างน้อยสุด และ ≤ RD3_ACCEPT (1.0) = คำตอบ · ไม่มี = "?"
#pragma once
#include <Arduino.h>
#include <FS.h>
#include "rd1.h"

#define RD3_MAX_ROWS 64
#define RD3_NPAR     5                   // F1 F2 F3 cent hilo
#define RD3_NCOL     17
#define RD3_ACCEPT   1.0f                // [ข้อเสนอ] คะแนนห่างที่ยอมรับ
#define RD3_NONE     -1e9f               // ช่องว่าง

const char RD3_HEADER[] = "label,show,ipa,kind,active,voiced,F1_lo,F1_hi,F2_lo,F2_hi,F3_lo,F3_hi,cent_lo,cent_hi,hilo_lo,hilo_hi,note";
const char RD3_DEFAULT_CSV[] =
  // ----- สระยาว 9 (ใช้เทียบ) -----
  "อี,ii,iː,vowel_long,1,Y,300,490,2000,2700,,,,,,,หน้า สูง\n"
  "อือ,uee,ɯː,vowel_long,1,Y,250,400,1150,1600,,,,,,,กลาง สูง ไม่ห่อปาก\n"
  "อู,uu,uː,vowel_long,1,Y,340,470,600,900,,,,,,,หลัง สูง ห่อปาก\n"
  "เอ,ee,eː,vowel_long,1,Y,420,600,1850,2350,,,,,,,หน้า กลาง\n"
  "เออ,ooe,ɤː,vowel_long,1,Y,400,600,1150,1500,,,,,,,กลาง กลาง\n"
  "โอ,oo,oː,vowel_long,1,Y,480,590,830,1100,,,,,,,หลัง กลาง ห่อปาก\n"
  "แอ,aae,ɛː,vowel_long,1,Y,590,720,1950,2350,,,,,,,หน้า ต่ำ\n"
  "อา,aa,aː,vowel_long,1,Y,720,1000,1000,2350,,,,,,,ต่ำ\n"
  "ออ,oor,ɔː,vowel_long,1,Y,610,760,1050,1500,,,,,,,หลัง ต่ำ ห่อปาก\n"
  // ----- สระสั้น 9 (ช่วงเดียวกับสระยาว → แยกด้วยความยาว 60-120 ms vs 150-300 ms) -----
  "อิ,i,i,vowel_short,0,Y,300,490,2000,2700,,,,,,,แยกจาก อี ด้วยความยาว\n"
  "อึ,ue,ɯ,vowel_short,0,Y,250,400,1150,1600,,,,,,,แยกจาก อือ ด้วยความยาว\n"
  "อุ,u,u,vowel_short,0,Y,340,470,600,900,,,,,,,แยกจาก อู ด้วยความยาว\n"
  "เอะ,e,e,vowel_short,0,Y,420,600,1850,2350,,,,,,,แยกจาก เอ ด้วยความยาว\n"
  "เออะ,oe,ɤ,vowel_short,0,Y,400,600,1150,1500,,,,,,,แยกจาก เออ ด้วยความยาว\n"
  "โอะ,o,o,vowel_short,0,Y,480,590,830,1100,,,,,,,แยกจาก โอ ด้วยความยาว\n"
  "แอะ,ae,ɛ,vowel_short,0,Y,590,720,1950,2350,,,,,,,แยกจาก แอ ด้วยความยาว\n"
  "อะ,a,a,vowel_short,0,Y,720,1000,1000,2350,,,,,,,แยกจาก อา ด้วยความยาว\n"
  "เอาะ,or,ɔ,vowel_short,0,Y,610,760,1050,1500,,,,,,,แยกจาก ออ ด้วยความยาว\n"
  // ----- สระประสม 3 (ค่า = จุดเริ่ม · F2 เลื่อนลงไปหา อา) -----
  "เอีย,ia,ia,diphthong,0,Y,250,400,1900,2600,,,,,,,เริ่มแบบ อี แล้วเลื่อนไป อา\n"
  "เอือ,uea,ɯa,diphthong,0,Y,250,400,1150,1600,,,,,,,เริ่มแบบ อือ แล้วเลื่อนไป อา\n"
  "อัว,ua,ua,diphthong,0,Y,250,400,600,950,,,,,,,เริ่มแบบ อู แล้วเลื่อนไป อา\n"
  // ----- พยัญชนะต้น 21 -----
  "ป,p,p,stop,0,N,,,,,,,500,1500,-20,0,ระเบิดสั้น ย่านต่ำกระจาย · 1 เฟรม → รอ RD3 จับจังหวะ (เปิดเองได้)\n"
  "ผ พ ภ,ph,pʰ,stop_asp,0,N,,,,,,,500,1500,-20,0,เหมือน ป + ลมตาม 60-90 ms\n"
  "บ,b,b,stop_voiced,0,Y,,,,,,,200,1500,-40,-15,ก้องก่อนระเบิด\n"
  "ต ฏ,t,t,stop,0,N,,,,,,,3000,6000,0,20,ระเบิดย่านสูง · 1 เฟรม → รอ RD3 (ทับช่วง ช ถ้าเปิด)\n"
  "ถ ท ธ ฐ ฑ ฒ,th,tʰ,stop_asp,0,N,,,,,,,3000,6000,0,20,เหมือน ต + ลมตาม\n"
  "ด ฎ,d,d,stop_voiced,0,Y,,,,,,,200,1500,-40,-15,ก้องก่อนระเบิด\n"
  "ก,k,k,stop,0,N,,,,,,,1500,3000,-10,10,ระเบิดย่านกลาง (กระจุก) · 1 เฟรม → รอ RD3\n"
  "ข ค ฆ,kh,kʰ,stop_asp,0,N,,,,,,,1500,3000,-10,10,เหมือน ก + ลมตาม\n"
  "จ,c,tɕ,affricate,0,N,,,,,,,2500,4500,3,40,เหมือน ช แต่สั้นกว่า\n"
  "ฉ ช ฌ,ch,tɕʰ,affricate,1,N,,,,,,,2500,4500,3,40,เสียดแทรก 2.5-4.5 kHz\n"
  "อ,q,ʔ,glottal,0,N,,,,,,,,,,,หยุดลมที่คอ = เงียบสั้น ไม่มีสเปกตรัม\n"
  "ม,m,m,nasal,1,Y,200,400,800,1300,,,200,800,-45,-15,นาสิก พลังงานต่ำ\n"
  "น ณ,n,n,nasal,1,Y,200,400,1300,1800,,,200,800,-45,-15,นาสิก\n"
  "ง,ng,ŋ,nasal,1,Y,200,400,1800,2400,,,200,800,-45,-15,นาสิก\n"
  "ฝ ฟ,f,f,fricative,1,N,,,,,,,2000,8000,-3,6,เบา กระจายกว้าง\n"
  "ซ ศ ษ ส,s,s,fricative,1,N,,,,,,,4500,9000,6,40,ซ่าแรง ย่านสูง\n"
  "ห ฮ,h,h,fricative,1,N,,,,,,,800,3000,-20,0,ลมเบา รูปตามสระที่ตามมา\n"
  "ร,r,r,liquid,1,Y,350,500,1100,1500,1600,2300,,,,,มักออกเป็น ล\n"
  "ล ฬ,l,l,liquid,1,Y,300,450,900,1300,2500,3200,,,,,\n"
  "ว,w,w,glide,0,Y,250,400,600,900,,,,,,,เหมือน อู แต่เลื่อนเร็ว\n"
  "ย ญ,y,j,glide,0,Y,250,350,2100,2600,,,,,,,เหมือน อี แต่เลื่อนเร็ว\n";

struct Rd3Row {
  char label[32], show[8], ipa[8], kind[14], note[96];
  bool active;
  int8_t voiced;                         // 1 = Y · 0 = N · -1 = ไม่สน
  float lo[RD3_NPAR], hi[RD3_NPAR];      // RD3_NONE = ว่าง
  bool has(int k) const { return lo[k] != RD3_NONE || hi[k] != RD3_NONE; }
};

struct Rd3Result { int best = -1, second = -1; float dist = 1e9f, dist2 = 1e9f; float par[RD3_NPAR] = {0}; bool voiced = false; };

class Rd3Table {
 public:
  Rd3Row rows[RD3_MAX_ROWS];
  int n = 0;
  bool dirty = false;                    // แก้ในหน้า TABLE แล้วยังไม่บันทึก
  char source[24] = "-";

  // อ่านจากการ์ด (ไม่มีไฟล์ → เขียนค่าเริ่มต้นก่อน) · ไม่มีการ์ด → ใช้ค่าเริ่มต้นในโค้ด
  void load(fs::FS *sd) {
    n = 0; dirty = false;
    if (sd) {
      // ตารางรุ่น 2 (2026-10-18): สระ 7 ตัวปรับตามเสียงผู้ใช้ที่บอร์ดวัดได้ · ไฟล์เดิมบนการ์ด → เก็บเป็น table_v1.csv (ไม่ลบ) แล้วเขียนตารางใหม่ครั้งเดียว
      if (!sd->exists("/rd3/v2.done")) {
        if (sd->exists("/rd3/table.csv")) {
          if (sd->exists("/rd3/table_v1.csv")) sd->remove("/rd3/table_v1.csv");
          sd->rename("/rd3/table.csv", "/rd3/table_v1.csv");
          Serial.println("RD3: table v2 -> old table kept as /rd3/table_v1.csv");
        }
        writeDefault(sd);
        File m = sd->open("/rd3/v2.done", FILE_WRITE); if (m) { m.println("table v2 2026-10-18"); m.close(); }
      }
      if (!sd->exists("/rd3/table.csv")) { writeDefault(sd); }
      File f = sd->open("/rd3/table.csv", FILE_READ);
      if (f) {                           // รูปแบบเก่า (ไม่มีคอลัมน์ ipa/active) → เก็บเป็น table_old.csv แล้วเขียนตารางใหม่
        String h = f.readStringUntil('\n');
        if (h.indexOf("label,show,ipa,kind,active") < 0) {
          f.close();
          if (sd->exists("/rd3/table_old.csv")) sd->remove("/rd3/table_old.csv");
          sd->rename("/rd3/table.csv", "/rd3/table_old.csv");
          Serial.println("RD3: old table format -> /rd3/table_old.csv");
          writeDefault(sd);
          f = sd->open("/rd3/table.csv", FILE_READ);
        } else f.seek(0);
      }
      if (f) {
        bool header = true;
        while (f.available() && n < RD3_MAX_ROWS) {
          String line = f.readStringUntil('\n');
          if (header) { header = false; continue; }
          parseLine(line.c_str());
        }
        f.close();
        strlcpy(source, "SD /rd3/table.csv", sizeof(source));
      }
    }
    if (!n) { loadDefault(); strlcpy(source, "built-in default", sizeof(source)); }
    Serial.printf("RD3 table: %d rows (%d active) from %s\n", n, activeCount(), source);
  }

  void loadDefault() {
    n = 0;
    const char *p = RD3_DEFAULT_CSV;
    while (*p && n < RD3_MAX_ROWS) {
      const char *e = strchr(p, '\n');
      String line(p, e ? e - p : strlen(p));
      parseLine(line.c_str());
      p = e ? e + 1 : p + strlen(p);
    }
  }

  void writeDefault(fs::FS *sd) {
    sd->mkdir("/rd3");
    File w = sd->open("/rd3/table.csv", FILE_WRITE);
    if (!w) return;
    w.write((const uint8_t *)"\xEF\xBB\xBF", 3);       // BOM: Excel อ่านภาษาไทยถูก
    w.println(RD3_HEADER); w.print(RD3_DEFAULT_CSV); w.close();
    Serial.println("RD3: wrote default /rd3/table.csv");
  }

  // บันทึกตารางที่แก้ในหน้า TABLE กลับลงการ์ด (เก็บไฟล์เดิมเป็น table.bak ก่อน)
  bool save(fs::FS *sd) {
    if (!sd) return false;
    sd->mkdir("/rd3");
    if (sd->exists("/rd3/table.bak")) sd->remove("/rd3/table.bak");
    if (sd->exists("/rd3/table.csv")) sd->rename("/rd3/table.csv", "/rd3/table.bak");
    File w = sd->open("/rd3/table.csv", FILE_WRITE);
    if (!w) return false;
    w.write((const uint8_t *)"\xEF\xBB\xBF", 3);
    w.println(RD3_HEADER);
    for (int i = 0; i < n; i++) {
      const Rd3Row &r = rows[i];
      w.printf("%s,%s,%s,%s,%d,%s", r.label, r.show, r.ipa, r.kind, r.active ? 1 : 0, r.voiced == 1 ? "Y" : r.voiced == 0 ? "N" : "");
      for (int k = 0; k < RD3_NPAR; k++) {
        if (r.lo[k] != RD3_NONE) w.printf(",%g", r.lo[k]); else w.print(",");
        if (r.hi[k] != RD3_NONE) w.printf(",%g", r.hi[k]); else w.print(",");
      }
      w.printf(",%s\n", r.note);
    }
    w.close();
    dirty = false;
    Serial.printf("RD3: saved %d rows to /rd3/table.csv (old -> table.bak)\n", n);
    return true;
  }

  int activeCount() const { int c = 0; for (int i = 0; i < n; i++) c += rows[i].active; return c; }

  // พารามิเตอร์จากเฟรม RD1: F1 F2 F3 · จุดศูนย์ถ่วง (Hz) · hilo (dB)
  static void features(const Rd1Frame &f, float par[RD3_NPAR]) {
    par[0] = f.fmt[0]; par[1] = f.fmt[1]; par[2] = f.fmt[2];
    double num = 0, den = 0, lo = 0, hi = 0;
    for (int b = 0; b < RD1_BANDS; b++) {
      float db = (f.band[b] - RD1_CFG.log_offset) * RD1_CFG.log_step_db;
      double pw = pow(10.0, db / 10.0), mid = (RD1_EDGES_HZ[b] + RD1_EDGES_HZ[b + 1]) / 2.0;
      num += pw * mid; den += pw;
      if (b < 8) lo += db; else if (b < 28) hi += db;   // 0–3 kHz (8 ช่วง) · 3–8.7 kHz (20 ช่วง)
    }
    par[3] = den > 0 ? num / den : 0;
    par[4] = hi / 20 - lo / 8;
  }

  Rd3Result match(const Rd1Frame &f) const {
    Rd3Result r;
    features(f, r.par);
    r.voiced = f.f0_cents != 0;
    for (int i = 0; i < n; i++) {
      const Rd3Row &w = rows[i];
      if (!w.active) continue;
      if (w.voiced == 1 && !r.voiced) continue;
      if (w.voiced == 0 && r.voiced) continue;
      float d = 0;
      bool ok = true, any = false;
      for (int k = 0; k < RD3_NPAR; k++) {
        if (!w.has(k)) continue;
        any = true;
        float v = r.par[k];
        if (k < 3 && v == 0) { ok = false; break; }           // ต้องใช้ฟอร์แมนต์ แต่เฟรมนี้หาไม่เจอ
        float l = w.lo[k] == RD3_NONE ? -1e9f : w.lo[k], h = w.hi[k] == RD3_NONE ? 1e9f : w.hi[k];
        float width = (w.lo[k] == RD3_NONE || w.hi[k] == RD3_NONE) ? max(1.0f, fabsf(l == -1e9f ? h : l) * 0.25f) : max(1.0f, h - l);
        float out = v < l ? l - v : v > h ? v - h : 0;
        d += (out / width) * (out / width);
      }
      if (!ok || !any) continue;
      if (d < r.dist) { r.second = r.best; r.dist2 = r.dist; r.best = i; r.dist = d; }
      else if (d < r.dist2) { r.second = i; r.dist2 = d; }
    }
    if (r.best >= 0 && r.dist > RD3_ACCEPT) r.best = -1;      // ห่างเกินไป = ไม่รู้จัก
    return r;
  }

  void print() const {
    Serial.printf("RD3 table (%s): %d rows, %d active%s\n", source, n, activeCount(), dirty ? "  (UNSAVED EDITS)" : "");
    static const char *P[RD3_NPAR] = {"F1", "F2", "F3", "cent", "hilo"};
    for (int i = 0; i < n; i++) {
      const Rd3Row &w = rows[i];
      Serial.printf("  %c %-12s %-4s %-5s %-12s voiced=%c", w.active ? '*' : ' ', w.label, w.show, w.ipa, w.kind,
                    w.voiced == 1 ? 'Y' : w.voiced == 0 ? 'N' : '-');
      for (int k = 0; k < RD3_NPAR; k++) if (w.has(k)) Serial.printf("  %s %g..%g", P[k], w.lo[k], w.hi[k]);
      Serial.println();
    }
  }

 private:
  void parseLine(const char *s) {
    if ((uint8_t)s[0] == 0xEF) s += 3;   // ข้าม BOM
    String col[RD3_NCOL];
    int c = 0;
    for (const char *p = s; *p; p++) {
      if (*p == ',' && c < RD3_NCOL - 1) { c++; continue; }   // คอลัมน์สุดท้าย (note) เก็บทั้งหมด
      if (*p != '\r') col[c] += *p;
    }
    if (!col[0].length() || col[0] == "label") return;
    Rd3Row &w = rows[n];
    strlcpy(w.label, col[0].c_str(), sizeof(w.label));
    strlcpy(w.show, col[1].length() ? col[1].c_str() : "?", sizeof(w.show));
    strlcpy(w.ipa, col[2].c_str(), sizeof(w.ipa));
    strlcpy(w.kind, col[3].c_str(), sizeof(w.kind));
    col[4].trim();
    w.active = col[4] != "0";
    w.voiced = col[5] == "Y" || col[5] == "y" ? 1 : col[5] == "N" || col[5] == "n" ? 0 : -1;
    for (int k = 0; k < RD3_NPAR; k++) {
      String a = col[6 + 2 * k], b = col[7 + 2 * k];
      a.trim(); b.trim();
      w.lo[k] = a.length() ? a.toFloat() : RD3_NONE;
      w.hi[k] = b.length() ? b.toFloat() : RD3_NONE;
    }
    strlcpy(w.note, col[16].c_str(), sizeof(w.note));
    n++;
  }
};

// แม่แบบ — คัดลอกเป็น secrets.h แล้วใส่ค่าจริง (secrets.h ไม่ขึ้น GitHub)
#pragma once
#define WIFI_SSID ""   // ชื่อ Wi-Fi (2.4 GHz เท่านั้น — ESP32-S3 ไม่รองรับ 5 GHz)
#define WIFI_PASS ""   // รหัส Wi-Fi
#define WIFI_SSID2 ""  // วงสำรอง — ลองต่อเมื่อวงแรกไม่ติด (เว้นว่างได้)
#define WIFI_PASS2 ""
#define OTA_PASS  ""   // รหัสสำหรับอัปโหลด OTA (upload.sh อ่านจากไฟล์นี้เอง)
// กุญแจอ่าน repo NRW_OTA (GitHub fine-grained token: Contents = Read-only เฉพาะ NRW_OTA)
#define OTA_GH_TOKEN ""

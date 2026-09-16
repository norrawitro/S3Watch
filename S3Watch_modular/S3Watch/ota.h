// ============================================================================
//  ota.h — WiFi + ตรวจเวอร์ชันจาก GitHub + อัปเดตเฟิร์มแวร์
//
//  WiFi เปิดเฉพาะตอนสั่งอัปเดต/ตั้งเวลา แล้วปิดทันที (ประหยัดไฟ)
//
//  ไฟล์ manifest ที่ otaUrl ชี้ไป เป็นข้อความธรรมดา 2 บรรทัด:
//      version=0.2.1
//      url=https://github.com/<user>/<repo>/releases/download/v0.2.1/S3Watch.bin
// ============================================================================
#pragma once
#include <Arduino.h>

typedef void (*OtaProgressCb)(const char *msg);

bool otaWifiConnect(uint32_t timeoutMs = 15000);
void otaWifiOff();
bool otaSyncNtp(OtaProgressCb cb);            // ตั้งเวลาจากอินเทอร์เน็ต → เขียนลง RTC
bool otaCheckAndUpdate(OtaProgressCb cb);     // ตรวจ + อัปเดต (รีบูตเองถ้าสำเร็จ)

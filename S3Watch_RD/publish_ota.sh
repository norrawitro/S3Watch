#!/usr/bin/env bash
# ปล่อยโค้ด S3Watch_RD ให้นาฬิกาดึงเอง (กดปุ่ม BOOT บนนาฬิกา หรือพิมพ์ u ทาง Serial)
# คอมไพล์ → วาง .bin + manifest.json ลง repo ส่วนตัว NRW_OTA (C:\Users\u\Projects\nrw_ota\s3watch) → push main
# ใช้: bash publish_ota.sh "<หมายเหตุ>"   · ไฟล์ทุกเวอร์ชันเก็บไว้ ไม่ลบ · บันทึกลง บันทึก.md
# .bin มีรหัส Wi-Fi ฝังอยู่ → วางได้เฉพาะ NRW_OTA (Private) ห้ามใส่ใน repo S3Watch
set -eu
cd "$(dirname "$0")"
NOTE=${1:-"-"}
OTA_REPO=/c/Users/u/Projects/nrw_ota
DEVICE=s3watch
SKETCH=S3Watch_RD
FQBN='esp32:esp32:esp32s3:CDCOnBoot=cdc,PSRAM=opi,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB'
CLI=/c/Users/u/AppData/Local/Programs/arduino-ide/resources/app/lib/backend/resources/arduino-cli.exe
LIBS='C:/Users/u/Documents/Arduino/libraries'
LOG="บันทึก.md"
VER=$(date +%y%m%d%H%M)

[ -f secrets.h ] || { echo "❌ ไม่มี secrets.h (คัดลอกจาก esp32s3/rec_s3/secrets.h) — นาฬิกาจะต่อ Wi-Fi/อัปต่อไม่ได้"; exit 1; }

out=$("$CLI" compile -b "$FQBN" --libraries "$LIBS" --build-property "compiler.cpp.extra_flags=-DFW_VERSION=$VER" --output-dir build . 2>&1) || true
echo "$out" | grep -E "error|Sketch uses" | tail -3
echo "$out" | grep -q "Sketch uses" || { echo "❌ คอมไพล์ไม่ผ่าน"; exit 1; }

BIN="build/$SKETCH.ino.bin"
FILE="$DEVICE/${SKETCH}_$VER.bin"
SIZE=$(stat -c %s "$BIN")
MD5=$(md5sum "$BIN" | cut -d' ' -f1)
mkdir -p "$OTA_REPO/$DEVICE"
cp "$BIN" "$OTA_REPO/$FILE"
cat > "$OTA_REPO/$DEVICE/manifest.json" <<EOF
{"version": $VER, "file": "$FILE", "size": $SIZE, "md5": "$MD5", "sketch": "$SKETCH", "note": "${NOTE//\"/\'}"}
EOF

git -C "$OTA_REPO" add "$FILE" "$DEVICE/manifest.json"
git -C "$OTA_REPO" -c user.name=norrawitro -c user.email=81816063+norrawitro@users.noreply.github.com commit -q -m "$DEVICE: $SKETCH fw $VER

${NOTE}"
git -C "$OTA_REPO" push -q origin HEAD:main
echo "✅ ปล่อยแล้ว fw $VER ($SIZE bytes) — กดปุ่ม BOOT บนนาฬิกาเพื่ออัปเดต"

echo "| $(date '+%Y-%m-%d %H:%M') | fw $VER | PUBLISH → NRW_OTA/$DEVICE | ✅ $SIZE B md5 ${MD5:0:8} · ${NOTE//|//} |" >> "$LOG"

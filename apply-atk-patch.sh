#!/usr/bin/env bash
# Run from the root of the libratbag repo.
# Copies driver files and applies all text patches.
set -e

DRIVER_DIR="$(dirname "$0")"
REPO_DIR="$(pwd)"

# Sanity check
if [ ! -f "$REPO_DIR/src/libratbag-private.h" ]; then
    echo "ERROR: Run this from the libratbag repo root."
    exit 1
fi

echo "=== Copying driver files ==="
cp "$DRIVER_DIR/driver-atk.c" "$REPO_DIR/src/"
cp "$DRIVER_DIR/driver-atk.h" "$REPO_DIR/src/"
echo "  src/driver-atk.c"
echo "  src/driver-atk.h"

echo ""
echo "=== Patching meson.build ==="
if grep -q "driver-atk" "$REPO_DIR/meson.build"; then
    echo "  already patched, skipping"
else
    sed -i "s|'src/driver-sinowealth-nubwo.c',|'src/driver-sinowealth-nubwo.c',\n\t'src/driver-atk.c',\n\t'src/driver-atk.h',|" \
        "$REPO_DIR/meson.build"
    echo "  done"
fi

echo ""
echo "=== Patching src/libratbag-private.h ==="
if grep -q "atk_driver" "$REPO_DIR/src/libratbag-private.h"; then
    echo "  already patched, skipping"
else
    sed -i "s|extern struct ratbag_driver sinowealth_nubwo_driver;|extern struct ratbag_driver sinowealth_nubwo_driver;\nextern struct ratbag_driver atk_driver;|" \
        "$REPO_DIR/src/libratbag-private.h"
    echo "  done"
fi

echo ""
echo "=== Patching src/libratbag.c ==="
if grep -q "atk_driver" "$REPO_DIR/src/libratbag.c"; then
    echo "  already patched, skipping"
else
    sed -i "s|ratbag_register_driver(ratbag, &roccat_emp_driver);|ratbag_register_driver(ratbag, &roccat_emp_driver);\n\tratbag_register_driver(ratbag, \&atk_driver);|" \
        "$REPO_DIR/src/libratbag.c"
    echo "  done"
fi

echo ""
echo "=== Patching src/libratbag-data.h (driver enum) ==="
# Find the enum that contains SINOWEALTH and add ATK after SINOWEALTH_NUBWO
if grep -q "ATK," "$REPO_DIR/src/libratbag-data.h" 2>/dev/null; then
    echo "  already patched, skipping"
else
    # Try to find SINOWEALTH_NUBWO in the enum and add ATK after it
    sed -i "s|SINOWEALTH_NUBWO,|SINOWEALTH_NUBWO,\n\tATK,|" \
        "$REPO_DIR/src/libratbag-data.h" 2>/dev/null && echo "  done" || \
        echo "  MANUAL: Add ATK to driver enum in src/libratbag-data.h"
fi

echo ""
echo "=== Patching src/libratbag-data.c (driver_id_map) ==="
if grep -q '"atk"' "$REPO_DIR/src/libratbag-data.c"; then
    echo "  already patched, skipping"
else
    sed -i 's|{ SINOWEALTH_NUBWO, "sinowealth_nubwo", NULL}|{ SINOWEALTH_NUBWO, "sinowealth_nubwo", NULL},\n\t{ ATK, "atk", NULL }|' \
        "$REPO_DIR/src/libratbag-data.c" && echo "  done" || \
        echo "  MANUAL: Add { ATK, \"atk\", NULL } to driver_id_map in src/libratbag-data.c"
fi

echo ""
echo "=== Creating device files ==="

cat > "$REPO_DIR/data/devices/atk-vxe-r1-nk.device" << 'DEVEOF'
[Device]
Name=ATK VXE R1 NK (Nearlink)
Driver=atk
DeviceType=mouse
DeviceMatch=usb:373b:1065
DeviceMatch=usb:373b:1066
DeviceMatch=usb:373b:10c7
DeviceMatch=usb:373b:10c8
DEVEOF

cat > "$REPO_DIR/data/devices/atk-vxe-r1-s-nk.device" << 'DEVEOF'
[Device]
Name=ATK VXE R1 S NK
Driver=atk
DeviceType=mouse
DeviceMatch=usb:373b:121f
DeviceMatch=usb:373b:1216
DeviceMatch=usb:373b:124d
DEVEOF

cat > "$REPO_DIR/data/devices/atk-vxe-r1-ultra-nk.device" << 'DEVEOF'
[Device]
Name=ATK VXE R1 Ultra NK
Driver=atk
DeviceType=mouse
DeviceMatch=usb:373b:11d2
DeviceMatch=usb:373b:11f9
DEVEOF

cat > "$REPO_DIR/data/devices/atk-vxe-r1.device" << 'DEVEOF'
[Device]
Name=ATK VXE R1
Driver=atk
DeviceType=mouse
DeviceMatch=usb:3554:f58f

[Device]
Name=ATK VXE R1 PRO NK
Driver=atk
DeviceType=mouse
DeviceMatch=usb:3554:f58c
DEVEOF

cat > "$REPO_DIR/data/devices/atk-f1.device" << 'DEVEOF'
[Device]
Name=ATK F1 PRO
Driver=atk
DeviceType=mouse
DeviceMatch=usb:373b:1030
DeviceMatch=usb:373b:1031

[Device]
Name=ATK F1 V2 NK
Driver=atk
DeviceType=mouse
DeviceMatch=usb:373b:10b5
DeviceMatch=usb:373b:10b6
DEVEOF

cat > "$REPO_DIR/data/devices/atk-a9.device" << 'DEVEOF'
[Device]
Name=ATK A9 Plus NK
Driver=atk
DeviceType=mouse
DeviceMatch=usb:373b:1085

[Device]
Name=ATK A9
Driver=atk
DeviceType=mouse
DeviceMatch=usb:373b:10f1
DEVEOF

echo "  data/devices/atk-vxe-r1-nk.device"
echo "  data/devices/atk-vxe-r1-s-nk.device"
echo "  data/devices/atk-vxe-r1-ultra-nk.device"
echo "  data/devices/atk-vxe-r1.device"
echo "  data/devices/atk-f1.device"
echo "  data/devices/atk-a9.device"

echo ""
echo "=== Done. Now build: ==="
echo "  meson setup build && ninja -C build"
echo ""
echo "=== Then test: ==="
echo "  lsusb | grep -i '373b\|3554'"
echo "  sudo ./build/ratbagd/ratbagd -v &"
echo "  sudo ./build/tools/ratbagctl list"
echo "  sudo ./build/tools/ratbagctl 'ATK VXE R1 NK' info"

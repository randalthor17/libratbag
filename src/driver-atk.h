/* SPDX-License-Identifier: MIT */
/*
 * ATK/VXE mouse HID protocol constants
 * Fully reverse engineered from ATK HUB v3.x (filter-B6RHekbD.js)
 *
 * Single protocol covers ALL variants: wired, 2.4GHz dongle, Nearlink.
 * NK/Nearlink differs only in capability flags (no far-distance, etc).
 */

#pragma once
#include <stdbool.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* HID interface                                                       */
/* ------------------------------------------------------------------ */

#define ATK_USAGE_PAGE 0xff04 /* vendor-defined */
#define ATK_USAGE 0x02
#define ATK_REPORT_ID 0x08
#define ATK_FRAME_SIZE 16

/* Frame byte offsets */
#define ATK_OFF_CMD 0     /* command ID */
#define ATK_OFF_STATUS 1  /* 0 in requests */
#define ATK_OFF_ADDR_HI 2 /* EEPROM address MSB */
#define ATK_OFF_ADDR_LO 3 /* EEPROM address LSB */
#define ATK_OFF_DATALEN 4 /* payload byte count */
#define ATK_OFF_PAYLOAD 5 /* data starts here (baseOffset) */
#define ATK_OFF_CHECKSUM 15

/*
 * Checksum: (0x55 - (0x08 + sum(frame[0..14]))) & 0xff
 * 0x08 = ATK_REPORT_ID is included in the sum.
 * Inline CRC for single EEPROM fields: (0x55 - value) & 0xff
 */
#define ATK_CHECKSUM_MAGIC 0x55

/* ------------------------------------------------------------------ */
/* Commands                                                            */
/* ------------------------------------------------------------------ */

#define ATK_CMD_GET_WIRELESS_ONLINE 0x03
#define ATK_CMD_GET_BATTERY 0x04
#define ATK_CMD_SET_PAIR 0x05
#define ATK_CMD_GET_PAIR_RESULT 0x06
#define ATK_CMD_SET_EEPROM 0x07
#define ATK_CMD_GET_EEPROM 0x08
#define ATK_CMD_RESTORE_FACTORY 0x09
#define ATK_CMD_GET_CURRENT_CONFIG 0x0e
#define ATK_CMD_SET_CURRENT_CONFIG 0x0f
#define ATK_CMD_GET_CIDMID 0x10
#define ATK_CMD_GET_VERSION 0x12
#define ATK_CMD_DONGLE_EXIT_PAIR 0x13
#define ATK_CMD_SET_FAR_DISTANCE 0x16 /* not for NK */
#define ATK_CMD_GET_FAR_DISTANCE 0x17 /* not for NK */
#define ATK_CMD_SET_DONGLE_LIGHT 0x18
#define ATK_CMD_GET_DONGLE_LIGHT 0x19
#define ATK_CMD_GET_DONGLE_VERSION 0x1d

/* ------------------------------------------------------------------ */
/* EEPROM addresses                                                    */
/* ------------------------------------------------------------------ */
/*
 * All values have an inline CRC byte immediately after:
 *   CRC = (0x55 - value) & 0xff
 * Multi-field blocks pack fields+CRC pairs contiguously.
 */

#define ATK_ADDR_REPORT_RATE 0x0000
/* 10-byte block from 0x0000:
 *   [0] reportRate code, [1] CRC
 *   [2] maxDpi (stage count), [3] CRC
 *   [4] currentDpi (active stage), [5] CRC
 *   [6] bhop, [7] CRC
 *   [8] buttonOperationMode, [9] CRC
 */
#define ATK_ADDR_SILENT_HEIGHT 0x000a
/* 2-byte block:
 *   [0] silentHeight (lift-off distance: 1=1mm, 2=2mm), [1] CRC
 */
#define ATK_ADDR_DPI1 0x000c
/* 4 bytes per stage (8 stages in 4 groups):
 *   [0] xDpiCode, [1] yDpiCode, [2] dpiEx, [3] CRC
 *   CRC = (0x55 - (xDpiCode + yDpiCode + dpiEx)) & 0xff
 */
#define ATK_ADDR_DPI3 0x0014
#define ATK_ADDR_DPI5 0x001c
#define ATK_ADDR_DPI7 0x0024
#define ATK_ADDR_DPI1_COLOR 0x002c
/* 4 bytes per stage color:
 *   [0] R, [1] G, [2] B
 *   [3] CRC = (0x55 - (R+G+B)) & 0xff
 */
#define ATK_ADDR_DPI3_COLOR 0x0034
#define ATK_ADDR_DPI5_COLOR 0x003c
#define ATK_ADDR_DPI7_COLOR 0x0044
#define ATK_ADDR_DPI_RGB_EFFECTS 0x004c
/* 8-byte block:
 *   [0] mode (0=Off,1=SteadyOn,2=Breathing), [1] CRC
 *   [2] brightness (16=Low,128=Medium,255=High), [3] CRC
 *   [4] speed (1=Slow,3=Medium,5=Fast), [5] CRC
 *   [6] dpiRGBEnabled (0/1), [7] CRC
 */
#define ATK_ADDR_KEY0 0x0060
/* 4 bytes per button (buttons 0-5 at +0, +4, +8, +12, +16, +20):
 *   [0] keyClass, [1] value1, [2] value2
 *   [3] CRC = (0x55 - (keyClass+value1+value2)) & 0xff
 */
#define ATK_ADDR_KEY1 0x0064
#define ATK_ADDR_KEY2 0x0068
#define ATK_ADDR_KEY3 0x006c
#define ATK_ADDR_KEY4 0x0070
#define ATK_ADDR_KEY5 0x0074
#define ATK_ADDR_DECORATION_LIGHT 0x00a0
/* 7-byte block (write len=7, read len=10):
 *   [0] mode (0=Off,1=Steady,2=Breathing,5=Rainbow)
 *   [1] R, [2] G, [3] B
 *   [4] speed (0=Slow,5=Medium,9=Fast)
 *   [5] brightness (0=Low,5=Medium,9=High)
 *   [6] CRC = (0x55 - sum([0..5])) & 0xff
 *   NOTE: write uses len=7, read uses len=10 (extra 3 bytes padding)
 */
#define ATK_ADDR_STABILIZATION_TIME 0x00a9
/* 10-byte block:
 *   [0] stabilizationTime (debounce, ms), [1] CRC
 *   [2] motionSync (0=off,1=on),          [3] CRC
 *   [4] closeLedTime (sleep, val*10=sec),  [5] CRC
 *   [6] linearCorrection (0=off,1=on),    [7] CRC
 *   [8] rippleControl (0=off,1=on),       [9] CRC
 */
#define ATK_ADDR_SENSOR_ENABLE 0x00b5
/* 6-byte block:
 *   [0] sensorSleepEnabled (0/1),             [1] CRC
 *   [2] sensorSleepTime (val*10=sec),          [3] CRC
 *   [4] sensorModel (0=Basic,1=Competitive),   [5] CRC
 */
#define ATK_ADDR_ANGLE 0x00bd
/* 4-byte block:
 *   [0] angle (snapping value 0-9),  [1] CRC
 *   [2] angleSwitch (0=off,1=on),    [3] CRC
 */
#define ATK_ADDR_ROLLING_DELAY 0x00e3
/* 2-byte block:
 *   [0] rollingDelay, [1] CRC
 */
#define ATK_ADDR_KEY_SHORTCUTS0 0x0100 /* 32 bytes per slot, 16 slots */
#define ATK_ADDR_MACRO0 0x0300         /* see macro format below */
/* Macro slot structure at addr macro_N = 0x0300 + N*0x180:
 *   [0..9]  header: macroCount (byte 0), rest padding
 *   [10..]  actions: macroCount * 5 bytes each (up to 70 actions)
 *   Each 5-byte macro action:
 *     byte 0: flags
 *       bits[6]=Up, bits[7]=Down (mutually exclusive; both 0 = Scroll)
 *       bits[2:0] = keyType (000=Modify, 001=General, 010=Media,
 *                            011=Power,  100=Mouse,   101=MoveXY, 110=Scroll)
 *     byte 1: value1 (HID key code)
 *     byte 2: value2 (modifier byte for Modify type)
 *     bytes 3-4: delay (uint16 LE, milliseconds)
 */
/* Shortcut slot structure at addr shortcuts_N = 0x0100 + N*32:
 *   [0]   shortcutCount (number of key combos, max 6)
 *   [1..] shortcutCount * 3 bytes each:
 *     byte 0: flags (same keyStatus/keyType as macro)
 *     byte 1: value1 (HID key code)
 *     byte 2: value2 (modifier)
 */
#define ATK_MACRO_SLOT_SIZE 0x0180 /* 384 bytes per slot */
#define ATK_SHORTCUT_SLOT_SIZE 32
#define ATK_MACRO_ACTION_SIZE 5
#define ATK_SHORTCUT_ACTION_SIZE 3
#define ATK_MACRO_MAX_ACTIONS 70

/* ------------------------------------------------------------------ */
/* Report rate codes                                                   */
/* ------------------------------------------------------------------ */

#define ATK_RATE_1000HZ 0x01
#define ATK_RATE_500HZ 0x02
#define ATK_RATE_250HZ 0x04
#define ATK_RATE_125HZ 0x08
#define ATK_RATE_2000HZ 0x10
#define ATK_RATE_4000HZ 0x20
#define ATK_RATE_8000HZ 0x40

/* ------------------------------------------------------------------ */
/* Sensor types                                                        */
/* ------------------------------------------------------------------ */

#define ATK_SENSOR_PAW3395 0      /* formula: step=50/100, raw=dpi/step-1 */
#define ATK_SENSOR_PAW3395SE 1    /* sparse LUT, 120 entries, max 10000 */
#define ATK_SENSOR_PAW3395ULTRA 2 /* formula, max 36000 */
#define ATK_SENSOR_PAW3950 3
#define ATK_SENSOR_PAW3950DM 4
#define ATK_SENSOR_PAW3950ULTRA 5 /* 10-DPI steps below 10050, max 42000 */
#define ATK_SENSOR_PAW3955MASTER 6
#define ATK_SENSOR_PAW3315 7
#define ATK_SENSOR_PAW3311 8 /* max 3200 DPI */
#define ATK_SENSOR_CORE26K 9 /* max 26000 DPI */

/* ------------------------------------------------------------------ */
/* Button key classes                                                  */
/* ------------------------------------------------------------------ */

#define ATK_KEY_CLASS_CLOSE 0 /* disabled */
#define ATK_KEY_CLASS_MOUSE 1
#define ATK_KEY_CLASS_DPI 2
#define ATK_KEY_CLASS_SCROLL_SIDE 3
#define ATK_KEY_CLASS_FIREPOWER 4
#define ATK_KEY_CLASS_SHORTCUT 5
#define ATK_KEY_CLASS_MACRO 6
#define ATK_KEY_CLASS_REPORT_RATE 7
#define ATK_KEY_CLASS_LIGHT_TOGGLE 8
#define ATK_KEY_CLASS_PROFILE 9
#define ATK_KEY_CLASS_DPI_LOCK 10
#define ATK_KEY_CLASS_WHEEL 11

/* Mouse button values (key class = ATK_KEY_CLASS_MOUSE) */
#define ATK_MOUSE_BTN_DISABLE 0
#define ATK_MOUSE_BTN_LEFT 1
#define ATK_MOUSE_BTN_RIGHT 2
#define ATK_MOUSE_BTN_MIDDLE 4
#define ATK_MOUSE_BTN_SIDE1 8
#define ATK_MOUSE_BTN_SIDE2 16

/* DPI button values (key class = ATK_KEY_CLASS_DPI) */
#define ATK_DPI_BTN_CYCLE 1
#define ATK_DPI_BTN_UP 2
#define ATK_DPI_BTN_DOWN 3

/* ------------------------------------------------------------------ */
/* Macro action flags (byte 0 of 5-byte action record)                */
/* ------------------------------------------------------------------ */

#define ATK_MACRO_KEY_STATUS_MASK 0xc0
#define ATK_MACRO_KEY_DOWN 0x80
#define ATK_MACRO_KEY_UP 0x40
#define ATK_MACRO_KEY_SCROLL 0x00 /* neither bit set */
#define ATK_MACRO_KEY_TYPE_MASK 0x07
#define ATK_MACRO_TYPE_MODIFY 0  /* modifier key */
#define ATK_MACRO_TYPE_GENERAL 1 /* normal key */
#define ATK_MACRO_TYPE_MEDIA 2
#define ATK_MACRO_TYPE_POWER 3
#define ATK_MACRO_TYPE_MOUSE 4
#define ATK_MACRO_TYPE_MOVEXY 5
#define ATK_MACRO_TYPE_SCROLL 6

/* ------------------------------------------------------------------ */
/* LED modes                                                           */
/* ------------------------------------------------------------------ */

/* DPI RGB indicator modes */
#define ATK_DPI_RGB_OFF 0
#define ATK_DPI_RGB_STEADY 1
#define ATK_DPI_RGB_BREATHING 2

/* Decoration/logo light modes */
#define ATK_DECO_OFF 0
#define ATK_DECO_STEADY 1
#define ATK_DECO_BREATHING 2
#define ATK_DECO_RAINBOW 5

/* ------------------------------------------------------------------ */
/* Battery response fields                                             */
/* ------------------------------------------------------------------ */
/*
 * GetBatteryLevel response (cmd 0x04):
 *   payload[0] = batteryLevel (0-100 %)
 *   payload[1] = batteryCharge (0=not charging, 1=charging)
 *   payload[2-3] = batteryVoltage (uint16 LE, mV)
 */

/* ------------------------------------------------------------------ */
/* Device capability flags                                             */
/* ------------------------------------------------------------------ */

#define ATK_FLAG_NO_FAR_DISTANCE (1 << 0)  /* NK/Nearlink: no range boost */
#define ATK_FLAG_NO_DPI_RGB (1 << 1)       /* no per-DPI RGB indicator */
#define ATK_FLAG_NO_SHOW_POWER (1 << 2)    /* don't display battery */
#define ATK_FLAG_MIDDLE_DPI_BTN (1 << 3)   /* middle button = DPI cycle */
#define ATK_FLAG_NO_ZERO_DEBOUNCE (1 << 4) /* debounce min = 1ms */

/* ------------------------------------------------------------------ */
/* PAW3395SE sparse DPI lookup table entry                            */
/* ------------------------------------------------------------------ */

struct ratbag_device;

struct atk_dpi_lut_entry {
  uint8_t code;
  uint16_t dpi;
};

struct atk_macro_action {
  uint8_t flags;
  uint8_t value1;
  uint8_t value2;
  uint16_t delay_ms;
};

struct atk_macro {
  uint8_t count;
  struct atk_macro_action actions[70];
};

/* ------------------------------------------------------------------ */
/* Advanced mouse settings (not in libratbag API, use sysfs/custom)   */
/* ------------------------------------------------------------------ */

struct atk_advanced_settings {
  /* Stabilization/performance block (0x00a9, 10 bytes) */
  uint8_t debounce_ms;    /* stabilizationTime: key debounce 0-20ms */
  bool motion_sync;       /* motionSync: sync polling to motion */
  uint8_t sleep_time_sec; /* closeLedTime: LED off after N seconds */
  bool linear_correction; /* linearCorrection: straight-line correction */
  bool ripple_control;    /* rippleControl: anti-ripple filter */

  /* Sensor sleep block (0x00b5, 6 bytes) */
  bool sensor_sleep_enabled;
  uint8_t sensor_sleep_sec; /* sensorSleepTime: wake on move after N sec */
  uint8_t sensor_model;     /* 0=Basic, 1=Competitive (Gaming) */

  /* Angle block (0x00bd, 4 bytes) */
  uint8_t angle_value; /* 0-9 snapping strength */
  bool angle_enabled;  /* angleSwitch */

  /* Rolling delay / bhop (0x00e3, 2 bytes) */
  uint8_t rolling_delay; /* anti-double-click delay */

  /* Lift-off distance (0x000a, 2 bytes) */
  uint8_t lod; /* 1=1mm, 2=2mm */
};

int atk_read_advanced(struct ratbag_device *device,
                      struct atk_advanced_settings *s);
int atk_write_advanced(struct ratbag_device *device,
                       const struct atk_advanced_settings *s);
int atk_read_dpi_color(struct ratbag_device *device, unsigned int stage,
                       uint8_t *r, uint8_t *g, uint8_t *b);
int atk_write_dpi_color(struct ratbag_device *device, unsigned int stage,
                        uint8_t r, uint8_t g, uint8_t b);
int atk_read_battery(struct ratbag_device *device, uint8_t *level,
                     bool *charging);
int atk_factory_reset(struct ratbag_device *device);

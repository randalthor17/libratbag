// SPDX-License-Identifier: MIT
/*
 * ATK/VXE mouse driver for libratbag
 *
 * Reverse engineered from ATK HUB v3.x (app.asar, filter-B6RHekbD.js).
 * Supports all ATK/VXE variants: wired, 2.4GHz, Nearlink.
 * NK/Nearlink uses identical protocol — only capability flags differ.
 *
 * Multi-profile: device banks EEPROM per profile.
 * Switch active bank with ATK_CMD_SET_CURRENT_CONFIG before EEPROM ops.
 *
 * Wire format:
 *   HID output report: buf[0]=0x08 (report ID), buf[1..16]=16-byte frame
 *   HID input report:  buf[0]=0x08 (report ID), buf[1..16]=16-byte response
 *
 * Frame layout (buf[1..16]):
 *   [0]    commandId
 *   [1]    commandStatus (0 in requests)
 *   [2]    eepromAddress high byte
 *   [3]    eepromAddress low byte
 *   [4]    dataValidLen
 *   [5-14] payload (baseOffset=5)
 *   [15]   checksum = (0x55 - (0x08 + sum(frame[0..14]))) & 0xff
 */

#include "driver-atk.h"
#include "config.h"
#include "libratbag-hidraw.h"
#include "libratbag-private.h"
#include "shared-macro.h"
#include <assert.h>
#include <errno.h>
#include <limits.h>
#include <linux/input.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#define ATK_NUM_PROFILES 4
#define ATK_NUM_DPI_STAGES 8
#define ATK_NUM_BUTTONS 6
#define ATK_NUM_LEDS 2
#define ATK_MAX_RETRIES 3

/*
 * Wire buffer: report ID byte + 16-byte frame = 17 bytes total.
 * ratbag_hidraw_output_report() calls write() directly, so buf[0]
 * must be the HID report ID.
 * ratbag_hidraw_read_input_report() also includes report ID at buf[0].
 */
#define ATK_WIRE_SIZE (1 + ATK_FRAME_SIZE)     /* 17 */
#define ATK_WIRE_PAYLOAD (1 + ATK_OFF_PAYLOAD) /* 6  */

/* ------------------------------------------------------------------ */
/* Device table                                                        */
/* ------------------------------------------------------------------ */

struct atk_device_info {
  uint8_t cid, mid;
  const char *name;
  int sensor;
  unsigned int max_dpi, min_dpi, dpi_step;
  uint32_t flags;
};

static const struct atk_device_info atk_devices[] = {
    /* VXE R1 */
    {2, 11, "VXE R1", ATK_SENSOR_PAW3395, 26000, 200, 50, 0},
    {2, 12, "VXE R1", ATK_SENSOR_PAW3395, 26000, 200, 50, 0},
    {2, 7, "VXE R1 PRO NK", ATK_SENSOR_PAW3395, 26000, 200, 50,
     ATK_FLAG_NO_FAR_DISTANCE},
    {2, 8, "VXE R1 PRO NK", ATK_SENSOR_PAW3395, 26000, 200, 50,
     ATK_FLAG_NO_FAR_DISTANCE},
    {2, 57, "VXE R1 NK", ATK_SENSOR_PAW3395SE, 10000, 50, 50,
     ATK_FLAG_NO_FAR_DISTANCE},
    {2, 67, "VXE R1 NK", ATK_SENSOR_PAW3395SE, 10000, 50, 50,
     ATK_FLAG_NO_FAR_DISTANCE},
    {2, 29, "VXE R1 SE", ATK_SENSOR_PAW3395SE, 18000, 50, 50, 0},
    {2, 30, "VXE R1 SE", ATK_SENSOR_PAW3395SE, 18000, 50, 50, 0},
    {2, 31, "VXE R1 SE+", ATK_SENSOR_PAW3395SE, 18000, 50, 50, 0},
    {2, 32, "VXE R1 SE+", ATK_SENSOR_PAW3395SE, 18000, 50, 50, 0},
    {2, 114, "VXE R1 Ultra NK", ATK_SENSOR_PAW3395, 26000, 200, 50,
     ATK_FLAG_NO_FAR_DISTANCE},
    {1, 20, "VXE R1 Ultra NK", ATK_SENSOR_PAW3395ULTRA, 36000, 200, 50,
     ATK_FLAG_NO_FAR_DISTANCE},
    {2, 127, "VXE R1 Ultra", ATK_SENSOR_PAW3950ULTRA, 42000, 100, 10, 0},
    {2, 117, "VXE R1 S NK", ATK_SENSOR_PAW3311, 3200, 400, 200,
     ATK_FLAG_NO_FAR_DISTANCE},
    /* ATK F1 */
    {2, 1, "ATK F1 PRO", ATK_SENSOR_PAW3395, 26000, 200, 50, 0},
    {2, 2, "ATK F1 PRO", ATK_SENSOR_PAW3395, 26000, 200, 50, 0},
    {2, 4, "ATK F1 PRO MAX", ATK_SENSOR_PAW3395, 26000, 200, 50, 0},
    {2, 6, "ATK F1 MOBA", ATK_SENSOR_PAW3395, 26000, 200, 50, 0},
    {2, 20, "ATK F1S", ATK_SENSOR_PAW3395, 26000, 200, 50, 0},
    {2, 93, "ATK F1 V2 Plus NK", ATK_SENSOR_PAW3395, 26000, 200, 50,
     ATK_FLAG_NO_FAR_DISTANCE},
    {2, 94, "ATK F1 V2 NK", ATK_SENSOR_PAW3395SE, 18000, 50, 50,
     ATK_FLAG_NO_FAR_DISTANCE},
    {2, 95, "ATK F1 V2 SE NK", ATK_SENSOR_PAW3311, 3200, 400, 200,
     ATK_FLAG_NO_FAR_DISTANCE},
    /* ATK A9 */
    {2, 83, "ATK A9 Plus NK", ATK_SENSOR_PAW3395, 26000, 200, 50,
     ATK_FLAG_NO_FAR_DISTANCE | ATK_FLAG_NO_DPI_RGB},
    {2, 84, "ATK A9 SE", ATK_SENSOR_PAW3395SE, 18000, 50, 50,
     ATK_FLAG_NO_FAR_DISTANCE | ATK_FLAG_NO_DPI_RGB},
    {2, 85, "ATK A9 Pro Max", ATK_SENSOR_PAW3395ULTRA, 36000, 200, 50, 0},
    {2, 88, "ATK A9 SE NK", ATK_SENSOR_PAW3395SE, 18000, 50, 50,
     ATK_FLAG_NO_FAR_DISTANCE | ATK_FLAG_NO_DPI_RGB},
    {2, 99, "ATK A9", ATK_SENSOR_PAW3395SE, 18000, 50, 50, ATK_FLAG_NO_DPI_RGB},
    /* ATK U2 */
    {2, 78, "ATK U2 Plus NK", ATK_SENSOR_PAW3395, 26000, 200, 50,
     ATK_FLAG_NO_FAR_DISTANCE},
    {2, 79, "ATK U2 NK", ATK_SENSOR_PAW3395SE, 18000, 50, 50,
     ATK_FLAG_NO_FAR_DISTANCE},
    {2, 92, "ATK U2 SE NK", ATK_SENSOR_PAW3311, 3200, 400, 200,
     ATK_FLAG_NO_FAR_DISTANCE},
    /* ATK X1 */
    {2, 76, "ATK X1 S NK", ATK_SENSOR_PAW3395, 26000, 200, 50,
     ATK_FLAG_NO_FAR_DISTANCE},
    {2, 77, "ATK X1 NK", ATK_SENSOR_PAW3395SE, 18000, 50, 50,
     ATK_FLAG_NO_FAR_DISTANCE},
    {2, 89, "ATK X1", ATK_SENSOR_PAW3395, 26000, 200, 50,
     ATK_FLAG_NO_FAR_DISTANCE},
    /* VXE V3 PRO */
    {2, 72, "VXE V3 PRO NK", ATK_SENSOR_PAW3395SE, 18000, 50, 50,
     ATK_FLAG_NO_FAR_DISTANCE | ATK_FLAG_NO_SHOW_POWER |
         ATK_FLAG_MIDDLE_DPI_BTN},
    {2, 73, "VXE V3 PRO NK", ATK_SENSOR_PAW3395SE, 18000, 50, 50,
     ATK_FLAG_NO_FAR_DISTANCE | ATK_FLAG_NO_SHOW_POWER |
         ATK_FLAG_MIDDLE_DPI_BTN},
    {0, 0, NULL, 0, 0, 0, 0, 0}};

/* ------------------------------------------------------------------ */
/* PAW3395SE sparse DPI LUT (120 entries, source-verified)            */
/* ------------------------------------------------------------------ */

static const struct atk_dpi_lut_entry atk_paw3395se_lut[] = {
    {0x01, 50},   {0x02, 100},  {0x03, 150},  {0x04, 200},  {0x05, 250},
    {0x06, 300},  {0x08, 350},  {0x09, 400},  {0x0a, 450},  {0x0b, 500},
    {0x0c, 550},  {0x0e, 600},  {0x0f, 650},  {0x10, 700},  {0x11, 750},
    {0x12, 800},  {0x13, 850},  {0x15, 900},  {0x16, 950},  {0x17, 1000},
    {0x18, 1050}, {0x19, 1100}, {0x1b, 1150}, {0x1c, 1200}, {0x1d, 1250},
    {0x1e, 1300}, {0x1f, 1350}, {0x20, 1400}, {0x22, 1450}, {0x23, 1500},
    {0x24, 1550}, {0x25, 1600}, {0x26, 1650}, {0x27, 1700}, {0x29, 1750},
    {0x2a, 1800}, {0x2b, 1850}, {0x2c, 1900}, {0x2d, 1950}, {0x2f, 2000},
    {0x30, 2050}, {0x31, 2100}, {0x32, 2150}, {0x33, 2200}, {0x34, 2250},
    {0x36, 2300}, {0x37, 2350}, {0x38, 2400}, {0x39, 2450}, {0x3a, 2500},
    {0x3b, 2550}, {0x3d, 2600}, {0x3e, 2650}, {0x3f, 2700}, {0x40, 2750},
    {0x41, 2800}, {0x43, 2850}, {0x44, 2900}, {0x45, 2950}, {0x46, 3000},
    {0x47, 3050}, {0x48, 3100}, {0x4a, 3150}, {0x4b, 3200}, {0x4c, 3250},
    {0x4d, 3300}, {0x4e, 3350}, {0x4f, 3400}, {0x51, 3450}, {0x52, 3500},
    {0x53, 3550}, {0x54, 3600}, {0x55, 3650}, {0x57, 3700}, {0x58, 3750},
    {0x59, 3800}, {0x5a, 3850}, {0x5b, 3900}, {0x5c, 3950}, {0x5e, 4000},
    {0x5f, 4050}, {0x60, 4100}, {0x61, 4150}, {0x62, 4200}, {0x63, 4250},
    {0x65, 4300}, {0x66, 4350}, {0x67, 4400}, {0x68, 4450}, {0x69, 4500},
    {0x6b, 4550}, {0x6c, 4600}, {0x6d, 4650}, {0x6e, 4700}, {0x6f, 4750},
    {0x70, 4800}, {0x72, 4850}, {0x73, 4900}, {0x74, 4950}, {0x75, 5000},
    {0xd4, 9050}, {0xd6, 9100}, {0xd7, 9150}, {0xd8, 9200}, {0xd9, 9250},
    {0xda, 9300}, {0xdb, 9350}, {0xdd, 9400}, {0xde, 9450}, {0xdf, 9500},
    {0xe0, 9550}, {0xe1, 9600}, {0xe3, 9650}, {0xe4, 9700}, {0xe5, 9750},
    {0xe6, 9800}, {0xe7, 9850}, {0xe8, 9900}, {0xea, 9950}, {0xeb, 10000},
};
#define ATK_SE_LUT_SIZE ARRAY_LENGTH(atk_paw3395se_lut)

/* ------------------------------------------------------------------ */
/* Private data                                                        */
/* ------------------------------------------------------------------ */

struct atk_data {
  uint8_t cid, mid;
  const struct atk_device_info *dev_info;
  int sensor;
  uint8_t fw_major, fw_minor;
  bool is_wireless;
  uint8_t battery_level;
  bool battery_charging;
};

/* ------------------------------------------------------------------ */
/* HID interface matching                                              */
/* ------------------------------------------------------------------ */

static int atk_test_hidraw(struct ratbag_device *device) {
  return (ratbag_hidraw_get_usage_page(device, ATK_REPORT_ID) ==
              ATK_USAGE_PAGE &&
          ratbag_hidraw_get_usage(device, ATK_REPORT_ID) == ATK_USAGE)
             ? 1
             : 0;
}

/* ------------------------------------------------------------------ */
/* Transport                                                           */
/* ------------------------------------------------------------------ */

static uint8_t atk_csum(const uint8_t frame[ATK_FRAME_SIZE]) {
  /* Report ID is included in the checksum but not in the frame array */
  unsigned int s = ATK_REPORT_ID;
  for (int i = 0; i < ATK_FRAME_SIZE - 1; i++)
    s += frame[i];
  return (ATK_CHECKSUM_MAGIC - s) & 0xff;
}

static void atk_frame_init(uint8_t frame[ATK_FRAME_SIZE], uint8_t cmd,
                           uint16_t addr, uint8_t len) {
  memset(frame, 0, ATK_FRAME_SIZE);
  frame[ATK_OFF_CMD] = cmd;
  frame[ATK_OFF_STATUS] = 0;
  frame[ATK_OFF_ADDR_HI] = addr >> 8;
  frame[ATK_OFF_ADDR_LO] = addr & 0xff;
  frame[ATK_OFF_DATALEN] = len;
}

static int atk_xfer(struct ratbag_device *device,
                    uint8_t frame_tx[ATK_FRAME_SIZE],
                    uint8_t frame_rx[ATK_FRAME_SIZE]) {
  /*
   * Wire format: buf[0] = report ID, buf[1..16] = frame.
   * ratbag_hidraw_output_report() calls write() directly,
   * ratbag_hidraw_read_input_report() reads into buf including report ID.
   */
  uint8_t tx[ATK_WIRE_SIZE], rx[ATK_WIRE_SIZE];
  int rc;

  frame_tx[ATK_OFF_CHECKSUM] = atk_csum(frame_tx);

  tx[0] = ATK_REPORT_ID;
  memcpy(&tx[1], frame_tx, ATK_FRAME_SIZE);

  for (int attempt = 0; attempt < ATK_MAX_RETRIES; attempt++) {
    rc = ratbag_hidraw_output_report(device, tx, ATK_WIRE_SIZE);
    if (rc < 0)
      continue;

    rc = ratbag_hidraw_read_input_report(device, rx, ATK_WIRE_SIZE, NULL);
    if (rc < 0)
      continue;

    /* rx[0] = report ID, rx[1] = commandId */
    if (rx[0] == ATK_REPORT_ID &&
        rx[1 + ATK_OFF_CMD] == frame_tx[ATK_OFF_CMD]) {
      memcpy(frame_rx, &rx[1], ATK_FRAME_SIZE);
      return 0;
    }
  }
  return -EIO;
}

/* ------------------------------------------------------------------ */
/* EEPROM helpers                                                      */
/* ------------------------------------------------------------------ */

#define P ATK_OFF_PAYLOAD

static int atk_read(struct ratbag_device *device, uint16_t addr, uint8_t len,
                    uint8_t *out) {
  uint8_t tx[ATK_FRAME_SIZE], rx[ATK_FRAME_SIZE];
  int rc;

  atk_frame_init(tx, ATK_CMD_GET_EEPROM, addr, len);
  rc = atk_xfer(device, tx, rx);
  if (rc == 0)
    memcpy(out, &rx[P], MIN((int)len, ATK_FRAME_SIZE - P));
  return rc;
}

static int atk_write(struct ratbag_device *device, uint16_t addr, uint8_t len,
                     const uint8_t *data) {
  uint8_t tx[ATK_FRAME_SIZE], rx[ATK_FRAME_SIZE];

  atk_frame_init(tx, ATK_CMD_SET_EEPROM, addr, len);
  memcpy(&tx[P], data, MIN((int)len, ATK_FRAME_SIZE - P));
  return atk_xfer(device, tx, rx);
}

/* Write one byte + its inline CRC = (0x55 - value) & 0xff */
static int atk_write_u8(struct ratbag_device *device, uint16_t addr,
                        uint8_t val) {
  uint8_t d[2] = {val, (ATK_CHECKSUM_MAGIC - val) & 0xff};
  return atk_write(device, addr, 2, d);
}

/* ------------------------------------------------------------------ */
/* Profile bank switching                                              */
/* ------------------------------------------------------------------ */

static int atk_switch_profile(struct ratbag_device *device, unsigned int idx) {
  uint8_t tx[ATK_FRAME_SIZE], rx[ATK_FRAME_SIZE];

  atk_frame_init(tx, ATK_CMD_SET_CURRENT_CONFIG, 0, 1);
  tx[P] = idx & 0x03;
  return atk_xfer(device, tx, rx);
}

static int atk_get_active_profile(struct ratbag_device *device, uint8_t *idx) {
  uint8_t tx[ATK_FRAME_SIZE], rx[ATK_FRAME_SIZE];
  int rc;

  atk_frame_init(tx, ATK_CMD_GET_CURRENT_CONFIG, 0, 0);
  rc = atk_xfer(device, tx, rx);
  if (rc == 0)
    *idx = rx[P] & 0x03;
  return rc;
}

/* ------------------------------------------------------------------ */
/* DPI codec                                                           */
/* ------------------------------------------------------------------ */

static uint8_t atk_se_enc(unsigned int dpi) {
  size_t i;
  unsigned int best = UINT_MAX;
  uint8_t code = atk_paw3395se_lut[0].code;

  for (i = 0; i < ATK_SE_LUT_SIZE; i++) {
    unsigned int d =
        (unsigned int)abs((int)atk_paw3395se_lut[i].dpi - (int)dpi);
    if (d < best) {
      best = d;
      code = atk_paw3395se_lut[i].code;
    }
  }
  return code;
}

static unsigned int atk_se_dec(uint8_t code) {
  size_t i;
  for (i = 0; i < ATK_SE_LUT_SIZE; i++)
    if (atk_paw3395se_lut[i].code == code)
      return atk_paw3395se_lut[i].dpi;
  return 0;
}

static void atk_paw3395_enc(unsigned int dpi, uint8_t *xc, uint8_t *yc,
                            uint8_t *ex) {
  unsigned int step = (dpi <= 30000) ? 50 : 100;
  unsigned int raw = dpi / step - 1;
  *xc = raw & 0xff;
  *yc = raw & 0xff;
  *ex = ((raw >> 8) & 3) << 2 | ((raw >> 8) & 3) << 6;
  if (dpi > 30000)
    *ex |= 0x11;
}

static unsigned int atk_paw3395_dec(uint8_t xc, uint8_t ex) {
  unsigned int raw = xc | (((ex >> 2) & 3) << 8);
  unsigned int dpi = 50 * (1 + raw);
  if (ex & 0x01)
    dpi *= 2;
  return dpi;
}

static uint16_t atk_dpi_addr(unsigned int stage) {
  static const uint16_t b[] = {ATK_ADDR_DPI1, ATK_ADDR_DPI3, ATK_ADDR_DPI5,
                               ATK_ADDR_DPI7};
  return b[stage / 2] + (stage % 2) * 4;
}

static uint16_t atk_color_addr(unsigned int stage) {
  static const uint16_t b[] = {ATK_ADDR_DPI1_COLOR, ATK_ADDR_DPI3_COLOR,
                               ATK_ADDR_DPI5_COLOR, ATK_ADDR_DPI7_COLOR};
  return b[stage / 2] + (stage % 2) * 4;
}

/* ------------------------------------------------------------------ */
/* Rate helpers                                                        */
/* ------------------------------------------------------------------ */

static const unsigned int atk_rates[] = {125, 250, 500, 1000, 2000, 4000, 8000};

static uint8_t atk_hz_enc(unsigned int hz) {
  switch (hz) {
  case 8000:
    return ATK_RATE_8000HZ;
  case 4000:
    return ATK_RATE_4000HZ;
  case 2000:
    return ATK_RATE_2000HZ;
  case 500:
    return ATK_RATE_500HZ;
  case 250:
    return ATK_RATE_250HZ;
  case 125:
    return ATK_RATE_125HZ;
  default:
    return ATK_RATE_1000HZ;
  }
}

static unsigned int atk_hz_dec(uint8_t c) {
  switch (c) {
  case ATK_RATE_8000HZ:
    return 8000;
  case ATK_RATE_4000HZ:
    return 4000;
  case ATK_RATE_2000HZ:
    return 2000;
  case ATK_RATE_500HZ:
    return 500;
  case ATK_RATE_250HZ:
    return 250;
  case ATK_RATE_125HZ:
    return 125;
  default:
    return 1000;
  }
}

/* ------------------------------------------------------------------ */
/* Button helpers                                                      */
/* ------------------------------------------------------------------ */

static const uint16_t atk_btn_addrs[6] = {ATK_ADDR_KEY0, ATK_ADDR_KEY1,
                                          ATK_ADDR_KEY2, ATK_ADDR_KEY3,
                                          ATK_ADDR_KEY4, ATK_ADDR_KEY5};

static void atk_btn_decode(struct ratbag_button *btn, const uint8_t r[4]) {
  struct ratbag_button_action a = {0};

  ratbag_button_enable_action_type(btn, RATBAG_BUTTON_ACTION_TYPE_BUTTON);
  ratbag_button_enable_action_type(btn, RATBAG_BUTTON_ACTION_TYPE_SPECIAL);
  ratbag_button_enable_action_type(btn, RATBAG_BUTTON_ACTION_TYPE_NONE);

  switch (r[0]) {
  case ATK_KEY_CLASS_MOUSE:
    a.type = RATBAG_BUTTON_ACTION_TYPE_BUTTON;
    switch (r[1]) {
    case ATK_MOUSE_BTN_LEFT:
      a.action.button = 1;
      break;
    case ATK_MOUSE_BTN_RIGHT:
      a.action.button = 2;
      break;
    case ATK_MOUSE_BTN_MIDDLE:
      a.action.button = 3;
      break;
    case ATK_MOUSE_BTN_SIDE1:
      a.action.button = 4;
      break;
    case ATK_MOUSE_BTN_SIDE2:
      a.action.button = 5;
      break;
    default:
      a.type = RATBAG_BUTTON_ACTION_TYPE_NONE;
    }
    break;
  case ATK_KEY_CLASS_DPI:
    a.type = RATBAG_BUTTON_ACTION_TYPE_SPECIAL;
    switch (r[1]) {
    case ATK_DPI_BTN_CYCLE:
      a.action.special = RATBAG_BUTTON_ACTION_SPECIAL_RESOLUTION_CYCLE_UP;
      break;
    case ATK_DPI_BTN_UP:
      a.action.special = RATBAG_BUTTON_ACTION_SPECIAL_RESOLUTION_UP;
      break;
    case ATK_DPI_BTN_DOWN:
      a.action.special = RATBAG_BUTTON_ACTION_SPECIAL_RESOLUTION_DOWN;
      break;
    default:
      a.type = RATBAG_BUTTON_ACTION_TYPE_NONE;
    }
    break;
  /*
   * ATK_KEY_CLASS_MACRO and ATK_KEY_CLASS_SHORTCUT store a slot index
   * in value1. Full macro support requires ratbag_macro_new() plumbing
   * which is deferred to a follow-up patch. Report as NONE for now so
   * piper does not crash.
   */
  default:
    a.type = RATBAG_BUTTON_ACTION_TYPE_NONE;
  }
  ratbag_button_set_action(btn, &a);
}

static void atk_btn_encode(const struct ratbag_button_action *a, uint8_t r[4]) {
  uint8_t k = ATK_KEY_CLASS_CLOSE, v1 = 0, v2 = 0;

  switch (a->type) {
  case RATBAG_BUTTON_ACTION_TYPE_BUTTON:
    k = ATK_KEY_CLASS_MOUSE;
    switch (a->action.button) {
    case 1:
      v1 = ATK_MOUSE_BTN_LEFT;
      break;
    case 2:
      v1 = ATK_MOUSE_BTN_RIGHT;
      break;
    case 3:
      v1 = ATK_MOUSE_BTN_MIDDLE;
      break;
    case 4:
      v1 = ATK_MOUSE_BTN_SIDE1;
      break;
    case 5:
      v1 = ATK_MOUSE_BTN_SIDE2;
      break;
    default:
      k = ATK_KEY_CLASS_CLOSE;
    }
    break;
  case RATBAG_BUTTON_ACTION_TYPE_SPECIAL:
    k = ATK_KEY_CLASS_DPI;
    switch (a->action.special) {
    case RATBAG_BUTTON_ACTION_SPECIAL_RESOLUTION_CYCLE_UP:
      v1 = ATK_DPI_BTN_CYCLE;
      break;
    case RATBAG_BUTTON_ACTION_SPECIAL_RESOLUTION_UP:
      v1 = ATK_DPI_BTN_UP;
      break;
    case RATBAG_BUTTON_ACTION_SPECIAL_RESOLUTION_DOWN:
      v1 = ATK_DPI_BTN_DOWN;
      break;
    default:
      k = ATK_KEY_CLASS_CLOSE;
    }
    break;
  default:
    break;
  }

  r[0] = k;
  r[1] = v1;
  r[2] = v2;
  r[3] = (ATK_CHECKSUM_MAGIC - (k + v1 + v2)) & 0xff;
}

/* ------------------------------------------------------------------ */
/* Profile read                                                        */
/* ------------------------------------------------------------------ */

static int atk_read_profile(struct ratbag_device *device,
                            struct ratbag_profile *profile) {
  struct atk_data *drv = ratbag_get_drv_data(device);
  struct ratbag_resolution *res;
  struct ratbag_button *btn;
  struct ratbag_led *led;
  uint8_t raw[10];
  int rc;

  /* Switch device to this profile's EEPROM bank */
  rc = atk_switch_profile(device, profile->index);
  if (rc < 0)
    return rc;

  /* Rate + DPI config block (10 bytes at 0x0000) */
  rc = atk_read(device, ATK_ADDR_REPORT_RATE, 10, raw);
  if (rc < 0)
    return rc;

  profile->hz = atk_hz_dec(raw[0]);
  ratbag_profile_set_report_rate_list(profile, atk_rates,
                                      ARRAY_LENGTH(atk_rates));

  unsigned int num_dpi = raw[2] & 0x0f;
  unsigned int cur_dpi = raw[4] & 0x0f;

  /* DPI stages */
  ratbag_profile_for_each_resolution(profile, res) {
    unsigned int idx = res->index;
    uint8_t rec[4];
    unsigned int dpi = 0;

    rc = atk_read(device, atk_dpi_addr(idx), 4, rec);
    if (rc < 0)
      continue;

    if (rec[0] == 0) {
      res->is_active = false;
      res->is_default = false;
      res->is_disabled = true;
      continue;
    }

    dpi = (drv->sensor == ATK_SENSOR_PAW3395SE)
              ? atk_se_dec(rec[0])
              : atk_paw3395_dec(rec[0], rec[2]);

    if (drv->dev_info)
      ratbag_resolution_set_dpi_list_from_range(res, drv->dev_info->min_dpi,
                                                drv->dev_info->max_dpi);
    else
      ratbag_resolution_set_dpi_list_from_range(res, 200, 26000);

    res->dpi_x = dpi;
    res->dpi_y = dpi;
    res->is_active = (idx < num_dpi);
    res->is_default = (idx == cur_dpi);
    res->is_disabled = false;
  }

  /* Buttons */
  ratbag_profile_for_each_button(profile, btn) {
    uint8_t rec[4];
    if (btn->index >= 6)
      break;
    rc = atk_read(device, atk_btn_addrs[btn->index], 4, rec);
    if (rc == 0)
      atk_btn_decode(btn, rec);
  }

  /* LEDs */
  ratbag_profile_for_each_led(profile, led) {
    switch (led->index) {

    case 0: { /* DPI RGB indicator (8 bytes at ATK_ADDR_DPI_RGB_EFFECTS) */
      if (drv->dev_info && (drv->dev_info->flags & ATK_FLAG_NO_DPI_RGB))
        break;

      rc = atk_read(device, ATK_ADDR_DPI_RGB_EFFECTS, 8, raw);
      if (rc < 0)
        break;

      if (!raw[6]) {
        led->mode = RATBAG_LED_OFF;
      } else if (raw[0] == ATK_DPI_RGB_BREATHING) {
        led->mode = RATBAG_LED_BREATHING;
      } else {
        led->mode = RATBAG_LED_ON;
      }

      /* brightness: 16=Low→64, 128=Medium→128, 255=High→255 */
      switch (raw[2]) {
      case 16:
        led->brightness = 64;
        break;
      case 128:
        led->brightness = 128;
        break;
      case 255:
        led->brightness = 255;
        break;
      default:
        led->brightness = raw[2];
      }

      led->modes = RATBAG_LED_OFF | RATBAG_LED_ON | RATBAG_LED_BREATHING;
      break;
    }

    case 1: { /* Decoration/logo light (10 bytes at ATK_ADDR_DECORATION_LIGHT)
               */
      rc = atk_read(device, ATK_ADDR_DECORATION_LIGHT, 10, raw);
      if (rc < 0)
        break;

      switch (raw[0]) {
      case ATK_DECO_OFF:
        led->mode = RATBAG_LED_OFF;
        break;
      case ATK_DECO_BREATHING:
        led->mode = RATBAG_LED_BREATHING;
        break;
      case ATK_DECO_RAINBOW:
        led->mode = RATBAG_LED_CYCLE;
        break;
      default:
        led->mode = RATBAG_LED_ON;
      }

      led->color.red = raw[1];
      led->color.green = raw[2];
      led->color.blue = raw[3];
      led->brightness = (raw[5] * 255) / 9;
      led->modes = RATBAG_LED_OFF | RATBAG_LED_ON | RATBAG_LED_BREATHING |
                   RATBAG_LED_CYCLE;
      break;
    }
    }
  }

  return 0;
}

/* ------------------------------------------------------------------ */
/* Probe                                                               */
/* ------------------------------------------------------------------ */

static int atk_probe(struct ratbag_device *device) {
  struct atk_data *drv;
  struct ratbag_profile *profile;
  uint8_t tx[ATK_FRAME_SIZE], rx[ATK_FRAME_SIZE];
  uint8_t active_profile = 0;
  int i, rc;

  rc = ratbag_find_hidraw(device, atk_test_hidraw);
  if (rc) {
    log_error(device->ratbag, "ATK: no config interface "
                              "(need usagePage=0xff04, usage=0x02)\n");
    return rc;
  }

  drv = zalloc(sizeof(*drv));
  if (!drv)
    return -ENOMEM;
  ratbag_set_drv_data(device, drv);

  /* Firmware version */
  atk_frame_init(tx, ATK_CMD_GET_VERSION, 0, 0);
  if (atk_xfer(device, tx, rx) == 0) {
    drv->fw_major = rx[P];
    drv->fw_minor = rx[P + 1];
    log_info(device->ratbag, "ATK: firmware %x.%02x\n", drv->fw_major,
             drv->fw_minor);
  }

  /* CID/MID identification */
  atk_frame_init(tx, ATK_CMD_GET_CIDMID, 0, 0);
  if (atk_xfer(device, tx, rx) == 0) {
    drv->cid = rx[P];
    drv->mid = rx[P + 1];
    log_info(device->ratbag, "ATK: CID=%d MID=%d\n", drv->cid, drv->mid);

    for (i = 0; atk_devices[i].name; i++) {
      if (atk_devices[i].cid == drv->cid && atk_devices[i].mid == drv->mid) {
        drv->dev_info = &atk_devices[i];
        drv->sensor = atk_devices[i].sensor;
        log_info(device->ratbag, "ATK: %s\n", drv->dev_info->name);
        break;
      }
    }

    if (!drv->dev_info) {
      log_info(device->ratbag,
               "ATK: unknown CID/MID %d/%d, "
               "defaulting to PAW3395\n",
               drv->cid, drv->mid);
      drv->sensor = ATK_SENSOR_PAW3395;
    }
  }

  /* Wireless + battery */
  atk_frame_init(tx, ATK_CMD_GET_WIRELESS_ONLINE, 0, 0);
  if (atk_xfer(device, tx, rx) == 0 && rx[P] == 1) {
    drv->is_wireless = true;

    atk_frame_init(tx, ATK_CMD_GET_BATTERY, 0, 0);
    if (atk_xfer(device, tx, rx) == 0) {
      drv->battery_level = rx[P];
      drv->battery_charging = rx[P + 1] != 0;
      log_info(device->ratbag, "ATK: battery %d%%%s\n", drv->battery_level,
               drv->battery_charging ? " (charging)" : "");
    }
  }

  /* Save which profile is currently active so we can restore it */
  atk_get_active_profile(device, &active_profile);

  ratbag_device_init_profiles(device, ATK_NUM_PROFILES, ATK_NUM_DPI_STAGES,
                              ATK_NUM_BUTTONS, ATK_NUM_LEDS);

  /* Read all 4 profiles (switches bank for each) */
  ratbag_device_for_each_profile(device, profile) {
    rc = atk_read_profile(device, profile);
    if (rc < 0)
      log_error(device->ratbag, "ATK: failed to read profile %d: %d\n",
                profile->index, rc);
  }

  /* Restore the original active profile */
  atk_switch_profile(device, active_profile);

  return 0;
}

/* ------------------------------------------------------------------ */
/* Remove                                                              */
/* ------------------------------------------------------------------ */

static void atk_remove(struct ratbag_device *device) {
  ratbag_close_hidraw(device);
  free(ratbag_get_drv_data(device));
}

/* ------------------------------------------------------------------ */
/* set_active_profile                                                  */
/* ------------------------------------------------------------------ */

static int atk_set_active_profile(struct ratbag_device *device,
                                  unsigned int idx) {
  return atk_switch_profile(device, idx);
}

/* ------------------------------------------------------------------ */
/* Commit                                                              */
/* ------------------------------------------------------------------ */

static int atk_commit(struct ratbag_device *device) {
  struct atk_data *drv = ratbag_get_drv_data(device);
  struct ratbag_profile *profile;
  struct ratbag_resolution *res;
  struct ratbag_button *btn;
  struct ratbag_led *led;
  uint8_t raw[10];
  int rc;

  ratbag_device_for_each_profile(device, profile) {
    if (!profile->dirty)
      continue;

    /* Switch to this profile's EEPROM bank */
    rc = atk_switch_profile(device, profile->index);
    if (rc < 0)
      return rc;

    /* Report rate */
    rc = atk_write_u8(device, ATK_ADDR_REPORT_RATE, atk_hz_enc(profile->hz));
    if (rc < 0)
      return rc;

    /* DPI stages */
    unsigned int num_active = 0, cur = 0;

    ratbag_profile_for_each_resolution(profile, res) {
      if (res->is_active)
        num_active = res->index + 1;
      if (res->is_default)
        cur = res->index;

      if (!res->dirty)
        continue;

      unsigned int dpi = res->dpi_x;
      uint8_t xc, yc, ex = 0, rec[4];

      if (drv->sensor == ATK_SENSOR_PAW3395SE) {
        xc = atk_se_enc(dpi);
        yc = xc;
        ex = 0;
      } else {
        atk_paw3395_enc(dpi, &xc, &yc, &ex);
      }

      rec[0] = xc;
      rec[1] = yc;
      rec[2] = ex;
      rec[3] = (ATK_CHECKSUM_MAGIC - (xc + yc + ex)) & 0xff;
      rc = atk_write(device, atk_dpi_addr(res->index), 4, rec);
      if (rc < 0)
        return rc;
    }

    rc = atk_read(device, ATK_ADDR_REPORT_RATE, 10, raw);
    if (rc == 0) {
      raw[2] = num_active;
      raw[3] = (ATK_CHECKSUM_MAGIC - num_active) & 0xff;
      raw[4] = cur;
      raw[5] = (ATK_CHECKSUM_MAGIC - cur) & 0xff;
      rc = atk_write(device, ATK_ADDR_REPORT_RATE, 10, raw);
    }
    if (rc < 0)
      return rc;

    /* Buttons */
    ratbag_profile_for_each_button(profile, btn) {
      uint8_t rec[4];
      if (!btn->dirty || btn->index >= 6)
        continue;
      atk_btn_encode(&btn->action, rec);
      rc = atk_write(device, atk_btn_addrs[btn->index], 4, rec);
      if (rc < 0)
        return rc;
    }

    /* LEDs */
    ratbag_profile_for_each_led(profile, led) {
      if (!led->dirty)
        continue;

      switch (led->index) {

      case 0: { /* DPI RGB indicator */
        if (drv->dev_info && (drv->dev_info->flags & ATK_FLAG_NO_DPI_RGB))
          break;

        /* Read-modify-write to preserve speed field */
        rc = atk_read(device, ATK_ADDR_DPI_RGB_EFFECTS, 8, raw);
        if (rc < 0)
          break;

        uint8_t mode = (led->mode == RATBAG_LED_BREATHING)
                           ? ATK_DPI_RGB_BREATHING
                           : ATK_DPI_RGB_STEADY;
        uint8_t en = (led->mode != RATBAG_LED_OFF) ? 1 : 0;
        uint8_t br = (led->brightness <= 64)    ? 16
                     : (led->brightness >= 220) ? 255
                                                : 128;

        raw[0] = mode;
        raw[1] = (ATK_CHECKSUM_MAGIC - mode) & 0xff;
        raw[2] = br;
        raw[3] = (ATK_CHECKSUM_MAGIC - br) & 0xff;
        raw[6] = en;
        raw[7] = (ATK_CHECKSUM_MAGIC - en) & 0xff;

        rc = atk_write(device, ATK_ADDR_DPI_RGB_EFFECTS, 8, raw);
        if (rc < 0)
          return rc;
        break;
      }

      case 1: { /* Decoration/logo light */
        /*
         * Layout (7 bytes written, 10 bytes read):
         *   [0] mode, [1] R, [2] G, [3] B,
         *   [4] speed (0-9), [5] brightness (0-9),
         *   [6] CRC = (0x55 - sum([0..5])) & 0xff
         */
        uint8_t d[7] = {0};

        switch (led->mode) {
        case RATBAG_LED_OFF:
          d[0] = ATK_DECO_OFF;
          break;
        case RATBAG_LED_BREATHING:
          d[0] = ATK_DECO_BREATHING;
          break;
        case RATBAG_LED_CYCLE:
          d[0] = ATK_DECO_RAINBOW;
          break;
        default:
          d[0] = ATK_DECO_STEADY;
        }

        d[1] = led->color.red;
        d[2] = led->color.green;
        d[3] = led->color.blue;
        d[4] = 5; /* medium speed */
        d[5] = (led->brightness * 9) / 255;

        unsigned int s = 0;
        for (int j = 0; j < 6; j++)
          s += d[j];
        d[6] = (ATK_CHECKSUM_MAGIC - (s & 0xff)) & 0xff;

        rc = atk_write(device, ATK_ADDR_DECORATION_LIGHT, 7, d);
        if (rc < 0)
          return rc;
        break;
      }
      }
    }
  }

  return 0;
}

/* ------------------------------------------------------------------ */
/* Driver registration                                                 */
/* ------------------------------------------------------------------ */

struct ratbag_driver atk_driver = {
    .name = "ATK",
    .id = "atk",
    .probe = atk_probe,
    .remove = atk_remove,
    .commit = atk_commit,
    .set_active_profile = atk_set_active_profile,
};

/* ================================================================== */
/* ADVANCED SETTINGS — not in libratbag API, call directly            */
/* ================================================================== */

int atk_read_advanced(struct ratbag_device *device,
                      struct atk_advanced_settings *s) {
  uint8_t raw[10];
  int rc;

  rc = atk_read(device, ATK_ADDR_STABILIZATION_TIME, 10, raw);
  if (rc < 0)
    return rc;
  s->debounce_ms = raw[0];
  s->motion_sync = raw[2] != 0;
  s->sleep_time_sec = raw[4] * 10;
  s->linear_correction = raw[6] != 0;
  s->ripple_control = raw[8] != 0;

  rc = atk_read(device, ATK_ADDR_SENSOR_ENABLE, 6, raw);
  if (rc < 0)
    return rc;
  s->sensor_sleep_enabled = raw[0] != 0;
  s->sensor_sleep_sec = raw[2] * 10;
  s->sensor_model = raw[4];

  rc = atk_read(device, ATK_ADDR_ANGLE, 4, raw);
  if (rc < 0)
    return rc;
  s->angle_value = raw[0];
  s->angle_enabled = raw[2] != 0;

  rc = atk_read(device, ATK_ADDR_ROLLING_DELAY, 2, raw);
  if (rc < 0)
    return rc;
  s->rolling_delay = raw[0];

  rc = atk_read(device, ATK_ADDR_SILENT_HEIGHT, 2, raw);
  if (rc < 0)
    return rc;
  s->lod = raw[0];

  return 0;
}

int atk_write_advanced(struct ratbag_device *device,
                       const struct atk_advanced_settings *s) {
  uint8_t raw[10];
  int rc;

#define CRC(v) ((ATK_CHECKSUM_MAGIC - (v)) & 0xff)

  raw[0] = s->debounce_ms;
  raw[1] = CRC(raw[0]);
  raw[2] = s->motion_sync ? 1 : 0;
  raw[3] = CRC(raw[2]);
  raw[4] = s->sleep_time_sec / 10;
  raw[5] = CRC(raw[4]);
  raw[6] = s->linear_correction ? 1 : 0;
  raw[7] = CRC(raw[6]);
  raw[8] = s->ripple_control ? 1 : 0;
  raw[9] = CRC(raw[8]);
  rc = atk_write(device, ATK_ADDR_STABILIZATION_TIME, 10, raw);
  if (rc < 0)
    return rc;

  raw[0] = s->sensor_sleep_enabled ? 1 : 0;
  raw[1] = CRC(raw[0]);
  raw[2] = s->sensor_sleep_sec / 10;
  raw[3] = CRC(raw[2]);
  raw[4] = s->sensor_model & 1;
  raw[5] = CRC(raw[4]);
  rc = atk_write(device, ATK_ADDR_SENSOR_ENABLE, 6, raw);
  if (rc < 0)
    return rc;

  raw[0] = s->angle_value;
  raw[1] = CRC(raw[0]);
  raw[2] = s->angle_enabled ? 1 : 0;
  raw[3] = CRC(raw[2]);
  rc = atk_write(device, ATK_ADDR_ANGLE, 4, raw);
  if (rc < 0)
    return rc;

  rc = atk_write_u8(device, ATK_ADDR_ROLLING_DELAY, s->rolling_delay);
  if (rc < 0)
    return rc;
  return atk_write_u8(device, ATK_ADDR_SILENT_HEIGHT, s->lod);

#undef CRC
}

/* ================================================================== */
/* MACRO READ/WRITE                                                    */
/* ================================================================== */

static int atk_read_macro(struct ratbag_device *device, int slot,
                          struct atk_macro *out) {
  uint16_t base = ATK_ADDR_MACRO0 + slot * ATK_MACRO_SLOT_SIZE;
  uint8_t header[10];
  int rc;

  if (slot < 0 || slot > 15)
    return -EINVAL;
  memset(out, 0, sizeof(*out));

  rc = atk_read(device, base, 10, header);
  if (rc < 0)
    return rc;

  out->count = header[0];
  if (out->count == 0 || out->count > ATK_MACRO_MAX_ACTIONS)
    return 0;

  for (int i = 0; i < (out->count + 1) / 2; i++) {
    uint8_t buf[10];
    uint16_t addr = base + 10 + i * 10;

    rc = atk_read(device, addr, 10, buf);
    if (rc < 0)
      return rc;

    for (int j = 0; j < 2; j++) {
      int idx = i * 2 + j;
      if (idx >= out->count)
        break;
      struct atk_macro_action *a = &out->actions[idx];
      a->flags = buf[j * 5 + 0];
      a->value1 = buf[j * 5 + 1];
      a->value2 = buf[j * 5 + 2];
      a->delay_ms = buf[j * 5 + 3] | (buf[j * 5 + 4] << 8);
    }
  }
  return 0;
}

static int atk_write_macro(struct ratbag_device *device, int slot,
                           const struct atk_macro *m) {
  uint16_t base = ATK_ADDR_MACRO0 + slot * ATK_MACRO_SLOT_SIZE;
  uint8_t header[10] = {0};
  int rc;

  if (slot < 0 || slot > 15)
    return -EINVAL;
  if (m->count > ATK_MACRO_MAX_ACTIONS)
    return -EINVAL;

  header[0] = m->count;
  rc = atk_write(device, base, 10, header);
  if (rc < 0)
    return rc;

  for (int i = 0; i < (m->count + 1) / 2; i++) {
    uint8_t buf[10] = {0};
    uint16_t addr = base + 10 + i * 10;

    for (int j = 0; j < 2; j++) {
      int idx = i * 2 + j;
      if (idx >= m->count)
        break;
      const struct atk_macro_action *a = &m->actions[idx];
      buf[j * 5 + 0] = a->flags;
      buf[j * 5 + 1] = a->value1;
      buf[j * 5 + 2] = a->value2;
      buf[j * 5 + 3] = a->delay_ms & 0xff;
      buf[j * 5 + 4] = (a->delay_ms >> 8) & 0xff;
    }

    rc = atk_write(device, addr, 10, buf);
    if (rc < 0)
      return rc;
  }
  return 0;
}

static int atk_erase_macro(struct ratbag_device *device, int slot) {
  struct atk_macro empty = {.count = 0};
  return atk_write_macro(device, slot, &empty);
}

/* ================================================================== */
/* PER-STAGE DPI COLOR                                                 */
/* ================================================================== */

int atk_read_dpi_color(struct ratbag_device *device, unsigned int stage,
                       uint8_t *r, uint8_t *g, uint8_t *b) {
  uint8_t rec[4];
  int rc;

  if (stage >= ATK_NUM_DPI_STAGES)
    return -EINVAL;
  rc = atk_read(device, atk_color_addr(stage), 4, rec);
  if (rc < 0)
    return rc;
  *r = rec[0];
  *g = rec[1];
  *b = rec[2];
  return 0;
}

int atk_write_dpi_color(struct ratbag_device *device, unsigned int stage,
                        uint8_t r, uint8_t g, uint8_t b) {
  uint8_t rec[4];
  unsigned int s;

  if (stage >= ATK_NUM_DPI_STAGES)
    return -EINVAL;
  rec[0] = r;
  rec[1] = g;
  rec[2] = b;
  s = r + g + b;
  rec[3] = (ATK_CHECKSUM_MAGIC - (s & 0xff)) & 0xff;
  return atk_write(device, atk_color_addr(stage), 4, rec);
}

/* ================================================================== */
/* BATTERY + FACTORY RESET                                            */
/* ================================================================== */

int atk_read_battery(struct ratbag_device *device, uint8_t *level,
                     bool *charging) {
  uint8_t tx[ATK_FRAME_SIZE], rx[ATK_FRAME_SIZE];
  int rc;

  atk_frame_init(tx, ATK_CMD_GET_BATTERY, 0, 0);
  rc = atk_xfer(device, tx, rx);
  if (rc < 0)
    return rc;
  *level = rx[P];
  *charging = rx[P + 1] != 0;
  return 0;
}

int atk_factory_reset(struct ratbag_device *device) {
  uint8_t tx[ATK_FRAME_SIZE], rx[ATK_FRAME_SIZE];
  atk_frame_init(tx, ATK_CMD_RESTORE_FACTORY, 0, 0);
  return atk_xfer(device, tx, rx);
}

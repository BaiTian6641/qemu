/*
 * ESP32-S3 RTC IO MUX
 *
 * 22 RTC GPIO pads with W1TS/W1TC atomic operations.
 * Register layout follows ESP32-S3 TRM v1.2 Chapter 6/39.
 *
 * Copyright (c) 2024-2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"

#define TYPE_ESP32S3_RTC_IO "esp32s3.rtc_io"
#define ESP32S3_RTC_IO(obj) OBJECT_CHECK(ESP32S3RtcIoState, (obj), TYPE_ESP32S3_RTC_IO)

#define ESP32S3_RTC_IO_REG_SIZE     0x200
#define ESP32S3_RTC_IO_GPIO_COUNT   22

/* ------------------------------------------------------------------ */
/*  Register offsets                                                   */
/* ------------------------------------------------------------------ */

#define RTC_IO_OUT_OFF            0x00
#define RTC_IO_OUT_W1TS_OFF       0x04
#define RTC_IO_OUT_W1TC_OFF       0x08
#define RTC_IO_ENABLE_OFF         0x0C
#define RTC_IO_ENABLE_W1TS_OFF    0x10
#define RTC_IO_ENABLE_W1TC_OFF    0x14
#define RTC_IO_STATUS_OFF         0x18
#define RTC_IO_STATUS_W1TS_OFF    0x1C
#define RTC_IO_STATUS_W1TC_OFF    0x20
#define RTC_IO_IN_OFF             0x24

/* GPIO pin config: 22 registers at 0x28..0x7C (stride 4) */
#define RTC_IO_PIN_OFF(n)         (0x28 + (n) * 0x04)
#define RTC_IO_PIN_LAST_OFF       (0x28 + 21 * 0x04)  /* 0x7C */

/* Per-pad config: 22 registers at 0x80..0xD8 (stride 4) */
#define RTC_IO_PAD_OFF(n)         (0x80 + (n) * 0x04)
#define RTC_IO_PAD_LAST_OFF       (0x80 + 21 * 0x04)  /* 0xD4 */

/* Misc registers */
#define RTC_IO_EXT_WAKEUP0_OFF   0xDC
#define RTC_IO_XTL_EXT_CTR_OFF   0xE0
#define RTC_IO_SAR_I2C_IO_OFF    0xE4
#define RTC_IO_TOUCH_CTRL_OFF    0xE8

/* Version */
#define RTC_IO_DATE_OFF           0x1FC

/* ------------------------------------------------------------------ */
/*  Device state                                                       */
/* ------------------------------------------------------------------ */

typedef struct ESP32S3RtcIoState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    /* GPIO output / enable / status / input */
    uint32_t out;
    uint32_t enable;
    uint32_t status;
    uint32_t in;

    /* Per-pin config (22 pins) */
    uint32_t pin[ESP32S3_RTC_IO_GPIO_COUNT];

    /* Per-pad config (22 pads) */
    uint32_t pad[ESP32S3_RTC_IO_GPIO_COUNT];

    /* Misc */
    uint32_t ext_wakeup0;
    uint32_t xtl_ext_ctr;
    uint32_t sar_i2c_io;
    uint32_t touch_ctrl;
} ESP32S3RtcIoState;

typedef struct ESP32S3RtcIoClass {
    SysBusDeviceClass parent_class;
} ESP32S3RtcIoClass;

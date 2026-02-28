/*
 * ESP32-S3 REGI2C analog bus stub
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"

#define TYPE_ESP32S3_REGI2C "esp32s3.regi2c"
#define ESP32S3_REGI2C(obj)           OBJECT_CHECK(ESP32S3RegI2CState, (obj), TYPE_ESP32S3_REGI2C)
#define ESP32S3_REGI2C_GET_CLASS(obj) OBJECT_GET_CLASS(ESP32S3RegI2CClass, obj, TYPE_ESP32S3_REGI2C)
#define ESP32S3_REGI2C_CLASS(klass)   OBJECT_CLASS_CHECK(ESP32S3RegI2CClass, klass, TYPE_ESP32S3_REGI2C)

/*
 * The REGI2C / I2C_MST analog config registers occupy a small window
 * starting at 0x6000E040 within the broader I2C_ANA_MST peripheral.
 * We model a 0x100-byte region at the base 0x6000E000 to absorb any
 * nearby accesses from the ROM or ESP-IDF analog calibration code.
 */
#define ESP32S3_REGI2C_BASE         0x6000E000
#define ESP32S3_REGI2C_REG_SIZE     0x100

/* Key register offsets (relative to base 0x6000E000) */
#define A_REGI2C_ANA_CONF0          0x040   /* I2C_MST_ANA_CONF0 */
#define A_REGI2C_ANA_CONFIG         0x044   /* ANA_CONFIG_REG */
#define A_REGI2C_ANA_CONFIG2        0x048   /* ANA_CONFIG2_REG */

/* Bit definitions for ANA_CONF0 */
#define REGI2C_BBPLL_STOP_FORCE_HIGH    (1 << 2)
#define REGI2C_BBPLL_STOP_FORCE_LOW     (1 << 3)
#define REGI2C_BBPLL_CAL_DONE           (1 << 24)

typedef struct ESP32S3RegI2CState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    /* Modeled registers */
    uint32_t ana_conf0;     /* 0x040 */
    uint32_t ana_config;    /* 0x044 */
    uint32_t ana_config2;   /* 0x048 */

    /* Catch-all backing for any other offset in the window */
    uint32_t regs[ESP32S3_REGI2C_REG_SIZE / 4];
} ESP32S3RegI2CState;

typedef struct ESP32S3RegI2CClass {
    SysBusDeviceClass parent_class;
} ESP32S3RegI2CClass;

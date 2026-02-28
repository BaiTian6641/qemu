/*
 * ESP32-S3 SENS (Analog Sensor) Controller (stub)
 *
 * Copyright (c) 2024-2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * Provides a register-level compatibility model at DR_REG_SENS_BASE
 * (0x60008800).  Covers SAR ADC measurement control, touch sensor
 * status/configuration, temperature sensor, and ULP co-processor
 * interrupt/state.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#pragma once

#include "hw/sysbus.h"

#define TYPE_ESP32S3_SENS  "esp32s3.sens"
#define ESP32S3_SENS(obj)  OBJECT_CHECK(ESP32S3SensState, (obj), \
                                        TYPE_ESP32S3_SENS)

/* ------------------------------------------------------------------ */
/*  Register offsets  (from sens_reg.h)                                */
/* ------------------------------------------------------------------ */
#define SENS_REG_SAR_READER1_CTRL       0x00
#define SENS_REG_SAR_READER1_STATUS     0x04
#define SENS_REG_SAR_MEAS1_CTRL1        0x08
#define SENS_REG_SAR_MEAS1_CTRL2        0x0C
#define SENS_REG_SAR_MEAS1_MUX          0x10
#define SENS_REG_SAR_ATTEN1             0x14
#define SENS_REG_SAR_AMP_CTRL1          0x18
#define SENS_REG_SAR_AMP_CTRL2          0x1C
#define SENS_REG_SAR_AMP_CTRL3          0x20
#define SENS_REG_SAR_READER2_CTRL       0x24
#define SENS_REG_SAR_READER2_STATUS     0x28
#define SENS_REG_SAR_MEAS2_CTRL1        0x2C
#define SENS_REG_SAR_MEAS2_CTRL2        0x30
#define SENS_REG_SAR_MEAS2_MUX          0x34
#define SENS_REG_SAR_ATTEN2             0x38
#define SENS_REG_SAR_POWER_XPD_SAR      0x3C
#define SENS_REG_SAR_SLAVE_ADDR1        0x40
#define SENS_REG_SAR_SLAVE_ADDR2        0x44
#define SENS_REG_SAR_SLAVE_ADDR3        0x48
#define SENS_REG_SAR_SLAVE_ADDR4        0x4C
#define SENS_REG_SAR_TSENS_CTRL         0x50
#define SENS_REG_SAR_TSENS_CTRL2        0x54
#define SENS_REG_SAR_I2C_CTRL           0x58
#define SENS_REG_SAR_TOUCH_CONF         0x5C
#define SENS_REG_SAR_TOUCH_DENOISE      0x60
/* TOUCH_THRES 1..14 at 0x64..0x98 */
#define SENS_REG_SAR_TOUCH_THRES_BASE   0x64
#define SENS_REG_SAR_TOUCH_CHN_ST       0x9C
/* TOUCH_STATUS 0..14 at 0xA0..0xD8 */
#define SENS_REG_SAR_TOUCH_STATUS_BASE  0xA0
#define SENS_REG_SAR_TOUCH_SLP_STATUS   0xDC
#define SENS_REG_SAR_TOUCH_APPR_STATUS  0xE0

/* ULP Co-processor registers (inside SENS block) */
#define SENS_REG_SAR_COCPU_STATE        0xE4
#define SENS_REG_SAR_COCPU_INT_RAW      0xE8
#define SENS_REG_SAR_COCPU_INT_ENA      0xEC
#define SENS_REG_SAR_COCPU_INT_ST       0xF0
#define SENS_REG_SAR_COCPU_INT_CLR      0xF4
#define SENS_REG_SAR_COCPU_DEBUG        0xF8
#define SENS_REG_SAR_HALL_CTRL          0xFC
#define SENS_REG_SAR_NOUSE              0x100
#define SENS_REG_SAR_PERI_CLK_GATE_CONF 0x104
#define SENS_REG_SAR_PERI_RESET_CONF    0x108
#define SENS_REG_COCPU_INT_ENA_W1TS     0x10C
#define SENS_REG_COCPU_INT_ENA_W1TC     0x110
#define SENS_REG_SAR_DEBUG_CONF         0x114
#define SENS_REG_SARDATE                0x1FC

#define SENS_REG_SIZE                   0x200  /* 512 bytes */

#define SENS_REG_COUNT  (SENS_REG_SIZE / 4)

/* COCPU interrupt bits (10 sources, from sens_reg.h) */
#define SENS_COCPU_INT_TOUCH_DONE       BIT(0)
#define SENS_COCPU_INT_TOUCH_INACTIVE   BIT(1)
#define SENS_COCPU_INT_TOUCH_ACTIVE     BIT(2)
#define SENS_COCPU_INT_SARADC1          BIT(3)
#define SENS_COCPU_INT_SARADC2          BIT(4)
#define SENS_COCPU_INT_TSENS            BIT(5)
#define SENS_COCPU_INT_START            BIT(6)
#define SENS_COCPU_INT_SW               BIT(7)
#define SENS_COCPU_INT_SWD              BIT(8)
#define SENS_COCPU_INT_SUPER_WDT        BIT(9)

/* Temperature sensor default: out_offset=0, power-off */
#define SENS_TSENS_CTRL_DEFAULT         0x00000000
/* SAR DATE register default */
#define SENS_DATE_DEFAULT               0x02003020

typedef struct {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[SENS_REG_COUNT];

    /* COCPU interrupt model */
    uint32_t cocpu_int_raw;
    uint32_t cocpu_int_ena;

    /* No dedicated IRQ for SENS — ULP events route via RTC interrupt */
} ESP32S3SensState;

/*
 * ESP32-S3 APB SAR ADC Controller (stub)
 *
 * Copyright (c) 2024-2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * Provides a register-level compatibility model at DR_REG_APB_SARADC_BASE
 * (0x60040000).  Enough for ESP-IDF ADC drivers to initialise without
 * hitting unimplemented-access warnings.  ADC conversion results return
 * deterministic mid-scale values.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#pragma once

#include "hw/sysbus.h"

#define TYPE_ESP32S3_APB_SARADC  "esp32s3.apb_saradc"
#define ESP32S3_APB_SARADC(obj)  OBJECT_CHECK(ESP32S3ApbSaradcState, (obj), \
                                              TYPE_ESP32S3_APB_SARADC)

/* ------------------------------------------------------------------ */
/*  Register offsets  (from apb_saradc_reg.h)                          */
/* ------------------------------------------------------------------ */
#define APB_SARADC_REG_CTRL             0x00
#define APB_SARADC_REG_CTRL2            0x04
#define APB_SARADC_REG_FILTER_CTRL1     0x08
#define APB_SARADC_REG_FSM_WAIT         0x0C
#define APB_SARADC_REG_SAR1_STATUS      0x10
#define APB_SARADC_REG_SAR2_STATUS      0x14
#define APB_SARADC_REG_SAR1_PATT_TAB1   0x18
#define APB_SARADC_REG_SAR1_PATT_TAB2   0x1C
#define APB_SARADC_REG_SAR1_PATT_TAB3   0x20
#define APB_SARADC_REG_SAR1_PATT_TAB4   0x24
#define APB_SARADC_REG_SAR2_PATT_TAB1   0x28
#define APB_SARADC_REG_SAR2_PATT_TAB2   0x2C
#define APB_SARADC_REG_SAR2_PATT_TAB3   0x30
#define APB_SARADC_REG_SAR2_PATT_TAB4   0x34
#define APB_SARADC_REG_ARB_CTRL         0x38
#define APB_SARADC_REG_FILTER_CTRL0     0x3C
#define APB_SARADC_REG_SAR1_DATA_STATUS 0x40
#define APB_SARADC_REG_THRES0_CTRL      0x44
#define APB_SARADC_REG_THRES1_CTRL      0x48
#define APB_SARADC_REG_THRES_CTRL       0x58
#define APB_SARADC_REG_INT_ENA          0x5C
#define APB_SARADC_REG_INT_RAW          0x60
#define APB_SARADC_REG_INT_ST           0x64
#define APB_SARADC_REG_INT_CLR          0x68
#define APB_SARADC_REG_DMA_CONF         0x6C
#define APB_SARADC_REG_CLKM_CONF        0x70
#define APB_SARADC_REG_SAR2_DATA_STATUS 0x78
#define APB_SARADC_REG_DATE             0x3FC

#define APB_SARADC_REG_SIZE             0x400

/* Number of 32-bit registers we store as R/W */
#define APB_SARADC_REG_COUNT  (APB_SARADC_REG_SIZE / 4)

/* Interrupt bits (from apb_saradc_reg.h):
 *   bit 0: THRES1_LOW
 *   bit 1: THRES0_LOW
 *   bit 2: THRES1_HIGH
 *   bit 3: THRES0_HIGH
 *   bit 4: APB_SARADC2_DONE
 *   bit 5: APB_SARADC1_DONE
 */
#define APB_SARADC_INT_THRES1_LOW   BIT(0)
#define APB_SARADC_INT_THRES0_LOW   BIT(1)
#define APB_SARADC_INT_THRES1_HIGH  BIT(2)
#define APB_SARADC_INT_THRES0_HIGH  BIT(3)
#define APB_SARADC_INT_SAR2_DONE    BIT(4)
#define APB_SARADC_INT_SAR1_DONE    BIT(5)

/* Default DATE register value */
#define APB_SARADC_DATE_DEFAULT     0x02003020

/* Mid-scale ADC reading for 12-bit conversion (deterministic stub) */
#define APB_SARADC_MIDSCALE_12BIT   0x800

typedef struct {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t regs[APB_SARADC_REG_COUNT];

    /* Interrupt model */
    uint32_t int_raw;
    uint32_t int_ena;
    qemu_irq irq;
} ESP32S3ApbSaradcState;

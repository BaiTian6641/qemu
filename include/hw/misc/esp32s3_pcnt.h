/*
 * ESP32-S3 Pulse Count Controller (PCNT)
 *
 * 4 counter units, each with 2 channels (comparators).
 * Register layout follows ESP32-S3 TRM v1.2 Chapter 30.
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

#define TYPE_ESP32S3_PCNT "esp32s3.pcnt"
#define ESP32S3_PCNT(obj) OBJECT_CHECK(ESP32S3PCNTState, (obj), TYPE_ESP32S3_PCNT)

#define ESP32S3_PCNT_REG_SIZE   0x100
#define ESP32S3_PCNT_UNIT_COUNT 4

/* ------------------------------------------------------------------ */
/*  Register offsets                                                   */
/* ------------------------------------------------------------------ */

/*
 * Per-unit config: 3 registers per unit, stride 0x0C
 *   Un_CONF0 = 0x00 + n*0x0C
 *   Un_CONF1 = 0x04 + n*0x0C
 *   Un_CONF2 = 0x08 + n*0x0C
 */
#define ESP32S3_PCNT_Un_CONF0_OFF(n)  (0x00 + (n) * 0x0C)
#define ESP32S3_PCNT_Un_CONF1_OFF(n)  (0x04 + (n) * 0x0C)
#define ESP32S3_PCNT_Un_CONF2_OFF(n)  (0x08 + (n) * 0x0C)

/* Counter value registers (read-only) */
#define ESP32S3_PCNT_Un_CNT_OFF(n)    (0x30 + (n) * 0x04)

/* Interrupt registers */
#define ESP32S3_PCNT_INT_RAW_OFF   0x40
#define ESP32S3_PCNT_INT_ST_OFF    0x44
#define ESP32S3_PCNT_INT_ENA_OFF   0x48
#define ESP32S3_PCNT_INT_CLR_OFF   0x4C

/* Status registers (read-only) */
#define ESP32S3_PCNT_Un_STATUS_OFF(n)  (0x50 + (n) * 0x04)

/* Global control */
#define ESP32S3_PCNT_CTRL_OFF       0x60

/* Version */
#define ESP32S3_PCNT_DATE_OFF       0xFC

/* INT_RAW bits: one bit per unit */
#define PCNT_S3_INT_UNIT(n)        (1 << (n))

/* CTRL register bits */
#define PCNT_S3_CTRL_CNT_RST(n)    (1 << ((n) * 2))
#define PCNT_S3_CTRL_CNT_PAUSE(n)  (1 << ((n) * 2 + 1))
#define PCNT_S3_CTRL_CLK_EN        (1 << 16)

/* CONF0 fields (per channel within a unit) */
#define PCNT_S3_CONF0_FILTER_THRES_SHIFT 0
#define PCNT_S3_CONF0_FILTER_THRES_MASK  0x3FF
#define PCNT_S3_CONF0_FILTER_EN          (1 << 10)
#define PCNT_S3_CONF0_THR_ZERO_EN        (1 << 11)
#define PCNT_S3_CONF0_THR_H_LIM_EN       (1 << 12)
#define PCNT_S3_CONF0_THR_L_LIM_EN       (1 << 13)
#define PCNT_S3_CONF0_THR_THRES0_EN      (1 << 14)
#define PCNT_S3_CONF0_THR_THRES1_EN      (1 << 15)
/* ch0 control mode fields at bits [16:31], ch1 at CONF1 */

/* STATUS register bits */
#define PCNT_S3_STATUS_THR_THRES1_LAT (1 << 0)
#define PCNT_S3_STATUS_THR_THRES0_LAT (1 << 1)
#define PCNT_S3_STATUS_L_LIM_LAT      (1 << 2)
#define PCNT_S3_STATUS_H_LIM_LAT      (1 << 3)
#define PCNT_S3_STATUS_ZERO_LAT       (1 << 4)

/* ------------------------------------------------------------------ */
/*  Device state                                                       */
/* ------------------------------------------------------------------ */

typedef struct ESP32S3PCNTState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    /* Per-unit configuration registers */
    uint32_t unit_conf0[ESP32S3_PCNT_UNIT_COUNT];
    uint32_t unit_conf1[ESP32S3_PCNT_UNIT_COUNT];
    uint32_t unit_conf2[ESP32S3_PCNT_UNIT_COUNT];

    /* Per-unit counter values */
    int16_t  unit_cnt[ESP32S3_PCNT_UNIT_COUNT];

    /* Per-unit status */
    uint32_t unit_status[ESP32S3_PCNT_UNIT_COUNT];

    /* Interrupt registers */
    uint32_t int_raw;
    uint32_t int_ena;

    /* Global control */
    uint32_t ctrl;
} ESP32S3PCNTState;

typedef struct ESP32S3PCNTClass {
    SysBusDeviceClass parent_class;
} ESP32S3PCNTClass;

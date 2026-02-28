/*
 * ESP32-S3 LED PWM Controller (LEDC)
 *
 * Low-speed only: 8 channels, 4 timers.
 * Register layout follows ESP32-S3 TRM v1.2 Chapter 35.
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

#define TYPE_ESP32S3_LEDC "esp32s3.ledc"
#define ESP32S3_LEDC(obj) OBJECT_CHECK(ESP32S3LEDCState, (obj), TYPE_ESP32S3_LEDC)

#define ESP32S3_LEDC_REG_SIZE      0x100
#define ESP32S3_LEDC_CHANNEL_COUNT 8
#define ESP32S3_LEDC_TIMER_COUNT   4

/* ------------------------------------------------------------------ */
/*  Register offsets (ESP32-S3 low-speed only layout)                  */
/* ------------------------------------------------------------------ */

/*
 * Channel registers: 8 channels, stride 0x14
 *   CHn_CONF0  = 0x00 + n*0x14
 *   CHn_HPOINT = 0x04 + n*0x14
 *   CHn_DUTY   = 0x08 + n*0x14
 *   CHn_CONF1  = 0x0C + n*0x14
 *   CHn_DUTY_R = 0x10 + n*0x14
 */
#define ESP32S3_LEDC_CH_CONF0_OFF(n)   (0x00 + (n) * 0x14)
#define ESP32S3_LEDC_CH_HPOINT_OFF(n)  (0x04 + (n) * 0x14)
#define ESP32S3_LEDC_CH_DUTY_OFF(n)    (0x08 + (n) * 0x14)
#define ESP32S3_LEDC_CH_CONF1_OFF(n)   (0x0C + (n) * 0x14)
#define ESP32S3_LEDC_CH_DUTY_R_OFF(n)  (0x10 + (n) * 0x14)
/* Last channel register: CH7_DUTY_R = 0x10 + 7*0x14 = 0x9C */
#define ESP32S3_LEDC_CH_REG_END        0xA0

/*
 * Timer registers: 4 timers, stride 0x08
 *   TIMERn_CONF  = 0xA0 + n*0x08
 *   TIMERn_VALUE = 0xA4 + n*0x08
 */
#define ESP32S3_LEDC_TIMER_CONF_OFF(n)  (0xA0 + (n) * 0x08)
#define ESP32S3_LEDC_TIMER_VALUE_OFF(n) (0xA4 + (n) * 0x08)
/* Last timer register: TIMER3_VALUE = 0xA4 + 3*0x08 = 0xBC */

/* Interrupt registers */
#define ESP32S3_LEDC_INT_RAW_OFF  0xC0
#define ESP32S3_LEDC_INT_ST_OFF   0xC4
#define ESP32S3_LEDC_INT_ENA_OFF  0xC8
#define ESP32S3_LEDC_INT_CLR_OFF  0xCC

/* Global config */
#define ESP32S3_LEDC_CONF_OFF     0xD0
#define ESP32S3_LEDC_DATE_OFF     0xFC

/* INT_RAW bits (20 sources):
 *   bits [3:0]   — timer 0-3 overflow
 *   bits [11:4]  — channel 0-7 duty change done
 *   bits [19:12] — channel 0-7 overflow count pulse
 */
#define LEDC_S3_INT_TIMER_OVF(n)     (1 << (n))
#define LEDC_S3_INT_DUTY_CHNG(n)     (1 << (4 + (n)))
#define LEDC_S3_INT_OVF_CNT(n)       (1 << (12 + (n)))

/* CH_CONF0 fields */
#define LEDC_S3_CONF0_TIMER_SEL_SHIFT 0
#define LEDC_S3_CONF0_TIMER_SEL_MASK  0x3
#define LEDC_S3_CONF0_SIG_OUT_EN      (1 << 2)
#define LEDC_S3_CONF0_IDLE_LV         (1 << 3)
#define LEDC_S3_CONF0_PARA_UP         (1 << 4)
#define LEDC_S3_CONF0_OVF_NUM_SHIFT   5
#define LEDC_S3_CONF0_OVF_CNT_EN      (1 << 15)
#define LEDC_S3_CONF0_OVF_CNT_RESET   (1 << 16)

/* TIMER_CONF fields */
#define LEDC_S3_TIMER_DUTY_RES_SHIFT  0
#define LEDC_S3_TIMER_DUTY_RES_MASK   0xF
#define LEDC_S3_TIMER_CLK_DIV_SHIFT   4
#define LEDC_S3_TIMER_CLK_DIV_MASK    (0x3FFFF << 4)
#define LEDC_S3_TIMER_PAUSE           (1 << 22)
#define LEDC_S3_TIMER_RST             (1 << 23)
#define LEDC_S3_TIMER_TICK_SEL        (1 << 24)
#define LEDC_S3_TIMER_PARA_UP         (1 << 25)

/* CONF (global) fields */
#define LEDC_S3_CONF_APB_CLK_SEL_SHIFT 0
#define LEDC_S3_CONF_APB_CLK_SEL_MASK  0x3
#define LEDC_S3_CONF_CLK_EN           (1U << 31)

/* ------------------------------------------------------------------ */
/*  Device state                                                       */
/* ------------------------------------------------------------------ */

typedef struct ESP32S3LEDCState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    /* Per-channel registers */
    uint32_t ch_conf0[ESP32S3_LEDC_CHANNEL_COUNT];
    uint32_t ch_hpoint[ESP32S3_LEDC_CHANNEL_COUNT];
    uint32_t ch_duty[ESP32S3_LEDC_CHANNEL_COUNT];
    uint32_t ch_conf1[ESP32S3_LEDC_CHANNEL_COUNT];
    uint32_t ch_duty_r[ESP32S3_LEDC_CHANNEL_COUNT];  /* read-only shadow */

    /* Per-timer registers */
    uint32_t timer_conf[ESP32S3_LEDC_TIMER_COUNT];
    uint32_t timer_value[ESP32S3_LEDC_TIMER_COUNT];

    /* Interrupt registers */
    uint32_t int_raw;
    uint32_t int_ena;

    /* Global config */
    uint32_t conf;
} ESP32S3LEDCState;

typedef struct ESP32S3LEDCClass {
    SysBusDeviceClass parent_class;
} ESP32S3LEDCClass;

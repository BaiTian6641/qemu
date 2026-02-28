/*
 * ESP32-S3 Clocks definition
 *
 * Copyright (c) 2023 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#pragma once

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"

#define TYPE_ESP32S3_CLOCK "esp32s3.soc.clk"
#define ESP32S3_CLOCK(obj) OBJECT_CHECK(ESP32S3ClockState, (obj), TYPE_ESP32S3_CLOCK)
#define ESP32S3_CLOCK_GET_CLASS(obj) OBJECT_GET_CLASS(ESP32S3ClockClass, obj, TYPE_ESP32S3_CLOCK)
#define ESP32S3_CLOCK_CLASS(klass) OBJECT_CLASS_CHECK(ESP32S3ClockClass, klass, TYPE_ESP32S3_CLOCK)


#define ESP32S3_SYSTEM_CPU_INTR_COUNT   4

/**
 * Value for SYSTEM_SOC_CLK_SEL
 */
#define ESP32S3_CLK_SEL_XTAL    0
#define ESP32S3_CLK_SEL_PLL     1
#define ESP32S3_CLK_SEL_RCFAST  2

/**
 * Values for SYSTEM_PLL_FREQ_SEL
 */
#define ESP32S3_FREQ_SEL_PLL_480    0
#define ESP32S3_FREQ_SEL_PLL_320    1

/**
 * Values for SYSTEM_CPUPERIOD_SEL
*/
#define ESP32S3_PERIOD_SEL_80       0
#define ESP32S3_PERIOD_SEL_160      1


typedef struct ESP32S3ClockState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    /* Core 1 control */
    uint32_t core1_ctrl0;           /* 0x000: CORE_1_CONTROL_0 (runstall) */
    uint32_t app_cpu_addr;          /* 0x004: CORE_1_CONTROL_1 (boot addr) */

    /* CPU peripheral clocking */
    uint32_t cpu_peri_clk_en;       /* 0x008 */
    uint32_t cpu_peri_rst_en;       /* 0x00C */

    /* Registers for clocks configuration and frequency dividers */
    uint32_t cpuperconf;            /* 0x010 */
    uint32_t mem_pd_mask;           /* 0x014 */

    /* Peripheral clock / reset gate registers */
    uint32_t perip_clk_en0;         /* 0x018 */
    uint32_t perip_clk_en1;         /* 0x01C */
    uint32_t perip_rst_en0;         /* 0x020 */
    uint32_t perip_rst_en1;         /* 0x024 */

    /* BT low-power clock divider */
    uint32_t bt_lpck_div_int;       /* 0x028 */
    uint32_t bt_lpck_div_frac;      /* 0x02C */

    /* IRQs for crosscore interrupts (0x030–0x03C) */
    qemu_irq irqs[ESP32S3_SYSTEM_CPU_INTR_COUNT];
    uint32_t levels;  /* Bitmap that keeps the level of the IRQs */

    /* Power / crypto control */
    uint32_t rsa_pd_ctrl;           /* 0x040 */
    uint32_t edma_ctrl;             /* 0x044 */
    uint32_t cache_control;         /* 0x048 */

    uint32_t sys_ext_dev_enc_dec_ctrl; /* 0x04C */

    /* RTC fast memory CRC */
    uint32_t rtc_fastmem_config;    /* 0x050 */
    uint32_t rtc_fastmem_crc;       /* 0x054 */

    /* ECO / clock gate */
    uint32_t redundant_eco_ctrl;    /* 0x058 */
    uint32_t clock_gate;            /* 0x05C */

    /* System clock configuration */
    uint32_t sysclk;                /* 0x060 */

    /* DATE register */
    uint32_t system_date;           /* 0xFFC */
} ESP32S3ClockState;

typedef struct ESP32S3ClockClass {
    SysBusDeviceClass parent_class;
    /* Virtual methods */
    uint32_t (*get_ext_dev_enc_dec_ctrl)(ESP32S3ClockState *s);
} ESP32S3ClockClass;


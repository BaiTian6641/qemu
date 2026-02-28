/*
 * ESP32-S3 SYSCON (APB_CTRL) register model
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

#define TYPE_ESP32S3_SYSCON "esp32s3.syscon"
#define ESP32S3_SYSCON(obj)           OBJECT_CHECK(ESP32S3SysconState, (obj), TYPE_ESP32S3_SYSCON)
#define ESP32S3_SYSCON_GET_CLASS(obj) OBJECT_GET_CLASS(ESP32S3SysconClass, obj, TYPE_ESP32S3_SYSCON)
#define ESP32S3_SYSCON_CLASS(klass)   OBJECT_CLASS_CHECK(ESP32S3SysconClass, klass, TYPE_ESP32S3_SYSCON)

/* Register space size: 0x400 bytes (offsets 0x000 – 0x3FC) */
#define ESP32S3_SYSCON_REG_SIZE     0x400

/* ---- Register offset definitions ---- */
#define A_SYSCON_SYSCLK_CONF               0x000
#define A_SYSCON_TICK_CONF                  0x004
#define A_SYSCON_CLK_OUT_EN                 0x008
#define A_SYSCON_WIFI_BB_CFG                0x00C
#define A_SYSCON_WIFI_BB_CFG_2              0x010
#define A_SYSCON_WIFI_CLK_EN                0x014
#define A_SYSCON_WIFI_RST_EN                0x018
#define A_SYSCON_HOST_INF_SEL               0x01C
#define A_SYSCON_EXT_MEM_PMS_LOCK           0x020
#define A_SYSCON_EXT_MEM_WRITEBACK_BYPASS   0x024
#define A_SYSCON_FLASH_ACE0_ATTR            0x028
#define A_SYSCON_FLASH_ACE1_ATTR            0x02C
#define A_SYSCON_FLASH_ACE2_ATTR            0x030
#define A_SYSCON_FLASH_ACE3_ATTR            0x034
#define A_SYSCON_FLASH_ACE0_ADDR            0x038
#define A_SYSCON_FLASH_ACE1_ADDR            0x03C
#define A_SYSCON_FLASH_ACE2_ADDR            0x040
#define A_SYSCON_FLASH_ACE3_ADDR            0x044
#define A_SYSCON_FLASH_ACE0_SIZE            0x048
#define A_SYSCON_FLASH_ACE1_SIZE            0x04C
#define A_SYSCON_FLASH_ACE2_SIZE            0x050
#define A_SYSCON_FLASH_ACE3_SIZE            0x054
#define A_SYSCON_SRAM_ACE0_ATTR             0x058
#define A_SYSCON_SRAM_ACE1_ATTR             0x05C
#define A_SYSCON_SRAM_ACE2_ATTR             0x060
#define A_SYSCON_SRAM_ACE3_ATTR             0x064
#define A_SYSCON_SRAM_ACE0_ADDR             0x068
#define A_SYSCON_SRAM_ACE1_ADDR             0x06C
#define A_SYSCON_SRAM_ACE2_ADDR             0x070
#define A_SYSCON_SRAM_ACE3_ADDR             0x074
#define A_SYSCON_SRAM_ACE0_SIZE             0x078
#define A_SYSCON_SRAM_ACE1_SIZE             0x07C
#define A_SYSCON_SRAM_ACE2_SIZE             0x080
#define A_SYSCON_SRAM_ACE3_SIZE             0x084
#define A_SYSCON_SPI_MEM_PMS_CTRL           0x088
#define A_SYSCON_SPI_MEM_REJECT_ADDR        0x08C
#define A_SYSCON_SDIO_CTRL                  0x090
#define A_SYSCON_REDCY_SIG0                 0x094
#define A_SYSCON_REDCY_SIG1                 0x098
#define A_SYSCON_FRONT_END_MEM_PD           0x09C
#define A_SYSCON_SPI_MEM_ECC_CTRL           0x0A0
/* 0x0A4 reserved */
#define A_SYSCON_CLKGATE_FORCE_ON           0x0A8
#define A_SYSCON_MEM_POWER_DOWN             0x0AC
#define A_SYSCON_MEM_POWER_UP               0x0B0
#define A_SYSCON_RETENTION_CTRL             0x0B4
#define A_SYSCON_RETENTION_CTRL1            0x0B8
#define A_SYSCON_RETENTION_CTRL2            0x0BC
#define A_SYSCON_RETENTION_CTRL3            0x0C0
#define A_SYSCON_RETENTION_CTRL4            0x0C4
#define A_SYSCON_RETENTION_CTRL5            0x0C8
#define A_SYSCON_DATE                       0x3FC

/* ---- Default values (from TRM / ESP-IDF) ---- */
#define SYSCON_WIFI_CLK_EN_DEFAULT          0xFFFCE030
#define SYSCON_WIFI_RST_EN_DEFAULT          0x00000000
#define SYSCON_DATE_DEFAULT                 0x02101150
#define SYSCON_CLK_OUT_EN_DEFAULT           0x000007FF
#define SYSCON_FRONT_END_MEM_PD_DEFAULT     0x00000055  /* all force_pu bits */
#define SYSCON_CLK_CONF_DEFAULT             0x00000001  /* pre_div_cnt = 1 */
#define SYSCON_TICK_CONF_DEFAULT            0x00010727  /* tick_enable=1, ck8m_tick=7, xtal_tick=39 */

/* Flash/SRAM ACE defaults */
#define SYSCON_ACE_ATTR_DEFAULT             0x000000FF
#define SYSCON_ACE_SIZE_DEFAULT             0x00001000
#define SYSCON_FLASH_ACE1_ADDR_DEFAULT      0x10000000
#define SYSCON_FLASH_ACE2_ADDR_DEFAULT      0x20000000
#define SYSCON_FLASH_ACE3_ADDR_DEFAULT      0x30000000
#define SYSCON_SRAM_ACE1_ADDR_DEFAULT       0x10000000
#define SYSCON_SRAM_ACE2_ADDR_DEFAULT       0x20000000
#define SYSCON_SRAM_ACE3_ADDR_DEFAULT       0x30000000

/* RGB_QEMU_ORIGIN signature register offset (custom QEMU extension) */
#ifndef RGB_QEMU_ORIGIN_REG
#define RGB_QEMU_ORIGIN_REG                 0x3F8
#endif

typedef struct ESP32S3SysconState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;

    /* Clock / reset configuration */
    uint32_t clk_conf;          /* 0x000 */
    uint32_t tick_conf;         /* 0x004 */
    uint32_t clk_out_en;        /* 0x008 */
    uint32_t wifi_bb_cfg;       /* 0x00C */
    uint32_t wifi_bb_cfg_2;     /* 0x010 */
    uint32_t wifi_clk_en;       /* 0x014 */
    uint32_t wifi_rst_en;       /* 0x018 */
    uint32_t host_inf_sel;      /* 0x01C */

    /* External memory PMS */
    uint32_t ext_mem_pms_lock;        /* 0x020 */
    uint32_t ext_mem_writeback_bypass;/* 0x024 */

    /* Flash ACE (access control) */
    uint32_t flash_ace_attr[4]; /* 0x028–0x034 */
    uint32_t flash_ace_addr[4]; /* 0x038–0x044 */
    uint32_t flash_ace_size[4]; /* 0x048–0x054 */

    /* SRAM ACE */
    uint32_t sram_ace_attr[4];  /* 0x058–0x064 */
    uint32_t sram_ace_addr[4];  /* 0x068–0x074 */
    uint32_t sram_ace_size[4];  /* 0x078–0x084 */

    /* Misc */
    uint32_t spi_mem_pms_ctrl;  /* 0x088 */
    uint32_t spi_mem_reject_addr;/* 0x08C (RO) */
    uint32_t sdio_ctrl;         /* 0x090 */
    uint32_t redcy_sig0;        /* 0x094 */
    uint32_t redcy_sig1;        /* 0x098 */
    uint32_t front_end_mem_pd;  /* 0x09C */
    uint32_t spi_mem_ecc_ctrl;  /* 0x0A0 */

    /* Memory power / retention */
    uint32_t clkgate_force_on;  /* 0x0A8 */
    uint32_t mem_power_down;    /* 0x0AC */
    uint32_t mem_power_up;      /* 0x0B0 */
    uint32_t retention_ctrl;    /* 0x0B4 */
    uint32_t retention_ctrl1;   /* 0x0B8 */
    uint32_t retention_ctrl2;   /* 0x0BC */
    uint32_t retention_ctrl3;   /* 0x0C0 */
    uint32_t retention_ctrl4;   /* 0x0C4 */
    uint32_t retention_ctrl5;   /* 0x0C8 */

    /* DATE / version register */
    uint32_t date;              /* 0x3FC */

    /* QEMU-specific: QEMU origin signature */
    uint32_t qemu_origin;      /* 0x3F8 (custom) */
} ESP32S3SysconState;

typedef struct ESP32S3SysconClass {
    SysBusDeviceClass parent_class;
} ESP32S3SysconClass;

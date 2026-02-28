/*
 * ESP32-S3 LCD_CAM Controller Model
 *
 * Copyright (c) 2024-2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * Refined model at 0x60041000.
 * Supports I80 (Intel 8080) and RGB LCD output modes via GDMA.
 * Implements all register bit-field semantics expected by the ESP-IDF
 * lcd_ll.h HAL layer (self-clearing bits, reset defaults, correct
 * bit positions for START/RESET/UPDATE/AFIFO_RESET).
 *
 * GDMA integration: on LCD_START, reads TX data through the GDMA
 * LCDCAM channel and fires LCD_TRANS_DONE.  Camera RX path is stub.
 *
 * Interrupt model: lcd_vsync, lcd_trans_done, cam_vsync, cam_hs (4 bits).
 * IRQ: ETS_LCD_CAM_INTR_SOURCE (24).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#pragma once

#include "hw/sysbus.h"

/* Forward-declare GDMA state so we can hold a pointer */
typedef struct ESPGdmaState ESPGdmaState;

#define TYPE_ESP32S3_LCD_CAM  "esp32s3.lcd_cam"
#define ESP32S3_LCD_CAM(obj)  OBJECT_CHECK(ESP32S3LcdCamState, (obj), TYPE_ESP32S3_LCD_CAM)

/* Register space: 0x100 (confirmed by TRM register map, last reg 0xFC) */
#define ESP32S3_LCD_CAM_REG_SIZE  0x100

/* ------------------------------------------------------------------ */
/* Register offsets (from DR_REG_LCD_CAM_BASE = 0x60041000)           */
/* ------------------------------------------------------------------ */
#define LCD_CAM_LCD_CLOCK_REG           0x000
#define LCD_CAM_CAM_CTRL_REG            0x004
#define LCD_CAM_CAM_CTRL1_REG           0x008
#define LCD_CAM_CAM_RGB_YUV_REG         0x00C
#define LCD_CAM_LCD_RGB_YUV_REG         0x010
#define LCD_CAM_LCD_USER_REG            0x014
#define LCD_CAM_LCD_MISC_REG            0x018
#define LCD_CAM_LCD_CTRL_REG            0x01C
#define LCD_CAM_LCD_CTRL1_REG           0x020
#define LCD_CAM_LCD_CTRL2_REG           0x024
#define LCD_CAM_LCD_CMD_VAL_REG         0x028
/* 0x02C is reserved */
#define LCD_CAM_LCD_DLY_MODE_REG        0x030
/* 0x034 is reserved */
#define LCD_CAM_LCD_DATA_DOUT_MODE_REG  0x038
/* 0x03C..0x060 reserved */
#define LCD_CAM_LC_DMA_INT_ENA_REG      0x064
#define LCD_CAM_LC_DMA_INT_RAW_REG      0x068
#define LCD_CAM_LC_DMA_INT_ST_REG       0x06C
#define LCD_CAM_LC_DMA_INT_CLR_REG      0x070
/* 0x074..0x0F8 reserved */
#define LCD_CAM_LC_REG_DATE_REG         0x0FC

/* ------------------------------------------------------------------ */
/* Bit definitions – LCD_USER_REG (0x14)                              */
/* Matching ESP-IDF soc/lcd_cam_reg.h for ESP32-S3                    */
/* ------------------------------------------------------------------ */
#define LCD_CAM_LCD_DOUT_CYCLELEN_S       0
#define LCD_CAM_LCD_DOUT_CYCLELEN_M       0x00001FFFU  /* bits [12:0] */
#define LCD_CAM_LCD_ALWAYS_OUT_EN         BIT(13)
#define LCD_CAM_LCD_8BITS_ORDER           BIT(19)
#define LCD_CAM_LCD_UPDATE                BIT(20)  /* self-clearing */
#define LCD_CAM_LCD_BIT_ORDER             BIT(21)
#define LCD_CAM_LCD_BYTE_ORDER            BIT(22)
#define LCD_CAM_LCD_2BYTE_EN              BIT(23)
#define LCD_CAM_LCD_DOUT                  BIT(24)
#define LCD_CAM_LCD_DUMMY                 BIT(25)
#define LCD_CAM_LCD_CMD                   BIT(26)
#define LCD_CAM_LCD_START                 BIT(27)
#define LCD_CAM_LCD_RESET                 BIT(28)  /* self-clearing, WO */
#define LCD_CAM_LCD_DUMMY_CYCLELEN_S      29
#define LCD_CAM_LCD_DUMMY_CYCLELEN_M      (0x3U << 29) /* bits [30:29] */
#define LCD_CAM_LCD_CMD_2_CYCLE_EN        BIT(31)

/* ------------------------------------------------------------------ */
/* Bit definitions – LCD_MISC_REG (0x18)                              */
/* ------------------------------------------------------------------ */
#define LCD_CAM_LCD_AFIFO_THRESHOLD_S     1
#define LCD_CAM_LCD_AFIFO_THRESHOLD_M     (0x1FU << 1) /* bits [5:1] */
#define LCD_CAM_LCD_VFK_CYCLELEN_S        6
#define LCD_CAM_LCD_VFK_CYCLELEN_M        (0x3FU << 6) /* bits [11:6] */
#define LCD_CAM_LCD_VBK_CYCLELEN_S        12
#define LCD_CAM_LCD_VBK_CYCLELEN_M        (0x1FFFU << 12) /* bits [24:12] */
#define LCD_CAM_LCD_NEXT_FRAME_EN         BIT(25)
#define LCD_CAM_LCD_BK_EN                 BIT(26)
#define LCD_CAM_LCD_AFIFO_RESET           BIT(27)  /* self-clearing, WO */
#define LCD_CAM_LCD_CD_DATA_SET           BIT(28)
#define LCD_CAM_LCD_CD_DUMMY_SET          BIT(29)
#define LCD_CAM_LCD_CD_CMD_SET            BIT(30)
#define LCD_CAM_LCD_CD_IDLE_EDGE          BIT(31)

/* ------------------------------------------------------------------ */
/* Bit definitions – CAM_CTRL_REG (0x04)                              */
/* ------------------------------------------------------------------ */
#define LCD_CAM_CAM_UPDATE                BIT(4)   /* self-clearing */

/* ------------------------------------------------------------------ */
/* Bit definitions – CAM_CTRL1_REG (0x08)                             */
/* ------------------------------------------------------------------ */
#define LCD_CAM_CAM_START                 BIT(29)
#define LCD_CAM_CAM_RESET                 BIT(30)  /* self-clearing, WO */
#define LCD_CAM_CAM_AFIFO_RESET           BIT(31)  /* self-clearing, WO */

/* ------------------------------------------------------------------ */
/* Interrupt bits (shared by RAW/ST/ENA/CLR at offsets 0x64-0x70)     */
/* ------------------------------------------------------------------ */
#define LCD_CAM_INT_LCD_VSYNC             BIT(0)
#define LCD_CAM_INT_LCD_TRANS_DONE        BIT(1)
#define LCD_CAM_INT_CAM_VSYNC             BIT(2)
#define LCD_CAM_INT_CAM_HS                BIT(3)

/* ------------------------------------------------------------------ */
/* LCD_CLOCK_REG (0x00) bit definitions                               */
/* ------------------------------------------------------------------ */
#define LCD_CAM_CLK_EN                    BIT(31)

/* ------------------------------------------------------------------ */
/* Misc defaults                                                      */
/* ------------------------------------------------------------------ */
/* LCD_MISC_REG reset default: lcd_afifo_threshold_num = 17 (bits [5:1]) */
#define LCD_CAM_LCD_MISC_RESET_VAL        (17U << 1)

/* DATE register default (0x02003020 per TRM) */
#define LCD_CAM_DATE_RESET_VAL            0x02003020U

#define ESP32S3_LCD_CAM_REGS_COUNT  (ESP32S3_LCD_CAM_REG_SIZE / 4)

typedef struct ESP32S3LcdCamState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t regs[ESP32S3_LCD_CAM_REGS_COUNT];
    uint32_t int_raw;
    uint32_t int_ena;

    /* Public: set by the machine before realize */
    ESPGdmaState *gdma;
} ESP32S3LcdCamState;

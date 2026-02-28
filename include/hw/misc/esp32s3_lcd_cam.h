/*
 * ESP32-S3 LCD_CAM Controller Model
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * R/W register store at 0x60041000.
 * MVP: absorbs driver accesses. DMA-driven display/camera path stub.
 * Interrupt model: lcd_vsync, lcd_trans_done, cam_vsync, cam_hs (4 bits).
 * IRQ: ETS_LCD_CAM_INTR_SOURCE (24).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#pragma once

#include "hw/sysbus.h"

#define TYPE_ESP32S3_LCD_CAM  "esp32s3.lcd_cam"
#define ESP32S3_LCD_CAM(obj)  OBJECT_CHECK(ESP32S3LcdCamState, (obj), TYPE_ESP32S3_LCD_CAM)

/* Register space: 0x100 confirmed by _Static_assert */
#define ESP32S3_LCD_CAM_REG_SIZE  0x100

/* Key register offsets */
#define LCD_CAM_LCD_CLOCK_REG        0x000
#define LCD_CAM_CAM_CTRL_REG         0x004
#define LCD_CAM_CAM_CTRL1_REG        0x008
#define LCD_CAM_CAM_RGB_YUV_REG      0x00C
#define LCD_CAM_LCD_RGB_YUV_REG      0x010
#define LCD_CAM_LCD_USER_REG         0x014
#define LCD_CAM_LCD_MISC_REG         0x018
#define LCD_CAM_LCD_CTRL_REG         0x01C
#define LCD_CAM_LCD_CTRL1_REG        0x020
#define LCD_CAM_LCD_CTRL2_REG        0x024
#define LCD_CAM_LCD_CMD_VAL_REG      0x028
#define LCD_CAM_LCD_DLY_MODE_REG     0x030
#define LCD_CAM_LCD_DATA_DOUT_MODE_REG 0x038
#define LCD_CAM_LC_DMA_INT_ENA_REG   0x064
#define LCD_CAM_LC_DMA_INT_RAW_REG   0x068
#define LCD_CAM_LC_DMA_INT_ST_REG    0x06C
#define LCD_CAM_LC_DMA_INT_CLR_REG   0x070
#define LCD_CAM_LC_REG_DATE_REG      0x0FC

/* Interrupt bits */
#define LCD_CAM_INT_LCD_VSYNC      BIT(0)
#define LCD_CAM_INT_LCD_TRANS_DONE BIT(1)
#define LCD_CAM_INT_CAM_VSYNC     BIT(2)
#define LCD_CAM_INT_CAM_HS        BIT(3)

#define ESP32S3_LCD_CAM_REGS_COUNT  (ESP32S3_LCD_CAM_REG_SIZE / 4)

typedef struct ESP32S3LcdCamState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t regs[ESP32S3_LCD_CAM_REGS_COUNT];
    uint32_t int_raw;
    uint32_t int_ena;
} ESP32S3LcdCamState;

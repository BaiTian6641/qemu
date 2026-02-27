/*
 * ESP32-S3 IO MUX peripheral
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

#define TYPE_ESP32S3_IOMUX "esp32s3.iomux"
#define ESP32S3_IOMUX(obj) OBJECT_CHECK(ESP32S3IOMuxState, (obj), TYPE_ESP32S3_IOMUX)

/* ESP32-S3 has 49 GPIO pads (GPIO0 - GPIO48) */
#define ESP32S3_IOMUX_GPIO_COUNT   49

/* Register offsets */
REG32(IO_MUX_PIN_CTRL, 0x0000)
    FIELD(IO_MUX_PIN_CTRL, CLK1, 0, 4)
    FIELD(IO_MUX_PIN_CTRL, CLK2, 4, 4)
    FIELD(IO_MUX_PIN_CTRL, CLK3, 8, 4)

/*
 * Per-GPIO pad registers: IO_MUX_GPIO0_REG at offset 0x0004,
 * IO_MUX_GPIO1_REG at offset 0x0008, ..., IO_MUX_GPIO48_REG at 0x00C4.
 * All share the same field layout.
 */
#define IO_MUX_GPIOn_REG_OFFSET(n) (0x0004 + (n) * 4)

/* Fields within each IO_MUX_GPIOn_REG */
REG32(IO_MUX_GPIO0, 0x0004)
    FIELD(IO_MUX_GPIO0, MCU_OE, 0, 1)
    FIELD(IO_MUX_GPIO0, SLP_SEL, 1, 1)
    FIELD(IO_MUX_GPIO0, MCU_WPD, 2, 1)
    FIELD(IO_MUX_GPIO0, MCU_WPU, 3, 1)
    FIELD(IO_MUX_GPIO0, MCU_IE, 4, 1)
    FIELD(IO_MUX_GPIO0, MCU_DRV, 5, 2)
    FIELD(IO_MUX_GPIO0, FUN_WPD, 7, 1)
    FIELD(IO_MUX_GPIO0, FUN_WPU, 8, 1)
    FIELD(IO_MUX_GPIO0, FUN_IE, 9, 1)
    FIELD(IO_MUX_GPIO0, FUN_DRV, 10, 2)
    FIELD(IO_MUX_GPIO0, MCU_SEL, 12, 3)
    FIELD(IO_MUX_GPIO0, FILTER_EN, 15, 1)

/* Version register */
REG32(IO_MUX_DATE, 0x00FC)

/* Total register region size */
#define ESP32S3_IOMUX_REGS_SIZE     0x100

/*
 * Default value for pad registers on reset.
 * Per TRM: MCU_SEL=0, FUN_DRV=2 (medium), FUN_IE=0, FUN_WPD=0, FUN_WPU=0.
 * Some pads may differ (e.g., SPI flash pads), but we use a uniform default.
 */
#define ESP32S3_IOMUX_GPIO_REG_DEFAULT  (0x2 << 10)  /* FUN_DRV = 2 */

/* IO_MUX_DATE version value (from TRM) */
#define ESP32S3_IOMUX_DATE_VERSION      0x2006050

typedef struct ESP32S3IOMuxState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    /* Pin control register */
    uint32_t pin_ctrl;

    /* Per-GPIO pad configuration registers */
    uint32_t gpio_reg[ESP32S3_IOMUX_GPIO_COUNT];

    /* Date/version register */
    uint32_t date_reg;
} ESP32S3IOMuxState;

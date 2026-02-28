/*
 * ESP32-S3 I2S Controller Model
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * R/W register store for I2S0 (0x6000F000) and I2S1 (0x6002D000).
 * MVP: register-level model absorbs driver accesses. Immediate TX completion.
 * Interrupt model: rx_done, tx_done, rx_hung, tx_hung (4 bits).
 * IRQ: ETS_I2S0_INTR_SOURCE (25), ETS_I2S1_INTR_SOURCE (26).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#pragma once

#include "hw/sysbus.h"

#define TYPE_ESP32S3_I2S  "esp32s3.i2s"
#define ESP32S3_I2S(obj)  OBJECT_CHECK(ESP32S3I2SState, (obj), TYPE_ESP32S3_I2S)

/* Register space */
#define ESP32S3_I2S_REG_SIZE   0x84

/* Register offsets (from i2s_struct.h) */
#define I2S_INT_RAW_REG        0x0C
#define I2S_INT_ST_REG         0x10
#define I2S_INT_ENA_REG        0x14
#define I2S_INT_CLR_REG        0x18
#define I2S_RX_CONF_REG        0x20
#define I2S_TX_CONF_REG        0x24
#define I2S_RX_CONF1_REG       0x28
#define I2S_TX_CONF1_REG       0x2C
#define I2S_RX_CLKM_CONF_REG  0x30
#define I2S_TX_CLKM_CONF_REG  0x34
#define I2S_RX_CLKM_DIV_REG   0x38
#define I2S_TX_CLKM_DIV_REG   0x3C
#define I2S_TX_PCM2PDM_CONF_REG  0x40
#define I2S_TX_PCM2PDM_CONF1_REG 0x44
#define I2S_RX_TDM_CTRL_REG   0x50
#define I2S_TX_TDM_CTRL_REG   0x54
#define I2S_RX_TIMING_REG     0x58
#define I2S_TX_TIMING_REG     0x5C
#define I2S_LC_HUNG_CONF_REG  0x60
#define I2S_RX_EOF_NUM_REG    0x64
#define I2S_CONF_SINGLE_DATA_REG 0x68
#define I2S_STATE_REG          0x6C
#define I2S_DATE_REG           0x80

/* Interrupt bits */
#define I2S_INT_RX_DONE   BIT(0)
#define I2S_INT_TX_DONE   BIT(1)
#define I2S_INT_RX_HUNG   BIT(2)
#define I2S_INT_TX_HUNG   BIT(3)

/* TX_CONF bits */
#define I2S_TX_RESET      BIT(0)
#define I2S_TX_FIFO_RESET BIT(1)
#define I2S_TX_START      BIT(2)
#define I2S_TX_UPDATE     BIT(3)

/* RX_CONF bits */
#define I2S_RX_RESET      BIT(0)
#define I2S_RX_FIFO_RESET BIT(1)
#define I2S_RX_START      BIT(2)
#define I2S_RX_UPDATE     BIT(3)

#define ESP32S3_I2S_REGS_COUNT  (ESP32S3_I2S_REG_SIZE / 4)

typedef struct ESP32S3I2SState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t regs[ESP32S3_I2S_REGS_COUNT];
    uint32_t int_raw;
    uint32_t int_ena;
} ESP32S3I2SState;

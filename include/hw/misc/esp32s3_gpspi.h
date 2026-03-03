/*
 * ESP32-S3 GP-SPI Controller Model (SPI2/SPI3)
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
 *
 * R/W register store for SPI2 (0x60024000) and SPI3 (0x60025000).
 * GP-SPI extends beyond flash-only SPI1. GDMA integration stub.
 * MVP: register-level model with command trigger + immediate completion.
 * Interrupt model: 21 DMA interrupt bits + trans_done.
 * IRQ: ETS_SPI2_INTR_SOURCE (21), ETS_SPI3_INTR_SOURCE (22).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#pragma once

#include "hw/sysbus.h"
#include "hw/dma/esp_gdma.h"

#define TYPE_ESP32S3_GPSPI  "esp32s3.gpspi"
#define ESP32S3_GPSPI(obj)  OBJECT_CHECK(ESP32S3GpSpiState, (obj), TYPE_ESP32S3_GPSPI)

/* Register space: 0xF4 from spi_struct.h */
#define ESP32S3_GPSPI_REG_SIZE  0x100  /* Rounded up to 256 bytes */

/* Key register offsets */
#define SPI_CMD_REG            0x00
#define SPI_ADDR_REG           0x04
#define SPI_CTRL_REG           0x08
#define SPI_CLOCK_REG          0x0C
#define SPI_USER_REG           0x10
#define SPI_USER1_REG          0x14
#define SPI_USER2_REG          0x18
#define SPI_MS_DLEN_REG        0x1C
#define SPI_MISC_REG           0x20
#define SPI_DIN_MODE_REG       0x24
#define SPI_DIN_NUM_REG        0x28
#define SPI_DOUT_MODE_REG      0x2C
#define SPI_DMA_CONF_REG       0x30
#define SPI_DMA_INT_ENA_REG    0x34
#define SPI_DMA_INT_CLR_REG    0x38
#define SPI_DMA_INT_RAW_REG    0x3C
#define SPI_DMA_INT_ST_REG     0x40
#define SPI_DMA_INT_SET_REG    0x44
/* 0x98 – 0xD4: DATA_BUF[0..15] (16 × 32-bit) */
#define SPI_W0_REG             0x98
#define SPI_W15_REG            0xD4
#define SPI_SLAVE_REG          0xE0
#define SPI_SLAVE1_REG         0xE4
#define SPI_CLK_GATE_REG       0xE8
#define SPI_DATE_REG           0xF0

/* DMA interrupt bits (selected) */
#define SPI_INT_TRANS_DONE     BIT(12)
#define SPI_INT_RD_DMA_DONE   BIT(8)
#define SPI_INT_WR_DMA_DONE   BIT(9)

/* CMD register bits */
#define SPI_CMD_USR            BIT(24)  /* SPI_USR: trigger user-defined transaction */
#define SPI_CMD_UPDATE         BIT(23)  /* SPI_UPDATE: latch config */

/* USER register bits */
#define SPI_USR_MOSI           BIT(27)  /* Enable MOSI data-out phase */
#define SPI_USR_MISO           BIT(28)  /* Enable MISO data-in phase */

/* DMA_CONF register bits */
#define SPI_DMA_TX_ENA         BIT(28)

/* Maximum bridge DMA capture (covers ESP-IDF max DMA segment of 4092 bytes) */
#define GPSPI_BRIDGE_DMA_MAX   4096

#define ESP32S3_GPSPI_REGS_COUNT  (ESP32S3_GPSPI_REG_SIZE / 4)

typedef struct ESP32S3GpSpiState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    ESPGdmaState *gdma;
    GdmaPeripheral gdma_periph;

    uint32_t regs[ESP32S3_GPSPI_REGS_COUNT];
    uint32_t int_raw;
    uint32_t int_ena;

    /* ---- Bridge fields (SPI → GUI) ---- */
    char *controller_name;          /* "spi2" or "spi3", set by machine model */
    uint8_t dc_gpio;                /* GPIO number for DC pin (0xFF = disabled) */
    bool bridge_enabled;            /* suppress emit until controller_name is set */

    /* DMA capture buffer — filled by try_dma, consumed by bridge emit */
    uint8_t dma_out_buf[GPSPI_BRIDGE_DMA_MAX];
    uint32_t dma_out_len;
} ESP32S3GpSpiState;

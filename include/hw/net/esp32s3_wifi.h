/*
 * ESP32-S3 Wi-Fi peripheral skeleton
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

#define TYPE_ESP32S3_WIFI "esp32s3.wifi"
#define ESP32S3_WIFI(obj)           OBJECT_CHECK(ESP32S3WifiState, (obj), TYPE_ESP32S3_WIFI)
#define ESP32S3_WIFI_GET_CLASS(obj) OBJECT_GET_CLASS(ESP32S3WifiClass, obj, TYPE_ESP32S3_WIFI)
#define ESP32S3_WIFI_CLASS(klass)   OBJECT_CLASS_CHECK(ESP32S3WifiClass, klass, TYPE_ESP32S3_WIFI)

/*
 * The ESP32-S3 Wi-Fi subsystem spans multiple register regions:
 *
 * - MAC layer (interrupt source: ETS_WIFI_MAC_INTR_SOURCE = 0)
 * - Baseband TX/RX (BB at 0x6001D000, NRX at 0x6001CC00)
 * - RF Front-end (FE at 0x60006000, FE2 at 0x60005000)
 * - DMA (SLC at 0x60018000, SLCHOST at 0x60015000)
 * - WiFi device control (WDEV at 0x3ff75000)
 *
 * This skeleton provides a unified device model that exposes
 * sub-regions for each hardware block. Initially, all registers
 * return reset defaults or zero.
 */

/* Sub-region sizes */
#define ESP32S3_WIFI_BB_SIZE       0x1000
#define ESP32S3_WIFI_NRX_SIZE      0x0400
#define ESP32S3_WIFI_FE_SIZE       0x1000
#define ESP32S3_WIFI_FE2_SIZE      0x1000
#define ESP32S3_WIFI_SLC_SIZE      0x1000
#define ESP32S3_WIFI_SLCHOST_SIZE  0x1000
#define ESP32S3_WIFI_WDEV_SIZE     0x1000

/* Generic stub register space for each sub-block */
#define ESP32S3_WIFI_REG_WORDS(sz) ((sz) / 4)

/*
 * Init-critical register offsets within each sub-block.
 * These must have proper reset defaults for esp_wifi_init() to succeed.
 * Offsets are relative to the sub-block base address.
 */

/* BB (Baseband) at 0x6001D000 */
#define BB_BBPD_CTRL_OFF           0x0054  /* Power-down control */
#define BB_BBPD_CTRL_DEFAULT       0x0000000A  /* FFT_FORCE_PU=1, DC_EST_FORCE_PU=1 */

/* NRX (Baseband RX) at 0x6001CC00 */
#define NRX_NRXPD_CTRL_OFF        0x00D4  /* Power-down control */
#define NRX_NRXPD_CTRL_DEFAULT    0x000000AA  /* All _FORCE_PU=1 (chan_est, rx_rot, vit, demap) */

/* FE (RF Front-End) at 0x60006000 */
#define FE_GEN_CTRL_OFF            0x0090  /* General control */
#define FE_GEN_CTRL_DEFAULT        0x00000020  /* IQ_EST_FORCE_PU=1 */

/* FE2 (RF Front-End 2) at 0x60005000 */
#define FE2_TX_INTERP_CTRL_OFF     0x00F0  /* TX interpolator control */
#define FE2_TX_INTERP_CTRL_DEFAULT 0x00000200  /* TX_INF_FORCE_PU=1 */

/* SLC DMA critical register offsets (for M1-T4 DMA ring support) */
#define SLC_CONF0_OFF              0x0000
#define SLC_0INT_RAW_OFF           0x0004
#define SLC_0INT_ST_OFF            0x0008
#define SLC_0INT_ENA_OFF           0x000C
#define SLC_0INT_CLR_OFF           0x0010
#define SLC_1INT_RAW_OFF           0x0014
#define SLC_1INT_ST_OFF            0x0018
#define SLC_1INT_ENA_OFF           0x001C
#define SLC_1INT_CLR_OFF           0x0020
#define SLC_0RXLINK_OFF            0x0024  /* SLC0 RX link descriptor pointer */
#define SLC_0TXLINK_OFF            0x0028  /* SLC0 TX link descriptor pointer */
#define SLC_1RXLINK_OFF            0x002C
#define SLC_1TXLINK_OFF            0x0030
/* SLC_RX/TX_STATUS */
#define SLC_RX_STATUS_OFF          0x0034
#define SLC_TX_STATUS_OFF          0x0038

/*
 * Wi-Fi interrupt lines exposed to the interrupt matrix.
 * The skeleton exposes 4 IRQ outputs matching the TRM sources.
 */
#define ESP32S3_WIFI_IRQ_MAC       0
#define ESP32S3_WIFI_IRQ_MAC_NMI   1
#define ESP32S3_WIFI_IRQ_PWR       2
#define ESP32S3_WIFI_IRQ_BB        3
#define ESP32S3_WIFI_IRQ_COUNT     4

#define ESP32S3_WIFI_IRQ_NAME "esp32s3-wifi-irq"


typedef struct ESP32S3WifiState {
    SysBusDevice parent_obj;

    /* Memory regions for each WiFi sub-block */
    MemoryRegion iomem_bb;       /* Baseband */
    MemoryRegion iomem_nrx;      /* NRX (baseband RX) */
    MemoryRegion iomem_fe;       /* RF Front-End */
    MemoryRegion iomem_fe2;      /* RF Front-End 2 */
    MemoryRegion iomem_slc;      /* SLC DMA */
    MemoryRegion iomem_slchost;  /* SLC Host */
    MemoryRegion iomem_wdev;     /* WiFi device control */

    /* IRQ lines */
    qemu_irq irq[ESP32S3_WIFI_IRQ_COUNT];

    /* Register backing stores */
    uint32_t bb_regs[ESP32S3_WIFI_REG_WORDS(ESP32S3_WIFI_BB_SIZE)];
    uint32_t nrx_regs[ESP32S3_WIFI_REG_WORDS(ESP32S3_WIFI_NRX_SIZE)];
    uint32_t fe_regs[ESP32S3_WIFI_REG_WORDS(ESP32S3_WIFI_FE_SIZE)];
    uint32_t fe2_regs[ESP32S3_WIFI_REG_WORDS(ESP32S3_WIFI_FE2_SIZE)];
    uint32_t slc_regs[ESP32S3_WIFI_REG_WORDS(ESP32S3_WIFI_SLC_SIZE)];
    uint32_t slchost_regs[ESP32S3_WIFI_REG_WORDS(ESP32S3_WIFI_SLCHOST_SIZE)];
    uint32_t wdev_regs[ESP32S3_WIFI_REG_WORDS(ESP32S3_WIFI_WDEV_SIZE)];

} ESP32S3WifiState;

typedef struct ESP32S3WifiClass {
    SysBusDeviceClass parent_class;
} ESP32S3WifiClass;

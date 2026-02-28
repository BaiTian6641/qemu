/*
 * ESP32-S3 Wi-Fi peripheral with SLC DMA and data-plane backend
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
#include "hw/net/esp32s3_wifi_backend.h"

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

/* SLC_CONF0 bit definitions */
#define SLC_CONF0_SLC0_TX_RST            BIT(0)
#define SLC_CONF0_SLC0_RX_RST            BIT(1)
#define SLC_CONF0_AHBM_FIFO_RST          BIT(2)
#define SLC_CONF0_AHBM_RST               BIT(3)
#define SLC_CONF0_SLC0_RX_AUTO_WRBACK    BIT(6)
#define SLC_CONF0_SLC0_RXLINK_AUTO_RET   BIT(10)
#define SLC_CONF0_SLC0_TXLINK_AUTO_RET   BIT(11)
#define SLC_CONF0_SLC1_TX_RST            BIT(16)
#define SLC_CONF0_SLC1_RX_RST            BIT(17)

/* SLC link register bit definitions */
#define SLC_LINK_ADDR_MASK     0x000FFFFF   /* bits [19:0]: descriptor address */
#define SLC_LINK_STOP          BIT(28)
#define SLC_LINK_START         BIT(29)
#define SLC_LINK_RESTART       BIT(30)
#define SLC_LINK_PARK          BIT(31)      /* read-only: 1=parked/idle */

/* SLC0 interrupt bit definitions — complete map from ESP-IDF sdio_slc_reg.h */
#define SLC0_INT_FRHOST_BIT0   BIT(0)   /* From-host interrupt bit 0 */
#define SLC0_INT_FRHOST_BIT1   BIT(1)
#define SLC0_INT_FRHOST_BIT2   BIT(2)
#define SLC0_INT_FRHOST_BIT3   BIT(3)
#define SLC0_INT_FRHOST_BIT4   BIT(4)
#define SLC0_INT_FRHOST_BIT5   BIT(5)
#define SLC0_INT_FRHOST_BIT6   BIT(6)
#define SLC0_INT_FRHOST_BIT7   BIT(7)
#define SLC0_INT_RX_START      BIT(8)   /* RX DMA started */
#define SLC0_INT_TX_START      BIT(9)   /* TX DMA started */
#define SLC0_INT_RX_UDF        BIT(10)  /* RX FIFO underflow */
#define SLC0_INT_TX_OVF        BIT(11)  /* TX FIFO overflow */
#define SLC0_INT_TOKEN0_1TO0   BIT(12)  /* Token0 counter → 0 */
#define SLC0_INT_TOKEN1_1TO0   BIT(13)  /* Token1 counter → 0 */
#define SLC0_INT_TX_DONE       BIT(14)  /* One TX descriptor processed */
#define SLC0_INT_TX_SUC_EOF    BIT(15)  /* TX chain hit eof=1 */
#define SLC0_INT_RX_DONE       BIT(16)  /* One RX descriptor processed */
#define SLC0_INT_RX_EOF        BIT(17)  /* RX chain hit eof=1 */
#define SLC0_INT_TOHOST        BIT(18)  /* Slave → host notification */
#define SLC0_INT_TX_DSCR_ERR   BIT(19)  /* TX descriptor error */
#define SLC0_INT_RX_DSCR_ERR   BIT(20)  /* RX descriptor error */
#define SLC0_INT_TX_DSCR_EMPTY BIT(21)  /* TX chain exhausted (next=NULL) */
#define SLC0_INT_HOST_RD_ACK   BIT(22)  /* Host read acknowledgment */
#define SLC0_INT_WR_RETRY_DONE BIT(23)  /* Write retry done */
#define SLC0_INT_TX_ERR_EOF    BIT(24)  /* TX error EOF */
#define SLC0_INT_CMD_DTC       BIT(25)  /* Command data transfer complete */
#define SLC0_INT_RX_QUICK_EOF  BIT(26)  /* RX quick EOF */

/*
 * lldesc_t — SLC DMA descriptor layout (3 x 32-bit words, 12 bytes)
 *
 * Word 0:  [11:0]  size   — buffer capacity
 *          [23:12] length — actual data length
 *          [28:24] offset — reserved by HW
 *          [29]    sosf   — start of sub-frame
 *          [30]    eof    — end of frame
 *          [31]    owner  — 1=HW owned, 0=SW owned
 * Word 1:  [31:0]  buf_ptr — buffer address
 * Word 2:  [31:0]  next    — next descriptor address (0 = end of chain)
 */
#define LLDESC_SIZE_BYTES      12
#define LLDESC_OWNER_MASK      0x80000000u
#define LLDESC_EOF_MASK        0x40000000u
#define LLDESC_SOSF_MASK       0x20000000u
#define LLDESC_LENGTH_MASK     0x00FFF000u
#define LLDESC_LENGTH_SHIFT    12
#define LLDESC_SIZE_MASK       0x00000FFFu
#define LLDESC_SW_OWNED        0
#define LLDESC_HW_OWNED        1

/* Maximum descriptors to walk per DMA trigger (safety valve) */
#define SLC_DMA_MAX_DESCRIPTORS  256

/*
 * WDEV register offsets (critical for mode/MAC configuration).
 * The blob writes MAC addresses and configuration here.
 */
#define WDEV_MAC_ADDR_LO_OFF       0x0088  /* MAC address [31:0]   */
#define WDEV_MAC_ADDR_HI_OFF       0x008C  /* MAC address [47:32]  */
#define WDEV_RXBUF_BASE_OFF        0x00E8  /* RX buffer base       */

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

    /* SLC DMA interrupt state (separate from generic register store) */
    uint32_t slc0_int_raw;
    uint32_t slc0_int_ena;
    uint32_t slc1_int_raw;
    uint32_t slc1_int_ena;

    /* SLC0 DMA engine state */
    uint32_t slc0_tx_link;       /* TX link register (addr + control bits) */
    uint32_t slc0_rx_link;       /* RX link register */
    hwaddr   slc0_tx_cur_desc;   /* current TX descriptor address (0=idle) */
    hwaddr   slc0_rx_cur_desc;   /* current RX descriptor address (0=idle) */
    bool     slc0_tx_running;    /* TX DMA engine active */
    bool     slc0_rx_running;    /* RX DMA engine active */

    /* Wi-Fi data-plane backend (S4) */
    Esp32s3WifiBackend backend;

    /* NIC configuration (exposed as device property) */
    NICConf  nic_conf;

} ESP32S3WifiState;

typedef struct ESP32S3WifiClass {
    SysBusDeviceClass parent_class;
} ESP32S3WifiClass;

/**
 * Update SLC0 IRQ output.  Called after modifying slc0_int_raw/slc0_int_ena.
 * Exposed so the backend can trigger IRQs after RX injection.
 */
void esp32s3_wifi_slc_update_irq(ESP32S3WifiState *s);

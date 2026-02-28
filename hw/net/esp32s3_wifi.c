/*
 * ESP32-S3 Wi-Fi peripheral with SLC DMA and data-plane backend
 *
 * Provides memory-mapped register regions for the major Wi-Fi hardware
 * blocks (Baseband, NRX, RF Front-End, SLC DMA, WDEV). The SLC block
 * implements proper interrupt RAW/ST/ENA/CLR semantics. The data-plane
 * backend bridges frames to/from the QEMU host networking stack
 * supporting STA, AP, and AP_STA modes.
 *
 * Copyright (c) 2024-2026 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "net/net.h"
#include "hw/net/esp32s3_wifi.h"
#include "hw/net/esp32s3_wifi_backend.h"
#include "trace.h"

#define WIFI_DEBUG 0

/* ---------- Generic sub-block register ops ---------- */

typedef struct WifiSubBlockInfo {
    const char  *name;
    uint32_t    *regs;
    size_t       n_words;
} WifiSubBlockInfo;

static uint64_t wifi_subblock_read(void *opaque, hwaddr addr, unsigned int size)
{
    WifiSubBlockInfo *info = (WifiSubBlockInfo *)opaque;
    uint32_t word_idx = addr / 4;

    if (word_idx < info->n_words) {
        uint32_t val = info->regs[word_idx];
#if WIFI_DEBUG
        qemu_log_mask(LOG_UNIMP,
                      "esp32s3_wifi: %s read addr=0x%04" HWADDR_PRIx " val=0x%08x\n",
                      info->name, addr, val);
#endif
        return val;
    }

    qemu_log_mask(LOG_GUEST_ERROR,
                  "esp32s3_wifi: %s bad read addr=0x%04" HWADDR_PRIx "\n",
                  info->name, addr);
    return 0;
}

static void wifi_subblock_write(void *opaque, hwaddr addr, uint64_t value,
                                unsigned int size)
{
    WifiSubBlockInfo *info = (WifiSubBlockInfo *)opaque;
    uint32_t word_idx = addr / 4;

    if (word_idx < info->n_words) {
#if WIFI_DEBUG
        qemu_log_mask(LOG_UNIMP,
                      "esp32s3_wifi: %s write addr=0x%04" HWADDR_PRIx " val=0x%08" PRIx64 "\n",
                      info->name, addr, value);
#endif
        info->regs[word_idx] = (uint32_t)value;
        return;
    }

    qemu_log_mask(LOG_GUEST_ERROR,
                  "esp32s3_wifi: %s bad write addr=0x%04" HWADDR_PRIx "\n",
                  info->name, addr);
}

static const MemoryRegionOps wifi_subblock_ops = {
    .read = wifi_subblock_read,
    .write = wifi_subblock_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* Static sub-block descriptors */
static WifiSubBlockInfo bb_info, nrx_info, fe_info, fe2_info;
static WifiSubBlockInfo slchost_info;

/* Helper to initialize one generic sub-block memory region */
static void wifi_init_subblock(ESP32S3WifiState *s, Object *obj,
                                MemoryRegion *mr, WifiSubBlockInfo *info,
                                const char *name, uint32_t *regs,
                                size_t n_words, size_t byte_size,
                                int mmio_index)
{
    info->name = name;
    info->regs = regs;
    info->n_words = n_words;

    memory_region_init_io(mr, obj, &wifi_subblock_ops, info,
                          name, byte_size);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), mr);
}

/* ---------- SLC DMA interrupt-aware register ops ---------- */

/*
 * The SLC (Serial Link Controller) is the Wi-Fi DMA engine. It has
 * two channels (SLC0 for Wi-Fi data, SLC1 for BT). Each has its own
 * interrupt RAW/ST/ENA/CLR registers following standard ESP interrupt
 * semantics:
 *   - INT_RAW: set by hardware events (and writable by software)
 *   - INT_ST:  read-only = INT_RAW & INT_ENA
 *   - INT_ENA: enables which RAW bits generate interrupts
 *   - INT_CLR: write-1-to-clear corresponding INT_RAW bits
 *
 * The MAC interrupt line is driven by (SLC0_INT_RAW & SLC0_INT_ENA).
 */

void esp32s3_wifi_slc_update_irq(ESP32S3WifiState *s)
{
    uint32_t slc0_pending = s->slc0_int_raw & s->slc0_int_ena;
    uint32_t slc1_pending = s->slc1_int_raw & s->slc1_int_ena;

    trace_esp32s3_wifi_slc_irq(s->slc0_int_raw, s->slc0_int_ena, slc0_pending);

    /* SLC0 drives the Wi-Fi MAC interrupt */
    qemu_set_irq(s->irq[ESP32S3_WIFI_IRQ_MAC], slc0_pending ? 1 : 0);
    /* SLC1 currently not wired (BT uses different interrupt path) */
    (void)slc1_pending;
}

/* ---------- SLC DMA descriptor engine ---------- */

/*
 * Read an lldesc_t (12 bytes / 3 words) from guest physical memory.
 * Returns true on success, false if the address is zero/invalid.
 */
static bool slc_dma_read_desc(hwaddr desc_addr,
                              uint32_t *word0, uint32_t *buf_ptr,
                              uint32_t *next_ptr)
{
    if (desc_addr == 0) {
        return false;
    }

    uint32_t buf[3];
    cpu_physical_memory_read(desc_addr, buf, LLDESC_SIZE_BYTES);
    *word0    = le32_to_cpu(buf[0]);
    *buf_ptr  = le32_to_cpu(buf[1]);
    *next_ptr = le32_to_cpu(buf[2]);
    return true;
}

/*
 * Write back the first word of a descriptor (flipping owner/length/eof).
 */
static void slc_dma_writeback_word0(hwaddr desc_addr, uint32_t word0)
{
    uint32_t val = cpu_to_le32(word0);
    cpu_physical_memory_write(desc_addr, &val, 4);
}

/*
 * Walk the SLC0 TX descriptor chain.
 *
 * TX processing: The Wi-Fi driver fills TX descriptors with data and sets
 * owner=HW. The DMA engine walks the chain, "transmits" the data (in our
 * skeleton this means just acknowledging the descriptors), flips owner
 * back to SW, and raises TX_DONE / TX_SUC_EOF interrupts.
 *
 * This implements the ownership transitions the ESP-IDF Wi-Fi driver
 * expects for its TX completion polling.
 */
static void slc0_dma_tx_run(ESP32S3WifiState *s)
{
    uint32_t word0, buf_ptr, next_ptr;
    int count = 0;

    while (s->slc0_tx_running && s->slc0_tx_cur_desc &&
           count < SLC_DMA_MAX_DESCRIPTORS) {

        if (!slc_dma_read_desc(s->slc0_tx_cur_desc, &word0, &buf_ptr,
                               &next_ptr)) {
            /* Invalid descriptor address → error */
            s->slc0_int_raw |= SLC0_INT_TX_DSCR_ERR;
            s->slc0_tx_running = false;
            break;
        }

        /* Only process HW-owned descriptors */
        if (!(word0 & LLDESC_OWNER_MASK)) {
            /* Descriptor owned by SW → DMA parks (waits) */
            s->slc0_tx_running = false;
            break;
        }

        /*
         * "Transmit" the descriptor: extract frame data from the buffer
         * and hand it to the data-plane backend for bridging to the host.
         */
        uint32_t frame_len = (word0 & LLDESC_LENGTH_MASK) >> LLDESC_LENGTH_SHIFT;
        if (frame_len > 0 && buf_ptr != 0 && s->backend.enabled) {
            uint8_t frame_buf[2400];
            if (frame_len > sizeof(frame_buf)) {
                frame_len = sizeof(frame_buf);
            }
            cpu_physical_memory_read(buf_ptr, frame_buf, frame_len);
            esp32s3_wifi_backend_tx(&s->backend, frame_buf, frame_len);
        }

        /* Flip owner to SW */
        word0 &= ~LLDESC_OWNER_MASK;
        slc_dma_writeback_word0(s->slc0_tx_cur_desc, word0);

        /* Raise TX_DONE for every descriptor */
        s->slc0_int_raw |= SLC0_INT_TX_DONE;

        /* If EOF is set, also raise TX_SUC_EOF */
        if (word0 & LLDESC_EOF_MASK) {
            s->slc0_int_raw |= SLC0_INT_TX_SUC_EOF;
        }

        /* Advance to next descriptor */
        if (next_ptr == 0) {
            /* Chain exhausted */
            s->slc0_int_raw |= SLC0_INT_TX_DSCR_EMPTY;
            s->slc0_tx_cur_desc = 0;
            s->slc0_tx_running = false;
        } else {
            s->slc0_tx_cur_desc = next_ptr;
        }
        count++;
    }

    esp32s3_wifi_slc_update_irq(s);
}

/*
 * Walk the SLC0 RX descriptor chain.
 *
 * RX processing: The Wi-Fi driver sets up empty RX descriptors with
 * owner=HW. When "data arrives" (simulated), the DMA engine walks the
 * chain, marks descriptors as filled (length set, owner flipped to SW),
 * and raises RX_DONE / RX_EOF interrupts.
 *
 * In this MVP skeleton, we don't inject real frames. The RX engine
 * runs when started but only transitions ownership of one full
 * descriptor chain to allow the driver's init polling to succeed.
 * Real frame injection will be added in M2.
 */
static void slc0_dma_rx_run(ESP32S3WifiState *s)
{
    uint32_t word0, buf_ptr, next_ptr;
    int count = 0;

    while (s->slc0_rx_running && s->slc0_rx_cur_desc &&
           count < SLC_DMA_MAX_DESCRIPTORS) {

        if (!slc_dma_read_desc(s->slc0_rx_cur_desc, &word0, &buf_ptr,
                               &next_ptr)) {
            s->slc0_int_raw |= SLC0_INT_RX_DSCR_ERR;
            s->slc0_rx_running = false;
            break;
        }

        /* Only process HW-owned descriptors */
        if (!(word0 & LLDESC_OWNER_MASK)) {
            /* SW still owns it → park */
            s->slc0_rx_running = false;
            break;
        }

        /*
         * For the MVP, simulate an empty RX completion: set length=0,
         * flip owner to SW. Real frame injection (M2) will set length
         * to actual data size and copy data into buf_ptr.
         */
        word0 &= ~LLDESC_OWNER_MASK;
        /* Clear length field and set it to 0 for empty completion */
        word0 &= ~LLDESC_LENGTH_MASK;
        slc_dma_writeback_word0(s->slc0_rx_cur_desc, word0);

        s->slc0_int_raw |= SLC0_INT_RX_DONE;

        if (word0 & LLDESC_EOF_MASK) {
            s->slc0_int_raw |= SLC0_INT_RX_EOF;
        }

        if (next_ptr == 0) {
            s->slc0_rx_cur_desc = 0;
            s->slc0_rx_running = false;
        } else {
            s->slc0_rx_cur_desc = next_ptr;
        }
        count++;
    }

    esp32s3_wifi_slc_update_irq(s);
}

/*
 * Handle writes to SLC0 link registers.
 * On START bit: load descriptor address and begin DMA.
 * On STOP bit: halt DMA engine.
 * On RESTART bit: resume from current position.
 */
static void slc0_tx_link_write(ESP32S3WifiState *s, uint32_t value)
{
    /* Always store the address portion */
    s->slc0_tx_link = (s->slc0_tx_link & ~SLC_LINK_ADDR_MASK) |
                      (value & SLC_LINK_ADDR_MASK);

    if (value & SLC_LINK_STOP) {
        s->slc0_tx_running = false;
        trace_esp32s3_wifi_slc_dma_stop("TX");
    }

    if (value & SLC_LINK_START) {
        s->slc0_tx_cur_desc = value & SLC_LINK_ADDR_MASK;
        s->slc0_tx_running = true;
        s->slc0_int_raw |= SLC0_INT_TX_START;
        trace_esp32s3_wifi_slc_dma_start("TX", s->slc0_tx_cur_desc);
        slc0_dma_tx_run(s);
    }

    if (value & SLC_LINK_RESTART) {
        if (s->slc0_tx_cur_desc) {
            s->slc0_tx_running = true;
            slc0_dma_tx_run(s);
        }
    }
}

static void slc0_rx_link_write(ESP32S3WifiState *s, uint32_t value)
{
    s->slc0_rx_link = (s->slc0_rx_link & ~SLC_LINK_ADDR_MASK) |
                      (value & SLC_LINK_ADDR_MASK);

    if (value & SLC_LINK_STOP) {
        s->slc0_rx_running = false;
        trace_esp32s3_wifi_slc_dma_stop("RX");
    }

    if (value & SLC_LINK_START) {
        s->slc0_rx_cur_desc = value & SLC_LINK_ADDR_MASK;
        s->slc0_rx_running = true;
        s->slc0_int_raw |= SLC0_INT_RX_START;
        trace_esp32s3_wifi_slc_dma_start("RX", s->slc0_rx_cur_desc);
        slc0_dma_rx_run(s);
    }

    if (value & SLC_LINK_RESTART) {
        if (s->slc0_rx_cur_desc) {
            s->slc0_rx_running = true;
            slc0_dma_rx_run(s);
        }
    }
}

/*
 * Handle SLC_CONF0 writes — process reset bits.
 */
static void slc_conf0_write(ESP32S3WifiState *s, uint32_t value)
{
    if (value & SLC_CONF0_SLC0_TX_RST) {
        s->slc0_tx_running = false;
        s->slc0_tx_cur_desc = 0;
    }
    if (value & SLC_CONF0_SLC0_RX_RST) {
        s->slc0_rx_running = false;
        s->slc0_rx_cur_desc = 0;
    }
    /* Store the register value (reset bits are self-clearing in real HW,
     * but the driver typically writes them then clears them separately) */
    s->slc_regs[SLC_CONF0_OFF / 4] = value;
}

static uint64_t wifi_slc_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3WifiState *s = (ESP32S3WifiState *)opaque;

    switch (addr) {
    case SLC_CONF0_OFF:
        return s->slc_regs[SLC_CONF0_OFF / 4];

    /* SLC0 interrupt registers */
    case SLC_0INT_RAW_OFF:
        return s->slc0_int_raw;
    case SLC_0INT_ST_OFF:
        return s->slc0_int_raw & s->slc0_int_ena;
    case SLC_0INT_ENA_OFF:
        return s->slc0_int_ena;
    case SLC_0INT_CLR_OFF:
        return 0; /* write-only */

    /* SLC1 interrupt registers */
    case SLC_1INT_RAW_OFF:
        return s->slc1_int_raw;
    case SLC_1INT_ST_OFF:
        return s->slc1_int_raw & s->slc1_int_ena;
    case SLC_1INT_ENA_OFF:
        return s->slc1_int_ena;
    case SLC_1INT_CLR_OFF:
        return 0;

    /* SLC0 link registers — return stored address + park status */
    case SLC_0TXLINK_OFF: {
        uint32_t val = s->slc0_tx_link & SLC_LINK_ADDR_MASK;
        if (!s->slc0_tx_running) {
            val |= SLC_LINK_PARK;
        }
        return val;
    }
    case SLC_0RXLINK_OFF: {
        uint32_t val = s->slc0_rx_link & SLC_LINK_ADDR_MASK;
        if (!s->slc0_rx_running) {
            val |= SLC_LINK_PARK;
        }
        return val;
    }

    /* SLC1 link registers — stub (parked) */
    case SLC_1RXLINK_OFF:
    case SLC_1TXLINK_OFF:
        return SLC_LINK_PARK;

    case SLC_RX_STATUS_OFF:
    case SLC_TX_STATUS_OFF:
        return s->slc_regs[addr / 4];

    default: {
        /* Fall back to generic register store */
        uint32_t word_idx = addr / 4;
        if (word_idx < ESP32S3_WIFI_REG_WORDS(ESP32S3_WIFI_SLC_SIZE)) {
            return s->slc_regs[word_idx];
        }
        return 0;
    }
    }
}

static void wifi_slc_write(void *opaque, hwaddr addr, uint64_t value,
                           unsigned int size)
{
    ESP32S3WifiState *s = (ESP32S3WifiState *)opaque;

    switch (addr) {
    case SLC_CONF0_OFF:
        slc_conf0_write(s, (uint32_t)value);
        break;

    /* SLC0 interrupt registers */
    case SLC_0INT_RAW_OFF:
        s->slc0_int_raw |= (uint32_t)value;
        esp32s3_wifi_slc_update_irq(s);
        break;
    case SLC_0INT_ENA_OFF:
        s->slc0_int_ena = (uint32_t)value;
        esp32s3_wifi_slc_update_irq(s);
        break;
    case SLC_0INT_CLR_OFF:
        s->slc0_int_raw &= ~(uint32_t)value;
        esp32s3_wifi_slc_update_irq(s);
        break;

    /* SLC1 interrupt registers */
    case SLC_1INT_RAW_OFF:
        s->slc1_int_raw |= (uint32_t)value;
        esp32s3_wifi_slc_update_irq(s);
        break;
    case SLC_1INT_ENA_OFF:
        s->slc1_int_ena = (uint32_t)value;
        esp32s3_wifi_slc_update_irq(s);
        break;
    case SLC_1INT_CLR_OFF:
        s->slc1_int_raw &= ~(uint32_t)value;
        esp32s3_wifi_slc_update_irq(s);
        break;

    /* SLC0 link registers — DMA engine control */
    case SLC_0TXLINK_OFF:
        slc0_tx_link_write(s, (uint32_t)value);
        break;
    case SLC_0RXLINK_OFF:
        slc0_rx_link_write(s, (uint32_t)value);
        break;

    /* SLC1 link registers — stub (ignored) */
    case SLC_1RXLINK_OFF:
    case SLC_1TXLINK_OFF:
        break;

    /* All other registers → generic store */
    default: {
        uint32_t word_idx = addr / 4;
        if (word_idx < ESP32S3_WIFI_REG_WORDS(ESP32S3_WIFI_SLC_SIZE)) {
            s->slc_regs[word_idx] = (uint32_t)value;
        }
        break;
    }
    }
}

static const MemoryRegionOps wifi_slc_ops = {
    .read = wifi_slc_read,
    .write = wifi_slc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ---------- WDEV sub-block with MAC address intercept ---------- */

static uint64_t wifi_wdev_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3WifiState *s = (ESP32S3WifiState *)opaque;
    uint32_t word_idx = addr / 4;

    if (word_idx < ESP32S3_WIFI_REG_WORDS(ESP32S3_WIFI_WDEV_SIZE)) {
#if WIFI_DEBUG
        qemu_log_mask(LOG_UNIMP,
                      "esp32s3_wifi: WDEV read addr=0x%04" HWADDR_PRIx " val=0x%08x\n",
                      addr, s->wdev_regs[word_idx]);
#endif
        return s->wdev_regs[word_idx];
    }
    return 0;
}

static void wifi_wdev_write(void *opaque, hwaddr addr, uint64_t value,
                            unsigned int size)
{
    ESP32S3WifiState *s = (ESP32S3WifiState *)opaque;
    uint32_t word_idx = addr / 4;

    if (word_idx >= ESP32S3_WIFI_REG_WORDS(ESP32S3_WIFI_WDEV_SIZE)) {
        return;
    }

    s->wdev_regs[word_idx] = (uint32_t)value;

    /* Intercept MAC address writes */
    switch (addr) {
    case WDEV_MAC_ADDR_LO_OFF: {
        /* Low 32 bits of MAC written — update backend after hi is written */
        break;
    }
    case WDEV_MAC_ADDR_HI_OFF: {
        /* High 16 bits of MAC — assemble full MAC and update backend */
        uint32_t lo = s->wdev_regs[WDEV_MAC_ADDR_LO_OFF / 4];
        uint32_t hi = (uint32_t)value;
        uint8_t mac[6];
        mac[0] = (lo >>  0) & 0xff;
        mac[1] = (lo >>  8) & 0xff;
        mac[2] = (lo >> 16) & 0xff;
        mac[3] = (lo >> 24) & 0xff;
        mac[4] = (hi >>  0) & 0xff;
        mac[5] = (hi >>  8) & 0xff;
        /* Detect if this is STA or AP MAC by comparing with known patterns.
         * The blob typically writes STA MAC first. */
        esp32s3_wifi_backend_set_sta_mac(&s->backend, mac);
        break;
    }
    default:
        break;
    }
}

static const MemoryRegionOps wifi_wdev_ops = {
    .read = wifi_wdev_read,
    .write = wifi_wdev_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

/* ---------- Device lifecycle ---------- */

static void esp32s3_wifi_reset_hold(Object *obj, ResetType type)
{
    ESP32S3WifiState *s = ESP32S3_WIFI(obj);

    memset(s->bb_regs, 0, sizeof(s->bb_regs));
    memset(s->nrx_regs, 0, sizeof(s->nrx_regs));
    memset(s->fe_regs, 0, sizeof(s->fe_regs));
    memset(s->fe2_regs, 0, sizeof(s->fe2_regs));
    memset(s->slc_regs, 0, sizeof(s->slc_regs));
    memset(s->slchost_regs, 0, sizeof(s->slchost_regs));
    memset(s->wdev_regs, 0, sizeof(s->wdev_regs));

    /* Clear SLC interrupt state */
    s->slc0_int_raw = 0;
    s->slc0_int_ena = 0;
    s->slc1_int_raw = 0;
    s->slc1_int_ena = 0;

    /* Reset SLC0 DMA engine */
    s->slc0_tx_link = 0;
    s->slc0_rx_link = 0;
    s->slc0_tx_cur_desc = 0;
    s->slc0_rx_cur_desc = 0;
    s->slc0_tx_running = false;
    s->slc0_rx_running = false;

    /*
     * Pre-load init-critical register defaults.
     * The ESP-IDF PHY init code (register_chipv7_phy ROM function) reads
     * power-down control registers in BB/NRX/FE/FE2 blocks and expects
     * them to have power-up defaults. Without these, the closed-source
     * PHY calibration may fail or behave unexpectedly.
     */
    s->bb_regs[BB_BBPD_CTRL_OFF / 4] = BB_BBPD_CTRL_DEFAULT;
    s->nrx_regs[NRX_NRXPD_CTRL_OFF / 4] = NRX_NRXPD_CTRL_DEFAULT;
    s->fe_regs[FE_GEN_CTRL_OFF / 4] = FE_GEN_CTRL_DEFAULT;
    s->fe2_regs[FE2_TX_INTERP_CTRL_OFF / 4] = FE2_TX_INTERP_CTRL_DEFAULT;

    /* De-assert all IRQs */
    for (int i = 0; i < ESP32S3_WIFI_IRQ_COUNT; i++) {
        qemu_set_irq(s->irq[i], 0);
    }

    /* Reset the data-plane backend */
    esp32s3_wifi_backend_reset(&s->backend);
}

static void esp32s3_wifi_init(Object *obj)
{
    ESP32S3WifiState *s = ESP32S3_WIFI(obj);

    /*
     * Initialize sub-block MMIO regions.
     * The machine file maps each region to its physical address.
     * MMIO region indices (sysbus_mmio_get_region index):
     *   0 = BB,  1 = NRX,  2 = FE,  3 = FE2,
     *   4 = SLC, 5 = SLCHOST, 6 = WDEV
     */
    wifi_init_subblock(s, obj, &s->iomem_bb, &bb_info,
                       "esp32s3.wifi.bb", s->bb_regs,
                       ESP32S3_WIFI_REG_WORDS(ESP32S3_WIFI_BB_SIZE),
                       ESP32S3_WIFI_BB_SIZE, 0);

    wifi_init_subblock(s, obj, &s->iomem_nrx, &nrx_info,
                       "esp32s3.wifi.nrx", s->nrx_regs,
                       ESP32S3_WIFI_REG_WORDS(ESP32S3_WIFI_NRX_SIZE),
                       ESP32S3_WIFI_NRX_SIZE, 1);

    wifi_init_subblock(s, obj, &s->iomem_fe, &fe_info,
                       "esp32s3.wifi.fe", s->fe_regs,
                       ESP32S3_WIFI_REG_WORDS(ESP32S3_WIFI_FE_SIZE),
                       ESP32S3_WIFI_FE_SIZE, 2);

    wifi_init_subblock(s, obj, &s->iomem_fe2, &fe2_info,
                       "esp32s3.wifi.fe2", s->fe2_regs,
                       ESP32S3_WIFI_REG_WORDS(ESP32S3_WIFI_FE2_SIZE),
                       ESP32S3_WIFI_FE2_SIZE, 3);

    /* SLC uses its own interrupt-aware ops instead of the generic subblock ops */
    memory_region_init_io(&s->iomem_slc, obj, &wifi_slc_ops, s,
                          "esp32s3.wifi.slc", ESP32S3_WIFI_SLC_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem_slc);

    wifi_init_subblock(s, obj, &s->iomem_slchost, &slchost_info,
                       "esp32s3.wifi.slchost", s->slchost_regs,
                       ESP32S3_WIFI_REG_WORDS(ESP32S3_WIFI_SLCHOST_SIZE),
                       ESP32S3_WIFI_SLCHOST_SIZE, 5);

    /* WDEV uses its own ops with MAC address / mode intercepts */
    memory_region_init_io(&s->iomem_wdev, obj, &wifi_wdev_ops, s,
                          "esp32s3.wifi.wdev", ESP32S3_WIFI_WDEV_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem_wdev);

    /* Initialize IRQ outputs */
    qdev_init_gpio_out_named(DEVICE(obj), s->irq, ESP32S3_WIFI_IRQ_NAME,
                             ESP32S3_WIFI_IRQ_COUNT);

    /* Initialize Wi-Fi data-plane backend */
    esp32s3_wifi_backend_init(s, &s->backend);
}

static void esp32s3_wifi_realize(DeviceState *dev, Error **errp)
{
    ESP32S3WifiState *s = ESP32S3_WIFI(dev);

    /* Realize the Wi-Fi backend (creates QEMU NIC) */
    esp32s3_wifi_backend_realize(&s->backend, dev, errp);
}

static void esp32s3_wifi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32s3_wifi_reset_hold;
    dc->realize = esp32s3_wifi_realize;
    device_class_set_props(dc, (Property[]) {
        DEFINE_NIC_PROPERTIES(ESP32S3WifiState, nic_conf),
        DEFINE_PROP_END_OF_LIST(),
    });
}

static const TypeInfo esp32s3_wifi_info = {
    .name = TYPE_ESP32S3_WIFI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3WifiState),
    .instance_init = esp32s3_wifi_init,
    .class_init = esp32s3_wifi_class_init,
    .class_size = sizeof(ESP32S3WifiClass),
};

static void esp32s3_wifi_register_types(void)
{
    type_register_static(&esp32s3_wifi_info);
}

type_init(esp32s3_wifi_register_types);

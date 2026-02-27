/*
 * ESP32-S3 Wi-Fi peripheral skeleton
 *
 * This is the initial skeleton device model for the ESP32-S3 Wi-Fi
 * subsystem. It provides memory-mapped register regions for the
 * major Wi-Fi hardware blocks (Baseband, NRX, RF Front-End, SLC DMA,
 * WDEV). All registers are currently stub implementations that accept
 * reads/writes without triggering real behavior.
 *
 * The device exposes multiple SysBus MMIO regions that the machine
 * file maps to the correct physical addresses.
 *
 * Future work:
 * - Implement init-critical registers for esp_wifi_init() path
 * - Implement SLC DMA ring descriptor handling
 * - Add interrupt generation for MAC/BB events
 * - Add backend for packet exchange with host
 *
 * Copyright (c) 2024 Espressif Systems (Shanghai) Co. Ltd.
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
#include "hw/net/esp32s3_wifi.h"

#define WIFI_DEBUG 0

/* ---------- Generic sub-block register ops ---------- */

/*
 * Generic read/write for Wi-Fi sub-blocks. Each sub-block is modeled
 * as a simple register store where reads return the last written value
 * (or reset default). This allows drivers to probe registers without
 * hitting unimplemented-device warnings.
 */
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

/* Static sub-block descriptors — one per region.
 * These are initialized in esp32s3_wifi_init and persist for the device lifetime. */
static WifiSubBlockInfo bb_info, nrx_info, fe_info, fe2_info;
static WifiSubBlockInfo slc_info, slchost_info, wdev_info;

/* Helper to initialize one sub-block memory region */
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

    /* De-assert all IRQs */
    for (int i = 0; i < ESP32S3_WIFI_IRQ_COUNT; i++) {
        qemu_set_irq(s->irq[i], 0);
    }
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

    wifi_init_subblock(s, obj, &s->iomem_slc, &slc_info,
                       "esp32s3.wifi.slc", s->slc_regs,
                       ESP32S3_WIFI_REG_WORDS(ESP32S3_WIFI_SLC_SIZE),
                       ESP32S3_WIFI_SLC_SIZE, 4);

    wifi_init_subblock(s, obj, &s->iomem_slchost, &slchost_info,
                       "esp32s3.wifi.slchost", s->slchost_regs,
                       ESP32S3_WIFI_REG_WORDS(ESP32S3_WIFI_SLCHOST_SIZE),
                       ESP32S3_WIFI_SLCHOST_SIZE, 5);

    wifi_init_subblock(s, obj, &s->iomem_wdev, &wdev_info,
                       "esp32s3.wifi.wdev", s->wdev_regs,
                       ESP32S3_WIFI_REG_WORDS(ESP32S3_WIFI_WDEV_SIZE),
                       ESP32S3_WIFI_WDEV_SIZE, 6);

    /* Initialize IRQ outputs */
    qdev_init_gpio_out_named(DEVICE(obj), s->irq, ESP32S3_WIFI_IRQ_NAME,
                             ESP32S3_WIFI_IRQ_COUNT);
}

static void esp32s3_wifi_class_init(ObjectClass *klass, void *data)
{
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32s3_wifi_reset_hold;
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

/*
 * ESP32-S3 Pulse Count Controller (PCNT)
 *
 * 4 counter units, each with 2 channels and configurable
 * high/low limit, dual threshold comparators, zero crossing.
 *
 * MVP: Full register read/write, counter reset via CTRL,
 * counter value accessible via U_CNT read, interrupt model.
 * Counter increment not driven by external signals in this
 * virtual model — firmware can test register semantics but
 * actual pulse counting requires GPIO integration.
 *
 * Register layout follows ESP32-S3 TRM v1.2 Chapter 30.
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
#include "hw/irq.h"
#include "hw/misc/esp32s3_pcnt.h"

#define PCNT_DEBUG 0

#if PCNT_DEBUG
#define PCNT_DPRINTF(fmt, ...) \
    qemu_log_mask(LOG_UNIMP, "esp32s3_pcnt: " fmt, ## __VA_ARGS__)
#else
#define PCNT_DPRINTF(fmt, ...) do {} while (0)
#endif

/* ================================================================== */
/*  IRQ helper                                                         */
/* ================================================================== */

static void esp32s3_pcnt_update_irq(ESP32S3PCNTState *s)
{
    int level = !!(s->int_raw & s->int_ena);
    qemu_set_irq(s->irq, level);
}

/* ================================================================== */
/*  MMIO read/write                                                    */
/* ================================================================== */

static uint64_t esp32s3_pcnt_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32S3PCNTState *s = ESP32S3_PCNT(opaque);

    /* Per-unit CONF registers (3 regs × 4 units = 0x00..0x2F) */
    if (addr < 0x30) {
        int unit = addr / 0x0C;
        int off  = addr % 0x0C;
        if (unit < ESP32S3_PCNT_UNIT_COUNT) {
            switch (off) {
            case 0x00: return s->unit_conf0[unit];
            case 0x04: return s->unit_conf1[unit];
            case 0x08: return s->unit_conf2[unit];
            }
        }
        return 0;
    }

    /* Counter value registers (read-only) */
    if (addr >= 0x30 && addr < 0x40) {
        int unit = (addr - 0x30) / 4;
        if (unit < ESP32S3_PCNT_UNIT_COUNT) {
            return (uint16_t)s->unit_cnt[unit];
        }
        return 0;
    }

    /* Status registers */
    if (addr >= 0x50 && addr < 0x60) {
        int unit = (addr - 0x50) / 4;
        if (unit < ESP32S3_PCNT_UNIT_COUNT) {
            return s->unit_status[unit];
        }
        return 0;
    }

    switch (addr) {
    case ESP32S3_PCNT_INT_RAW_OFF:
        return s->int_raw;
    case ESP32S3_PCNT_INT_ST_OFF:
        return s->int_raw & s->int_ena;
    case ESP32S3_PCNT_INT_ENA_OFF:
        return s->int_ena;
    case ESP32S3_PCNT_INT_CLR_OFF:
        return 0;
    case ESP32S3_PCNT_CTRL_OFF:
        return s->ctrl;
    case ESP32S3_PCNT_DATE_OFF:
        return 0x18110800;  /* VERSION */
    default:
        PCNT_DPRINTF("read: unhandled 0x%03" HWADDR_PRIx "\n", addr);
        return 0;
    }
}

static void esp32s3_pcnt_write(void *opaque, hwaddr addr,
                               uint64_t value, unsigned int size)
{
    ESP32S3PCNTState *s = ESP32S3_PCNT(opaque);

    /* Per-unit CONF registers */
    if (addr < 0x30) {
        int unit = addr / 0x0C;
        int off  = addr % 0x0C;
        if (unit < ESP32S3_PCNT_UNIT_COUNT) {
            switch (off) {
            case 0x00: s->unit_conf0[unit] = (uint32_t)value; return;
            case 0x04: s->unit_conf1[unit] = (uint32_t)value; return;
            case 0x08: s->unit_conf2[unit] = (uint32_t)value; return;
            }
        }
        return;
    }

    /* Counter value registers are read-only */
    if (addr >= 0x30 && addr < 0x40) {
        return;
    }

    /* Status registers are read-only */
    if (addr >= 0x50 && addr < 0x60) {
        return;
    }

    switch (addr) {
    case ESP32S3_PCNT_INT_RAW_OFF:
        s->int_raw |= (uint32_t)value;
        esp32s3_pcnt_update_irq(s);
        break;
    case ESP32S3_PCNT_INT_CLR_OFF:
        s->int_raw &= ~(uint32_t)value;
        esp32s3_pcnt_update_irq(s);
        break;
    case ESP32S3_PCNT_INT_ENA_OFF:
        s->int_ena = (uint32_t)value;
        esp32s3_pcnt_update_irq(s);
        break;
    case ESP32S3_PCNT_CTRL_OFF: {
        uint32_t old_ctrl = s->ctrl;
        s->ctrl = (uint32_t)value;
        /* Check for counter reset (rising edge on CNT_RST bit) */
        for (int u = 0; u < ESP32S3_PCNT_UNIT_COUNT; u++) {
            uint32_t rst_bit = PCNT_S3_CTRL_CNT_RST(u);
            if ((value & rst_bit) && !(old_ctrl & rst_bit)) {
                s->unit_cnt[u] = 0;
                s->unit_status[u] = 0;
                PCNT_DPRINTF("unit %d counter reset\n", u);
            }
        }
        break;
    }
    case ESP32S3_PCNT_DATE_OFF:
        break;
    default:
        PCNT_DPRINTF("write: unhandled 0x%03" HWADDR_PRIx " = 0x%08" PRIx64 "\n",
                    addr, value);
        break;
    }
}

/* ================================================================== */
/*  Lifecycle                                                          */
/* ================================================================== */

static const MemoryRegionOps esp32s3_pcnt_ops = {
    .read  = esp32s3_pcnt_read,
    .write = esp32s3_pcnt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32s3_pcnt_reset_hold(Object *obj, ResetType type)
{
    ESP32S3PCNTState *s = ESP32S3_PCNT(obj);

    memset(s->unit_conf0, 0, sizeof(s->unit_conf0));
    memset(s->unit_conf1, 0, sizeof(s->unit_conf1));
    memset(s->unit_conf2, 0, sizeof(s->unit_conf2));
    memset(s->unit_cnt, 0, sizeof(s->unit_cnt));
    memset(s->unit_status, 0, sizeof(s->unit_status));

    s->int_raw = 0;
    s->int_ena = 0;
    s->ctrl = 0;
}

static void esp32s3_pcnt_realize(DeviceState *dev, Error **errp)
{
    esp32s3_pcnt_reset_hold(OBJECT(dev), RESET_TYPE_COLD);
}

static void esp32s3_pcnt_init(Object *obj)
{
    ESP32S3PCNTState *s = ESP32S3_PCNT(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32s3_pcnt_ops, s,
                          TYPE_ESP32S3_PCNT, ESP32S3_PCNT_REG_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static void esp32s3_pcnt_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);
    rc->phases.hold = esp32s3_pcnt_reset_hold;
    dc->realize = esp32s3_pcnt_realize;
}

static const TypeInfo esp32s3_pcnt_info = {
    .name          = TYPE_ESP32S3_PCNT,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32S3PCNTState),
    .instance_init = esp32s3_pcnt_init,
    .class_init    = esp32s3_pcnt_class_init,
    .class_size    = sizeof(ESP32S3PCNTClass),
};

static void esp32s3_pcnt_register_types(void)
{
    type_register_static(&esp32s3_pcnt_info);
}

type_init(esp32s3_pcnt_register_types)
